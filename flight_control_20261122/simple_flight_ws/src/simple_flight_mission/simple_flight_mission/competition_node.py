"""Live camera + two payload doors; preserves the existing 20Hz flight loop."""
import argparse
from collections import deque
from dataclasses import asdict
import json
import os
from pathlib import Path
import signal
import select
import sys
import threading
import time
from urllib.parse import urlparse
from urllib.request import urlopen

from simple_flight_payload.door import build_command, command_request, default_config, validate_output
from .competition_controller import CompetitionController
from .hover_controller import _fresh
from .route_node import RouteNode, ROUTE_PARAMETER_NAMES, ROUTE_TEXT, route_config_path
from .route_config import load_route_config
from .preflight_summary import format_preflight_summary
from .visual_task import RELEASE_THRESHOLD_PX


PAYLOAD_PARAMETERS = tuple(f"PWM_AUX_{kind}{aux}" for aux in (7, 8) for kind in ("FUNC", "MIN", "MAX")) + ("PWM_AUX_REV",)

TASK_TEXT = dict(MARK="稳定悬停并持续展示正确标注", SEARCH="寻找任务目标", SCAN="沿救援通道边移动边寻找人员", PROBE_X="标定X方向", PROBE_Y="标定Y方向",
                 PROBE_RECOVER="人员处于画面边缘，返回标定起点",
                 CALIB_RETURN="标定后回位", ALIGN="视觉对准",
                 OPEN_WAIT="等待开门反馈", DROP_DWELL="投放停留", CLOSE_WAIT="等待关门反馈",
                 RETURN_ANCHOR="返回原航点", DONE="投放完成", FAILED="视觉任务失败")
WAIT_TEXT = dict(camera_alignment_missing="缺少有效地面相机映射，保持位置，不执行探测",
                 alignment_diverged_mapping_disabled="纠偏误差连续增大，停止使用映射并保持，不重新探测",
                 search_target="等待有效目标", command_ramping="微调目标移动中",
                 score_dwell_reset_unstable_or_not_visible="不稳定或缺少新鲜画框，三秒计时重置",
                 score_stable_annotation_dwell="低速悬停且标注可见，持续计时",
                 person_clipped_continue_search="人像贴边裁切，停止旧修正并沿原通道继续寻找完整目标",
                 alignment_diverged_recalibrate="对准误差连续增大，停止旧修正并重新标定一次",
                 alignment_diverged_return_anchor="重新标定后仍未收敛，先回第5点",
                 probe_target_at_edge_return_origin="探测后人员离开视野，先回标定起点",
                 probe_retry_opposite_baseline="重新捕获人员，反向探测一次",
                 searching_rescue_aisle="沿既定通道持续搜索，不在中点停留",
                 waiting_live_search_frame="等待实时画面，暂停继续搜索",
                 person_acquired_stop_scan="已找到人员，停止盲扫并获取对准数据",
                 person_acquired_start_alignment="已找到人员，开始对准",
                 person_far_follow_aisle="已找到人员，先沿通道靠近目标",
                 search_exhausted_return_anchor="到达搜索终点仍未找到人员，先回第5点",
                 return_task_anchor="投放结束，先回原任务航点",
                 person_outside_fine_radius_searching_aisle="人员超出精对准范围，沿既定救援通道继续搜索",
                 vehicle_moving="等待飞机低速稳定", tracking_error_over_10cm="位置跟踪误差超过10厘米",
                 waiting_fresh_alignment_frame="等待新画面继续对准",
                 probe_vehicle_moving="标定等待速度降至0.10m/s内",
                 probe_baseline_under_4cm="标定实际位移不足4厘米",
                 probe_settle_and_sample="标定位移有效，等待稳定后的新画面",
                 probe_resample_invalid="标定样本暂不可用，继续原探测并采新画面",
                 waiting_three_center_samples="等待3帧目标中心估计",
                 holding_estimated_target="保持估计的目标中心",
                 centered_open_immediately="中心确认，立即开门")
VISUAL_ERROR_TEXT = dict(visual_calibration_target_lost="标定时有效目标丢失",
                        visual_alignment_not_converging="人员对准重采标定后仍未收敛",
                        visual_calibration_insufficient_motion="标定实际位移不足4厘米",
                        visual_calibration_not_settled="标定时飞机未稳定",
                        visual_calibration_sample_timeout="标定新画面采样超时",
                        visual_calibration_invalid="标定矩阵无效或实际位移方向不独立",
                        visual_task_timeout="视觉任务超过45秒",
                        correct_target_not_found="未找到对应任务目标",
                        visual_alignment_bound="视觉修正超出35厘米范围")


def task_progress_text(status):
    diagnostic = status.get("task_diagnostics") or {}
    phase = status.get("task_phase")
    parts = ["舱门" + str(status.get("task_door")), TASK_TEXT.get(phase, str(phase))]
    if phase in ("MARK", "SEARCH", "SCAN", "ALIGN", "PROBE_X", "PROBE_Y", "PROBE_RECOVER", "RETURN_ANCHOR"):
        reason = diagnostic.get("wait_reason")
        parts.append(WAIT_TEXT.get(reason, str(reason)))
    elif diagnostic.get("failure"):
        parts.append(str(diagnostic["failure"]))
    for field, label, unit in (("pixel_error", "图像误差", "px"),
                               ("filtered_pixel_error", "补偿滤波误差", "px"),
                               ("position_error_m", "位置误差", "m"),
                               ("probe_displacement_m", "标定实测位移", "m")):
        value = diagnostic.get(field)
        if value is not None:
            parts.append(f"{label}={value:.2f}{unit}")
    if diagnostic.get("release_policy") == "stable_annotated_dwell":
        parts.append(f"稳定标注={diagnostic.get('score_dwell_elapsed_s', 0):.1f}/{diagnostic.get('score_dwell_required_s', 3.2):.1f}s")
    if diagnostic.get("last_calibration_error"):
        parts.append("最近标定校验=" + diagnostic["last_calibration_error"])
    if diagnostic.get("target_rejection"):
        parts.append("目标无效=" + diagnostic["target_rejection"])
    return "；".join(parts)


class VisionFeed:
    def __init__(self, url):
        self.url = url
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.value = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            try:
                with urlopen(self.url, timeout=.4) as response:
                    body = response.read(262145)
                    if len(body) > 262144:
                        raise ValueError("视觉响应过大")
                    value = json.loads(body)
                if not isinstance(value, dict):
                    raise ValueError("视觉响应不是对象")
                value["received_at"] = time.monotonic()
            except Exception as error:
                value = dict(valid=False, error=str(error), received_at=time.monotonic())
            with self.lock:
                self.value = value
            self.stop.wait(.15)

    def snapshot(self):
        with self.lock:
            return self.value

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.)


class CompetitionNode(RouteNode):
    parameter_names = ROUTE_PARAMETER_NAMES + PAYLOAD_PARAMETERS

    def log_state(self, status):
        if self.options.mode != "check":
            super().log_state(status)
            reason = status.get("exit_reason")
            if reason and reason != getattr(self, "exit_reason_reported", None):
                self.exit_reason_reported = reason
                detail = {"swa_low": "SWA(CH7)接管条件触发",
                          "throttle_raise_then_low": "油门先抬高再拉低，接管条件触发",
                          "manual_mode_selected": "飞控已切换为人工模式"}.get(reason, reason)
                self.node.get_logger().warning("退出原因：" + detail)

    def __init__(self, options, route, doors):
        super().__init__(options, route)
        self.phase_text = dict(ROUTE_TEXT)
        self.phase_text["LAND_WAIT"] = "飞控反馈已落地，确认地面静止后正常锁桨，等待实际上锁反馈"
        if options.start_trigger == "notebook":
            self.phase_text["READY"] = ("起飞准备完成，等待Enter确认；SWA保持低档" if getattr(options, "wait_enter", False)
                                        else "笔记本启动授权已就绪，将自动请求OFFBOARD；SWA保持低档")
        self.controller = CompetitionController(options.mode, route, doors, options.start_trigger,
                                                require_enter=getattr(options, "wait_enter", False),
                                                camera_alignment=getattr(options, "camera_alignment_data", None))
        if getattr(options, "camera_alignment_data", None) is None:
            self.node.get_logger().warning("未配置有效地面相机映射：阈值外保持位置，不执行空中探测或猜方向纠偏")
        if route.point_names:
            self.node.get_logger().info("航线坐标来源="+route.coordinate_source+"；"+
                " → ".join(f"{name}({point.forward_m:.3f},{point.left_m:.3f})" for name,point in zip(route.point_names,route.points))+
                f"；人员搜索终点={route.search_point_name}{route.search_xy}")
        self.enter_prompted = False
        self.startup_deadline = self.started + 60.
        self.feed = VisionFeed(options.vision_url)
        self.payload_pending = None
        self.disarm_pending = None
        self.disarm_submitted = False
        self.payload_queue = deque([(1, "close"), (2, "close")] if options.mode == "execute" else [])
        self.closed_initially = set()
        self.last_payload = None
        self.status_written = None
        self.exit_started = None
        self.task_report_key = self.task_report_at = None
        self.visual_error_reported = None
        self.CommandLong = None
        self.payload_client = None
        if options.mode == "execute":
            from mavros_msgs.srv import CommandLong
            self.CommandLong = CommandLong
            self.payload_client = self.node.create_client(CommandLong, "/mavros/cmd/command")

    def status_extra(self, now):
        task = self.controller.task
        return dict(competition=True, start_trigger=self.options.start_trigger,
                    point_name=(self.controller.route_config.point_names[self.controller.point_index]
                                if self.controller.point_index is not None and self.controller.route_config.point_names else None),
                    coordinate_source=self.controller.route_config.coordinate_source,
                    coordinate_sha256=self.controller.route_config.coordinate_sha256,
                    search_point_name=self.controller.route_config.search_point_name,
                    waiting_for_enter=self.controller.require_enter and not self.controller.enter_confirmed
                    and self.controller.phase == "READY",
                    landed_disarm_requested=self.controller.disarm_requested,
                    landed_disarm_ack_accepted=self.controller.disarm_ack_accepted,
                    task_phase=task.phase if task else "IDLE", task_door=task.door if task else None,
                    completed_task_count=len(self.controller.completed_tasks),
                    payload_initialized=self.controller.payload_initialized,
                    last_payload=self.last_payload, visual_error=self.controller.visual_error,
                    camera_valid=bool((self.controller.vision_data or {}).get("valid")),
                    measured_image_jacobian=self.controller.jacobian,
                    task_diagnostics=task.diagnostic(now) if task else None,
                    return_reserve_s=self.controller.return_reserve_s)

    def _report_task(self, now):
        status = self.last_status or {}
        if self.options.mode != "execute" or status.get("task_phase") in (None, "IDLE"):
            return
        error = status.get("visual_error")
        if error and error != self.visual_error_reported:
            self.visual_error_reported = error
            self.node.get_logger().warning("取消投放并返航：" + VISUAL_ERROR_TEXT.get(error, error))
        key = (status.get("phase"), status.get("task_door"), status.get("task_phase"))
        active = status.get("phase") == "POINT_HOLD"
        if key != self.task_report_key or active and (self.task_report_at is None or now - self.task_report_at >= 1.):
            self.task_report_key, self.task_report_at = key, now
            self._event("visual_task", status)
            if active:
                self.node.get_logger().info(task_progress_text(status))

    def _payload_update(self, now, send=False):
        if self.payload_pending is not None:
            future, door, action, sent, initial = self.payload_pending
            if future.done() or now - sent > 2.:
                accepted = False
                try:
                    if future.done():
                        response = future.result()
                        accepted = bool(response.success and response.result == 0)
                    else:
                        future.cancel()
                except Exception:
                    pass
                self.payload_pending = None
                self.last_payload = dict(door=door, action=action, ack_accepted=accepted,
                                         physical_position_confirmed=False, monotonic=now)
                self._event("payload_ack", self.last_payload)
                if initial:
                    if accepted:
                        self.closed_initially.add(door)
                        self.controller.payload_initialized = self.closed_initially == {1, 2}
                    else:
                        self.controller.request_exit("initial_door_close_failed_or_unknown")
                        self.payload_queue.clear()
                else:
                    self.controller.payload_ack = (door, action, accepted)
        if not send or self.payload_pending is not None or not self.payload_queue or self.payload_client is None:
            return
        vehicle = self.snapshot()
        if not _fresh(vehicle.state, now, 1.5) or not (vehicle.state.value or {}).get("connected"):
            return
        if not _fresh(vehicle.parameters, now, 10.) or not self.payload_client.service_is_ready():
            return
        door, action = self.payload_queue[0]
        initial = not self.controller.payload_initialized
        if initial and (vehicle.state.value or {}).get("armed"):
            self.controller.request_exit("initial_door_close_requires_disarmed")
            self.payload_queue.clear()
            return
        if action == "open":
            task = self.controller.task
            if (self.controller.phase != "POINT_HOLD" or self.controller.returning or task is None
                    or task.door != door or task.phase != "OPEN_WAIT"
                    or (vehicle.state.value or {}).get("mode") != "OFFBOARD"
                    or not (vehicle.state.value or {}).get("armed")):
                self.payload_queue.popleft()
                return
        try:
            command = build_command(self.controller.doors, door, action)
            validate_output(command, vehicle.parameters.value or {})
        except ValueError as error:
            self.controller.request_exit("payload_mapping_invalid")
            self.node.get_logger().error(str(error))
            self.payload_queue.clear()
            return
        self.payload_queue.popleft()
        self._event("payload_send", command.description())
        future = self.payload_client.call_async(command_request(command, self.CommandLong.Request))
        self.payload_pending = (future, door, action, now, initial)

    def _disarm_update(self, now):
        if self.disarm_pending is not None:
            future, sent = self.disarm_pending
            if future.done() or now-sent > 2.:
                accepted = False
                try:
                    accepted = bool(future.done() and future.result().success)
                except Exception:
                    pass
                self.disarm_pending = None
                self.controller.disarm_ack_accepted = accepted
                self._event("landed_disarm_ack", dict(accepted=accepted, actual_state_authoritative=True))
                if not accepted:
                    self.node.get_logger().warning("正常锁桨请求未确认接受；继续等待真实armed反馈，不强制、不重复发送。")
        if self.controller.take_landed_disarm_request() is None:
            return
        if (self.disarm_submitted or not self.controller.landed_disarm_evidence(self.snapshot(), now)
                or "arm" in self.pending or self.arm_client is None or not self.arm_client.service_is_ready()):
            self.controller.disarm_ack_accepted = False
            self.node.get_logger().warning("落地锁桨条件已变化或服务不可用，未发送锁桨请求。")
            return
        request = self.CommandBool.Request()
        request.value = False  # Normal MAVROS disarm; no force parameter.
        self.disarm_submitted = True
        try:
            self.disarm_pending = (self.arm_client.call_async(request), now)
        except Exception:
            self.controller.disarm_ack_accepted = False
            self.node.get_logger().warning("正常锁桨请求提交失败，不重发；等待真实armed反馈。")
            return
        event = dict(command="disarm", monotonic=now, force=False)
        self.command_events.append(event)
        self._event("command_request", event)
        self.node.get_logger().info("请求：落地后正常锁桨一次（非强制），等待实际armed=false反馈。")

    def _poll_enter(self, now):
        if (not self.controller.require_enter or self.controller.enter_confirmed
                or self.controller.phase != "READY"):
            return
        # Keep the ROS loop and ground setpoint stream alive while waiting.
        self.startup_deadline = now + 60.
        import termios
        try:
            fd = sys.stdin.fileno()
            if not self.enter_prompted:
                termios.tcflush(fd, termios.TCIFLUSH)
                self.enter_prompted = True
                self.node.get_logger().info("准备完成：两门已关闭，静止检查和目标预发送完成。按 Enter 请求OFFBOARD并自动起飞；Ctrl+C取消。")
                return
            if not select.select([fd], [], [], 0)[0]:
                return
            line = os.read(fd, 4096)
            if not line:
                self.request_stop()
                return
            if line not in (b"\n", b"\r\n"):
                self.node.get_logger().info("请只按 Enter 确认起飞，或按Ctrl+C取消。")
                return
            if self.controller.confirm_start(self.snapshot(), now):
                self.node.get_logger().info("已收到Enter确认，将请求OFFBOARD并自动起飞；SWA保持低档。")
            else:
                self.node.get_logger().warning("当前起飞条件变化，未接受确认；恢复条件后重新按Enter。")
        except (OSError, ValueError, termios.error):
            self.request_stop()

    def _tick(self):
        now = time.monotonic()
        self.controller.vision_data = self.feed.snapshot()
        self._payload_update(now)
        super()._tick()
        self._disarm_update(time.monotonic())
        self._poll_enter(time.monotonic())
        self._report_task(now)
        action = self.controller.take_payload_action()
        if action:
            self.payload_queue.append(action)
        self._payload_update(time.monotonic(), send=True)
        if self.last_status is not None and (self.status_written is None or now - self.status_written >= .5):
            self.recorder.snapshot(self.options.status_file, self.last_status)
            self.status_written = now
        if self.options.mode == "execute" and now > self.startup_deadline and self.controller.mission_started_at is None:
            self.controller.request_exit("startup_not_ready_within_60s")
            self.request_stop()
        if self.controller.phase == "EXITED":
            self.exit_started = now if self.exit_started is None else self.exit_started
            if ((self.controller.handover_confirmed and self.payload_pending is None and self.disarm_pending is None)
                    or now - self.exit_started > 2.2):
                self.finished = True

    def close(self):
        self.feed.close()
        super().close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="比赛任务；默认只读。execute 才会关闭舱门并启动飞行。")
    parser.add_argument("--route", type=int, choices=(1, 2, 3, 4), required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--execute", action="store_true")
    modes.add_argument("--check", action="store_true")
    parser.add_argument("--describe", action="store_true")
    parser.add_argument("--start-trigger", choices=("notebook", "rc"), default="notebook")
    parser.add_argument("--wait-enter", action="store_true", help="完整准备后等待终端Enter，才允许请求OFFBOARD")
    parser.add_argument("--door-config", type=Path)
    parser.add_argument("--camera-alignment", type=Path, help="地面标定相机映射；缺失时不进行空中探测")
    parser.add_argument("--vision-url", default="http://127.0.0.1:8088/api/status")
    parser.add_argument("--report-dir", type=Path, default=Path.home() / "simple_flight_ws/log/competition")
    parser.add_argument("--status-file", type=Path, default=Path.home() / "simple_flight_ws/log/competition_status.json")
    parser.add_argument("--duration", type=float, default=8.)
    options = parser.parse_args(argv)
    options.mode = "execute" if options.execute else "check"
    if options.wait_enter and (not options.execute or options.start_trigger != "notebook"):
        parser.error("--wait-enter仅用于--execute和notebook启动")
    if not 0 < options.duration <= 120:
        parser.error("只读时长必须在0..120秒内")
    parsed = urlparse(options.vision_url)
    if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost"):
        parser.error("视觉接口必须为 Nano 本机 HTTP 地址")
    try:
        from .camera_alignment import load_alignment
        options.camera_alignment_data = load_alignment(options.camera_alignment)
        route = load_route_config(route_config_path(options.route))
        doors = json.loads((options.door_config or default_config()).read_text(encoding="utf-8"))
        commands = [build_command(doors, door, action).description() for door in (1, 2) for action in ("open", "close")]
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    if options.describe:
        print(json.dumps(dict(mode=options.mode, route=asdict(route), doors=commands,
                              budget_s=180, image_axis_calibration="ground_precalibrated_no_airborne_probes",
                              camera_alignment_configured=options.camera_alignment_data is not None, release_threshold_px=RELEASE_THRESHOLD_PX), ensure_ascii=False))
        return 0
    if options.wait_enter and not sys.stdin.isatty():
        parser.error("Enter确认需要交互终端，不能通过管道或后台启动")
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    runner, previous, stop = None, {}, [False]
    try:
        runner = CompetitionNode(options, route, doors)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, lambda *_: stop.__setitem__(0, True))
        while rclpy.ok() and not runner.finished:
            if stop[0]:
                runner.request_stop()
            rclpy.spin_once(runner.node, timeout_sec=.02)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if runner is not None:
            runner.close()
        if rclpy.ok():
            rclpy.shutdown()
    if runner is None or runner.last_output is None:
        return 2
    if options.mode == "check":
        print(format_preflight_summary(runner.last_status), flush=True)
        return 2 if runner.last_output.reasons else 0
    return 0 if runner.controller.mission_completed and len(runner.controller.completed_tasks) == 2 else 2


if __name__ == "__main__":
    raise SystemExit(main())
