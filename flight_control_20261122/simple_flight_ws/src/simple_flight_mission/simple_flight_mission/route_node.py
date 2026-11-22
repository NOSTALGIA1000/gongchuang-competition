"""Independent configurable route entry. No flight on boot or by default."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import sys
import time
import uuid

from .hover_controller import Observation, PARAMETER_NAMES
from .hover_node import PHASE_TEXT, HoverNode, _pose_values
from .route_config import load_route_config
from .route_controller import LANDING_PARAMETER_NAMES, RouteController


ROUTE_PARAMETER_NAMES = PARAMETER_NAMES + LANDING_PARAMETER_NAMES + ("EKF2_GPS_CTRL",)
ROUTE_TEXT = dict(PHASE_TEXT, ORIGIN_HOLD="起飞点上方稳定悬停",
                  TRANSIT="正在按配置飞向航点", POINT_HOLD="实际到点，正在计时悬停",
                  RETURN="保持初始朝向，沿返航航线连续通过", RETURN_HOLD="已到起飞点上方，稳定悬停",
                  DESCEND="保持起飞点水平位置，低速下降；两种人工接管动作均有效",
                  LAND_WAIT="飞控反馈已落地，等待飞控自动上锁；程序不发送上锁命令")


def build_parser():
    p = argparse.ArgumentParser(description="可配置相对航点往返；默认只读，不运行旧跑点入口")
    modes = p.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true", help="只读检查（默认）")
    modes.add_argument("--mode-test", action="store_true", help="拆桨未解锁模式测试，只保持地面目标")
    modes.add_argument("--execute", action="store_true", help="SWA 触发自动起飞、航点、返回和降落")
    selection = p.add_mutually_exclusive_group()
    selection.add_argument("--config", type=Path, help="任务 YAML；更改此文件无需修改 Python")
    selection.add_argument("--route", type=int, choices=(1, 2, 3, 4), help="预设路线编号；省略时使用兼容的单点配置")
    p.add_argument("--duration", type=float, help="只读检查秒数")
    p.add_argument("--report-dir", type=Path)
    p.add_argument("--describe", action="store_true", help="仅打印配置，不加载 ROS 或连接飞机")
    return p


def parse_options(argv=None):
    p = build_parser()
    options = p.parse_args(argv)
    options.mode = "execute" if options.execute else "mode-test" if options.mode_test else "check"
    if options.duration is not None and (options.mode != "check" or not math.isfinite(options.duration) or options.duration <= 0):
        p.error("--duration 仅适用于只读检查，且必须为有限正数")
    return options


def route_config_path(route):
    if route is None:
        return None
    share = None
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        pass
    else:
        try:
            share = Path(get_package_share_directory("simple_flight_mission"))
        except PackageNotFoundError:
            pass
    # Source checkout preview works without ROS. Once the package is installed,
    # require its own config rather than silently falling back to another copy.
    base = share if share is not None else Path(__file__).resolve().parents[1]
    path = base / "config" / ("route_%d.yaml" % route)
    if not path.is_file():
        raise ValueError("路径 %d 尚未配置；缺少配置文件：%s" % (route, path))
    return path


def selected_route_config(options):
    return load_route_config(options.config or route_config_path(getattr(options, "route", None)))


def raw_target_fields(output):
    x, y, z, yaw = output.target
    vz = getattr(output, "velocity_z", None)
    # FRAME_LOCAL_NED=1 is the MAVLink frame identifier. MAVROS receives
    # these numerical coordinates/velocities in ROS ENU and transforms to NED.
    # Ignore AX/AY/AZ and yaw rate. Normal: ignore VX/VY/VZ.
    # Descent: ignore PZ/VX/VY, retain finite VZ for PX4 landing detection.
    mask = 64 | 128 | 256 | 2048 | 8 | 16 | (32 if vz is None else 4)
    return dict(coordinate_frame=1, type_mask=mask,
                position=(float(x), float(y), float(z)),
                velocity=(0., 0., 0. if vz is None else float(vz)), yaw=float(yaw))


def raw_receiver_ready(infos):
    receivers = [info for info in infos
                 if info.node_namespace.rstrip("/")+"/"+info.node_name == "/mavros/setpoint_raw"
                 and info.topic_type == "mavros_msgs/msg/PositionTarget"]
    return len(receivers) == 1


class RouteNode(HoverNode):
    """Reuse qualified input freshness, service requests and graph exclusion.

Initialization creates no control messages. Replace the base publisher before
the executor spins; only /setpoint_raw/local is used by this entry.
"""
    parameter_names = ROUTE_PARAMETER_NAMES
    phase_text = ROUTE_TEXT

    def status_extra(self, now):
        return {}

    def log_state(self, status):
        suffix = "；原因=" + "；".join(status["reasons"]) if status["reasons"] else ""
        point = "；航点=" + str(status["point_number"]) if status["point_number"] is not None else ""
        self.node.get_logger().info(self.phase_text[status["phase"]] + point + suffix)

    def __init__(self, options, config=None, node=None):
        if config is None:
            config = selected_route_config(options)
        import rclpy
        from mavros_msgs.msg import PositionTarget
        node = node or rclpy.create_node("simple_route_"+uuid.uuid4().hex[:8], use_global_arguments=False)
        super().__init__(options, config.core_config(), node)
        self.controller = RouteController(options.mode, config)
        self.PositionTarget = PositionTarget
        if self.target_pub is not None:
            self.node.destroy_publisher(self.target_pub)
            self.target_pub = self.node.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 1)
        self.node.destroy_publisher(self.status_pub)
        self.status_pub = self.node.create_publisher(self.String, "/simple_flight/route/status", 10)
        self._event("route_configuration", asdict(config))

    def _graph(self):
        super()._graph()
        try:
            infos = self.node.get_subscriptions_info_by_topic("/mavros/setpoint_raw/local")
            ready = raw_receiver_ready(infos)
        except Exception:
            ready = False
        self.controller.note_target_transport(time.monotonic(), ready)

    def _vision(self, message):
        super()._vision(message)
        pose = _pose_values(message.pose.pose)
        self._store("external_heading", None if pose is None else pose[3], message)
        self.controller.external_heading = self.values["external_heading"]

    def _parameters(self, now):
        if self.param_future is not None:
            if self.param_future.done():
                try:
                    response = self.param_future.result()
                    values = {}
                    for name, value in zip(self.parameter_names, response.values):
                        if value.type == 2:
                            values[name] = value.integer_value
                        elif value.type == 3:
                            values[name] = value.double_value
                    self.values["parameters"] = Observation(values, now)
                except Exception as error:
                    self.node.get_logger().warning("读取飞控参数失败："+str(error))
                self.param_future = None
            elif now-self.param_requested_at > 2.0:
                self.param_future.cancel()
                self.param_future = None
        if (self.param_future is None and (self.param_last_attempt is None or now-self.param_last_attempt >= 5)
                and self.param_client.service_is_ready()):
            req = self.GetParameters.Request()
            req.names = list(self.parameter_names)
            self.param_future = self.param_client.call_async(req)
            self.param_requested_at = self.param_last_attempt = now

    def _tick(self):
        started = time.monotonic()
        interval = None if self._last_tick_started is None else started-self._last_tick_started
        self._last_tick_started = started
        if interval is not None:
            self._max_tick_interval_s = max(self._max_tick_interval_s, interval)
        self._tick_interval_s = interval
        try:
            self._tick_impl()
        finally:
            self._last_tick_work_s = time.monotonic()-started

    def _tick_impl(self):
        now = time.monotonic()
        try:
            self._parameters(now)
            self._command_results(now)
            vehicle = self.snapshot()
            out = self.controller.step(vehicle, now)
            self.last_output = out
            if out.request_mode is not None:
                self._request(out.request_mode, now)
            if out.arm:
                self._request("arm", now)
            if out.target is not None:
                if self.target_pub is None:
                    raise RuntimeError("只读运行不得发布目标")
                values = raw_target_fields(out)
                msg = self.PositionTarget()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.header.frame_id = self.pose_frame
                msg.coordinate_frame, msg.type_mask = values["coordinate_frame"], values["type_mask"]
                msg.position.x, msg.position.y, msg.position.z = values["position"]
                msg.velocity.x, msg.velocity.y, msg.velocity.z = values["velocity"]
                msg.yaw = values["yaw"]
                self.target_pub.publish(msg)
                self.controller.note_target_sent(time.monotonic())
            state, rc = vehicle.state.value or {}, vehicle.rc.value or []
            index = self.controller.point_index
            heading_errors = self.controller.heading_errors(vehicle)
            stage = getattr(self.controller, "stage", None)
            if stage is None:
                stage = (getattr(self.controller.route_config.points[index], "stage", "waypoint")
                         if index is not None else out.phase.lower())
            status = dict(run_id=self.run_id, operating_mode=self.options.mode, phase=out.phase,
                          stage=stage,
                          mission_elapsed_s=getattr(self.controller, "mission_elapsed", lambda _: None)(now),
                          return_reason=getattr(self.controller, "return_reason", None),
                          actual_mode=state.get("mode"), armed=state.get("armed"),
                          reasons=list(out.reasons), target=out.target,
                          velocity_z=getattr(out, "velocity_z", None), exit_reason=out.exit_reason,
                          ready_for_switch=out.ready_for_switch, handover_confirmed=out.handover_confirmed,
                          mission_completed=self.controller.mission_completed,
                          point_number=None if index is None else index+1,
                          point_count=len(self.controller.route_config.points),
                          locked_origin=self.controller.origin, final_leg_goal=self.controller.goal,
                          locked_landing_parameters=self.controller.landing_parameters,
                          actual_pose=vehicle.pose.value, actual_velocity=vehicle.velocity.value,
                          external_heading_rad=self.controller.external_heading.value,
                          heading_reference_error_deg=None if heading_errors is None else math.degrees(heading_errors[0]),
                          heading_target_error_deg=None if heading_errors is None or heading_errors[1] is None else math.degrees(heading_errors[1]),
                          landed_state=vehicle.landed.value,
                          swa_pwm=rc[6] if len(rc)>6 else None, throttle_pwm=rc[2] if len(rc)>2 else None,
                          main_switch_pwm=rc[4] if len(rc)>4 else None,
                          other_setpoint_publishers=self.other_publishers, updated_monotonic=now,
                          boot_id=self.boot_id)
            status.update(self.status_extra(now))
            status.update(route_tick_interval_s=self._tick_interval_s,
                          previous_route_tick_work_s=self._last_tick_work_s,
                          max_route_tick_interval_s=self._max_tick_interval_s,
                          recorder=self.recorder.stats())
            self.last_status = status
            self._record_telemetry(status, now)
            key = (out.phase, index, out.exit_reason, out.reasons)
            changed = key != self._last_phase_key
            if changed or self.last_report is None or now-self.last_report >= 1.:
                msg = self.String()
                msg.data = json.dumps(status, ensure_ascii=False, allow_nan=False)
                self.status_pub.publish(msg)
                self.last_report = now
                if changed:
                    self._last_phase_key = key
                    self.log_state(status)
                    self._event("state", status)
            if self.options.mode == "check" and self.options.duration is not None and now-self.started >= self.options.duration:
                self.finished = True
            if self.stop_at is not None:
                if ((out.phase == "EXITED" and out.handover_confirmed and now-self.stop_at >= .5)
                        or now-self.stop_at >= self.controller.config.handover_timeout_s+1.2
                        or self.options.mode == "check"):
                    self.finished = True
        except Exception as error:
            self.controller.request_exit("adapter_exception")
            self.node.get_logger().error("往返任务异常，停止自动目标："+str(error))
            if self.options.mode != "check":
                try:
                    self._request("POSCTL", now)
                except Exception:
                    pass
            if self.stop_at is None:
                self.stop_at = now
            elif now-self.stop_at >= self.controller.config.handover_timeout_s+1.2:
                self.finished = True


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in arguments:
        from rclpy.utilities import remove_ros_args
        arguments = remove_ros_args(args=arguments)
    options = parse_options(arguments)
    try:
        config = selected_route_config(options)
    except (OSError, ValueError, TypeError) as error:
        build_parser().error(str(error))
    if options.describe:
        print(json.dumps(dict(mode=options.mode, config=asdict(config),
                              coordinate_rule="absolute_offsets_from_takeoff_in_initial_heading",
                              return_and_land=True), ensure_ascii=False, allow_nan=False))
        return 0
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    runner, previous, stop = None, {}, [False]
    try:
        runner = RouteNode(options, config)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, lambda *_: stop.__setitem__(0, True))
        while rclpy.ok() and not runner.finished:
            if stop[0]:
                runner.request_stop()
            rclpy.spin_once(runner.node, timeout_sec=.02)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if runner is not None:
            runner.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 2 if options.mode == "check" and runner is not None and runner.last_output is not None and runner.last_output.reasons else 0


if __name__ == "__main__":
    raise SystemExit(main())
