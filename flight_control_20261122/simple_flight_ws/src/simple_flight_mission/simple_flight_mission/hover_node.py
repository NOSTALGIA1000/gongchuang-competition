"""Independent ROS hover entry. Default operation is read-only."""
import argparse
from dataclasses import asdict, fields
import json
import math
from pathlib import Path
import signal
import sys
import time
import uuid

from .hover_controller import (
    HoverConfig, HoverController, Observation, PARAMETER_NAMES, Vehicle,
)
from .flight_recorder import FlightRecorder


CONTROL_TOPICS = (
    "/mavros/setpoint_position/local", "/mavros/setpoint_position/global",
    "/mavros/setpoint_raw/local", "/mavros/setpoint_raw/global",
    "/mavros/setpoint_raw/attitude", "/mavros/setpoint_velocity/cmd_vel",
    "/mavros/setpoint_attitude/attitude", "/mavros/setpoint_trajectory/local",
)
PHASE_TEXT = {
    "CHECK": "只读检查：不会发布目标、切模式或解锁",
    "WAITING": "等待飞机静止、SWA低档及地面检查通过",
    "PRESTREAM": "正在预发送地面目标，请保持SWA低档",
    "READY": "准备完成，请将SWA由低档拨到高档",
    "SWITCHING": "等待飞控实际进入OFFBOARD",
    "ARMING": "已进入OFFBOARD，等待飞控实际解锁反馈",
    "TAKEOFF": "正在上升至起点上方目标高度",
    "HOVER": "正在悬停，可用SWA回低或油门抬高后拉低交还人工",
    "MODE_TEST": "拆桨模式测试：保持未解锁，SWA回低可退出",
    "HANDOVER": "已冻结爬升，正在交还人工，无需人工确认",
    "EXITED": "本次任务已退出，不会自动重新接管",
}


def build_parser():
    p = argparse.ArgumentParser(description="独立程序悬停；默认只读，跑点程序不参与")
    modes = p.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true", help="只读检查（默认）")
    modes.add_argument("--mode-test", action="store_true", help="拆桨模式测试：有目标和模式请求，无解锁接口")
    modes.add_argument("--execute", action="store_true", help="SWA触发自动解锁、起飞及悬停，须先完成台架验收")
    p.add_argument("--duration", type=float, help="只读检查持续秒数")
    p.add_argument("--config", type=Path)
    p.add_argument("--report-dir", type=Path, help="保存本次状态与请求事件的独立目录")
    p.add_argument("--describe", action="store_true", help="仅说明配置，不加载ROS或连接飞机")
    return p


def parse_options(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    args.mode = "execute" if args.execute else "mode-test" if args.mode_test else "check"
    if args.duration is not None:
        if args.mode != "check" or not math.isfinite(args.duration) or args.duration <= 0:
            parser.error("--duration只适用于只读检查，且必须为有限正数")
    return args


def load_hover_config(path=None):
    if path is None:
        return HoverConfig()
    import yaml
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    allowed = {f.name for f in fields(HoverConfig)}
    if not isinstance(value, dict) or not set(value) <= allowed:
        raise ValueError("配置必须是HoverConfig支持的字段；不接受航点或未知字段")
    return HoverConfig(**value)


def _pose_values(pose):
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if not all(map(math.isfinite, values)) or abs(norm-1.) > .02:
        return None
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    tilt = math.acos(max(-1., min(1., 1-2*(q.x*q.x+q.y*q.y))))
    return (p.x, p.y, p.z, yaw, tilt)


def estimator_valid(message):
    # PX4 v1.16 Ekf::get_ekf_soln_status sets CONST_POS also when
    # vehicle_at_rest. Instead require horizontal aiding plus valid estimates.
    required = ("attitude_status_flag", "velocity_horiz_status_flag",
                "velocity_vert_status_flag", "pos_horiz_rel_status_flag",
                "pos_vert_abs_status_flag", "pred_pos_horiz_rel_status_flag")
    return (all(getattr(message, name, False) for name in required)
            and not getattr(message, "accel_error_status_flag", True))


class HoverNode:
    """Thin ROS adapter; no parameter writer or disarm service path."""
    def __init__(self, options, config=None, node=None):
        import rclpy
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
        from mavros_msgs.msg import State, RCIn, ExtendedState, EstimatorStatus, SysStatus
        from mavros_msgs.srv import SetMode, CommandBool
        from nav_msgs.msg import Odometry
        from rcl_interfaces.srv import GetParameters
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.options = options
        self.run_id = uuid.uuid4().hex
        self.node = node or rclpy.create_node("simple_hover_" + self.run_id[:8], use_global_arguments=False)
        self.controller = HoverController(options.mode, config)
        self.PoseStamped, self.String = PoseStamped, String
        self.SetMode, self.CommandBool, self.GetParameters = SetMode, CommandBool, GetParameters
        self.started = time.monotonic()
        self.stop_at = None
        self.finished = False
        self.values = {}
        self.stamps = {}
        self.pose_frame = "map"
        self.other_publishers = 0
        self.command_events = []
        self.pending = {}
        self.param_future = None
        self.param_requested_at = None
        self.param_last_attempt = None
        self.last_output = None
        self.last_status = None
        self.last_report = None
        self._last_phase_key = None
        self.last_telemetry = None
        self._last_tick_started = None
        self._last_tick_work_s = None
        self._max_tick_interval_s = 0.
        event_path = None
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        if options.report_dir is not None:
            # Each run has its own child folder, never overwrite an older run.
            report = options.report_dir / self.run_id
            report.mkdir(parents=True, exist_ok=False)
            event_path = report / "events.jsonl"
        self.recorder = FlightRecorder(path=event_path)
        self._event("start", {"mode": options.mode, "config": asdict(self.controller.config), "boot_id": self.boot_id})
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions = []
        definitions = (
            (State, "/mavros/state", self._state),
            (RCIn, "/mavros/rc/in", self._rc),
            (ExtendedState, "/mavros/extended_state", self._landed),
            (PoseStamped, "/mavros/local_position/pose", self._pose),
            (TwistStamped, "/mavros/local_position/velocity_local", self._velocity),
            (Odometry, "/Odometry", self._lio),
            (PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov", self._vision),
            (EstimatorStatus, "/mavros/estimator_status", self._estimator),
            (SysStatus, "/mavros/sys_status", self._health),
            (String, "/simple_flight/boot/status", self._boot),
        )
        for cls, topic, callback in definitions:
            self.subscriptions.append(self.node.create_subscription(cls, topic, callback, qos))
        self.status_pub = self.node.create_publisher(String, "/simple_flight/hover/status", 10)
        self.param_client = self.node.create_client(GetParameters, "/mavros/param/get_parameters")
        self.target_pub = self.mode_client = self.arm_client = None
        if options.mode != "check":
            self.target_pub = self.node.create_publisher(PoseStamped, CONTROL_TOPICS[0], 1)
            self.mode_client = self.node.create_client(SetMode, "/mavros/set_mode")
        if options.mode == "execute":
            self.arm_client = self.node.create_client(CommandBool, "/mavros/cmd/arming")
        self.graph_timer = self.node.create_timer(.5, self._graph)
        self.timer = self.node.create_timer(1/self.controller.config.setpoint_hz, self._tick)
        self.node.get_logger().info(PHASE_TEXT[self.controller.phase] + "；运行方式=" + options.mode)

    def _event(self, kind, data):
        self.recorder.emit(kind, data)

    def _record_telemetry(self, status, now):
        deadline = getattr(self, '_next_telemetry_at', None)
        if deadline is None:
            deadline = now
        if now + 1e-9 >= deadline:
            self.recorder.emit("telemetry", status, monotonic=now, critical=False)
            self.last_telemetry = now
            # Follow fixed deadlines rather than accumulating callback jitter.
            # Skip missed slots instead of emitting a catch-up burst.
            slots = max(1, math.floor((now-deadline+1e-9)/.1)+1)
            self._next_telemetry_at = deadline+slots*.1

    def _store(self, key, value, msg=None):
        age = None
        if msg is not None:
            stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
            if stamp <= 0:
                value, age = None, float("inf")
            else:
                if stamp <= self.stamps.get(key, -1):
                    return
                self.stamps[key] = stamp
                age = (self.node.get_clock().now().nanoseconds-stamp)/1e9
        self.values[key] = Observation(value, time.monotonic(), age)

    def _state(self, m):
        self._store("state", dict(mode=m.mode, armed=m.armed, connected=m.connected, manual_input=m.manual_input))

    def _rc(self, m):
        self._store("rc", list(m.channels), m)

    def _landed(self, m):
        self._store("landed", m.landed_state)

    def _pose(self, m):
        self.pose_frame = m.header.frame_id or "map"
        self._store("pose", _pose_values(m.pose), m)

    def _velocity(self, m):
        v = m.twist.linear
        self._store("velocity", (v.x, v.y, v.z), m)

    def _lio(self, m):
        self._store("lio", _pose_values(m.pose.pose) is not None, m)

    def _vision(self, m):
        self._store("vision", _pose_values(m.pose.pose) is not None, m)

    def _estimator(self, m):
        self._store("estimator", bool(estimator_valid(m)))

    def _health(self, m):
        common = m.sensors_present & m.sensors_enabled & m.sensors_health
        self._store("health", dict(rc_ready=bool(common & (1 << 16)),
                                   prearm_ready=bool(m.sensors_health & (1 << 28))))

    def _boot(self, m):
        try:
            value = json.loads(m.data)
            if not isinstance(value, dict):
                value = None
        except (ValueError, TypeError):
            value = None
        self._store("boot", value)

    def _graph(self):
        try:
            self.other_publishers = sum(
                1 for topic in CONTROL_TOPICS for info in self.node.get_publishers_info_by_topic(topic)
                if (info.node_name, info.node_namespace) != (self.node.get_name(), self.node.get_namespace()))
        except Exception:
            # A graph query failure is not evidence of exclusivity.
            self.other_publishers = 1

    def _parameters(self, now):
        if self.param_future is not None:
            if self.param_future.done():
                try:
                    response = self.param_future.result()
                    values = {}
                    for name, value in zip(PARAMETER_NAMES, response.values):
                        if value.type == 2:
                            values[name] = value.integer_value
                        elif value.type == 3:
                            values[name] = value.double_value
                    self.values["parameters"] = Observation(values, now)
                except Exception as error:
                    self.node.get_logger().warning("读取飞控参数失败：" + str(error))
                self.param_future = None
            elif now-self.param_requested_at > 2.0:
                self.param_future.cancel()
                self.param_future = None
        if (self.param_future is None and (self.param_last_attempt is None or now-self.param_last_attempt >= 5)
                and self.param_client.service_is_ready()):
            req = self.GetParameters.Request()
            req.names = list(PARAMETER_NAMES)
            self.param_future = self.param_client.call_async(req)
            self.param_requested_at = self.param_last_attempt = now

    def _command_results(self, now):
        for kind, (future, sent) in list(self.pending.items()):
            if not future.done() and now-sent <= 1.0:
                continue
            accepted = False
            try:
                if future.done():
                    result = future.result()
                    accepted = bool(result.success if kind == "arm" else result.mode_sent)
            except Exception as error:
                self.node.get_logger().warning("控制请求返回异常：" + str(error))
            self._event("command_result", dict(command=kind, accepted=accepted))
            self.controller.command_result(kind, accepted)
            del self.pending[kind]

    def _request(self, kind, now):
        if kind in self.pending:
            return
        client = self.arm_client if kind == "arm" else self.mode_client
        if client is None:
            raise RuntimeError("This operating mode has no " + kind + " interface")
        if not client.service_is_ready():
            self.controller.command_result(kind, False)
            return
        req = self.CommandBool.Request() if kind == "arm" else self.SetMode.Request()
        if kind == "arm":
            req.value = True  # No disarm call exists in this adapter.
        else:
            req.base_mode, req.custom_mode = 0, kind
        self.pending[kind] = (client.call_async(req), now)
        event = dict(command=kind, monotonic=now)
        self.command_events.append(event)
        self._event("command_request", event)
        self.node.get_logger().info("请求：" + ("解锁一次" if kind == "arm" else kind))

    def snapshot(self):
        return Vehicle(boot_id=self.boot_id, other_setpoint_publishers=self.other_publishers,
                       **{key: self.values.get(key, Observation()) for key in (
                           "state", "rc", "pose", "velocity", "landed", "estimator", "health",
                           "boot", "lio", "vision", "parameters")})

    def request_stop(self):
        if self.stop_at is None:
            self.stop_at = time.monotonic()
            self.controller.request_exit("operator_stop")

    def _tick(self):
        now = time.monotonic()
        try:
            self._parameters(now)
            self._command_results(now)
            output = self.controller.step(self.snapshot(), now)
            self.last_output = output
            if output.request_mode is not None:
                self._request(output.request_mode, now)
            if output.arm:
                self._request("arm", now)
            if output.target is not None:
                if self.target_pub is None:
                    raise RuntimeError("Read-only mode attempted to publish a target")
                x, y, z, yaw = output.target
                m = self.PoseStamped()
                m.header.stamp = self.node.get_clock().now().to_msg()
                m.header.frame_id = self.pose_frame
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
                m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                self.target_pub.publish(m)
                self.controller.note_target_sent(time.monotonic())
            state = self.values.get("state", Observation()).value or {}
            rc = self.values.get("rc", Observation()).value or []
            status = dict(run_id=self.run_id, operating_mode=self.options.mode,
                          phase=output.phase, actual_mode=state.get("mode"), armed=state.get("armed"),
                          reasons=list(output.reasons), target=output.target, exit_reason=output.exit_reason,
                          ready_for_switch=output.ready_for_switch,
                          handover_confirmed=output.handover_confirmed,
                          swa_pwm=rc[6] if len(rc)>6 else None,
                          throttle_pwm=rc[2] if len(rc)>2 else None,
                          manual_control_enabled=state.get("manual_input"),
                          rc_link_healthy=(self.values.get("health", Observation()).value or {}).get("rc_ready"),
                          rc_age_s=(None if self.values.get("rc", Observation()).at is None
                                    else now-self.values["rc"].at),
                          other_setpoint_publishers=self.other_publishers,
                          updated_monotonic=now, boot_id=self.boot_id)
            self.last_status = status
            phase_key = (output.phase, output.exit_reason, output.reasons)
            changed = phase_key != self._last_phase_key
            if changed or self.last_report is None or now-self.last_report >= 1.0:
                message = self.String()
                message.data = json.dumps(status, ensure_ascii=False, allow_nan=False)
                self.status_pub.publish(message)
                self.last_report = now
                if changed:
                    self._last_phase_key = phase_key
                    suffix = "；原因=" + "；".join(output.reasons) if output.reasons else ""
                    self.node.get_logger().info(PHASE_TEXT[output.phase] + suffix)
                    self._event("state", status)
            if self.options.mode == "check" and self.options.duration is not None and now-self.started >= self.options.duration:
                self.finished = True
            if self.stop_at is not None:
                if ((output.phase == "EXITED" and output.handover_confirmed and now-self.stop_at >= .5)
                        or now-self.stop_at >= self.controller.config.handover_timeout_s+1.2
                        or self.options.mode == "check"):
                    self.finished = True
        except Exception as error:
            # Do not continue producing a climbing target after an adapter error.
            self.controller.request_exit("adapter_exception")
            self.node.get_logger().error("悬停节点异常，停止自动目标：" + str(error))
            if self.options.mode != "check":
                try:
                    self._request("POSCTL", now)
                except Exception:
                    pass
            if self.stop_at is None:
                self.stop_at = now
            elif now-self.stop_at >= self.controller.config.handover_timeout_s+1.2:
                self.finished = True

    def close(self):
        self._event("close", dict(last_status=self.last_status, recorder=self.recorder.stats()))
        if not self.recorder.close(timeout=2.):
            self.node.get_logger().warning("日志线程2秒内未排空；不阻塞控制退出，部分记录可能未落盘。")
        elif self.recorder.stats()["write_errors"]:
            self.node.get_logger().warning("日志写入出现错误："+str(self.recorder.stats()["last_error"]))
        stats = self.recorder.stats()
        if stats["dropped_telemetry"] or stats["dropped_critical"]:
            self.node.get_logger().warning(f"日志队列丢弃：telemetry={stats['dropped_telemetry']}，critical={stats['dropped_critical']}。")
        self.node.destroy_node()


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in arguments:
        from rclpy.utilities import remove_ros_args
        arguments = remove_ros_args(args=arguments)
    options = parse_options(arguments)
    try:
        config = load_hover_config(options.config)
    except (OSError, ValueError, TypeError) as error:
        build_parser().error(str(error))
    if options.describe:
        print(json.dumps(dict(mode=options.mode, config=asdict(config),
                              rc_channels=dict(swa=7, throttle=3, main_mode=5)), ensure_ascii=False))
        return 0
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import ExternalShutdownException
    stop = [False]
    previous = {}
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    runner = None
    try:
        runner = HoverNode(options, config)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, lambda *_: stop.__setitem__(0, True))
        while rclpy.ok() and not runner.finished:
            if stop[0]:
                runner.request_stop()
            rclpy.spin_once(runner.node, timeout_sec=.02)
    except (KeyboardInterrupt, ExternalShutdownException):
        # External context loss prevents further ROS requests; PX4 timeout and
        # the native RC mode switch remain the independent exit paths.
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
