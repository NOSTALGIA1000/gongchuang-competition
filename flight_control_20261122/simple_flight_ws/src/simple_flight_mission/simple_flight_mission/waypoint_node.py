"""Manual-hover to simple incremental waypoint mission.

The default invocation is a read-only preview.  The execute path is
deliberately explicit and only takes over an already armed, stabilized
vehicle after the configured RC switch is moved to its takeover position.
"""

import argparse
import json
import math
import time
from pathlib import Path

from .mission_config import MissionConfig, load_config


def quaternion_yaw(q):
    """Return yaw from a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def expand_world_points(origin, yaw_rad, relative_points):
    """Convert forward/left offsets into MAVROS ENU world points."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    ox, oy, oz = origin
    return [
        (ox + forward * c - left * s,
         oy + forward * s + left * c,
         oz)
        for forward, left in relative_points
    ]


def _finite_pose(pose):
    return all(math.isfinite(value) for value in (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    ))


def format_bench_status(mode, armed, position, horizontal_speed, position_age):
    """Format the read-only bench status shown to the operator."""
    if position is None:
        position_text = "位置=未知 高度=未知"
    else:
        position_text = (
            f"位置=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) "
            f"高度={position[2]:.2f} m"
        )
    speed_text = "未知" if not math.isfinite(horizontal_speed) else f"{horizontal_speed:.2f} m/s"
    age_text = "未知" if position_age is None else f"{position_age:.2f} s"
    return (
        f"模式={mode} 解锁={armed} {position_text} "
        f"水平速度={speed_text} 位置年龄={age_text}"
    )


def _rc_pwm(channels, channel):
    index = channel - 1
    if index < 0 or index >= len(channels):
        return None
    value = int(channels[index])
    return None if value in (0, 65535) else value


def rc_switch_active(channels, channel=5, high_pwm=1800):
    value = _rc_pwm(channels, channel)
    return value is not None and value >= high_pwm


def rc_switch_low(channels, channel=5, low_pwm=1200):
    value = _rc_pwm(channels, channel)
    return value is not None and value <= low_pwm


def manual_stick_active(channels, threshold=0.2):
    """Return whether roll, pitch, or yaw shows a deliberate stick input.

    Throttle is intentionally excluded because a manual hover requires it to
    be away from its low endpoint and it must not trigger takeover aborts.
    """
    for channel in (1, 2, 4):
        value = _rc_pwm(channels, channel)
        if value is not None and abs((value - 1500.0) / 500.0) > threshold:
            return True
    return False


def throttle_drop_active(channels, baseline_pwm, drop_pwm=150):
    """Return whether throttle dropped materially below the hover baseline."""
    value = _rc_pwm(channels, 3)
    return (value is not None and baseline_pwm is not None
            and value <= baseline_pwm - drop_pwm)


def offboard_wait_mode_allowed(mode):
    return mode in ("POSCTL", "OFFBOARD")


def mode_request_expired(started_at, now, timeout_s=2.0):
    return started_at is not None and now - started_at >= timeout_s


def format_mode_request_result(mode, attempt, mode_sent):
    return f"{mode} 第 {attempt} 次服务返回 mode_sent={bool(mode_sent)}"


def mode_retry_due(last_request_at, now, interval_s=1.0):
    return last_request_at is not None and now - last_request_at >= interval_s


class WaypointMissionNode:
    """ROS 2 node implementing preview and explicit execute modes."""

    def __init__(self, config: MissionConfig, execute: bool,
                 config_path: str, node=None, bench: bool = False):
        import rclpy
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from mavros_msgs.msg import ExtendedState, RCIn, State
        from mavros_msgs.srv import SetMode
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self.rclpy = rclpy
        self.node = node or rclpy.create_node("waypoint_mission")
        self.PoseStamped = PoseStamped
        self.execute = execute
        self.config = config
        self.config_path = config_path
        self.set_mode_type = SetMode
        self.pose = None
        self.velocity = None
        self.state = None
        self.extended_state = None
        self.boot_status = None
        self.last_pose_monotonic = None
        self.last_velocity_monotonic = None
        self.last_observed_position = None
        self.position_jump_detected = False
        self.last_rc = None
        self.rc_active_since = None
        self.throttle_baseline_pwm = None
        self.phase = "拆桨检查" if bench else ("等待 SWA" if execute else "预演")
        self.origin = None
        self.yaw_rad = None
        self.targets = []
        self.target_index = 0
        self.segment_start = None
        self.segment_started_at = None
        self.hold_started_at = None
        self.stable_started_at = None
        self.prewarm_started_at = None
        self.return_hold_started_at = None
        self.mode_future = None
        self.mode_requested = None
        self.mode_attempts = 0
        self.mode_request_started_at = None
        self.mode_last_request_at = None
        self.offboard_wait_started_at = None
        self.abort_reason = None
        self.preview_printed = False
        self.orientation = None
        self.takeover_switch_seen_low = False
        self.switch_takeover_active = False

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pose_sub = self.node.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._on_pose, sensor_qos)
        self.velocity_sub = self.node.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local",
            self._on_velocity, sensor_qos)
        self.state_sub = self.node.create_subscription(
            State, "/mavros/state", self._on_state, 10)
        self.extended_state_sub = self.node.create_subscription(
            ExtendedState, "/mavros/extended_state", self._on_extended_state, 10)
        self.rc_sub = self.node.create_subscription(
            RCIn, "/mavros/rc/in", self._on_rc, 10)
        self.boot_sub = self.node.create_subscription(
            String, "/simple_flight/boot/status", self._on_boot_status, 10)
        self.setpoint_pub = self.node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.mode_client = self.node.create_client(SetMode, "/mavros/set_mode")
        self.timer = self.node.create_timer(
            1.0 / self.config.setpoint_hz, self._tick)
        self.report_timer = self.node.create_timer(1.0, self._report)

        if execute:
            self.node.get_logger().info(
                "实飞模式已选择；确认人工起飞并悬停，SWA 先置低档，检查通过后拨到高档才会接管")

    def _on_pose(self, msg):
        if _finite_pose(msg.pose):
            current = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            if (self.last_observed_position is not None
                    and math.dist(current, self.last_observed_position)
                    > self.config.max_position_jump_m):
                self.position_jump_detected = True
            self.last_observed_position = current
            self.pose = msg.pose
            self.last_pose_monotonic = time.monotonic()

    def _on_velocity(self, msg):
        self.velocity = msg.twist
        self.last_velocity_monotonic = time.monotonic()

    def _on_state(self, msg):
        self.state = msg

    def _on_extended_state(self, msg):
        self.extended_state = msg

    def _on_rc(self, msg):
        self.last_rc = list(msg.channels)
        if rc_switch_low(self.last_rc, self.config.rc_takeover_channel,
                         self.config.rc_takeover_low_pwm):
            self.takeover_switch_seen_low = True

    def _on_boot_status(self, msg):
        try:
            self.boot_status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.boot_status = None

    def _accept_switch(self):
        # A zero-period timer is one-shot only after this callback destroys it.
        reasons = self._preflight_reasons()
        if reasons:
            self.node.get_logger().error("无法接管：" + "；".join(reasons))
            return
        self.phase = "稳定悬停确认"
        self.switch_takeover_active = True
        # Capture the manual hover throttle before any OFFBOARD control starts.
        # This also lets a low-throttle action abort during the 3 s confirmation.
        self.throttle_baseline_pwm = _rc_pwm(self.last_rc or [], 3)
        self.stable_started_at = time.monotonic()
        self.node.get_logger().info("SWA 已拨到接管档，开始确认 3 秒稳定悬停")

    def _preflight_reasons(self):
        reasons = []
        if self.state is None or not self.state.connected:
            reasons.append("FCU 未连接")
        if self.state is None or not self.state.armed:
            reasons.append("无人机未解锁")
        if self.state is None or self.state.mode != "POSCTL":
            reasons.append("当前模式不是 POSCTL")
        if self.state is None or not self.state.manual_input:
            reasons.append("遥控输入未就绪")
        if self.pose is None or self.last_pose_monotonic is None:
            reasons.append("没有本地位置")
        elif time.monotonic() - self.last_pose_monotonic > self.config.max_position_age_s:
            reasons.append("位置数据已过期")
        if self.pose is not None and not (
                self.config.min_height_m <= self.pose.position.z <= self.config.max_height_m):
            reasons.append("当前高度不在 0.8～1.5 m")
        if self.boot_status is not None:
            if self.boot_status.get("state") not in ("READY", "ARMED"):
                reasons.append("定位服务未就绪")
            snapshot = self.boot_status.get("snapshot", {})
            if not snapshot.get("position_valid", False):
                reasons.append("定位无效")
        return reasons

    def _tick(self):
        if self.phase == "拆桨检查":
            return
        if self.phase == "预演":
            if self.pose is not None and not self.preview_printed:
                self._print_preview()
            return
        if self.phase == "等待 SWA":
            if (self.takeover_switch_seen_low and
                    rc_switch_active(self.last_rc or [],
                                     self.config.rc_takeover_channel,
                                     self.config.rc_takeover_high_pwm)):
                self._accept_switch()
            return
        if self.phase in ("已中止", "已完成"):
            return
        if self.phase == "稳定悬停确认":
            self._tick_stabilizing()
        elif self.phase == "OFFBOARD预发送":
            self._tick_prewarm()
        elif self.phase == "等待 OFFBOARD":
            self._tick_wait_offboard()
        elif self.phase in ("飞往航点", "航点悬停", "返回点悬停"):
            self._tick_mission()
        elif self.phase == "等待人工降落":
            self._tick_landing_wait()
        self._publish_target()

    def _tick_stabilizing(self):
        if self._common_health_failed(expected_mode="POSCTL"):
            return
        if self._speed() > self.config.arrival_speed_mps:
            self.stable_started_at = time.monotonic()
            return
        if time.monotonic() - self.stable_started_at < self.config.startup_stable_s:
            return
        self.origin = (self.pose.position.x, self.pose.position.y,
                       self.pose.position.z)
        self.yaw_rad = quaternion_yaw(self.pose.orientation)
        self.orientation = self.pose.orientation
        self.targets = expand_world_points(
            self.origin, self.yaw_rad, self.config.relative_points())
        self.target_index = 0
        self.segment_start = self.origin
        self.segment_started_at = time.monotonic()
        self.prewarm_started_at = time.monotonic()
        self.phase = "OFFBOARD预发送"
        self.node.get_logger().info("原点和初始 yaw 已锁定，开始 OFFBOARD 预发送")

    def _tick_prewarm(self):
        if self._common_health_failed(expected_mode="POSCTL"):
            return
        if time.monotonic() - self.prewarm_started_at < self.config.offboard_prewarm_s:
            return
        self._request_mode("OFFBOARD")
        self.offboard_wait_started_at = time.monotonic()
        self.phase = "等待 OFFBOARD"

    def _tick_wait_offboard(self):
        if self.state is None or not offboard_wait_mode_allowed(self.state.mode):
            self._abort("模式已离开 POSCTL/OFFBOARD")
            return
        if self._common_health_failed(expected_mode=None):
            return
        if (self.offboard_wait_started_at is not None
                and time.monotonic() - self.offboard_wait_started_at >= 6.0):
            self._abort("OFFBOARD 6 秒内未切换成功")
            return
        if self.state is not None and self.state.mode == "OFFBOARD":
            self.phase = "飞往航点"
            self.segment_started_at = time.monotonic()
            self.node.get_logger().info("已进入 OFFBOARD，开始执行航点")
        elif (self.mode_future is not None and not self.mode_future.done()
              and mode_request_expired(self.mode_request_started_at, time.monotonic())):
            self.mode_future.cancel()
            self.mode_future = None
            self.mode_request_started_at = None
            self.node.get_logger().warning("OFFBOARD 请求超时，准备重试")
        elif (self.mode_future is None and self.mode_attempts < 3
              and (self.mode_last_request_at is None
                   or mode_retry_due(self.mode_last_request_at, time.monotonic()))):
            self._request_mode("OFFBOARD")
        elif self.mode_future is not None and self.mode_future.done():
            response = self.mode_future.result()
            self.mode_future = None
            self.mode_request_started_at = None
            mode_sent = bool(response and response.mode_sent)
            self.node.get_logger().info(format_mode_request_result(
                "OFFBOARD", self.mode_attempts, mode_sent))
            if not mode_sent:
                if self.mode_attempts < 3:
                    self._request_mode("OFFBOARD")
                else:
                    self._abort("OFFBOARD 请求失败")

    def _tick_mission(self):
        if self._takeover_or_bad_state():
            return
        if self.phase == "飞往航点":
            if self._at_target():
                self.phase = "返回点悬停" if self.target_index == len(self.targets) - 1 else "航点悬停"
                self.hold_started_at = time.monotonic()
                return
            if time.monotonic() - self.segment_started_at > self.config.segment_timeout_s:
                self._abort(f"第 {self.target_index + 1} 段超过 30 秒")
        elif self.phase == "航点悬停":
            if not self._at_target():
                self.phase = "飞往航点"
                self.segment_started_at = time.monotonic()
                return
            if time.monotonic() - self.hold_started_at >= self.config.hold_s:
                self.target_index += 1
                self.segment_start = self._position()
                self.segment_started_at = time.monotonic()
                self.phase = "飞往航点"
        elif self.phase == "返回点悬停":
            if not self._at_target():
                self.phase = "飞往航点"
                self.segment_started_at = time.monotonic()
                return
            if time.monotonic() - self.hold_started_at >= self.config.hold_s:
                self._request_mode("POSCTL")
                self.phase = "等待人工降落"
                self.node.get_logger().info("已回到原点上方，交还 POSCTL，请人工降落")

    def _tick_landing_wait(self):
        if self.state is not None and not self.state.armed:
            landed = self.extended_state is None or self.extended_state.landed_state == 1
            if landed:
                self.phase = "已完成"
                self.node.get_logger().info("检测到已落地且已解除解锁，任务完成")

    def _publish_target(self):
        if not self.execute or self.origin is None:
            return
        target = self._current_target()
        msg = self.PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
        msg.pose.orientation = self.orientation if self.orientation is not None else msg.pose.orientation
        self.setpoint_pub.publish(msg)

    def _current_target(self):
        if self.phase == "飞往航点" and self.segment_start is not None:
            target = self.targets[self.target_index]
            distance = math.dist(self.segment_start, target)
            duration = distance / self.config.max_speed_mps if distance else 0.0
            fraction = 1.0 if duration == 0 else min(
                1.0, (time.monotonic() - self.segment_started_at) / duration)
            return tuple(start + (end - start) * fraction
                         for start, end in zip(self.segment_start, target))
        if self.phase in ("等待人工降落", "返回点悬停") and self.targets:
            return self.targets[-1]
        return self._position() if self.pose is not None else (0.0, 0.0, 0.0)

    def _at_target(self):
        if self.pose is None or not self.targets:
            return False
        target = self.targets[self.target_index]
        horizontal_error = math.hypot(
            self.pose.position.x - target[0], self.pose.position.y - target[1])
        return horizontal_error <= self.config.arrival_radius_m and self._speed() <= self.config.arrival_speed_mps

    def _position(self):
        return (self.pose.position.x, self.pose.position.y, self.pose.position.z)

    def _speed(self):
        if self.velocity is None:
            return float("inf")
        return math.hypot(self.velocity.linear.x, self.velocity.linear.y)

    def _takeover_or_bad_state(self):
        if self.switch_takeover_active and rc_switch_low(
                self.last_rc or [], self.config.rc_takeover_channel,
                self.config.rc_takeover_low_pwm):
            self._abort("SWA 已回到安全档")
            return True
        if self.state is None or self.state.mode != "OFFBOARD":
            self._abort("模式已离开 OFFBOARD")
            return True
        if self.last_pose_monotonic is None or time.monotonic() - self.last_pose_monotonic > self.config.max_position_age_s:
            self._abort("位置数据超过 0.3 秒未更新")
            return True
        if self._speed() > self.config.max_horizontal_speed_mps:
            self._abort("水平速度超过 0.8 m/s")
            return True
        if self._rc_takeover():
            self._abort("检测到遥控器明显动作")
            return True
        return False

    def _common_health_failed(self, expected_mode=None):
        if self.switch_takeover_active and rc_switch_low(
                self.last_rc or [], self.config.rc_takeover_channel,
                self.config.rc_takeover_low_pwm):
            self._abort("SWA 已回到安全档")
            return True
        if self.state is None or not self.state.connected or not self.state.armed:
            self._abort("飞控连接或解锁状态异常")
            return True
        if expected_mode and self.state.mode != expected_mode:
            self._abort("模式已离开 " + expected_mode)
            return True
        if self.last_pose_monotonic is None or time.monotonic() - self.last_pose_monotonic > self.config.max_position_age_s:
            self._abort("位置数据超过 0.3 秒未更新")
            return True
        if self._speed() > self.config.max_horizontal_speed_mps:
            self._abort("水平速度超过 0.8 m/s")
            return True
        if self.position_jump_detected:
            self._abort("位置单次跳变超过 0.5 m")
            return True
        if self._rc_takeover():
            self._abort("检测到遥控器明显动作")
            return True
        return False

    def _rc_takeover(self):
        if not self.last_rc:
            return False
        active = (manual_stick_active(self.last_rc, self.config.takeover_threshold)
                  or throttle_drop_active(
                      self.last_rc, self.throttle_baseline_pwm,
                      self.config.throttle_drop_pwm))
        if active:
            if self.rc_active_since is None:
                self.rc_active_since = time.monotonic()
            return time.monotonic() - self.rc_active_since >= self.config.takeover_hold_s
        self.rc_active_since = None
        return False

    def _request_mode(self, mode):
        if self.mode_future is not None and not self.mode_future.done():
            return
        if not self.mode_client.service_is_ready():
            self.node.get_logger().warning("模式服务尚未就绪：" + mode)
            return
        request = self.set_mode_type.Request()
        request.base_mode = 0
        request.custom_mode = mode
        self.mode_requested = mode
        self.mode_attempts += 1
        self.mode_request_started_at = time.monotonic()
        self.mode_last_request_at = self.mode_request_started_at
        self.mode_future = self.mode_client.call_async(request)

    def _abort(self, reason):
        if self.phase in ("已中止", "已完成", "等待人工降落"):
            return
        self.abort_reason = reason
        self.phase = "已中止"
        self.node.get_logger().error("任务已中止：" + reason)
        try:
            self._request_mode("POSCTL")
        except Exception as exc:
            self.node.get_logger().warning("退出时未能请求 POSCTL：" + str(exc))

    def _report(self):
        mode = self.state.mode if self.state is not None else "未知"
        armed = self.state.armed if self.state is not None else False
        if self.phase == "拆桨检查":
            age = (None if self.last_pose_monotonic is None
                   else time.monotonic() - self.last_pose_monotonic)
            position = self._position() if self.pose is not None else None
            self.node.get_logger().info(format_bench_status(
                mode, armed, position, self._speed(), age))
            return
        self.node.get_logger().info(
            f"阶段={self.phase} 模式={mode} 解锁={armed} 航点={self.target_index + 1}/4")

    def _print_preview(self):
        self.preview_printed = True
        yaw = quaternion_yaw(self.pose.orientation)
        origin = (self.pose.position.x, self.pose.position.y, self.pose.position.z)
        points = expand_world_points(origin, yaw, self.config.relative_points())
        self.node.get_logger().info("预演模式：不调用解锁、模式切换或运动控制服务")
        self.node.get_logger().info(f"接管原点={origin} 初始 yaw={yaw:.3f} rad")
        self.node.get_logger().info(f"展开航点={points}")
        self.node.get_logger().info(
            "阈值：最大速度 0.3 m/s，到点误差 0.2 m，稳定 2 s，航段超时 30 s")


def build_parser():
    parser = argparse.ArgumentParser(description="简单三段增量航点任务")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--execute", action="store_true", help="允许执行实飞控制")
    modes.add_argument("--bench", action="store_true", help="拆桨只读检查，不发送控制服务")
    parser.add_argument(
        "--config", default="/home/jetson/simple_flight_ws/config/mission.yaml",
        help="任务配置 YAML 路径")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    import rclpy

    config = load_config(Path(args.config))
    node = None
    try:
        rclpy.init()
        node = WaypointMissionNode(config, args.execute, args.config, bench=args.bench)
        rclpy.spin(node.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        if (node is not None and args.execute and rclpy.ok()
                and node.phase not in ("等待 SWA", "已中止", "已完成")):
            node._abort("终端 Ctrl+C")
    finally:
        if node is not None:
            node.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
