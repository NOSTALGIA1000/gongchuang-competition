"""Message-level ROS integration only. Never open a serial port.

Requires ROS_DOMAIN_ID=77 and ROS_LOCALHOST_ONLY=1 before importing ROS.
This is a virtual telemetry/service peer, not PX4 SITL or a flight model.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

if os.environ.get("ROS_DOMAIN_ID") != "77" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("Refusing test: require isolated ROS_DOMAIN_ID=77 and ROS_LOCALHOST_ONLY=1")

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from mavros_msgs.msg import State, RCIn, ExtendedState, EstimatorStatus, SysStatus
from mavros_msgs.srv import SetMode, CommandBool
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String

from simple_flight_mission.hover_controller import EXPECTED_PARAMETERS
from simple_flight_mission.hover_node import HoverNode, parse_options


class VirtualVehicle(Node):
    def __init__(self):
        super().__init__("hover_virtual_vehicle")
        self.mode = "POSCTL"
        self.armed = False
        self.swa = 1024
        self.throttle = 1024
        self.z = 4.2
        self.last_z = self.z
        self.target = None
        self.targets = []
        self.mode_calls = []
        self.arm_calls = []
        self.statuses = []
        self.reject_offboard = False
        self.reject_position = False
        self.arm_delay = 0.0
        self.arm_due = None
        self.pose_enabled = True
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.run_id = "virtual_" + str(time.monotonic_ns())
        self.pub = {}
        for key, cls, topic in (
            ("state", State, "/mavros/state"), ("rc", RCIn, "/mavros/rc/in"),
            ("landed", ExtendedState, "/mavros/extended_state"),
            ("pose", PoseStamped, "/mavros/local_position/pose"),
            ("velocity", TwistStamped, "/mavros/local_position/velocity_local"),
            ("lio", Odometry, "/Odometry"),
            ("vision", PoseWithCovarianceStamped, "/mavros/vision_pose/pose_cov"),
            ("estimator", EstimatorStatus, "/mavros/estimator_status"),
            ("health", SysStatus, "/mavros/sys_status"),
            ("boot", String, "/simple_flight/boot/status"),
        ):
            self.pub[key] = self.create_publisher(cls, topic, 10)
        self.sub = self.create_subscription(PoseStamped, "/mavros/setpoint_position/local", self.on_target, 10)
        self.status_sub = self.create_subscription(String, "/simple_flight/hover/status", self.on_status, 10)
        self.test_services = [
            self.create_service(GetParameters, "/mavros/param/get_parameters", self.parameters),
            self.create_service(SetMode, "/mavros/set_mode", self.set_mode),
            self.create_service(CommandBool, "/mavros/cmd/arming", self.arm),
        ]
        self.timer = self.create_timer(.05, self.publish)

    def parameters(self, req, response):
        values = dict(EXPECTED_PARAMETERS, COM_OF_LOSS_T=1.0, RC_OFFB_TH=.5)
        response.values = []
        for name in req.names:
            p = ParameterValue()
            value = values.get(name)
            if type(value) is int:
                p.type, p.integer_value = 2, value
            elif isinstance(value, float):
                p.type, p.double_value = 3, value
            response.values.append(p)
        return response

    def set_mode(self, req, response):
        self.mode_calls.append(req.custom_mode)
        rejected = ((req.custom_mode == "OFFBOARD" and self.reject_offboard)
                    or (req.custom_mode == "POSCTL" and self.reject_position))
        response.mode_sent = not rejected
        if not rejected:
            self.mode = req.custom_mode
        return response

    def arm(self, req, response):
        self.arm_calls.append(req.value)
        assert req.value is True, "No test may send disarm"
        if self.arm_delay:
            self.arm_due = time.monotonic() + self.arm_delay
        else:
            self.armed = True
        response.success, response.result = True, 0
        return response

    def on_target(self, m):
        self.target = m
        self.targets.append((time.monotonic(), m.pose.position.z))

    def on_status(self, m):
        self.statuses.append(json.loads(m.data))

    def publish(self):
        now = time.monotonic()
        if self.arm_due is not None and now >= self.arm_due:
            self.armed, self.arm_due = True, None
        self.last_z = self.z
        if self.armed and self.mode == "OFFBOARD" and self.target is not None:
            self.z += max(-.012, min(.012, self.target.pose.position.z-self.z))
        stamp = self.get_clock().now().to_msg()
        state = State()
        state.header.stamp = stamp
        state.connected, state.armed, state.manual_input, state.mode = True, self.armed, self.mode != "OFFBOARD", self.mode
        self.pub["state"].publish(state)
        rc = RCIn()
        rc.header.stamp = stamp
        rc.channels = [1514, 1514, self.throttle, 1514, 2003, 1514, self.swa]
        self.pub["rc"].publish(rc)
        landed = ExtendedState()
        landed.landed_state = 2 if self.z > 4.25 else 1
        self.pub["landed"].publish(landed)
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "map"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 2.0, -3.0, self.z
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(.35), math.cos(.35)
        if self.pose_enabled:
            self.pub["pose"].publish(pose)
        velocity = TwistStamped()
        velocity.header = pose.header
        velocity.twist.linear.z = (self.z-self.last_z)/.05
        self.pub["velocity"].publish(velocity)
        lio = Odometry()
        lio.header, lio.pose.pose = pose.header, pose.pose
        self.pub["lio"].publish(lio)
        vision = PoseWithCovarianceStamped()
        vision.header, vision.pose.pose = pose.header, pose.pose
        self.pub["vision"].publish(vision)
        est = EstimatorStatus()
        est.attitude_status_flag = est.velocity_horiz_status_flag = est.velocity_vert_status_flag = True
        est.pos_horiz_rel_status_flag = est.pos_vert_abs_status_flag = True
        est.pred_pos_horiz_rel_status_flag = True
        est.const_pos_mode_status_flag = not self.armed
        self.pub["estimator"].publish(est)
        health = SysStatus()
        health.sensors_present = health.sensors_enabled = 1 << 16
        health.sensors_health = (1 << 16) | (1 << 28)
        self.pub["health"].publish(health)
        boot = String()
        boot.data = json.dumps(dict(boot_id=self.boot_id, run_id=self.run_id, fault=None,
                                   localization_active=True, ready_to_arm=True,
                                   updated_monotonic=now, valid_until_monotonic=now+2.5))
        self.pub["boot"].publish(boot)


def pump(executor, seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=.01)


def until(executor, condition, seconds, description):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=.01)
        if condition():
            return
    raise AssertionError("Timed out: " + description)


def run_case(name, mode, test):
    executor = SingleThreadedExecutor()
    vehicle = VirtualVehicle()
    options = parse_options(["--"+mode] if mode != "check" else [])
    runner = HoverNode(options)
    executor.add_node(vehicle)
    executor.add_node(runner.node)
    start = time.monotonic()
    try:
        test(executor, vehicle, runner)
        result = dict(name=name, passed=True, seconds=round(time.monotonic()-start, 3),
                      arm_calls=vehicle.arm_calls, mode_calls=vehicle.mode_calls,
                      max_relative_height=max([z-4.2 for _, z in vehicle.targets] or [0]),
                      final_phase=runner.controller.phase)
        print(json.dumps(result), flush=True)
        return result
    finally:
        executor.remove_node(runner.node)
        executor.remove_node(vehicle)
        runner.close()
        vehicle.destroy_node()
        executor.shutdown()


def ready(executor, runner):
    until(executor, lambda: runner.controller.phase == "READY", 13,
          "READY: " + str(runner.last_status))


def begin(executor, vehicle, runner, phase="TAKEOFF"):
    ready(executor, runner)
    vehicle.swa = 2003
    until(executor, lambda: runner.controller.phase == phase, 13, phase)


def readonly(executor, vehicle, runner):
    pump(executor, 2)
    assert runner.target_pub is runner.mode_client is runner.arm_client is None
    assert not vehicle.targets and not vehicle.mode_calls and not vehicle.arm_calls
    clients = runner.node.get_client_names_and_types_by_node(runner.node.get_name(), runner.node.get_namespace())
    assert not any(n in ("/mavros/set_mode", "/mavros/cmd/arming") for n, _ in clients)
    assert runner.last_status["reasons"] == []


def mode_test(executor, vehicle, runner):
    begin(executor, vehicle, runner, "MODE_TEST")
    pump(executor, .3)
    assert runner.arm_client is None and not vehicle.arm_calls
    assert all(abs(z-4.2) < 1e-6 for _, z in vehicle.targets)
    vehicle.swa = 1024
    until(executor, lambda: runner.controller.phase == "EXITED", 2, "mode-test exit")


def takeoff_swa(executor, vehicle, runner):
    begin(executor, vehicle, runner, "HOVER")
    assert vehicle.arm_calls == [True]
    assert abs(vehicle.z-5.2) < .1
    vehicle.swa = 1024
    until(executor, lambda: runner.controller.phase == "EXITED", 2, "SWA handover")
    pump(executor, .2)
    count, mode_count = len(vehicle.targets), len(vehicle.mode_calls)
    vehicle.swa = 2003
    pump(executor, .6)
    assert len(vehicle.targets) == count and vehicle.arm_calls == [True]
    assert len(vehicle.mode_calls) == mode_count
    assert runner.controller.handover_confirmed


def throttle_exit(executor, vehicle, runner):
    begin(executor, vehicle, runner)
    pump(executor, .6)
    assert runner.controller.phase == "TAKEOFF"
    vehicle.throttle = 1514
    pump(executor, .4)
    vehicle.throttle = 1024
    until(executor, lambda: runner.controller.phase == "EXITED", 2, "throttle handover")
    assert runner.last_output.exit_reason == "throttle_raise_then_low"


def mode_rejected(executor, vehicle, runner):
    vehicle.reject_offboard = True
    ready(executor, runner)
    vehicle.swa = 2003
    until(executor, lambda: runner.controller.phase == "EXITED", 3, "mode rejection")
    assert not vehicle.arm_calls


def stale_position(executor, vehicle, runner):
    begin(executor, vehicle, runner)
    vehicle.pose_enabled = False
    until(executor, lambda: runner.controller.phase == "EXITED", 2, "stale pose exit")
    assert "pose_stale" in runner.last_output.exit_reason
    assert runner.last_output.target is None


def late_arm(executor, vehicle, runner):
    vehicle.arm_delay = .8
    ready(executor, runner)
    vehicle.swa = 2003
    until(executor, lambda: len(vehicle.arm_calls) == 1, 3, "pending arm")
    vehicle.swa = 1024
    until(executor, lambda: runner.controller.phase == "EXITED", 2, "cancel pending arm")
    pump(executor, 1.0)
    assert vehicle.armed and runner.controller.phase == "EXITED"
    assert runner.last_output.target is None and vehicle.arm_calls == [True]
    assert vehicle.z == 4.2


def handover_timeout(executor, vehicle, runner):
    begin(executor, vehicle, runner)
    vehicle.reject_position = True
    vehicle.swa = 1024
    until(executor, lambda: runner.controller.phase == "EXITED", 2, "handover timeout")
    assert runner.last_output.target is None and not runner.controller.handover_confirmed
    pump(executor, .1)
    count = len(vehicle.targets)
    pump(executor, .4)
    assert len(vehicle.targets) == count


def subprocess_signal():
    executor = SingleThreadedExecutor()
    vehicle = VirtualVehicle()
    executor.add_node(vehicle)
    proc = subprocess.Popen([sys.executable, "-m", "simple_flight_mission.hover_node", "--execute", "--ros-args"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        until(executor, lambda: any(s["phase"] == "READY" for s in vehicle.statuses), 15, "child READY")
        vehicle.swa = 2003
        until(executor, lambda: vehicle.armed, 4, "child armed")
        pump(executor, .2)
        proc.send_signal(signal.SIGINT)
        until(executor, lambda: proc.poll() is not None, 4, "SIGINT process exit")
        output = proc.communicate(timeout=2)[0]
        assert proc.returncode == 0, output
        assert vehicle.mode == "POSCTL" and vehicle.arm_calls == [True]
        return dict(name="real_process_SIGINT_and_ros_args", passed=True, arm_calls=vehicle.arm_calls,
                    mode_calls=vehicle.mode_calls, child_returncode=proc.returncode)
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGTERM)
            pump(executor, 3)
            if proc.poll() is None:
                proc.kill()
        executor.remove_node(vehicle)
        vehicle.destroy_node()
        executor.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    results = []
    try:
        for name, mode, test in (
            ("readonly_no_control_interfaces", "check", readonly),
            ("mode_test_no_arm", "mode-test", mode_test),
            ("relative_takeoff_hover_SWA_exit", "execute", takeoff_swa),
            ("throttle_raise_low_exit", "execute", throttle_exit),
            ("mode_request_rejected", "execute", mode_rejected),
            ("pose_timeout", "execute", stale_position),
            ("late_arm_after_cancel", "execute", late_arm),
            ("handover_timeout_stops_targets", "execute", handover_timeout),
        ):
            results.append(run_case(name, mode, test))
        results.append(subprocess_signal())
        result = dict(passed=True, isolation=dict(ROS_DOMAIN_ID=77, ROS_LOCALHOST_ONLY=1),
                      scope="ROS message/service integration; NOT PX4 SITL or actual flight", cases=results)
    except Exception as error:
        result = dict(passed=False, error=repr(error), completed_cases=results)
        raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
        if rclpy.ok():
            rclpy.shutdown()
    print("ALL_ISOLATED_CASES_PASSED", flush=True)


if __name__ == "__main__":
    main()
