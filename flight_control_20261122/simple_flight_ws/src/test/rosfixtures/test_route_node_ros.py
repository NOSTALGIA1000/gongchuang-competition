"""Isolated ROS message/service test, NOT PX4 SITL or landing-detector proof.

Hard require localhost domain 77 before importing ROS. No serial port, MAVLink
socket, parameter writer, or real FCU process is created by this test.
"""
import argparse
import json
from copy import deepcopy
from dataclasses import replace
import math
import os
from pathlib import Path
import sys
import time

if os.environ.get("ROS_DOMAIN_ID") != "77" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("Refusing test: require ROS_DOMAIN_ID=77 and ROS_LOCALHOST_ONLY=1")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src/simple_flight_mission"))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from mavros_msgs.msg import State, RCIn, ExtendedState, EstimatorStatus, SysStatus, PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from test_hover_node_ros import VirtualVehicle, pump, until
from simple_flight_mission.route_config import RouteConfig, RoutePoint, load_route_config
from simple_flight_mission.route_node import RouteNode, parse_options


class RoutePeer(VirtualVehicle):
    def __init__(self):
        self.xyz, self.raw, self.flown, self.touch_at = [2., -3., 4.2], None, False, None
        self.raw_targets = []
        self.vision_yaw_offset = 0.
        self.case_name = None
        self.last_progress_key = None
        super().__init__()
        self.destroy_subscription(self.sub)
        self.destroy_subscription(self.status_sub)
        self.transport_node = Node("setpoint_raw", namespace="/mavros")
        self.sub = self.transport_node.create_subscription(PositionTarget, "/mavros/setpoint_raw/local", self.on_raw, 10)
        self.status_sub = self.create_subscription(String, "/simple_flight/route/status", self.on_status, 10)

    def parameters(self, request, response):
        response = super().parameters(request, response)
        landing = dict(LNDMC_Z_VEL_MAX=.25, COM_DISARM_LAND=2., MPC_LAND_SPEED=.7,
                       MPC_LAND_CRWL=.3, MPC_LAND_ALT3=1., MPC_Z_VEL_MAX_DN=1.5)
        for name, value in zip(request.names, response.values):
            if name in landing:
                value.type, value.double_value = 3, landing[name]
            elif name == "EKF2_GPS_CTRL":
                value.type, value.integer_value = 2, 0
        return response

    def on_raw(self, message):
        self.raw = message
        self.raw_targets.append((time.monotonic(), message))

    def on_status(self, message):
        super().on_status(message)
        status = self.statuses[-1]
        key = (status["phase"], status["point_number"])
        if self.case_name and key != self.last_progress_key:
            print(json.dumps(dict(case=self.case_name, phase=key[0], point=key[1],
                                  elapsed_s=status["mission_elapsed_s"],
                                  return_reason=status["return_reason"])), flush=True)
            self.last_progress_key = key

    def publish(self):
        now, previous = time.monotonic(), tuple(self.xyz)
        if self.mode == "OFFBOARD" and self.armed and self.raw is not None:
            for i, target in enumerate((self.raw.position.x, self.raw.position.y, self.raw.position.z)):
                if i == 2 and self.raw.type_mask & PositionTarget.IGNORE_PZ:
                    assert not self.raw.type_mask & PositionTarget.IGNORE_VZ
                    self.xyz[i] += self.raw.velocity.z*.05
                else:
                    self.xyz[i] += max(-.012, min(.012, target-self.xyz[i]))
            self.xyz[2] = max(4.2, self.xyz[2])
            self.flown |= self.xyz[2] > 4.4
            if self.flown and self.xyz[2] <= 4.200001:
                self.touch_at = now if self.touch_at is None else self.touch_at
                if now-self.touch_at >= 2:
                    self.armed = False  # Peer telemetry only; NOT PX4's detector.
        stamp = self.get_clock().now().to_msg()
        state = State()
        state.header.stamp = stamp
        state.connected, state.armed, state.manual_input, state.mode = True, self.armed, self.mode != "OFFBOARD", self.mode
        self.pub["state"].publish(state)
        rc = RCIn()
        rc.header.stamp, rc.channels = stamp, [1514, 1514, self.throttle, 1514, 2003, 1514, self.swa]
        self.pub["rc"].publish(rc)
        landed = ExtendedState()
        landed.landed_state = 1 if self.xyz[2] <= 4.200001 else 2
        self.pub["landed"].publish(landed)
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "map"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.xyz
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(.35), math.cos(.35)
        if self.pose_enabled:
            self.pub["pose"].publish(pose)
        velocity = TwistStamped()
        velocity.header = pose.header
        velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z = [(a-b)/.05 for a,b in zip(self.xyz, previous)]
        self.pub["velocity"].publish(velocity)
        lio, vision = Odometry(), PoseWithCovarianceStamped()
        lio.header, lio.pose.pose = pose.header, pose.pose
        vision.header, vision.pose.pose = pose.header, deepcopy(pose.pose)
        vision.pose.pose.orientation.z = math.sin((.7+self.vision_yaw_offset)/2)
        vision.pose.pose.orientation.w = math.cos((.7+self.vision_yaw_offset)/2)
        self.pub["lio"].publish(lio)
        self.pub["vision"].publish(vision)
        est = EstimatorStatus()
        est.attitude_status_flag = est.velocity_horiz_status_flag = est.velocity_vert_status_flag = True
        est.pos_horiz_rel_status_flag = est.pos_vert_abs_status_flag = est.pred_pos_horiz_rel_status_flag = True
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


def run_case(kind):
    executor, peer = SingleThreadedExecutor(), RoutePeer()
    peer.case_name = kind
    config = RouteConfig(height_m=.3, origin_hold_s=.2, return_hold_s=.2,
                         arrival_stable_s=.2, points=(RoutePoint(.3, 0., .2),))
    if kind == "default_complete":
        config = RouteConfig()
    elif kind == "multi_complete":
        config = RouteConfig(height_m=.3, origin_hold_s=.2, return_hold_s=.2,
                             arrival_stable_s=.2,
                             points=(RoutePoint(.3, 0., .2), RoutePoint(.3, .3, .2)))
    elif kind in ("route1_complete", "route1_timeout"):
        config = load_route_config(Path(__file__).resolve().parents[1] /
                                   "src/simple_flight_mission/config/route_1.yaml")
        assert config.mission_timeout_s == 300.0
    node = RouteNode(parse_options([] if kind == "check" else ["--execute"]), config)
    timeout_test_limit = None
    executor.add_node(peer)
    executor.add_node(peer.transport_node)
    executor.add_node(node.node)
    try:
        if kind == "check":
            pump(executor, 3)
            assert not peer.raw_targets and not peer.mode_calls and not peer.arm_calls
            assert node.target_pub is node.arm_client is node.mode_client is None
        elif kind == "heading_disagreement":
            until(executor, lambda: node.last_status and node.last_status["ready_for_switch"], 12, "READY")
            peer.swa = 2003
            until(executor, lambda: node.last_status["phase"] == "TAKEOFF", 5, "TAKEOFF")
            peer.vision_yaw_offset = math.radians(-20)
            until(executor, lambda: node.last_status["phase"] == "EXITED", 4, "heading mismatch exit")
            assert "heading_reference_disagreement" in node.last_status["exit_reason"]
            assert peer.mode == "POSCTL" and peer.arm_calls == [True]
            assert not node.controller.mission_completed
            pump(executor, .4)
            count = len(peer.raw_targets)
            pump(executor, .6)
            assert len(peer.raw_targets) == count
        else:
            until(executor, lambda: node.last_status and node.last_status["ready_for_switch"], 12, "READY")
            peer.swa = 2003
            if kind == "route1_timeout":
                # Shorten ONLY this isolated test after entering the 2->3 leg.
                # Production route_1.yaml remains at 300 seconds. This exercises
                # the real timer/return branch without a five-minute test wait.
                until(executor, lambda: node.last_status["phase"] == "TRANSIT"
                      and node.last_status["point_number"] == 3
                      and node.controller.progress >= .5, 90, "partway from point 2 to point 3")
                timeout_test_limit = node.controller.mission_elapsed(time.monotonic()) + .4
                node.controller.route_config = replace(config, mission_timeout_s=timeout_test_limit)
            until(executor, lambda: node.last_status["phase"] == "DESCEND", 240, "DESCEND")
            assert peer.arm_calls == [True]
            local_descent_at = time.monotonic()
            first_mask = None if peer.raw is None else peer.raw.type_mask
            # The controller's local state changes before a different ROS
            # subscription callback necessarily consumes its newest target.
            until(executor, lambda: peer.raw is not None and peer.raw.type_mask == 2524
                  and peer.raw.velocity.z < 0, .8, "received descent target")
            print(json.dumps(dict(case=kind, local_phase="DESCEND", first_received_mask=first_mask,
                                  descent_received_mask=peer.raw.type_mask,
                                  descent_received_vz=peer.raw.velocity.z,
                                  receive_wait_s=time.monotonic()-local_descent_at)), flush=True)
            assert peer.raw is not None and peer.raw.type_mask == 2524 and peer.raw.velocity.z < 0
            assert peer.raw.velocity.z == -.3  # FCU crawl rate; detector remains .25.
            if kind == "swa_exit":
                peer.swa = 1024
                until(executor, lambda: node.last_status["phase"] == "EXITED", 4, "SWA exit")
                assert not node.controller.mission_completed and peer.mode == "POSCTL"
                assert node.last_status["exit_reason"] == "swa_low"
            elif kind == "throttle_exit":
                peer.throttle = 1514
                pump(executor, .35)
                peer.throttle = 1024
                until(executor, lambda: node.last_status["phase"] == "EXITED", 4, "throttle exit")
                assert not node.controller.mission_completed and peer.mode == "POSCTL"
                assert node.last_status["exit_reason"] == "throttle_raise_then_low"
            else:
                until(executor, lambda: node.last_status["phase"] == "EXITED", 12, "landing feedback and disarm")
                assert node.controller.mission_completed and not peer.armed and peer.mode == "POSCTL"
            pump(executor, .4)
            count = len(peer.raw_targets)
            pump(executor, .6)
            assert len(peer.raw_targets) == count
            visited = []
            for status in peer.statuses:
                if status["phase"] == "POINT_HOLD" and (not visited or visited[-1] != status["point_number"]):
                    visited.append(status["point_number"])
            expected = {"multi_complete": [1, 2], "route1_complete": [1, 2, 3, 4, 5],
                        "route1_timeout": [1, 2]}.get(kind, [1])
            assert visited == expected, (kind, visited, expected)
            if kind in ("route1_complete", "route1_timeout"):
                returning = []
                for status in peer.statuses:
                    if status["phase"] == "RETURN" and (not returning or returning[-1] != status["point_number"]):
                        returning.append(status["point_number"])
                assert returning == [2, 1, None], returning
                expected_reason = "mission_timeout" if kind == "route1_timeout" else "route_complete"
                assert node.last_status["return_reason"] == expected_reason
                assert node.last_status["mission_completed"] and node.last_status["landed_state"] == 1
                assert all(abs(math.remainder(message.yaw-.7, 2*math.pi)) < 1e-5
                           for _, message in peer.raw_targets)
                assert math.dist(peer.xyz[:2], (2., -3.)) <= config.arrival_xy_m
                origin = node.last_status["locked_origin"]
                assert math.dist(origin[:3], (2., -3., 4.2)) < 1e-6
                assert abs(math.remainder(origin[3]-.7, 2*math.pi)) < 1e-6
                # Independent geometry/dwell checks, in addition to visit order.
                coordinates = ((2.2, 0.), (2.2, 1.4), (0., 1.4), (2.2, 1.4), (2.2, 2.7))
                for i, status in enumerate(peer.statuses):
                    if status["phase"] != "POINT_HOLD" or (i and peer.statuses[i-1]["phase"] == "POINT_HOLD"):
                        continue
                    number = status["point_number"]
                    forward, left = coordinates[number-1]
                    expected_goal = (2.+math.cos(.7)*forward-math.sin(.7)*left,
                                     -3.+math.sin(.7)*forward+math.cos(.7)*left, 5.2)
                    assert math.dist(status["final_leg_goal"][:3], expected_goal) < 1e-6
                    assert math.dist(status["actual_pose"][:2], expected_goal[:2]) <= config.arrival_xy_m
                    end = next(s for s in peer.statuses[i+1:] if s["phase"] != "POINT_HOLD")
                    dwell = end["updated_monotonic"]-status["updated_monotonic"]
                    assert dwell >= (3. if number in (3, 5) else .5)-.08, (number, dwell)
        return dict(case=kind, passed=True, raw_target_count=len(peer.raw_targets),
                    arm_requests=peer.arm_calls, modes=peer.mode_calls,
                    point_count=len(config.points), height_m=config.height_m,
                    fcu_landing_parameters=node.controller.landing_parameters,
                    configured_timeout_s=config.mission_timeout_s,
                    isolated_timeout_override_s=timeout_test_limit,
                    mission_elapsed_s=node.controller.mission_elapsed(time.monotonic()),
                    return_reason=node.controller.return_reason,
                    phase_transitions=[dict(phase=s["phase"], point=s["point_number"],
                                            elapsed_s=s["mission_elapsed_s"])
                                       for i, s in enumerate(peer.statuses)
                                       if i == 0 or (s["phase"], s["point_number"]) !=
                                       (peer.statuses[i-1]["phase"], peer.statuses[i-1]["point_number"])])
    finally:
        node.close()
        peer.transport_node.destroy_node()
        peer.destroy_node()
        executor.shutdown()


def main():
    cases = ("check", "complete", "swa_exit", "throttle_exit", "default_complete",
             "multi_complete", "heading_disagreement", "route1_complete", "route1_timeout")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", nargs="+", choices=cases, default=cases[:7])
    parser.add_argument("--output", type=Path, help="Save isolated-test results as JSON")
    options = parser.parse_args()
    report = dict(passed=False, message_peer_only=True, hardware_test=False, isolated_domain=77,
                  localhost_only=True, results=[])
    rclpy.init(args=[])
    try:
        probe = rclpy.create_node("route_isolation_probe")
        try:
            until_at = time.monotonic()+2
            while time.monotonic() < until_at:
                rclpy.spin_once(probe, timeout_sec=.05)
            unexpected = [name for name in probe.get_node_names() if name != probe.get_name()]
            if unexpected:
                raise RuntimeError("Refusing occupied isolated domain: "+str(unexpected))
        finally:
            probe.destroy_node()
        print(json.dumps(dict(isolated_domain=77, localhost_only=True,
                              message_peer_only=True, hardware_test=False)), flush=True)
        for kind in options.cases:
            report["results"].append(run_case(kind))
        report["passed"] = True
        print(json.dumps(report, indent=2))
    except Exception as error:
        report["error"] = repr(error)
        report["failed_case"] = kind if "kind" in locals() else "isolation_probe"
        raise
    finally:
        if options.output:
            options.output.parent.mkdir(parents=True, exist_ok=True)
            options.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
