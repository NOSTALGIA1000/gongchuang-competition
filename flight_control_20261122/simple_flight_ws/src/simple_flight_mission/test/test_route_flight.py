"""Contract tests for configurable relative routes, not a flight dynamics model."""
from dataclasses import replace
import math

import pytest

from test.test_hover_controller import Rig
from simple_flight_mission.hover_controller import Observation


def api():
    from simple_flight_mission.route_config import RouteConfig, RoutePoint, load_route_config
    from simple_flight_mission.route_controller import RouteController
    return RouteConfig, RoutePoint, load_route_config, RouteController


class RouteRig(Rig):
    transport_available = True
    heading_available = True
    heading_offset = 0.
    heading_wire_age = 0.

    def tick(self, publish=True, **overrides):
        self.controller.note_target_transport(self.t+.05, self.transport_available)
        if self.heading_available:
            pose = overrides.get("pose")
            yaw = self.pose[3] if pose is None else pose.value[3]
            self.controller.external_heading = Observation(yaw+self.heading_offset, self.t+.05, self.heading_wire_age)
        return super().tick(publish=publish, **overrides)


def route_rig(config=None, mode="execute"):
    Config, _, _, Controller = api()
    rig = RouteRig(mode)
    rig.controller = Controller(mode, config or Config())
    rig.params.update(EKF2_GPS_CTRL=0, COM_DISARM_LAND=2.0, LNDMC_Z_VEL_MAX=0.25,
                      MPC_LAND_SPEED=.7, MPC_LAND_CRWL=.3, MPC_LAND_ALT3=1., MPC_Z_VEL_MAX_DN=1.5)
    return rig


def follow(rig, ticks=1):
    outputs = []
    for _ in range(ticks):
        out = rig.tick()
        outputs.append(out)
        if out.target is not None:
            x, y, z, yaw = out.target
            if getattr(out, "velocity_z", None) is not None:
                z = max(4.2, rig.pose[2] + out.velocity_z * .05)
            rig.pose = (x, y, z, yaw, 0.0)
            if z > 4.4:
                rig.landed = 2
    return outputs


def until_phase(rig, phase, ticks=900):
    for _ in range(ticks):
        out = follow(rig)[0]
        if out.phase == phase:
            return out
        assert out.phase not in ("HANDOVER", "EXITED"), out
    raise AssertionError((phase, rig.last))


def test_yaml_multiple_points_are_absolute_offsets_not_accumulated(tmp_path):
    _, _, load, _ = api()
    path = tmp_path / "my_route.yaml"
    path.write_text("horizontal_mps: 0.2\npoints:\n"
                    "  - {forward_m: 2, left_m: 0, hold_s: 3}\n"
                    "  - {forward_m: 2, left_m: 1, hold_s: 4}\n", encoding="utf-8")
    config = load(path)
    targets = config.world_points((10., 20., 4.2, math.pi/2))
    assert targets[0] == pytest.approx((10., 22., 5.2, math.pi/2))
    assert targets[1] == pytest.approx((9., 22., 5.2, math.pi/2))
    assert config.points[1].hold_s == 4


def test_path1_configuration_has_named_stages_and_explicit_return_route():
    _, _, load, _ = api()
    path = __import__("pathlib").Path(__file__).resolve().parents[1] / "config/route_1.yaml"
    config = load(path)
    assert [point.stage for point in config.points] == [
        "obstacle_1", "obstacle_2", "task_zone_1", "obstacle_3", "task_zone_2"
    ]
    assert [(point.forward_m, point.left_m) for point in config.points] == [
        (2.2, 0.0), (2.2, 1.4), (0.0, 1.4), (2.2, 1.4), (2.2, 2.7)
    ]
    assert [point.hold_s for point in config.points] == [0.1, 0.1, 1.0, 0.1, 1.0]
    assert config.return_indices == (3, 1, 0)


@pytest.mark.parametrize("text", [
    "points: []", "points: null", "points: [{forward_m: .nan}]",
    "points: [{forward_m: 1, hold_s: -1}]", "horizontal_mps: true",
    "horizontal_mps: 0", "descent_mps: .inf", "points: [{forward_m: 1, left: 2}]",
    "height_m: 9", "arrival_stable_s: 0", "unknown: 1",
    "points: [{forward_m: true}]", "horizontal_mps: 0.2\nhorizontal_mps: 0.3",
])
def test_bad_config_is_rejected_before_ros_or_hardware(text, tmp_path):
    _, _, load, _ = api()
    path = tmp_path / "bad.yaml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises((ValueError, TypeError)):
        load(path)


def test_default_check_and_mode_test_have_no_arm_or_climb():
    rig = route_rig(mode="check")
    for _ in range(140):
        out = rig.tick()
        assert out.target is None and out.request_mode is None and not out.arm
    rig = route_rig(mode="mode-test")
    rig.active()
    for _ in range(60):
        out = rig.tick()
        assert out.phase == "MODE_TEST" and out.target[2] == 4.2 and not out.arm


def test_slow_descent_incompatible_with_landing_detector_refuses_start():
    rig = route_rig()
    rig.params.update(MPC_LAND_SPEED=.15, MPC_LAND_CRWL=.15)
    for _ in range(140):
        out = rig.tick()
        assert out.target is None and not out.arm
    assert "landing_descent_incompatible" in out.reasons


@pytest.mark.parametrize("value", [None, -1, 0, float("nan")])
def test_missing_or_disabled_landed_auto_disarm_refuses_start(value):
    rig = route_rig()
    rig.params["COM_DISARM_LAND"] = value
    out = rig.tick()
    assert "landing_auto_disarm_invalid" in out.reasons


def test_complete_default_route_waits_for_actual_landing_and_disarm():
    rig = route_rig()
    rig.active()
    outputs = []
    phases = {}
    for _ in range(1000):
        out = follow(rig)[0]
        outputs.append(out)
        phases.setdefault(out.phase, rig.t)
        if out.phase == "DESCEND" and rig.pose[2] <= 4.200001:
            break
    assert {"ORIGIN_HOLD", "TRANSIT", "POINT_HOLD", "RETURN", "RETURN_HOLD", "DESCEND"} <= phases.keys()
    assert phases["TRANSIT"] - phases["ORIGIN_HOLD"] >= 3
    assert phases["RETURN"] - phases["POINT_HOLD"] >= 3
    assert phases["DESCEND"] - phases["RETURN_HOLD"] >= 2
    forward = [(out.target[0]-2)*math.cos(.7) + (out.target[1]+3)*math.sin(.7)
               for out in outputs if out.target is not None]
    assert max(forward) == pytest.approx(1.)
    assert rig.pose[:2] == pytest.approx((2., -3.))
    for out in outputs:
        assert not out.arm
        if out.target is not None:
            assert out.target[3] == .7
    # Height alone cannot declare landing or stop sending controlled descent.
    for out in follow(rig, 10):
        assert out.phase == "DESCEND" and out.velocity_z == -.3
    rig.landed = 1
    assert rig.tick().phase == "LAND_WAIT"
    for _ in range(10):
        out = rig.tick()
        assert out.phase == "LAND_WAIT" and out.velocity_z == -.3
    rig.armed = False
    out = rig.tick()
    assert out.phase == "HANDOVER" and out.request_mode == "POSCTL"
    assert out.exit_reason == "mission_complete"
    rig.mode = "POSCTL"
    out = rig.tick()
    assert out.phase == "EXITED" and out.target is None
    assert rig.controller.mission_completed
    rig.rc[6] = 1024
    rig.tick()
    rig.rc[6] = 2003
    assert rig.tick().target is None


def test_transit_targets_are_rate_limited_and_do_not_run_far_ahead_of_aircraft():
    rig = route_rig()
    rig.active()
    until_phase(rig, "TRANSIT")
    previous = rig.last.target
    for _ in range(60):
        out = rig.tick()  # Aircraft intentionally remains at segment start.
        assert math.dist(previous[:2], out.target[:2]) <= .010001
        assert math.dist(rig.pose[:2], out.target[:2]) <= .250001
        previous = out.target
    assert out.phase == "TRANSIT"


def test_arrival_uses_actual_pose_velocity_and_resets_hold_after_drift():
    rig = route_rig()
    rig.active()
    until_phase(rig, "POINT_HOLD")
    follow(rig, 35)
    rig.pose = (rig.pose[0] + .2, rig.pose[1], rig.pose[2], .7, 0.)
    rig.tick()
    follow(rig, 60)
    assert rig.last.phase == "POINT_HOLD"
    until_phase(rig, "RETURN", ticks=80)


@pytest.mark.parametrize("phase", ["TRANSIT", "POINT_HOLD", "RETURN", "DESCEND", "LAND_WAIT"])
@pytest.mark.parametrize("exit_kind", ["swa", "throttle", "stop"])
def test_every_route_and_landing_phase_keeps_existing_takeover_and_never_reenters(phase, exit_kind):
    rig = route_rig()
    rig.active()
    until_phase(rig, "DESCEND" if phase == "LAND_WAIT" else phase)
    if phase == "LAND_WAIT":
        for _ in range(180):
            follow(rig)
            if rig.pose[2] <= 4.200001:
                break
        rig.landed = 1
        assert rig.tick().phase == "LAND_WAIT"
    if exit_kind == "swa":
        rig.rc[6] = 1024
    elif exit_kind == "throttle":
        rig.rc[2] = 1514
        for _ in range(6):
            rig.tick()
        rig.rc[2] = 1024
    else:
        rig.controller.request_exit()
    for _ in range(4):
        out = rig.tick()
    assert out.phase == "HANDOVER" and out.request_mode in (None, "POSCTL")
    assert getattr(out, "velocity_z", None) is None  # Descent immediately cancelled.
    assert out.target[:3] == pytest.approx(rig.pose[:3])
    rig.mode = "POSCTL"
    assert rig.tick().target is None
    rig.rc[6] = 2003
    assert rig.tick().target is None


def test_multi_point_configuration_visits_in_order_then_returns():
    Config, Point, _, _ = api()
    rig = route_rig(Config(points=(Point(1., 0., .2), Point(1., 1., .2))))
    rig.active()
    visited = []
    for _ in range(1000):
        out = follow(rig)[0]
        if out.phase == "POINT_HOLD" and (not visited or visited[-1][0] != out.point_index):
            visited.append((out.point_index, out.target))
        if out.phase == "RETURN_HOLD":
            break
        assert out.phase not in ("HANDOVER", "EXITED"), out
    assert [v[0] for v in visited] == [0, 1]
    assert visited[1][1][:2] == pytest.approx((2.+math.cos(.7)-math.sin(.7), -3.+math.sin(.7)+math.cos(.7)))
    assert out.target[:2] == pytest.approx((2., -3.))


def test_route_corridor_is_enforced_and_longer_config_is_not_limited_to_old_hover_radius():
    Config, Point, _, _ = api()
    rig = route_rig(Config(points=(Point(2., 0., .2),)))
    rig.active()
    until_phase(rig, "POINT_HOLD")
    assert math.dist(rig.pose[:2], (2., -3.)) == pytest.approx(2.)
    rig.pose = (rig.pose[0]-math.sin(.7)*.5, rig.pose[1]+math.cos(.7)*.5, rig.pose[2], .7, 0.)
    out = rig.tick()
    assert out.phase == "HANDOVER" and "route_corridor" in out.exit_reason


def test_stale_position_during_descent_stops_targets_without_using_old_pose():
    rig = route_rig()
    rig.active()
    until_phase(rig, "DESCEND")
    out = rig.tick(pose=Observation(rig.pose, rig.t-1., 0.))
    assert out.phase == "HANDOVER" and out.target is None


def test_explicit_return_indices_are_used_instead_of_direct_home_return():
    Config, Point, _, _ = api()
    config = Config(points=(Point(1., hold_s=.05), Point(2., hold_s=.05), Point(3., hold_s=.05)),
                    return_indices=(1, 0), origin_hold_s=.05, return_hold_s=.05)
    rig = route_rig(config)
    rig.active()
    for _ in range(1000):
        out = follow(rig)[0]
        if out.phase == "RETURN":
            assert rig.controller.goal[:2] == pytest.approx(config.world_points(rig.controller.origin)[1][:2])
            return
        assert out.phase not in ("HANDOVER", "EXITED"), out
    raise AssertionError("explicit return route did not start")


def path1_config():
    return api()[2](__import__("pathlib").Path(__file__).resolve().parents[1] / "config/route_1.yaml")


def path2_config():
    return api()[2](__import__("pathlib").Path(__file__).resolve().parents[1] / "config/route_2.yaml")


def path3_config():
    return api()[2](__import__("pathlib").Path(__file__).resolve().parents[1] / "config/route_3.yaml")


def test_path2_configuration_has_expected_stages_and_return_route():
    config = path2_config()
    assert [(point.forward_m, point.left_m) for point in config.points] == [
        (2.2, 0.0), (2.2, 1.4), (0.0, 1.4), (1.2, 1.4), (1.2, 2.7)
    ]
    assert [point.stage for point in config.points] == [
        "obstacle_1", "obstacle_2", "task_zone_1", "obstacle_3", "task_zone_2"
    ]
    assert [point.hold_s for point in config.points] == [0.1, 0.1, 3.0, 0.1, 3.0]
    assert config.return_indices == (3, 1, 0)


def test_path2_visits_all_points_then_returns_via_four_two_one_home():
    rig = route_rig(path2_config())
    rig.active()
    visits = []
    previous = None
    for _ in range(3000):
        out = follow(rig)[0]
        key = (out.phase, getattr(out, "point_index", None))
        if key != previous and out.phase in ("POINT_HOLD", "RETURN_HOLD"):
            visits.append(key)
        previous = key
        assert out.phase not in ("HANDOVER", "EXITED"), out
        if out.phase == "DESCEND":
            break
    assert visits == [("POINT_HOLD", i) for i in range(5)] + [("RETURN_HOLD", None)]
    assert out.phase == "DESCEND"


def test_path2_return_passes_four_and_two_without_holding():
    rig = route_rig(path2_config())
    rig.active()
    out = until_phase(rig, "RETURN", ticks=3000)
    assert out.point_index == 3
    seen = []
    for _ in range(500):
        out = follow(rig)[0]
        if out.phase == "RETURN" and out.point_index not in seen:
            seen.append(out.point_index)
        assert out.phase != "RETURN_HOLD" or out.point_index is None
        if out.phase == "DESCEND":
            break
    assert seen[:3] == [3, 1, 0]


def test_path3_configuration_has_expected_stages_and_return_route():
    config = path3_config()
    assert [(point.forward_m, point.left_m) for point in config.points] == [
        (1.7, 0.0), (1.7, 1.4), (0.0, 1.4), (2.2, 1.4), (2.2, 2.7)
    ]
    assert [point.stage for point in config.points] == [
        "obstacle_1", "obstacle_2", "task_zone_1", "obstacle_3", "task_zone_2"
    ]
    assert [point.hold_s for point in config.points] == [0.1, 0.1, 3.0, 0.1, 3.0]
    assert config.return_indices == (3, 1, 0)


def test_path3_visits_all_points_then_returns_via_four_two_one_home():
    rig = route_rig(path3_config())
    rig.active()
    visits = []
    previous = None
    for _ in range(3000):
        out = follow(rig)[0]
        key = (out.phase, getattr(out, "point_index", None))
        if key != previous and out.phase in ("POINT_HOLD", "RETURN_HOLD"):
            visits.append(key)
        previous = key
        assert out.phase not in ("HANDOVER", "EXITED"), out
        if out.phase == "DESCEND":
            break
    assert visits == [("POINT_HOLD", i) for i in range(5)] + [("RETURN_HOLD", None)]
    assert out.phase == "DESCEND"


def test_path3_return_passes_four_two_one_without_holding():
    rig = route_rig(path3_config())
    rig.active()
    out = until_phase(rig, "RETURN", ticks=3000)
    assert out.point_index == 3
    seen = []
    for _ in range(500):
        out = follow(rig)[0]
        if out.phase == "RETURN" and out.point_index not in seen:
            seen.append(out.point_index)
        assert out.phase != "RETURN_HOLD" or out.point_index is None
        if out.phase == "DESCEND":
            break
    assert seen[:3] == [3, 1, 0]


def test_path1_visits_all_points_then_returns_via_two_one_home():
    rig = route_rig(path1_config())
    rig.active()
    visits = []
    previous = None
    for _ in range(2600):
        out = follow(rig)[0]
        key = (out.phase, getattr(out, "point_index", None))
        if key != previous and out.phase in ("POINT_HOLD", "RETURN_HOLD"):
            visits.append(key)
        previous = key
        assert out.phase not in ("HANDOVER", "EXITED"), out
        if out.phase == "DESCEND":
            break
    assert visits == [("POINT_HOLD", i) for i in range(5)] + [("RETURN_HOLD", None)]
    assert out.phase == "DESCEND"
    assert rig.controller.return_reason == "route_complete"


def test_return_advances_through_intermediate_waypoint_while_still_moving():
    rig = route_rig(path1_config())
    rig.active()
    out = until_phase(rig, "RETURN", ticks=2600)
    assert out.point_index == 3

    for _ in range(100):
        start, goal = rig.controller.segment_start, rig.controller.goal
        length = math.dist(start[:2], goal[:2])
        velocity = (tuple(.2*(goal[i]-start[i])/length for i in range(2)) + (0.,)
                    if length>1e-9 else (.2,0.,0.))
        assert math.hypot(*velocity[:2]) > rig.controller.route_config.arrival_speed_mps
        out = rig.tick(velocity=Observation(velocity, rig.t+.05, 0.))
        if out.target is not None:
            rig.pose = (*out.target, 0.0)
        if out.point_index == 0:
            break
        assert out.phase == "RETURN"
        assert out.phase != "RETURN_HOLD"

    assert out.phase == "RETURN"
    assert out.point_index == 0


@pytest.mark.parametrize("leg", [0, 1, 2, 3, 4])
def test_timeout_reverses_only_traversed_legs_without_target_jump(leg):
    rig = route_rig(path1_config())
    rig.active()
    for _ in range(2200):
        out = follow(rig)[0]
        if out.phase == "TRANSIT" and out.point_index == leg and rig.controller.progress > .3:
            break
    else:
        raise AssertionError("leg not reached")
    previous = out.target
    rig.controller.mission_started_at = rig.t-rig.controller.route_config.mission_timeout_s
    out = follow(rig)[0]
    assert out.phase == "RETURN"
    assert out.target == pytest.approx(previous)
    assert rig.controller.return_reason == "mission_timeout"
    visits = []
    previous_key = None
    for _ in range(1600):
        out = follow(rig)[0]
        if out.phase == "RETURN":
            assert math.dist(previous[:2], out.target[:2]) <= .015001
        if out.phase == "RETURN_HOLD" and previous_key != (out.phase, out.point_index):
            visits.append(out.point_index)
        previous_key = (out.phase, out.point_index)
        previous = out.target
        assert out.phase not in ("HANDOVER", "EXITED"), out
        if out.phase == "DESCEND":
            break
    assert visits == [None]
    assert out.phase == "DESCEND"


def test_mission_clock_starts_at_switch_trigger_and_freezes_after_landed_disarmed():
    rig = route_rig()
    rig.ready()
    assert rig.controller.mission_elapsed(rig.t) == 0
    rig.rc[6] = 2003
    assert rig.tick().phase == "SWITCHING"
    started = rig.t
    assert rig.controller.mission_started_at == started
    rig.mode = "OFFBOARD"
    assert rig.tick().arm
    rig.armed = True
    until_phase(rig, "DESCEND")
    for _ in range(180):
        follow(rig)
        if rig.pose[2] <= 4.200001:
            break
    rig.landed, rig.armed = 1, False
    assert rig.tick().exit_reason == "mission_complete"
    elapsed = rig.controller.mission_elapsed(rig.t)
    assert elapsed == pytest.approx(rig.t-started)
    assert rig.controller.mission_elapsed(rig.t+10) == elapsed


def test_unstable_hold_has_no_separate_countdown():
    Config, Point, _, _ = api()
    config = Config(points=(Point(1.),), origin_hold_s=.1,
                    transit_timeout_extra_s=.1, mission_timeout_s=300.)
    rig = route_rig(config)
    rig.active()
    until_phase(rig, "POINT_HOLD")
    for _ in range(400):
        out = rig.tick(velocity=Observation((.11, 0., 0.), rig.t+.05, 0.))
        assert out.phase == "POINT_HOLD"


def test_landing_timeout_and_spurious_landed_above_ground_do_not_force_disarm():
    rig = route_rig()
    rig.active()
    until_phase(rig, "DESCEND")
    rig.landed = 1
    out = rig.tick()
    assert out.phase == "HANDOVER" and out.exit_reason == "landed_above_origin"
    rig = route_rig()
    rig.active()
    until_phase(rig, "DESCEND")
    for _ in range(600):
        out = rig.tick()
        assert not out.arm
        if out.phase == "HANDOVER":
            break
    assert out.exit_reason == "landing_timeout"


def test_long_route_does_not_expand_the_allowed_takeoff_drift():
    Config, Point, _, _ = api()
    rig = route_rig(Config(points=(Point(3.),)))
    rig.active()
    # Slow shift avoids the independent 0.5 m pose-jump guard.
    for _ in range(5):
        rig.pose = (rig.pose[0]+.1, *rig.pose[1:])
        out = rig.tick()
    assert out.phase == "HANDOVER" and "route_corridor" in out.exit_reason


def test_new_boot_generation_during_transit_is_never_silently_accepted():
    rig = route_rig()
    rig.active()
    until_phase(rig, "TRANSIT")
    rig.boot_run = "a-restarted-localizer"
    out = rig.tick()
    assert out.phase == "HANDOVER" and out.target is None
    assert "localization_generation_changed" in out.exit_reason


def test_raw_control_source_stall_during_route_aborts_before_another_moving_target():
    rig = route_rig()
    rig.active()
    until_phase(rig, "TRANSIT")
    for _ in range(5):
        out = rig.tick(publish=False)
    assert out.phase == "HANDOVER" and out.exit_reason == "target_stream_stalled"


def test_started_high_never_arms_or_prestreams():
    rig = route_rig()
    rig.rc[6] = 2003
    for _ in range(200):
        out = rig.tick()
        assert out.target is None and not out.arm


def test_zero_length_waypoint_and_return_do_not_divide_by_zero():
    Config, Point, _, _ = api()
    rig = route_rig(Config(points=(Point(0., 0., .2),)))
    rig.active()
    until_phase(rig, "DESCEND")
    assert rig.last.target[:2] == pytest.approx((2., -3.))


def test_landing_outside_arrival_tolerance_is_not_reported_as_mission_success():
    rig = route_rig()
    rig.active()
    until_phase(rig, "DESCEND")
    for _ in range(180):
        follow(rig)
        if rig.pose[2] <= 4.200001:
            break
    rig.pose = (2.2, -3., 4.2, .7, 0.)
    rig.landed, rig.armed = 1, False
    out = rig.tick()
    assert out.phase == "HANDOVER"
    assert out.exit_reason == "landed_outside_target"
    assert not rig.controller.mission_completed


def fcu_rate_rig(config=None):
    rig = route_rig(config)
    rig.params.update(LNDMC_Z_VEL_MAX=.25, MPC_LAND_SPEED=.7, MPC_LAND_CRWL=.3,
                      MPC_LAND_ALT3=1., MPC_Z_VEL_MAX_DN=1.5)
    return rig


def test_one_meter_landing_reads_fcu_crawl_rate_without_changing_detector_parameter():
    rig = fcu_rate_rig()
    original = dict(rig.params)
    rig.active()
    out = until_phase(rig, "DESCEND")
    assert out.velocity_z == -.3
    assert rig.params == original and rig.params["LNDMC_Z_VEL_MAX"] == .25


def test_fcu_landing_rate_changes_to_crawl_at_fcu_height_and_never_speeds_up_again():
    Config, _, _, _ = api()
    rig = fcu_rate_rig(Config(height_m=1.2))
    rig.params["MPC_LAND_ALT3"] = .5
    rig.active()
    assert until_phase(rig, "DESCEND").velocity_z == -.7
    for _ in range(100):
        out = follow(rig)[0]
        if out.velocity_z == -.3:
            break
    assert out.velocity_z == -.3
    # A small position fluctuation above the threshold cannot accelerate descent.
    rig.pose = (*rig.pose[:2], 4.75, *rig.pose[3:])
    assert rig.tick().velocity_z == -.3


def test_fcu_rate_descent_has_a_specific_guard_without_relaxing_horizontal_or_ascent_limits():
    Config, _, _, _ = api()
    rig = fcu_rate_rig(Config(height_m=1.2))
    rig.params["MPC_LAND_ALT3"] = .5
    rig.active()
    until_phase(rig, "DESCEND")
    out = rig.tick(velocity=Observation((0., 0., -.7), rig.t+.05, 0.))
    assert out.phase == "DESCEND"
    out = rig.tick(velocity=Observation((.9, 0., -.7), rig.t+.05, 0.))
    assert out.phase == "HANDOVER" and "velocity_limit" in out.exit_reason
    rig = fcu_rate_rig(Config(height_m=1.2))
    rig.params["MPC_LAND_ALT3"] = .5
    rig.active()
    until_phase(rig, "DESCEND")
    out = rig.tick(velocity=Observation((0., 0., .7), rig.t+.05, 0.))
    assert out.phase == "HANDOVER" and "velocity_limit" in out.exit_reason


def test_changing_fcu_landing_parameters_in_flight_aborts_instead_of_changing_speed():
    rig = fcu_rate_rig()
    rig.active()
    until_phase(rig, "TRANSIT")
    rig.params["MPC_LAND_CRWL"] = .35
    out = rig.tick()
    assert out.phase == "HANDOVER" and "landing_parameters_changed" in out.exit_reason


@pytest.mark.parametrize("missing", ["MPC_LAND_SPEED", "MPC_LAND_CRWL", "MPC_LAND_ALT3", "MPC_Z_VEL_MAX_DN"])
def test_missing_fcu_landing_speed_parameter_prevents_start(missing):
    rig = fcu_rate_rig()
    del rig.params[missing]
    out = rig.tick()
    assert out.target is None and "landing_speed_parameters_invalid" in out.reasons


def test_manual_descent_source_is_rejected():
    Config, _, _, _ = api()
    with pytest.raises(ValueError):
        Config(descent_source="manual")


def test_crawl_transition_allows_aircraft_to_decelerate_from_fcu_normal_descent():
    Config, _, _, _ = api()
    rig = fcu_rate_rig(Config(height_m=1.2))
    rig.params["MPC_LAND_ALT3"] = .5
    rig.active()
    until_phase(rig, "DESCEND")
    for _ in range(100):
        out = follow(rig)[0]
        if out.velocity_z == -.3:
            break
    assert out.velocity_z == -.3
    out = rig.tick(velocity=Observation((0., 0., -.7), rig.t+.05, 0.))
    assert out.phase == "DESCEND" and out.velocity_z == -.3


def test_missing_raw_receiver_prevents_prestream_and_ready():
    rig = route_rig()
    rig.transport_available = False
    for _ in range(150):
        out = rig.tick()
        assert out.target is None and not out.arm
    assert "mavros_raw_receiver_unavailable" in out.reasons


@pytest.mark.parametrize("value", [None, 7, False])
def test_indoor_route_requires_known_disabled_gnss_fusion_before_any_target(value):
    rig = route_rig()
    rig.params["EKF2_GPS_CTRL"] = value
    for _ in range(140):
        out = rig.tick()
        assert out.target is None and out.request_mode is None and not out.arm
    assert "indoor_gnss_fusion_not_disabled" in out.reasons


def test_enabling_gnss_during_route_cancels_task_and_returns_to_position():
    rig = route_rig()
    rig.active()
    until_phase(rig, "DESCEND")
    rig.params["EKF2_GPS_CTRL"] = 7
    out = rig.tick()
    assert out.phase == "HANDOVER" and out.request_mode == "POSCTL" and not out.arm
    assert "indoor_gnss_fusion_not_disabled" in out.exit_reason


@pytest.mark.parametrize("case", ["missing", "stale", "disagrees"])
def test_heading_input_must_be_fresh_and_agree_before_any_target(case):
    rig = route_rig()
    if case == "missing":
        rig.heading_available = False
    elif case == "stale":
        rig.heading_wire_age = .5
    else:
        rig.heading_offset = math.radians(-15)
    for _ in range(140):
        out = rig.tick()
        assert out.target is None and out.request_mode is None and not out.arm
    assert any("heading" in reason for reason in out.reasons)


def test_real_flight_yaw_divergence_aborts_before_starting_a_horizontal_leg():
    rig = route_rig()
    rig.active()
    rig.heading_offset = math.radians(-35)
    out = rig.tick()
    assert out.phase == "HANDOVER" and out.request_mode == "POSCTL" and not out.arm
    assert "heading_reference_disagreement" in out.exit_reason


def test_arrival_timer_also_requires_actual_heading_to_match_locked_heading():
    rig = route_rig()
    rig.active()
    until_phase(rig, "ORIGIN_HOLD")
    x, y, z, yaw, tilt = rig.pose
    rig.pose = (x, y, z, yaw+math.radians(7), tilt)
    for _ in range(120):
        assert rig.tick().phase == "ORIGIN_HOLD"


def test_flight_profile_refuses_external_position_without_external_yaw():
    rig = route_rig()
    rig.params["EKF2_EV_CTRL"] = 3
    for _ in range(140):
        out = rig.tick()
        assert out.target is None and not out.arm
    assert any("EKF2_EV_CTRL" in r and "11" in r for r in out.reasons)


def test_heading_comparison_wraps_at_pi():
    rig = route_rig(mode="check")
    x, y, z, _, tilt = rig.pose
    rig.pose = (x, y, z, math.pi-.01, tilt)
    rig.heading_offset = -2*math.pi+.02
    for _ in range(140):
        out = rig.tick()
    assert not out.reasons


def test_stale_external_heading_in_flight_requests_immediate_handover():
    rig = route_rig()
    rig.active()
    rig.heading_wire_age = .5
    out = rig.tick()
    assert out.phase == "HANDOVER" and out.request_mode == "POSCTL"
    assert "external_heading_stale" in out.exit_reason


def test_both_heading_sources_turning_away_from_locked_heading_also_aborts():
    rig = route_rig()
    rig.active()
    x, y, z, yaw, tilt = rig.pose
    rig.pose = (x, y, z, yaw-math.radians(15), tilt)
    out = rig.tick()
    assert out.phase == "HANDOVER" and "heading_target_drift" in out.exit_reason


def test_nonfinite_external_heading_prevents_ready():
    rig = route_rig()
    rig.heading_offset = float("nan")
    for _ in range(140):
        out = rig.tick()
        assert out.target is None and not out.arm
    assert "external_heading_invalid" in out.reasons


def test_losing_raw_receiver_in_offboard_immediately_hands_back_without_arming():
    rig = route_rig(mode="mode-test")
    rig.active()
    rig.transport_available = False
    out = rig.tick()
    assert out.phase == "HANDOVER" and out.request_mode == "POSCTL" and not out.arm
    assert "mavros_raw_receiver_unavailable" in out.exit_reason
