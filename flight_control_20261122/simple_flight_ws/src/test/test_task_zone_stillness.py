"""Task-zone departure dwell and approach taper (route 1 opts in).

The competition awards points for a hover above each task zone that lasts more
than three seconds and for an annotation that stays visible for three seconds.
Historically the controller pre-charged that dwell, so the aircraft left about
1.4 s after opening a door. These tests pin the dwell and the approach taper;
the stillness gate that used to sit here was removed by request.
"""
from dataclasses import replace

import pytest

from simple_flight_mission.hover_controller import Observation
from simple_flight_mission.route_controller import RouteController
from test.test_rescue_aisle_workflow import RescueRig


def _config(**changes):
    from pathlib import Path
    from simple_flight_mission.route_config import load_route_config
    root = Path(__file__).resolve().parents[1]
    return replace(load_route_config(root / "config/route_1.yaml"), **changes)


def _zone_rig(**changes):
    rig = RescueRig(1)
    rig.controller.route_config = _config(**changes)
    rig.controller.config = rig.controller.route_config.core_config()
    return rig


def _at_zone(rig, index=2):
    controller = rig.controller
    anchor = rig.pose[:4]
    controller.world_route = [anchor] * 5
    controller.goal = anchor
    controller.point_index = index
    controller._phase("POINT_HOLD", rig.t)
    controller.external_heading = Observation(anchor[3], rig.t, 0.)
    return controller


def _sample(rig, velocity=(0., 0., 0.), pose=None):
    base = rig.vehicle()
    return replace(base,
                   pose=Observation(pose or rig.pose, rig.t, 0.),
                   velocity=Observation(velocity, rig.t, 0.))


def test_route_1_opts_in_and_other_routes_keep_the_old_timing():
    from pathlib import Path
    from simple_flight_mission.route_config import load_route_config
    root = Path(__file__).resolve().parents[1]
    route1 = load_route_config(root / "config/route_1.yaml")
    assert route1.zone_dwell_after_task is True
    assert route1.zone_approach_taper == 1.
    assert route1.zone_approach_min_lead_m == .04
    for number in (2, 3, 4):
        other = load_route_config(root / f"config/route_{number}.yaml")
        assert other.zone_dwell_after_task is False
        assert other.zone_approach_taper is None


@pytest.mark.parametrize("changes", [
    dict(zone_approach_taper=1.5),       # would never slow down
    dict(zone_approach_taper=0.),        # zero lead cannot converge
    dict(zone_approach_min_lead_m=.25),  # no room below the cruise lead
])
def test_unsound_approach_configuration_is_rejected(changes):
    with pytest.raises(ValueError):
        _config(**changes)


def test_dwell_switch_must_be_a_boolean():
    with pytest.raises(ValueError):
        _config(zone_dwell_after_task=1)


def test_no_departure_before_the_task_completes():
    rig = _zone_rig()
    controller = _at_zone(rig)
    for _ in range(200):  # ten seconds at the waypoint, still unfinished
        rig.t += .05
        controller._hold_at = rig.t
        assert not controller._hold_complete(rig.t, 3.)


def test_departure_requires_the_full_dwell_after_completion():
    rig = _zone_rig()
    controller = _at_zone(rig)
    controller.completed_tasks.add(2)
    controller._hold_at = rig.t
    for elapsed in (.5, 1., 2., 2.95):
        assert not controller._hold_complete(rig.t + elapsed, 3.)
    assert controller._hold_complete(rig.t + 3., 3.)


def test_drift_restarts_the_dwell_like_the_original_contract():
    rig = _zone_rig()
    controller = _at_zone(rig)
    controller.completed_tasks.add(2)
    controller._hold_at = rig.t
    for _ in range(20):
        rig.t += .05
        controller._stable_arrival(_sample(rig, velocity=(.3, 0., 0.)), rig.t)
    # Moving off the waypoint clears the dwell, exactly as before this change.
    assert controller._hold_at is None
    assert not controller._hold_complete(rig.t, 3.)
    rig.t += .05
    assert controller._stable_arrival(_sample(rig), rig.t)
    controller._hold_at = rig.t
    assert not controller._hold_complete(rig.t + 2.9, 3.)
    assert controller._hold_complete(rig.t + 3., 3.)


def test_routes_without_the_switch_keep_immediate_dwell():
    rig = RescueRig(2)
    controller = rig.controller
    anchor = rig.pose[:4]
    controller.world_route = [anchor] * 5
    controller.goal = anchor
    controller.point_index = 2
    controller._phase("POINT_HOLD", rig.t)
    controller.completed_tasks.add(2)
    controller._hold_at = rig.t - 3.
    # Legacy timing is pre-charged, so the waypoint releases at once.
    assert controller._hold_complete(rig.t, 3.)


def test_approach_lead_tapers_only_inside_a_task_zone():
    from pathlib import Path
    from simple_flight_mission.route_config import load_route_config
    root = Path(__file__).resolve().parents[1]
    config = load_route_config(root / "config/route_1.yaml")
    controller = RouteController("check", config)
    controller.point_index = 2
    length = 2.2
    leads = [controller._target_lead(along, length)
             for along in (0., .5, 1., 1.5, 1.9, 2.1, 2.18)]
    assert leads[0] == pytest.approx(config.target_lead_m)
    assert all(a >= b - 1e-9 for a, b in zip(leads, leads[1:]))
    assert leads[-1] == pytest.approx(config.zone_approach_min_lead_m)
    assert max(leads) <= config.target_lead_m

    controller.point_index = 1  # obstacle_2 keeps the fixed lead
    assert controller._target_lead(2.1, length) == pytest.approx(config.target_lead_m)
    controller.point_index = 2
    controller.returning = True  # the return leg is not tapered either
    assert controller._target_lead(2.1, length) == pytest.approx(config.target_lead_m)


def test_routes_without_the_contract_keep_the_fixed_lead():
    rig = RescueRig(2)
    controller = rig.controller
    controller.point_index = 0
    assert controller.route_config.zone_approach_taper is None
    assert controller._target_lead(1.9, 2.2) == pytest.approx(
        controller.route_config.target_lead_m)


@pytest.mark.parametrize("index", [2, 4])
def test_zone_anchors_are_the_world_waypoints(index):
    rig = _zone_rig()
    rig.active()
    for _ in range(4000):
        out = rig.advance()
        if out.phase == "POINT_HOLD" and out.point_index == index:
            controller = rig.controller
            assert controller._at_task_zone()
            anchor = controller.world_route[index]
            assert controller.goal[:2] == pytest.approx(anchor[:2])
            return
        assert out.phase not in ("HANDOVER", "EXITED"), out
    pytest.fail("never reached the task zone")


def _blind_hazard(monkeypatch):
    """Every hazardous frame arrives with no detections."""
    import test.test_rescue_aisle_workflow as mod
    real = mod.frame

    def blind(now, kind=2):
        snapshot = real(now, kind=kind)
        if kind == 2:
            snapshot["detections"] = []
        return snapshot
    monkeypatch.setattr(mod, "frame", blind)


def test_unrecognised_hazardous_target_then_completes_the_person_task(monkeypatch):
    """Skip the failed fire drop, still run the rescue drop, then return."""
    _blind_hazard(monkeypatch)
    rig = RescueRig(1, person=True)
    rig.active()
    reached_obstacle_3 = False
    for _ in range(9000):
        out = rig.advance()
        if out.phase == "TRANSIT" and out.point_index == 3:
            reached_obstacle_3 = True
            assert not rig.controller.returning
            assert rig.controller.visual_error == "correct_target_not_found"
            assert 2 in rig.controller.completed_tasks
            assert not any(door == 1 for door, _ in rig.actions), "no fire drop"
        if rig.controller.returning:
            assert reached_obstacle_3, "must keep flying after the failed fire task"
            assert any(door == 2 and action == "open" for door, action in rig.actions),                 "the rescue drop must still happen"
            assert not any(door == 1 for door, _ in rig.actions), "no fire drop"
            assert 4 in rig.controller.completed_tasks
            assert rig.controller.return_reason == "route_complete",                 "continue with the normal configured return, not a breadcrumb abort"
            assert rig.controller.visual_error == "correct_target_not_found"
            return
        assert out.phase not in ("HANDOVER", "EXITED"), out
    pytest.fail("route did not continue after the unrecognised hazardous target")


def test_abort_still_happens_when_the_switch_is_off(monkeypatch):
    _blind_hazard(monkeypatch)
    rig = RescueRig(1)
    rig.controller.route_config = replace(
        rig.controller.route_config, continue_after_target_not_found=False)
    rig.active()
    for _ in range(6000):
        out = rig.advance()
        if rig.controller.returning:
            assert rig.controller.return_reason == "mission_timeout"
            return
        assert out.phase not in ("HANDOVER", "EXITED"), out
    pytest.fail("expected the mission to abort without the switch")


def _feed_marked(task, kind=2, t0=.1, t1=1., step=.1, start=1):
    """Step a task with a fresh centered frame every `step` seconds."""
    from test.test_competition import frame as snapshot_frame
    outs = []
    n = start
    t = t0
    while t <= t1 + 1e-9:
        outs.append(task.step((0., 0., 1., 0., 0.), (0., 0., 0.), t,
                              snapshot_frame(t, kind=kind, sequence=n)))
        n += 1
        t = round(t + step, 6)
    return outs, n


def test_marked_dwell_requires_three_seconds_before_opening():
    from simple_flight_mission.visual_task import VisualTask
    task = VisualTask(1, (0., 0., 1., 0.), [], 0., mark_hold_s=3.)
    outs, n = _feed_marked(task, t0=.1, t1=2.9)
    assert not any(o.action == "open" for o in outs), "one frame must not open the door"
    from test.test_competition import frame as snapshot_frame
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 3.1,
                    snapshot_frame(3.1, kind=2, sequence=n))
    assert out.action == "open"


def test_a_gap_in_the_marked_frames_restarts_the_dwell():
    from simple_flight_mission.visual_task import VisualTask
    from test.test_competition import frame as snapshot_frame
    task = VisualTask(1, (0., 0., 1., 0.), [], 0., mark_hold_s=3.)
    _feed_marked(task, t0=.1, t1=2.0)
    # A 1 s gap exceeds the continuity window, so the dwell starts over.
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 3.0,
                    snapshot_frame(3.0, kind=2, sequence=99))
    assert out.action is None
    outs, _ = _feed_marked(task, t0=3.1, t1=5.9, start=100)
    assert not any(o.action == "open" for o in outs)
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 6.1,
                    snapshot_frame(6.1, kind=2, sequence=200))
    assert out.action == "open"


def test_bounded_wait_drops_with_the_best_available_frame():
    from simple_flight_mission.visual_task import VisualTask
    from test.test_competition import frame as snapshot_frame
    # Frames arrive 1 s apart, so the 3 s window never completes.
    task = VisualTask(1, (0., 0., 1., 0.), [], 0., mark_hold_s=3., mark_max_wait_s=4.)
    _feed_marked(task, t0=.1, t1=3.1, step=1.0)
    assert not task.open_sent
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 4.2,
                    snapshot_frame(4.2, kind=2, sequence=50))
    assert out.action == "open", "the deadline must release with a valid frame"
    assert task.wait_reason == "mark_deadline_best_effort"


def test_bounded_wait_gives_the_waypoint_up_when_nothing_is_visible():
    from simple_flight_mission.visual_task import VisualTask
    from test.test_competition import frame as snapshot_frame
    task = VisualTask(1, (0., 0., 1., 0.), [], 0., mark_hold_s=3., mark_max_wait_s=4.)
    _feed_marked(task, t0=.1, t1=.6)
    blank = snapshot_frame(4.5, kind=2, sequence=60)
    blank["detections"] = []
    task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 4.5, blank)
    assert task.failure == "correct_target_not_found"
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 4.6, blank)
    assert out.failed == "correct_target_not_found"


def test_dwell_is_off_unless_configured():
    from simple_flight_mission.visual_task import VisualTask
    from test.test_competition import frame as snapshot_frame
    task = VisualTask(1, (0., 0., 1., 0.), [], 0.)  # no mark_hold_s
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1,
                    snapshot_frame(.1, kind=2))
    assert out.action == "open"


def test_route_one_keeps_the_annotation_for_three_seconds():
    config = _config()
    assert config.mark_hold_s == 3.
    assert config.mark_max_wait_s == 12.
    for number in (2, 3, 4):
        from pathlib import Path
        from simple_flight_mission.route_config import load_route_config
        other = load_route_config(Path(__file__).resolve().parents[1] / f"config/route_{number}.yaml")
        assert other.mark_hold_s == 0.
