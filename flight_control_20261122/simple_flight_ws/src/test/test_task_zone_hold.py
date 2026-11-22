"""Departure timing at the two stationary scoring waypoints."""
import pytest

from simple_flight_mission.hover_controller import Observation
from test.test_rescue_aisle_workflow import RescueRig


@pytest.mark.parametrize("index", [2, 4])
def test_task_completion_and_continuous_dwell_are_independent(index):
    rig = RescueRig(1)
    controller = rig.controller
    anchor = rig.pose[:4]
    controller.world_route = [anchor] * 5
    controller.goal = anchor
    controller.point_index = index
    controller._phase("POINT_HOLD", 10.)
    controller.external_heading = Observation(anchor[3], 10., 0.)
    controller._hold_at = 10.
    assert controller._stable_arrival(rig.vehicle(), 10.)
    assert not controller._hold_complete(14., 3.)
    controller.completed_tasks.add(index)
    assert not controller._hold_complete(13., 3.)
    assert controller._hold_complete(13.05, 3.)
    assert not controller._hold_complete(13.05, 5.)
    controller.goal = (anchor[0] + .2, *anchor[1:])
    assert not controller._stable_arrival(rig.vehicle(), 14.)
    assert controller._hold_at is None


def test_speed_excursion_after_completion_requires_a_new_hold():
    rig = RescueRig(1)
    rig.active()
    for _ in range(4000):
        rig.advance()
        if 2 in rig.controller.completed_tasks:
            break
    else:
        pytest.fail("Did not reach completed first task")
    controller = rig.controller
    assert controller.phase == "POINT_HOLD"
    out = rig.tick(velocity=Observation((.2, 0., 0.), rig.t + .05, 0.))
    assert out.phase == "POINT_HOLD" and controller._hold_at is None
    rig.advance()
    recovered = controller._hold_at
    assert recovered is not None
    for _ in range(65):
        out = rig.advance()
        if out.phase == "TRANSIT":
            assert 3. < rig.t - recovered <= 3.06
            return
    pytest.fail("Did not depart after continuous recovery")


def test_return_deadline_preempts_unfinished_hold():
    rig = RescueRig(1)
    rig.active()
    for _ in range(4000):
        out = rig.advance()
        if out.phase == "POINT_HOLD" and out.point_index == 2:
            break
    else:
        pytest.fail("Did not reach first scoring waypoint")
    rig.controller.competition_started_at = rig.t - 179.
    rig.advance()
    assert rig.controller.returning
    assert rig.controller.visual_error == "return_time_reserved"
