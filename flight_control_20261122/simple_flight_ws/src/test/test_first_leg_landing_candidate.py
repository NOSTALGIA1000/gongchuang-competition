"""Pure controller checks for the path 1 candidate."""
from dataclasses import replace
import math

import pytest

from simple_flight_mission.route_config import RouteConfig, RoutePoint
from test.test_route_flight import follow, route_rig, until_phase


def test_first_leg_target_ramps_and_other_leg_keeps_nominal_speed():
    config = RouteConfig(height_m=.5, horizontal_mps=.3, first_leg_speed_mps=.24,
                         points=(RoutePoint(1., hold_s=.1), RoutePoint(2., hold_s=.1)))
    rig = route_rig(config)
    rig.active()
    start = until_phase(rig, 'TRANSIT')
    targets = [start.target] + [out.target for out in follow(rig, 23)]
    speeds = [math.dist(a[:2], b[:2]) / .05 for a, b in zip(targets, targets[1:])]
    assert speeds[0] == pytest.approx(.012)
    assert all(a <= b + 1e-9 for a, b in zip(speeds, speeds[1:]))
    assert speeds[-1] == pytest.approx(.24)
    assert max(speeds) <= .24 + 1e-9
    assert all(target[2:] == start.target[2:] for target in targets)

    until_phase(rig, 'POINT_HOLD')
    start = until_phase(rig, 'TRANSIT')
    out = follow(rig)[0]
    assert math.dist(start.target[:2], out.target[:2]) / .05 == pytest.approx(.3)


def return_rig():
    config = RouteConfig(height_m=.5, horizontal_mps=.3, return_hold_s=3., quick_home_hold_s=1.,
                         points=(RoutePoint(1., hold_s=.1),))
    rig = route_rig(config)
    rig.active()
    until_phase(rig, 'RETURN_HOLD')
    return rig


def test_centered_home_hold_shortens_wait_with_old_descent_rate():
    rig = return_rig()
    started = rig.t
    out = until_phase(rig, 'DESCEND')
    assert 1. <= rig.t - started <= 1.1
    assert out.velocity_z == pytest.approx(-.3)
    assert out.target[:2] == rig.controller.origin[:2]
    assert rig.landed == 2
    assert not rig.controller.mission_completed


def test_offcenter_home_hold_uses_old_wait_and_resets_quick_timer():
    rig = return_rig()
    started = rig.t
    home = rig.pose
    for _ in range(12):
        assert rig.tick().phase == 'RETURN_HOLD'
    rig.pose = (home[0]+.11, *home[1:])
    for _ in range(20):
        assert rig.tick().phase == 'RETURN_HOLD'
    rig.pose = home
    for _ in range(15):
        assert rig.tick().phase == 'RETURN_HOLD'
    rig.pose = (home[0]+.11, *home[1:])
    while rig.t - started < 2.95 - 1e-9:
        assert rig.tick().phase == 'RETURN_HOLD'
    assert rig.tick().phase == 'DESCEND'


@pytest.mark.parametrize('changed', [{'first_leg_speed_mps': .31}, {'first_leg_speed_mps': True},
                                    {'quick_home_hold_s': .5}, {'quick_home_hold_s': 3.5}])
def test_invalid_candidate_limits_rejected(changed):
    with pytest.raises(ValueError):
        replace(RouteConfig(horizontal_mps=.3, return_hold_s=3.), **changed)
