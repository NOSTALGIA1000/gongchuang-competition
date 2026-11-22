from dataclasses import replace

import pytest

from simple_flight_mission.hover_controller import Observation
from test.test_rescue_aisle_workflow import RescueRig


def at_takeoff_height(route):
    rig = RescueRig(route)
    rig.active()
    c = rig.controller
    x, y, z, yaw = c.origin
    rig.pose = (x, y, z + c.route_config.height_m, yaw, 0.)
    rig.landed = 2
    return rig


def test_route_one_waits_at_origin_while_moving_and_releases_on_recovery():
    rig = at_takeoff_height(1)
    for _ in range(8):
        out = rig.tick(velocity=Observation((-.04, .04, 0.), rig.t+.05, 0.))
        assert out.phase == 'TAKEOFF'
        assert out.target[:2] == rig.controller.origin[:2]
    out = rig.tick(velocity=Observation((0., -.05, 0.), rig.t+.05, 0.))
    assert out.phase == 'TRANSIT'


def test_persistent_drift_preserves_takeoff_timeout():
    rig = at_takeoff_height(1)
    for _ in range(320):
        out = rig.tick(velocity=Observation((0., .08, 0.), rig.t+.05, 0.))
        if out.phase != 'TAKEOFF':
            assert out.phase == 'HANDOVER'
            assert out.exit_reason == 'takeoff_timeout'
            return
    pytest.fail('Takeoff timeout did not fire')


@pytest.mark.parametrize('route', [2, 3, 4])
def test_existing_route_departure(route):
    rig = at_takeoff_height(route)
    out = rig.tick(velocity=Observation((0., .08, 0.), rig.t+.05, 0.))
    assert out.phase == 'TRANSIT'


@pytest.mark.parametrize('value', [0., -.1, float('nan'), True, .11])
def test_invalid_limit(value):
    with pytest.raises(ValueError):
        replace(RescueRig(1).controller.route_config, takeoff_horizontal_speed_max_mps=value)
