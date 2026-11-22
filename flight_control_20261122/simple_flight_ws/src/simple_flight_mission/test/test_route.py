import math

import pytest

from simple_flight_mission.route import MissionRoute


def test_front_is_heading_direction_in_local_enu_xy():
    #起飞朝向 yaw=90 度（ENU +Y）时，前方 1 m 应落在 +Y。
    route = MissionRoute(origin=(10., 20., 1.5), yaw_rad=math.pi / 2,
                         points=[('front', 1.0, 0.0), ('left', 0.0, 1.0)])
    world = route.world_points()
    assert world[0] == pytest.approx((10., 21., 1.5))
    assert world[1] == pytest.approx((9., 20., 1.5))  # 左侧在机头 90 度时是 -X


def test_left_is_ninety_degrees_counterclockwise_from_heading():
    route = MissionRoute(origin=(0., 0., 1.0), yaw_rad=0.0,
                         points=[('front', 1.0, 0.0), ('left', 0.0, 1.0)])
    assert route.world_points()[0] == pytest.approx((1., 0., 1.0))
    assert route.world_points()[1] == pytest.approx((0., 1., 1.0))
