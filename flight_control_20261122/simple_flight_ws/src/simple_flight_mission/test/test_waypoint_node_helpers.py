import math

import pytest

from simple_flight_mission.waypoint_node import (
    expand_world_points,
    manual_stick_active,
    throttle_drop_active,
    offboard_wait_mode_allowed,
    mode_request_expired,
    format_mode_request_result,
    mode_retry_due,
    rc_switch_active,
    rc_switch_low,
)


def test_expand_world_points_uses_fixed_initial_heading():
    points = expand_world_points(
        origin=(10.0, 20.0, 1.0),
        yaw_rad=math.pi / 2,
        relative_points=[(1.0, 0.0), (1.0, 1.0), (2.0, 1.0), (0.0, 0.0)],
    )
    expected = [
        (10.0, 21.0, 1.0),
        (9.0, 21.0, 1.0),
        (9.0, 22.0, 1.0),
        (10.0, 20.0, 1.0),
    ]
    assert len(points) == len(expected)
    for actual, point in zip(points, expected):
        assert actual == pytest.approx(point)


def test_rc_switch_uses_one_based_channel_and_high_threshold():
    channels = [1514, 1514, 1024, 1514, 2003]
    assert rc_switch_active(channels, channel=5, high_pwm=1800)
    assert not rc_switch_active(channels, channel=5, high_pwm=2100)


def test_rc_switch_low_requires_present_valid_pwm():
    assert rc_switch_low([1514, 1514, 1024, 1514, 1024], channel=5, low_pwm=1200)
    assert not rc_switch_low([1514, 1514, 1024, 1514, 1514], channel=5, low_pwm=1200)
    assert not rc_switch_low([], channel=5, low_pwm=1200)


def test_manual_stick_detection_ignores_hover_throttle_channel():
    channels = [1514, 1514, 1900, 1514, 2003]
    assert not manual_stick_active(channels, threshold=0.2)
    channels[1] = 1700
    assert manual_stick_active(channels, threshold=0.2)


def test_throttle_drop_detection_uses_hover_baseline():
    channels = [1514, 1514, 1800, 1514, 2003]
    assert not throttle_drop_active(channels, baseline_pwm=1800, drop_pwm=150)
    channels[2] = 1600
    assert throttle_drop_active(channels, baseline_pwm=1800, drop_pwm=150)
    channels[2] = 1950
    assert not throttle_drop_active(channels, baseline_pwm=1800, drop_pwm=150)


def test_offboard_wait_allows_transition_from_posctl_to_offboard():
    assert offboard_wait_mode_allowed("POSCTL")
    assert offboard_wait_mode_allowed("OFFBOARD")
    assert not offboard_wait_mode_allowed("STABILIZED")


def test_mode_request_timeout_is_detected():
    assert mode_request_expired(10.0, 12.0, timeout_s=2.0)
    assert not mode_request_expired(10.0, 11.9, timeout_s=2.0)
    assert not mode_request_expired(None, 12.0, timeout_s=2.0)


def test_mode_request_result_includes_attempt_and_response():
    assert format_mode_request_result("OFFBOARD", 2, True) == \
        "OFFBOARD 第 2 次服务返回 mode_sent=True"


def test_mode_retry_waits_between_requests():
    assert not mode_retry_due(10.0, 10.5, interval_s=1.0)
    assert mode_retry_due(10.0, 11.0, interval_s=1.0)
    assert not mode_retry_due(None, 11.0, interval_s=1.0)
