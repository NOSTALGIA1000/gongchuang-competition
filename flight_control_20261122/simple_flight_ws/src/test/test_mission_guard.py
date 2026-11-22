import pytest

from simple_flight_mission.guard import MissionGuard


def make_healthy_snapshot(age_s=0.05, mode="AUTO.LOITER"):
    return {
        "now": 100.0,
        "fcu_connected": True,
        "armed": True,
        "mode": mode,
        "expected_mode": "AUTO.LOITER",
        "position_valid": True,
        "height_valid": True,
        "lio_age_s": age_s,
        "max_position_age_s": 0.3,
        "position_error_m": 0.15,
        "max_position_error_m": 0.5,
        "horizontal_speed_mps": 0.3,
        "vertical_speed_mps": 0.1,
        "max_horizontal_speed_mps": 0.8,
        "max_vertical_speed_mps": 0.6,
        "battery_voltage_v": 25.0,
        "min_battery_voltage_v": 24.0,
    }


def test_healthy_snapshot_passes():
    assert MissionGuard.evaluate(make_healthy_snapshot()) == []

def test_each_violation_is_reported_with_reason():
    cases = [
        ({"fcu_connected": False}, "fcu"),
        ({"mode": "POSCTL"}, "mode"),
        ({"lio_age_s": 1.0}, "stale"),
        ({"position_error_m": 2.0}, "error"),
        ({"horizontal_speed_mps": 5.0}, "speed"),
        ({"battery_voltage_v": 22.0}, "battery"),
    ]
    for overrides, needle in cases:
        snapshot = make_healthy_snapshot()
        snapshot.update(overrides)
        reasons = MissionGuard.evaluate(snapshot)
        assert reasons, overrides
        assert any(needle in reason for reason in reasons), (overrides, reasons)

def test_disarmed_or_disconnected_fcu_fails_closed():
    snapshot = make_healthy_snapshot()
    snapshot["armed"] = False
    assert any("fcu" in reason for reason in MissionGuard.evaluate(snapshot))
