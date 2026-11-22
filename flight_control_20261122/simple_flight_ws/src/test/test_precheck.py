import pytest

from simple_flight_mission.precheck import Precheck


def healthy_snapshot():
    return {
        "fcu_connected": True,
        "armed": False,
        "mode": "AUTO.LOITER",
        "expected_mode": "AUTO.LOITER",
        "position_valid": True,
        "height_valid": True,
        "lio_age_s": 0.05,
        "max_position_age_s": 0.3,
        "battery_voltage_v": 25.0,
        "min_battery_voltage_v": 24.0,
    }

def test_ground_precheck_passes_when_disarmed_and_position_ok():
    assert Precheck.evaluate(healthy_snapshot()) == []

def test_ground_precheck_rejects_armed_or_bad_position():
    s = healthy_snapshot()
    s["armed"] = True
    assert Precheck.evaluate(s)
    s2 = healthy_snapshot()
    s2["position_valid"] = False
    assert Precheck.evaluate(s2)

def test_ground_precheck_rejects_low_battery_or_stale_lio():
    s = healthy_snapshot()
    s["battery_voltage_v"] = 23.0
    assert Precheck.evaluate(s)
    s2 = healthy_snapshot()
    s2["lio_age_s"] = 5.0
    assert Precheck.evaluate(s2)
