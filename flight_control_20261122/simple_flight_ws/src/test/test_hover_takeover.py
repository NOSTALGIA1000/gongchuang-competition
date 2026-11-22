"""Behavior contracts for takeover, without ROS or hardware commands."""

import math

import pytest

from simple_flight_mission.hover_takeover import HoverTakeover, TakeoverConfig


def channels(swa=2003, throttle=1024):
    return [1514, 1514, throttle, 1514, 2003, 1514, swa]


def requested_gate():
    gate = HoverTakeover()
    gate.observe(channels(swa=1024), 0.0)
    gate.observe(channels(swa=1024), 0.25)
    assert gate.observe(channels(), 0.30, entry_ready=True).action == "start"
    return gate


def active_gate():
    gate = requested_gate()
    gate.confirm_offboard(0.31)
    return gate


def test_high_at_start_never_becomes_start_without_low_first():
    gate = HoverTakeover()
    for t in (0.0, 0.25, 0.5):
        assert gate.observe(channels(), t).action == "none"
    assert gate.phase == "WAIT_LOW"


def test_low_must_persist_before_start_then_start_is_a_single_event():
    gate = HoverTakeover()
    gate.observe(channels(swa=1024), 0.0)
    assert gate.observe(channels(), 0.1).action == "none"
    gate.observe(channels(swa=1024), 0.2)
    gate.observe(channels(swa=1024), 0.45)
    assert gate.observe(channels(), 0.50, entry_ready=True).action == "start"
    assert gate.observe(channels(), 0.55).action == "none"


def test_high_while_not_ready_requires_another_low_high_cycle():
    gate = HoverTakeover()
    gate.observe(channels(swa=1024), 0.0)
    gate.observe(channels(swa=1024), 0.25)
    assert gate.observe(channels(), 0.3, entry_ready=False).action == "none"
    assert gate.observe(channels(), 0.4, entry_ready=True).action == "none"
    assert gate.phase == "WAIT_LOW"


def test_start_is_denied_unless_adapter_explicitly_reports_ready():
    gate = HoverTakeover()
    gate.observe(channels(swa=1024), 0.0)
    gate.observe(channels(swa=1024), 0.25)
    assert gate.observe(channels(), 0.3).action == "none"
    assert gate.phase == "WAIT_LOW"


def test_start_with_raised_throttle_is_consumed_and_requires_new_switch_cycle():
    gate = HoverTakeover()
    gate.observe(channels(swa=1024), 0.0)
    gate.observe(channels(swa=1024), 0.25)
    assert gate.observe(channels(throttle=1514), 0.3, entry_ready=True).action == "none"
    assert gate.observe(channels(), 0.4, entry_ready=True).action == "none"


def test_initial_low_throttle_is_not_an_exit():
    gate = active_gate()
    for t in (0.4, 0.6, 0.8):
        assert gate.observe(channels(), t).action == "none"
    assert gate.phase == "ACTIVE"


def test_raise_before_actual_offboard_does_not_arm_throttle_exit():
    gate = requested_gate()
    gate.observe(channels(throttle=1514), 0.4)
    gate.observe(channels(throttle=1514), 0.65)
    gate.confirm_offboard(0.66)
    gate.observe(channels(), 0.7)
    assert gate.observe(channels(), 0.85).action == "none"


def test_sustained_raise_then_low_requests_exit_without_human_confirmation():
    gate = active_gate()
    gate.observe(channels(throttle=1514), 0.4)
    gate.observe(channels(throttle=1514), 0.65)
    gate.observe(channels(throttle=1024), 0.7)
    decision = gate.observe(channels(throttle=1024), 0.85)
    assert (decision.action, decision.reason, gate.phase) == (
        "exit", "throttle_raise_then_low", "EXITED")


def test_short_throttle_raise_is_ignored():
    gate = active_gate()
    gate.observe(channels(throttle=1514), 0.4)
    gate.observe(channels(throttle=1514), 0.5)
    gate.observe(channels(), 0.55)
    assert gate.observe(channels(), 0.75).action == "none"


def test_short_low_pulse_after_raise_does_not_exit():
    gate = active_gate()
    gate.observe(channels(throttle=1514), 0.4)
    gate.observe(channels(throttle=1514), 0.65)
    gate.observe(channels(), 0.7)
    assert gate.observe(channels(throttle=1200), 0.75).action == "none"
    gate.observe(channels(), 0.8)
    assert gate.observe(channels(), 0.95).action == "exit"


@pytest.mark.parametrize("active", [False, True])
def test_swa_low_exits_immediately_even_before_arming(active):
    gate = active_gate() if active else requested_gate()
    decision = gate.observe(channels(swa=1024), 0.4)
    assert (decision.action, decision.reason) == ("exit", "swa_low")


def test_exit_is_latched_even_after_another_low_high_cycle():
    gate = active_gate()
    gate.observe(channels(swa=1024), 0.4)
    gate.observe(channels(swa=1024), 0.65)
    assert gate.observe(channels(), 0.7).action == "none"
    assert gate.phase == "EXITED"
    assert gate.confirm_offboard(0.71).action == "none"


@pytest.mark.parametrize("bad", [[], [1514, 1514, 1024, 1514], channels(swa=65535), channels(throttle=0), channels(swa=2500)])
def test_missing_or_invalid_control_channels_exit_active_task(bad):
    gate = active_gate()
    decision = gate.observe(bad, 0.4)
    assert (decision.action, decision.reason) == ("exit", "invalid_rc")


def test_missing_channel_before_start_discards_old_low_qualification():
    gate = HoverTakeover()
    gate.observe(channels(swa=1024), 0.0)
    gate.observe(channels(swa=1024), 0.25)
    gate.observe([], 0.3)
    assert gate.observe(channels(), 0.4, entry_ready=True).action == "none"


def test_stale_rc_requests_exit_without_waiting_for_another_frame():
    gate = active_gate()
    assert gate.poll(0.59).action == "none"
    decision = gate.poll(0.61)
    assert (decision.action, decision.reason) == ("exit", "rc_timeout")


def test_large_gap_between_frames_cannot_complete_raise_or_trigger_start():
    gate = active_gate()
    gate.observe(channels(throttle=1514), 0.4)
    assert gate.observe(channels(throttle=1514), 0.8).reason == "rc_timeout"
    waiting = HoverTakeover()
    waiting.observe(channels(swa=1024), 0.0)
    waiting.observe(channels(swa=1024), 0.25)
    assert waiting.observe(channels(), 0.65, entry_ready=True).action == "none"
    assert waiting.phase == "WAIT_LOW"


def test_stale_waiting_gate_requires_low_again():
    gate = HoverTakeover()
    gate.observe(channels(swa=1024), 0.0)
    gate.observe(channels(swa=1024), 0.25)
    gate.poll(0.6)
    assert gate.observe(channels(), 0.65, entry_ready=True).action == "none"


def test_duplicate_frames_cannot_accumulate_time_and_old_frames_do_not_replace_new():
    gate = active_gate()
    gate.observe(channels(throttle=1514), 0.4)
    for _ in range(10):
        assert gate.observe(channels(throttle=1514), 0.4).action == "none"
    assert gate.observe(channels(swa=1024), 0.35).action == "none"
    gate.observe(channels(), 0.5)
    assert gate.observe(channels(), 0.7).action == "none"


def test_cannot_confirm_offboard_without_an_accepted_trigger():
    with pytest.raises(RuntimeError):
        HoverTakeover().confirm_offboard(0.0)


def test_stale_rc_cannot_activate_even_after_an_accepted_trigger():
    gate = requested_gate()
    decision = gate.confirm_offboard(0.7)
    assert (decision.action, decision.reason, gate.phase) == ("exit", "rc_timeout", "EXITED")


@pytest.mark.parametrize("t", [math.nan, math.inf, -1.0])
def test_bad_monotonic_time_is_rejected(t):
    with pytest.raises(ValueError):
        HoverTakeover().observe(channels(), t)


@pytest.mark.parametrize("kwargs", [
    {"swa_low_pwm": 1800, "swa_high_pwm": 1200},
    {"throttle_low_pwm": 1500, "throttle_raise_pwm": 1400},
    {"rc_timeout_s": 0.0}, {"raise_hold_s": -0.1},
    {"raise_hold_s": math.nan}, {"swa_channel": 0},
    {"swa_channel": 3}, {"swa_channel": 1.5},
    {"throttle_raise_pwm": 2500},
])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        TakeoverConfig(**kwargs)
