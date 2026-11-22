"""Exercise the real control policy with deterministic vehicle observations."""
from dataclasses import replace
import math

import pytest

from simple_flight_mission.hover_controller import (
    EXPECTED_PARAMETERS, HoverConfig, HoverController, Observation, Vehicle,
)


class Rig:
    def __init__(self, mode="execute"):
        self.controller = HoverController(mode=mode)
        self.t = 0.0
        self.pose = (2.0, -3.0, 4.2, 0.7, 0.0)
        self.rc = [1514, 1514, 1024, 1514, 2003, 1514, 1024]
        self.mode = "POSCTL"
        self.armed = False
        self.landed = 1
        self.params = dict(EXPECTED_PARAMETERS, COM_OF_LOSS_T=1.0, RC_OFFB_TH=0.5)
        self.boot_run = "run-one"
        self.last = None

    def vehicle(self):
        def obs(v, wire=None):
            return Observation(v, self.t, wire)
        return Vehicle(
            boot_id="boot-one",
            state=obs(dict(mode=self.mode, armed=self.armed, connected=True, manual_input=True)),
            rc=obs(list(self.rc), 0.0), pose=obs(self.pose, 0.0),
            velocity=obs((0.0, 0.0, 0.0), 0.0), landed=obs(self.landed),
            lio=obs(True, 0.0), vision=obs(True, 0.0), estimator=obs(True),
            health=obs(dict(rc_ready=True, prearm_ready=True)),
            boot=obs(dict(boot_id="boot-one", run_id=self.boot_run, fault=None,
                          localization_active=True, ready_to_arm=True,
                          updated_monotonic=self.t, valid_until_monotonic=self.t + 2.5)),
            parameters=obs(dict(self.params)),
        )

    def tick(self, publish=True, **overrides):
        self.t += 0.05
        v = replace(self.vehicle(), **overrides)
        self.last = self.controller.step(v, self.t)
        if self.last.target is not None and publish:
            self.controller.note_target_sent(self.t)
        return self.last

    def ready(self):
        for _ in range(130):
            out = self.tick()
            if out.phase == "READY":
                return out
        raise AssertionError((self.last.phase, self.last.reasons))

    def triggered(self):
        self.ready()
        self.rc[6] = 2003
        out = self.tick()
        assert out.phase == "SWITCHING"
        return out

    def active(self):
        self.triggered()
        self.mode = "OFFBOARD"
        out = self.tick()
        if self.controller.mode == "mode-test":
            assert out.phase == "MODE_TEST"
            return out
        assert out.arm is True
        self.armed = True
        return self.tick()


def test_default_check_mode_never_emits_targets_modes_or_arm_even_with_switch_high():
    rig = Rig(mode="check")
    for i in range(140):
        rig.rc[6] = 2003 if i > 110 else 1024
        out = rig.tick()
        assert (out.target, out.request_mode, out.arm) == (None, None, False)


def test_no_observations_and_wrong_mapping_prevent_prestream():
    controller = HoverController(mode="execute")
    assert controller.step(Vehicle(), 0.0).target is None
    rig = Rig()
    rig.params["RC_MAP_OFFB_SW"] = 10
    for _ in range(130):
        out = rig.tick()
        assert out.target is None and not out.arm
    assert any("RC_MAP_OFFB_SW" in r for r in out.reasons)


def test_elapsed_time_alone_does_not_count_as_successful_prestream():
    rig = Rig()
    for _ in range(180):
        out = rig.tick(publish=False)
    assert out.phase == "PRESTREAM"
    assert not out.ready_for_switch and not out.arm


def test_prestream_is_ground_target_and_never_climbs_before_actual_arm():
    rig = Rig()
    rig.ready()
    assert rig.last.target == (2.0, -3.0, 4.2, 0.7)
    rig.rc[6] = 2003
    out = rig.tick()
    assert out.request_mode == "OFFBOARD" and not out.arm
    rig.controller.command_result("OFFBOARD", True)
    for _ in range(10):
        out = rig.tick()
        assert out.target[2] == 4.2 and not out.arm
    assert out.phase == "SWITCHING"


def test_native_mode_feedback_may_arrive_before_high_rc_callback():
    rig = Rig()
    rig.ready()
    rig.mode = "OFFBOARD"
    out = rig.tick()
    assert not out.arm
    rig.rc[6] = 2003
    out = rig.tick()
    assert out.phase in ("SWITCHING", "ARMING")
    out = rig.tick() if not out.arm else out
    assert out.arm


@pytest.mark.parametrize("operating_mode", ["mode-test", "execute"])
def test_offboard_manual_input_mode_flag_is_not_radio_link_health(operating_mode):
    rig = Rig(mode=operating_mode)
    rig.triggered()
    rig.mode = "OFFBOARD"
    state = rig.vehicle().state
    fields = dict(state.value, manual_input=False)
    out = rig.tick(state=replace(state, value=fields))
    assert out.phase == ("MODE_TEST" if operating_mode == "mode-test" else "ARMING")
    assert out.arm is (operating_mode == "execute")


def test_actual_radio_health_failure_in_offboard_still_exits():
    rig = Rig()
    rig.active()
    out = rig.tick(health=Observation(dict(rc_ready=False, prearm_ready=True), rig.t))
    assert out.phase == "HANDOVER" and out.exit_reason == "rc_not_ready"


def test_arming_is_requested_once_and_ack_does_not_start_climbing():
    rig = Rig()
    rig.triggered()
    rig.mode = "OFFBOARD"
    assert rig.tick().arm
    rig.controller.command_result("arm", True)
    for _ in range(10):
        out = rig.tick()
        assert not out.arm and out.target[2] == 4.2
    assert out.phase == "ARMING"


def test_takeoff_is_relative_one_meter_and_locks_xy_yaw():
    rig = Rig()
    rig.active()
    previous_z = 4.2
    for _ in range(150):
        out = rig.tick()
        assert out.target[:2] == (2.0, -3.0)
        assert out.target[3] == 0.7
        assert previous_z <= out.target[2] <= 5.2
        assert out.target[2] - previous_z <= 0.010001
        previous_z = out.target[2]
        rig.pose = (*out.target[:3], 0.7, 0.0)
        if out.target[2] > 4.25:
            rig.landed = 2
    assert out.phase == "HOVER"
    assert out.target[2] == pytest.approx(5.2)


def test_mode_test_never_arms_and_stays_at_ground_target():
    rig = Rig(mode="mode-test")
    rig.active()
    for _ in range(20):
        out = rig.tick()
        assert not out.arm and out.target[2] == 4.2


def test_swa_low_freezes_at_current_height_and_hands_back_without_confirmation():
    rig = Rig()
    rig.active()
    rig.pose = (2.0, -3.0, 4.55, 0.7, 0.0)
    rig.rc[6] = 1024
    out = rig.tick()
    assert out.phase == "HANDOVER" and out.request_mode == "POSCTL"
    assert out.target[2] == 4.55 and not out.arm
    for _ in range(5):
        out = rig.tick()
        assert out.target[2] == 4.55
    rig.mode = "POSCTL"
    out = rig.tick()
    assert out.target is None and out.handover_confirmed
    rig.rc[6] = 2003
    for _ in range(10):
        out = rig.tick()
        assert out.target is None and not out.arm and out.request_mode is None


def test_throttle_initial_low_is_ignored_then_raise_low_exits():
    rig = Rig()
    rig.active()
    for _ in range(10):
        assert rig.tick().phase == "TAKEOFF"
    rig.rc[2] = 1514
    for _ in range(7):
        rig.tick()
    rig.rc[2] = 1024
    for _ in range(4):
        out = rig.tick()
    assert out.phase == "HANDOVER"
    assert out.exit_reason == "throttle_raise_then_low"


def test_manual_mode_change_stops_targets_and_respects_selected_mode():
    rig = Rig()
    rig.active()
    rig.mode = "ALTCTL"
    out = rig.tick()
    assert out.target is None and out.request_mode is None and not out.arm
    assert out.phase == "EXITED"


def test_handover_timeout_stops_targets_even_without_mode_ack():
    rig = Rig()
    rig.active()
    rig.rc[6] = 1024
    rig.tick()
    for _ in range(24):
        out = rig.tick()
    assert out.target is None and not out.handover_confirmed
    assert out.phase == "EXITED"


@pytest.mark.parametrize("source", ["pose", "rc", "lio", "vision", "velocity", "state", "estimator", "health", "boot", "parameters"])
def test_stale_critical_input_exits_instead_of_continuing_climb(source):
    rig = Rig()
    rig.active()
    original = getattr(rig.vehicle(), source)
    out = rig.tick(**{source: replace(original, at=rig.t - 20.0)})
    assert out.phase in ("HANDOVER", "EXITED") and not out.arm


def test_recently_received_but_old_stamped_pose_is_rejected():
    rig = Rig()
    rig.active()
    out = rig.tick(pose=Observation(rig.pose, rig.t, 2.0))
    assert out.target is None and out.request_mode == "POSCTL"


@pytest.mark.parametrize("fault", ["generation", "publisher", "height", "horizontal", "jump", "tilt"])
def test_flight_faults_latch_exit(fault):
    rig = Rig()
    rig.active()
    changes = {}
    if fault == "generation":
        rig.boot_run = "another-run"
    elif fault == "publisher":
        changes["other_setpoint_publishers"] = 1
    elif fault == "height":
        rig.pose = (2.0, -3.0, 5.6, 0.7, 0.0)
    elif fault in ("horizontal", "jump"):
        rig.pose = (3.0, -3.0, 4.2, 0.7, 0.0)
    else:
        rig.pose = (2.0, -3.0, 4.2, 0.7, 0.6)
    out = rig.tick(**changes)
    assert out.phase in ("HANDOVER", "EXITED")
    assert out.exit_reason


def test_mode_request_and_arm_feedback_timeouts_never_force_progress():
    rig = Rig()
    rig.triggered()
    for _ in range(100):
        out = rig.tick()
        assert not out.arm
    assert out.phase == "EXITED"
    rig = Rig()
    rig.triggered()
    rig.mode = "OFFBOARD"
    assert rig.tick().arm
    for _ in range(85):
        out = rig.tick()
        assert not out.arm
    assert out.phase == "EXITED" and out.target is None


def test_rejected_service_result_cancels_and_late_arm_cannot_restart_task():
    rig = Rig()
    rig.triggered()
    rig.controller.command_result("OFFBOARD", False)
    assert rig.tick().phase == "HANDOVER"
    assert rig.tick().phase == "EXITED"
    rig = Rig()
    rig.triggered()
    rig.mode = "OFFBOARD"
    assert rig.tick().arm
    rig.rc[6] = 1024
    assert rig.tick().request_mode == "POSCTL"
    rig.mode = "POSCTL"
    rig.armed = True  # Previously sent arm request completes late.
    out = rig.tick()
    assert out.phase == "EXITED" and out.target is None and not out.arm


def test_publisher_stall_after_ready_never_arms_on_next_high():
    rig = Rig()
    rig.ready()
    for _ in range(5):
        rig.tick(publish=False)
    rig.rc[6] = 2003
    out = rig.tick(publish=False)
    assert out.phase == "EXITED" and not out.arm


def test_operator_stop_uses_same_bounded_handover_and_never_disarms():
    rig = Rig()
    rig.active()
    rig.controller.request_exit("operator_stop")
    out = rig.tick()
    assert out.request_mode == "POSCTL" and not out.arm
    rig.mode = "POSCTL"
    assert rig.tick().target is None


def test_cancel_during_mode_switch_requests_position_despite_old_position_feedback():
    rig = Rig()
    rig.triggered()
    rig.rc[6] = 1024
    out = rig.tick()
    assert out.phase == "HANDOVER" and out.request_mode == "POSCTL"
    assert not out.handover_confirmed
    assert rig.tick().handover_confirmed


def test_late_offboard_after_exit_is_rejected_without_resuming_targets_or_arming():
    rig = Rig()
    rig.triggered()
    rig.rc[6] = 1024
    rig.tick()
    rig.tick()
    rig.mode = "OFFBOARD"
    rig.rc[6] = 2003
    out = rig.tick()
    assert out.phase == "EXITED" and out.target is None and not out.arm
    assert out.request_mode == "POSCTL" and not out.handover_confirmed


@pytest.mark.parametrize("kwargs", [
    {"height_m": 0.0}, {"height_m": math.nan}, {"climb_mps": -0.1},
    {"prestream_s": 0.5}, {"setpoint_hz": 1.0}, {"handover_timeout_s": math.inf},
])
def test_bad_config_is_rejected_before_any_connection(kwargs):
    with pytest.raises(ValueError):
        HoverConfig(**kwargs)
