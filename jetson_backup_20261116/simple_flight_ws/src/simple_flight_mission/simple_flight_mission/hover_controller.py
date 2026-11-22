"""Deterministic hover policy. All hardware I/O lives in hover_node."""
from dataclasses import dataclass, field
import math
from typing import Any, Optional, Tuple

from .hover_takeover import HoverTakeover


EXPECTED_PARAMETERS = {
    "RC_MAP_OFFB_SW": 7, "RC_MAP_FLTMODE": 5, "RC_MAP_THROTTLE": 3,
    "COM_FLTMODE6": 2, "COM_RC_OVERRIDE": 1, "COM_OBL_RC_ACT": 0,
    "COM_RCL_EXCEPT": 0, "EKF2_EV_CTRL": 11, "EKF2_HGT_REF": 3,
    "RC7_REV": 1.0,
}
PARAMETER_NAMES = tuple(EXPECTED_PARAMETERS) + ("COM_OF_LOSS_T", "RC_OFFB_TH")


@dataclass(frozen=True)
class HoverConfig:
    height_m: float = 1.0
    climb_mps: float = 0.2
    setpoint_hz: float = 20.0
    stable_s: float = 3.0
    prestream_s: float = 2.0
    stream_gap_s: float = 0.15
    mode_timeout_s: float = 4.0
    arm_timeout_s: float = 3.0
    arm_settle_s: float = 0.5
    takeoff_timeout_s: float = 15.0
    handover_timeout_s: float = 1.0
    max_horizontal_m: float = 0.6
    max_height_extra_m: float = 0.3
    max_pose_jump_m: float = 0.5
    max_tilt_rad: float = 0.35
    max_horizontal_speed_mps: float = 0.8
    max_vertical_speed_mps: float = 0.6

    def __post_init__(self):
        for value in self.__dict__.values():
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError("Hover settings must be finite positive numbers")
        if not 0.3 <= self.height_m <= 1.2 or self.climb_mps > 0.3:
            raise ValueError("This stage supports heights 0.3..1.2 m and climb <=0.3 m/s")
        if not 10 <= self.setpoint_hz <= 50 or self.prestream_s < 2:
            raise ValueError("Require 10..50 Hz and at least 2 seconds prestream")
        if self.stream_gap_s <= 1.0 / self.setpoint_hz or self.stream_gap_s > 0.3:
            raise ValueError("Stream gap must exceed one period and be <=0.3 seconds")
        if self.handover_timeout_s > 2.0:
            raise ValueError("Handover target holding must be bounded to <=2 seconds")


@dataclass(frozen=True)
class Observation:
    value: Any = None
    at: Optional[float] = None
    wire_age: Optional[float] = None


@dataclass(frozen=True)
class Vehicle:
    boot_id: str = "unknown"
    state: Observation = field(default_factory=Observation)
    rc: Observation = field(default_factory=Observation)
    pose: Observation = field(default_factory=Observation)
    velocity: Observation = field(default_factory=Observation)
    landed: Observation = field(default_factory=Observation)
    estimator: Observation = field(default_factory=Observation)
    health: Observation = field(default_factory=Observation)
    boot: Observation = field(default_factory=Observation)
    lio: Observation = field(default_factory=Observation)
    vision: Observation = field(default_factory=Observation)
    parameters: Observation = field(default_factory=Observation)
    other_setpoint_publishers: int = 0


@dataclass(frozen=True)
class Output:
    phase: str
    target: Optional[Tuple[float, float, float, float]] = None
    request_mode: Optional[str] = None
    arm: bool = False
    reasons: tuple = ()
    exit_reason: Optional[str] = None
    ready_for_switch: bool = False
    handover_confirmed: bool = False


def _fresh(obs, now, limit, stamped=False):
    if obs.at is None or not math.isfinite(obs.at) or not 0 <= now - obs.at <= limit:
        return False
    if stamped:
        return (obs.wire_age is not None and math.isfinite(obs.wire_age)
                and obs.wire_age >= -0.02
                and -0.02 <= obs.wire_age + now - obs.at <= limit)
    return True


def _finite(values, size):
    return (isinstance(values, (tuple, list)) and len(values) == size
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in values))


def _rc_valid(rc):
    return (isinstance(rc, (tuple, list)) and len(rc) >= 7
            and all(type(rc[i]) is int and 900 <= rc[i] <= 2100 for i in (0, 1, 2, 3, 4, 6)))


class HoverController:
    def __init__(self, mode="check", config=None):
        if mode not in ("check", "mode-test", "execute"):
            raise ValueError("Unknown hover mode")
        self.mode = mode
        self.config = config or HoverConfig()
        self.phase = "CHECK" if mode == "check" else "WAITING"
        self.gate = HoverTakeover()
        self.origin = None
        self._run_id = None
        self._stable_since = None
        self._stable_anchor = None
        self._phase_since = None
        self._stream_since = None
        self._stream_count = 0
        self._last_sent = None
        self._last_mode_request = None
        self._last_pose = None
        self._last_pose_at = None
        self._last_step = None
        self._arm_requested = False
        self._hover_since = None
        self._airborne_seen = False
        self._unclaimed_since = None
        self._pending_exit = None
        self._exit_reason = None
        self._freeze_target = None
        self.handover_confirmed = False

    def request_exit(self, reason="operator_stop"):
        if self.phase not in ("HANDOVER", "EXITED"):
            self._pending_exit = str(reason)

    def command_result(self, command, accepted):
        # An acknowledgement is never used as mode/armed telemetry.
        if not accepted and command in ("OFFBOARD", "arm"):
            self.request_exit(command + "_request_rejected")

    def note_target_sent(self, now):
        if self._last_sent is None or now - self._last_sent > self.config.stream_gap_s:
            self._stream_since = now
            self._stream_count = 0
        self._last_sent = now
        self._stream_count += 1

    def _out(self, target=None, request_mode=None, arm=False, reasons=()):
        return Output(self.phase, target, request_mode, arm, tuple(reasons),
                      self._exit_reason, self.phase == "READY", self.handover_confirmed)

    def _base_reasons(self, v, now, ground=False):
        reasons = []
        for key, limit, stamped in (
                ("state", 1.5, False), ("rc", .3, True), ("pose", .3, True),
                ("velocity", .3, True), ("landed", 1., False),
                ("estimator", 1.5, False), ("health", 1.5, False),
                ("boot", 2.5, False), ("lio", .3, True),
                ("vision", .3, True), ("parameters", 10., False)):
            if not _fresh(getattr(v, key), now, limit, stamped):
                reasons.append(key + "_stale")
        state, health, boot = v.state.value or {}, v.health.value or {}, v.boot.value or {}
        if not state.get("connected"):
            reasons.append("fcu_disconnected")
        # HEARTBEAT.manual_input is mode enablement, not RC receiver health.
        # It normally clears in OFFBOARD; continue requiring actual RC health.
        if not health.get("rc_ready"):
            reasons.append("rc_not_ready")
        if not _rc_valid(v.rc.value):
            reasons.append("rc_invalid")
        if not _finite(v.pose.value, 5) or not _finite(v.velocity.value, 3):
            reasons.append("pose_or_velocity_invalid")
        if v.estimator.value is not True or v.lio.value is not True or v.vision.value is not True:
            reasons.append("position_estimator_invalid")
        if (boot.get("boot_id") != v.boot_id or not boot.get("run_id")
                or boot.get("fault") or not boot.get("localization_active")):
            reasons.append("boot_localization_invalid")
        updated, expires = boot.get("updated_monotonic"), boot.get("valid_until_monotonic")
        if (not isinstance(updated, (int, float)) or not isinstance(expires, (int, float))
                or not math.isfinite(updated) or not math.isfinite(expires)
                or not updated <= now <= expires or expires - updated > 5):
            reasons.append("boot_status_expired")
        if self._run_id is not None and self._run_id != boot.get("run_id"):
            reasons.append("localization_generation_changed")
        params = v.parameters.value or {}
        for name, expected in EXPECTED_PARAMETERS.items():
            if params.get(name) != expected:
                reasons.append("parameter:" + name + "=" + str(params.get(name)) + ";expected=" + str(expected))
        for name, upper in (("COM_OF_LOSS_T", 1.0), ("RC_OFFB_TH", .9)):
            value = params.get(name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= upper:
                reasons.append("parameter:" + name + "_out_of_range")
        if v.other_setpoint_publishers:
            reasons.append("other_setpoint_publisher")
        if _finite(v.pose.value, 5) and abs(v.pose.value[4]) > self.config.max_tilt_rad:
            reasons.append("tilt_limit")
        if _finite(v.velocity.value, 3):
            vx, vy, vz = v.velocity.value
            if math.hypot(vx, vy) > self.config.max_horizontal_speed_mps or abs(vz) > self.config.max_vertical_speed_mps:
                reasons.append("velocity_limit")
        if ground:
            if state.get("armed") or v.landed.value != 1:
                reasons.append("requires_disarmed_on_ground")
            if not health.get("prearm_ready"):
                reasons.append("prearm_not_ready")
        return reasons

    def _ground_reasons(self, v, now):
        reasons = self._base_reasons(v, now, ground=True)
        state = v.state.value or {}
        allowed = ("POSCTL", "OFFBOARD") if self.phase in ("READY", "SWITCHING") else ("POSCTL",)
        if state.get("mode") not in allowed:
            reasons.append("requires_position_mode")
        if state.get("mode") == "POSCTL" and not state.get("manual_input"):
            reasons.append("manual_control_not_enabled")
        if self.phase in ("WAITING", "PRESTREAM") and not (v.boot.value or {}).get("ready_to_arm"):
            reasons.append("boot_not_ready")
        if _rc_valid(v.rc.value):
            if v.rc.value[2] > self.gate.config.throttle_low_pwm:
                reasons.append("throttle_not_low")
            if v.rc.value[4] < 1800:
                reasons.append("main_switch_not_position")
        if _finite(v.velocity.value, 3) and math.sqrt(sum(x*x for x in v.velocity.value)) > .1:
            reasons.append("not_stationary")
        return reasons

    def _begin_exit(self, v, now, reason):
        if self.phase in ("HANDOVER", "EXITED"):
            return self._handover(v, now) if self.phase == "HANDOVER" else self._out()
        self._exit_reason = reason
        self._pending_exit = None
        self._phase_since = now
        self._last_mode_request = None
        state = v.state.value or {}
        if (self.phase != "SWITCHING" and _fresh(v.state, now, 1.5)
                and state.get("mode") != "OFFBOARD"):
            self.phase = "EXITED"
            self.handover_confirmed = True
            return self._out()
        if self.origin is None:
            self.phase = "EXITED"
            return self._out()
        self.phase = "HANDOVER"
        source_fault = any(s in reason for s in ("stale", "invalid", "generation", "pose_jump"))
        if _fresh(v.pose, now, .3, True) and _finite(v.pose.value, 5) and not source_fault:
            self._freeze_target = (*v.pose.value[:3], self.origin[3])
        else:
            self._freeze_target = None
        return self._handover(v, now)

    def _handover(self, v, now):
        state = v.state.value or {}
        if (_fresh(v.state, now, 1.5) and v.state.at > self._phase_since
                and state.get("mode") != "OFFBOARD"):
            self.phase = "EXITED"
            self.handover_confirmed = True
            return self._out()
        if now - self._phase_since >= self.config.handover_timeout_s:
            self.phase = "EXITED"
            return self._out(reasons=("handover_unconfirmed_targets_stopped",))
        request = None
        if self._last_mode_request is None or now - self._last_mode_request >= .2:
            request = "POSCTL"
            self._last_mode_request = now
        # Never extend holding on stale position even during handover.
        target = self._freeze_target if _fresh(v.pose, now, .3, True) else None
        return self._out(target=target, request_mode=request)

    def step(self, v, now):
        if not math.isfinite(now) or now < 0 or (self._last_step is not None and now < self._last_step):
            raise ValueError("Invalid controller monotonic time")
        self._last_step = now
        if self.mode == "check":
            return self._out(reasons=self._ground_reasons(v, now))
        if self.phase == "EXITED":
            fresh_state = _fresh(v.state, now, 1.5)
            was_confirmed = self.handover_confirmed
            if fresh_state:
                self.handover_confirmed = (v.state.value or {}).get("mode") != "OFFBOARD"
            request = None
            if (fresh_state and not self.handover_confirmed and self.origin is not None
                    and (was_confirmed or self._last_mode_request is None or now-self._last_mode_request >= .2)):
                request, self._last_mode_request = "POSCTL", now
            return self._out(request_mode=request)
        if self._pending_exit is not None:
            return self._begin_exit(v, now, self._pending_exit)
        if self.phase == "HANDOVER":
            return self._handover(v, now)

        ground = self.phase in ("WAITING", "PRESTREAM", "READY", "SWITCHING", "MODE_TEST")
        reasons = self._ground_reasons(v, now) if self.phase in ("WAITING", "PRESTREAM", "READY", "SWITCHING") else self._base_reasons(v, now, ground=ground)
        if self.origin is not None and _finite(v.pose.value, 5):
            p = v.pose.value
            if math.hypot(p[0]-self.origin[0], p[1]-self.origin[1]) > self.config.max_horizontal_m:
                reasons.append("horizontal_bound")
            if p[2] - self.origin[2] > self.config.height_m + self.config.max_height_extra_m or p[2] - self.origin[2] < -.3:
                reasons.append("height_bound")
        if _fresh(v.pose, now, .3, True) and _finite(v.pose.value, 5) and v.pose.at != self._last_pose_at:
            if self._last_pose is not None and self.origin is not None and math.dist(v.pose.value[:3], self._last_pose[:3]) > self.config.max_pose_jump_m:
                reasons.append("pose_jump")
            self._last_pose, self._last_pose_at = v.pose.value, v.pose.at
        if reasons:
            if self.phase == "WAITING":
                self._stable_since = self._stable_anchor = None
                self.gate.poll(now)
                return self._out(reasons=reasons)
            return self._begin_exit(v, now, ";".join(reasons))

        state = v.state.value
        if self.phase not in ("WAITING", "PRESTREAM") and (self._last_sent is None or now - self._last_sent > self.config.stream_gap_s):
            return self._begin_exit(v, now, "target_stream_stalled")
        gate_event = self.gate.observe(v.rc.value, v.rc.at, entry_ready=self.phase == "READY")
        timeout = self.gate.poll(now)
        if gate_event.action == "exit" or timeout.action == "exit":
            return self._begin_exit(v, now, gate_event.reason or timeout.reason)

        if self.phase == "WAITING":
            if v.rc.value[6] > self.gate.config.swa_low_pwm:
                self._stable_since = self._stable_anchor = None
                return self._out(reasons=("swa_must_be_low",))
            if self._stable_anchor is None or math.dist(v.pose.value[:3], self._stable_anchor[:3]) > .05:
                self._stable_since, self._stable_anchor = now, v.pose.value
            if now - self._stable_since + 1e-9 >= self.config.stable_s:
                self.origin = tuple(v.pose.value[:4])
                self._run_id = v.boot.value["run_id"]
                self.phase = "PRESTREAM"
            return self._out(target=self.origin if self.phase == "PRESTREAM" else None)

        if self.phase == "PRESTREAM":
            if math.dist(v.pose.value[:3], self.origin[:3]) > .1:
                return self._begin_exit(v, now, "moved_during_prestream")
            if (self._stream_since is not None and now-self._stream_since + 1e-9 >= self.config.prestream_s
                    and self._stream_count >= math.ceil(self.config.prestream_s*self.config.setpoint_hz*.8)):
                self.phase = "READY"
            return self._out(target=self.origin)

        if self.phase == "READY":
            if gate_event.action == "start":
                self.origin = tuple(v.pose.value[:4])
                self.phase, self._phase_since = "SWITCHING", now
                self._last_mode_request = now
                return self._out(target=self.origin, request_mode="OFFBOARD")
            if state["mode"] == "OFFBOARD":
                if self._unclaimed_since is None:
                    self._unclaimed_since = now
                elif now-self._unclaimed_since >= .2:
                    return self._begin_exit(v, now, "offboard_without_switch_trigger")
            return self._out(target=self.origin)

        if self.phase == "SWITCHING":
            if state["mode"] == "OFFBOARD":
                decision = self.gate.confirm_offboard(now)
                if decision.action == "exit":
                    return self._begin_exit(v, now, decision.reason)
                self.phase = "MODE_TEST" if self.mode == "mode-test" else "ARMING"
                self._phase_since = now
                if self.mode == "execute":
                    self._arm_requested = True
                    return self._out(target=self.origin, arm=True)
                return self._out(target=self.origin)
            if now-self._phase_since >= self.config.mode_timeout_s:
                return self._begin_exit(v, now, "offboard_mode_timeout")
            request = None
            if now-self._last_mode_request >= .5:
                request, self._last_mode_request = "OFFBOARD", now
            return self._out(target=self.origin, request_mode=request)

        if state["mode"] != "OFFBOARD":
            return self._begin_exit(v, now, "manual_mode_selected")
        if self.phase == "MODE_TEST":
            return self._out(target=self.origin)
        if self.phase == "ARMING":
            if state.get("armed") and self._arm_requested:
                self.phase, self._phase_since = "TAKEOFF", now
            elif now-self._phase_since >= self.config.arm_timeout_s:
                return self._begin_exit(v, now, "arm_feedback_timeout")
            return self._out(target=self.origin)
        if not state.get("armed"):
            return self._begin_exit(v, now, "vehicle_disarmed")
        self._airborne_seen |= v.landed.value == 2
        if self._airborne_seen and v.landed.value == 1:
            return self._begin_exit(v, now, "landed_during_task")
        elapsed = max(0.0, now-self._phase_since-self.config.arm_settle_s)
        rise = min(self.config.height_m, self.config.climb_mps*elapsed)
        z = min(self.origin[2]+rise, v.pose.value[2]+.25)
        target = (self.origin[0], self.origin[1], z, self.origin[3])
        if self.phase == "TAKEOFF":
            if self._takeoff_ready(v, now):
                self.phase = "HOVER"
            if self.phase == "TAKEOFF" and now-self._phase_since >= self.config.takeoff_timeout_s:
                return self._begin_exit(v, now, "takeoff_timeout")
        return self._out(target=target)

    def _takeoff_ready(self, vehicle, now):
        """Default hover entry retains its existing one-second settling rule."""
        if (abs(vehicle.pose.value[2]-self.origin[2]-self.config.height_m) <= .1
                and abs(vehicle.velocity.value[2]) <= .15):
            if self._hover_since is None:
                self._hover_since = now
            else:
                return now-self._hover_since >= 1.0
        else:
            self._hover_since = None
        return False
