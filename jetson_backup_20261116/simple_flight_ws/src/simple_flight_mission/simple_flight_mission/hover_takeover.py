"""RC intent detection for a future hover controller, with no flight I/O.

An accepted trigger is an intent, not permission to arm or publish targets.
The ROS adapter must enforce all vehicle checks and confirm actual OFFBOARD
feedback. These classes never connect to ROS, change modes, or arm motors.
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class TakeoverConfig:
    swa_channel: int = 7
    throttle_channel: int = 3
    swa_low_pwm: int = 1200
    swa_high_pwm: int = 1800
    throttle_low_pwm: int = 1100
    throttle_raise_pwm: int = 1400
    swa_low_hold_s: float = 0.2
    raise_hold_s: float = 0.2
    throttle_low_hold_s: float = 0.1
    rc_timeout_s: float = 0.3

    def __post_init__(self):
        for value in (self.swa_channel, self.throttle_channel):
            if type(value) is not int or not 1 <= value <= 18:
                raise ValueError("RC channels must be integers from 1 to 18")
        if self.swa_channel == self.throttle_channel:
            raise ValueError("SWA and throttle must use different channels")
        for value in (self.swa_low_pwm, self.swa_high_pwm,
                      self.throttle_low_pwm, self.throttle_raise_pwm):
            if type(value) is not int or not 900 <= value <= 2100:
                raise ValueError("PWM thresholds must be integers in 900..2100")
        if self.swa_low_pwm >= self.swa_high_pwm:
            raise ValueError("SWA thresholds must leave a middle band")
        if self.throttle_low_pwm >= self.throttle_raise_pwm:
            raise ValueError("Throttle thresholds must leave a middle band")
        for value in (self.swa_low_hold_s, self.raise_hold_s,
                      self.throttle_low_hold_s, self.rc_timeout_s):
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("Durations must be finite and positive")


@dataclass(frozen=True)
class Decision:
    phase: str
    action: str = "none"
    reason: Optional[str] = None


def _time(value):
    if (not isinstance(value, (int, float)) or isinstance(value, bool)
            or not math.isfinite(value) or value < 0):
        raise ValueError("Monotonic time must be finite and nonnegative")
    return float(value)


def _pwm(channels, number):
    if channels is None or len(channels) < number:
        return None
    value = channels[number - 1]
    return value if type(value) is int and 900 <= value <= 2100 else None


class HoverTakeover:
    """One task's trigger/exit latch; construct a new instance for a new task.

    Call observe only for newly received RC frames. Call poll from the main
    loop even when no frames arrive. entry_ready is the adapter's complete
    preflight verdict; it is deliberately not derived from PWM values.
    """

    def __init__(self, config=None, notebook=False):
        self.config = config if config is not None else TakeoverConfig()
        self.phase = "WAIT_LOW"
        self.last_received_at = None
        self._low_since = None
        self._raise_since = None
        self._throttle_low_since = None
        self._raise_qualified = False
        self.exit_reason = None
        self.notebook = notebook
        self.notebook_requested = False
        self.start_allowed = True
        self.swa_high_seen = False

    def request_notebook_start(self):
        if not self.notebook or self.phase == "EXITED":
            raise RuntimeError("Notebook start is unavailable")
        self.notebook_requested = True

    def _decision(self, action="none", reason=None):
        return Decision(self.phase, action, reason)

    def _reset_waiting(self):
        self.phase = "WAIT_LOW"
        self._low_since = None

    def _exit(self, reason):
        if self.phase == "EXITED":
            return self._decision()
        self.phase = "EXITED"
        self.exit_reason = reason
        return self._decision("exit", reason)

    def poll(self, now):
        now = _time(now)
        if self.phase == "EXITED":
            return self._decision()
        if (self.last_received_at is not None
                and now - self.last_received_at > self.config.rc_timeout_s):
            if self.phase in ("REQUESTED", "ACTIVE"):
                return self._exit("rc_timeout")
            self._reset_waiting()
        return self._decision()

    def observe(self, channels, received_at, entry_ready=False):
        now = _time(received_at)
        entry_ready = entry_ready is True and self.start_allowed
        if self.phase == "EXITED":
            return self._decision()
        # Replayed/duplicate callbacks cannot qualify an action or overwrite
        # the last fresh input. The adapter must also reject stale wire data.
        if self.last_received_at is not None and now <= self.last_received_at:
            return self._decision()
        expiration = self.poll(now)
        if expiration.action == "exit":
            return expiration
        self.last_received_at = now
        swa = _pwm(channels, self.config.swa_channel)
        throttle = _pwm(channels, self.config.throttle_channel)
        if swa is None or throttle is None:
            if self.phase in ("REQUESTED", "ACTIVE"):
                return self._exit("invalid_rc")
            self._reset_waiting()
            return self._decision()

        if self.phase in ("REQUESTED", "ACTIVE"):
            if swa >= self.config.swa_high_pwm:
                self.swa_high_seen = True
            if swa <= self.config.swa_low_pwm and (not self.notebook or self.swa_high_seen):
                return self._exit("swa_low")
            if self.phase == "ACTIVE":
                return self._observe_throttle(throttle, now)
            return self._decision()

        if (self.notebook and self.notebook_requested and self.phase == "WAIT_HIGH"
                and entry_ready is True and swa <= self.config.swa_low_pwm
                and throttle <= self.config.throttle_low_pwm):
            self.notebook_requested = False
            self.phase = "REQUESTED"
            return self._decision("start", "explicit_notebook_start")

        if swa <= self.config.swa_low_pwm:
            if self._low_since is None:
                self._low_since = now
            if now - self._low_since + 1e-9 >= self.config.swa_low_hold_s:
                self.phase = "WAIT_HIGH"
        elif self.phase == "WAIT_HIGH" and swa >= self.config.swa_high_pwm:
            if entry_ready is not True or throttle > self.config.throttle_low_pwm:
                # A rejected edge is consumed. Recovery while still high may
                # not unexpectedly start a task.
                self._reset_waiting()
            else:
                self.phase = "REQUESTED"
                return self._decision("start", "swa_low_then_high")
        elif self.phase == "WAIT_LOW":
            self._low_since = None
        return self._decision()

    def confirm_offboard(self, now):
        """Adapter calls this after actual, fresh OFFBOARD telemetry only."""
        now = _time(now)
        if self.phase in ("EXITED", "ACTIVE"):
            return self._decision()
        if self.phase != "REQUESTED":
            raise RuntimeError("OFFBOARD cannot be accepted without a trigger")
        if now < self.last_received_at:
            raise ValueError("OFFBOARD confirmation predates the RC frame")
        expiration = self.poll(now)
        if expiration.action == "exit":
            return expiration
        self.phase = "ACTIVE"
        self._raise_since = None
        self._throttle_low_since = None
        self._raise_qualified = False
        return self._decision()

    def _observe_throttle(self, pwm, now):
        if not self._raise_qualified:
            if pwm >= self.config.throttle_raise_pwm:
                if self._raise_since is None:
                    self._raise_since = now
                if now - self._raise_since + 1e-9 >= self.config.raise_hold_s:
                    self._raise_qualified = True
            else:
                self._raise_since = None
        if self._raise_qualified and pwm <= self.config.throttle_low_pwm:
            if self._throttle_low_since is None:
                self._throttle_low_since = now
            if now - self._throttle_low_since + 1e-9 >= self.config.throttle_low_hold_s:
                return self._exit("throttle_raise_then_low")
        else:
            self._throttle_low_since = None
        return self._decision()
