"""Configuration and route expansion for the simple waypoint mission."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class MissionConfig:
    route: tuple[tuple[float, float], ...]
    max_speed_mps: float = 0.3
    arrival_radius_m: float = 0.2
    arrival_speed_mps: float = 0.1
    hold_s: float = 2.0
    segment_timeout_s: float = 30.0
    max_position_age_s: float = 0.3
    max_position_jump_m: float = 0.5
    max_horizontal_speed_mps: float = 0.8
    min_height_m: float = 0.8
    max_height_m: float = 1.5
    setpoint_hz: float = 20.0
    offboard_prewarm_s: float = 2.0
    takeover_threshold: float = 0.2
    takeover_hold_s: float = 0.3
    throttle_drop_pwm: float = 150.0
    startup_stable_s: float = 3.0
    rc_takeover_channel: int = 5
    rc_takeover_low_pwm: int = 1200
    rc_takeover_high_pwm: int = 1800

    @classmethod
    def from_mapping(cls, mapping):
        raw_route = mapping.get("route", [])
        if len(raw_route) != 3:
            raise ValueError("route must contain exactly 3 incremental actions")
        route = []
        for action in raw_route:
            forward = float(action.get("forward_m", 0.0))
            left = float(action.get("left_m", 0.0))
            up = float(action.get("up_m", 0.0))
            if not all(math.isfinite(value) for value in (forward, left, up)):
                raise ValueError("route actions must be finite")
            if abs(up) > 1e-9:
                raise ValueError("route z/up_m must be zero")
            route.append((forward, left))

        limits = dict(mapping.get("limits") or {})
        values = {
            "max_speed_mps": 0.3,
            "arrival_radius_m": 0.2,
            "arrival_speed_mps": 0.1,
            "hold_s": 2.0,
            "segment_timeout_s": 30.0,
            "max_position_age_s": 0.3,
            "max_position_jump_m": 0.5,
            "max_horizontal_speed_mps": 0.8,
            "min_height_m": 0.8,
            "max_height_m": 1.5,
            "setpoint_hz": 20.0,
            "offboard_prewarm_s": 2.0,
            "takeover_threshold": 0.2,
            "takeover_hold_s": 0.3,
            "throttle_drop_pwm": 150.0,
            "startup_stable_s": 3.0,
        }
        for key in values:
            if key in limits:
                values[key] = float(limits[key])
            if not math.isfinite(values[key]) or values[key] <= 0:
                raise ValueError(f"{key} must be positive and finite")
        if values["min_height_m"] >= values["max_height_m"]:
            raise ValueError("min_height_m must be below max_height_m")
        rc = dict(mapping.get("rc_takeover") or {})
        channel = int(rc.get("channel", 5))
        low_pwm = int(rc.get("low_pwm", 1200))
        high_pwm = int(rc.get("high_pwm", 1800))
        if channel < 1 or low_pwm < 800 or high_pwm > 2200 or low_pwm >= high_pwm:
            raise ValueError("invalid rc_takeover thresholds")
        return cls(route=tuple(route), rc_takeover_channel=channel,
                   rc_takeover_low_pwm=low_pwm,
                   rc_takeover_high_pwm=high_pwm, **values)

    def relative_points(self):
        """Expand incremental forward/left moves and append the origin."""
        forward = left = 0.0
        points = []
        for delta_forward, delta_left in self.route:
            forward += delta_forward
            left += delta_left
            points.append((forward, left))
        points.append((0.0, 0.0))
        return points


def load_config(path):
    path = Path(path)
    with path.open("r", encoding="utf-8") as stream:
        return MissionConfig.from_mapping(yaml.safe_load(stream) or {})
