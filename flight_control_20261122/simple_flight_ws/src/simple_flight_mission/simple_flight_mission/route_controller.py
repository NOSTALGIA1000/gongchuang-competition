"""Configurable fixed-heading route; reuse qualified hover startup and takeover.

No hardware I/O or disarm request exists here. PX4 must confirm landing and
perform its own automatic disarm. Slow descent compatibility is a start guard.
"""
from dataclasses import dataclass
import math
from typing import Optional

from .hover_controller import HoverController, Observation, Output, _finite, _fresh
from .route_config import RouteConfig, _number


@dataclass(frozen=True)
class RouteOutput(Output):
    velocity_z: Optional[float] = None  # ENU m/s. Ignore position Z when set.
    point_index: Optional[int] = None  # Zero based; status also supplies one based.


ACTIVE_PHASES = frozenset(("ORIGIN_HOLD", "TRANSIT", "POINT_HOLD", "RETURN",
                           "RETURN_HOLD", "DESCEND", "LAND_WAIT"))
LANDING_PARAMETER_NAMES = ("LNDMC_Z_VEL_MAX", "COM_DISARM_LAND", "MPC_LAND_SPEED",
                           "MPC_LAND_CRWL", "MPC_LAND_ALT3", "MPC_Z_VEL_MAX_DN")
_KEEP_POINT_INDEX = object()


def distance_to_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx + dy*dy
    u = 0.0 if length2 == 0 else max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length2))
    return math.hypot(p[0]-a[0]-u*dx, p[1]-a[1]-u*dy)


class RouteController(HoverController):
    def __init__(self, mode="check", config=None):
        self.route_config = config or RouteConfig()
        super().__init__(mode, self.route_config.core_config())
        self.point_index = None
        self.world_route = ()
        self.goal = None
        self.segment_start = None
        self.progress = 0.0
        self._arrival_at = self._hold_at = None
        self._landing_at = None
        self.landing_parameters = None
        self._crawl_descent = False
        self.returning = False
        self.return_cursor = None
        self.mission_started_at = None
        self.mission_ended_at = None
        self.return_reason = None
        self._last_commanded_target = None
        self._breadcrumbs = []
        self._return_points = ()
        self.target_transport = Observation()
        self.external_heading = Observation()
        self.mission_completed = False

    def note_target_transport(self, now, ready):
        self.target_transport = Observation(ready is True, now)

    def heading_errors(self, vehicle):
        if not _number(self.external_heading.value) or not _finite(vehicle.pose.value, 5):
            return None
        yaw = self.external_heading.value
        reference = abs(math.remainder(yaw-vehicle.pose.value[3], 2*math.pi))
        locked = None if self.origin is None else abs(math.remainder(yaw-self.origin[3], 2*math.pi))
        return reference, locked

    @staticmethod
    def _landing_speeds(params):
        names = ("MPC_LAND_SPEED", "MPC_LAND_CRWL", "MPC_LAND_ALT3", "MPC_Z_VEL_MAX_DN")
        if not all(_number(params.get(name)) for name in names):
            return None
        normal, crawl, height, limit = (params[name] for name in names)
        if not (0 < crawl <= normal and limit > 0 and height >= 0):
            return None
        # Never silently clamp the FCU's settings to a local task preference.
        # This entry refuses effective descent rates outside its tested envelope.
        if min(normal, limit) > 1.0:
            return None
        return min(normal, limit), min(crawl, limit)

    def _descent_velocity(self, vehicle):
        params = self.landing_parameters
        # This flat-floor task applies the FCU's crawl-height setting to the
        # measured height relative to its locked ground origin. It does not
        # switch to AUTO.LAND or require a separate rangefinder measurement.
        if vehicle.pose.value[2]-self.origin[2] <= params["MPC_LAND_ALT3"]:
            self._crawl_descent = True
        normal, crawl = self._landing_speeds(params)
        return -(crawl if self._crawl_descent else normal)

    def _base_reasons(self, v, now, ground=False):
        reasons = super()._base_reasons(v, now, ground)
        if not _fresh(self.target_transport, now, 1.5):
            reasons.append("mavros_raw_receiver_stale")
        elif self.target_transport.value is not True:
            reasons.append("mavros_raw_receiver_unavailable")
        if not _fresh(self.external_heading, now, .3, True):
            reasons.append("external_heading_stale")
        elif not _number(self.external_heading.value):
            reasons.append("external_heading_invalid")
        else:
            errors = self.heading_errors(v)
            if errors is not None:
                if errors[0] > math.radians(10):
                    reasons.append("heading_reference_disagreement")
                if (errors[1] is not None and errors[1] > math.radians(10)
                        and self.phase not in ("CHECK", "WAITING", "HANDOVER", "EXITED")):
                    reasons.append("heading_target_drift")
        if ("velocity_limit" in reasons and self.phase in ("DESCEND", "LAND_WAIT")
                and self.landing_parameters is not None and _finite(v.velocity.value, 3)):
            # Replace ONLY the qualified hover's symmetric vertical limit during
            # FCU-rate descent. Keep its horizontal and upward velocity checks.
            normal, _ = self._landing_speeds(self.landing_parameters)
            # Keep room to decelerate from the normal rate after commanding
            # crawl. Changing a target cannot instantaneously change velocity.
            rate = normal
            vx, vy, vz = v.velocity.value
            if (math.hypot(vx, vy) <= self.config.max_horizontal_speed_mps
                    and -max(self.config.max_vertical_speed_mps, rate+.15) <= vz <= self.config.max_vertical_speed_mps):
                reasons.remove("velocity_limit")
        if (self.origin is not None and self.phase not in ACTIVE_PHASES
                and _finite(v.pose.value, 5)
                and math.dist(v.pose.value[:2], self.origin[:2]) > self.route_config.path_margin_m):
            reasons.append("route_corridor")
        params = v.parameters.value or {}
        # This entry is for indoor LiDAR localization. GNSS aiding can change
        # the horizontal reference even while external position remains fused.
        gps_ctrl = params.get("EKF2_GPS_CTRL")
        if not _number(gps_ctrl) or gps_ctrl != 0:
            reasons.append("indoor_gnss_fusion_not_disabled")
        speeds = self._landing_speeds(params)
        if speeds is None:
            reasons.append("landing_speed_parameters_invalid")
        if (self.landing_parameters is not None
                and any(params.get(name) != value for name, value in self.landing_parameters.items())):
            reasons.append("landing_parameters_changed")
        threshold, disarm = params.get("LNDMC_Z_VEL_MAX"), params.get("COM_DISARM_LAND")
        if not _number(threshold) or not 0 < threshold <= .5:
            reasons.append("landing_detector_parameter_invalid")
        elif speeds is not None and min(speeds) < 1.1*threshold + .005:
            # PX4 v1.16 MulticopterLandDetector requires a finite descent
            # velocity setpoint >= 1.1*LNDMC_Z_VEL_MAX before ground contact.
            reasons.append("landing_descent_incompatible")
        if not _number(disarm) or not 0 < disarm <= 5:
            reasons.append("landing_auto_disarm_invalid")
        return reasons

    def _route_out(self, velocity_z=None):
        return RouteOutput(self.phase, self.goal, velocity_z=velocity_z, point_index=self.point_index)

    def _phase(self, phase, now):
        self.phase, self._phase_since = phase, now
        self._arrival_at = self._hold_at = None

    def _arrived(self, v):
        c, p = self.route_config, v.pose.value
        return (math.dist(p[:2], self.goal[:2]) <= c.arrival_xy_m
                and abs(p[2]-self.goal[2]) <= c.arrival_z_m
                and _number(self.external_heading.value)
                and abs(math.remainder(self.external_heading.value-self.goal[3], 2*math.pi)) <= math.radians(5)
                and abs(math.remainder(p[3]-self.goal[3], 2*math.pi)) <= math.radians(5)
                and math.sqrt(sum(q*q for q in v.velocity.value)) <= c.arrival_speed_mps)

    def _stable_arrival(self, v, now):
        if not self._arrived(v):
            self._arrival_at = self._hold_at = None
            return False
        if self._arrival_at is None:
            self._arrival_at = now
        return now-self._arrival_at + 1e-9 >= self.route_config.arrival_stable_s

    def _hold_complete(self, now, hold_s):
        return self._hold_at is not None and now-self._hold_at + 1e-9 >= hold_s

    def _target_lead(self, along, length):
        """Setpoint lead, optionally tapered into a task-zone waypoint."""
        c = self.route_config
        if (c.zone_approach_taper is None or self.returning
                or self.point_index is None
                or not c.points[self.point_index].stage.startswith("task_zone")):
            return c.target_lead_m
        # With position-only OFFBOARD control the steady travel speed is the
        # FCU position gain times this lead, so shrinking the lead is what
        # brakes. A constant ratio gives a smooth, purely proportional tail;
        # the floor keeps the setpoint closing instead of stalling short.
        remaining = max(0., length-along)
        return min(c.target_lead_m, max(c.zone_approach_min_lead_m,
                                        remaining*c.zone_approach_taper))

    def _complete_takeoff(self, now):
        self.world_route = self.route_config.world_points(self.origin)
        self.goal = (*self.origin[:2], self.origin[2]+self.route_config.height_m, self.origin[3])
        self._breadcrumbs = [(self.goal, None)]
        self._phase("ORIGIN_HOLD", now)
        return self._route_out()

    def _start_segment(self, goal, now, returning=False, point_index=_KEEP_POINT_INDEX, start=None):
        self.segment_start, self.goal, self.progress = self.goal if start is None else start, goal, 0.0
        if point_index is not _KEEP_POINT_INDEX:
            self.point_index = point_index
        self._phase("RETURN" if returning else "TRANSIT", now)

    def mission_elapsed(self, now):
        if self.mission_started_at is None:
            return 0.0
        end = now if self.mission_ended_at is None else self.mission_ended_at
        return max(0.0, end-self.mission_started_at)

    @staticmethod
    def _same_place(a, b):
        return math.dist(a[:3], b[:3]) < 1e-8

    def _record_reached(self):
        # Erase completed loops: point 4 physically revisits point 2. This
        # leaves only the corridor needed to get home from the current leg.
        for cursor, (goal, _) in enumerate(self._breadcrumbs):
            if self._same_place(goal, self.goal):
                self._breadcrumbs = self._breadcrumbs[:cursor]
                break
        self._breadcrumbs.append((self.goal, self.point_index))

    def _return_index(self, goal, index):
        # Give repeated physical points the configured return-point identity.
        for candidate in self.route_config.return_indices:
            if self._same_place(goal, self.world_route[candidate]):
                return candidate
        return index

    def _begin_return(self, now, reason="route_complete", start=None):
        self.returning = True
        self.return_reason = reason
        self.return_cursor = 0
        home = (*self.origin[:2], self.origin[2]+self.route_config.height_m, self.origin[3])
        if start is None:
            start = self._last_commanded_target or self.goal
        if reason == "mission_timeout":
            points = list(reversed(self._breadcrumbs))
            if len(points) > 1 and self._same_place(points[0][0], start):
                points.pop(0)
            self._return_points = tuple((goal, self._return_index(goal, index))
                                        for goal, index in points) or ((home, None),)
        else:
            self._return_points = tuple((self.world_route[index], index)
                                        for index in self.route_config.return_indices) + ((home, None),)
        goal, index = self._return_points[0]
        self._start_segment(goal, now, returning=True, point_index=index, start=start)

    def step(self, v, now):
        out = self._step(v, now)
        if out.target is not None:
            self._last_commanded_target = out.target
        if (self.mission_started_at is not None and self.mission_ended_at is None
                and self._airborne_seen and _fresh(v.state, now, 1.5)
                and _fresh(v.landed, now, 1.5) and v.landed.value == 1
                and not (v.state.value or {}).get("armed", True)):
            self.mission_ended_at = now
        return out

    def _step(self, v, now):
        previous = self._last_step
        if self.phase not in ACTIVE_PHASES:
            out = super().step(v, now)
            if out.phase == "SWITCHING" and self.mode == "execute" and self.mission_started_at is None:
                self.mission_started_at = now
            if out.phase == "SWITCHING" and self.landing_parameters is None:
                params = v.parameters.value
                self.landing_parameters = {name: params[name] for name in LANDING_PARAMETER_NAMES}
            if (self.mission_started_at is not None and not self.returning
                    and self.mission_elapsed(now) >= self.route_config.mission_timeout_s
                    and out.phase not in ("HANDOVER", "EXITED")):
                if not self._airborne_seen:
                    return self._begin_exit(v, now, "mission_timeout_before_takeoff")
                # Timeout while still climbing: descend at home from the
                # current height, rather than completing the climb first.
                self.returning, self.return_reason = True, "mission_timeout"
                self.point_index = None
                self.goal = (*self.origin[:2], v.pose.value[2], self.origin[3])
                self._phase("DESCEND", now)
                self._landing_at = now
                return self._route_out(velocity_z=self._descent_velocity(v))
            if out.phase == "HOVER" and self.mode == "execute":
                return self._complete_takeoff(now)
            return out

        if not math.isfinite(now) or now < 0 or (previous is not None and now < previous):
            raise ValueError("Invalid controller monotonic time")
        self._last_step = now
        if self._pending_exit is not None:
            return self._begin_exit(v, now, self._pending_exit)
        reasons = self._base_reasons(v, now)
        p = v.pose.value
        if _finite(p, 5):
            if not self.origin[2]-.3 <= p[2] <= self.origin[2]+self.route_config.height_m+.3:
                reasons.append("height_bound")
            a = self.segment_start if self.phase in ("TRANSIT", "RETURN") else self.goal
            if distance_to_segment(p, a, self.goal) > self.route_config.path_margin_m:
                reasons.append("route_corridor")
            if _fresh(v.pose, now, .3, True) and v.pose.at != self._last_pose_at:
                if self._last_pose is not None and math.dist(p[:3], self._last_pose[:3]) > self.config.max_pose_jump_m:
                    reasons.append("pose_jump")
                self._last_pose, self._last_pose_at = p, v.pose.at
        if reasons:
            return self._begin_exit(v, now, ";".join(reasons))
        if self._last_sent is None or now-self._last_sent > self.config.stream_gap_s:
            return self._begin_exit(v, now, "target_stream_stalled")
        decision = self.gate.observe(v.rc.value, v.rc.at, entry_ready=False)
        timeout = self.gate.poll(now)
        if decision.action == "exit" or timeout.action == "exit":
            return self._begin_exit(v, now, decision.reason or timeout.reason)
        state = v.state.value
        if state["mode"] != "OFFBOARD":
            return self._begin_exit(v, now, "manual_mode_selected")
        landing = self.phase in ("DESCEND", "LAND_WAIT")
        if not state.get("armed"):
            if landing and self._airborne_seen and v.landed.value == 1 and abs(p[2]-self.origin[2]) <= .2:
                if math.dist(p[:2], self.origin[:2]) > self.route_config.arrival_xy_m:
                    return self._begin_exit(v, now, "landed_outside_target")
                self.mission_completed = True
                return self._begin_exit(v, now, "mission_complete")
            return self._begin_exit(v, now, "vehicle_disarmed")
        self._airborne_seen |= v.landed.value == 2
        if v.landed.value == 1 and self._airborne_seen and not landing:
            return self._begin_exit(v, now, "landed_during_task")

        c = self.route_config
        if self.phase in ("TRANSIT", "RETURN"):
            segment_limit = (math.dist(self.segment_start[:2], self.goal[:2]) / c.horizontal_mps
                             + c.arrival_stable_s + c.transit_timeout_extra_s)
            if now-self._phase_since > segment_limit:
                reason = "return_segment_timeout" if self.returning else "transit_segment_timeout"
                return self._begin_exit(v, now, reason)
        elif self.phase == "RETURN_HOLD":
            hold_limit = (c.return_hold_s if self.point_index is None
                          else c.points[self.point_index].hold_s)
            if now-self._phase_since > hold_limit+c.arrival_stable_s+c.transit_timeout_extra_s:
                return self._begin_exit(v, now, "return_hold_timeout")
        if landing:
            slow_speed = min(self._landing_speeds(self.landing_parameters))
            limit = c.height_m/slow_speed + c.landing_timeout_extra_s
            if now-self._landing_at >= limit:
                return self._begin_exit(v, now, "landing_timeout")
            if v.landed.value == 1:
                if abs(p[2]-self.origin[2]) > .2:
                    return self._begin_exit(v, now, "landed_above_origin")
                if self.phase != "LAND_WAIT":
                    self._phase("LAND_WAIT", now)
            elif self.phase == "LAND_WAIT":
                self._phase("DESCEND", now)
            return self._route_out(velocity_z=self._descent_velocity(v))

        if self.mission_started_at is not None and not self.returning and self.phase not in ("DESCEND", "LAND_WAIT"):
            if now-self.mission_started_at >= c.mission_timeout_s:
                self._begin_return(now, reason="mission_timeout")
                return RouteOutput(self.phase, self.segment_start, point_index=self.point_index)

        if self.phase in ("TRANSIT", "RETURN"):
            length = math.dist(self.segment_start[:2], self.goal[:2])
            if length > 0:
                dx, dy = (self.goal[i]-self.segment_start[i] for i in (0, 1))
                along = ((p[0]-self.segment_start[0])*dx+(p[1]-self.segment_start[1])*dy)/length
                self.progress = min(length, self.progress+c.horizontal_mps*(now-previous),
                                    max(self.progress, along+self._target_lead(along, length)))
                ratio = self.progress/length
                target = (self.segment_start[0]+ratio*dx, self.segment_start[1]+ratio*dy, *self.goal[2:])
            else:
                target = self.goal
            if self.phase == "RETURN" and self.return_cursor+1 < len(self._return_points):
                # Keep the return trajectory moving through corridor points.
                # Start the next leg at the current commanded target so the
                # setpoint remains continuous and the corner is bounded by
                # the configured look-ahead distance.
                if self.progress >= length-1e-9:
                    self.return_cursor += 1
                    goal, next_index = self._return_points[self.return_cursor]
                    self._start_segment(goal, now, returning=True,
                                        point_index=next_index, start=target)
                    return RouteOutput(self.phase, target, point_index=self.point_index)
            if self.progress >= length-1e-9 and self._stable_arrival(v, now):
                returning = self.phase == "RETURN"
                if not returning:
                    self._record_reached()
                self._phase("RETURN_HOLD" if returning else "POINT_HOLD", now)
                # Arrival was already stable for one second; start hold now.
                self._arrival_at = now-c.arrival_stable_s
                self._hold_at = now
                return self._route_out()
            return RouteOutput(self.phase, target, point_index=self.point_index)

        if self.phase == "ORIGIN_HOLD":
            hold_s = c.origin_hold_s
        elif self.phase == "RETURN_HOLD":
            hold_s = c.return_hold_s if self.point_index is None else c.points[self.point_index].hold_s
            if (self.return_reason == "mission_timeout" and self.point_index is not None
                    and c.points[self.point_index].stage.startswith("task_zone")):
                hold_s = 0.0
        else:
            hold_s = c.points[self.point_index].hold_s
        if self._stable_arrival(v, now):
            if self._hold_at is None:
                self._hold_at = now
            if self._hold_complete(now, hold_s):
                if self.phase == "RETURN_HOLD":
                    if self.point_index is None:
                        self._phase("DESCEND", now)
                        self._landing_at = now
                        self._crawl_descent = c.height_m <= self.landing_parameters["MPC_LAND_ALT3"]
                        return self._route_out(velocity_z=self._descent_velocity(v))
                    self.return_cursor += 1
                    goal, next_index = self._return_points[self.return_cursor]
                    self._start_segment(goal, now, returning=True, point_index=next_index)
                    return RouteOutput(self.phase, self.segment_start, point_index=self.point_index)
                if self.phase == "ORIGIN_HOLD":
                    self.point_index = 0
                    self._start_segment(self.world_route[0], now)
                elif not self.returning and self.point_index+1 < len(c.points):
                    self.point_index += 1
                    self._start_segment(self.world_route[self.point_index], now)
                elif c.zone_dwell_after_task and _finite(p, 5):
                    # Leave the scoring waypoint from where the aircraft really
                    # is. Starting the leg at the waypoint would turn any
                    # residual offset into a permanent cross-track bias on the
                    # whole return.
                    self._begin_return(now, start=(*p[:2], *self.goal[2:]))
                else:
                    self._begin_return(now)
                # First target of a new leg remains at the previous point.
                return RouteOutput(self.phase, self.segment_start, point_index=self.point_index)
        return self._route_out()
