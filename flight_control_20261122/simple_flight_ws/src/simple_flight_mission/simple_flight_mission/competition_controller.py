"""Competition orchestration on the existing guarded route controller."""
from dataclasses import replace
import math

from simple_flight_payload.door import build_command, validate_output
from .hover_takeover import HoverTakeover
from .hover_controller import _finite, _fresh
from .route_controller import RouteController, ACTIVE_PHASES
from .visual_task import VisualTask


class CompetitionController(RouteController):
    def __init__(self, mode, route, doors, start_trigger="notebook", require_enter=False, camera_alignment=None):
        if any(point.stage=="task_zone_2" for point in route.points) and not route.search_xy:
            raise ValueError("人员任务缺少按点名配置的搜索终点")
        super().__init__(mode, replace(route, mission_timeout_s=180.))
        if require_enter and (mode != "execute" or start_trigger != "notebook"):
            raise ValueError("Enter确认仅用于笔记本执行入口")
        self.require_enter = require_enter
        self.enter_confirmed = not require_enter
        self.doors = doors
        self.start_trigger = start_trigger
        self.gate = HoverTakeover(notebook=start_trigger == "notebook")
        self.gate.start_allowed = self.enter_confirmed
        if mode == "execute" and start_trigger == "notebook" and self.enter_confirmed:
            self.gate.request_notebook_start()
        self.vision_data = None
        self.payload_initialized = False
        self.task = None
        self.completed_tasks = set()
        self.jacobian = None
        self.camera_alignment = camera_alignment
        self.payload_ack = None
        self.payload_action = None
        self.visual_error = None
        self.stream_id = None
        self.return_reserve_s = None
        self.competition_started_at = None
        self.landed_disarm_since = None
        self.disarm_requested = False
        self.disarm_ack_accepted = None
        self._landed_disarm_pending = False

    def _takeoff_ready(self, vehicle, now):
        """Competition departure uses current feedback, without a hover timer."""
        p, c = vehicle.pose.value, self.route_config
        return (self._airborne_seen
                and abs(p[2]-self.origin[2]-c.height_m) <= c.arrival_z_m
                and math.dist(p[:2], self.origin[:2]) <= c.arrival_xy_m
                and abs(math.remainder(p[3]-self.origin[3], 2*math.pi)) <= math.radians(5)
                and abs(math.remainder(self.external_heading.value-self.origin[3], 2*math.pi)) <= math.radians(5))

    def _complete_takeoff(self, now):
        out = super()._complete_takeoff(now)
        self.point_index = 0
        self._start_segment(self.world_route[0], now)
        return replace(out, phase=self.phase, target=self.segment_start, point_index=self.point_index)

    def _at_task_zone(self):
        return (self.phase == "POINT_HOLD" and not self.returning
                and self.point_index is not None
                and self.route_config.points[self.point_index].stage.startswith("task_zone"))

    def _hold_complete(self, now, hold_s):
        # A scoring waypoint never departs mid-task, and its dwell is timed
        # from the moment the task actually finished.
        if self._at_task_zone() and self.route_config.zone_dwell_after_task:
            return (self.point_index in self.completed_tasks
                    and super()._hold_complete(now, hold_s))
        return super()._hold_complete(now, hold_s)

    def _stable_arrival(self, vehicle, now):
        # Corridor hold_s already supplies the short confirmation interval.
        # Keep checking arrival throughout that hold, without a second timer.
        if (not self.returning and self.point_index is not None
                and (self.phase in ("TRANSIT", "POINT_HOLD")
                     and self.route_config.points[self.point_index].stage.startswith("obstacle_")
                     or self.phase == "POINT_HOLD" and self.point_index in self.completed_tasks)):
            if not self._arrived(vehicle):
                self._arrival_at = self._hold_at = None
                return False
            return True
        return super()._stable_arrival(vehicle, now)

    def confirm_start(self, vehicle, now):
        if (not self.require_enter or self.enter_confirmed or self.phase != "READY"
                or self._ground_reasons(vehicle, now)
                or vehicle.rc.value[6] > self.gate.config.swa_low_pwm
                or self._last_sent is None or now-self._last_sent > self.config.stream_gap_s):
            return False
        self.enter_confirmed = self.gate.start_allowed = True
        self.gate.request_notebook_start()
        self.competition_started_at = now
        return True

    def mission_elapsed(self, now):
        if self.competition_started_at is None:
            return super().mission_elapsed(now)
        end = now if self.mission_ended_at is None else self.mission_ended_at
        return max(0., end - self.competition_started_at)

    def _ground_reasons(self, vehicle, now):
        reasons = super()._ground_reasons(vehicle, now)
        from .visual_task import finite, capture_time
        data = self.vision_data or {}
        age, received = data.get("age_s"), data.get("received_at")
        if (data.get("source") != "camera" or data.get("valid") is not True
                or not finite(age) or not finite(received)
                or not 0 <= age + now - received <= .75
                or not finite(capture_time(data)) or not 0 <= now-capture_time(data) <= .75):
            reasons.append("camera_not_fresh")
        try:
            for door in (1, 2):
                validate_output(build_command(self.doors, door, "close"), vehicle.parameters.value or {})
        except ValueError:
            reasons.append("payload_mapping_or_calibration_invalid")
        if self.mode == "execute" and not self.payload_initialized:
            reasons.append("payload_initial_close_pending")
        return reasons

    def take_payload_action(self):
        action, self.payload_action = self.payload_action, None
        return action

    def landed_disarm_evidence(self, vehicle, now):
        return (self.mode == "execute" and self.phase == "LAND_WAIT" and self.returning
                and self.point_index is None and self._airborne_seen and self.origin is not None
                and _fresh(vehicle.state, now, 1.5) and _fresh(vehicle.landed, now, 1.)
                and _fresh(vehicle.pose, now, .3, True) and _fresh(vehicle.velocity, now, .3, True)
                and (vehicle.state.value or {}).get("connected") is True
                and (vehicle.state.value or {}).get("armed") is True
                and (vehicle.state.value or {}).get("mode") == "OFFBOARD"
                and vehicle.landed.value == 1
                and _finite(vehicle.pose.value, 5) and _finite(vehicle.velocity.value, 3)
                and abs(vehicle.pose.value[2]-self.origin[2]) <= .2
                and math.dist(vehicle.pose.value[:2], self.origin[:2]) <= self.route_config.arrival_xy_m
                and math.sqrt(sum(v*v for v in vehicle.velocity.value)) <= self.route_config.arrival_speed_mps)

    def _update_landed_disarm(self, vehicle, now):
        if not self.landed_disarm_evidence(vehicle, now):
            self.landed_disarm_since = None
            self._landed_disarm_pending = False
            return
        if self.landed_disarm_since is None:
            self.landed_disarm_since = now
        if not self.disarm_requested and now-self.landed_disarm_since >= .5:
            self.disarm_requested = self._landed_disarm_pending = True

    def take_landed_disarm_request(self):
        pending, self._landed_disarm_pending = self._landed_disarm_pending, False
        return True if pending else None

    def _cancel_task(self):
        if self.task and self.task.open_sent and not self.task.close_sent:
            self.task.close_sent = True
            self.payload_action = (self.task.door, "close")

    def _return_budget(self, vehicle):
        points = [tuple(vehicle.pose.value[:2])]
        points.extend(goal[:2] for goal, _ in reversed(self._breadcrumbs))
        distance = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
        params = self.landing_parameters or vehicle.parameters.value or {}
        speed = min(params.get("MPC_LAND_CRWL", .3), params.get("MPC_LAND_SPEED", .3))
        landing = max(0., vehicle.pose.value[2] - self.origin[2]) / max(.05, speed)
        return distance / self.route_config.horizontal_mps + landing + self.route_config.return_hold_s + 15.

    def step(self, vehicle, now):
        if (self.mode == "execute" and self.start_trigger == "notebook" and self.enter_confirmed
                and self.competition_started_at is None):
            # Notebook start includes initial door close, stabilization and
            # prestream time; do not silently add those to the 180s budget.
            self.competition_started_at = now
        data = self.vision_data or {}
        stream = data.get("stream_id")
        if self.stream_id is None and stream:
            self.stream_id = stream
        changed_stream = stream and self.stream_id and stream != self.stream_id
        if (self.phase in ACTIVE_PHASES and self.phase not in ("DESCEND", "LAND_WAIT")
                and not self.returning and self.origin is not None and _finite(vehicle.pose.value, 5)):
            self.return_reserve_s = self._return_budget(vehicle)
            if changed_stream or self.mission_elapsed(now) >= max(0., 180. - self.return_reserve_s):
                self._cancel_task()
                self.visual_error = "vision_process_restarted" if changed_stream else "return_time_reserved"
                self._begin_return(now, "mission_timeout")
        if self.phase == "POINT_HOLD":
            point = self.route_config.points[self.point_index]
            if point.stage.startswith("task_zone"):
                if not self.route_config.zone_dwell_after_task:
                    self._hold_at = (now - point.hold_s if self.point_index in self.completed_tasks
                                     else now)
                elif self.point_index not in self.completed_tasks:
                    # Stay put until the task itself finishes.
                    self._hold_at = now
        out = super().step(vehicle, now)
        self._update_landed_disarm(vehicle, now)
        if out.phase != "POINT_HOLD" or self.returning:
            if out.phase in ("HANDOVER", "EXITED", "RETURN"):
                self._cancel_task()
            return out
        point = self.route_config.points[self.point_index]
        if not point.stage.startswith("task_zone") or self.point_index in self.completed_tasks:
            return out
        door = 1 if point.stage == "task_zone_1" else 2
        return self._task_step(out, door, vehicle, now, point)

    def _task_step(self, out, door, vehicle, now, point):
        if self.task is None or self.task.door != door:
            anchor = self.world_route[self.point_index]
            scans = []
            if door == 2:
                # Use the named opposite end, including its calibrated lateral
                # coordinate. Both directions rejoin this route's waypoint 5.
                yaw = self.origin[3]
                forward,left=self.route_config.search_xy
                scans.append((self.origin[0] + forward * math.cos(yaw) - left * math.sin(yaw),
                              self.origin[1] + forward * math.sin(yaw) + left * math.cos(yaw),
                              anchor[2], anchor[3]))
            if self.jacobian is None and self.camera_alignment is not None:
                from .camera_alignment import world_jacobian
                self.jacobian = world_jacobian(self.camera_alignment, anchor[3],
                                               self.route_config.height_m,
                                               (self.vision_data or {}).get("image_size"))
            self.task = VisualTask(door, anchor, scans, now, self.jacobian,
                                   arrival_xy_m=self.route_config.arrival_xy_m,
                                   mark_hold_s=self.route_config.mark_hold_s,
                                   mark_max_wait_s=self.route_config.mark_max_wait_s)
        ack = None
        if self.payload_ack is not None:
            ack_door, action, accepted = self.payload_ack
            if ack_door == door:
                ack = (action, accepted)
            self.payload_ack = None
        result = self.task.step(vehicle.pose.value, vehicle.velocity.value, now, self.vision_data, ack)
        if result.action:
            self.payload_action = (door, result.action)
        # A task may explicitly invalidate a nonconverging cached matrix.
        self.jacobian = self.task.jacobian
        if result.failed:
            self.visual_error = result.failed
            self._cancel_task()
            if (self.route_config.continue_after_target_not_found
                    and point.stage == "task_zone_1"
                    and result.failed in ("correct_target_not_found", "visual_task_timeout")):
                # Skip the hazardous drop but keep flying the route: the person
                # task and the configured return still run.
                self.completed_tasks.add(self.point_index)
                self.task = None
                self.goal = self.world_route[self.point_index]
                self._last_commanded_target = self.goal
                self._hold_at = None
                return replace(out, target=self.goal)
            self._begin_return(now, "mission_timeout")
            return replace(out, phase=self.phase, target=self.segment_start, point_index=self.point_index)
        self.goal = result.target
        self._last_commanded_target = result.target
        if result.done:
            self.completed_tasks.add(self.point_index)
            self.goal = self.world_route[self.point_index]
            self._hold_at = (None if self.route_config.zone_dwell_after_task
                             else now - point.hold_s)
        return replace(out, target=self.goal)
