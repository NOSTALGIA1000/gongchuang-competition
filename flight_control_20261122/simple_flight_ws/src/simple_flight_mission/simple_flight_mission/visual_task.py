"""Bounded task-zone visual alignment. Pure policy; no hardware or HTTP I/O."""
from dataclasses import dataclass
from collections import deque
import math
from statistics import median


RELEASE_THRESHOLD_PX = 250
# The competition awards an annotated drop only when the marking stays visible
# for more than three seconds. Open the door after that much continuous marked
# video, but never let the sortie hang above a zone waiting for it.
MARK_HOLD_S = 3.0
MARK_GAP_S = .75
MARK_MAX_WAIT_S = 12.0
CALIBRATION_SPEED_MPS = .10
CALIBRATION_SETTLE_S = .10
CALIBRATION_SAMPLE_FRAMES = 1


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def capture_time(snapshot):
    # The mission adapter only accepts loopback HTTP: both processes share
    # the host monotonic clock. Preserve the source timestamp across HTTP.
    return snapshot.get("captured_monotonic_s", snapshot["received_at"]-snapshot["age_s"])


def inverse(matrix):
    (a, b), (c, d) = matrix
    determinant = a * d - b * c
    if not math.isfinite(determinant) or abs(determinant) < 1e-8:
        raise ValueError("视觉标定矩阵不可逆")
    return ((d / determinant, -b / determinant), (-c / determinant, a / determinant))


def multiply(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(2)) for j in range(2)) for i in range(2))


def estimate_jacobian(samples):
    """Estimate image-pixel / actual-world-position Jacobian from three poses."""
    origin, x, y = samples
    positions = tuple(tuple(sample[0][i] - origin[0][i] for sample in (x, y)) for i in range(2))
    pixels = tuple(tuple(sample[1][i] - origin[1][i] for sample in (x, y)) for i in range(2))
    norm = lambda m: math.sqrt(sum(v * v for row in m for v in row))
    position_inverse = inverse(positions)
    # Accuracy comes from measured, independent baselines, not closeness to
    # the nominal 8cm setpoint. Tiny/parallel motion cannot calibrate two axes.
    spans = [math.dist(origin[0], sample[0]) for sample in (x, y)]
    if min(spans) < .04 or norm(positions) * norm(position_inverse) > 8:
        raise ValueError("视觉标定实测位移不足或两方向接近平行")
    matrix = multiply(pixels, position_inverse)
    inv = inverse(matrix)
    if not 30 <= norm(matrix) <= 10000 or norm(matrix) * norm(inv) > 12:
        raise ValueError("视觉位移标定变化过小或病态")
    return matrix


@dataclass(frozen=True)
class Target:
    center: tuple
    captured: float
    sequence: object
    size: tuple


def select_target(snapshot, kind, now, previous=None, diagnostics=None):
    def reject(reason):
        if diagnostics is not None:
            diagnostics["target_rejection"] = reason
        return None

    if diagnostics is not None:
        diagnostics["target_rejection"] = None
    if not isinstance(snapshot, dict) or snapshot.get("source") != "camera":
        return reject("not_live_camera")
    if not all(snapshot.get(k) is True for k in ("valid", "frame_fresh", "enabled")):
        return reject("camera_invalid_or_disabled")
    age, received = snapshot.get("age_s"), snapshot.get("received_at")
    if not finite(age) or not finite(received) or not 0 <= age <= .75 or not 0 <= now - received <= .75:
        return reject("frame_stale")
    captured = capture_time(snapshot)
    if not finite(captured) or not 0 <= now - captured <= .75:
        return reject("frame_stale")
    size = snapshot.get("image_size")
    if not isinstance(size, (list, tuple)) or len(size) != 2 or not all(finite(v) and v > 0 for v in size):
        return reject("image_size_invalid")
    detections = snapshot.get("detections")
    if not isinstance(detections, list) or len(detections) > 100:
        return reject("detections_invalid")
    candidates = []
    edge_person = False
    for detection in detections:
        if not isinstance(detection, dict) or type(detection.get("class_id")) is not int or detection["class_id"] != kind:
            continue
        if detection.get("name") != {0: "Person", 2: "Hazardous"}[kind]:
            continue
        confidence, box = detection.get("confidence"), detection.get("box")
        if not finite(confidence) or not .4 <= confidence <= 1:
            continue
        if not isinstance(box, (list, tuple)) or len(box) != 4 or not all(finite(v) for v in box):
            continue
        x1, y1, x2, y2 = box
        if not (0 <= x1 < x2 <= size[0] and 0 <= y1 < y2 <= size[1]):
            continue
        center = ((x1 + x2) / 2, (y1 + y2) / 2)
        distance = math.dist(center, previous) if previous is not None else 0.
        if previous is not None and distance > max(size) * .3:
            continue
        # Detector boxes are clipped to the image. Their centers no longer
        # represent a fixed person, so cannot stop search or measure motion.
        if kind == 0 and (x1 <= 2 or y1 <= 2 or x2 >= size[0] - 2 or y2 >= size[1] - 2):
            edge_person = True
            continue
        candidates.append((distance, -confidence, center))
    if not candidates:
        return reject("person_box_at_image_edge" if edge_person else "no_matching_class_confidence_box_or_track")
    center = min(candidates)[2]
    return Target(center, captured, snapshot.get("sequence", captured), tuple(size))


@dataclass(frozen=True)
class TaskOutput:
    target: tuple
    phase: str
    action: str = None
    done: bool = False
    failed: str = None


class VisualTask:
    def __init__(self, door, anchor, scan_points, now, jacobian=None, arrival_xy_m=.15,
                 mark_hold_s=0., mark_max_wait_s=MARK_MAX_WAIT_S):
        self.door = door
        self.mark_hold_s = mark_hold_s
        self.mark_max_wait_s = mark_max_wait_s
        self.kind = 2 if door == 1 else 0
        self.anchor = tuple(anchor)
        self.desired = self.commanded = self.anchor
        self.scan_points = list(scan_points)
        self.jacobian = jacobian
        self.arrival_xy_m = arrival_xy_m
        self.phase = "SEARCH"
        self.started = self.changed = now
        self.last_step = now
        self.last_center = self.last_sequence = None
        self.arrived_at = None
        self.samples = []
        self.last_calibration_error = None
        self.last_calibration_candidate = None
        self.sample_frames = []
        self.sample_origin = None
        self.reversed_probe_axes = set()
        self.recover_probe_phase = None
        self.align_origin = self.anchor
        self.open_sent = self.close_sent = False
        self.failure = None
        self.return_failure = None
        self.last_camera_sequence = None
        self.scan_paused = False
        self.observation = {}
        self.wait_reason = "search_target"
        self.pose_history = deque(maxlen=64)
        self.world_centers = deque(maxlen=5)
        self.alignment_reference = None
        self.alignment_last_captured = None
        self.alignment_growth_frames = 0
        self.alignment_recoveries = 0
        self.mark_since = self.mark_last = self.mark_first_seen = None

    def _phase(self, phase, now):
        self.phase, self.changed = phase, now
        self.arrived_at = None
        self.sample_frames = []
        self.alignment_reference = None
        self.alignment_last_captured = None
        self.alignment_growth_frames = 0

    def diagnostic(self, now):
        result = dict(self.observation, phase=self.phase, phase_elapsed_s=max(0., now - self.changed),
                    desired_xy=self.desired[:2], commanded_xy=self.commanded[:2],
                    wait_reason=self.wait_reason, failure=self.failure,
                    release_policy="center_then_open", release_threshold_px=RELEASE_THRESHOLD_PX,
                    image_axis_calibration="ground_precalibrated_no_airborne_probes",
                    camera_alignment_ready=self.jacobian is not None,
                    calibration_speed_mps=CALIBRATION_SPEED_MPS,
                    calibration_settle_s=CALIBRATION_SETTLE_S,
                    calibration_sample_frames=CALIBRATION_SAMPLE_FRAMES,
                    last_calibration_error=self.last_calibration_error,
                    last_calibration_candidate=self.last_calibration_candidate)
        result.update(search_endpoint_xy=self.scan_points[-1][:2] if self.scan_points else None,
                      return_anchor_reason=self.return_failure,
                      alignment_growth_frames=self.alignment_growth_frames,
                      alignment_recoveries=self.alignment_recoveries,
                      personnel_fast_approach=self._personnel_fast_approach())
        if "actual_xy" in result:
            result["position_error_m"] = math.dist(result["actual_xy"], self.desired[:2])
            result["probe_arrived"] = result["position_error_m"] <= .025 and result["speed_mps"] <= .1
            result["route_arrived"] = result["position_error_m"] <= self.arrival_xy_m and result["speed_mps"] <= .1
        if result.get("captured_monotonic") is not None:
            result["frame_age_s"] = max(0., now - result["captured_monotonic"])
        result["command_complete"] = math.dist(self.commanded[:2], self.desired[:2]) <= 1e-6
        return result

    def _captured_pose(self, captured):
        for (at, a), (bt, b) in zip(self.pose_history, list(self.pose_history)[1:]):
            if at <= captured <= bt:
                ratio = 0. if bt == at else (captured - at) / (bt - at)
                return tuple(a[i] + ratio * (b[i] - a[i]) for i in range(2))
        if self.pose_history and captured == self.pose_history[-1][0]:
            return self.pose_history[-1][1]
        return None

    def _probe_ready(self, pose, speed):
        command_done = math.dist(self.commanded[:2], self.desired[:2]) <= 1e-6
        displacement = math.dist(pose[:2], self.sample_origin[:2])
        self.observation["probe_displacement_m"] = displacement
        if not command_done:
            self.wait_reason = "command_ramping"
        elif speed > CALIBRATION_SPEED_MPS:
            self.wait_reason = "probe_vehicle_moving"
        elif math.dist(pose[:2], self.desired[:2]) > .10:
            self.wait_reason = "tracking_error_over_10cm"
        elif displacement < .04:
            self.wait_reason = "probe_baseline_under_4cm"
        else:
            self.wait_reason = "probe_settle_and_sample"
            return True
        return False

    def _target_world_center(self, target, fresh, pose, now):
        """Pair pixels with capture-time pose; filter a fixed world point."""
        while self.world_centers and now - self.world_centers[0][0] > .75:
            self.world_centers.popleft()
        if target is not None and fresh and (not self.world_centers or
                                             target.captured > self.world_centers[-1][0]):
            captured_pose = self._captured_pose(target.captured)
            if captured_pose is not None:
                error = tuple(target.center[i] - target.size[i] / 2 for i in range(2))
                inv = inverse(self.jacobian)
                point = tuple(captured_pose[i] - sum(inv[i][j] * error[j] for j in range(2))
                              for i in range(2))
                self.world_centers.append((target.captured, point))
        self.observation["center_sample_count"] = len(self.world_centers)
        if len(self.world_centers) < 3:
            self.observation["filtered_pixel_error"] = None
            return None
        center = tuple(median(sample[1][i] for sample in self.world_centers) for i in range(2))
        # Filter the stationary target estimate, not aircraft motion.
        error = tuple(sum(self.jacobian[i][j] * (pose[j] - center[j]) for j in range(2))
                      for i in range(2))
        self.observation.update(target_world_xy=center, filtered_pixel_error=math.hypot(*error))
        return center

    def _personnel_fast_approach(self):
        return False  # Both task types use measured, <=3cm corrections.

    def _recover_diverging_alignment(self, target, fresh, pose, now):
        # Compare distinct capture-time observations only after actual motion.
        # Delayed/frozen frames and ordinary box jitter must not invalidate a
        # working matrix. No fixed dwell is added to a converging approach.
        if self.door != 2 or not target or not fresh:
            return False
        if self.alignment_last_captured is not None and target.captured <= self.alignment_last_captured:
            return False
        captured_pose = self._captured_pose(target.captured)
        if captured_pose is None:
            return False
        self.alignment_last_captured = target.captured
        error = self.observation['pixel_error']
        if self.alignment_reference is None or error < self.alignment_reference[0]:
            self.alignment_reference = (error, captured_pose)
            self.alignment_growth_frames = 0
            return False
        best_error, reference_pose = self.alignment_reference
        growing = (math.dist(captured_pose, reference_pose) >= .04
                   and error > best_error + max(20., .15 * best_error))
        self.alignment_growth_frames = self.alignment_growth_frames + 1 if growing else 0
        if self.alignment_growth_frames < 3:
            return False
        self._hold_here(pose)
        self.jacobian = None
        self.world_centers.clear()
        self.samples = []
        self.sample_origin = None
        self.reversed_probe_axes.clear()
        self.recover_probe_phase = None
        self.last_center = None
        self.alignment_recoveries += 1
        if self.alignment_recoveries > 1:
            self.return_failure = 'visual_alignment_not_converging'
            self.desired = self.anchor
            self._phase('RETURN_ANCHOR', now)
            self.wait_reason = 'alignment_diverged_return_anchor'
        else:
            self._phase('SEARCH', now)
            self.wait_reason = 'alignment_diverged_mapping_disabled'
        return True

    def _align(self, target, fresh, pose, speed, now):
        if self._recover_diverging_alignment(target, fresh, pose, now):
            return None
        center = self._target_world_center(target, fresh, pose, now)
        tracking_error = math.dist(pose[:2], self.desired[:2])
        command_done = math.dist(self.commanded[:2], self.desired[:2]) <= 1e-6
        fast_approach = self._personnel_fast_approach()
        speed_limit = .25 if fast_approach else .1
        if not command_done or speed > speed_limit or tracking_error > .10:
            self.wait_reason = ("command_ramping" if not command_done else
                                "vehicle_moving" if speed > speed_limit else "tracking_error_over_10cm")
            return None
        if not target or not fresh:
            self.wait_reason = "waiting_fresh_alignment_frame"
            return None
        if center is None:
            self.wait_reason = "waiting_three_center_samples"
            return None
        if self.door == 1 and math.dist(center, self.align_origin[:2]) > .35:
            self.failure = "visual_alignment_bound"
            return None
        # Damped, short corrections tolerate scale error in the online Jacobian.
        # Fresh raw centering is handled before this method, without any dwell.
        if math.dist(center, self.desired[:2]) > .01:
            delta = tuple(.35 * (center[i] - pose[i]) for i in range(2))
            length = math.hypot(*delta)
            step_limit = .08 if fast_approach else .03
            ratio = min(1., step_limit / max(length, 1e-9))
            proposed = tuple(pose[i] + ratio * delta[i] for i in range(2)) + self.anchor[2:]
            if self.door == 1 and math.dist(proposed[:2], self.align_origin[:2]) > .35:
                self.failure = "visual_alignment_bound"
            else:
                self.desired = proposed
                self.wait_reason = "holding_estimated_target"
            return None
        self.wait_reason = "holding_estimated_target"
        return None

    def _next_scan(self, now):
        if not self.scan_points:
            return False
        # Keep the endpoint available if acquisition/calibration interrupts the
        # scan. Never dwell at intermediate sampling points.
        self.desired = self.scan_points[-1]
        self.last_center = None
        self.world_centers.clear()
        self.scan_paused = False
        self._phase("SCAN", now)
        return True

    def _hold_here(self, pose):
        self.desired = self.commanded = (*pose[:2], *self.anchor[2:])

    def _return_without_person(self, now):
        self.return_failure = "correct_target_not_found"
        self.desired = self.anchor
        self.wait_reason = "search_exhausted_return_anchor"
        self._phase("RETURN_ANCHOR", now)

    def _scan(self, target, fresh, pose, speed, now, camera_valid, camera_captured, camera_fresh):
        endpoint = self.scan_points[-1]
        if not camera_valid:
            if not self.scan_paused:
                self._hold_here(pose)
            self.scan_paused = True
            self.arrived_at = None
            self.wait_reason = "waiting_live_search_frame"
            return
        self.scan_paused = False
        if target is not None:
            if self.jacobian is None:
                self._hold_here(pose)
                self.align_origin = self.desired
                self._phase("SEARCH", now)
                self.wait_reason = "person_acquired_stop_scan"
                return
            center = self._target_world_center(target, fresh, pose, now)
            if center is None:
                self.wait_reason = "waiting_three_center_samples"
                return
            # The aisle constrains blind search. Once a person is acquired,
            # approach the measured target in short steps, including laterally.
            # The 35cm task-zone radius belongs to hazardous delivery only.
            self._hold_here(pose)
            self.align_origin = self.desired
            self._phase("ALIGN", now)
            self.wait_reason = "person_acquired_start_alignment"
            return
        self.desired = endpoint
        self.last_center = None
        self.world_centers.clear()
        self.wait_reason = "searching_rescue_aisle"
        reached = (math.dist(self.commanded[:2], endpoint[:2]) <= 1e-6
                   and math.dist(pose[:2], endpoint[:2]) <= self.arrival_xy_m and speed <= .1)
        if not reached:
            self.arrived_at = None
        else:
            if self.arrived_at is None:
                self.arrived_at = now
            # Confirm absence with a frame captured at/after arrival, not with
            # a cached observation from the last moving portion of the aisle.
            if camera_fresh and camera_captured >= self.arrived_at:
                self._return_without_person(now)

    def _move(self, pose, now):
        dt = max(0., min(.1, now - self.last_step))
        self.last_step = now
        distance = math.dist(self.commanded[:2], self.desired[:2])
        speed_limit = .20 if self._personnel_fast_approach() else .12
        ratio = 1. if distance == 0 else min(1., speed_limit * dt / distance)
        self.commanded = tuple(self.commanded[i] + ratio * (self.desired[i] - self.commanded[i]) for i in range(2)) + self.anchor[2:]
        return self.commanded

    def _sample(self, target, fresh, arrived, now, pose):
        if not arrived:
            self.arrived_at = None
            self.sample_frames = []
            return None
        if self.arrived_at is None:
            self.arrived_at = now
        if (not target or not fresh or target.captured < self.arrived_at + CALIBRATION_SETTLE_S
                or now - self.arrived_at < CALIBRATION_SETTLE_S):
            return None
        captured_pose = self._captured_pose(target.captured)
        if captured_pose is None:
            return None
        if self.phase in ("PROBE_X", "PROBE_Y") and math.dist(captured_pose, self.sample_origin[:2]) < .04:
            self.wait_reason = "probe_baseline_under_4cm"
            return None
        self.sample_frames.append((captured_pose, target.center))
        if len(self.sample_frames) < CALIBRATION_SAMPLE_FRAMES:
            return None
        return tuple(tuple(sum(s[k][i] for s in self.sample_frames) / len(self.sample_frames) for i in range(2)) for k in range(2))

    def _recover_edge_probe(self, pose, speed, now, camera_valid, camera_fresh, camera_captured):
        # A first target can sit at the image edge. Do not infer camera axes:
        # return to the measured origin and try the opposite baseline once.
        if (self.door != 2 or self.phase in self.reversed_probe_axes
                or not camera_valid or not camera_fresh or self.arrived_at is None
                or camera_captured < self.arrived_at + CALIBRATION_SETTLE_S
                or now - self.changed < .8 or not self._probe_ready(pose, speed)):
            return False
        self.reversed_probe_axes.add(self.phase)
        self.recover_probe_phase = self.phase
        self.desired = self.sample_origin
        self.wait_reason = "probe_target_at_edge_return_origin"
        self._phase("PROBE_RECOVER", now)
        return True

    def step(self, pose, velocity, now, snapshot, ack=None):
        if not finite(now) or now < self.last_step:
            raise ValueError("任务时钟无效")
        if self.phase == "DONE":
            return TaskOutput(self.anchor, self.phase, done=True)
        if self.phase == "FAILED":
            return TaskOutput(self._move(pose, now), self.phase, failed=self.failure)
        self.pose_history.append((now, tuple(pose[:2])))
        self.observation = {}
        target = select_target(snapshot, self.kind, now, self.last_center, self.observation)
        camera_valid = (target is not None or self.observation.get("target_rejection")
                        in ("no_matching_class_confidence_box_or_track", "person_box_at_image_edge"))
        camera_captured = capture_time(snapshot) if camera_valid else None
        camera_sequence = snapshot.get("sequence", camera_captured) if camera_valid else None
        camera_fresh = camera_valid and camera_sequence != self.last_camera_sequence
        if camera_fresh:
            self.last_camera_sequence = camera_sequence
        fresh = target is not None and target.sequence != self.last_sequence
        if fresh:
            self.last_sequence = target.sequence
            self.last_center = target.center
        speed = math.sqrt(sum(v * v for v in velocity))
        position_error = math.dist(pose[:2], self.desired[:2])
        probe_arrived = position_error <= .025 and speed <= .1
        route_arrived = position_error <= self.arrival_xy_m and speed <= .1
        arrived = probe_arrived if self.phase in ("PROBE_X", "PROBE_Y") else route_arrived
        self.observation.update(actual_xy=tuple(pose[:2]), speed_mps=speed,
                                position_error_m=math.dist(pose[:2], self.desired[:2]),
                                probe_arrived=probe_arrived, route_arrived=route_arrived,
                                target_valid=target is not None,
                                fresh_sequence=fresh, sequence=target.sequence if target else None,
                                captured_monotonic=target.captured if target else None,
                                frame_age_s=now - target.captured if target else None,
                                camera_frame_valid=camera_valid, camera_fresh_sequence=camera_fresh,
                                target_center_px=target.center if target else None,
                                image_size=target.size if target else None,
                                pixel_error=None if target is None else math.dist(
                                    target.center, (target.size[0] / 2, target.size[1] / 2)))
        action = None
        if now - self.started > 45:
            self.failure = "visual_task_timeout"
        if ack is not None and not ack[1]:
            self.failure = "payload_command_failed_or_unknown"
        if (self.door == 1 and self.phase in ("ALIGN", "PROBE_X", "PROBE_Y", "PROBE_RECOVER", "CALIB_RETURN")
                and math.dist(pose[:2], self.align_origin[:2]) > .35):
            self.failure = "visual_alignment_bound"
        if self.failure:
            self.desired = self.anchor
            if self.open_sent and not self.close_sent:
                action, self.close_sent = "close", True
            self._phase("FAILED", now)
            return TaskOutput(self._move(pose, now), self.phase, action, failed=self.failure)

        # A fresh, correctly classified, centered frame counts as a marked
        # frame. The door opens only once the marking has stayed visible for
        # the scored duration, so a single lucky frame is not enough.
        marked_now = (fresh and target is not None
                      and self.observation.get("pixel_error") is not None
                      and self.observation["pixel_error"] <= RELEASE_THRESHOLD_PX)
        if marked_now:
            if self.mark_first_seen is None:
                self.mark_first_seen = target.captured
            if self.mark_since is None or target.captured - self.mark_last > MARK_GAP_S:
                self.mark_since = target.captured
            self.mark_last = target.captured
        elif self.mark_last is not None and now - self.mark_last > MARK_GAP_S:
            self.mark_since = self.mark_last = None

        def release(reason):
            self.desired = self.commanded = (*pose[:2], *self.anchor[2:])
            self.open_sent = True
            self._phase("OPEN_WAIT", now)
            self.wait_reason = reason
            return TaskOutput(self._move(pose, now), self.phase, action="open")

        if self.mark_hold_s <= 0:
            # Marked dwell disabled: a single centered frame opens the door.
            if (self.phase in ("SEARCH", "SCAN", "PROBE_X", "PROBE_Y",
                               "PROBE_RECOVER", "CALIB_RETURN", "ALIGN")) and marked_now:
                return release("centered_open_immediately")
        elif (self.phase in ("SEARCH", "SCAN", "PROBE_X", "PROBE_Y", "PROBE_RECOVER", "CALIB_RETURN", "ALIGN")
                and marked_now and self.mark_since is not None
                and self.mark_last - self.mark_since >= self.mark_hold_s - 1e-9):
            return release("marked_hold_satisfied")
        if (self.mark_hold_s > 0 and self.mark_first_seen is not None
                and now - self.mark_first_seen > self.mark_max_wait_s):
            # Bounded wait: never hover indefinitely for a mark that will not
            # stay steady. Drop with whatever valid frame exists, else give the
            # waypoint up so the route can continue.
            if marked_now:
                return release("mark_deadline_best_effort")
            self.failure = "correct_target_not_found"

        if self.phase == "SEARCH":
            if self.door == 2 and not camera_valid:
                self.wait_reason = "waiting_live_search_frame"
                self.arrived_at = None
                self.sample_frames = []
                return TaskOutput(self._move(pose, now), self.phase)
            if self.door == 2 and target is None and camera_valid:
                if self._next_scan(now):
                    self.wait_reason = "searching_rescue_aisle"
                else:
                    self._return_without_person(now)
                return TaskOutput(self._move(pose, now), self.phase)
            if target is not None and self.jacobian is None:
                self.wait_reason = "camera_alignment_missing"
                return TaskOutput(self._move(pose, now), self.phase)
            sample = self._sample(target, fresh, arrived, now, pose)
            if sample:
                self.align_origin = (*sample[0], *self.anchor[2:])
                self._phase("ALIGN", now)
            elif arrived and self.arrived_at is not None and now - self.arrived_at > 2.5 and target is None:
                if not self._next_scan(now):
                    self.failure = "correct_target_not_found"
        elif self.phase == "SCAN":
            self._scan(target, fresh, pose, speed, now, camera_valid, camera_captured, camera_fresh)
        elif self.phase == "ALIGN":
            if (self.door == 2 and camera_fresh
                    and self.observation.get('target_rejection') == 'person_box_at_image_edge'):
                self._hold_here(pose)
                # A bad matrix can push an initially complete box out of
                # frame before three growth samples arrive. Do not reuse it
                # when the scan next reacquires the same person.
                self.jacobian = None
                self.samples = []
                self.reversed_probe_axes.clear()
                if not self._next_scan(now):
                    self._return_without_person(now)
                self.wait_reason = 'person_clipped_continue_search'
            else:
                action = self._align(target, fresh, pose, speed, now)
        elif self.phase == "OPEN_WAIT":
            if ack == ("open", True):
                self._phase("DROP_DWELL", now)
            elif now - self.changed > 3:
                self.failure = "open_ack_timeout"
        elif self.phase == "DROP_DWELL":
            if now - self.changed >= 1.0:
                self.close_sent, action = True, "close"
                self._phase("CLOSE_WAIT", now)
        elif self.phase == "CLOSE_WAIT":
            if ack == ("close", True):
                self.desired = self.anchor
                self.wait_reason = "return_task_anchor"
                self._phase("RETURN_ANCHOR", now)
            elif now - self.changed > 3:
                self.failure = "close_ack_timeout"
        elif self.phase == "RETURN_ANCHOR":
            if arrived and math.dist(self.commanded[:2], self.anchor[:2]) <= 1e-6:
                if self.return_failure:
                    self.failure = self.return_failure
                    self._phase("FAILED", now)
                    return TaskOutput(self.anchor, self.phase, failed=self.failure)
                self._phase("DONE", now)
                return TaskOutput(self.anchor, self.phase, done=True)
        return TaskOutput(self._move(pose, now), self.phase, action)
