"""Pose output checks; estimator movement is not an external accuracy measure."""

import math
from simple_flight.lidar_health import StreamStats


class PoseStats:
    def __init__(self):
        self.stream = StreamStats()
        self.first_position = None
        self.last_position = None
        self.first_quaternion = None
        self.low = None
        self.high = None
        self.maximum_displacement = 0.
        self.maximum_angle = 0.
        self.frames = set()

    def add(self, received_at, stamp_ns, position, quaternion,
            frame_id='camera_init', child_frame_id='body'):
        finite = all(math.isfinite(x) for x in [*position, *quaternion])
        norm = math.sqrt(sum(x*x for x in quaternion)) if finite else 0.
        valid = finite and abs(norm - 1.) <= .05 and bool(frame_id) and bool(child_frame_id)
        self.stream.add(received_at, stamp_ns, valid)
        self.frames.add((frame_id, child_frame_id))
        if not valid:
            return False
        q = [x / norm for x in quaternion]
        if self.first_position is None:
            self.first_position = list(position)
            self.first_quaternion = q
            self.low = list(position)
            self.high = list(position)
        self.last_position = list(position)
        self.low = [min(a, b) for a, b in zip(self.low, position)]
        self.high = [max(a, b) for a, b in zip(self.high, position)]
        distance = math.sqrt(sum((a-b)**2 for a, b in zip(self.first_position, position)))
        cosine = min(1., abs(sum(a*b for a, b in zip(self.first_quaternion, q))))
        self.maximum_displacement = max(self.maximum_displacement, distance)
        self.maximum_angle = max(self.maximum_angle, math.degrees(2.*math.acos(cosine)))
        return True

    def summary(self, now, max_drift_m=.1, max_rotation_deg=3.):
        stream = self.stream.summary(now, minimum_hz=5., maximum_gap_s=1.)
        output_passed = stream['passed'] and len(self.frames) == 1
        return {
            'output_passed': output_passed,
            'static_goal_passed': (output_passed and self.maximum_displacement <= max_drift_m
                                   and self.maximum_angle <= max_rotation_deg),
            'stream': stream, 'frames': [list(pair) for pair in sorted(self.frames)],
            'first_position_m': self.first_position, 'last_position_m': self.last_position,
            'axis_range_m': ([b-a for a, b in zip(self.low, self.high)] if self.low else None),
            'maximum_displacement_m': self.maximum_displacement,
            'endpoint_displacement_m': (math.dist(self.first_position, self.last_position)
                                        if self.first_position is not None else None),
            'maximum_orientation_change_deg': self.maximum_angle,
            'static_goal': {'max_displacement_m': max_drift_m,
                            'max_orientation_change_deg': max_rotation_deg},
        }
