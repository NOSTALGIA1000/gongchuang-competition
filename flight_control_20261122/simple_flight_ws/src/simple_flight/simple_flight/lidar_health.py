"""Receive-side stream checks independent of ROS and wall-clock time."""


class StreamStats:
    def __init__(self):
        self.count = 0
        self.invalid = 0
        self.first = None
        self.last = None
        self.last_stamp = None
        self.regressions = 0
        self.duplicates = 0
        self.maximum_gap = 0.0
        self.total_points = 0
        self.minimum_points = None
        self.maximum_points = 0

    def add(self, received_at, stamp_ns, valid=True, point_count=0):
        if self.first is None:
            self.first = received_at
        if self.last is not None:
            self.maximum_gap = max(self.maximum_gap, received_at - self.last)
        if self.last_stamp is not None:
            self.regressions += int(stamp_ns < self.last_stamp)
            self.duplicates += int(stamp_ns == self.last_stamp)
        self.last = received_at
        self.last_stamp = stamp_ns
        self.count += 1
        self.invalid += int(not valid or stamp_ns <= 0)
        self.total_points += point_count
        if point_count:
            self.minimum_points = min(self.minimum_points or point_count, point_count)
            self.maximum_points = max(self.maximum_points, point_count)

    def summary(self, now, minimum_hz, maximum_gap_s):
        span = (self.last - self.first) if self.count >= 2 else 0.0
        hz = (self.count - 1) / span if span > 0 else 0.0
        age = now - self.last if self.last is not None else None
        failures = []
        if self.count < 2:
            failures.append('insufficient_messages')
        if hz < minimum_hz:
            failures.append('low_receive_rate')
        if age is None or age > maximum_gap_s:
            failures.append('stale')
        if self.maximum_gap > maximum_gap_s:
            failures.append('receive_gap')
        if self.invalid:
            failures.append('invalid_data')
        if self.regressions:
            failures.append('stamp_regression')
        if self.duplicates:
            failures.append('duplicate_stamp')
        return {
            'passed': not failures, 'failures': failures,
            'messages': self.count, 'observed_hz': hz,
            'latest_receive_age_s': age, 'maximum_receive_gap_s': self.maximum_gap,
            'invalid_messages': self.invalid, 'stamp_regressions': self.regressions,
            'duplicate_stamps': self.duplicates, 'last_header_stamp_ns': self.last_stamp,
            'total_points': self.total_points, 'minimum_points_per_frame': self.minimum_points,
            'maximum_points_per_frame': self.maximum_points,
        }
