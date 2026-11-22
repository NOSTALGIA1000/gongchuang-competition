"""Bounded pre-aiding LIO recovery; never a flight-time restart policy."""
import math


def current_generation_stamp(stamp_ns, spawned_ros_ns):
    return spawned_ros_ns is not None and stamp_ns > spawned_ros_ns


def expected_startup_exit(phase, expected_pid, actual_pid):
    return (phase in ('STOPPING', 'SPAWN_REQUESTED') and expected_pid is not None
            and expected_pid == actual_pid)


class FreshLioWindow:
    def __init__(self, hold_s=3., max_gap_s=.3):
        self.hold_s, self.max_gap_s = hold_s, max_gap_s
        self.reset()

    def reset(self):
        self.since = self.last_sample = None

    def observe(self, now, valid):
        if not valid:
            self.reset()
            return
        if self.last_sample is None or now - self.last_sample > self.max_gap_s:
            self.since = now
        self.last_sample = now

    def ready(self, now):
        if self.last_sample is None or now - self.last_sample > self.max_gap_s:
            self.reset()
            return False
        return self.since is not None and now - self.since >= self.hold_s


class StartupRetryGate:
    def __init__(self, wait_s=15., max_retries=2, stop_timeout_s=10.):
        self.wait_s, self.max_retries = wait_s, max_retries
        self.stop_timeout_s = stop_timeout_s
        self.retries = self.generation = 0
        self.spawned_at = self.stopping_since = None
        self.fresh_wait_since = None
        self.phase = 'NOT_STARTED'
        self.fault = None

    def spawned(self, now):
        self.spawned_at = now
        self.fresh_wait_since = now
        self.stopping_since = None
        self.generation += 1
        self.phase = 'RUNNING'

    def mark_fresh(self, now):
        if self.phase == 'RUNNING':
            self.fresh_wait_since = now

    def next_action(self, now, snapshot):
        s = snapshot
        if self.fault:
            return None
        if (self.phase == 'STOPPING' and s.get('process_alive')
                and now - self.stopping_since > self.stop_timeout_s):
            self.fault = 'Startup LIO stop exceeded 10 seconds; no second instance started'
            return None
        if any(s.get(key) for key in ('bridge_ever_started', 'armed_ever_seen',
                                      'external_pose_ever_observed', 'initialized_seen', 'armed')):
            return None
        if (not s.get('connected') or not s.get('network')
                or not 0 <= s.get('state_age', math.inf) <= 3.
                or not 0 <= s.get('lidar_age', math.inf) <= .3
                or not 0 <= s.get('imu_age', math.inf) <= .2):
            return None
        if self.phase == 'STOPPING':
            if not s.get('process_alive'):
                self.phase = 'SPAWN_REQUESTED'
                return 'spawn'
            return None
        if self.phase == 'SPAWN_REQUESTED':
            return 'spawn'
        if self.phase != 'RUNNING' or not s.get('process_alive'):
            return None
        if s.get('lio_age', math.inf) <= .3:
            self.mark_fresh(now)
            return None
        if now - self.fresh_wait_since < self.wait_s:
            return None
        if self.retries >= self.max_retries:
            self.fault = 'Startup LIO remained stale after two retries; inspect on ground'
            return None
        self.retries += 1
        self.phase, self.stopping_since = 'STOPPING', now
        return 'stop'
