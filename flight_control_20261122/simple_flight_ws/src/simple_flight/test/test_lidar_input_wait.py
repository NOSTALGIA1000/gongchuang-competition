"""A topic name in discovery is insufficient: require a fresh typed sample."""
import struct
import unittest
from test_lidar_cdr import fixture

try:
    from simple_flight.lidar_input_wait import sample_ready
except ImportError:
    sample_ready = None


class LidarInputWaitTests(unittest.TestCase):
    def check(self, raw, now):
        self.assertIsNotNone(sample_ready, 'Fresh typed input check is not implemented')
        return sample_ready(raw, now)

    def test_fresh_nonempty_custom_sample_passes(self):
        self.assertTrue(self.check(fixture(), 123100000456))

    def test_old_sample_does_not_allow_algorithm_start(self):
        self.assertFalse(self.check(fixture(), 124000000456))

    def test_future_sample_does_not_allow_algorithm_start(self):
        self.assertFalse(self.check(fixture(), 122000000456))

    def test_empty_cloud_is_not_readiness(self):
        self.assertFalse(self.check(fixture(count=0), 123100000456))

    def test_corrupt_payload_cannot_fake_input(self):
        self.assertFalse(self.check(fixture()[:-5], 123100000456))
        self.assertFalse(self.check(b'not a ROS CDR payload', 123100000456))

    def test_zero_measurement_time_is_rejected(self):
        raw = bytearray(fixture())
        struct.pack_into('<iI', raw, 4, 0, 0)
        self.assertFalse(self.check(bytes(raw), 1000000))
