import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simple_flight.lidar_health import StreamStats


class StreamStatsTests(unittest.TestCase):
    def test_stale_stream_fails_even_after_many_valid_messages(self):
        stats = StreamStats()
        for i in range(20):
            stats.add(i / 10, 1_000_000_000 + i * 100_000_000)
        result = stats.summary(4.0, minimum_hz=8.0, maximum_gap_s=1.0)
        self.assertFalse(result['passed'])
        self.assertIn('stale', result['failures'])

    def test_timestamp_regression_is_not_hidden_by_receive_rate(self):
        stats = StreamStats()
        stats.add(0.0, 200)
        stats.add(0.1, 100)
        result = stats.summary(0.15, minimum_hz=8.0, maximum_gap_s=1.0)
        self.assertFalse(result['passed'])
        self.assertEqual(result['stamp_regressions'], 1)

    def test_invalid_data_fails(self):
        stats = StreamStats()
        stats.add(0.0, 100, valid=False)
        stats.add(0.1, 200)
        result = stats.summary(0.15, minimum_hz=8.0, maximum_gap_s=1.0)
        self.assertFalse(result['passed'])
        self.assertEqual(result['invalid_messages'], 1)

    def test_continuous_valid_stream_passes(self):
        stats = StreamStats()
        for i in range(20):
            stats.add(i / 10, 1_000_000_000 + i * 100_000_000, point_count=100)
        result = stats.summary(1.95, minimum_hz=8.0, maximum_gap_s=1.0)
        self.assertTrue(result['passed'])
        self.assertAlmostEqual(result['observed_hz'], 10.0)
        self.assertEqual(result['total_points'], 2000)


if __name__ == '__main__':
    unittest.main()
