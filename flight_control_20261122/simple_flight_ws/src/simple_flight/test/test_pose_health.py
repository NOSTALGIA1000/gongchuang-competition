import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simple_flight.pose_health import PoseStats


class PoseHealthTests(unittest.TestCase):
    def test_missing_data_cannot_pass(self):
        r = PoseStats().summary(2.)
        self.assertFalse(r['output_passed'])

    def test_drift_uses_all_samples_and_not_only_endpoint(self):
        s = PoseStats()
        for i, x in enumerate([0., .3, 0.]):
            s.add(i * .1, 1_000_000_000 + i * 100_000_000, [x, 0., 0.], [0., 0., 0., 1.])
        r = s.summary(.25)
        self.assertTrue(r['output_passed'])
        self.assertAlmostEqual(r['maximum_displacement_m'], .3)
        self.assertEqual(r['endpoint_displacement_m'], 0.)
        self.assertFalse(r['static_goal_passed'])

    def test_nonunit_orientation_fails(self):
        s = PoseStats()
        s.add(0., 100, [0., 0., 0.], [0., 0., 0., 0.])
        s.add(.1, 200, [0., 0., 0.], [0., 0., 0., 1.])
        self.assertFalse(s.summary(.15)['output_passed'])

    def test_nan_position_fails(self):
        s = PoseStats()
        s.add(0., 100, [math.nan, 0., 0.], [0., 0., 0., 1.])
        s.add(.1, 200, [0., 0., 0.], [0., 0., 0., 1.])
        self.assertFalse(s.summary(.15)['output_passed'])

    def test_stale_and_timestamp_regression_fail(self):
        s = PoseStats()
        s.add(0., 200, [0., 0., 0.], [0., 0., 0., 1.])
        s.add(.1, 100, [0., 0., 0.], [0., 0., 0., 1.])
        self.assertFalse(s.summary(2.)['output_passed'])

    def test_quaternion_sign_flip_is_same_orientation(self):
        s = PoseStats()
        for i, sign in enumerate([1., -1., 1.]):
            s.add(i * .1, 100 + i, [0., 0., 0.], [0., 0., 0., sign])
        r = s.summary(.25)
        self.assertTrue(r['static_goal_passed'])
        self.assertEqual(r['maximum_orientation_change_deg'], 0.)
