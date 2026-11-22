"""Catch reversed lever arms, ignored attitude, and invalid/stale pose forwarding."""
import math
import unittest

try:
    from simple_flight.pose_adapter import PoseAdapter
except ImportError:
    PoseAdapter = None


class PoseAdapterTests(unittest.TestCase):
    def test_world_heading_alignment_rotates_both_position_and_orientation(self):
        self.assertIsNotNone(PoseAdapter)
        adapter = PoseAdapter(sensor_in_body=(0., 0., .05), world_yaw=math.pi/2)
        adapter.accept(1_000_000_000, 1_020_000_000,
                        (0., 0., .05), (0., 0., 0., 1.))
        p, q = adapter.accept(1_500_000_000, 1_520_000_000,
                              (.5, 0., .05), (0., 0., 0., 1.))
        self.assertAlmostEqual(p[0], 0., places=10)
        self.assertAlmostEqual(p[1], .5)
        self.assertAlmostEqual(q[2], math.sqrt(.5))
        self.assertAlmostEqual(q[3], math.sqrt(.5))

    def make_adapter(self):
        self.assertIsNotNone(PoseAdapter, 'PoseAdapter is not implemented')
        return PoseAdapter(sensor_in_body=(0., 0., .05))

    def test_mount_offset_removed_and_first_body_pose_is_origin(self):
        adapter = self.make_adapter()
        p, q = adapter.accept(1_000_000_000, 1_050_000_000,
                              (2., 3., 1.05), (0., 0., 0., 1.))
        self.assertEqual(p, (0., 0., 0.))
        p, q = adapter.accept(1_100_000_000, 1_150_000_000,
                              (2., 3., 1.15), (0., 0., 0., 1.))
        self.assertAlmostEqual(p[2], .1)

    def test_tilt_compensation_keeps_body_origin_fixed(self):
        # Rotating a sensor 5 cm above a fixed center by +90 deg about y
        # moves the sensor to x=+5cm, z=0. No body translation occurred.
        adapter = self.make_adapter()
        adapter.accept(1_000_000_000, 1_020_000_000, (0., 0., .05), (0., 0., 0., 1.))
        p, q = adapter.accept(1_600_000_000, 1_620_000_000,
                              (.05, 0., 0.), (0., math.sqrt(.5), 0., math.sqrt(.5)))
        for value in p:
            self.assertAlmostEqual(value, 0., places=10)

    def test_arbitrary_initial_heading_preserved_with_world_motion(self):
        adapter = self.make_adapter()
        q = (0., 0., math.sqrt(.5), math.sqrt(.5))
        adapter.accept(1_000_000_000, 1_050_000_000, (0., 0., .05), q)
        p, out_q = adapter.accept(1_500_000_000, 1_550_000_000, (0., .3, .05), q)
        self.assertAlmostEqual(p[0], 0.)
        self.assertAlmostEqual(p[1], .3)
        self.assertEqual(out_q, q)

    def test_bad_or_old_sample_does_not_set_origin_or_advance_stamp(self):
        adapter = self.make_adapter()
        fixtures = [((float('nan'), 0., 0.), (0., 0., 0., 1.), 1_050_000_000),
                    ((0., 0., 0.), (0., 0., 0., 0.), 1_050_000_000),
                    ((0., 0., 0.), (0., 0., 0., 1.), 1_400_000_000),
                    ((0., 0., 0.), (0., 0., 0., 1.), 800_000_000)]
        for p, q, now in fixtures:
            with self.assertRaises(ValueError):
                adapter.accept(1_000_000_000, now, p, q)
        p, _ = adapter.accept(1_000_000_000, 1_050_000_000,
                               (2., 0., .05), (0., 0., 0., 1.))
        self.assertEqual(p, (0., 0., 0.))

    def test_duplicate_dropped_but_time_reset_and_jump_latch(self):
        adapter = self.make_adapter()
        pose = ((0., 0., .05), (0., 0., 0., 1.))
        adapter.accept(1_000_000_000, 1_050_000_000, *pose)
        with self.assertRaises(ValueError):
            adapter.accept(1_000_000_000, 1_060_000_000, *pose)
        adapter.accept(1_100_000_000, 1_150_000_000, *pose)
        with self.assertRaises(ValueError):
            adapter.accept(1_050_000_000, 1_150_000_000, *pose)
        with self.assertRaises(ValueError):
            adapter.accept(1_200_000_000, 1_250_000_000, *pose)
        other = self.make_adapter()
        other.accept(1_000_000_000, 1_050_000_000, *pose)
        with self.assertRaises(ValueError):
            other.accept(1_100_000_000, 1_150_000_000,
                         (10., 0., .05), pose[1])
        with self.assertRaises(ValueError):
            other.accept(1_200_000_000, 1_250_000_000, *pose)

    def test_long_measurement_gap_latches_instead_of_reanchoring(self):
        adapter = self.make_adapter()
        pose = ((0., 0., .05), (0., 0., 0., 1.))
        adapter.accept(1_000_000_000, 1_050_000_000, *pose)
        with self.assertRaises(ValueError):
            adapter.accept(3_000_000_000, 3_050_000_000, *pose)


if __name__ == '__main__':
    unittest.main()
