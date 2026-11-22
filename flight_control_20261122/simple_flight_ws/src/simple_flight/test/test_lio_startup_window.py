"""A bridge can only start from a fresh, stable current LIO generation."""
import unittest

try:
    from simple_flight.startup_retry import FreshLioWindow, current_generation_stamp
except ImportError:
    FreshLioWindow = current_generation_stamp = None


class LioStartupWindowTests(unittest.TestCase):
    def test_requires_three_seconds_of_continuous_fresh_measurements(self):
        self.assertIsNotNone(FreshLioWindow)
        window = FreshLioWindow(hold_s=3., max_gap_s=.3)
        for index in range(30):
            window.observe(index / 10., True)
            self.assertFalse(window.ready(index / 10.))
        window.observe(3., True)
        self.assertTrue(window.ready(3.))
        self.assertFalse(window.ready(3.31))
        window.observe(3.4, True)
        self.assertFalse(window.ready(3.4))

    def test_invalid_sample_and_explicit_generation_reset_remove_old_stability(self):
        self.assertIsNotNone(FreshLioWindow)
        window = FreshLioWindow()
        for index in range(32):
            window.observe(index / 10., True)
        self.assertTrue(window.ready(3.1))
        window.observe(3.2, False)
        self.assertFalse(window.ready(3.2))
        window.observe(3.3, True)
        window.reset()
        self.assertFalse(window.ready(3.4))

    def test_previous_generation_timestamps_never_pass(self):
        self.assertIsNotNone(current_generation_stamp)
        self.assertFalse(current_generation_stamp(100, None))
        self.assertFalse(current_generation_stamp(99, 100))
        self.assertFalse(current_generation_stamp(100, 100))
        self.assertTrue(current_generation_stamp(101, 100))


if __name__ == '__main__':
    unittest.main()
