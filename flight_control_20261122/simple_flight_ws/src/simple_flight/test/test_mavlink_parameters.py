import struct
import unittest

try:
    from simple_flight.mavlink_parameters import decode_parameter
except ImportError:
    decode_parameter = None


class ParameterTests(unittest.TestCase):
    def test_bench_plan_changes_only_position_fusion_and_height_reference(self):
        import simple_flight.mavlink_parameters as module
        self.assertTrue(hasattr(module, 'external_pose_plan'), 'Bench plan not implemented')
        before = {'EKF2_EV_CTRL': 0, 'EKF2_HGT_REF': 1, 'COM_ARM_WO_GPS': 0}
        self.assertEqual(module.external_pose_plan(before, before),
                         {'EKF2_EV_CTRL': 3, 'EKF2_HGT_REF': 3})
        self.assertEqual(before['COM_ARM_WO_GPS'], 0)
        current = dict(before, EKF2_EV_CTRL=3, EKF2_HGT_REF=3)
        self.assertEqual(module.external_pose_plan(current, before), {})
        with self.assertRaises(ValueError):
            module.external_pose_plan(dict(before, EKF2_EV_CTRL=15), before)

    def test_px4_integer_bits_are_not_interpreted_as_float_magnitude(self):
        self.assertIsNotNone(decode_parameter, 'Parameter decoder not implemented')
        wire_one = struct.unpack('<f', b'\x01\x00\x00\x00')[0]
        self.assertEqual(decode_parameter(wire_one, 6), 1)
        wire_negative = struct.unpack('<f', struct.pack('<i', -1))[0]
        self.assertEqual(decode_parameter(wire_negative, 6), -1)

    def test_real_parameter_stays_real_and_unknown_encoding_rejected(self):
        self.assertIsNotNone(decode_parameter, 'Parameter decoder not implemented')
        self.assertAlmostEqual(decode_parameter(.1, 9), .1)
        with self.assertRaises(ValueError):
            decode_parameter(0., 10)


if __name__ == '__main__':
    unittest.main()
