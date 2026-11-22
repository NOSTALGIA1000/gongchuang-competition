"""Guard against false boot readiness and unsafe automatic recovery."""
import unittest

try:
    from simple_flight.boot_policy import ReadinessGuard, decode_sys_status
except ImportError:
    ReadinessGuard = decode_sys_status = None


def healthy():
    return dict(network=True, connected=True, state_age=.1, armed=False, mode='POSCTL',
                initialized=True, bridge_fault=None, process_failure=None,
                lidar_age=.05, imu_age=.01, lio_age=.06, vision_age=.07,
                fcu_pose_age=.03, bridge_age=.2, estimator_age=.2,
                position_valid=True, height_valid=True, rc_ready=True,
                sys_status_age=.2, prearm_ready=True, parameters_ok=True)


class BootReadinessTests(unittest.TestCase):
    def guard(self):
        self.assertIsNotNone(ReadinessGuard, 'Boot readiness guard not implemented')
        return ReadinessGuard(hold_s=3.)

    def test_requires_continuous_healthy_window(self):
        g = self.guard()
        self.assertFalse(g.update(0., healthy())['ready_to_arm'])
        self.assertTrue(g.update(3.1, healthy())['ready_to_arm'])

    def test_rc_late_start_waits_and_recovers_without_latching(self):
        g = self.guard()
        missing = dict(healthy(), rc_ready=False)
        self.assertFalse(g.update(0., missing)['ready_to_arm'])
        self.assertIsNone(g.fault)
        g.update(10., healthy())
        self.assertTrue(g.update(13.1, healthy())['ready_to_arm'])

    def test_missing_devices_before_initialization_remain_waiting(self):
        g = self.guard()
        s = dict(healthy(), initialized=False, network=False, connected=False,
                 state_age=float('inf'), lio_age=float('inf'), vision_age=float('inf'))
        self.assertEqual(g.update(100., s)['state'], 'WAITING')
        self.assertIsNone(g.fault)

    def test_armed_keeps_localization_and_does_not_require_rc(self):
        g = self.guard()
        g.update(0., healthy())
        g.update(4., healthy())
        result = g.update(5., dict(healthy(), armed=True, rc_ready=False, prearm_ready=False))
        self.assertEqual(result['state'], 'ARMED')
        self.assertTrue(result['localization_active'])
        self.assertFalse(result['ready_to_arm'])
        self.assertIsNone(g.fault)

    def test_source_loss_after_initialization_latches_across_disarm(self):
        g = self.guard()
        g.update(0., healthy())
        result = g.update(4., dict(healthy(), armed=True, lio_age=1.2, vision_age=1.2))
        self.assertEqual(result['state'], 'FAULT')
        self.assertFalse(g.update(8., healthy())['ready_to_arm'])
        self.assertIsNotNone(g.fault)

    def test_short_old_measurement_resets_ready_window(self):
        g = self.guard()
        g.update(0., healthy())
        self.assertTrue(g.update(4., healthy())['ready_to_arm'])
        self.assertFalse(g.update(5., dict(healthy(), vision_age=.4))['ready_to_arm'])
        self.assertIsNone(g.fault)
        g.update(6., healthy())
        self.assertFalse(g.update(8., healthy())['ready_to_arm'])
        self.assertTrue(g.update(9.1, healthy())['ready_to_arm'])

    def test_process_exit_or_bridge_fault_cannot_auto_clear(self):
        for key in ['process_failure', 'bridge_fault']:
            g = self.guard()
            self.assertEqual(g.update(0., dict(healthy(), **{key: 'failed'}))['state'], 'FAULT')
            self.assertFalse(g.update(20., healthy())['ready_to_arm'])

    def test_stale_telemetry_and_bad_configuration_never_ready(self):
        for key, value in [('state_age', 4.), ('sys_status_age', 4.),
                           ('estimator_age', 4.), ('parameters_ok', False),
                           ('prearm_ready', False), ('height_valid', False)]:
            g = self.guard()
            s = dict(healthy(), **{key: value})
            g.update(0., s)
            self.assertFalse(g.update(10., s)['ready_to_arm'], key)

    def test_rc_all_three_bits_required_and_prearm_decoded(self):
        self.assertIsNotNone(decode_sys_status)
        rc, prearm = 1 << 16, 1 << 28
        import struct
        payload = struct.pack('<III', rc | prearm, rc | prearm, rc | prearm)
        self.assertEqual(decode_sys_status(payload), (True, True))
        self.assertEqual(decode_sys_status(struct.pack('<III', 0, rc | prearm, rc | prearm)),
                         (False, True))
        self.assertEqual(decode_sys_status(struct.pack('<III', rc | prearm, rc | prearm, prearm)),
                         (False, True))
        self.assertEqual(decode_sys_status(b'bad'), (False, False))

    def test_px4_prearm_bit_only_in_health_is_valid(self):
        import struct
        rc, prearm = 1 << 16, 1 << 28
        self.assertEqual(decode_sys_status(struct.pack('<III', rc, rc, rc | prearm)),
                         (True, True))

    def test_other_mode_prearm_is_not_position_clearance(self):
        g = self.guard()
        s = dict(healthy(), mode='STABILIZED')
        g.update(0., s)
        self.assertFalse(g.update(5., s)['ready_to_arm'])
        self.assertIsNone(g.fault)


if __name__ == '__main__':
    unittest.main()
