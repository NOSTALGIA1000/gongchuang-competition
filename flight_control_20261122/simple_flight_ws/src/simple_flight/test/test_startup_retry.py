"""Only pre-aiding, disarmed startup recovery may replace a LIO generation."""
import unittest

try:
    from simple_flight.startup_retry import StartupRetryGate, expected_startup_exit
except ImportError:
    StartupRetryGate = expected_startup_exit = None


def eligible(**changes):
    snapshot = dict(process_alive=True, connected=True, state_age=.1, armed=False,
                    network=True, lidar_age=.05, imu_age=.01,
                    armed_ever_seen=False, bridge_ever_started=False,
                    external_pose_ever_observed=False, lio_age=1.4)
    snapshot.update(changes)
    return snapshot


class StartupRetryTests(unittest.TestCase):
    def test_expected_exit_stays_expected_while_spawn_request_is_pending(self):
        self.assertIsNotNone(expected_startup_exit)
        self.assertTrue(expected_startup_exit('STOPPING', 42, 42))
        self.assertTrue(expected_startup_exit('SPAWN_REQUESTED', 42, 42))
        self.assertFalse(expected_startup_exit('RUNNING', 42, 42))
        self.assertFalse(expected_startup_exit('STOPPING', 42, 43))
        self.assertFalse(expected_startup_exit('SPAWN_REQUESTED', None, 42))

    def gate(self):
        self.assertIsNotNone(StartupRetryGate, 'Startup-only retry gate not implemented')
        gate = StartupRetryGate(wait_s=15., max_retries=2, stop_timeout_s=10.)
        gate.spawned(0.)
        return gate

    def test_waits_fifteen_seconds_then_requests_one_stop(self):
        gate = self.gate()
        self.assertIsNone(gate.next_action(14.9, eligible()))
        self.assertEqual(gate.next_action(15., eligible()), 'stop')
        self.assertEqual(gate.retries, 1)
        self.assertIsNone(gate.next_action(16., eligible()))

    def test_sticky_safety_boundaries_and_fresh_lio_prevent_retry(self):
        for changes in [dict(armed=True), dict(armed_ever_seen=True),
                        dict(connected=False), dict(state_age=3.1),
                        dict(network=False), dict(lidar_age=.4), dict(imu_age=.3),
                        dict(bridge_ever_started=True),
                        dict(external_pose_ever_observed=True), dict(initialized_seen=True),
                        dict(lio_age=.1)]:
            gate = self.gate()
            self.assertIsNone(gate.next_action(20., eligible(**changes)), str(changes))
            self.assertEqual(gate.retries, 0)

    def test_exit_must_be_confirmed_then_fresh_unarmed_state_rechecked(self):
        gate = self.gate()
        self.assertEqual(gate.next_action(15., eligible()), 'stop')
        self.assertIsNone(gate.next_action(16., eligible()))
        self.assertIsNone(gate.next_action(17., eligible(process_alive=False, state_age=5.)))
        self.assertIsNone(gate.next_action(17.5, eligible(process_alive=False, network=False)))
        self.assertEqual(gate.next_action(18., eligible(process_alive=False)), 'spawn')
        self.assertIsNone(gate.next_action(18.1, eligible(process_alive=False, state_age=4.)))
        self.assertEqual(gate.next_action(18.2, eligible(process_alive=False)), 'spawn')
        gate.spawned(19.)
        self.assertIsNone(gate.next_action(33.9, eligible()))
        self.assertEqual(gate.next_action(34., eligible()), 'stop')

    def test_short_stale_sample_after_healthy_period_does_not_restart(self):
        gate = self.gate()
        self.assertIsNone(gate.next_action(14., eligible(lio_age=.1)))
        self.assertIsNone(gate.next_action(16., eligible(lio_age=.31)))
        self.assertEqual(gate.next_action(29., eligible()), 'stop')

    def test_stop_timeout_latches_and_never_spawns_second_instance(self):
        gate = self.gate()
        gate.next_action(15., eligible())
        self.assertIsNone(gate.next_action(25.1, eligible()))
        self.assertIsNotNone(gate.fault)
        self.assertIsNone(gate.next_action(30., eligible(process_alive=False)))

    def test_at_most_two_retry_generations(self):
        gate = self.gate()
        for now in (15., 31.):
            self.assertEqual(gate.next_action(now, eligible()), 'stop')
            self.assertEqual(gate.next_action(now + .5, eligible(process_alive=False)), 'spawn')
            gate.spawned(now + 1.)
        self.assertEqual(gate.retries, 2)
        self.assertIsNone(gate.next_action(48., eligible()))
        self.assertIsNotNone(gate.fault)

    def test_sticky_boundary_arriving_during_stop_forbids_respawn(self):
        for key in ('armed_ever_seen', 'bridge_ever_started', 'external_pose_ever_observed'):
            gate = self.gate()
            gate.next_action(15., eligible())
            self.assertIsNone(gate.next_action(16., eligible(process_alive=False, **{key: True})))


if __name__ == '__main__':
    unittest.main()
