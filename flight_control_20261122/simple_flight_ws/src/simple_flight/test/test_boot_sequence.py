"""Boot devices may arrive late, but no localization stage is restarted."""
import math
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import json

try:
    from simple_flight.boot_supervisor import next_startup_action, probe_network
except ImportError:
    next_startup_action = None


def available():
    return dict(serial=True, connected=True, state_age=.1, armed=False,
                network=True, lidar_age=.05, imu_age=.01, lio_age=.05,
                parameters_ok=True, lio_stable=True, fault=None, process_failure=None)


class StartupSequenceTests(unittest.TestCase):
    def test_disconnected_network_cable_waits_before_driver_start(self):
        address = dict(flags=['UP'], addr_info=[dict(local='192.168.1.5', prefixlen=24)])
        route = dict(dev='enP8p1s0')
        with patch('simple_flight.boot_supervisor.subprocess.run', side_effect=[
                SimpleNamespace(stdout=json.dumps([address])),
                SimpleNamespace(stdout=json.dumps([route]))]):
            self.assertFalse(probe_network()[0])

    def action(self, started, **changes):
        self.assertIsNotNone(next_startup_action, 'Boot supervisor not implemented')
        return next_startup_action(set(started), dict(available(), **changes))

    def test_serial_and_connected_disarmed_state_precede_sensor_startup(self):
        self.assertIsNone(self.action([], serial=False))
        self.assertEqual(self.action([]), 'mavros')
        for changes in [dict(connected=False), dict(state_age=4.), dict(armed=True)]:
            self.assertIsNone(self.action(['mavros'], **changes))
        self.assertEqual(self.action(['mavros']), 'lidar')

    def test_network_cloud_and_real_imu_required_before_lio(self):
        self.assertIsNone(self.action(['mavros'], network=False))
        for changes in [dict(lidar_age=math.inf), dict(imu_age=math.inf)]:
            self.assertIsNone(self.action(['mavros', 'lidar'], **changes))
        self.assertEqual(self.action(['mavros', 'lidar']), 'fastlio')

    def test_fresh_lio_and_parameter_readback_precede_bridge(self):
        started = ['mavros', 'lidar', 'fastlio']
        self.assertIsNone(self.action(started, lio_age=1.))
        self.assertIsNone(self.action(started, lio_stable=False))
        self.assertEqual(self.action(started, parameters_ok=False), 'check_parameters')
        self.assertEqual(self.action(started), 'bridge')

    def test_fault_and_already_started_bridge_never_restart_any_stage(self):
        started = ['mavros', 'lidar', 'fastlio', 'bridge']
        self.assertIsNone(self.action(started))
        self.assertIsNone(self.action(started, armed=True))
        self.assertIsNone(self.action([], fault='latched'))
        self.assertIsNone(self.action([], process_failure='failed'))


if __name__ == '__main__':
    unittest.main()
