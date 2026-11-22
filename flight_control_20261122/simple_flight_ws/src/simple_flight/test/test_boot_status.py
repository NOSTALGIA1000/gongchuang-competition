import unittest
from unittest.mock import patch
from contextlib import redirect_stdout
import io
import json
try:
    from simple_flight.boot_status import validate_status
except ImportError:
    validate_status = None


class StatusTests(unittest.TestCase):
    def test_fault_is_visible_even_when_vehicle_is_armed(self):
        from simple_flight import boot_status
        data = dict(boot_id='boot', run_id='run', pid=123, armed=True,
                    mode='POSCTL', fault='localization lost', reasons=['localization lost'],
                    ready_to_arm=False, updated_monotonic=10., valid_until_monotonic=13.)
        output = io.StringIO()
        with patch.object(boot_status.Path, 'read_text', side_effect=[json.dumps(data), 'boot']), \
                patch.object(boot_status.time, 'monotonic', return_value=11.), \
                patch.object(boot_status, '_alive', return_value=True), redirect_stdout(output):
            with self.assertRaises(SystemExit):
                boot_status.main([])
        self.assertIn('localization lost', output.getvalue())

    def check(self, **changes):
        self.assertIsNotNone(validate_status)
        data = dict(boot_id='boot', run_id='run', pid=123,
                    updated_monotonic=10., valid_until_monotonic=13.)
        data.update(changes)
        return validate_status(data, 'boot', 11., lambda pid: pid == 123)

    def test_fresh_process_and_boot_required(self):
        self.assertIsNone(self.check())
        self.assertIsNotNone(self.check(boot_id='old'))
        self.assertIsNotNone(self.check(pid=456))

    def test_expired_or_future_status_rejected(self):
        self.assertIsNotNone(self.check(valid_until_monotonic=10.5))
        self.assertIsNotNone(self.check(updated_monotonic=12.))

    def test_missing_fields_rejected(self):
        self.assertIsNotNone(self.check(run_id=None))
        self.assertIsNotNone(self.check(updated_monotonic=float('nan')))
