"""Offline checks for channel isolation and uncalibrated door commands."""
import contextlib
import io
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from types import SimpleNamespace

from simple_flight_payload.door import (
    build_command, command_request, ground_test_request, main, validate_output)


def calibration():
    return {"doors": {str(n): {"min_us": 1000, "max_us": 2000,
                              "open_us": None, "close_us": None} for n in (1, 2)}}


class DoorTests(unittest.TestCase):
    def test_uncalibrated_open_and_close_are_rejected(self):
        for door in (1, 2):
            for action in ("open", "close"):
                with self.subTest(door=door, action=action), self.assertRaisesRegex(ValueError, "未标定"):
                    build_command(calibration(), door, action)

    def test_one_door_changes_only_its_own_actuator(self):
        for door, pwm in ((1, 1000), (2, 2000)):
            command = build_command(calibration(), door, "set", pwm)
            self.assertEqual(command.aux, door + 6)
            self.assertEqual(command.params[door - 1], -1.0 if door == 1 else 1.0)
            self.assertEqual(command.params[6], 0.0)
            self.assertTrue(all(math.isnan(v) for i, v in enumerate(command.params[:6]) if i != door - 1))

    def test_each_door_uses_its_own_measured_open_and_close(self):
        config = calibration()
        config["doors"]["1"].update(open_us=1800, close_us=1300)
        config["doors"]["2"].update(open_us=1200, close_us=1700)
        self.assertAlmostEqual(build_command(config, 1, "open").params[0], .6)
        self.assertAlmostEqual(build_command(config, 2, "close").params[1], .4)

    def test_ros_request_preserves_untouched_channels_and_uses_set_actuator(self):
        command = build_command(calibration(), 2, "set", 1600)
        request = command_request(command, SimpleNamespace)
        self.assertEqual(request.command, 187)
        self.assertFalse(request.broadcast)
        self.assertEqual(request.confirmation, 0)
        self.assertTrue(math.isnan(request.param1))
        self.assertAlmostEqual(request.param2, .2)
        self.assertEqual(request.param7, 0.)

    def test_ground_test_only_addresses_qgc_peripheral_functions_with_timeout(self):
        for door in (1, 2):
            request = ground_test_request(build_command(calibration(), door, "set", 1500), SimpleNamespace)
            self.assertEqual(request.command, 310)
            self.assertEqual(request.param1, 0.)
            self.assertEqual(request.param2, 2.)
            self.assertEqual(request.param5, 1300 + door)
            self.assertEqual((request.param3, request.param4, request.param6, request.param7), (0., 0., 0., 0.))

    def test_invalid_channels_pulses_and_extra_pwm_are_rejected(self):
        for door, action, pwm in ((3, "set", 1500), (True, "set", 1500),
                                 (1, "set", 2001), (1, "set", float("nan")),
                                 (1, "set", True), (1, "open", 1500)):
            with self.subTest(door=door, action=action, pwm=pwm), self.assertRaises(ValueError):
                build_command(calibration(), door, action, pwm)

    def test_wrong_mapping_range_or_reversal_prevents_send(self):
        command = build_command(calibration(), 2, "set", 1500)
        params = dict(PWM_AUX_FUNC8=302, PWM_AUX_MIN8=1000, PWM_AUX_MAX8=2000, PWM_AUX_REV=0)
        validate_output(command, params)
        for changes in (dict(PWM_AUX_FUNC8=101), dict(PWM_AUX_MAX8=1900), dict(PWM_AUX_REV=128)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_output(command, dict(params, **changes))

    def test_preview_never_creates_ros_client(self):
        stream = io.StringIO()
        with patch("simple_flight_payload.door.send_command", side_effect=AssertionError("no I/O")), contextlib.redirect_stdout(stream):
            self.assertEqual(main(["1", "set", "--pwm", "1500"]), 0)
        result = json.loads(stream.getvalue())
        self.assertEqual(result["mode"], "preview")
        self.assertEqual(result["aux"], 7)
        self.assertIsNone(result["params"][1])

    def test_user_supplied_angles_use_explicit_standard_mapping(self):
        path = Path(__file__).resolve().parents[1] / "config" / "doors.json"
        config = json.loads(path.read_text(encoding="utf-8"))
        for door in (1, 2):
            self.assertEqual(build_command(config, door, "open").pwm_us, 1000)
            self.assertEqual(build_command(config, door, "close").pwm_us, 2000)

    def test_open_ground_preview_is_home180_open0_close180(self):
        stream = io.StringIO()
        with patch("simple_flight_payload.door.send_command", side_effect=AssertionError("no I/O")), \
                contextlib.redirect_stdout(stream):
            self.assertEqual(main(["1", "open", "--ground-test"]), 0)
        result = json.loads(stream.getvalue())
        self.assertEqual(result["command"], 310)
        self.assertEqual(result["sequence"], ["home_180", "open_0", "close_180"])
        self.assertEqual(result["home_pwm_us"], 2000)
        self.assertEqual(result["open_pwm_us"], 1000)


if __name__ == "__main__":
    unittest.main()
