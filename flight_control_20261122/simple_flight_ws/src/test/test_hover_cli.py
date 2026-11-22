"""CLI and configuration are usable before importing ROS."""
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from simple_flight_mission.hover_node import build_parser, load_hover_config, parse_options, estimator_valid


def test_default_is_check_and_unsafe_options_are_mutually_exclusive():
    assert parse_options([]).mode == "check"
    assert parse_options(["--mode-test"]).mode == "mode-test"
    assert parse_options(["--execute"]).mode == "execute"
    with pytest.raises(SystemExit):
        parse_options(["--mode-test", "--execute"])
    with pytest.raises(SystemExit):
        parse_options(["--execute", "--duration", "1"])


def test_bounded_readonly_duration_must_be_positive():
    assert parse_options(["--duration", "10"]).duration == 10
    for value in ("0", "-1", "nan", "inf"):
        with pytest.raises(SystemExit):
            parse_options(["--duration", value])


def test_config_rejects_unknown_or_invalid_fields(tmp_path):
    config = tmp_path / "hover.yaml"
    config.write_text("height_m: 1.0\nclimb_mps: 0.2\n", encoding="utf-8")
    assert load_hover_config(config).height_m == 1.0
    config.write_text("route: [1, 2]\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_hover_config(config)
    config.write_text("height_m: .nan\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_hover_config(config)


def test_describe_does_not_import_ros_or_connect_to_vehicle():
    completed = subprocess.run(
        [sys.executable, "-m", "simple_flight_mission.hover_node", "--describe"],
        cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout)
    assert result["mode"] == "check"
    assert result["config"]["height_m"] == 1.0
    assert result["rc_channels"] == {"swa": 5, "throttle": 3, "main_mode": 7}


def test_px4_at_rest_constant_position_bit_does_not_reject_valid_ground_aiding():
    flags = SimpleNamespace(attitude_status_flag=True, velocity_horiz_status_flag=True,
                            velocity_vert_status_flag=True, pos_horiz_rel_status_flag=True,
                            pos_vert_abs_status_flag=True, pred_pos_horiz_rel_status_flag=True,
                            const_pos_mode_status_flag=True, accel_error_status_flag=False)
    assert estimator_valid(flags)
    flags.pred_pos_horiz_rel_status_flag = False
    assert not estimator_valid(flags)
    flags.pred_pos_horiz_rel_status_flag = True
    flags.accel_error_status_flag = True
    assert not estimator_valid(flags)
