"""Recording labels must survive the Nano's incorrect/revised wall clock."""
import importlib.util
from pathlib import Path

import pytest


@pytest.fixture
def naming():
    path = Path(__file__).parents[1] / "scripts/recording_name.py"
    spec = importlib.util.spec_from_file_location("recording_name", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_computer_time_uses_shanghai_and_elapsed_time(naming):
    ref = {"boot_id": "boot-a", "uptime_s": 100., "computer_unix_s": 1793368800.}
    assert naming.recording_stamp(ref, "boot-a", 165., 0.) == ("20261030_220105", True)


def test_board_clock_jump_does_not_change_calibrated_label(naming):
    ref = {"boot_id": "boot-a", "uptime_s": 100., "computer_unix_s": 1793368800.}
    assert naming.recording_stamp(ref, "boot-a", 165., 2000000000.) == naming.recording_stamp(ref, "boot-a", 165., 0.)


@pytest.mark.parametrize("ref", [None, {}, {"boot_id": "boot-old", "uptime_s": 100., "computer_unix_s": 1793368800.},
                                      {"boot_id": "boot-a", "uptime_s": 200., "computer_unix_s": 1793368800.}])
def test_missing_or_previous_boot_reference_is_explicitly_unsynced(naming, ref):
    stamp, calibrated = naming.recording_stamp(ref, "boot-a", 165., 1793368800.)
    assert stamp == "20261030_220000"
    assert calibrated is False
