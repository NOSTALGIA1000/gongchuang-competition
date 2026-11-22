"""Verify offline replay doesn't mistake a simulation for real takeover."""

import json
from pathlib import Path
import subprocess
import sys

import pytest

from simple_flight_mission.hover_replay import main, replay


def row(t, stream, data):
    return {"elapsed_s": t, "stream": stream, "data": data}


def rc(t, throttle=1024, swa=2003):
    return row(t, "rc", {"channels": [1514, 1514, throttle, 1514, swa]})


def ground_ready():
    return [
        row(0.0, "state", {"mode": "POSCTL", "armed": False,
                           "connected": True, "manual_input": True}),
        row(0.0, "extended", {"landed_state": 1}),
        row(0.0, "boot", {"ready_to_arm": True}),
        rc(0.0, swa=1024), rc(0.25, swa=1024), rc(0.30),
    ]


def test_default_replay_does_not_invent_offboard_from_rc_intent():
    result = replay(ground_ready() + [rc(0.4, 1514), rc(0.65, 1514), rc(0.7), rc(0.85)])
    assert result["simulated_offboard"] is False
    assert result["offboard_confirmations"] == []
    assert [e["action"] for e in result["events"]] == ["start"]
    assert result["final_phase"] == "REQUESTED"


def test_explicit_simulation_can_validate_throttle_exit_and_is_labeled():
    result = replay(ground_ready() + [rc(0.4, 1514), rc(0.65, 1514), rc(0.7), rc(0.85)],
                    simulated_offboard_at=0.31)
    assert result["simulated_offboard"] is True
    assert result["offboard_confirmations"] == [{"elapsed_s": 0.31, "source": "simulation"}]
    assert [(e["action"], e["reason"]) for e in result["events"]] == [
        ("start", "swa_low_then_high"), ("exit", "throttle_raise_then_low")]


def test_actual_offboard_requires_recorded_mode_feedback():
    actual_mode = row(0.31, "state", {"mode": "OFFBOARD", "armed": False,
                                     "connected": True, "manual_input": True})
    result = replay(ground_ready() + [actual_mode, rc(0.4, swa=1024)])
    assert result["offboard_confirmations"] == [{"elapsed_s": 0.31, "source": "recorded_mode"}]
    assert result["events"][-1]["reason"] == "swa_low"


def test_altered_rc_recording_remains_labeled_as_synthetic_in_result():
    rows = ground_ready()
    rows[-1]["synthetic_fields"] = ["data.channels[6]"]
    assert replay(rows)["synthetic_rc_input"] is True
    assert replay(ground_ready())["synthetic_rc_input"] is False


def test_other_streams_advance_timeout_when_rc_stops():
    result = replay(ground_ready() + [row(0.7, "extended", {"landed_state": 1})],
                    simulated_offboard_at=0.31)
    assert result["events"][-1]["reason"] == "rc_timeout"


def test_missing_or_stale_mode_and_missing_ground_check_cannot_authorize_start():
    assert replay([rc(0.0, swa=1024), rc(0.25, swa=1024), rc(0.3)])["events"] == []
    rows = ground_ready()[:3] + [rc(2.0, swa=1024), rc(2.25, swa=1024), rc(2.3)]
    assert replay(rows)["events"] == []
    rows = [x for x in ground_ready() if x["stream"] != "extended"]
    assert replay(rows)["events"] == []


@pytest.mark.parametrize("at", [0.1, 10.0, -1.0, float("nan")])
def test_impossible_simulation_time_is_rejected(at):
    with pytest.raises(ValueError):
        replay(ground_ready(), simulated_offboard_at=at)


def test_unordered_recording_is_rejected_instead_of_sorting_away_a_problem():
    with pytest.raises(ValueError):
        replay([rc(1.0), rc(0.5)])


def test_cli_runs_without_ros_and_writes_machine_readable_evidence(tmp_path):
    source = tmp_path / "samples.jsonl"
    source.write_text("\n".join(json.dumps(x) for x in ground_ready()), encoding="utf-8")
    out = tmp_path / "replay.json"
    completed = subprocess.run(
        [sys.executable, "-m", "simple_flight_mission.hover_replay", str(source), "--output", str(out)],
        cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr
    assert json.loads(out.read_text(encoding="utf-8"))["final_phase"] == "REQUESTED"
    assert json.loads(completed.stdout)["simulated_offboard"] is False


def test_cli_rejects_overwriting_input_recording(tmp_path):
    source = tmp_path / "samples.jsonl"
    content = "\n".join(json.dumps(x) for x in ground_ready())
    source.write_text(content, encoding="utf-8")
    with pytest.raises(SystemExit) as exc:
        main([str(source), "--output", str(source)])
    assert exc.value.code == 2
    assert source.read_text(encoding="utf-8") == content
