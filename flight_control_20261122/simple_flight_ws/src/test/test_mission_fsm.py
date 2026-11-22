import pytest

from simple_flight_mission.state import MissionFSM


def test_route_advances_only_through_hold_cycles():
    fsm = MissionFSM(3)
    fsm.start_route()
    for idx in range(2):
        assert fsm.state == "RUNNING"
        assert fsm.target_index == idx
        fsm.report_reached(idx)
        fsm.hold_elapsed = 2.5
        fsm.ack_hold()
    fsm.report_reached(2)
    assert fsm.state == "HOLDING"

def test_out_of_sequence_reached_rejected():
    fsm = MissionFSM(2)
    fsm.start_route()
    with pytest.raises(RuntimeError):
        fsm.report_reached(1)

def test_abort_latches_and_requires_operator_reset():
    fsm = MissionFSM(2)
    fsm.start_route()
    fsm.abort("guard: position stale")
    assert fsm.state == "ABORTING"
    assert fsm.abort_reason == "guard: position stale"
    assert fsm.can_resume_without_operator_reset() is False

def test_hold_before_required_time_rejected():
    fsm = MissionFSM(1)
    fsm.start_route()
    fsm.report_reached(0)
    with pytest.raises(RuntimeError):
        fsm.ack_hold()
