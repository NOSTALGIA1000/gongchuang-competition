import pytest

from simple_flight_mission.state import MissionFSM


def test_route_progresses_only_through_hold_then_ack():
    fsm = MissionFSM(route_length=2)
    assert fsm.state == 'IDLE'
    fsm.start_route()
    assert fsm.state == 'RUNNING' and fsm.target_index == 0
    fsm.report_reached(0)
    assert fsm.state == 'HOLDING'
    fsm.hold_elapsed = 2.0
    fsm.ack_hold()
    assert fsm.target_index == 1 and fsm.state == 'RUNNING'


def test_fault_latches_and_blocks_restart():
    fsm = MissionFSM(route_length=2)
    fsm.start_route()
    fsm.abort('position lost')
    assert fsm.state == 'ABORTING'
    with pytest.raises(RuntimeError):
        fsm.start_route()
    assert not fsm.can_resume_without_operator_reset()
