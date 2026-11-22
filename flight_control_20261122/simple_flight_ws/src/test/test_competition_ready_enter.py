"""Enter authorizes OFFBOARD only after completed ground preparation."""
from dataclasses import replace

import pytest

from simple_flight_mission.competition_controller import CompetitionController
from simple_flight_mission.hover_controller import Observation
from test.test_competition_search_departure import competition_rig
from test.test_rescue_aisle_workflow import RescueRig


def enter_rig():
    rig=competition_rig()
    old=rig.controller
    rig.controller=CompetitionController('execute',old.route_config,old.doors,'notebook',require_enter=True)
    rig.controller.payload_initialized=True
    return rig


def test_ready_keeps_ground_stream_without_mode_or_arm_until_enter():
    rig=enter_rig();rig.ready()
    for _ in range(100):
        out=rig.tick()
        assert out.phase=='READY' and out.request_mode is None and not out.arm
        assert out.target==rig.controller.origin
    assert rig.controller.confirm_start(rig.vehicle(),rig.t)
    out=rig.tick()
    assert out.request_mode=='OFFBOARD' and not out.arm
    rig.mode='OFFBOARD'
    assert rig.tick().arm


def test_swa_edge_cannot_bypass_the_enter_requirement():
    rig=enter_rig();rig.ready()
    for value in [2003]*10+[1024]*10+[2003]*10:
        rig.rc[6]=value
        out=rig.tick()
        assert out.phase=='READY' and out.request_mode is None and not out.arm


def test_confirmation_before_ready_or_after_stale_state_is_rejected():
    rig=enter_rig()
    assert not rig.controller.confirm_start(rig.vehicle(),rig.t)
    rig.ready()
    bad=replace(rig.vehicle(),state=Observation(dict(connected=True,armed=False,mode='POSCTL'),rig.t-2.))
    assert not rig.controller.confirm_start(bad,rig.t)
    assert not rig.controller.enter_confirmed
    assert rig.tick().phase=='READY'


def test_long_ready_wait_does_not_consume_the_competition_budget():
    rig=enter_rig();rig.ready()
    for _ in range(4000):
        out=rig.tick()
        assert out.phase=='READY' and out.request_mode is None and not out.arm
    assert rig.controller.mission_elapsed(rig.t)==0.
    assert rig.controller.confirm_start(rig.vehicle(),rig.t)
    confirmed_at=rig.t
    assert not rig.controller.confirm_start(rig.vehicle(),rig.t+1.)
    assert rig.controller.competition_started_at==confirmed_at
    assert rig.tick().request_mode=='OFFBOARD'


def test_enter_cli_rejects_read_only_mode_without_initializing_ros():
    from simple_flight_mission.competition_node import main
    with pytest.raises(SystemExit) as error:
        main(['--route','1','--check','--wait-enter'])
    assert error.value.code==2


def test_enter_cli_rejects_closed_noninteractive_input_before_ros(monkeypatch):
    from types import SimpleNamespace
    from simple_flight_mission import competition_node
    monkeypatch.setattr(competition_node.sys,'stdin',SimpleNamespace(isatty=lambda:False))
    with pytest.raises(SystemExit) as error:
        competition_node.main(['--route','1','--execute','--wait-enter'])
    assert error.value.code==2


@pytest.mark.parametrize('route',[1,2,3,4])
def test_completed_rescue_returns_after_stable_hold_at_waypoint_five(route):
    rig=RescueRig(route,person=True);rig.reach_rescue()
    stable_since = hold_s = None
    for _ in range(1600):
        if rig.controller.task.phase == 'DONE':
            stable_since = rig.controller._hold_at
            hold_s = rig.controller.route_config.points[rig.controller.point_index].hold_s
        rig.advance()
        if rig.controller.returning:
            assert stable_since is not None
            if route == 1:
                # Route 1 honours the configured dwell at the scoring waypoint.
                assert hold_s - .05 <= rig.t - stable_since <= hold_s + .60
            else:
                assert 3. < rig.t - stable_since <= 3.06
            return
    pytest.fail('Rescue did not finish through waypoint 5')
