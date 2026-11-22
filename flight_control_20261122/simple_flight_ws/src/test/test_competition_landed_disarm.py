"""Normal disarm is one-shot, after fresh stable home landing, never forced."""
from types import SimpleNamespace

import pytest

from simple_flight_mission.competition_node import CompetitionNode
from simple_flight_mission.hover_controller import Observation
from test.test_competition_search_departure import competition_rig


def landed_rig():
    rig=competition_rig();rig.active()
    c=rig.controller
    c.phase='LAND_WAIT';c._landing_at=rig.t
    c.goal=(*c.origin[:2],c.origin[2]+1.,c.origin[3])
    c._airborne_seen=True;c.returning=True;c.point_index=None
    rig.landed=1
    return rig


def collect_request(rig,ticks=16,**changes):
    requests=[]
    for _ in range(ticks):
        out=rig.tick(**changes)
        request=rig.controller.take_landed_disarm_request()
        if request:requests.append(request)
        assert not out.arm
    return requests


def test_stable_landing_requests_once_and_ack_does_not_fake_disarmed_feedback():
    rig=landed_rig()
    assert collect_request(rig,8)==[]
    assert collect_request(rig,10)==[True]
    assert collect_request(rig,30)==[]
    rig.controller.disarm_ack_accepted=True
    assert not rig.controller.mission_completed
    assert rig.tick().phase=='LAND_WAIT'
    rig.armed=False
    assert rig.tick().exit_reason=='mission_complete'
    assert rig.controller.mission_completed


def test_bounce_resets_ground_confirmation_before_any_disarm():
    rig=landed_rig()
    assert collect_request(rig,7)==[]
    rig.landed=2
    assert collect_request(rig,2)==[]
    rig.landed=1
    assert collect_request(rig,8)==[]
    assert collect_request(rig,10)==[True]


@pytest.mark.parametrize('fault',['airborne','above_ground','outside_home','moving','stale_ground','stale_pose','stale_state','manual','no_flight','check'])
def test_invalid_or_nonlanding_evidence_never_requests_disarm(fault):
    rig=landed_rig();c=rig.controller
    for _ in range(18):
        changes={}
        if fault=='airborne':rig.landed=2
        elif fault=='above_ground':rig.pose=(*rig.pose[:2],c.origin[2]+.25,*rig.pose[3:])
        elif fault=='outside_home':rig.pose=(c.origin[0]+.2,*rig.pose[1:])
        elif fault=='moving':changes['velocity']=Observation((.2,0.,0.),rig.t+.05,0.)
        elif fault=='stale_ground':changes['landed']=Observation(1,rig.t-2.)
        elif fault=='stale_pose':changes['pose']=Observation(rig.pose,rig.t-1.,0.)
        elif fault=='stale_state':changes['state']=Observation(rig.vehicle().state.value,rig.t-2.)
        elif fault=='manual':rig.mode='POSCTL'
        elif fault=='no_flight':c._airborne_seen=False
        else:c.mode='check';c.phase='CHECK'
        rig.tick(**changes)
        assert c.take_landed_disarm_request() is None


def ready_request():
    rig=landed_rig()
    for _ in range(14):rig.tick()
    return rig


def fake_node(rig,available=True):
    sent=[];events=[]
    future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=True,result=0))
    client=SimpleNamespace(service_is_ready=lambda:available,call_async=lambda request:sent.append(request.value) or future)
    node=CompetitionNode.__new__(CompetitionNode)
    node.controller=rig.controller;node.snapshot=rig.vehicle
    node.arm_client=client;node.pending={};node.disarm_pending=None;node.disarm_submitted=False
    node.CommandBool=SimpleNamespace(Request=lambda:SimpleNamespace(value=None))
    node.command_events=[];node._event=lambda kind,data:events.append((kind,data))
    node.node=SimpleNamespace(get_logger=lambda:SimpleNamespace(info=lambda s:None,warning=lambda s:None))
    return node,sent,events


def test_adapter_submits_only_normal_false_once_and_keeps_actual_state_authoritative():
    rig=ready_request();node,sent,events=fake_node(rig)
    node._disarm_update(rig.t)
    assert sent==[False]
    node._disarm_update(rig.t+.05)
    node._disarm_update(rig.t+.1)
    assert sent==[False] and rig.controller.disarm_ack_accepted is True
    assert not rig.controller.mission_completed


@pytest.mark.parametrize('fault',['manual','airborne','service_unavailable'])
def test_adapter_rechecks_evidence_and_never_queues_a_stale_disarm(fault):
    rig=ready_request();node,sent,events=fake_node(rig,available=fault!='service_unavailable')
    if fault=='manual':rig.mode='POSCTL'
    elif fault=='airborne':rig.landed=2
    node._disarm_update(rig.t)
    node._disarm_update(rig.t+.05)
    assert sent==[]


@pytest.mark.parametrize('reply',['rejected','timeout','exception'])
def test_failed_or_unknown_disarm_never_retries_or_declares_completion(reply):
    rig=ready_request();node,sent,events=fake_node(rig)
    def submit(request):
        sent.append(request.value)
        if reply=='exception':raise OSError('offline fake failure')
        return SimpleNamespace(done=lambda:reply!='timeout',result=lambda:SimpleNamespace(success=False))
    node.arm_client.call_async=submit
    node._disarm_update(rig.t)
    node._disarm_update(rig.t+2.1)
    node._disarm_update(rig.t+2.2)
    assert sent==[False] and rig.controller.disarm_ack_accepted is False
    assert not rig.controller.mission_completed


def test_fcu_auto_disarmed_first_needs_no_redundant_request():
    rig=landed_rig()
    assert collect_request(rig,4)==[]
    rig.armed=False
    assert rig.tick().exit_reason=='mission_complete'
    assert rig.controller.take_landed_disarm_request() is None
