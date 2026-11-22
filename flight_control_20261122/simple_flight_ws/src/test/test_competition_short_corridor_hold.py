"""Competition corridor hold is 0.1s total after arrival, without a 1s prewait."""
import math

import pytest

from test.test_rescue_aisle_workflow import RescueRig
from simple_flight_mission.hover_controller import Observation


@pytest.mark.parametrize('route',[1,2,3,4])
def test_each_corridor_point_leaves_after_short_hold_without_extra_second(route):
    rig=RescueRig(route)
    rig.active()
    ready_at={};elapsed={}
    c=rig.controller
    arrival_check=c._arrived
    def observe_arrival(vehicle):
        ready=arrival_check(vehicle)
        if ready and c.phase=='TRANSIT' and c.point_index in (0,1,3):
            length=math.dist(c.segment_start[:2],c.goal[:2])
            if c.progress>=length-1e-9:
                ready_at.setdefault(c.point_index,rig.t)
        return ready
    c._arrived=observe_arrival
    for _ in range(4000):
        c=rig.controller
        old_phase,old_index=c.phase,c.point_index
        out=rig.advance()
        assert out.phase not in ('HANDOVER','EXITED'),out
        if old_phase=='POINT_HOLD' and old_index in (0,1,3) and out.phase=='TRANSIT':
            assert old_index in ready_at
            elapsed[old_index]=rig.t-ready_at[old_index]
            assert .09<=elapsed[old_index]<=.200001,(route,old_index,elapsed[old_index])
        if len(elapsed)==3:return
    pytest.fail(('Did not complete all three corridor holds',elapsed))


@pytest.mark.parametrize('fault',['position','velocity','heading'])
def test_short_hold_resets_when_arrival_conditions_are_lost(fault):
    rig=RescueRig(1);rig.active()
    for _ in range(1000):
        out=rig.advance()
        if out.phase=='POINT_HOLD' and out.point_index==0:break
    else:pytest.fail('Did not reach first corridor hold')
    for _ in range(6):
        changes={}
        if fault=='position':
            x,y,z,yaw,extra=rig.pose
            changes['pose']=Observation((x+.2,y,z,yaw,extra),rig.t+.05,0.)
        elif fault=='velocity':
            changes['velocity']=Observation((.2,0.,0.),rig.t+.05,0.)
        else:rig.heading_offset=math.radians(6)
        out=rig.tick(**changes)
        assert out.phase=='POINT_HOLD' and out.point_index==0
        assert rig.controller._hold_at is None
    rig.heading_offset=0.
    out=rig.advance();recovered=rig.t
    assert out.phase=='POINT_HOLD'
    for _ in range(5):
        out=rig.advance()
        if out.phase=='TRANSIT':
            assert rig.t-recovered>=.099999
            return
    pytest.fail('Did not leave after fresh continuous short hold')


@pytest.mark.parametrize('phase,index,returning',[
    ('TRANSIT',2,False),('RETURN',0,True),('RETURN_HOLD',None,True)])
def test_task_zones_and_return_keep_the_original_arrival_confirmation(phase,index,returning):
    rig=RescueRig(1);c=rig.controller
    c.origin=rig.pose[:4]
    c.goal=rig.pose[:4]
    c.phase=phase;c.point_index=index;c.returning=returning
    c.external_heading=Observation(rig.pose[3],0.,0.)
    assert not c._stable_arrival(rig.vehicle(),0.)
    assert not c._stable_arrival(rig.vehicle(),.2)
    assert c._stable_arrival(rig.vehicle(),1.)
