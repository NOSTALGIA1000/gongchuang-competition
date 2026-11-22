"""The 35cm task-zone limit belongs to hazardous delivery only."""
import math

import pytest

from simple_flight_mission.visual_task import VisualTask
from test.test_competition import frame


@pytest.mark.parametrize('initial_phase',['SEARCH','SCAN','ALIGN'])
def test_personnel_target_outside_35cm_can_be_approached_dropped_and_returned(initial_phase):
    anchor=(2.87,2.7,1.,0.)
    task=VisualTask(2,anchor,[(1.2,2.7,1.,0.)],0.,jacobian=((160.,0.),(0.,160.)))
    task.phase=initial_phase
    actual=(*anchor,0.)
    person=(2.4,3.5)
    actions=[];pending=None;left_aisle=False
    for i in range(1,901):
        now=i*.05
        u,v=320+160*(actual[0]-person[0]),240+160*(actual[1]-person[1])
        snapshot=frame(now,kind=0,box=(u-12,v-12,u+12,v+12))
        ack=pending;pending=None
        out=task.step(actual,(0.,0.,0.),now,snapshot,ack)
        assert out.failed is None,(out,task.diagnostic(now))
        assert math.dist(out.target[:2],actual[:2])<=.010001
        if out.action:
            actions.append(out.action);pending=(out.action,True)
            if out.action=='open':assert math.hypot(u-320,v-240)<=50.
        if actual[1]>3.05:left_aisle=True
        actual=(*out.target,0.)
        if out.done:
            assert left_aisle and actions==['open','close']
            assert math.dist(actual[:2],anchor[:2])<=task.arrival_xy_m
            return
    pytest.fail('Personnel did not complete delivery and return to waypoint 5')


@pytest.mark.parametrize('phase',['PROBE_X','PROBE_Y','PROBE_RECOVER','CALIB_RETURN','ALIGN'])
def test_personnel_actual_displacement_beyond_35cm_is_not_a_task_bound_failure(phase):
    task=VisualTask(2,(0.,0.,1.,0.),[],0.,jacobian=((160.,0.),(0.,160.)))
    task.phase=phase
    task.sample_origin=task.anchor
    task.samples=[((0.,0.),(360.,240.))]
    out=task.step((.5,0.,1.,0.,0.),(0.,0.,0.),.1,frame(.1,kind=0))
    assert out.failed is None and out.action=='open'


@pytest.mark.parametrize('phase',['PROBE_X','PROBE_Y','PROBE_RECOVER','CALIB_RETURN','ALIGN'])
def test_hazardous_actual_displacement_beyond_35cm_still_blocks_release(phase):
    task=VisualTask(1,(0.,0.,1.,0.),[],0.,jacobian=((160.,0.),(0.,160.)))
    task.phase=phase
    out=task.step((.5,0.,1.,0.,0.),(0.,0.,0.),.1,frame(.1))
    assert out.failed=='visual_alignment_bound' and out.action is None
