"""Personnel far approach is quicker, retaining hazardous and near policies."""
import math

import pytest

from simple_flight_mission.visual_task import VisualTask
from test.test_competition import frame


def prepared(door,error):
    task=VisualTask(door,(0.,0.,1.,0.),[],0.,jacobian=((400.,0.),(0.,400.)))
    task.phase='ALIGN'
    pose=(0.,0.,1.,0.,0.)
    for i in range(1,4):
        now=i*.05
        task.step(pose,(0.,0.,0.),now,frame(now,kind=0 if door==2 else 2,
                  box=(300+error,220,340+error,260)))
    return task


def test_far_person_generates_a_larger_bounded_approach_step():
    task=prepared(2,120.)
    assert .07 < math.hypot(*task.desired[:2]) <= .080001
    assert math.hypot(*task.commanded[:2]) <= .010001
    assert not task.open_sent


@pytest.mark.parametrize('door,error',[(1,120.),(2,60.)])
def test_hazardous_and_near_person_keep_original_fine_step_and_speed(door,error):
    task=prepared(door,error)
    assert math.hypot(*task.desired[:2]) <= .030001
    assert math.hypot(*task.commanded[:2]) <= .006001
    assert not task.open_sent


def test_far_person_can_refresh_approach_while_following_the_previous_step():
    task=prepared(2,120.)
    task.commanded=task.desired
    old=task.desired
    out=task.step((0.,0.,1.,0.,0.),(.16,0.,0.),.2,
                  frame(.2,kind=0,box=(420,220,460,260)))
    assert task.wait_reason != 'vehicle_moving'
    assert out.failed is None and out.action is None
    assert task.desired==old


def test_far_approach_does_not_accept_a_stale_frame_for_a_new_goal():
    task=prepared(2,120.)
    old=task.desired
    task.commanded=old
    out=task.step((0.,0.,1.,0.,0.),(0.,0.,0.),.2,
                  dict(frame(.2,kind=0,box=(420,220,460,260)),age_s=1.))
    assert task.desired==old and out.action is None


def test_near_person_slows_an_unfinished_far_step_back_to_fine_speed():
    task=prepared(2,120.)
    previous=task.commanded
    out=task.step((0.,0.,1.,0.,0.),(0.,0.,0.),.2,
                  frame(.2,kind=0,box=(360,220,400,260)))
    assert math.dist(previous[:2],out.target[:2])<=.006001
    assert out.action is None


def test_fast_approach_still_waits_when_vehicle_speed_is_excessive():
    task=prepared(2,120.)
    task.commanded=task.desired
    old=task.desired
    task.step((0.,0.,1.,0.,0.),(.26,0.,0.),.2,
              frame(.2,kind=0,box=(420,220,460,260)))
    assert task.desired==old and task.wait_reason=='vehicle_moving'


@pytest.mark.parametrize('rotated',[False,True])
def test_delayed_personnel_delivery_and_return_finishes_faster_than_old_fine_only_policy(rotated):
    from test.test_rescue_delayed_feedback import run_rescue
    task,out,actual,actions,phases,elapsed=run_rescue(-1,.4,.3,.4,rotated)
    assert out.done and actions==['open','close']
    assert elapsed < 15., 'Previous fine-only policy took 18.65s / 16.65s in this virtual scenario'
    assert math.dist(actual,task.anchor[:2])<=task.arrival_xy_m
