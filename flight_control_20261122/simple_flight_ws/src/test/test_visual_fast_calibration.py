"""Speed-first user policy, retaining fresh frames and measurable direction."""
import pytest

from simple_flight_mission.visual_task import VisualTask
from test.test_competition import frame


@pytest.mark.parametrize('door,kind',[(1,2),(2,0)])
@pytest.mark.parametrize('error',[12.,20.35,24.])
def test_nearly_centered_target_skips_calibration_and_opens_immediately(door,kind,error):
    task=VisualTask(door,(0.,0.,1.,0.),[],0.)
    out=task.step((0.,0.,1.,0.,0.),(.09,0.,0.),.1,
                  frame(.1,kind=kind,box=(300+error,220,340+error,260)))
    assert out.action=='open' and out.phase=='OPEN_WAIT'
    assert task.jacobian is None and not task.samples
    assert task.diagnostic(.1)['release_threshold_px']==50


def test_target_outside_relaxed_release_radius_still_needs_alignment():
    task=VisualTask(1,(0.,0.,1.,0.),[],0.)
    out=task.step((0.,0.,1.,0.,0.),(0.,0.,0.),.1,
                  frame(.1,box=(350.01,220,390.01,260)))
    assert out.action is None and not task.open_sent


def probe():
    task=VisualTask(1,(0.,0.,1.,0.),[],0.)
    task.phase='PROBE_X';task.sample_origin=task.anchor
    task.samples=[((0.,0.),(360.,240.))]
    task.desired=task.commanded=(.08,0.,1.,0.)
    return task


def test_one_new_frame_after_100ms_at_moderate_speed_finishes_probe():
    task=probe();pose=(.08,0.,1.,0.,0.)
    assert task.step(pose,(.09,0.,0.),.1,frame(.1,box=(372,220,412,260))).phase=='PROBE_X'
    out=task.step(pose,(.09,0.,0.),.201,frame(.201,box=(372,220,412,260)))
    assert out.phase=='PROBE_Y' and len(task.samples)==2
    assert task.samples[-1][0]==(.08,0.)


def test_probe_still_rejects_a_frame_captured_before_the_sample_window():
    task=probe();pose=(.08,0.,1.,0.,0.)
    task.step(pose,(.09,0.,0.),.1,frame(.1,box=(372,220,412,260)))
    old=dict(frame(.21,box=(372,220,412,260)),age_s=.08)
    assert task.step(pose,(.09,0.,0.),.21,old).phase=='PROBE_X'
    assert len(task.samples)==1


def test_fast_probe_does_not_accept_excessive_motion_or_unfinished_command():
    task=probe();pose=(.08,0.,1.,0.,0.)
    for i in range(1,6):
        now=i*.1
        assert task.step(pose,(.11,0.,0.),now,frame(now,box=(372,220,412,260))).phase=='PROBE_X'
    assert len(task.samples)==1
    task=probe();task.commanded=(0.,0.,1.,0.)
    task.step(pose,(0.,0.,0.),.1,frame(.1,box=(372,220,412,260)))
    assert task.wait_reason=='command_ramping' and len(task.samples)==1


@pytest.mark.parametrize('changes',[dict(source='image_replay'),dict(valid=False),dict(age_s=1.)])
def test_relaxed_release_still_rejects_unusable_frames(changes):
    task=VisualTask(1,(0.,0.,1.,0.),[],0.)
    image=dict(frame(.1,box=(320,220,360,260)),**changes)
    assert task.step((0.,0.,1.,0.,0.),(0.,0.,0.),.1,image).action is None
