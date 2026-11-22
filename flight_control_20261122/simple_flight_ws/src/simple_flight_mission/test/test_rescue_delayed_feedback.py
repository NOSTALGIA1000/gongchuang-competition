"""Delayed camera and tracking feedback contracts; no ROS or device access."""
import math

import pytest

from simple_flight_mission.visual_task import VisualTask


def run_rescue(direction, person_fraction, camera_delay, tracking_tau, rotated=False, measured=True,
               initial_matrix=None):
    start, end = (1.2, 2.87) if direction == 1 else (2.87, 1.2)
    anchor = (start, 2.7, 1., 0.)
    matrix = ((0., -400.), (400., 0.)) if rotated else ((400., 0.), (0., 400.))
    task = VisualTask(2, anchor, [(end, 2.7, 1., 0.)], 0.,
                      (initial_matrix or matrix) if measured else None)
    person = None if person_fraction is None else (start + (end-start)*person_fraction, 2.82)
    actual = list(anchor[:2])
    velocity = (0., 0., 0.)
    history = [(0., tuple(actual))]
    snapshot = None
    actions = []
    phases = set()
    pending = None
    sequence = 0
    for tick in range(1, 1001):
        now = round(tick*.05, 8)
        history.append((now, tuple(actual)))
        if tick % 3 == 0:
            captured = max(0., now-camera_delay)
            a = max(i for i, row in enumerate(history) if row[0] <= captured+1e-9)
            b = min(a+1, len(history)-1)
            ta, pa = history[a]; tb, pb = history[b]
            ratio = 0. if tb == ta else (captured-ta)/(tb-ta)
            pose = tuple(pa[k]+ratio*(pb[k]-pa[k]) for k in range(2))
            detections = []
            if person is not None:
                pixel = tuple((320., 240.)[i] + sum(matrix[i][j]*(pose[j]-person[j]) for j in range(2)) for i in range(2))
                u, v = pixel
                if 20 <= u <= 620 and 20 <= v <= 460:
                    detections = [dict(class_id=0, name='Person', confidence=.9,
                                       box=[u-20, v-20, u+20, v+20])]
            sequence += 1
            snapshot = dict(source='camera', valid=True, frame_fresh=True, enabled=True,
                            received_at=now, age_s=now-captured, sequence=sequence,
                            image_size=[640, 480], detections=detections)
        ack = None
        if pending and now >= pending[0]:
            ack = (pending[1], True); pending = None
        out = task.step((*actual, 1., 0., 0.), velocity, now, snapshot, ack)
        phases.add(out.phase)
        if out.action:
            actions.append(out.action); pending = (now+.2, out.action)
        if out.done or out.failed:
            return task, out, actual, actions, phases, now
        previous = tuple(actual)
        gain = 1-math.exp(-.05/tracking_tau)
        actual = [actual[i]+gain*(out.target[i]-actual[i]) for i in range(2)]
        velocity = tuple((actual[i]-previous[i])/.05 for i in range(2))+(0.,)
    pytest.fail('Rescue task did not terminate within 50 seconds')


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('camera_delay,tracking_tau',[(.15,.2),(.3,.4)])
@pytest.mark.parametrize('person_fraction',[None,.15,.5,.9])
@pytest.mark.parametrize('rotated',[False,True])
def test_delayed_rescue_rejoins_anchor_before_success_or_normal_search_failure(direction,camera_delay,tracking_tau,person_fraction,rotated):
    task,out,actual,actions,phases,elapsed = run_rescue(direction,person_fraction,camera_delay,tracking_tau,rotated)
    assert math.dist(actual,task.anchor[:2]) <= task.arrival_xy_m, (out,actual,elapsed)
    if person_fraction is None:
        assert out.failed == 'correct_target_not_found' and actions == []
    else:
        assert out.done and out.failed is None, (out,task.diagnostic(elapsed),elapsed)
        assert actions == ['open','close']
    assert elapsed < 45.


@pytest.mark.parametrize('direction',[-1,1])
@pytest.mark.parametrize('rotated',[False,True])
def test_delayed_uncalibrated_rescue_measures_then_drops_and_returns(direction,rotated):
    task,out,actual,actions,phases,elapsed = run_rescue(direction,.6,.2,.3,rotated,measured=False)
    assert out.done and out.failed is None, (out,task.diagnostic(elapsed),elapsed)
    assert {'PROBE_X','PROBE_Y'} <= phases
    assert actions == ['open','close']
    assert math.dist(actual,task.anchor[:2]) <= task.arrival_xy_m


def edge_probe():
    task = VisualTask(2,(2.,2.7,1.,0.),[(1.2,2.7,1.,0.)],0.)
    task.phase = 'PROBE_X'
    task.sample_origin = task.anchor
    task.samples = [((2.,2.7),(600.,240.))]
    task.desired = task.commanded = (2.08,2.7,1.,0.)
    task.arrived_at = .2
    return task


@pytest.mark.parametrize('fault',['disabled','stale','cached','moving','unfinished'])
def test_probe_recovery_requires_live_absence_and_completed_settled_motion(fault):
    task = edge_probe()
    snapshot = dict(source='camera',valid=True,frame_fresh=True,enabled=True,
                    received_at=1.,age_s=0.,sequence=10,image_size=[640,480],detections=[])
    speed = .0
    if fault == 'disabled':snapshot['enabled'] = False
    elif fault == 'stale':snapshot['age_s'] = 1.
    elif fault == 'cached':task.last_camera_sequence = 10
    elif fault == 'moving':speed = .11
    else:task.commanded = task.anchor
    out = task.step((2.08,2.7,1.,0.,0.),(speed,0.,0.),1.,snapshot)
    assert out.phase == 'PROBE_X' and not task.reversed_probe_axes
    assert out.action is None


def test_probe_recovery_attempt_is_bounded_and_does_not_reset_task_deadline():
    task = edge_probe()
    def empty(now):
        return dict(source='camera',valid=True,frame_fresh=True,enabled=True,
                    received_at=now,age_s=0.,sequence=now,image_size=[640,480],detections=[])
    out = task.step((2.08,2.7,1.,0.,0.),(0.,0.,0.),1.,empty(1.))
    assert out.phase == 'PROBE_RECOVER' and task.started == 0.
    assert task.reversed_probe_axes == {'PROBE_X'}
    # A second loss on the reversed probe must retain the original failure.
    task.phase = 'PROBE_X';task.changed = 1.;task.arrived_at = 1.2
    task.desired = task.commanded = (1.92,2.7,1.,0.)
    out = task.step((1.92,2.7,1.,0.,0.),(0.,0.,0.),7.1,empty(7.1))
    assert task.phase != 'PROBE_RECOVER'
    out = task.step((1.92,2.7,1.,0.,0.),(0.,0.,0.),7.2,empty(7.2))
    assert out.failed == 'visual_calibration_target_lost' and out.action is None


def test_recovery_keeps_the_original_45_second_task_limit():
    task = edge_probe()
    task.phase = 'PROBE_RECOVER'
    task.desired = task.anchor
    out = task.step((*task.anchor,0.),(0.,0.,0.),45.1,None)
    assert out.failed == 'visual_task_timeout' and out.action is None
