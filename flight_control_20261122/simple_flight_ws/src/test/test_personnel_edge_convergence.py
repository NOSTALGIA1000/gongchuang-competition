"""Route-2 clipped-person / diverging alignment regressions; no device I/O."""
import math

import pytest

from simple_flight_mission.visual_task import VisualTask, select_target
from test.test_competition import frame


@pytest.mark.parametrize('box', [(346, 0, 386, 38.5), (0, 80, 40, 160),
                                 (600, 80, 640, 160), (300, 440, 340, 480)])
def test_clipped_person_cannot_stop_aisle_search_or_supply_a_calibration_sample(box):
    task = VisualTask(2, (2.2, 2.7, 1., 0.), [(1.2, 2.7, 1., 0.)], 0.)
    for tick in range(1, 12):
        now = tick * .1
        out = task.step((2.2, 2.7, 1., 0., 0.), (0., 0., 0.), now,
                        frame(now, kind=0, box=box))
        assert out.action is None and not task.samples
        assert task.observation['camera_frame_valid']
    assert out.phase == 'SCAN' and out.target[0] < 2.2
    assert task.observation['target_rejection'] == 'person_box_at_image_edge'


def test_complete_person_is_selected_over_a_higher_confidence_clipped_person():
    image = frame(1., kind=0, box=(346, 0, 386, 38.5))
    image['detections'].append(dict(class_id=0, name='Person', confidence=.5,
                                    box=[350, 30, 390, 90]))
    assert select_target(image, 0, 1.).center == (370., 60.)


def test_person_fully_entering_frame_stops_scan_and_starts_measured_calibration():
    task = VisualTask(2, (2.2, 2.7, 1., 0.), [(1.2, 2.7, 1., 0.)], 0.)
    for tick in range(1, 10):
        now = tick * .1
        box = (346, 0, 386, 38.5) if tick < 4 else (346, 30, 386, 90)
        out = task.step((2.2, 2.7, 1., 0., 0.), (0., 0., 0.), now,
                        frame(now, kind=0, box=box))
        if out.phase == 'PROBE_X':
            break
    assert out.phase == 'PROBE_X' and len(task.samples) == 1
    assert task.samples[0][1] == (366., 60.) and out.action is None


LOG_MATRIX = ((165.31828332509866, -324.9904574761765),
              (82.8086505349591, -14.50154513505413))


def diverging_replay(task, offset=0.):
    # First complete target has the logged 196px error; subsequent fresh
    # observations grow as measured aircraft displacement exceeds 4cm.
    for tick, (u, v) in enumerate(((366, 47.03125), (380, 42), (430, 35),
                                  (460, 35), (480, 40)), 1):
        now = offset + tick * .2
        pose = (.04 * (tick-1), 0., 1., 0., 0.)
        out = task.step(pose, (0., 0., 0.), now,
                        frame(now, kind=0, box=(u-20, v-20, u+20, v+20)))
    return out, pose, now


def test_sustained_error_growth_stops_bad_correction_and_invalidates_matrix():
    task = VisualTask(2, (0., 0., 1., 0.), [(1.2, 0., 1., 0.)], 0., LOG_MATRIX)
    task.phase = 'ALIGN'
    out, pose, now = diverging_replay(task)
    assert out.phase == 'SEARCH' and out.action is None and out.failed is None
    assert out.target[:2] == pose[:2] and task.desired[:2] == pose[:2]
    assert task.jacobian is None and not task.world_centers and not task.samples
    assert task.wait_reason == 'alignment_diverged_recalibrate'
    assert task.started == 0.
    # Full target can immediately start a fresh calibration, without a new
    # fixed confirmation wait or resetting the overall deadline.
    for tick in range(1, 5):
        t = now + tick*.1
        out = task.step(pose, (0., 0., 0.), t,
                        frame(t, kind=0, box=(430, 30, 470, 70)))
    assert out.phase == 'PROBE_X' and len(task.samples) == 1


@pytest.mark.parametrize('fault', ['no_actual_motion', 'old_capture', 'same_sequence', 'noise'])
def test_divergence_requires_actual_motion_and_several_distinct_new_observations(fault):
    task = VisualTask(2, (0., 0., 1., 0.), [], 0., LOG_MATRIX)
    task.phase = 'ALIGN'
    task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1,
              frame(.1, kind=0, box=(346, 27, 386, 67)))
    for tick in range(1, 7):
        now = .1+tick*.1
        u, v = (370 + tick % 2 * 5, 47) if fault == 'noise' else (480, 40)
        image = frame(now, kind=0, box=(u-20, v-20, u+20, v+20))
        if fault == 'old_capture':
            image['age_s'] = now-.1
        if fault == 'same_sequence':
            image['sequence'] = .1
        pose = (0. if fault == 'no_actual_motion' else tick*.03, 0., 1., 0., 0.)
        task.step(pose, (0., 0., 0.), now, image)
    assert task.phase == 'ALIGN' and task.jacobian == LOG_MATRIX


def test_repeated_divergence_returns_to_original_waypoint5_before_failure():
    task = VisualTask(2, (-.5, 0., 1., 0.), [(1.2, 0., 1., 0.)], 0., LOG_MATRIX)
    task.phase = 'ALIGN'
    diverging_replay(task)
    # Emulate the one permitted fresh calibration also proving unreliable.
    task.jacobian = LOG_MATRIX
    task._phase('ALIGN', 1.1)
    out, pose, now = diverging_replay(task, 1.2)
    assert out.phase == 'RETURN_ANCHOR' and out.failed is None and out.action is None
    assert task.desired == task.anchor and task.started == 0.
    for tick in range(1, 100):
        t = now + tick*.1
        pose = (*out.target, 0.)
        out = task.step(pose, (0., 0., 0.), t, frame(t, kind=0))
        if out.failed:
            break
    assert out.failed == 'visual_alignment_not_converging'
    assert math.dist(pose[:2], task.anchor[:2]) <= task.arrival_xy_m


def test_clipping_during_alignment_stops_old_correction_and_resumes_approved_scan():
    task = VisualTask(2, (2.2, 2.7, 1., 0.), [(1.2, 2.7, 1., 0.)], 0., LOG_MATRIX)
    task.phase = 'ALIGN'
    task.desired = task.commanded = (2.28, 2.78, 1., 0.)
    out = task.step((*task.anchor, 0.), (0., 0., 0.), .1,
                    frame(.1, kind=0, box=(346, 0, 386, 38.5)))
    assert out.phase == 'SCAN' and out.action is None
    assert out.target[1] == 2.7 and out.target[0] < 2.2
    assert not task.world_centers and task.jacobian is None


def test_controller_drops_its_cached_matrix_when_visual_task_invalidates_it():
    from test.test_rescue_aisle_workflow import RescueRig
    from simple_flight_mission.visual_task import TaskOutput
    rig = RescueRig(2)
    rig.reach_rescue()
    task = rig.controller.task
    def invalidated(*args):
        task.jacobian = None
        return TaskOutput(task.anchor, 'SEARCH')
    task.step = invalidated
    rig.advance()
    assert rig.controller.jacobian is None


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('rotated', [False, True])
def test_wrong_matrix_recovers_with_delayed_camera_then_drops_and_rejoins_anchor(direction, rotated):
    from test.test_rescue_delayed_feedback import run_rescue
    wrong = ((0., 400.), (-400., 0.)) if rotated else ((-400., 0.), (0., -400.))
    task, out, actual, actions, phases, elapsed = run_rescue(
        direction, .15, .3, .4, rotated, initial_matrix=wrong)
    assert out.done and actions == ['open', 'close'], (out, task.diagnostic(elapsed))
    assert task.alignment_recoveries == 1
    assert {'PROBE_X', 'PROBE_Y'} <= phases
    assert math.dist(actual, task.anchor[:2]) <= task.arrival_xy_m
    assert elapsed < 45.


@pytest.mark.parametrize('phase', ['PROBE_X', 'PROBE_Y'])
def test_clipped_person_during_probe_cannot_become_a_matrix_sample(phase):
    task = VisualTask(2, (0., 0., 1., 0.), [(1.2, 0., 1., 0.)], 0.)
    task.phase = phase
    task.sample_origin = task.anchor
    task.samples = [((0., 0.), (366., 60.))]
    task.desired = task.commanded = (.08, 0., 1., 0.)
    for now in (.1, .3, .5):
        out = task.step((.08, 0., 1., 0., 0.), (0., 0., 0.), now,
                        frame(now, kind=0, box=(346, 0, 386, 38.5)))
    assert out.phase == phase and len(task.samples) == 1
    assert task.jacobian is None and out.action is None
