"""Relaxed release and recovery from an early, nearly parallel Y sample."""
import pytest

from simple_flight_mission.visual_task import VisualTask
from test.test_competition import frame


@pytest.mark.parametrize('door,kind', [(1, 2), (2, 0)])
@pytest.mark.parametrize('phase', ['SEARCH', 'ALIGN', 'PROBE_Y'])
@pytest.mark.parametrize('error', [49.975619055695546, 50.])
def test_fresh_target_within_50px_opens_without_calibration(door, kind, phase, error):
    task = VisualTask(door, (0., 0., 1., 0.), [], 0.)
    task.phase = phase
    if phase == 'ALIGN':
        task.jacobian = ((400., 0.), (0., 400.))
    if phase == 'PROBE_Y':
        task.sample_origin = task.anchor
        task.samples = [((0., 0.), (420., 240.)), ((.05, 0.), (440., 240.))]
    previous_samples = list(task.samples)
    previous_matrix = task.jacobian
    image = frame(.1, kind=kind, box=(300+error, 220, 340+error, 260))
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1, image)
    assert out.action == 'open' and out.phase == 'OPEN_WAIT'
    assert task.jacobian == previous_matrix and task.samples == previous_samples
    assert task.diagnostic(.1)['release_threshold_px'] == 50


@pytest.mark.parametrize('door,kind', [(1, 2), (2, 0)])
def test_beyond_50px_still_aligns_instead_of_dropping_or_aborting(door, kind):
    task = VisualTask(door, (0., 0., 1., 0.), [], 0., jacobian=((400., 0.), (0., 400.)))
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1,
                    frame(.1, kind=kind, box=(350.01, 220, 390.01, 260)))
    assert out.action is None and out.failed is None and not task.open_sent


@pytest.mark.parametrize('door,kind', [(1, 2), (2, 0)])
def test_50px_policy_still_rejects_stale_frames(door, kind):
    task = VisualTask(door, (0., 0., 1., 0.), [], 0.)
    image = dict(frame(.1, kind=kind, box=(349, 220, 389, 260)), age_s=1.)
    assert task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1, image).action is None


def y_probe():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0.)
    task.phase = 'PROBE_Y'
    task.sample_origin = task.anchor
    task.samples = [((0., 0.), (420., 240.)), ((.05, 0.), (440., 240.))]
    task.desired = task.commanded = (0., .08, 1., 0.)
    return task


def probe_tick(task, now, xy):
    u, v = 420+400*xy[0], 240+400*xy[1]
    return task.step((*xy, 1., 0., 0.), (0., 0., 0.), now,
                     frame(now, box=(u-20, v-20, u+20, v+20)))


def test_nearly_parallel_y_sample_waits_for_independent_motion_and_recovers():
    task = y_probe()
    probe_tick(task, .1, (.075, .02))
    rejected = probe_tick(task, .201, (.075, .02))
    assert rejected.failed is None and rejected.phase == 'PROBE_Y'
    assert task.failure is None and task.jacobian is None and len(task.samples) == 2
    assert task.desired == (0., .08, 1., 0.)
    assert task.diagnostic(.201)['last_calibration_error']
    recovered = probe_tick(task, .35, (0., .08))
    assert recovered.phase == 'CALIB_RETURN' and recovered.failed is None
    assert len(task.samples) == 3
    assert task.jacobian[0] == pytest.approx((400., 0.))
    assert task.jacobian[1] == pytest.approx((0., 400.))


def test_permanently_invalid_y_samples_do_not_extend_original_6s_window():
    task = y_probe()
    for now in (.1, .201, 1., 3., 5.9, 6.01, 6.06):
        out = probe_tick(task, now, (.075, .02))
    assert out.failed == 'visual_calibration_invalid' and out.action is None
    assert task.jacobian is None and len(task.samples) == 2
