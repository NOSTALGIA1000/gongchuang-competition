"""User policy: fresh correct target centered -> open now, with no MARK dwell."""
import pytest

from simple_flight_mission.visual_task import VisualTask
from test.test_competition import frame


@pytest.mark.parametrize("door,kind", ((1, 2), (2, 0)))
def test_centered_target_opens_on_the_same_step_without_calibration(door, kind):
    task = VisualTask(door, (0., 0., 1., 0.), [], 0.)
    out = task.step((0., 0., 1., 0., 0.), (.08, 0., 0.), .1, frame(.1, kind=kind))
    assert out.action == "open" and out.phase == "OPEN_WAIT"
    assert task.jacobian is None and not task.samples


def test_align_does_not_wait_for_speed_settle_filter_or_command_ramp_when_centered():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0., jacobian=((400., 0.), (0., 192.)))
    task.phase = "ALIGN"
    task.desired = (.08, 0., 1., 0.)
    out = task.step((0., 0., 1., 0., 0.), (.087, 0., 0.), .1,
                    frame(.1, box=(307, 224, 347, 264)))  # raw error=sqrt(7²+4²)
    assert out.action == "open" and out.phase == "OPEN_WAIT"
    assert out.target == task.anchor


@pytest.mark.parametrize("changes", (dict(age_s=1.), dict(valid=False), dict(source="image_replay")))
def test_invalid_or_stale_centered_frame_cannot_open(changes):
    task = VisualTask(1, (0., 0., 1., 0.), [], 0.)
    image = dict(frame(.1), **changes)
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1, image)
    assert out.action is None and not task.open_sent


@pytest.mark.parametrize("door,wrong_kind", ((1, 0), (2, 2)))
def test_wrong_class_cannot_trigger_either_door(door, wrong_kind):
    task = VisualTask(door, (0., 0., 1., 0.), [], 0.)
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1, frame(.1, kind=wrong_kind))
    assert out.action is None


def test_cached_sequence_cannot_trigger_open_but_the_next_fresh_centered_frame_can():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0.)
    task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1,
              frame(.1, box=(360, 220, 400, 260), sequence=1))
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .2, frame(.2, sequence=1))
    assert out.action is None
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .3, frame(.3, sequence=2))
    assert out.action == "open"


def test_off_center_never_opens_and_first_centered_frame_opens_immediately():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0.)
    for i in range(1, 5):
        now = i * .1
        out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), now,
                        frame(now, box=(360, 220, 400, 260)))
        assert out.action is None
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .5, frame(.5))
    assert out.action == "open"


def test_task_bound_still_blocks_open():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0., jacobian=((400., 0.), (0., 400.)))
    task.phase = "ALIGN"
    out = task.step((.36, 0., 1., 0., 0.), (0., 0., 0.), .1, frame(.1))
    assert out.action is None and out.failed == "visual_alignment_bound"


def test_drop_dwell_begins_after_open_ack_then_closes_once():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0.)
    assert task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1, frame(.1)).action == "open"
    assert task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .2, frame(.2), ("open", True)).phase == "DROP_DWELL"
    assert task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 1.19, frame(1.19)).action is None
    assert task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 1.21, frame(1.21)).action == "close"
    assert task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 1.3, frame(1.3), ("close", True)).phase == "RETURN_ANCHOR"
    assert task.step((0., 0., 1., 0., 0.), (0., 0., 0.), 1.4, frame(1.4)).done


def test_visual_correction_is_damped_and_capped_at_three_centimeters():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0., jacobian=((400., 0.), (0., 400.)))
    task.phase = "ALIGN"
    for i in range(1, 9):
        now = i * .1
        task.step((0., 0., 1., 0., 0.), (0., 0., 0.), now,
                  frame(now, box=(380, 220, 420, 260)))
        if task.desired[0] < 0:
            break
    assert -.030001 <= task.desired[0] < 0.


def test_status_and_terminal_explicitly_describe_immediate_release():
    from simple_flight_mission.competition_node import task_progress_text
    task = VisualTask(1, (0., 0., 1., 0.), [], 0.)
    out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), .1, frame(.1))
    diagnostic = task.diagnostic(.1)
    text = task_progress_text(dict(task_phase=out.phase, task_door=1, task_diagnostics=diagnostic))
    assert diagnostic["release_policy"] == "center_then_open"
    assert "计时" not in text and "要求=" not in text
