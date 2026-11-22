"""Calibration and alignment regressions for the immediate-release policy."""


from simple_flight_mission.visual_task import VisualTask
from test.test_competition import frame


def alignment_task():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0.,
                      jacobian=((400., 0.), (0., 400.)))
    task.phase = "ALIGN"
    task.desired = task.commanded = (.054, 0., 1., 0.)
    return task


def test_tracking_residual_does_not_block_visual_correction():
    task = alignment_task()
    for i in range(1, 9):
        now = i * .1
        task.step((0., 0., 1., 0., 0.), (0., 0., 0.), now,
                  frame(now, box=(360, 220, 400, 260)))
    assert task.desired[0] < 0., "5.4cm tracking residual must not freeze ALIGN"


def test_motion_period_image_cannot_drive_next_correction():
    task = alignment_task()
    original = task.desired
    for i in range(1, 8):
        now = i * .1
        old = dict(frame(now, box=(360, 220, 400, 260)), age_s=now - .1)
        task.step((0., 0., 1., 0., 0.), (0., 0., 0.), now, old)
    assert task.desired == original
    for now in (.8, 1., 1.2):
        task.step((0., 0., 1., 0., 0.), (0., 0., 0.), now,
                  frame(now, box=(360, 220, 400, 260)))
    assert task.desired[0] < 0.


def test_probe_rejects_insufficient_actual_displacement():
    task = alignment_task()
    task.phase = "PROBE_X"
    task.sample_origin = (0., 0., 1., 0.)
    task.samples = [((0., 0.), (420., 240.))]
    for i in range(1, 10):
        now = i * .1
        task.step((0., 0., 1., 0., 0.), (0., 0., 0.), now, frame(now, box=(400, 220, 440, 260)))
    assert len(task.samples) == 1 and task.phase == "PROBE_X"


def test_probe_accepts_measured_baseline_with_four_centimeter_tracking_residual():
    task = alignment_task()
    task.phase = "PROBE_X"
    task.sample_origin = (0., 0., 1., 0.)
    task.samples = [((0., 0.), (360., 240.))]
    task.desired = task.commanded = (.08, 0., 1., 0.)
    for i in range(1, 11):
        now = i * .1
        task.step((.045, .03, 1., 0., 0.), (.02, 0., 0.), now,
                  frame(now, box=(358, 229, 398, 269)))
        if task.phase == "PROBE_Y":
            break
    assert task.phase == "PROBE_Y"
    assert len(task.samples) == 2


def test_both_off_nominal_probes_recover_the_actual_image_matrix():
    task = alignment_task()
    task.phase = "PROBE_X"
    task.sample_origin = (0., 0., 1., 0.)
    task.samples = [((0., 0.), (420., 240.))]
    task.desired = task.commanded = (.08, 0., 1., 0.)
    for i in range(1, 100):
        now = i * .05
        x, y = (.045, .03) if task.phase == "PROBE_X" else (-.02, .06)
        u, v = 420 + 400 * x, 240 + 300 * y
        task.step((x, y, 1., 0., 0.), (.02, 0., 0.), now,
                  frame(now, box=(u-20, v-20, u+20, v+20)))
        if task.phase == "CALIB_RETURN":
            break
    assert task.phase == "CALIB_RETURN" and task.failure is None
    assert abs(task.jacobian[0][0] - 400) < 1e-6
    assert abs(task.jacobian[1][1] - 300) < 1e-6
    assert abs(task.jacobian[0][1]) < 1e-6
    assert abs(task.jacobian[1][0]) < 1e-6


def test_weak_or_parallel_actual_calibration_baselines_are_rejected():
    import pytest
    from simple_flight_mission.visual_task import estimate_jacobian
    for x, y in (((.02, 0.), (0., .08)), ((.08, 0.), (.08, .005))):
        samples = [((0., 0.), (320., 240.))]
        for px, py in (x, y):
            samples.append(((px, py), (320+400*px, 240+300*py)))
        with pytest.raises(ValueError):
            estimate_jacobian(samples)


def test_calibration_pairs_the_image_with_its_capture_time_pose():
    task = alignment_task()
    task.pose_history.extend(((1., (0., 0.)), (1.1, (.01, .02)), (1.2, (.02, .04))))
    captured = task._captured_pose(1.15)
    assert abs(captured[0] - .015) < 1e-9
    assert abs(captured[1] - .03) < 1e-9
    assert task._captured_pose(.9) is None


def test_visual_failure_prints_the_actual_return_reason_once():
    from types import SimpleNamespace
    from simple_flight_mission.competition_node import CompetitionNode
    warnings = []
    node = CompetitionNode.__new__(CompetitionNode)
    node.options = SimpleNamespace(mode="execute")
    node.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=warnings.append))
    node._event = lambda *args: None
    node.task_report_key = node.task_report_at = node.visual_error_reported = None
    node.last_status = dict(phase="RETURN", task_phase="FAILED", task_door=1,
                            visual_error="visual_calibration_not_settled")
    node._report_task(1.)
    node._report_task(2.)
    assert warnings == ["取消投放并返航：标定时飞机未稳定"]


def test_alignment_uses_capture_pose_to_avoid_a_delayed_image_goal_bias():
    task = alignment_task()
    for i in range(1, 18):
        now = i * .1
        captured = max(.1, now - .2)
        # Keep the target outside the 50px release radius to test pose pairing.
        # Camera Jacobian is +400px/m; fixed target is world x=-18cm.
        u = 320 + 400 * (.02 * captured + .18)
        snapshot = dict(frame(now, box=(u-20, 220, u+20, 260)), age_s=now-captured)
        task.step((.02*now, 0., 1., 0., 0.), (.02, 0., 0.), now, snapshot)
    assert abs(task.observation["target_world_xy"][0] + .18) < .001


def test_return_uses_configured_route_arrival_not_probe_precision():
    task = alignment_task()
    task.phase = "RETURN_ANCHOR"
    task.desired = task.anchor
    for i in range(1, 10):
        now = i * .1
        out = task.step((.054, 0., 1., 0., 0.), (0., 0., 0.), now, frame(now))
    assert out.done


def test_search_can_sample_actual_pose_with_route_level_tracking_residual():
    task = alignment_task()
    task.phase = "SEARCH"
    for i in range(1, 9):
        now = i * .1
        task.step((0., 0., 1., 0., 0.), (0., 0., 0.), now, frame(now, box=(360, 220, 400, 260)))
    assert task.phase in ("ALIGN",)


def test_subphase_changes_are_logged_and_periodic_records_are_bounded():
    from types import SimpleNamespace
    from simple_flight_mission.competition_node import CompetitionNode
    events, messages = [], []
    node = CompetitionNode.__new__(CompetitionNode)
    node.options = SimpleNamespace(mode="execute")
    node.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=messages.append))
    node._event = lambda kind, status: events.append((kind, dict(status)))
    node.task_report_key = node.task_report_at = None
    node.last_status = dict(phase="POINT_HOLD", task_phase="ALIGN", task_door=1,
                            task_diagnostics=alignment_task().diagnostic(0.))
    node._report_task(0.)
    node._report_task(.1)
    node.last_status["task_phase"] = "OPEN_WAIT"
    node._report_task(.2)
    node._report_task(.3)
    node._report_task(1.3)
    assert len(events) == len(messages) == 3
    assert [item[1]["task_phase"] for item in events] == ["ALIGN", "OPEN_WAIT", "OPEN_WAIT"]
    node.options.mode = "check"
    node._report_task(3.)
    assert len(events) == 3
