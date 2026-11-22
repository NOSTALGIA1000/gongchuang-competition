"""Focused offline competition checks, no devices or ROS graph."""
import math

from simple_flight_mission.visual_task import estimate_jacobian, select_target, VisualTask


def frame(now, box=(300, 220, 340, 260), kind=2, sequence=None):
    return dict(source="camera", valid=True, frame_fresh=True, enabled=True,
                age_s=0., received_at=now, sequence=now if sequence is None else sequence,
                image_size=[640, 480], detections=[dict(class_id=kind,
                name="Hazardous" if kind == 2 else "Person", confidence=.9, box=list(box))])


def test_image_axis_rotation_is_measured_instead_of_assumed():
    # Image u = -400*y, v = 300*x; camera orientation is not prescribed.
    samples = [((0., 0.), (320., 240.)), ((.08, 0.), (320., 264.)), ((0., .08), (288., 240.))]
    matrix = estimate_jacobian(samples)
    assert abs(matrix[0][0]) < 1e-8 and abs(matrix[0][1] + 400) < 1e-8
    assert abs(matrix[1][0] - 300) < 1e-8 and abs(matrix[1][1]) < 1e-8


def test_replay_stale_wrong_class_or_malformed_box_is_not_a_target():
    assert select_target(frame(1.), 2, 1.) is not None
    for changes in (dict(source="image_replay"), dict(valid=False), dict(age_s=2.),
                    dict(received_at=-10.), dict(detections=[]), dict(image_size=[0, 480])):
        assert select_target(dict(frame(1.), **changes), 2, 1.) is None
    assert select_target(frame(1., kind=0), 2, 1.) is None
    assert select_target(frame(1., box=(0, 0, math.nan, 10)), 2, 1.) is None


def test_a_centered_target_opens_once_closes_and_returns_to_anchor():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0., jacobian=((400., 0.), (0., 400.)))
    actions = []
    ack = None
    out = None
    for i in range(1, 180):
        now = i * .1
        out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), now, frame(now), ack)
        ack = None
        if out.action:
            actions.append(out.action)
            ack = (out.action, True)
        if out.done:
            break
    assert actions == ["open", "close"]
    assert out.done and not out.failed


def test_stale_frozen_frame_cannot_open_without_a_current_target():
    task = VisualTask(1, (0., 0., 1., 0.), [], 0., jacobian=((400., 0.), (0., 400.)))
    actions = []
    for i in range(20, 100):
        now = i * .1
        # Cached original frame becomes stale despite repeated HTTP responses.
        snapshot = dict(frame(.1), age_s=max(0., now - .1), received_at=now)
        out = task.step((0., 0., 1., 0., 0.), (0., 0., 0.), now, snapshot)
        actions.append(out.action)
    assert "open" not in actions


def test_notebook_start_is_explicit_single_use_and_swa_can_still_abort():
    from simple_flight_mission.hover_takeover import HoverTakeover
    gate = HoverTakeover(notebook=True)
    channels = [1500, 1500, 1024, 1500, 2000, 1500, 1024]
    assert gate.observe(channels, 0.).action == "none"
    assert gate.observe(channels, .25, entry_ready=True).action == "none"
    assert gate.observe(channels, .3, entry_ready=True).action == "none"
    gate.request_notebook_start()
    assert gate.observe(channels, .4, entry_ready=True).action == "start"
    gate.confirm_offboard(.45)
    assert gate.observe(channels, .5).action == "none"
    channels[6] = 2000
    assert gate.observe(channels, .6).action == "none"
    channels[6] = 1024
    assert gate.observe(channels, .7).action == "exit"
    assert gate.observe(channels, .8, entry_ready=True).action == "none"


def test_notebook_budget_includes_time_spent_waiting_for_preflight():
    from simple_flight_mission.competition_controller import CompetitionController
    from simple_flight_mission.hover_controller import Vehicle
    from simple_flight_mission.route_config import RouteConfig
    config = {"doors": {str(n): dict(min_us=1000, max_us=2000, open_us=1056, close_us=1500) for n in (1, 2)}}
    controller = CompetitionController("execute", RouteConfig(), config, "notebook")
    controller.step(Vehicle(), 10.)
    controller.step(Vehicle(), 15.)
    assert controller.mission_started_at is None
    assert controller.mission_elapsed(15.) == 5.


def test_complete_competition_has_exactly_two_drop_cycles_and_lands():
    import json
    from pathlib import Path
    from simple_flight_mission.competition_controller import CompetitionController
    from simple_flight_mission.route_config import load_route_config
    from test.test_route_flight import RouteRig, follow

    class CompetitionRig(RouteRig):
        def tick(self, **changes):
            now = self.t + .05
            controller = self.controller
            kind = 0 if controller.task and controller.task.door == 2 else 2
            snapshot = frame(now, kind=kind, sequence=int(now * 5))
            snapshot["stream_id"] = "fixed-camera-process"
            if controller.world_route:
                goal = controller.world_route[4 if kind == 0 else 2]
                dx, dy = goal[0] - self.pose[0], goal[1] - self.pose[1]
                u, v = 320 - 100 * dx + 400 * dy, 240 + 300 * dx + 50 * dy
                snapshot["detections"][0]["box"] = [u - 20, v - 20, u + 20, v + 20]
            controller.vision_data = snapshot
            return super().tick(**changes)

    root = Path(__file__).resolve().parents[1]
    route = load_route_config(root / "config/route_4.yaml")
    doors = json.loads((root.parent / "simple_flight_payload/config/doors.json").read_text())
    rig = CompetitionRig("execute")
    rig.controller = CompetitionController("execute", route, doors, "rc")
    rig.controller.payload_initialized = True
    rig.params.update(EKF2_GPS_CTRL=0, COM_DISARM_LAND=2., LNDMC_Z_VEL_MAX=.25,
                      MPC_LAND_SPEED=.7, MPC_LAND_CRWL=.3, MPC_LAND_ALT3=1., MPC_Z_VEL_MAX_DN=1.5,
                      PWM_AUX_FUNC7=301, PWM_AUX_FUNC8=302, PWM_AUX_MIN7=1000,
                      PWM_AUX_MAX7=2000, PWM_AUX_MIN8=1000, PWM_AUX_MAX8=2000, PWM_AUX_REV=0)
    rig.active()
    actions = []
    for _ in range(6000):
        out = follow(rig)[0]
        action = rig.controller.take_payload_action()
        if action:
            actions.append(action)
            rig.controller.payload_ack = (*action, True)
        if out.phase == "DESCEND" and rig.pose[2] <= 4.20001:
            rig.landed, rig.armed = 1, False
        if out.phase in ("HANDOVER", "EXITED"):
            break
    assert rig.controller.visual_error is None
    assert actions == [(1, "open"), (1, "close"), (2, "open"), (2, "close")]
    assert rig.controller.mission_completed
    assert len(rig.controller.completed_tasks) == 2
