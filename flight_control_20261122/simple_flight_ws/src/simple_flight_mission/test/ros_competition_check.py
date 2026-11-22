"""Isolated ROS/message rehearsal. Not real flight, PX4 SITL or a physical test."""
import os
if os.environ.get("ROS_DOMAIN_ID") != "77" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("Refusing test: require ROS_DOMAIN_ID=77 and ROS_LOCALHOST_ONLY=1")

import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent / "rosfixtures"))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from mavros_msgs.srv import CommandLong
from test_route_node_ros import RoutePeer
from test_hover_node_ros import pump, until
from simple_flight_mission.competition_node import CompetitionNode
from simple_flight_mission.route_config import RouteConfig, RoutePoint
from simple_flight_payload.door import build_command, default_config


class Peer(RoutePeer):
    def __init__(self):
        self.payload_commands = []
        super().__init__()
        self.payload_service = self.create_service(CommandLong, "/mavros/cmd/command", self.payload)

    def arm(self, request, response):
        # Competition runs the normal landed-disarm path; keep it entirely virtual.
        self.arm_calls.append(request.value)
        self.armed = request.value
        response.success, response.result = True, 0
        return response

    def parameters(self, request, response):
        response = super().parameters(request, response)
        values = dict(PWM_AUX_FUNC7=301, PWM_AUX_FUNC8=302, PWM_AUX_MIN7=1000,
                      PWM_AUX_MAX7=2000, PWM_AUX_MIN8=1000, PWM_AUX_MAX8=2000, PWM_AUX_REV=0)
        for name, value in zip(request.names, response.values):
            if name in values:
                value.type, value.integer_value = 2, values[name]
        return response

    def payload(self, request, response):
        assert request.command == 187 and request.param7 == 0
        params = [getattr(request, "param" + str(i)) for i in range(1, 7)]
        changed = [i for i, value in enumerate(params) if not math.isnan(value)]
        assert len(changed) == 1 and changed[0] in (0, 1)
        self.payload_commands.append((changed[0] + 1, params[changed[0]]))
        response.success, response.result = True, 0
        return response


def run_case(mode):
    peer = Peer()
    runner = None
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            now = time.monotonic()
            task = runner.controller.task if runner else None
            kind = 0 if task and task.door == 2 else 2
            u, v = 320., 240.
            if runner and runner.controller.world_route:
                goal = runner.controller.world_route[4 if kind == 0 else 2] if len(runner.controller.world_route) > 2 else runner.controller.world_route[1 if kind == 0 else 0]
                dx, dy = goal[0] - peer.xyz[0], goal[1] - peer.xyz[1]
                u, v = 320 - 100 * dx + 400 * dy, 240 + 300 * dx + 50 * dy
            data = dict(source="camera", valid=True, frame_fresh=True, enabled=True,
                        age_s=0., stream_id="isolated-virtual-camera", sequence=int(now * 5),
                        image_size=[640, 480], detections=[dict(class_id=kind,
                        name="Person" if kind == 0 else "Hazardous", confidence=.9,
                        box=[u - 20, v - 20, u + 20, v + 20])])
            body = json.dumps(data).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    directory = Path(tempfile.mkdtemp(prefix="competition_ros_"))
    route = RouteConfig(height_m=.3, origin_hold_s=.2, return_hold_s=.2,
                        arrival_stable_s=.2, horizontal_mps=.3,
                        points=(RoutePoint(.3, 0., .2, "task_zone_1"),
                                RoutePoint(.3, .3, .2, "task_zone_2")), return_indices=(0,),
                        search_point_name="5.1", search_xy=(.6, 0.))
    options = SimpleNamespace(mode=mode, report_dir=directory, duration=3. if mode == "check" else None,
                              start_trigger="notebook", vision_url=f"http://127.0.0.1:{server.server_port}/api/status",
                              status_file=directory / "status.json")
    doors = json.loads(default_config().read_text())
    executor = SingleThreadedExecutor()
    try:
        runner = CompetitionNode(options, route, doors)
        for node in (peer, peer.transport_node, runner.node):
            executor.add_node(node)
        if mode == "check":
            pump(executor, 4.)
            assert not peer.raw_targets and not peer.arm_calls and not peer.mode_calls and not peer.payload_commands
            assert runner.target_pub is runner.arm_client is runner.mode_client is runner.payload_client is None
        else:
            until(executor, lambda: runner.finished, 70., "complete competition: " + str(runner.last_status))
            assert runner.controller.mission_completed, runner.last_status
            assert len(runner.controller.completed_tasks) == 2, runner.last_status
            assert peer.arm_calls == [True, False]
            assert not peer.armed and peer.mode == "POSCTL"
            expected = [(door, build_command(doors, door, action).params[door - 1])
                        for door, action in ((1, "close"), (2, "close"), (1, "open"),
                                             (1, "close"), (2, "open"), (2, "close"))]
            assert [door for door, _ in peer.payload_commands] == [door for door, _ in expected]
            assert all(math.isclose(actual, value, abs_tol=1e-6)
                       for (_, actual), (_, value) in zip(peer.payload_commands, expected))
        result = dict(case=mode, passed=True, isolated=True, real_vehicle=False,
                      arm_calls=peer.arm_calls, payload_commands=peer.payload_commands,
                      final_phase=runner.controller.phase, report=str(directory))
        print(json.dumps(result), flush=True)
        return result
    finally:
        server.shutdown()
        server.server_close()
        if runner:
            runner.close()
        peer.transport_node.destroy_node()
        peer.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    rclpy.init(args=[])
    try:
        results = [run_case(mode) for mode in ("check", "execute")]
        print(json.dumps(dict(passed=True, cases=len(results), results=results)), flush=True)
    finally:
        rclpy.shutdown()
