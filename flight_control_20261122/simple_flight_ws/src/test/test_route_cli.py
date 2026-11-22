"""Configuration selection and the MAVROS boundary, without a ROS dependency."""
import json
from pathlib import Path
import subprocess
import sys
from types import ModuleType

import pytest


def test_raw_position_and_velocity_masks_do_not_combine_conflicting_vertical_targets():
    from simple_flight_mission.route_node import raw_target_fields
    from simple_flight_mission.route_controller import RouteOutput
    out = RouteOutput("TRANSIT", (2., -3., 5.2, .7))
    fields = raw_target_fields(out)
    assert fields == dict(coordinate_frame=1, type_mask=2552,
                          position=(2., -3., 5.2), velocity=(0., 0., 0.), yaw=.7)
    out = RouteOutput("DESCEND", (2., -3., 5.2, .7), velocity_z=-.15)
    fields = raw_target_fields(out)
    assert fields["type_mask"] == 2524  # Ignore position Z; use finite ENU VZ.
    assert fields["velocity"] == (0., 0., -.15)
    assert fields["position"][:2] == (2., -3.)


def test_describe_custom_yaml_expands_points_without_importing_ros(tmp_path):
    config = tmp_path / "custom.yaml"
    config.write_text("points:\n  - {forward_m: 2, hold_s: 4}\n  - {forward_m: 3, left_m: 1, hold_s: 2}\n", encoding="utf-8")
    proc = subprocess.run([sys.executable, "-m", "simple_flight_mission.route_node",
                           "--describe", "--config", str(config)],
                          capture_output=True, text=True, encoding="utf-8")
    assert proc.returncode == 0, proc.stderr
    result = json.loads(proc.stdout)
    assert result["mode"] == "check"
    assert result["config"]["points"][0]["forward_m"] == 2
    assert len(result["config"]["points"]) == 2
    assert result["coordinate_rule"] == "absolute_offsets_from_takeoff_in_initial_heading"


def test_default_yaml_matches_approved_route_and_can_be_loaded_from_source():
    from simple_flight_mission.route_config import load_route_config
    config = load_route_config(Path(__file__).resolve().parents[1]/"config/route.yaml")
    assert len(config.points) == 1
    assert config.points[0].forward_m == 1 and config.points[0].left_m == 0 and config.points[0].hold_s == 3
    assert (config.height_m, config.origin_hold_s, config.return_hold_s) == (1., 3., 2.)


def test_route_cli_defaults_to_check_and_conflicting_modes_are_rejected():
    from simple_flight_mission.route_node import parse_options
    assert parse_options([]).mode == "check"
    assert parse_options(["--execute", "--route", "1"]).route == 1
    with pytest.raises(SystemExit):
        parse_options(["--route", "0"])
    with pytest.raises(SystemExit):
        parse_options(["--execute", "--mode-test"])
    with pytest.raises(SystemExit):
        parse_options(["--execute", "--duration", "1"])
    with pytest.raises(SystemExit):
        parse_options(["--route", "1", "--config", "custom.yaml"])


def test_describe_route_one_uses_measured_path_without_ros():
    proc = subprocess.run([sys.executable, "-m", "simple_flight_mission.route_node",
                           "--describe", "--route", "1"],
                          capture_output=True, text=True, encoding="utf-8")
    assert proc.returncode == 0, proc.stderr
    config = json.loads(proc.stdout)["config"]
    assert [(p["forward_m"], p["left_m"]) for p in config["points"]] == [
        (2.2, 0.), (2.2, 1.4), (0., 1.4), (2.2, 1.4), (2.2, 2.7)]
    assert config["return_indices"] == [3, 1, 0]
    assert config["mission_timeout_s"] == 300.


def test_describe_route_two_uses_measured_path_without_ros():
    proc = subprocess.run([sys.executable, "-m", "simple_flight_mission.route_node",
                           "--describe", "--route", "2"],
                          capture_output=True, text=True, encoding="utf-8")
    assert proc.returncode == 0, proc.stderr
    config = json.loads(proc.stdout)["config"]
    assert [(p["forward_m"], p["left_m"]) for p in config["points"]] == [
        (2.2, 0.), (2.2, 1.4), (0., 1.4), (1.2, 1.4), (1.2, 2.7)]
    assert [p["hold_s"] for p in config["points"]] == [0.1, 0.1, 3.0, 0.1, 3.0]
    expected_return = [3, 1, 0]
    assert config["return_indices"] == expected_return
    assert config["mission_timeout_s"] == 300.


def test_describe_route_three_uses_measured_path_without_ros():
    proc = subprocess.run([sys.executable, "-m", "simple_flight_mission.route_node",
                           "--describe", "--route", "3"],
                          capture_output=True, text=True, encoding="utf-8")
    assert proc.returncode == 0, proc.stderr
    config = json.loads(proc.stdout)["config"]
    assert [(p["forward_m"], p["left_m"]) for p in config["points"]] == [
        (1.7, 0.), (1.7, 1.4), (0., 1.4), (2.2, 1.4), (2.2, 2.7)]
    assert [p["hold_s"] for p in config["points"]] == [0.1, 0.1, 3.0, 0.1, 3.0]
    expected_return = [3, 1, 0]
    assert config["return_indices"] == expected_return
    assert config["mission_timeout_s"] == 300.


def test_describe_route_four_combines_path_three_entry_and_path_two_exit():
    proc = subprocess.run([sys.executable, "-m", "simple_flight_mission.route_node",
                           "--describe", "--route", "4"],
                          capture_output=True, text=True, encoding="utf-8")
    assert proc.returncode == 0, proc.stderr
    result = json.loads(proc.stdout)
    assert result["mode"] == "check"
    config = result["config"]
    assert [(p["forward_m"], p["left_m"]) for p in config["points"]] == [
        (1.7, 0.), (1.7, 1.4), (0., 1.4), (1.2, 1.4), (1.2, 2.7)]
    expected_return = [3, 1, 0]
    assert config["return_indices"] == expected_return


def _mock_ament(monkeypatch, share=None, missing=False):
    packages = ModuleType("ament_index_python.packages")
    class PackageNotFoundError(Exception):
        pass
    def get_package_share_directory(name):
        assert name == "simple_flight_mission"
        if missing:
            raise PackageNotFoundError(name)
        return str(share)
    packages.PackageNotFoundError = PackageNotFoundError
    packages.get_package_share_directory = get_package_share_directory
    monkeypatch.setitem(sys.modules, "ament_index_python", ModuleType("ament_index_python"))
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", packages)


def test_route_selection_prefers_installed_ament_share(monkeypatch, tmp_path):
    from simple_flight_mission.route_node import route_config_path
    config = tmp_path / "config" / "route_1.yaml"
    config.parent.mkdir()
    config.write_text("points: [{forward_m: 8}]\n", encoding="utf-8")
    _mock_ament(monkeypatch, tmp_path)
    assert route_config_path(1) == config


def test_route_selection_falls_back_to_source_if_package_is_not_indexed(monkeypatch):
    from simple_flight_mission import route_node
    from simple_flight_mission.route_node import route_config_path
    monkeypatch.setattr(route_node,"__file__",str(Path(__file__).resolve().parents[1]/"simple_flight_mission/route_node.py"))
    _mock_ament(monkeypatch, missing=True)
    assert route_config_path(1) == Path(__file__).resolve().parents[1]/"config/route_1.yaml"


def test_route_selection_falls_back_to_source_without_ament(monkeypatch):
    from simple_flight_mission import route_node
    from simple_flight_mission.route_node import route_config_path
    monkeypatch.setattr(route_node,"__file__",str(Path(__file__).resolve().parents[1]/"simple_flight_mission/route_node.py"))
    monkeypatch.setitem(sys.modules, "ament_index_python", None)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", None)
    assert route_config_path(1) == Path(__file__).resolve().parents[1]/"config/route_1.yaml"


def test_selected_route_config_respects_route_option(monkeypatch):
    from simple_flight_mission.route_node import parse_options, selected_route_config
    _mock_ament(monkeypatch, Path(__file__).resolve().parents[1])
    config = selected_route_config(parse_options(["--route", "1"]))
    assert len(config.points) == 5
    assert config.points[-1].forward_m == 2.2
    assert config.points[-1].left_m == 2.7


def test_missing_installed_route_does_not_silently_select_source(monkeypatch, tmp_path):
    from simple_flight_mission.route_node import route_config_path
    _mock_ament(monkeypatch, tmp_path)
    with pytest.raises(ValueError, match="尚未配置"):
        route_config_path(1)


def test_outbound_hold_does_not_start_while_actual_aircraft_is_still_moving():
    from test.test_route_flight import route_rig, until_phase
    from simple_flight_mission.hover_controller import Observation
    rig = route_rig()
    rig.active()
    until_phase(rig, "POINT_HOLD")
    for _ in range(80):
        out = rig.tick(velocity=Observation((.2, 0., 0.), rig.t+.05, 0.))
        assert out.phase == "POINT_HOLD"
    for _ in range(60):
        assert rig.tick().phase == "POINT_HOLD"


def test_nonfinite_or_unbounded_route_is_rejected_during_config_validation():
    from simple_flight_mission.route_config import RouteConfig, RoutePoint
    with pytest.raises(ValueError):
        RouteConfig(points=(RoutePoint(1e308, 1e308),))


def test_only_the_typed_mavros_receiver_counts_as_target_transport():
    from types import SimpleNamespace
    from simple_flight_mission.route_node import raw_receiver_ready
    def endpoint(name, ns="/mavros", kind="mavros_msgs/msg/PositionTarget"):
        return SimpleNamespace(node_name=name, node_namespace=ns, topic_type=kind)
    assert not raw_receiver_ready([])
    assert not raw_receiver_ready([endpoint("observer", "/")])
    assert not raw_receiver_ready([endpoint("setpoint_raw", kind="geometry_msgs/msg/PoseStamped")])
    assert raw_receiver_ready([endpoint("observer", "/"), endpoint("setpoint_raw")])
    assert not raw_receiver_ready([endpoint("setpoint_raw"), endpoint("setpoint_raw")])


def test_route_adapter_requests_gnss_fusion_parameter_from_fcu():
    from simple_flight_mission.route_node import ROUTE_PARAMETER_NAMES
    assert "EKF2_GPS_CTRL" in ROUTE_PARAMETER_NAMES
