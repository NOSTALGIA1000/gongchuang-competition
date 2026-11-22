from simple_flight_mission.waypoint_node import build_parser, format_bench_status


def test_bench_mode_is_explicit_and_never_execute():
    args = build_parser().parse_args(["--bench"])
    assert args.bench is True
    assert args.execute is False


def test_bench_status_includes_height_and_speed():
    text = format_bench_status("POSCTL", False, (1.0, 2.0, 0.84), 0.12, 0.08)
    assert "高度=0.84 m" in text
    assert "水平速度=0.12 m/s" in text
    assert "位置年龄=0.08 s" in text
