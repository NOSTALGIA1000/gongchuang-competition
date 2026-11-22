import pytest

from simple_flight_mission.mission_config import MissionConfig, load_config


def test_incremental_actions_expand_from_origin_and_append_return():
    config = MissionConfig.from_mapping({
        "route": [
            {"forward_m": 1.0, "left_m": 0.0},
            {"forward_m": 0.0, "left_m": 1.0},
            {"forward_m": 1.0, "left_m": 0.0},
        ],
        "limits": {"max_speed_mps": 0.3, "arrival_radius_m": 0.2,
                    "arrival_speed_mps": 0.1, "hold_s": 2.0,
                    "segment_timeout_s": 30.0},
    })

    expected = [(1.0, 0.0), (1.0, 1.0), (2.0, 1.0), (0.0, 0.0)]
    assert len(config.relative_points()) == len(expected)
    for actual, point in zip(config.relative_points(), expected):
        assert actual == pytest.approx(point)


def test_config_rejects_wrong_route_length_and_nonzero_vertical_action():
    with pytest.raises(ValueError, match="exactly 3"):
        MissionConfig.from_mapping({"route": [{"forward_m": 1.0, "left_m": 0.0}]})

    with pytest.raises(ValueError, match="z"):
        MissionConfig.from_mapping({
            "route": [
                {"forward_m": 1.0, "left_m": 0.0, "up_m": 0.1},
                {"forward_m": 0.0, "left_m": 1.0},
                {"forward_m": 1.0, "left_m": 0.0},
            ]
        })


def test_load_config_reads_yaml(tmp_path):
    path = tmp_path / "mission.yaml"
    path.write_text(
        "route:\n"
        "  - {forward_m: 1.0, left_m: 0.0}\n"
        "  - {forward_m: 0.0, left_m: 1.0}\n"
        "  - {forward_m: 1.0, left_m: 0.0}\n",
        encoding="utf-8",
    )
    assert load_config(path).relative_points()[-1] == pytest.approx((0.0, 0.0))


def test_config_defaults_to_swa_ch5_takeover_thresholds():
    config = MissionConfig.from_mapping({
        "route": [
            {"forward_m": 1.0, "left_m": 0.0},
            {"forward_m": 0.0, "left_m": 1.0},
            {"forward_m": 1.0, "left_m": 0.0},
        ]
    })
    assert config.rc_takeover_channel == 5
    assert config.rc_takeover_low_pwm == 1200
    assert config.rc_takeover_high_pwm == 1800
    assert config.throttle_drop_pwm == 150
