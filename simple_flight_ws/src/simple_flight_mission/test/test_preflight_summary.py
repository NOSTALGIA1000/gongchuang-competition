from simple_flight_mission.preflight_summary import format_preflight_summary


def ready(**changes):
    return dict(phase="CHECK", operating_mode="check", reasons=[], actual_mode="POSCTL",
                armed=False, swa_pwm=1024, main_switch_pwm=2003, throttle_pwm=1024, **changes)


def test_success_is_explicit_and_reports_switches_without_startup_noise():
    text = format_preflight_summary(ready())
    assert "预检通过" in text and "POSCTL" in text and "2003" in text
    assert "state_stale" not in text
    assert "未发送" in text


def test_final_blockers_are_readable_and_duplicate_rc_causes_are_grouped():
    status = ready()
    status["reasons"] = ["rc_stale", "rc_not_ready", "rc_invalid", "requires_position_mode"]
    text = format_preflight_summary(status)
    assert "预检未通过" in text and "未处于Position" in text
    assert text.count("遥控输入未就绪") == 1


def test_unknown_real_blocker_is_preserved_instead_of_hidden():
    status = ready()
    status["reasons"] = ["unexpected_real_fault"]
    assert "unexpected_real_fault" in format_preflight_summary(status)
    assert "预检未通过" in format_preflight_summary(status)


def test_absent_or_incomplete_status_never_reports_success():
    for status in (None, {}, dict(phase="CHECK"), dict(phase="TAKEOFF", reasons=[])):
        text = format_preflight_summary(status)
        assert "预检通过" not in text


def test_check_state_changes_do_not_spam_startup_discovery_reasons():
    from types import SimpleNamespace
    from unittest.mock import patch
    from simple_flight_mission.competition_node import CompetitionNode
    from simple_flight_mission.route_node import RouteNode
    runner = object.__new__(CompetitionNode)
    runner.options = SimpleNamespace(mode="check")
    with patch.object(RouteNode, "log_state", side_effect=AssertionError("startup spam")):
        runner.log_state(dict(reasons=["state_stale", "parameters_stale"]))


def test_execute_mode_preserves_live_state_and_fault_logging():
    from types import SimpleNamespace
    from unittest.mock import patch
    from simple_flight_mission.competition_node import CompetitionNode
    from simple_flight_mission.route_node import RouteNode
    runner = object.__new__(CompetitionNode)
    runner.options = SimpleNamespace(mode="execute")
    status = dict(reasons=["real_flight_fault"])
    with patch.object(RouteNode, "log_state") as log:
        runner.log_state(status)
    log.assert_called_once_with(status)
