"""Flight-safety snapshot evaluation; returns list of abort reasons."""


class MissionGuard:
    @staticmethod
    def evaluate(snapshot):
        reasons = []
        add = reasons.append
        if not snapshot.get("fcu_connected"):
            add("fcu disconnected")
        if not snapshot.get("armed"):
            add("fcu disarmed")
        expected_mode = snapshot.get("expected_mode")
        if expected_mode and snapshot.get("mode") != expected_mode:
            add("mode changed to " + str(snapshot.get("mode")))
        if not (snapshot.get("position_valid") and snapshot.get("height_valid")):
            add("position invalid")
        if snapshot.get("lio_age_s", 0.0) > snapshot.get("max_position_age_s", 0.3):
            add("position stale")
        if snapshot.get("position_error_m", 0.0) > snapshot.get("max_position_error_m", 0.5):
            add("position error excessive")
        if snapshot.get("horizontal_speed_mps", 0.0) > snapshot.get("max_horizontal_speed_mps", 0.8):
            add("horizontal speed excessive")
        if snapshot.get("vertical_speed_mps", 0.0) > snapshot.get("max_vertical_speed_mps", 0.6):
            add("vertical speed excessive")
        if snapshot.get("battery_voltage_v", 99.0) < snapshot.get("min_battery_voltage_v", 24.0):
            add("battery voltage low")
        return reasons
