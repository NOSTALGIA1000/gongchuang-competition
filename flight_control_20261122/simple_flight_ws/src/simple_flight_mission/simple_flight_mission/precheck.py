"""Ground precheck: no control commands, only condition checks."""


class Precheck:
    @staticmethod
    def evaluate(snapshot):
        reasons = []
        if not snapshot.get("fcu_connected"):
            reasons.append("fcu disconnected")
        if snapshot.get("armed"):
            reasons.append("ground precheck requires disarm")
        expected_mode = snapshot.get("expected_mode")
        if expected_mode and snapshot.get("mode") != expected_mode:
            reasons.append("unexpected mode " + str(snapshot.get("mode")))
        if not (snapshot.get("position_valid") and snapshot.get("height_valid")):
            reasons.append("position invalid")
        if snapshot.get("lio_age_s", 99.0) > snapshot.get("max_position_age_s", 0.3):
            reasons.append("lio stale")
        if snapshot.get("battery_voltage_v", 0.0) < snapshot.get("min_battery_voltage_v", 24.0):
            reasons.append("battery low")
        return reasons
