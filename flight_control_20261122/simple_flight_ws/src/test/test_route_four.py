"""Route-four sequence check, not clearance or flight-dynamics validation."""
from pathlib import Path

from simple_flight_mission.route_config import load_route_config
from test.test_route_flight import follow, route_rig


def test_route_four_visits_task_zones_then_returns_via_four_two_one_home():
    config = load_route_config(Path(__file__).resolve().parents[1] / "config/route_4.yaml")
    rig = route_rig(config)
    rig.active()
    visits, return_indices, previous = [], [], None
    for _ in range(3000):
        out = follow(rig)[0]
        key = (out.phase, getattr(out, "point_index", None))
        if key != previous and out.phase in ("POINT_HOLD", "RETURN_HOLD"):
            visits.append(key)
        if out.phase == "RETURN" and out.point_index not in return_indices:
            return_indices.append(out.point_index)
        previous = key
        assert out.phase not in ("HANDOVER", "EXITED"), out
        if out.phase == "DESCEND":
            break
    assert visits == [("POINT_HOLD", i) for i in range(5)] + [("RETURN_HOLD", None)]
    assert return_indices == [3, 1, 0, None]  # Final None is the takeoff origin.
    assert out.phase == "DESCEND"
