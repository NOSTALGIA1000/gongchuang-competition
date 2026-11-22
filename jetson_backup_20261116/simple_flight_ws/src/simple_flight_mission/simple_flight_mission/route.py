"""Route geometry in the local ENU frame used by MAVROS local coordinates."""
import math


class MissionRoute:
    """Build world-frame waypoints from body-frame directions."""

    def __init__(self, origin, yaw_rad, points):
        if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
            raise ValueError("origin must be three finite numbers")
        if not math.isfinite(yaw_rad):
            raise ValueError("yaw must be finite")
        if not points:
            raise ValueError("route needs at least one point")
        self.origin = tuple(origin)
        self.yaw_rad = yaw_rad
        self.points = list(points)

    def world_points(self):
        """Return [(x, y, z), ...] ENU waypoints offset from origin."""
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        result = []
        for _label, forward_m, left_m in self.points:
            if not (math.isfinite(forward_m) and math.isfinite(left_m)):
                raise ValueError("waypoint offsets must be finite")
            x = self.origin[0] + forward_m * c - left_m * s
            y = self.origin[1] + forward_m * s + left_m * c
            result.append((x, y, self.origin[2]))
        return result
