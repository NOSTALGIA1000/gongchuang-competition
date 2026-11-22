"""Local FLU pose adaptation. Never converts ENU/NED (MAVROS owns that)."""
import math


class PoseAdapter:
    def __init__(self, sensor_in_body=(0., 0., .05), max_age=.3,
                 max_gap=1., max_speed=2., max_turn_rate=math.pi, world_yaw=0.):
        if len(sensor_in_body) != 3 or not all(math.isfinite(x) for x in sensor_in_body):
            raise ValueError('Invalid mounting vector')
        self.sensor_in_body = tuple(sensor_in_body)
        if not math.isfinite(world_yaw):
            raise ValueError('Invalid world heading alignment')
        self.world_yaw = world_yaw
        self.max_age, self.max_gap = max_age, max_gap
        self.max_speed, self.max_turn_rate = max_speed, max_turn_rate
        self.origin = self.previous = None
        self.fault = None

    def latch(self, reason):
        self.fault = reason
        raise ValueError(reason + '; restart and revalidate on bench')

    def accept(self, stamp_ns, now_ns, position, quaternion):
        if self.fault:
            raise ValueError(self.fault)
        if (len(position) != 3 or len(quaternion) != 4
                or not all(math.isfinite(v) for v in (*position, *quaternion))):
            raise ValueError('Nonfinite/malformed pose')
        norm = math.sqrt(sum(v*v for v in quaternion))
        if not .99 <= norm <= 1.01:
            raise ValueError('Invalid unit quaternion')
        if stamp_ns <= 0 or not -.1 <= (now_ns-stamp_ns)/1.e9 <= self.max_age:
            raise ValueError('Stale/future/zero measurement time')
        q = tuple(v/norm for v in quaternion)
        if self.previous is not None:
            old_stamp, old_p, old_q = self.previous
            dt = (stamp_ns-old_stamp)/1.e9
            if dt == 0:
                raise ValueError('Duplicate measurement')
            if dt < 0:
                self.latch('Measurement clock reset')
            if dt > self.max_gap:
                self.latch('Measurement gap')
            step = math.dist(position, old_p)
            dot = min(1., abs(sum(a*b for a, b in zip(q, old_q))))
            turn = 2*math.acos(dot)
            if step > .1 + self.max_speed*dt or turn > .15 + self.max_turn_rate*dt:
                self.latch('Pose discontinuity')
        # Rotate the body->sensor lever arm into the local world, then subtract.
        x, y, z, w = q
        tx, ty, tz = self.sensor_in_body
        lever = ((1-2*y*y-2*z*z)*tx + (2*x*y-2*z*w)*ty + (2*x*z+2*y*w)*tz,
                 (2*x*y+2*z*w)*tx + (1-2*x*x-2*z*z)*ty + (2*y*z-2*x*w)*tz,
                 (2*x*z-2*y*w)*tx + (2*y*z+2*x*w)*ty + (1-2*x*x-2*y*y)*tz)
        body = tuple(p-d for p, d in zip(position, lever))
        if self.origin is None:
            self.origin = body
        self.previous = (stamp_ns, tuple(position), q)
        px, py, pz = (p-o for p, o in zip(body, self.origin))
        c, s = math.cos(self.world_yaw), math.sin(self.world_yaw)
        sh, ch = math.sin(self.world_yaw/2), math.cos(self.world_yaw/2)
        aligned_q = (ch*x-sh*y, ch*y+sh*x, ch*z+sh*w, ch*w-sh*z)
        return (c*px-s*py, s*px+c*py, pz), aligned_q
