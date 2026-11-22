"""Subscribe to MID-360 cloud/IMU only, and save bounded validation evidence."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from simple_flight.lidar_health import StreamStats
from simple_flight.lidar_cdr import CloudView


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu

    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--startup-timeout', type=float, default=20.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.duration <= 0 or args.startup_timeout <= 0:
        parser.error('durations must be positive')

    class Checker(Node):
        def __init__(self):
            super().__init__('simple_lidar_reception_check')
            self.stats = {'lidar': StreamStats(), 'imu': StreamStats()}
            self.snapshot = None
            self.snapshot_at = 0.0
            self.last_imu = None
            self.create_subscription(CustomMsg, '/livox/lidar', self.cloud, qos_profile_sensor_data, raw=True)
            self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)

        def stamp(self, message):
            return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

        def cloud(self, raw):
            now = time.monotonic()
            try:
                message = CloudView(raw)
                valid = (message.point_num > 0 and all(
                    math.isfinite(v) for i in range(min(64, message.point_num))
                    for v in message.point(i)[1:]))
            except (ValueError, UnicodeError):
                self.stats['lidar'].add(now, 0, False)
                return
            self.stats['lidar'].add(now, message.stamp_ns, valid, message.point_num)
            if now - self.snapshot_at >= 1.0 and valid:
                stride = max(1, (message.point_num + 4095) // 4096)
                self.snapshot = {
                    'frame_id': message.frame_id, 'stamp_ns': message.stamp_ns,
                    'timebase_ns': message.timebase, 'point_count': message.point_num,
                    'first_point_offset_ns': message.point(0)[0],
                    'last_point_offset_ns': message.point(message.point_num - 1)[0],
                    'points_xyz': [list(message.point(i)[1:]) for i in range(0, message.point_num, stride)],
                }
                self.snapshot_at = now

        def imu(self, message):
            values = [message.linear_acceleration.x, message.linear_acceleration.y,
                      message.linear_acceleration.z, message.angular_velocity.x,
                      message.angular_velocity.y, message.angular_velocity.z]
            self.stats['imu'].add(time.monotonic(), self.stamp(message), all(map(math.isfinite, values)))
            self.last_imu = {
                'frame_id': message.header.frame_id,
                'linear_acceleration_as_published': values[:3],
                'angular_velocity_as_published': values[3:],
                'note': 'Units must be checked against the selected driver before LIO integration.',
            }

    rclpy.init()
    node = Checker()
    start = time.monotonic()
    measurement_start = None
    next_progress = None
    interrupted = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if measurement_start is None:
                if all(s.count for s in node.stats.values()):
                    measurement_start = now
                    node.stats = {'lidar': StreamStats(), 'imu': StreamStats()}
                    next_progress = now + 30.0
                    print('Both streams detected; timed measurement started.', flush=True)
                elif now - start >= args.startup_timeout:
                    break
            elif now - measurement_start >= args.duration:
                break
            elif now >= next_progress:
                print(json.dumps({'elapsed_s': round(now - measurement_start, 1),
                                  'lidar_frames': node.stats['lidar'].count,
                                  'imu_messages': node.stats['imu'].count}), flush=True)
                next_progress = now + 30.0
    except KeyboardInterrupt:
        interrupted = True
    now = time.monotonic()
    measured = now - measurement_start if measurement_start is not None else 0.0
    streams = {
        'lidar': node.stats['lidar'].summary(now, minimum_hz=8.0, maximum_gap_s=1.0),
        'imu': node.stats['imu'].summary(now, minimum_hz=150.0, maximum_gap_s=0.5),
    }
    result = {
        'passed': (not interrupted and measured >= args.duration
                   and all(s['passed'] for s in streams.values())),
        'requested_duration_s': args.duration, 'measured_duration_s': measured,
        'startup_wait_s': (measurement_start or now) - start,
        'device_wall_clock_utc': datetime.now(timezone.utc).isoformat(),
        'measurement_clock': 'time.monotonic; no cross-device time synchronization assumed',
        'cloud_finite_check': 'first 64 xyz points per frame; point count checked for every frame',
        'interrupted': interrupted, 'streams': streams, 'last_imu': node.last_imu,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    if node.snapshot:
        args.output.with_name(args.output.stem + '_cloud.json').write_text(
            json.dumps(node.snapshot, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if result['passed'] else 1)
