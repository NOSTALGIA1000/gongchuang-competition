"""Observe Odometry only. No publishers, flight interfaces, or calibration writes."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from simple_flight.pose_health import PoseStats


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry

    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--warmup', type=float, default=10.)
    parser.add_argument('--require-static', action='store_true',
                        help='Only set when a person confirms the airframe is stationary.')
    parser.add_argument('--max-drift', type=float, default=.1)
    parser.add_argument('--max-rotation-deg', type=float, default=3.)
    parser.add_argument('--max-age', type=float, default=.3,
                        help='Maximum host time minus measurement stamp, seconds; not hardware latency.')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if min(args.duration, args.max_drift, args.max_rotation_deg, args.max_age) <= 0 or args.warmup < 0:
        parser.error('duration/limits must be positive; warmup must be nonnegative')
    rclpy.init()
    node = rclpy.create_node('simple_pose_reception_check')
    state = {'stats': PoseStats(), 'first_receive': None, 'samples': [], 'measurement_start': None}

    def callback(message):
        now = time.monotonic()
        if state['first_receive'] is None:
            state['first_receive'] = now
        if state['measurement_start'] is None:
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        position = [p.x, p.y, p.z]
        quaternion = [q.x, q.y, q.z, q.w]
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        valid = state['stats'].add(now, stamp, position, quaternion,
                                   message.header.frame_id, message.child_frame_id)
        age = (node.get_clock().now().nanoseconds - stamp) / 1.e9
        state['samples'].append({
            'elapsed_s': now - state['measurement_start'], 'stamp_ns': stamp,
            'position_m': position if valid else None,
            'quaternion_xyzw': quaternion if valid else None,
            'host_stamp_age_s': age, 'valid': valid,
        })

    node.create_subscription(Odometry, '/Odometry', callback,
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    begin = time.monotonic()
    progress = begin
    interrupted = False
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if state['measurement_start'] is None:
                if state['first_receive'] is not None and now - state['first_receive'] >= args.warmup:
                    state['measurement_start'] = now
                    progress = now + 30.
                    print('Odometry detected; warmup complete; measurement started.', flush=True)
                elif state['first_receive'] is None and now - begin >= 30.:
                    break
            elif now - state['measurement_start'] >= args.duration:
                break
            elif now >= progress:
                print(json.dumps({'elapsed_s': round(now-state['measurement_start'], 1),
                                  'messages': state['stats'].stream.count,
                                  'maximum_displacement_m': state['stats'].maximum_displacement}), flush=True)
                progress = now + 30.
    except KeyboardInterrupt:
        interrupted = True
    now = time.monotonic()
    measured = now - state['measurement_start'] if state['measurement_start'] is not None else 0.
    result = state['stats'].summary(now, args.max_drift, args.max_rotation_deg)
    ages = sorted(sample['host_stamp_age_s'] for sample in state['samples'])
    fresh = bool(ages) and ages[0] >= -.1 and ages[-1] <= args.max_age
    result.update({
        'passed': (not interrupted and measured >= args.duration and fresh
                   and result['output_passed']
                   and (not args.require_static or result['static_goal_passed'])),
        'requested_duration_s': args.duration, 'measured_duration_s': measured,
        'warmup_s': args.warmup, 'static_confirmed_by_operator': args.require_static,
        'freshness_passed': fresh,
        'freshness_limit_s': args.max_age,
        'host_stamp_age_s': ({'min': ages[0], 'max': ages[-1],
                              'p95': ages[int((len(ages)-1)*.95)]} if ages else None),
        'device_wall_clock_utc': datetime.now(timezone.utc).isoformat(),
        'measurement_clock': 'time.monotonic', 'interrupted': interrupted,
        'limitation': 'Estimator output consistency only; no ground-truth accuracy, extrinsic or hardware time calibration is proved.',
    })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    args.output.with_name(args.output.stem+'_samples.json').write_text(
        json.dumps(state['samples'], allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if result['passed'] else 1)
