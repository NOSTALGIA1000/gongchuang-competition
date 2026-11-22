"""Wait for real CustomMsg data, without relying on a CLI discovery snapshot."""
import argparse
import math
import struct
import time

from simple_flight.lidar_cdr import CloudView


def sample_ready(raw, now_ns):
    try:
        cloud = CloudView(raw)
        return (cloud.point_num > 0 and cloud.stamp_ns > 0
                and -100_000_000 <= now_ns - cloud.stamp_ns <= 300_000_000)
    except (ValueError, UnicodeError, struct.error):
        return False


def message_ready(message, now_ns):
    """Validate a normal typed CustomMsg subscription message."""
    try:
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        return (message.point_num > 0 and stamp > 0
                and -100_000_000 <= now_ns - stamp <= 300_000_000)
    except (AttributeError, TypeError, ValueError):
        return False


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=10.)
    options = parser.parse_args(args)
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from livox_ros_driver2.msg import CustomMsg
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=[])
    node = rclpy.create_node('simple_wait_lidar_input')
    received = dict(ready=False, rejected=0)

    def on_cloud(message):
        if message_ready(message, node.get_clock().now().nanoseconds):
            received['ready'] = True
        else:
            received['rejected'] += 1

    # Use a normal typed callback. rclpy's raw=True path can fail to convert
    # the Livox sequence into a Python object on the Jetson build, which
    # aborts the supervisor before FAST-LIO is launched.
    node.create_subscription(CustomMsg, '/livox/lidar', on_cloud,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                             )
    begin = time.monotonic()
    try:
        while rclpy.ok() and not received['ready'] and time.monotonic() - begin < options.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if received['ready']:
        print('Received fresh nonempty Livox CustomMsg; localization input is available.', flush=True)
        return 0
    print('No fresh nonempty Livox CustomMsg within the input wait; rejected samples: '
          + str(received['rejected']), flush=True)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
