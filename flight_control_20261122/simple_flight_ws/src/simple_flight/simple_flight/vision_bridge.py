"""External pose aiding with a bench default; no flight command clients."""
import json
import math
import time
from collections import deque
from simple_flight.pose_adapter import PoseAdapter


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('simple_vision_bridge')
    defaults = {'input_topic': '/Odometry', 'output_topic': '/mavros/vision_pose/pose_cov',
                'state_topic': '/mavros/state', 'sensor_in_body': [0., 0., .05],
                'source_frame': 'camera_init', 'source_child_frame': 'body',
                'max_age_s': .3, 'max_gap_s': 1., 'position_sigma_m': .1,
                'angle_sigma_rad': .1, 'imu_topic': '/mavros/imu/data',
                'operation_mode': 'bench',
                'status_topic': '/simple_flight/vision_bridge/status'}
    params = {key: node.declare_parameter(key, value).value for key, value in defaults.items()}
    if params['operation_mode'] not in ('bench', 'manual_position'):
        raise ValueError('operation_mode must be bench or manual_position')
    if any(not math.isfinite(params[key]) or params[key] <= 0 for key in
           ('max_age_s', 'max_gap_s', 'position_sigma_m', 'angle_sigma_rad')):
        raise ValueError('Limits and assumed noise must be positive and finite')
    adapter = PoseAdapter(params['sensor_in_body'], params['max_age_s'], params['max_gap_s'])
    output = node.create_publisher(PoseWithCovarianceStamped, params['output_topic'], 10)
    status_pub = node.create_publisher(String, params['status_topic'], 10)
    state = {'connected': False, 'armed': False, 'heartbeat': None,
             'last_output': None, 'accepted': 0, 'rejected': 0, 'reason': 'waiting for FCU',
             'world_yaw_alignment_rad': None, 'operation_mode': params['operation_mode']}
    imu = {'message': None, 'received': None}
    candidates = deque(maxlen=20)

    def yaw(q):
        if not all(math.isfinite(x) for x in q) or abs(sum(x*x for x in q)-1.) > .02:
            raise ValueError('Invalid heading quaternion')
        x, y, z, w = q
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

    def on_imu(msg):
        imu.update(message=msg, received=time.monotonic())

    def on_state(msg):
        state.update(connected=msg.connected, armed=msg.armed, heartbeat=time.monotonic())
        if msg.armed and params['operation_mode'] == 'bench':
            adapter.fault = adapter.fault or 'Armed vehicle: bench bridge stopped'
        elif msg.armed and adapter.origin is None:
            adapter.fault = adapter.fault or 'Armed before disarmed stationary initialization'

    def on_pose(msg):
        nonlocal adapter
        now = time.monotonic()
        try:
            if adapter.fault:
                raise ValueError(adapter.fault)
            if (not state['connected'] or state['heartbeat'] is None
                    or now-state['heartbeat'] > 3.):
                raise ValueError('FCU must be connected with fresh state')
            # The origin and heading can only be created on the ground. Once
            # initialized, manual aiding keeps them across arm/disarm transitions.
            if state['armed'] and (params['operation_mode'] != 'manual_position'
                                   or adapter.origin is None):
                adapter.latch('Armed vehicle without initialized manual position aiding')
            if msg.header.frame_id != params['source_frame'] or msg.child_frame_id != params['source_child_frame']:
                adapter.latch('Unexpected source coordinate frame')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            stamp_ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
            if state['world_yaw_alignment_rad'] is None:
                current = imu['message']
                if current is None or now-imu['received'] > .2:
                    raise ValueError('Waiting for fresh FCU attitude for world heading alignment')
                age = (node.get_clock().now().nanoseconds-stamp_ns)/1.e9
                if not -.1 <= age <= params['max_age_s']:
                    raise ValueError('Stale source during heading alignment')
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                    raise ValueError('Invalid source during heading alignment')
                w = current.angular_velocity
                if math.sqrt(w.x*w.x+w.y*w.y+w.z*w.z) > .1:
                    candidates.clear()
                    raise ValueError('Keep airframe stationary for initial heading alignment')
                source_yaw = yaw((q.x, q.y, q.z, q.w))
                candidates.append(((p.x, p.y, p.z), source_yaw, stamp_ns))
                if len(candidates) < 20:
                    raise ValueError('Collecting stationary alignment samples')
                first_p, first_yaw, first_stamp = candidates[0]
                if (stamp_ns-first_stamp < 1_500_000_000
                        or any(math.dist(pos, first_p) > .02 for pos, _, _ in candidates)
                        or any(abs(math.remainder(angle-first_yaw, 2*math.pi)) > math.radians(2)
                               for _, angle, _ in candidates)):
                    raise ValueError('Need two seconds of stationary poses for alignment')
                iq = current.orientation
                alignment = math.remainder(yaw((iq.x, iq.y, iq.z, iq.w))-source_yaw, 2*math.pi)
                adapter = PoseAdapter(params['sensor_in_body'], params['max_age_s'],
                                      params['max_gap_s'], world_yaw=alignment)
                state['world_yaw_alignment_rad'] = alignment
                node.get_logger().info('Fixed initial world-heading alignment rad='+str(alignment))
            p_body, q_body = adapter.accept(stamp_ns, node.get_clock().now().nanoseconds,
                                            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            result = PoseWithCovarianceStamped()
            result.header.stamp = msg.header.stamp
            result.header.frame_id = 'odom'
            result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = p_body
            (result.pose.pose.orientation.x, result.pose.pose.orientation.y,
             result.pose.pose.orientation.z, result.pose.pose.orientation.w) = q_body
            # Explicit conservative assumptions, not calibrated uncertainty.
            for index in (0, 7, 14):
                result.pose.covariance[index] = params['position_sigma_m']**2
            for index in (21, 28, 35):
                result.pose.covariance[index] = params['angle_sigma_rad']**2
            output.publish(result)
            state.update(last_output=now, accepted=state['accepted']+1, reason='forwarding new measurements')
        except ValueError as exc:
            state.update(rejected=state['rejected']+1, reason=str(exc))

    def monitor():
        now = time.monotonic()
        if state['last_output'] is not None and now-state['last_output'] > params['max_gap_s']:
            adapter.fault = adapter.fault or 'Output gap: inspect and restart only disarmed on ground'
        report = {key: value for key, value in state.items() if key not in ('heartbeat', 'last_output')}
        report.update(fault=adapter.fault, origin=adapter.origin,
                      initialized=adapter.origin is not None,
                      last_output_age_s=None if state['last_output'] is None else now-state['last_output'])
        payload = json.dumps(report, allow_nan=False)
        message = String()
        message.data = payload
        status_pub.publish(message)
        node.get_logger().info(payload)

    node.create_subscription(State, params['state_topic'], on_state, 10)
    node.create_subscription(Imu, params['imu_topic'], on_imu,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(Odometry, params['input_topic'], on_pose,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_timer(2., monitor)
    node.get_logger().info('operation_mode='+params['operation_mode']+
                           '; disarmed stationary initialization required; no flight commands; '
                           'body offset approximate; input time retained; MAVROS owns ENU/NED conversion.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
