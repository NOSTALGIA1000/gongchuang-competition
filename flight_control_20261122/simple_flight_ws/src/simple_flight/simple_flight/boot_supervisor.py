"""Start localization once per boot and publish observable readiness.

This supervisor has no arm, mode, actuator, mission or setpoint interface.
An error is latched for inspection; healthy child processes keep running.
"""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import uuid
from concurrent.futures import ThreadPoolExecutor

from simple_flight.boot_policy import ReadinessGuard, decode_sys_status
from simple_flight.lidar_cdr import CloudView
from simple_flight.startup_retry import (FreshLioWindow, StartupRetryGate,
                                         current_generation_stamp, expected_startup_exit)


EXPECTED_PARAMETERS = dict(EKF2_EV_CTRL=11, EKF2_HGT_REF=3,
                           EKF2_EV_POS_X=0., EKF2_EV_POS_Y=0., EKF2_EV_POS_Z=0.)
SCRIPTS = dict(mavros='start_mavros_bench.sh', lidar='start_lidar.sh',
               fastlio='start_localization.sh', bridge='start_position_aiding.sh')


def next_startup_action(started, s):
    """Return at most one stage, without restarting or bypassing missing input."""
    if s.get('fault') or s.get('process_failure') or 'bridge' in started:
        return None
    if 'mavros' not in started:
        return 'mavros' if s.get('serial') else None
    if (not s.get('connected') or s.get('state_age', math.inf) > 3.
            or s.get('armed') or not s.get('network')):
        return None
    if 'lidar' not in started:
        return 'lidar'
    if s.get('lidar_age', math.inf) > .3 or s.get('imu_age', math.inf) > .2:
        return None
    if 'fastlio' not in started:
        return 'fastlio'
    if s.get('lio_age', math.inf) > .3 or not s.get('lio_stable'):
        return None
    if not s.get('parameters_ok'):
        return 'check_parameters'
    return 'bridge'


def probe_network():
    """Read the configured address and discovery route; never mutate networking."""
    iface = 'enP8p1s0'
    try:
        addresses = json.loads(subprocess.run(
            ['ip', '-j', '-4', 'addr', 'show', 'dev', iface],
            check=True, capture_output=True, text=True, timeout=1.5).stdout)
        routes = json.loads(subprocess.run(
            ['ip', '-j', 'route', 'get', '255.255.255.255'],
            check=True, capture_output=True, text=True, timeout=1.5).stdout)
        address_ok = any(item.get('local') == '192.168.1.5'
                         and item.get('prefixlen') == 24
                         for link in addresses for item in link.get('addr_info', []))
        carrier_ok = any('LOWER_UP' in link.get('flags', []) for link in addresses)
        route_ok = any(route.get('dev') == iface for route in routes)
        return address_ok and carrier_ok and route_ok, dict(
            address_ok=address_ok, carrier_ok=carrier_ok, route_ok=route_ok)
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        return False, dict(error=str(exc))


def finite_pose(pose):
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    return (all(map(math.isfinite, values))
            and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) <= .02)


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from livox_ros_driver2.msg import CustomMsg
    from mavros_msgs.msg import EstimatorStatus, Mavlink, State
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String

    class BootSupervisor(Node):
        def __init__(self):
            super().__init__('simple_flight_boot_supervisor')
            self.root = Path(self.declare_parameter(
                'workspace_root', '/home/jetson/simple_flight_ws').value)
            self.serial = Path('/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00')
            self.share = self.root / 'install/simple_flight/share/simple_flight'
            self.run_id = uuid.uuid4().hex
            self.started_monotonic = time.monotonic()
            try:
                self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            except OSError:
                self.boot_id = 'unavailable'
            self.logs = self.root / 'log' / ('boot_' + self.run_id)
            self.logs.mkdir(parents=True, exist_ok=True)
            self.status_path = self.root / 'log/boot_status.json'
            self.guard = ReadinessGuard(hold_s=3.)
            self.processes, self.log_handles, self.started = {}, {}, set()
            self.log_paths, self.process_history = {}, []
            self.startup_retry = StartupRetryGate()
            self.lio_window = FreshLioWindow()
            self.lio_spawned_ros_ns = None
            self.expected_lio_stop_pid = None
            self.armed_ever_seen = self.external_pose_ever_observed = False
            self.process_failure = None
            self.streams = {key: dict(received=None, stamp=None, rejected=0)
                            for key in ('lidar', 'imu', 'lio', 'vision', 'fcu_pose')}
            self.state = None
            self.state_at = self.estimator_at = self.sys_status_at = None
            self.position_valid = self.height_valid = self.rc_ready = self.prearm_ready = False
            self.bridge = {}
            self.bridge_at = self.bridge_initialized_at = None
            self.parameters_ok = False
            self.parameter_values = {}
            self.parameter_error = 'waiting for parameter service'
            self.parameter_future = None
            self.parameter_requested_at = self.parameter_last_attempt = None
            self.parameter_last_success = None
            self.network = False
            self.network_detail = {}
            self.network_at = None
            self.network_future = None
            self.network_requested_at = None
            self.workers = ThreadPoolExecutor(max_workers=1)
            self.last_report_state = None
            self.publisher = self.create_publisher(String, '/simple_flight/boot/status', 10)
            self.getter = self.create_client(GetParameters, '/mavros/param/get_parameters')
            # This monitor needs the newest complete scan, not a backlog.
            # Recover dropped DDS fragments while retaining all timestamp guards.
            # Keep this separate from the IMU/pose sensor-data profiles.
            raw_lidar_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            subscriptions = [
                (State, '/mavros/state', self.on_state, 10, False),
                (CustomMsg, '/livox/lidar', self.on_lidar, raw_lidar_qos, True),
                (Imu, '/livox/imu', self.on_imu, qos_profile_sensor_data, False),
                (Odometry, '/Odometry', self.on_lio, qos_profile_sensor_data, False),
                (PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov',
                 self.on_vision, qos_profile_sensor_data, False),
                (PoseStamped, '/mavros/local_position/pose', self.on_fcu_pose,
                 qos_profile_sensor_data, False),
                (EstimatorStatus, '/mavros/estimator_status', self.on_estimator,
                 qos_profile_sensor_data, False),
                (Mavlink, '/uas1/mavlink_source', self.on_mavlink,
                 qos_profile_sensor_data, False),
                (String, '/simple_flight/vision_bridge/status', self.on_bridge, 10, False),
            ]
            for cls, topic, callback, qos, raw in subscriptions:
                self.create_subscription(cls, topic, self.safe_callback(callback), qos, raw=raw)
            self.create_timer(1., self.tick)
            self.get_logger().info('Localization startup only; no flight commands; run_id=' + self.run_id)

        def latch(self, reason):
            if not self.guard.fault:
                self.guard.fault = str(reason)
                self.get_logger().error(str(reason))

        def safe_callback(self, callback):
            def guarded(message):
                try:
                    callback(message)
                except Exception as exc:
                    self.latch('Supervisor callback ' + callback.__name__ + ': ' + str(exc))
            return guarded

        @staticmethod
        def elapsed(now, received):
            return math.inf if received is None else max(0., now - received)

        def record(self, key, stamp_ns, valid):
            stream = self.streams[key]
            now_ns = self.get_clock().now().nanoseconds
            age = (now_ns - stamp_ns) / 1.e9
            if (not valid or stamp_ns <= 0 or not -.1 <= age <= .3
                    or (stream['stamp'] is not None and stamp_ns <= stream['stamp'])):
                stream['rejected'] += 1
                return False
            stream.update(received=time.monotonic(), stamp=stamp_ns)
            return True

        def stream_age(self, key, now, now_ns):
            stream = self.streams[key]
            if stream['received'] is None:
                return math.inf
            return max(self.elapsed(now, stream['received']),
                       (now_ns - stream['stamp']) / 1.e9)

        @staticmethod
        def stamp(message):
            return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

        def on_state(self, message):
            self.state = message
            self.state_at = time.monotonic()
            self.armed_ever_seen |= bool(message.armed)
            if message.armed and 'bridge' not in self.started:
                self.latch('Armed before supervised disarmed initialization; restart only on ground')

        def on_lidar(self, raw):
            try:
                cloud = CloudView(raw)
                valid = cloud.point_num > 0 and all(
                    math.isfinite(value) for index in range(min(64, cloud.point_num))
                    for value in cloud.point(index)[1:])
                self.record('lidar', cloud.stamp_ns, valid)
            except (ValueError, UnicodeError):
                self.streams['lidar']['rejected'] += 1

        def on_imu(self, message):
            a, w = message.linear_acceleration, message.angular_velocity
            self.record('imu', self.stamp(message),
                        all(map(math.isfinite, (a.x, a.y, a.z, w.x, w.y, w.z))))

        def on_lio(self, message):
            stamp = self.stamp(message)
            if not current_generation_stamp(stamp, self.lio_spawned_ros_ns):
                self.streams['lio']['rejected'] += 1
                return
            accepted = self.record('lio', stamp, finite_pose(message.pose.pose)
                                   and message.header.frame_id == 'camera_init'
                                   and message.child_frame_id == 'body')
            now = time.monotonic()
            self.lio_window.observe(now, accepted)
            if accepted:
                self.startup_retry.mark_fresh(now)

        def on_vision(self, message):
            # Even malformed or unsolicited external pose forbids rebuilding LIO.
            self.external_pose_ever_observed = True
            if 'bridge' not in self.started:
                self.latch('Unsolicited external pose before supervised bridge startup')
            self.record('vision', self.stamp(message), finite_pose(message.pose.pose)
                        and message.header.frame_id == 'odom')

        def on_fcu_pose(self, message):
            self.record('fcu_pose', self.stamp(message), finite_pose(message.pose))

        def on_estimator(self, message):
            self.position_valid = bool(message.pos_horiz_rel_status_flag
                                       or message.pos_horiz_abs_status_flag)
            self.height_valid = bool(message.pos_vert_abs_status_flag)
            self.estimator_at = time.monotonic()

        def on_mavlink(self, message):
            if (message.framing_status != 1 or message.sysid != 1
                    or message.compid != 1 or message.msgid != 1):
                return
            payload = struct.pack('<' + 'Q' * len(message.payload64), *message.payload64)
            self.rc_ready, self.prearm_ready = decode_sys_status(payload[:message.len])
            self.sys_status_at = time.monotonic()

        def on_bridge(self, message):
            report = json.loads(message.data)
            if not isinstance(report, dict):
                raise ValueError('Bridge status must be a JSON object')
            if 'bridge' not in self.started:
                return
            self.bridge = report
            self.bridge_at = time.monotonic()
            if report.get('operation_mode') != 'manual_position':
                self.latch('Unexpected bridge operation mode')
            if report.get('initialized') and self.bridge_initialized_at is None:
                self.bridge_initialized_at = self.bridge_at

        def update_network(self, now):
            if self.network_future is not None and self.network_future.done():
                self.network, self.network_detail = self.network_future.result()
                self.network_at = now
                self.network_future = None
            if (self.network_future is None and (self.network_requested_at is None
                                                 or now - self.network_requested_at >= 2.)):
                self.network_future = self.workers.submit(probe_network)
                self.network_requested_at = now
            if self.network_at is None or now - self.network_at > 6.:
                self.network = False

        def update_parameters(self, now, request=False):
            if self.parameter_future is not None:
                if self.parameter_future.done():
                    future, self.parameter_future = self.parameter_future, None
                    try:
                        response = future.result()
                        if len(response.values) != len(EXPECTED_PARAMETERS):
                            raise ValueError('incomplete parameter response')
                        values = {}
                        for name, value in zip(EXPECTED_PARAMETERS, response.values):
                            if value.type == 2:
                                values[name] = value.integer_value
                            elif value.type == 3 and math.isfinite(value.double_value):
                                values[name] = value.double_value
                            else:
                                raise ValueError('parameter not loaded: ' + name)
                        self.parameter_values = values
                        wrong = [name for name, expected in EXPECTED_PARAMETERS.items()
                                 if abs(values[name] - expected) > 1.e-8]
                        self.parameters_ok = not wrong
                        if wrong:
                            self.parameter_error = 'readback mismatch: ' + ', '.join(wrong)
                            self.latch('FCU localization parameter ' + self.parameter_error)
                        else:
                            self.parameter_error = None
                            self.parameter_last_success = now
                    except Exception as exc:
                        self.parameters_ok = False
                        self.parameter_error = str(exc)
                elif now - self.parameter_requested_at > 12.:
                    self.parameter_future.cancel()
                    self.parameter_future = None
                    self.parameters_ok = False
                    self.parameter_error = 'parameter service timeout'
            periodic = ('bridge' in self.started and self.parameter_last_success is not None
                        and now - self.parameter_last_success >= 10.)
            if (self.parameter_future is None and (request or periodic)
                    and (self.parameter_last_attempt is None
                         or now - self.parameter_last_attempt >= 3.)
                    and self.getter.service_is_ready()):
                self.parameter_last_attempt = self.parameter_requested_at = now
                self.parameter_future = self.getter.call_async(
                    GetParameters.Request(names=list(EXPECTED_PARAMETERS)))

        def check_processes(self):
            for name, process in self.processes.items():
                code = process.poll()
                if code is not None:
                    for entry in reversed(self.process_history):
                        if entry['pid'] == process.pid:
                            entry['returncode'] = code
                            break
                if name == 'fastlio' and expected_startup_exit(
                        self.startup_retry.phase, self.expected_lio_stop_pid, process.pid):
                    continue
                if code is not None and not self.process_failure:
                    self.process_failure = name + ' exited with code ' + str(code)
                    self.latch(self.process_failure)

        @staticmethod
        def group_alive(process):
            try:
                os.killpg(process.pid, 0)
                return True
            except ProcessLookupError:
                return False

        def start_process(self, name, startup_replacement=False):
            # This set is never cleared, including after a child exits.
            if self.guard.fault or (name in self.started and not startup_replacement):
                return
            if startup_replacement:
                if (name != 'fastlio' or self.startup_retry.phase != 'SPAWN_REQUESTED'
                        or self.armed_ever_seen or self.external_pose_ever_observed
                        or 'bridge' in self.started):
                    self.latch('Unsafe startup LIO replacement refused')
                    return
                previous = self.processes['fastlio']
                if previous.poll() is None or self.group_alive(previous):
                    self.latch('Previous LIO process group remains alive; replacement refused')
                    return
            if (self.state is None or self.state.armed
                    or self.elapsed(time.monotonic(), self.state_at) > 3.) and name != 'mavros':
                return
            suffix = '_retry' + str(self.startup_retry.retries) if startup_replacement else ''
            log_path = self.logs / (name + suffix + '.log')
            log = log_path.open('ab', buffering=0)
            spawned_ros_ns = self.get_clock().now().nanoseconds
            try:
                process = subprocess.Popen(
                    ['/bin/bash', str(self.share / 'scripts' / SCRIPTS[name])],
                    cwd=str(self.root), stdin=subprocess.DEVNULL, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True)
            except Exception:
                log.close()
                raise
            if name in self.log_handles:
                self.log_handles[name].close()
            self.log_handles[name] = log
            self.log_paths[name] = log_path
            self.processes[name] = process
            self.started.add(name)
            self.process_history.append(dict(name=name, pid=process.pid,
                                             spawned_monotonic=time.monotonic(),
                                             spawned_ros_ns=spawned_ros_ns,
                                             log=str(log_path), returncode=None))
            if name == 'fastlio':
                self.expected_lio_stop_pid = None
                self.lio_spawned_ros_ns = spawned_ros_ns
                self.streams['lio'].update(received=None, stamp=None)
                self.lio_window.reset()
                self.startup_retry.spawned(time.monotonic())
            self.get_logger().info('Started ' + name + ', pid=' + str(process.pid))

        def update_startup_retry(self, now, snapshot):
            process = self.processes.get('fastlio')
            if process is None or self.guard.fault:
                return
            alive = process.poll() is None
            if self.startup_retry.phase == 'STOPPING':
                alive = alive or self.group_alive(process)
            retry_snapshot = dict(snapshot, process_alive=alive,
                                  armed_ever_seen=self.armed_ever_seen,
                                  initialized_seen=self.guard.initialized_seen,
                                  bridge_ever_started='bridge' in self.started,
                                  external_pose_ever_observed=self.external_pose_ever_observed)
            action = self.startup_retry.next_action(now, retry_snapshot)
            if self.startup_retry.fault:
                self.latch(self.startup_retry.fault)
            if action == 'stop':
                if process.poll() is not None or os.getpgid(process.pid) != process.pid:
                    self.latch('LIO identity changed before startup stop request')
                    return
                self.expected_lio_stop_pid = process.pid
                os.killpg(process.pid, signal.SIGINT)
                self.get_logger().warning('Startup LIO remained stale; requested pre-aiding retry '
                                          + str(self.startup_retry.retries) + '/2')
            elif action == 'spawn':
                self.start_process('fastlio', startup_replacement=True)

        def snapshot(self, now):
            now_ns = self.get_clock().now().nanoseconds
            ages = {key + '_age': self.stream_age(key, now, now_ns) for key in self.streams}
            if ages['lio_age'] > .3:
                self.lio_window.reset()
            vision_seen = self.streams['vision']['received'] is not None
            initialized = bool(self.bridge.get('initialized')) and vision_seen
            if (self.bridge_initialized_at is not None and not vision_seen
                    and now - self.bridge_initialized_at > 1.):
                self.latch('Bridge initialized but no valid vision output observed within one second')
            return dict(
                ages, serial=self.serial.exists(), network=self.network,
                connected=bool(self.state and self.state.connected),
                armed=bool(self.state and self.state.armed),
                mode=self.state.mode if self.state else None,
                state_age=self.elapsed(now, self.state_at), initialized=initialized,
                lio_stable=self.lio_window.ready(now),
                bridge_fault=self.bridge.get('fault'),
                bridge_age=self.elapsed(now, self.bridge_at),
                estimator_age=self.elapsed(now, self.estimator_at),
                position_valid=self.position_valid, height_valid=self.height_valid,
                sys_status_age=self.elapsed(now, self.sys_status_at),
                rc_ready=bool(self.rc_ready and self.state and self.state.manual_input),
                prearm_ready=self.prearm_ready, parameters_ok=self.parameters_ok,
                process_failure=self.process_failure, fault=self.guard.fault)

        @staticmethod
        def json_safe(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {key: BootSupervisor.json_safe(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [BootSupervisor.json_safe(item) for item in value]
            return value

        def publish_status(self, now, snapshot, decision):
            report = dict(decision, boot_id=self.boot_id, run_id=self.run_id, pid=os.getpid(),
                          mode=snapshot.get('mode'), armed=snapshot.get('armed', False),
                          started_monotonic=self.started_monotonic,
                          uptime_s=now - self.started_monotonic,
                          updated_monotonic=now, valid_until_monotonic=now + 2.5,
                          snapshot=snapshot, network_detail=self.network_detail,
                          parameters=self.parameter_values, parameter_error=self.parameter_error,
                          processes={name: dict(pid=process.pid, returncode=process.poll(),
                                                log=str(self.log_paths[name]))
                                     for name, process in self.processes.items()},
                          process_history=self.process_history,
                          startup_retry=dict(retries=self.startup_retry.retries,
                                             generation=self.startup_retry.generation,
                                             expected_stop_pid=self.expected_lio_stop_pid,
                                             phase=self.startup_retry.phase,
                                             fresh_wait_since=self.startup_retry.fresh_wait_since,
                                             fault=self.startup_retry.fault,
                                             lio_spawned_ros_ns=self.lio_spawned_ros_ns,
                                             bridge_ever_started='bridge' in self.started,
                                             armed_ever_seen=self.armed_ever_seen,
                                             external_pose_ever_observed=self.external_pose_ever_observed),
                          rejected_measurements={key: stream['rejected']
                                                 for key, stream in self.streams.items()},
                          prearm_scope='SYS_STATUS health bit for the current flight mode',
                          note='No flight commands. An expired status file is not readiness.')
            payload = json.dumps(self.json_safe(report), ensure_ascii=False, allow_nan=False)
            try:
                message = String()
                message.data = payload
                self.publisher.publish(message)
            except Exception as exc:
                self.latch('Status publication failed: ' + str(exc))
            try:
                temporary = self.status_path.with_name('boot_status.' + self.run_id + '.tmp')
                temporary.write_text(payload + '\n', encoding='utf-8')
                temporary.replace(self.status_path)
            except OSError as exc:
                self.latch('Status file write failed: ' + str(exc))
            marker = (decision['state'], tuple(decision['reasons']))
            if marker != self.last_report_state:
                self.get_logger().info(payload)
                self.last_report_state = marker

        def tick(self):
            now = time.monotonic()
            try:
                self.check_processes()
                self.update_network(now)
                self.update_parameters(now)
                snapshot = self.snapshot(now)
                decision = self.guard.update(now, snapshot)
                self.update_startup_retry(now, snapshot)
                snapshot['fault'] = self.guard.fault
                action = (None if self.startup_retry.phase in ('STOPPING', 'SPAWN_REQUESTED')
                          else next_startup_action(self.started, snapshot))
                if action == 'check_parameters':
                    self.update_parameters(now, request=True)
                elif action is not None:
                    self.start_process(action)
                if self.guard.fault:
                    decision = self.guard.update(now, snapshot)
                self.publish_status(now, snapshot, decision)
            except Exception as exc:
                # Diagnostic and filesystem failures must not tear down aiding.
                self.latch('Supervisor timer failure: ' + str(exc))
                try:
                    self.publish_status(now, {}, dict(
                        state='FAULT', ready_to_arm=False, localization_active=False,
                        fault=self.guard.fault, reasons=[self.guard.fault]))
                except Exception as reporting_exc:
                    self.get_logger().error('Fault status unavailable: ' + str(reporting_exc))

        def stop_children(self):
            # Only explicit process/service shutdown reaches here, not WAIT/FAULT.
            for process in reversed(list(self.processes.values())):
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
            deadline = time.monotonic() + 10.
            for process in reversed(list(self.processes.values())):
                if process.poll() is None:
                    try:
                        process.wait(timeout=max(.01, deadline - time.monotonic()))
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
            for handle in self.log_handles.values():
                handle.close()
            self.workers.shutdown(wait=False, cancel_futures=True)

    rclpy.init()
    node = BootSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_children()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
