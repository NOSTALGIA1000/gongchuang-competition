"""Independent door commands. Preview by default; never arm or change modes."""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time


MAV_CMD_DO_SET_ACTUATOR = 187
MAV_CMD_ACTUATOR_TEST = 310
GROUND_OPEN_HOLD_S = 1.5
GROUND_CLOSE_HOLD_S = 1.5


def _pulse(value):
    return type(value) in (int, float) and math.isfinite(value)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("配置字段重复：" + key)
        result[key] = value
    return result


def default_config():
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        pass
    else:
        try:
            return Path(get_package_share_directory("simple_flight_payload")) / "config" / "doors.json"
        except PackageNotFoundError:
            pass
    return Path(__file__).resolve().parents[1] / "config" / "doors.json"


@dataclass(frozen=True)
class DoorCommand:
    door: int
    action: str
    pwm_us: float
    min_us: float
    max_us: float

    @property
    def aux(self):
        return self.door + 6

    @property
    def params(self):
        # NaN means leave the other actuator unchanged. Never substitute zero.
        values = [math.nan] * 6 + [0.0]
        values[self.door - 1] = 2 * (self.pwm_us - self.min_us) / (self.max_us - self.min_us) - 1
        return tuple(values)

    def description(self):
        return dict(door=self.door, action=self.action, aux=self.aux,
                    actuator_set=self.door, pwm_us=self.pwm_us,
                    command=MAV_CMD_DO_SET_ACTUATOR,
                    params=[None if math.isnan(p) else p for p in self.params])


def build_command(config, door, action, pwm=None):
    if type(door) is not int or door not in (1, 2):
        raise ValueError("舱门只能是 1（消防/AUX7）或 2（救援/AUX8）")
    if action not in ("open", "close", "set") or (action != "set" and pwm is not None):
        raise ValueError("open/close 读取标定配置；只有 set 接受 --pwm")
    if not isinstance(config, dict) or set(config) != {"doors"}:
        raise ValueError("配置顶层只能包含 doors")
    doors = config["doors"]
    if not isinstance(doors, dict) or set(doors) != {"1", "2"}:
        raise ValueError("必须分别配置舱门 1 和 2")
    for settings in doors.values():
        required = {"min_us", "max_us", "open_us", "close_us"}
        angles = {"angle_min_deg", "angle_max_deg", "open_deg", "close_deg"}
        if not isinstance(settings, dict) or not required <= set(settings) or not set(settings) <= required | angles:
            raise ValueError("每个舱门必须包含 min_us/max_us/open_us/close_us")
        lo, hi = settings["min_us"], settings["max_us"]
        if not (_pulse(lo) and _pulse(hi) and 800 <= lo < hi <= 2200):
            raise ValueError("PWM 范围必须为 800..2200 内的递增有限数；以舵机额定范围为准")
        for key in ("open_us", "close_us"):
            value = settings[key]
            if value is not None and not (_pulse(value) and lo <= value <= hi):
                raise ValueError(key + " 必须为空或在该舱门 PWM 范围内")
        if settings["open_us"] is not None and settings["open_us"] == settings["close_us"]:
            raise ValueError("打开和关闭位置不能相同")
        if set(settings) & angles:
            if not angles <= set(settings) or not all(_pulse(settings[k]) for k in angles):
                raise ValueError("角度映射必须完整且有限")
            a, b = settings["angle_min_deg"], settings["angle_max_deg"]
            if not a < b or not all(a <= settings[k] <= b for k in ("open_deg", "close_deg")):
                raise ValueError("舱门角度超出配置映射范围")
            if settings["open_deg"] == settings["close_deg"]:
                raise ValueError("开关角度不能相同")
    settings = doors[str(door)]
    value = pwm if action == "set" else settings[action + "_us"]
    if value is None and action != "set" and action + "_deg" in settings:
        ratio = (settings[action + "_deg"] - settings["angle_min_deg"]) / (settings["angle_max_deg"] - settings["angle_min_deg"])
        value = round(settings["min_us"] + ratio * (settings["max_us"] - settings["min_us"]))
    if value is None:
        raise ValueError("舱门位置未标定：先用 set --pwm 调试，再填写 open_us/close_us")
    if not _pulse(value) or not settings["min_us"] <= value <= settings["max_us"]:
        raise ValueError("目标 PWM 必须在该舱门配置范围内")
    return DoorCommand(door, action, float(value), settings["min_us"], settings["max_us"])


def validate_output(command, parameters):
    expected = {f"PWM_AUX_FUNC{command.aux}": 300 + command.door,
                f"PWM_AUX_MIN{command.aux}": command.min_us,
                f"PWM_AUX_MAX{command.aux}": command.max_us}
    for name, value in expected.items():
        if parameters.get(name) != value:
            raise ValueError(f"飞控 {name}={parameters.get(name)!r}，需要 {value}；未发送命令")
    reverse = parameters.get("PWM_AUX_REV")
    if not _pulse(reverse) or int(reverse) != reverse or reverse < 0:
        raise ValueError("无法确认 PWM_AUX_REV；未发送命令")
    if int(reverse) & (1 << (command.aux - 1)):
        raise ValueError("目标 AUX 配置了反向范围，与当前 PWM 标定不符；未发送命令")


def command_request(command, request_type):
    request = request_type()
    request.broadcast = False
    request.command = MAV_CMD_DO_SET_ACTUATOR
    request.confirmation = 0
    for index, value in enumerate(command.params, 1):
        setattr(request, "param" + str(index), value)
    return request


def ground_test_request(command, request_type, timeout_s=2.0):
    """QGC slider namespace (function 1301/1302). timeout_s is how long PX4 holds the value."""
    if type(command.door) is not int or command.door not in (1, 2):
        raise ValueError("Ground test only permits the two payload doors")
    if not _pulse(timeout_s) or not 0.05 <= timeout_s <= 10:
        raise ValueError("ground-test 保持时间必须在 0.05..10 秒")
    request = request_type()
    request.broadcast = False
    request.command = MAV_CMD_ACTUATOR_TEST
    request.confirmation = 0
    values = (command.params[command.door - 1], float(timeout_s), 0.0, 0.0, float(1300 + command.door), 0.0, 0.0)
    for index, value in enumerate(values, 1):
        setattr(request, "param" + str(index), value)
    return request


def send_command(command, namespace="/mavros", timeout=8.0, ground_test=False, return_command=None):
    """Ground calibration transport. Flight integration must use async calls."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandLong
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters

    # Custom context must use a matching executor. Bare rclpy.spin_once()
    # builds the global executor against the default context and crashes with
    # AttributeError: __enter__ when that context was never initialized.
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node("simple_payload_door", context=context, use_global_arguments=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    state = [None, None]
    prefix = namespace.rstrip("/")

    def received(message):
        state[:] = [message, time.monotonic()]

    def guard():
        message, at = state
        if message is None or time.monotonic() - at > 1.5 or not message.connected:
            raise RuntimeError("没有新鲜的飞控连接状态；未发送命令")
        if message.armed:
            raise RuntimeError("此 CLI 仅用于未解锁地面标定；未发送命令")

    def wait_response(client, request, label, transmitted=False):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(label + "服务不可用")
        guard()
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done():
            future.cancel()
            suffix = "；命令可能已执行，不自动重发，请现场确认" if transmitted else "；未发送舱门命令"
            raise RuntimeError(label + "响应超时" + suffix)
        result = future.result()
        if result is None:
            raise RuntimeError(label + "没有返回有效响应")
        return result

    def read_params(names):
        request = GetParameters.Request()
        request.names = list(names)
        response = wait_response(parameters, request, "参数读取")
        values = {}
        for name, value in zip(names, response.values):
            if value.type == 2:
                values[name] = value.integer_value
            elif value.type == 3:
                values[name] = value.double_value
        return values

    def send_test(client, door_command, label, timeout_s):
        # ACTUATOR_TEST (310) matches QGC Aux sliders and actually drives AUX7/8.
        response = wait_response(
            client, ground_test_request(door_command, CommandLong.Request, timeout_s),
            label, transmitted=True)
        if not response.success or response.result != 0:
            raise RuntimeError(f"飞控未接受命令：success={response.success}, result={response.result}；不自动重发")
        return int(response.result)

    def send_set(client, door_command, label):
        # Updates the persistent actuator setpoint used after ACTUATOR_TEST expires.
        response = wait_response(client, command_request(door_command, CommandLong.Request),
                                 label, transmitted=True)
        if not response.success or response.result != 0:
            raise RuntimeError(f"飞控未接受命令：success={response.success}, result={response.result}；不自动重发")
        return int(response.result)

    def wait_s(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            guard()
            executor.spin_once(timeout_sec=.05)

    def ensure_home_180(home_command):
        """Make 180°/close the default restore target (not mid/90°)."""
        dis_name = f"PWM_AUX_DIS{home_command.aux}"
        before = read_params([dis_name]).get(dis_name)
        changed = before != home_command.pwm_us
        if changed:
            request = SetParameters.Request()
            parameter = Parameter()
            parameter.name = dis_name
            # PX4 PWM_* are integers in the param protocol.
            parameter.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                             integer_value=int(home_command.pwm_us))
            request.parameters = [parameter]
            response = wait_response(param_set, request, "写入上锁脉宽")
            if not response.results or not response.results[0].successful:
                reason = response.results[0].reason if response.results else "unknown"
                raise RuntimeError(f"无法把 {dis_name} 设为关门位 {home_command.pwm_us}：{reason}")
            after = read_params([dis_name]).get(dis_name)
            if after != home_command.pwm_us:
                raise RuntimeError(f"{dis_name} 写入后为 {after!r}，需要 {home_command.pwm_us}")
        # Persistent setpoint + short physical move to the close/home pose.
        send_set(client, home_command, "锁定关门设定")
        send_test(client, home_command, "关舱门到初始180°", GROUND_CLOSE_HOLD_S)
        wait_s(0.4)
        return dict(disarmed_pwm_name=dis_name, disarmed_pwm_before=before,
                    disarmed_pwm_us=home_command.pwm_us, disarmed_pwm_changed=changed)

    try:
        subscription = node.create_subscription(State, prefix + "/state", received,
                                                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        deadline = time.monotonic() + timeout
        while state[0] is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        guard()
        parameters = node.create_client(GetParameters, prefix + "/param/get_parameters")
        param_set = node.create_client(SetParameters, prefix + "/param/set_parameters")
        names = [f"PWM_AUX_FUNC{command.aux}", f"PWM_AUX_MIN{command.aux}",
                 f"PWM_AUX_MAX{command.aux}", "PWM_AUX_REV"]
        values = read_params(names)
        validate_output(command, values)
        if return_command is not None:
            validate_output(return_command, values)
        client = node.create_client(CommandLong, prefix + "/cmd/command")
        # Desired motion: home 180° → open 0° → home 180°. Never restore to mid/90°.
        if ground_test and command.action == "open" and return_command is not None:
            home = ensure_home_180(return_command)
            mav_result = send_test(client, command, "开舱门到0°", GROUND_OPEN_HOLD_S + .3)
            wait_s(GROUND_OPEN_HOLD_S)
            mav_result = send_test(client, return_command, "关舱门回180°", GROUND_CLOSE_HOLD_S + .3)
            wait_s(0.5)
            send_set(client, return_command, "保持关门180°设定")
            return dict(ack_accepted=True, mav_result=mav_result, physical_position_confirmed=False,
                        transport="ACTUATOR_TEST", sequence=["home_180", "open_0", "close_180"],
                        open_hold_s=GROUND_OPEN_HOLD_S, close_hold_s=GROUND_CLOSE_HOLD_S,
                        home_pwm_us=return_command.pwm_us, open_pwm_us=command.pwm_us, **home)
        if ground_test and command.action == "close":
            home = ensure_home_180(command)
            return dict(ack_accepted=True, mav_result=0, physical_position_confirmed=False,
                        transport="ACTUATOR_TEST", sequence=["home_180"], **home)
        if ground_test:
            mav_result = send_test(client, command, "舱门命令", 2.0)
            return dict(ack_accepted=True, mav_result=mav_result, physical_position_confirmed=False,
                        transport="ACTUATOR_TEST")
        response = wait_response(client, command_request(command, CommandLong.Request),
                                 "舱门命令", transmitted=True)
        if not response.success or response.result != 0:
            raise RuntimeError(f"飞控未接受命令：success={response.success}, result={response.result}；不自动重发")
        return dict(ack_accepted=True, mav_result=int(response.result), physical_position_confirmed=False,
                    transport="DO_SET_ACTUATOR")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)


def main(argv=None):
    parser = argparse.ArgumentParser(description="舱门 1=消防/AUX7；2=救援/AUX8。默认仅预览。")
    parser.add_argument("door", type=int, choices=(1, 2))
    parser.add_argument("action", choices=("open", "close", "set"))
    parser.add_argument("--config", type=Path)
    parser.add_argument("--pwm", type=float, help="set 调试脉宽，单位微秒")
    parser.add_argument("--execute", action="store_true", help="实际发送一次；仅未解锁地面标定")
    parser.add_argument("--ground-test", action="store_true",
                        help="地面测试：初始/关闭=180°，打开=0°；不解锁")
    parser.add_argument("--namespace", default="/mavros")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 30:
        parser.error("timeout 必须为 0..30 秒内的正数")
    if not args.namespace.startswith("/") or args.namespace.rstrip("/") == "":
        parser.error("namespace 必须为绝对 MAVROS 命名空间，如 /mavros")
    try:
        path = args.config if args.config is not None else default_config()
        config = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
        command = build_command(config, args.door, args.action, args.pwm)
        result = command.description()
        result.update(mode="execute" if args.execute else "preview", config=str(path))
        return_command = None
        if args.ground_test and args.action == "open":
            return_command = build_command(config, args.door, "close")
            result.update(command=MAV_CMD_ACTUATOR_TEST, transport="ACTUATOR_TEST",
                          sequence=["home_180", "open_0", "close_180"],
                          open_hold_s=GROUND_OPEN_HOLD_S, close_hold_s=GROUND_CLOSE_HOLD_S,
                          home_pwm_us=return_command.pwm_us, open_pwm_us=command.pwm_us)
        elif args.ground_test and args.action == "close":
            result.update(command=MAV_CMD_ACTUATOR_TEST, transport="ACTUATOR_TEST",
                          sequence=["home_180"], home_pwm_us=command.pwm_us)
        elif args.ground_test:
            request = ground_test_request(command, type("PreviewRequest", (), {}))
            result.update(command=MAV_CMD_ACTUATOR_TEST, transport="ACTUATOR_TEST",
                          ground_test_seconds=2,
                          params=[getattr(request, "param" + str(i)) for i in range(1, 8)])
        if args.execute:
            result.update(send_command(command, args.namespace, args.timeout, args.ground_test,
                                       return_command))
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 0
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print("舱门命令失败：" + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
