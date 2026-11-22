"""PX4 classic PARAM_VALUE uses bytewise encoding for integer parameters."""
import struct


def external_pose_plan(current, baseline):
    """Only position + height fusion; stop if another operator changed either."""
    targets = {'EKF2_EV_CTRL': 11, 'EKF2_HGT_REF': 3}
    changes = {}
    for name, desired in targets.items():
        if name not in current or name not in baseline:
            raise ValueError('Required parameter not backed up/read: '+name)
        if current[name] not in (baseline[name], desired):
            raise ValueError('Unexpected parameter state: '+name)
        if current[name] != desired:
            changes[name] = desired
    return changes


def decode_parameter(wire_value, parameter_type):
    if parameter_type == 9:
        return float(wire_value)
    formats = {1: '<B', 2: '<b', 3: '<H', 4: '<h', 5: '<I', 6: '<i'}
    if parameter_type not in formats:
        raise ValueError('Unsupported classic MAVLink parameter type')
    fmt = formats[parameter_type]
    raw = struct.pack('<f', wire_value)
    return struct.unpack(fmt, raw[:struct.calcsize(fmt)])[0]

