"""Pure boot readiness policy; never sends flight commands or controls aiding."""
import math
import struct


def decode_sys_status(payload):
    """PX4 1.16: RC requires all three bits; prearm is health-only."""
    if len(payload) < 12:
        return False, False
    present, enabled, health = struct.unpack_from('<III', payload)
    common = present & enabled & health
    return bool(common & (1 << 16)), bool(health & (1 << 28))


class ReadinessGuard:
    def __init__(self, hold_s=3.):
        self.hold_s = hold_s
        self.fault = None
        self.healthy_since = None
        self.initialized_seen = False

    def update(self, now, s):
        self.initialized_seen |= bool(s.get('initialized'))
        reasons = []
        for key in ('process_failure', 'bridge_fault'):
            if s.get(key) and not self.fault:
                self.fault = str(s[key])
        # Outages before first initialization are normal startup waits. After
        # initialization do not silently resume a potentially shifted map.
        if self.initialized_seen and not self.fault:
            for key in ('lidar_age', 'imu_age', 'lio_age', 'vision_age'):
                if s.get(key, math.inf) > 1.:
                    self.fault = key + ' 超过 1 秒，需落地后检查并人工重启'
                    break
            if not self.fault and s.get('state_age', math.inf) > 3.:
                self.fault = '飞控状态超过 3 秒未更新'
        checks = [
            ('network', '等待雷达网口'), ('connected', '等待飞控连接'),
            ('initialized', '等待静止初始化'), ('parameters_ok', '等待飞控定位参数核对'),
            ('position_valid', '飞控水平位置尚未有效'),
            ('height_valid', '飞控高度尚未有效'),
        ]
        for key, label in checks:
            if not s.get(key, False):
                reasons.append(label)
        limits = dict(state_age=3., lidar_age=.3, imu_age=.2, lio_age=.3,
                      vision_age=.3, fcu_pose_age=.3, bridge_age=3.5,
                      estimator_age=3., sys_status_age=3.)
        for key, limit in limits.items():
            age = s.get(key, math.inf)
            if not math.isfinite(age) or age > limit or age < 0:
                reasons.append(key + ' 消息未就绪或已过期')
        # RC and prearm checks affect the display only, never the pose stream.
        if not s.get('rc_ready', False):
            reasons.append('飞控手动输入未有效')
        if not s.get('prearm_ready', False):
            reasons.append('飞控预飞检查尚未通过')
        if s.get('mode') != 'POSCTL':
            reasons.append('定位正常后请用遥控器切到 Position 定点档')
        if reasons or self.fault or s.get('armed', False):
            self.healthy_since = None
        elif self.healthy_since is None:
            self.healthy_since = now
        ready = self.healthy_since is not None and now - self.healthy_since >= self.hold_s
        if not reasons and not ready and not s.get('armed', False):
            reasons.append('正在累计连续正常 3 秒')
        active = (bool(s.get('initialized')) and not self.fault
                  and s.get('vision_age', math.inf) <= .3
                  and s.get('lio_age', math.inf) <= .3)
        state = ('FAULT' if self.fault else 'ARMED' if s.get('armed', False)
                 else 'READY' if ready else 'WAITING')
        return dict(state=state, ready_to_arm=bool(ready),
                    localization_active=bool(active), fault=self.fault,
                    reasons=[self.fault] if self.fault else reasons)
