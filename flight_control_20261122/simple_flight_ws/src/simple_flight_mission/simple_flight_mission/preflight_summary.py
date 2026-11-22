"""Readable final preflight result; never changes readiness decisions."""

LABELS = {
    "requires_position_mode": "飞控未处于Position模式",
    "main_switch_not_position": "主模式开关未处于Position对应档位",
    "rc_stale": "遥控输入未就绪",
    "rc_not_ready": "遥控输入未就绪",
    "rc_invalid": "遥控输入未就绪",
    "prearm_not_ready": "飞控预飞检查未通过",
    "throttle_not_low": "油门未处最低位",
    "requires_disarmed_on_ground": "需要未解锁、已落地状态",
    "fcu_disconnected": "飞控尚未连接",
    "state_stale": "飞控状态未收到或已过期",
    "boot_stale": "定位监督器状态未收到或已过期",
    "boot_not_ready": "定位监督器尚未就绪",
    "boot_localization_invalid": "定位链路尚未就绪",
    "boot_status_expired": "定位监督器状态已过期",
    "position_estimator_invalid": "飞控定位估计尚未有效",
    "camera_not_fresh": "相机画面未就绪或已过期",
    "payload_mapping_or_calibration_invalid": "舱门输出映射或标定不符",
    "parameters_stale": "飞控参数尚未读全或已过期",
    "mavros_raw_receiver_unavailable": "MAVROS目标接收接口未就绪",
    "mavros_raw_receiver_stale": "MAVROS目标接收接口尚在发现中",
    "not_stationary": "飞机尚未静止",
}


def format_preflight_summary(status):
    if (not isinstance(status, dict) or status.get("phase") != "CHECK"
            or status.get("operating_mode") != "check" or not isinstance(status.get("reasons"), list)):
        return "预检未通过：没有完整的只读检查结果，请查看运行日志。"
    reasons = status["reasons"]
    lines = ["预检未通过" if reasons else "预检通过"]
    display = lambda key: "未收到" if status.get(key) is None else str(status[key])
    lines.append("模式=" + display("actual_mode") + " | SWA=" + display("swa_pwm")
                 + " | 主模式(CH5)=" + display("main_switch_pwm") + " | 油门=" + display("throttle_pwm"))
    if reasons:
        messages = list(dict.fromkeys(LABELS.get(reason, reason) for reason in reasons))
        lines.append("阻塞：" + "；".join(messages))
    lines.append("只读检查，未发送飞行或舵机命令。")
    return "\n".join(lines)
