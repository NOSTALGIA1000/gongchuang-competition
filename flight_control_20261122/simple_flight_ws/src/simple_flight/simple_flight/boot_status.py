"""Read a fresh supervisor result without opening the flight-controller port."""
import argparse
import json
import math
import os
from pathlib import Path
import time


def validate_status(data, boot_id, now, pid_alive):
    try:
        updated = float(data['updated_monotonic'])
        expires = float(data['valid_until_monotonic'])
        if data['boot_id'] != boot_id or not data.get('run_id'):
            return '状态来自旧开机或缺少运行标识'
        if not math.isfinite(updated) or not math.isfinite(expires):
            return '状态时间无效'
        if updated > now or now > expires or expires - updated > 5.:
            return '状态已过期，请检查开机服务'
        if not pid_alive(int(data['pid'])):
            return '状态进程已退出'
    except (KeyError, ValueError, TypeError):
        return '状态内容不完整'
    return None


def _alive(pid):
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def main(args=None):
    parser = argparse.ArgumentParser(description='检查手动定点开机状态，不发送飞行指令')
    parser.add_argument('--status-file', default='/home/jetson/simple_flight_ws/log/boot_status.json')
    parser.add_argument('--json', action='store_true')
    ns = parser.parse_args(args)
    try:
        data = json.loads(Path(ns.status_file).read_text())
        error = validate_status(data, Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                                time.monotonic(), _alive)
    except (OSError, ValueError) as exc:
        data, error = {}, '未读到开机状态：' + str(exc)
    if ns.json:
        print(json.dumps(dict(data, status_error=error), ensure_ascii=False, indent=2))
    elif error:
        print('未就绪：' + error)
    else:
        print('模式：{}；解锁：{}；定位持续接入：{}'.format(
            data.get('mode', '?'), data.get('armed', False), data.get('localization_active', False)))
        if data.get('ready_to_arm'):
            print('定点地面条件已就绪，可由现场人员尝试遥控器解锁。')
        elif data.get('fault'):
            print('故障：' + str(data['fault']))
        elif data.get('armed'):
            print('飞控已解锁；此命令仅显示状态。')
        else:
            print('未就绪：' + '；'.join(data.get('reasons') or [data.get('state', '?')] ))
    raise SystemExit(0 if not error and data.get('ready_to_arm') else 2)
