#!/usr/bin/env python3
"""Name recordings from a PC time reference, without changing the system clock."""
import argparse
from datetime import datetime, timedelta, timezone
import json
import math
from pathlib import Path
import sys
import time
import uuid


def recording_stamp(reference, boot_id, uptime_s, fallback_unix_s):
    calibrated = False
    unix_s = fallback_unix_s
    try:
        elapsed = uptime_s - float(reference["uptime_s"])
        candidate = float(reference["computer_unix_s"]) + elapsed
        if reference["boot_id"] == boot_id and elapsed >= 0 and math.isfinite(candidate):
            unix_s, calibrated = candidate, True
    except (KeyError, TypeError, ValueError):
        pass
    stamp = datetime.fromtimestamp(unix_s, timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
    return stamp, calibrated


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sync-unix", type=float, help="Current PC Unix timestamp; naming only")
    parser.add_argument("--reference", type=Path, default=Path.home()/"simple_flight_ws/config/recording_clock.json")
    args = parser.parse_args()
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    uptime_s = time.clock_gettime(time.CLOCK_BOOTTIME)
    if args.sync_unix is not None:
        if not math.isfinite(args.sync_unix) or not 1577836800 <= args.sync_unix < 4102444800:
            parser.error("PC timestamp must be between 2020 and 2100")
        reference = dict(boot_id=boot_id, uptime_s=uptime_s, computer_unix_s=args.sync_unix)
        args.reference.parent.mkdir(parents=True, exist_ok=True)
        temp = args.reference.with_suffix("."+uuid.uuid4().hex+".tmp")
        temp.write_text(json.dumps(reference), encoding="utf-8")
        temp.replace(args.reference)
        stamp, _ = recording_stamp(reference, boot_id, uptime_s, time.time())
        print("Recording filename clock (Shanghai): "+stamp)
        return
    try:
        reference = json.loads(args.reference.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        reference = None
    stamp, calibrated = recording_stamp(reference, boot_id, uptime_s, time.time())
    if not calibrated:
        print("Recording time is UNSYNCED. Run the PC export script with -SyncOnly after each Nano reboot.", file=sys.stderr)
    print("competition_"+("" if calibrated else "UNSYNCED_")+stamp+"_"+uuid.uuid4().hex[:8])


if __name__ == "__main__":
    main()
