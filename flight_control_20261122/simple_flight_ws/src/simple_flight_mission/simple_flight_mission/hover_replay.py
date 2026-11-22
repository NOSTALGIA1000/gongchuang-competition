"""Replay recorded RC intent locally. No ROS imports or vehicle connections.

The ground conditions here only gate an offline RC test. This is not the
complete preflight checker for the future flight node.
"""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path

from .hover_takeover import HoverTakeover


def replay(rows, simulated_offboard_at=None):
    if (simulated_offboard_at is not None
            and (not math.isfinite(simulated_offboard_at) or simulated_offboard_at < 0)):
        raise ValueError("Simulation time must be finite and nonnegative")
    gate = HoverTakeover()
    latest = {}
    events = []
    confirmations = []
    modes = []
    previous_t = None
    rc_count = 0
    armed_observed = False
    simulation_done = False
    synthetic_rc_input = False

    def record(t, decision, source):
        if decision.action != "none":
            events.append(dict(asdict(decision), elapsed_s=t, source=source))

    def fresh_data(key, now, max_age):
        item = latest.get(key)
        return item[1] if item is not None and 0 <= now - item[0] <= max_age else {}

    def ground_ready(t):
        state = fresh_data("state", t, 1.5)
        extended = fresh_data("extended", t, 1.0)
        boot = fresh_data("boot", t, 2.5)
        return (state.get("mode") == "POSCTL" and state.get("armed") is False
                and state.get("connected") is True and state.get("manual_input") is True
                and extended.get("landed_state") == 1 and boot.get("ready_to_arm") is True)

    for item in rows:
        synthetic_rc_input = synthetic_rc_input or bool(item.get("synthetic_fields"))
        t = item["elapsed_s"]
        if (not isinstance(t, (int, float)) or isinstance(t, bool)
                or not math.isfinite(t) or t < 0):
            raise ValueError("Invalid recording time")
        if previous_t is not None and t < previous_t:
            raise ValueError("Recording times must be nondecreasing")
        if (simulated_offboard_at is not None and not simulation_done
                and t >= simulated_offboard_at):
            if gate.phase != "REQUESTED":
                raise ValueError("Simulation time must follow an accepted start trigger")
            decision = gate.confirm_offboard(simulated_offboard_at)
            record(simulated_offboard_at, decision, "simulation")
            if gate.phase != "ACTIVE":
                raise ValueError("Simulation cannot activate with stale RC input")
            confirmations.append({"elapsed_s": simulated_offboard_at, "source": "simulation"})
            simulation_done = True
        record(t, gate.poll(t), "timeout_check")
        key = item["stream"]
        data = item["data"]
        latest[key] = (t, data)
        if key == "state":
            mode = data.get("mode")
            if not modes or modes[-1] != mode:
                modes.append(mode)
            armed_observed = armed_observed or bool(data.get("armed"))
        if key == "rc":
            rc_count += 1
            record(t, gate.observe(data.get("channels"), t, entry_ready=ground_ready(t)), "rc")
        state = fresh_data("state", t, 1.5)
        if (simulated_offboard_at is None and gate.phase == "REQUESTED"
                and state.get("mode") == "OFFBOARD" and state.get("connected") is True):
            decision = gate.confirm_offboard(t)
            record(t, decision, "recorded_mode")
            if gate.phase == "ACTIVE":
                confirmations.append({"elapsed_s": t, "source": "recorded_mode"})
        previous_t = t
    if simulated_offboard_at is not None and not simulation_done:
        raise ValueError("Simulation time is outside the recording")
    return {
        "kind": "offline_rc_replay",
        "simulated_offboard": simulated_offboard_at is not None,
        "synthetic_rc_input": synthetic_rc_input,
        "offboard_confirmations": confirmations,
        "events": events,
        "final_phase": gate.phase,
        "rc_samples": rc_count,
        "armed_observed": armed_observed,
        "recorded_modes": modes,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Offline RC-intent replay only; cannot control an aircraft.")
    parser.add_argument("input", type=Path, help="Recorded JSONL input")
    parser.add_argument("--simulate-offboard-at", type=float, metavar="SECONDS",
                        help="Explicitly inject a simulated mode confirmation (not actual flight evidence)")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.output is not None and args.output.resolve() == args.input.resolve():
        parser.error("Output must not overwrite the input recording")
    try:
        with args.input.open(encoding="utf-8") as source:
            result = replay((json.loads(line) for line in source if line.strip()),
                            simulated_offboard_at=args.simulate_offboard_at)
        rendered = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered + "\n", encoding="utf-8")
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.error(str(error))
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
