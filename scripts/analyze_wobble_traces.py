#!/usr/bin/env python3
"""Offline analysis of SO-ARM101 PassiveBackendTrace and GUI teleop JSONL.

Read-only: no imports from hardware, no serial operations, no controller changes.
Sparse backend traces contain actual writes, not suppressed planner waypoints.
Cycle-level hold statistics are therefore reconstructed on a nominal time grid.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path

JOINTS = (
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "so101_gripper",
)
TICKS_PER_RAD = 4095.0 / (2.0 * math.pi)
STARTS = {"leg_start", "condition_start", "reset_start", "replay_start"}
ENDS = {"leg_end", "condition_end", "reset_end", "replay_end"}


def load(path: Path) -> list[dict]:
    result = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        item = json.loads(line)
        if not isinstance(item, dict):
            raise ValueError(f"{path}:{number} is not a JSON object")
        result.append(item)
    if not result:
        raise ValueError(f"{path}: empty trace")
    return result


def quantile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    v = sorted(values)
    p = (len(v) - 1) * q
    lo, hi = math.floor(p), math.ceil(p)
    return v[lo] * (hi - p) + v[hi] * (p - lo) if lo != hi else v[lo]


def segments(rows: list[dict]) -> list[tuple[str, list[dict]]]:
    found = []
    opened = None
    for index, event in enumerate(rows):
        if event.get("event") != "marker":
            continue
        marker = str(event.get("marker", ""))
        if marker in STARTS:
            opened = (index, str(event.get("destination") or event.get("condition") or marker))
        elif marker in ENDS and opened is not None:
            start, label = opened
            found.append((f"{label}#{len(found) + 1}", rows[start:index + 1]))
            opened = None
    if found:
        return found
    return [("whole_trace", rows)]


def timestamp(row: dict) -> float | None:
    for key in ("monotonic_s", "elapsed_s"):
        if row.get(key) is not None:
            return float(row[key])
    if row.get("event") == "teleop_frame":
        if row.get("leader_timestamp") is not None:
            return float(row["leader_timestamp"])
        if row.get("sample") is not None and row.get("frequency_hz"):
            return float(row["sample"]) / float(row["frequency_hz"])
    return None


def raw_gripper(value: float, raw_range: tuple[int, int] | None, reverse: bool) -> int | None:
    if raw_range is None:
        return None
    normalized = 1.0 - value if reverse else value
    return round(raw_range[0] + normalized * (raw_range[1] - raw_range[0]))


def channel_events(rows: list[dict], joint: str, raw_range: tuple[int, int] | None,
                   reverse: bool) -> tuple[list[tuple], list[tuple]]:
    commands, feedback = [], []
    for row in rows:
        time_s = timestamp(row)
        if time_s is None:
            continue
        event = row.get("event")
        if joint == "so101_gripper":
            if event in ("tool_command", "tool_feedback") and row.get("actuator") == joint:
                value = float(row["position"])
                entry = (time_s, raw_gripper(value, raw_range, reverse), value, row)
                (commands if event == "tool_command" else feedback).append(entry)
            elif event == "teleop_frame":
                for key, target in (("command_gripper", commands), ("actual_gripper", feedback)):
                    if row.get(key) is not None:
                        value = float(row[key])
                        target.append((time_s, raw_gripper(value, raw_range, reverse), value, row))
            continue
        if event in ("command", "feedback", "teleop_frame"):
            is_gui = event == "teleop_frame"
            for is_command in ((True, False) if is_gui else (event == "command",)):
                raw_key = ("command_joints_raw" if is_command else "actual_joints_raw") if is_gui else "joints_raw"
                rad_key = ("command_joints_rad" if is_command else "actual_joints_rad") if is_gui else "joints_rad"
                raws, radians = row.get(raw_key), row.get(rad_key)
                if not isinstance(raws, dict) and not isinstance(radians, dict):
                    continue
                raw = raws.get(joint) if isinstance(raws, dict) else None
                rad = radians.get(joint) if isinstance(radians, dict) else None
                if raw is not None or rad is not None:
                    (commands if is_command else feedback).append(
                        (time_s, int(raw) if raw is not None else None,
                         float(rad) if rad is not None else None, row)
                    )
    return sorted(commands, key=lambda row: row[0]), sorted(feedback, key=lambda row: row[0])


def servo_speed_raw(row: dict, joint: str) -> int | None:
    """Read effective speed from the passive trace without guessing hardware EEPROM."""
    value = row.get("speed_raw")
    if isinstance(value, dict):
        value = value.get(joint)
    return int(value) if isinstance(value, (int, float)) else None


def run_lengths(values: list[int]) -> list[int]:
    """Length in zero-increment *intervals*, not number of equal positions."""
    result, count = [], 0
    for previous, current in zip(values, values[1:]):
        if current == previous:
            count += 1
        elif count:
            result.append(count)
            count = 0
    if count:
        result.append(count)
    return result


def sample_hold(entries: list[tuple], query: float) -> tuple | None:
    index = bisect.bisect_right([e[0] for e in entries], query) - 1
    return entries[index] if index >= 0 else None


def pearson(left: list[float], right: list[float]) -> float | None:
    if len(left) < 5:
        return None
    a, b = statistics.mean(left), statistics.mean(right)
    da, db = [x - a for x in left], [x - b for x in right]
    denominator = math.sqrt(sum(x*x for x in da) * sum(x*x for x in db))
    return sum(x*y for x, y in zip(da, db)) / denominator if denominator > 1e-12 else None


def lag_ms(commands: list[tuple], feedback: list[tuple], dt: float,
           max_lag_s: float) -> float | None:
    """First-difference correlation, linear-interpolated encoder feedback.

    This is only a coarse lag estimate: sparse feedback and serial jitter
    limit its resolution. Never treat the peak as a measured servo latency.
    """
    c = [e for e in commands if e[1] is not None]
    f = [e for e in feedback if e[1] is not None]
    if len(c) < 5 or len(f) < 5:
        return None
    start, end = max(c[0][0], f[0][0]), min(c[-1][0], f[-1][0])
    n = math.floor((end - start) / dt) + 1
    if n < 12 or n > 100000:
        return None
    ft = [e[0] for e in f]
    xs, ys = [], []
    for i in range(n):
        t = start + i * dt
        cmd = sample_hold(c, t)
        j = bisect.bisect_right(ft, t) - 1
        if cmd is None or j < 0 or j + 1 >= len(f):
            continue
        ta, ra = f[j][0], f[j][1]
        tb, rb = f[j + 1][0], f[j + 1][1]
        fraction = (t - ta) / (tb - ta) if tb > ta else 0.0
        xs.append(float(cmd[1]))
        ys.append(float(ra) + (float(rb) - float(ra)) * fraction)
    vx = [b-a for a,b in zip(xs, xs[1:])]
    vy = [b-a for a,b in zip(ys, ys[1:])]
    if sum(v != 0 for v in vx) < 4:
        return None
    best = (-2.0, 0)
    for shift in range(min(math.floor(max_lag_s / dt), len(vx) // 3) + 1):
        correlation = pearson(vx[:len(vx)-shift] if shift else vx,
                              vy[shift:] if shift else vy)
        if correlation is not None and correlation > best[0]:
            best = (correlation, shift)
    return round(best[1] * dt * 1000, 2) if best[0] >= 0.2 else None


def analyze_joint(rows: list[dict], joint: str, frequency: float,
                  raw_range: tuple[int, int] | None, reverse: bool,
                  max_lag_s: float, mode: str) -> dict:
    commands, feedback = channel_events(rows, joint, raw_range, reverse)
    result = {
        "joint": joint, "commands": len(commands), "feedback": len(feedback),
        "zero_runs": None, "max_zero_cycles": None, "zero_cycles_fraction": None,
        "tick_impulses_final25": None, "tick_impulses_per_s_final25": None,
        "hunting_reversals": None, "following_peak": None, "following_rms": None,
        "following_unit": "normalized" if joint == "so101_gripper" else "rad",
        "lag_ms": None, "subtick_tail_ms_estimate": None,
        "quantized_flat_tail_ms": None, "longest_write_gap_ms": None,
        "write_interval_p95_ms": None, "servo_speed_raw_min": None,
        "servo_speed_raw_max": None, "servo_speed_raw_changes": None,
        "post_gap_speed_raw_max": None, "post_gap_speed_raw_samples": None,
        "notes": [],
    }
    if not commands:
        result["notes"].append("no commands for joint")
        return result
    speeds = [(cmd[0], servo_speed_raw(cmd[3], joint)) for cmd in commands]
    known_speeds = [value for _, value in speeds if value is not None]
    if known_speeds:
        result["servo_speed_raw_min"] = min(known_speeds)
        result["servo_speed_raw_max"] = max(known_speeds)
        result["servo_speed_raw_changes"] = sum(
            a is not None and b is not None and a != b
            for a, b in zip((x[1] for x in speeds), (x[1] for x in speeds[1:]))
        )
        post_gap = [
            value for (previous_time, _), (current_time, value) in zip(speeds, speeds[1:])
            if value is not None and current_time - previous_time >= 3.0 / frequency - 1e-6
        ]
        result["post_gap_speed_raw_max"] = max(post_gap, default=None)
        result["post_gap_speed_raw_samples"] = len(post_gap)
    if len(commands) > 1:
        gaps = [1000*(b[0]-a[0]) for a,b in zip(commands, commands[1:])]
        result["longest_write_gap_ms"] = round(max(gaps), 2)
        result["write_interval_p95_ms"] = round(quantile(gaps, 0.95), 2)
    if mode == "final_target":
        result["notes"].append("one-shot endpoint: cycle-level quantization metrics not applicable")
    dt = 1.0 / frequency
    raw_commands = [e for e in commands if e[1] is not None]
    if not raw_commands:
        result["notes"].append("raw ticks unavailable; provide real gripper calibration if applicable")
    if mode != "final_target" and len(raw_commands) >= 2:
        start, end = raw_commands[0][0], raw_commands[-1][0]
        n = math.floor((end-start)/dt + 1e-6) + 1
        if 2 <= n <= 100000:
            times = [start+i*dt for i in range(n)]
            held = [sample_hold(raw_commands, t) for t in times]
            ticks = [int(e[1]) if e is not None else int(raw_commands[0][1]) for e in held]
            runs = run_lengths(ticks)
            result["zero_runs"] = dict(sorted(Counter(runs).items()))
            result["max_zero_cycles"] = max(runs, default=0)
            result["zero_cycles_fraction"] = round(sum(runs)/(n-1), 5)
            delta = [b-a for a,b in zip(ticks, ticks[1:])]
            tail_start = max(1, math.ceil((n-1)*0.75))
            impulse = sum(delta[i] != 0 and delta[i-1] == 0
                          for i in range(tail_start, len(delta)))
            result["tick_impulses_final25"] = impulse
            tail_seconds = (n-1-tail_start)*dt
            result["tick_impulses_per_s_final25"] = (
                round(impulse/tail_seconds, 3) if tail_seconds > 0 else None
            )
            suffix_zeros = 0
            for change in reversed(delta):
                if change != 0:
                    break
                suffix_zeros += 1
            result["quantized_flat_tail_ms"] = round(suffix_zeros*dt*1000, 2)
            # Sparse-write interval average, not exact missing float waypoints.
            intervals = []
            for a,b in zip(raw_commands, raw_commands[1:]):
                cycles = max(1, (b[0]-a[0])/dt)
                desired_float_delta = (abs(float(b[2])-float(a[2])) * TICKS_PER_RAD
                                       if joint != "so101_gripper" and a[2] is not None and b[2] is not None
                                       else abs(float(b[1])-float(a[1])))
                ticks_per_cycle = desired_float_delta/cycles
                intervals.append((b[0]-a[0], ticks_per_cycle))
            tail_s = 0.0
            for seconds, ticks_per_cycle in reversed(intervals):
                if ticks_per_cycle >= 1.0:
                    break
                tail_s += seconds
            result["subtick_tail_ms_estimate"] = round(tail_s*1000, 2)
            result["notes"].append("nominal-grid held targets; sub-tick tail estimated from sparse writes")
        else:
            result["notes"].append("insufficient or excessive nominal cycles")
    # Match measured samples to the last goal actually transmitted.
    errors = []
    for measured in feedback:
        goal = sample_hold(commands, measured[0])
        if goal is None:
            continue
        if measured[2] is not None and goal[2] is not None:
            errors.append(float(goal[2])-float(measured[2]))
    if errors:
        result["following_peak"] = round(max(abs(e) for e in errors), 6)
        result["following_rms"] = round(math.sqrt(sum(e*e for e in errors)/len(errors)), 6)
    if raw_commands and len(feedback) >= 2:
        goal_deltas = [b[1]-a[1] for a,b in zip(raw_commands, raw_commands[1:])]
        monotonic = all(v >= 0 for v in goal_deltas) or all(v <= 0 for v in goal_deltas)
        measured_raw = [e[1] for e in feedback if e[1] is not None and
                        raw_commands[0][0] <= e[0] <= (feedback[-1][0] if mode == "final_target" else raw_commands[-1][0])]
        velocities = [b-a for a,b in zip(measured_raw, measured_raw[1:]) if b != a]
        if monotonic:
            result["hunting_reversals"] = sum(a*b < 0 for a,b in zip(velocities, velocities[1:]))
        else:
            result["notes"].append("command direction reverses; hunting index not comparable")
        if mode != "final_target":
            result["lag_ms"] = lag_ms(commands, feedback, dt, max_lag_s)
    return result


def analyze(path: Path, label: str, frequency_override: float | None,
            raw_range: tuple[int,int] | None, reversed_gripper: bool,
            segment_filter: str | None, max_lag_s: float) -> list[dict]:
    rows = load(path)
    meta = rows[0]
    frequency = frequency_override or float(
        meta.get("command_frequency_hz") or meta.get("frequency_hz") or
        next((float(row["frequency_hz"]) for row in rows
              if row.get("event") == "teleop_frame" and row.get("frequency_hz")), None) or
        (20 if meta.get("kind") in ("exact_teleop_command_replay",
                                   "poststudy_exact_teleop_command_replay") else 50)
    )
    if frequency <= 0 or not math.isfinite(frequency):
        raise ValueError("command frequency must be positive")
    result = []
    for segment_name, data in segments(rows):
        if segment_filter and segment_filter.lower() not in segment_name.lower():
            continue
        mode = str(meta.get("execution_mode", ""))
        if not mode:
            mode = "final_target" if "final_target" in label.lower() else "streamed"
        values = [analyze_joint(data, j, frequency, raw_range, reversed_gripper,
                                max_lag_s, mode) for j in JOINTS]
        guard_times = [float(row["call_ms"]) for row in data
                       if row.get("event") == "hardware_state"
                       and row.get("call_ms") is not None]
        write_times = [float(row["call_ms"]) for row in data
                       if row.get("event") == "command"
                       and row.get("call_ms") is not None]
        result.append({
            "trace": label, "path": str(path), "segment": segment_name, "mode": mode,
            "frequency_hz": frequency, "source_kind": meta.get("kind"),
            "calibration_id": meta.get("calibration_id"), "joints": values,
            "guard_poll_count": len(guard_times),
            "guard_poll_p95_ms": round(quantile(guard_times, 0.95), 2) if guard_times else None,
            "goal_write_count": len(write_times),
            "goal_write_p95_ms": round(quantile(write_times, 0.95), 2) if write_times else None,
            "limitations": [
                "No suppressed continuous planner waypoints are present in passive traces.",
                "Nominal-cycle holds and sub-tick tails are reconstructions/estimates, not per-cycle bus captures.",
                "Following error is matched to latest sent command, not an unsent planner setpoint.",
                "Lag uses resampled encoder feedback and cannot resolve latency below feedback cadence.",
                "Gripper raw ticks require its real calibrated range and drive mode.",
                "Guard/write call durations include Python wrappers and are not per-packet bus turnaround timings.",
                "Raw servo speed settings are command register values; do not interpret them as verified degrees/second.",
            ],
        })
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", action="append", required=True, metavar="NAME=FILE")
    parser.add_argument("--frequency", type=float, help="override cadence for every trace")
    parser.add_argument("--segment", help="case-insensitive substring, e.g. Sleep or sleep2")
    parser.add_argument("--gripper-raw-range", nargs=2, type=int, metavar=("MIN", "MAX"))
    parser.add_argument("--gripper-reversed", action="store_true")
    parser.add_argument("--max-lag-ms", type=float, default=500.0)
    parser.add_argument("--out-json", type=Path)
    parser.add_argument("--out-csv", type=Path)
    args = parser.parse_args()
    if args.frequency is not None and (not math.isfinite(args.frequency) or args.frequency <= 0):
        parser.error("--frequency must be positive and finite")
    if not math.isfinite(args.max_lag_ms) or args.max_lag_ms < 0:
        parser.error("--max-lag-ms must be nonnegative and finite")
    bounds = tuple(args.gripper_raw_range) if args.gripper_raw_range else None
    if bounds and not (0 <= bounds[0] < bounds[1] <= 4095):
        parser.error("--gripper-raw-range must be within 0..4095 and MIN < MAX")
    all_results = []
    for arg in args.trace:
        label, separator, filename = arg.partition("=")
        if not separator:
            filename = label
            label = Path(filename).stem
        all_results.extend(analyze(Path(filename).expanduser(), label, args.frequency,
                                   bounds, args.gripper_reversed, args.segment,
                                   args.max_lag_ms/1000.0))
    flattened = []
    for result in all_results:
        print(f"\n{result['trace']} / {result['segment']} ({result['mode']}, {result['frequency_hz']:g} Hz)")
        print("joint                  zero_max zero_frac impuls25 rate/s hunt peak_err rms_err  lag_ms subtick_ms")
        for joint in result["joints"]:
            row = {key: value for key, value in result.items() if key != "joints"}
            row.update(joint)
            flattened.append(row)
            def display(value: object) -> str:
                return "NA" if value is None else str(value)
            print(f"{joint['joint']:<22} {display(joint['max_zero_cycles']):>8} "
                  f"{display(joint['zero_cycles_fraction']):>9} "
                  f"{display(joint['tick_impulses_final25']):>8} "
                  f"{display(joint['tick_impulses_per_s_final25']):>6} "
                  f"{display(joint['hunting_reversals']):>4} "
                  f"{display(joint['following_peak']):>8} "
                  f"{display(joint['following_rms']):>7} "
                  f"{display(joint['lag_ms']):>7} "
                  f"{display(joint['subtick_tail_ms_estimate']):>10}")
            if joint["zero_runs"]:
                print(f"    zero-run lengths (cycles: occurrences): {joint['zero_runs']}")
            if joint["servo_speed_raw_min"] is not None:
                print(
                    f"    speed_raw={joint['servo_speed_raw_min']}..{joint['servo_speed_raw_max']} "
                    f"changes={joint['servo_speed_raw_changes']}; "
                    f"post-gap (>=3 cycles) max={display(joint['post_gap_speed_raw_max'])} "
                    f"n={joint['post_gap_speed_raw_samples']}"
                )
    if args.out_json:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        args.out_json.write_text(json.dumps(all_results, indent=2)+"\n", encoding="utf-8")
    if args.out_csv and flattened:
        args.out_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.out_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=flattened[0].keys())
            writer.writeheader()
            for row in flattened:
                writer.writerow({k: json.dumps(v) if isinstance(v, (list,dict)) else v
                                 for k,v in row.items()})
    if not all_results:
        print("No matching trace segments.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
