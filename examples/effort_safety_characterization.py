"""Collect SO-ARM101 motor-effort traces and estimate safety-stop latency.

This hardware-only utility records current/load feedback together with arm and
stock-gripper position while running one deliberately small test motion. It is
intended to characterize normal effort, contact thresholds, and the software
interlock's response time before unattended operation.

Examples:

    python examples/effort_safety_characterization.py \
        --port /dev/ttyACM0 --mode joint --condition unloaded

    python examples/effort_safety_characterization.py \
        --port /dev/ttyACM0 --mode joint --condition soft-contact \
        --joint elbow_flex --delta-deg 5 --prompt-before-motion

    python examples/effort_safety_characterization.py \
        --port /dev/ttyACM0 --mode gripper --condition foam-contact \
        --gripper-speed 80 --prompt-before-motion

The output is a long-form CSV (one row per motor per sample) plus a JSON summary.
The latency estimate is limited by serial-bus read time and the requested sample
rate; it is useful for tuning and comparison, not certification.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from soarm101_motion import SOArmAPI, SOARM101Config
from soarm101_motion.constants import ALL_MOTORS, ARM_JOINTS


def _percentile(values: Sequence[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _resolved_current_limit(config: SOARM101Config, motor: str) -> int | None:
    return config.motor_current_trip_raw.get(motor, config.effort_current_trip_raw)


def _resolved_load_limit(config: SOARM101Config, motor: str) -> int | None:
    return config.motor_load_trip_raw.get(motor, config.effort_load_trip_raw)


def _threshold_exceeded(
    config: SOARM101Config,
    motor: str,
    effort: Mapping[str, int],
) -> bool:
    current_limit = _resolved_current_limit(config, motor)
    load_limit = _resolved_load_limit(config, motor)
    current = abs(int(effort["current_raw"]))
    load = abs(int(effort["load_raw"]))
    return bool(
        (current_limit is not None and current >= current_limit)
        or (load_limit is not None and load >= load_limit)
    )


def _default_output(condition: str, mode: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    safe_condition = "".join(char if char.isalnum() or char in "-_" else "_" for char in condition)
    return Path("effort-data") / f"{stamp}-{mode}-{safe_condition}.csv"


def _build_config(args: argparse.Namespace) -> SOARM101Config:
    kwargs: dict[str, Any] = {
        "port": args.port,
        "robot_id": args.robot_id,
        "effort_safety_enabled": not args.disable_interlock,
        "effort_trip_consecutive_samples": args.trip_samples,
        "trajectory_feedback_interval_s": args.feedback_interval,
    }
    if args.current_trip is not None:
        kwargs["effort_current_trip_raw"] = args.current_trip
    if args.load_trip is not None:
        kwargs["effort_load_trip_raw"] = args.load_trip
    return SOARM101Config(**kwargs)


def _sample(
    arm: SOArmAPI,
) -> tuple[list[float], float, dict[str, dict[str, int]], float]:
    started = time.perf_counter()
    joints = arm.get_servo_angle(is_radian=True)
    gripper = arm.get_gripper_position()
    efforts = {motor: arm.get_motor_effort(motor) for motor in ALL_MOTORS}
    return joints, gripper, efforts, time.perf_counter() - started


def _start_motion(
    arm: SOArmAPI,
    args: argparse.Namespace,
) -> Any | None:
    if args.mode == "observe":
        return None
    if args.mode == "joint":
        return arm.set_servo_angle(
            {args.joint: args.delta_deg},
            speed=args.joint_speed_deg_s,
            mvacc=args.joint_accel_deg_s2,
            wait=False,
            relative=True,
            is_radian=False,
        )
    if args.mode == "gripper":
        return arm.set_gripper_position(
            args.gripper_target,
            speed=args.gripper_speed,
            acceleration=args.gripper_acceleration,
            wait=False,
        )
    raise ValueError(args.mode)


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    fieldnames = [
        "wall_time_utc",
        "elapsed_s",
        "sample_index",
        "condition",
        "mode",
        "phase",
        "command_active",
        "trip_latched",
        "trip_message",
        "motor",
        "current_raw",
        "load_raw",
        "current_limit_raw",
        "load_limit_raw",
        "threshold_exceeded",
        *[f"{joint}_rad" for joint in ARM_JOINTS],
        "gripper_position",
        "max_joint_speed_rad_s",
        "gripper_speed_norm_s",
        "sample_read_duration_s",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _summarize(
    *,
    args: argparse.Namespace,
    config: SOARM101Config,
    output: Path,
    sample_metrics: Sequence[Mapping[str, Any]],
    rows: Sequence[Mapping[str, Any]],
    first_threshold_s: float | None,
    trip_s: float | None,
    stop_s: float | None,
    motion_error: str | None,
) -> dict[str, Any]:
    elapsed = [float(sample["elapsed_s"]) for sample in sample_metrics]
    effective_rate = None
    if len(elapsed) > 1 and elapsed[-1] > elapsed[0]:
        effective_rate = (len(elapsed) - 1) / (elapsed[-1] - elapsed[0])

    per_motor: dict[str, Any] = {}
    for motor in ALL_MOTORS:
        motor_rows = [row for row in rows if row["motor"] == motor]
        currents = [abs(int(row["current_raw"])) for row in motor_rows]
        loads = [abs(int(row["load_raw"])) for row in motor_rows]
        per_motor[motor] = {
            "max_abs_current_raw": max(currents) if currents else None,
            "p95_abs_current_raw": _percentile(currents, 0.95),
            "median_abs_current_raw": statistics.median(currents) if currents else None,
            "max_abs_load_raw": max(loads) if loads else None,
            "p95_abs_load_raw": _percentile(loads, 0.95),
            "median_abs_load_raw": statistics.median(loads) if loads else None,
            "current_trip_raw": _resolved_current_limit(config, motor),
            "load_trip_raw": _resolved_load_limit(config, motor),
        }

    crossing_to_trip_ms = None
    if first_threshold_s is not None and trip_s is not None:
        crossing_to_trip_ms = (trip_s - first_threshold_s) * 1000.0

    trip_to_stop_ms = None
    if trip_s is not None and stop_s is not None:
        trip_to_stop_ms = (stop_s - trip_s) * 1000.0

    return {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "csv": str(output),
        "condition": args.condition,
        "mode": args.mode,
        "robot_id": args.robot_id,
        "requested_sample_hz": args.sample_hz,
        "effective_sample_hz": effective_rate,
        "sample_count": len(sample_metrics),
        "trajectory_feedback_interval_s": config.trajectory_feedback_interval_s,
        "interlock_enabled": config.effort_safety_enabled,
        "effort_trip_consecutive_samples": config.effort_trip_consecutive_samples,
        "first_observed_threshold_crossing_s": first_threshold_s,
        "effort_trip_latched_s": trip_s,
        "estimated_motion_stopped_s": stop_s,
        "observed_crossing_to_trip_ms": crossing_to_trip_ms,
        "observed_trip_to_stop_ms": trip_to_stop_ms,
        "motion_error": motion_error,
        "per_motor": per_motor,
        "measurement_note": (
            "Threshold-crossing and stop times are external telemetry estimates. Serial reads "
            "consume time and can lag the controller; compare repeated trials rather than "
            "treating these values as certified reaction times."
        ),
    }


def run(args: argparse.Namespace) -> int:
    if args.sample_hz <= 0:
        raise ValueError("sample-hz must be positive")
    if args.duration <= 0:
        raise ValueError("duration must be positive")
    if args.post_event_s < 0:
        raise ValueError("post-event-s must be non-negative")
    if args.stop_samples < 1:
        raise ValueError("stop-samples must be at least 1")

    config = _build_config(args)
    output = args.output or _default_output(args.condition, args.mode)
    summary_path = output.with_suffix(".summary.json")

    print("SO-ARM101 effort characterization")
    print(f"  mode: {args.mode}")
    print(f"  condition: {args.condition}")
    print(f"  output: {output}")
    print(f"  interlock: {'ON' if config.effort_safety_enabled else 'OFF'}")
    print(
        "  WARNING: this is a software hold, not an emergency stop. Keep people out of the "
        "robot workspace and use only compliant test obstructions."
    )
    if not args.yes:
        input("Clear the workspace and make sure power can be removed immediately. Press ENTER: ")

    rows: list[dict[str, Any]] = []
    sample_metrics: list[dict[str, Any]] = []
    first_threshold_s: float | None = None
    trip_s: float | None = None
    stop_s: float | None = None
    motion_error: str | None = None
    settled_count = 0

    with SOArmAPI(config) as arm:
        arm.motion_enable(True)

        if args.mode == "gripper":
            print(f"Pre-positioning gripper at {args.gripper_start:.3f}")
            arm.set_gripper_position(
                args.gripper_start,
                speed=args.gripper_speed,
                acceleration=args.gripper_acceleration,
                wait=True,
            )

        if args.prompt_before_motion and args.mode != "observe" and not args.yes:
            input(
                "Arrange the payload/compliant test object with hands clear of the robot, "
                "then press ENTER to start the measured motion: "
            )

        handle = _start_motion(arm, args)
        started = time.perf_counter()
        nominal_period = 1.0 / args.sample_hz
        next_sample = started
        finish_after: float | None = None
        previous_joints: list[float] | None = None
        previous_gripper: float | None = None
        previous_elapsed: float | None = None
        sample_index = 0

        while True:
            now = time.perf_counter()
            if handle is None:
                if now - started >= args.duration:
                    break
            else:
                if now - started >= args.duration:
                    print("Test duration reached before command completed; issuing software stop.")
                    arm.stop(wait=True)
                    if finish_after is None:
                        finish_after = time.perf_counter() + args.post_event_s
                if finish_after is not None and now >= finish_after:
                    break

            if now < next_sample:
                time.sleep(next_sample - now)

            sample_started = time.perf_counter()
            elapsed = sample_started - started
            joints, gripper, efforts, read_duration = _sample(arm)
            trip_message = arm.effort_trip_message
            command_active = bool(handle is not None and not handle.done)

            if trip_message is not None and trip_s is None:
                trip_s = elapsed
                finish_after = time.perf_counter() + args.post_event_s
                print(f"Effort trip latched at t={trip_s:.3f}s: {trip_message}")
            elif handle is not None and handle.done and finish_after is None:
                finish_after = time.perf_counter() + args.post_event_s

            any_threshold = any(
                _threshold_exceeded(config, motor, efforts[motor]) for motor in ALL_MOTORS
            )
            if any_threshold and first_threshold_s is None:
                first_threshold_s = elapsed

            max_joint_speed = 0.0
            gripper_speed = 0.0
            if previous_elapsed is not None and elapsed > previous_elapsed:
                dt = elapsed - previous_elapsed
                if previous_joints is not None:
                    max_joint_speed = max(
                        abs(current - previous) / dt
                        for current, previous in zip(joints, previous_joints, strict=True)
                    )
                if previous_gripper is not None:
                    gripper_speed = abs(gripper - previous_gripper) / dt

            if trip_s is not None and stop_s is None and previous_elapsed is not None:
                velocity = gripper_speed if args.mode == "gripper" else max_joint_speed
                stop_limit = (
                    args.gripper_stop_speed if args.mode == "gripper" else args.stop_velocity_rad_s
                )
                if velocity <= stop_limit:
                    settled_count += 1
                    if settled_count >= args.stop_samples:
                        stop_s = elapsed
                        print(f"Motion appears settled at t={stop_s:.3f}s")
                else:
                    settled_count = 0

            phase = "active"
            if trip_s is not None:
                phase = "post-trip"
            elif handle is not None and handle.done:
                phase = "post-command"

            wall_time = datetime.now(timezone.utc).isoformat()
            joint_values = {
                f"{name}_rad": value for name, value in zip(ARM_JOINTS, joints, strict=True)
            }
            for motor in ALL_MOTORS:
                effort = efforts[motor]
                rows.append(
                    {
                        "wall_time_utc": wall_time,
                        "elapsed_s": f"{elapsed:.6f}",
                        "sample_index": sample_index,
                        "condition": args.condition,
                        "mode": args.mode,
                        "phase": phase,
                        "command_active": command_active,
                        "trip_latched": trip_message is not None,
                        "trip_message": trip_message or "",
                        "motor": motor,
                        "current_raw": int(effort["current_raw"]),
                        "load_raw": int(effort["load_raw"]),
                        "current_limit_raw": _resolved_current_limit(config, motor),
                        "load_limit_raw": _resolved_load_limit(config, motor),
                        "threshold_exceeded": _threshold_exceeded(config, motor, effort),
                        **joint_values,
                        "gripper_position": f"{gripper:.6f}",
                        "max_joint_speed_rad_s": f"{max_joint_speed:.6f}",
                        "gripper_speed_norm_s": f"{gripper_speed:.6f}",
                        "sample_read_duration_s": f"{read_duration:.6f}",
                    }
                )

            sample_metrics.append(
                {
                    "elapsed_s": elapsed,
                    "read_duration_s": read_duration,
                    "max_joint_speed_rad_s": max_joint_speed,
                    "gripper_speed_norm_s": gripper_speed,
                    "trip_latched": trip_message is not None,
                    "threshold_exceeded": any_threshold,
                }
            )
            previous_joints = joints
            previous_gripper = gripper
            previous_elapsed = elapsed
            sample_index += 1
            next_sample += nominal_period
            if next_sample < time.perf_counter():
                next_sample = time.perf_counter()

        if handle is not None and handle.done:
            try:
                handle.wait(0)
            except Exception as exc:  # expected when a safety interlock aborts motion
                motion_error = f"{type(exc).__name__}: {exc}"

        if not args.yes:
            if arm.effort_trip_message:
                print("The interlock remains latched; the arm is holding its measured position.")
            input(
                "Prepare to support the arm when torque is released. Press ENTER to disconnect "
                "and relax the motors: "
            )

    _write_csv(output, rows)
    summary = _summarize(
        args=args,
        config=config,
        output=output,
        sample_metrics=sample_metrics,
        rows=rows,
        first_threshold_s=first_threshold_s,
        trip_s=trip_s,
        stop_s=stop_s,
        motion_error=motion_error,
    )
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {output}")
    print(f"Wrote {summary_path}")
    if first_threshold_s is not None:
        print(f"First sampled threshold crossing: {first_threshold_s:.3f}s")
    if trip_s is not None:
        print(f"Interlock latch observed: {trip_s:.3f}s")
    if stop_s is not None and trip_s is not None:
        print(f"Estimated trip-to-settle latency: {(stop_s - trip_s) * 1000.0:.1f} ms")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Feetech serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--mode", choices=("observe", "joint", "gripper"), default="observe")
    parser.add_argument("--condition", default="unloaded", help="Free-text trial label")
    parser.add_argument("--output", type=Path, help="CSV path; JSON summary is written beside it")
    parser.add_argument("--duration", type=float, default=8.0, help="Maximum trial duration")
    parser.add_argument("--post-event-s", type=float, default=1.0)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument(
        "--feedback-interval",
        type=float,
        default=0.10,
        help="Controller effort/following-error feedback interval in seconds",
    )
    parser.add_argument("--disable-interlock", action="store_true")
    parser.add_argument("--current-trip", type=int, help="Override global raw current trip")
    parser.add_argument("--load-trip", type=int, help="Override global raw load trip")
    parser.add_argument("--trip-samples", type=int, default=2)

    parser.add_argument("--joint", choices=ARM_JOINTS, default="elbow_flex")
    parser.add_argument("--delta-deg", type=float, default=5.0)
    parser.add_argument("--joint-speed-deg-s", type=float, default=5.0)
    parser.add_argument("--joint-accel-deg-s2", type=float, default=20.0)

    parser.add_argument("--gripper-start", type=float, default=1.0)
    parser.add_argument("--gripper-target", type=float, default=0.0)
    parser.add_argument("--gripper-speed", type=int, default=80)
    parser.add_argument("--gripper-acceleration", type=int, default=10)

    parser.add_argument(
        "--stop-velocity-rad-s",
        type=float,
        default=0.03,
        help="Joint-motion settled threshold used only for latency estimation",
    )
    parser.add_argument(
        "--gripper-stop-speed",
        type=float,
        default=0.02,
        help="Normalized gripper speed threshold used only for latency estimation",
    )
    parser.add_argument("--stop-samples", type=int, default=2)
    parser.add_argument("--prompt-before-motion", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Skip interactive safety prompts")
    return parser


def main() -> int:
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
