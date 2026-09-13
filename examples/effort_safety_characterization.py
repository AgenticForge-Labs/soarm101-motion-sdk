"""Collect SO-ARM101 effort traces and estimate software safety-stop timing.

Hardware only. The script records STS3215 current/load feedback, arm joint
positions, gripper position, threshold state, and interlock state while running
one small test motion. Results are written as CSV plus a JSON summary.

This is characterization tooling, not an emergency-stop certification test.
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
    lo = math.floor(position)
    hi = math.ceil(position)
    if lo == hi:
        return ordered[lo]
    weight = position - lo
    return ordered[lo] * (1.0 - weight) + ordered[hi] * weight


def _parse_motor_limits(values: Sequence[str]) -> dict[str, int]:
    result: dict[str, int] = {}
    for item in values:
        try:
            motor, raw_value = item.split("=", 1)
            value = int(raw_value)
        except ValueError as exc:
            raise ValueError(f"expected MOTOR=VALUE, got {item!r}") from exc
        if motor not in ALL_MOTORS:
            raise ValueError(f"unknown motor {motor!r}; choose from {', '.join(ALL_MOTORS)}")
        if value <= 0:
            raise ValueError("motor effort limits must be positive")
        result[motor] = value
    return result


def _current_limit(config: SOARM101Config, motor: str) -> int | None:
    return config.motor_current_trip_raw.get(motor, config.effort_current_trip_raw)


def _load_limit(config: SOARM101Config, motor: str) -> int | None:
    return config.motor_load_trip_raw.get(motor, config.effort_load_trip_raw)


def _exceeded(config: SOARM101Config, motor: str, effort: Mapping[str, int]) -> bool:
    current = abs(int(effort["current_raw"]))
    load = abs(int(effort["load_raw"]))
    current_limit = _current_limit(config, motor)
    load_limit = _load_limit(config, motor)
    return bool(
        (current_limit is not None and current >= current_limit)
        or (load_limit is not None and load >= load_limit)
    )


def _default_output(condition: str, mode: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    safe = "".join(char if char.isalnum() or char in "-_" else "_" for char in condition)
    return Path("effort-data") / f"{stamp}-{mode}-{safe}.csv"


def _build_config(args: argparse.Namespace) -> SOARM101Config:
    kwargs: dict[str, Any] = {
        "port": args.port,
        "robot_id": args.robot_id,
        "effort_safety_enabled": not args.disable_interlock,
        "effort_trip_consecutive_samples": args.trip_samples,
        "trajectory_feedback_interval_s": args.feedback_interval,
        "motor_current_trip_raw": _parse_motor_limits(args.motor_current_trip),
        "motor_load_trip_raw": _parse_motor_limits(args.motor_load_trip),
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


def _start_motion(arm: SOArmAPI, args: argparse.Namespace) -> Any | None:
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
    return arm.set_gripper_position(
        args.gripper_target,
        speed=args.gripper_speed,
        acceleration=args.gripper_acceleration,
        wait=False,
    )


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
    args: argparse.Namespace,
    config: SOARM101Config,
    output: Path,
    rows: Sequence[Mapping[str, Any]],
    samples: Sequence[Mapping[str, float]],
    first_threshold_s: float | None,
    trip_s: float | None,
    stop_s: float | None,
    motion_error: str | None,
) -> dict[str, Any]:
    elapsed = [float(sample["elapsed_s"]) for sample in samples]
    effective_rate = None
    if len(elapsed) > 1 and elapsed[-1] > elapsed[0]:
        effective_rate = (len(elapsed) - 1) / (elapsed[-1] - elapsed[0])

    read_durations = [float(sample["read_duration_s"]) for sample in samples]
    per_motor: dict[str, Any] = {}
    for motor in ALL_MOTORS:
        motor_rows = [row for row in rows if row["motor"] == motor]
        currents = [abs(int(row["current_raw"])) for row in motor_rows]
        loads = [abs(int(row["load_raw"])) for row in motor_rows]
        per_motor[motor] = {
            "max_abs_current_raw": max(currents) if currents else None,
            "median_abs_current_raw": statistics.median(currents) if currents else None,
            "p95_abs_current_raw": _percentile(currents, 0.95),
            "max_abs_load_raw": max(loads) if loads else None,
            "median_abs_load_raw": statistics.median(loads) if loads else None,
            "p95_abs_load_raw": _percentile(loads, 0.95),
            "current_trip_raw": _current_limit(config, motor),
            "load_trip_raw": _load_limit(config, motor),
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
        "mean_sample_read_duration_s": (
            statistics.mean(read_durations) if read_durations else None
        ),
        "max_sample_read_duration_s": max(read_durations) if read_durations else None,
        "sample_count": len(samples),
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
            "External serial polling consumes time and can lag the controller. Use repeated "
            "trials for tuning; these values are not certified emergency-stop reaction times."
        ),
    }


def run(args: argparse.Namespace) -> int:
    if args.sample_hz <= 0 or args.duration <= 0:
        raise ValueError("sample-hz and duration must be positive")
    if args.post_event_s < 0 or args.stop_samples < 1:
        raise ValueError("post-event-s must be non-negative and stop-samples at least 1")

    config = _build_config(args)
    output = args.output or _default_output(args.condition, args.mode)
    summary_path = output.with_suffix(".summary.json")

    print("SO-ARM101 effort characterization")
    print(f"  mode={args.mode} condition={args.condition}")
    print(f"  output={output}")
    print(f"  interlock={'ON' if config.effort_safety_enabled else 'OFF'}")
    print(
        "  WARNING: software hold only. Keep people outside the robot workspace and use "
        "only compliant test obstructions."
    )
    if not args.yes:
        input("Clear the workspace and keep power cutoff accessible. Press ENTER to continue: ")

    rows: list[dict[str, Any]] = []
    samples: list[dict[str, float]] = []
    first_threshold_s: float | None = None
    trip_s: float | None = None
    stop_s: float | None = None
    motion_error: str | None = None
    settled_count = 0

    with SOArmAPI(config) as arm:
        arm.motion_enable(True)

        if args.mode == "gripper":
            arm.set_gripper_position(
                args.gripper_start,
                speed=args.gripper_speed,
                acceleration=args.gripper_acceleration,
                wait=True,
            )

        if args.prompt_before_motion and args.mode != "observe" and not args.yes:
            input(
                "Arrange the payload/compliant obstruction with hands clear, then press ENTER "
                "to start the measured motion: "
            )

        handle = _start_motion(arm, args)
        started = time.perf_counter()
        period = 1.0 / args.sample_hz
        next_sample = started
        finish_after: float | None = None
        forced_stop = False
        previous_joints: list[float] | None = None
        previous_gripper: float | None = None
        previous_elapsed: float | None = None
        sample_index = 0

        while True:
            now = time.perf_counter()
            elapsed_now = now - started
            if handle is None and elapsed_now >= args.duration:
                break
            if handle is not None and elapsed_now >= args.duration and not forced_stop:
                print("Maximum trial duration reached; issuing software hold.")
                arm.stop()
                forced_stop = True
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

            any_threshold = any(_exceeded(config, motor, efforts[motor]) for motor in ALL_MOTORS)
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
                limit = (
                    args.gripper_stop_speed if args.mode == "gripper" else args.stop_velocity_rad_s
                )
                if velocity <= limit:
                    settled_count += 1
                    if settled_count >= args.stop_samples:
                        stop_s = elapsed
                        print(f"Motion appears settled at t={stop_s:.3f}s")
                else:
                    settled_count = 0

            phase = "post-trip" if trip_s is not None else "active"
            if trip_s is None and handle is not None and handle.done:
                phase = "post-command"

            wall_time = datetime.now(timezone.utc).isoformat()
            joint_columns = {
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
                        "current_limit_raw": _current_limit(config, motor),
                        "load_limit_raw": _load_limit(config, motor),
                        "threshold_exceeded": _exceeded(config, motor, effort),
                        **joint_columns,
                        "gripper_position": f"{gripper:.6f}",
                        "max_joint_speed_rad_s": f"{max_joint_speed:.6f}",
                        "gripper_speed_norm_s": f"{gripper_speed:.6f}",
                        "sample_read_duration_s": f"{read_duration:.6f}",
                    }
                )

            samples.append(
                {
                    "elapsed_s": elapsed,
                    "read_duration_s": read_duration,
                }
            )
            previous_joints = joints
            previous_gripper = gripper
            previous_elapsed = elapsed
            sample_index += 1
            next_sample += period
            if next_sample < time.perf_counter():
                next_sample = time.perf_counter()

        if handle is not None and handle.done:
            try:
                handle.wait(0)
            except Exception as exc:  # expected when the safety interlock aborts motion
                motion_error = f"{type(exc).__name__}: {exc}"

        if not args.yes:
            if arm.effort_trip_message:
                print("Interlock remains latched; the arm is still holding its measured position.")
            input("Support the arm before torque release. Press ENTER to disconnect and relax: ")

    _write_csv(output, rows)
    summary = _summarize(
        args,
        config,
        output,
        rows,
        samples,
        first_threshold_s,
        trip_s,
        stop_s,
        motion_error,
    )
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {output}")
    print(f"Wrote {summary_path}")
    if first_threshold_s is not None:
        print(f"First sampled threshold crossing: {first_threshold_s:.3f}s")
    if trip_s is not None:
        print(f"Interlock latch observed: {trip_s:.3f}s")
    if trip_s is not None and stop_s is not None:
        print(f"Estimated trip-to-settle latency: {(stop_s - trip_s) * 1000.0:.1f} ms")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Feetech serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--robot-id", default="so101")
    parser.add_argument("--mode", choices=("observe", "joint", "gripper"), default="observe")
    parser.add_argument("--condition", default="unloaded", help="Free-text trial label")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--post-event-s", type=float, default=1.0)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--feedback-interval", type=float, default=0.10)

    parser.add_argument("--disable-interlock", action="store_true")
    parser.add_argument("--current-trip", type=int, help="Override global raw current trip")
    parser.add_argument("--load-trip", type=int, help="Override global raw load trip")
    parser.add_argument("--trip-samples", type=int, default=2)
    parser.add_argument(
        "--motor-current-trip",
        action="append",
        default=[],
        metavar="MOTOR=VALUE",
        help="Repeatable per-motor current threshold override",
    )
    parser.add_argument(
        "--motor-load-trip",
        action="append",
        default=[],
        metavar="MOTOR=VALUE",
        help="Repeatable per-motor load threshold override",
    )

    parser.add_argument("--joint", choices=ARM_JOINTS, default="elbow_flex")
    parser.add_argument("--delta-deg", type=float, default=5.0)
    parser.add_argument("--joint-speed-deg-s", type=float, default=5.0)
    parser.add_argument("--joint-accel-deg-s2", type=float, default=20.0)

    parser.add_argument("--gripper-start", type=float, default=1.0)
    parser.add_argument("--gripper-target", type=float, default=0.0)
    parser.add_argument("--gripper-speed", type=int, default=80)
    parser.add_argument("--gripper-acceleration", type=int, default=10)

    parser.add_argument("--stop-velocity-rad-s", type=float, default=0.03)
    parser.add_argument("--gripper-stop-speed", type=float, default=0.02)
    parser.add_argument("--stop-samples", type=int, default=2)
    parser.add_argument("--prompt-before-motion", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Skip interactive safety prompts")
    return parser


def main() -> int:
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
