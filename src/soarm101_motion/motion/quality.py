"""Analysis helpers for supervised motion-quality experiments."""

from __future__ import annotations

import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterable

from soarm101_motion.constants import ARM_JOINTS


def load_jsonl(path: str | Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for line_number, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), start=1):
        text = line.strip()
        if not text:
            continue
        value = json.loads(text)
        if not isinstance(value, dict):
            raise ValueError(f"{path}: line {line_number} is not a JSON object")
        rows.append(value)
    return rows


def recorded_teleop_settings(rows: Iterable[dict[str, Any]]) -> dict[str, float]:
    """Return the latest complete teleop safety settings from a GUI session log."""
    settings_rows = [dict(row) for row in rows if row.get("event") == "teleop_settings"]
    if not settings_rows:
        raise ValueError("GUI session log contains no teleop_settings event")

    latest = settings_rows[-1]
    required = {
        "max_joint_speed_rad_s": "max_joint_speed_rad_s",
        "max_joint_acceleration_rad_s2": "max_joint_acceleration_rad_s2",
        "max_command_step_rad": "max_command_step_rad",
        "following_error_limit_rad": "following_error_limit_rad",
    }
    result: dict[str, float] = {}
    for output_name, field_name in required.items():
        value = latest.get(field_name)
        if value is None:
            raise ValueError(f"recorded teleop setting {field_name} is missing")
        number = float(value)
        if not math.isfinite(number) or number <= 0:
            raise ValueError(f"recorded teleop setting {field_name} is invalid")
        result[output_name] = number
    return result


def teleop_frames(rows: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    return [dict(row) for row in rows if row.get("event") == "teleop_frame"]


def _finite(values: Iterable[float | int | None]) -> list[float]:
    result: list[float] = []
    for value in values:
        if value is None:
            continue
        number = float(value)
        if math.isfinite(number):
            result.append(number)
    return result


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = q * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def summarize_teleop_frames(frames: list[dict[str, Any]]) -> dict[str, Any]:
    if not frames:
        raise ValueError("teleop trace contains no teleop_frame events")

    intervals = _finite(frame.get("interval_ms") for frame in frames)
    processing = _finite(frame.get("processing_ms") for frame in frames)
    sample_age = _finite(frame.get("sample_age_ms") for frame in frames)

    raw_step_magnitudes: list[int] = []
    raw_step_by_joint: dict[str, list[int]] = {name: [] for name in ARM_JOINTS}
    zero_raw_transitions = 0
    raw_transitions = 0
    previous_raw: dict[str, int] | None = None

    following_by_joint: dict[str, list[float]] = {name: [] for name in ARM_JOINTS}
    for frame in frames:
        following = frame.get("following_error_rad")
        if isinstance(following, dict):
            for name in ARM_JOINTS:
                if name in following:
                    value = float(following[name])
                    if math.isfinite(value):
                        following_by_joint[name].append(abs(value))

        raw = frame.get("command_joints_raw")
        if isinstance(raw, dict) and all(name in raw for name in ARM_JOINTS):
            current_raw = {name: int(raw[name]) for name in ARM_JOINTS}
            if previous_raw is not None:
                deltas = {
                    name: abs(current_raw[name] - previous_raw[name])
                    for name in ARM_JOINTS
                }
                raw_transitions += 1
                if all(delta == 0 for delta in deltas.values()):
                    zero_raw_transitions += 1
                for name, delta in deltas.items():
                    raw_step_by_joint[name].append(delta)
                    raw_step_magnitudes.append(delta)
            previous_raw = current_raw

    frequencies = _finite(frame.get("frequency_hz") for frame in frames)
    nominal_frequency = statistics.median(frequencies) if frequencies else None
    if nominal_frequency is None and intervals:
        median_interval = statistics.median(intervals)
        nominal_frequency = 1000.0 / median_interval if median_interval > 0 else None

    summary: dict[str, Any] = {
        "frame_count": len(frames),
        "duration_s": (
            float(frames[-1]["monotonic_s"]) - float(frames[0]["monotonic_s"])
            if all("monotonic_s" in frame for frame in (frames[0], frames[-1]))
            else (
                sum(intervals) / 1000.0
                if intervals
                else None
            )
        ),
        "nominal_frequency_hz": nominal_frequency,
        "interval_ms": {
            "median": statistics.median(intervals) if intervals else None,
            "p95": _percentile(intervals, 0.95),
            "max": max(intervals) if intervals else None,
        },
        "processing_ms": {
            "median": statistics.median(processing) if processing else None,
            "p95": _percentile(processing, 0.95),
            "max": max(processing) if processing else None,
        },
        "sample_age_ms": {
            "median": statistics.median(sample_age) if sample_age else None,
            "p95": _percentile(sample_age, 0.95),
            "max": max(sample_age) if sample_age else None,
        },
        "raw_encoder_steps": {
            "transition_count": raw_transitions,
            "zero_complete_target_fraction": (
                zero_raw_transitions / raw_transitions if raw_transitions else None
            ),
            "all_joints_median_abs_ticks": (
                statistics.median(raw_step_magnitudes) if raw_step_magnitudes else None
            ),
            "all_joints_p95_abs_ticks": _percentile(
                [float(value) for value in raw_step_magnitudes], 0.95
            ),
            "by_joint": {
                name: {
                    "median_abs_ticks": (
                        statistics.median(values) if values else None
                    ),
                    "p95_abs_ticks": _percentile(
                        [float(value) for value in values], 0.95
                    ),
                    "zero_fraction": (
                        sum(value == 0 for value in values) / len(values)
                        if values
                        else None
                    ),
                }
                for name, values in raw_step_by_joint.items()
            },
        },
        "following_error_rad": {
            name: {
                "median_abs": (
                    statistics.median(values) if values else None
                ),
                "p95_abs": _percentile(values, 0.95),
                "max_abs": max(values) if values else None,
            }
            for name, values in following_by_joint.items()
        },
    }
    return summary


def write_frame_extract(frames: list[dict[str, Any]], path: str | Path) -> Path:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as handle:
        for frame in frames:
            handle.write(json.dumps(frame, ensure_ascii=False, default=str) + "\n")
    return output


def analyze_teleop_frame_continuity(
    frames: list[dict[str, Any]],
    *,
    frequency_hz: float,
) -> dict[str, Any]:
    """Check that captured teleop frames form one complete accepted stream.

    Live teleop assigns a monotonically increasing sample number for every accepted
    command and records the limiter's command velocity. Exact replay is only valid
    across adjacent recorded samples; a missing/restarted sample must never be
    silently interpreted as one larger 20 Hz command step.
    """
    if not frames:
        raise ValueError("teleop trace contains no frames")
    if not math.isfinite(frequency_hz) or frequency_hz <= 0:
        raise ValueError("frequency_hz must be positive and finite")

    gaps: list[dict[str, int]] = []
    velocity_mismatches: list[dict[str, Any]] = []
    dt = 1.0 / frequency_hz

    previous_frame: dict[str, Any] | None = None
    for replay_index, frame in enumerate(frames, start=1):
        sample_value = frame.get("sample")
        if not isinstance(sample_value, int):
            raise ValueError(f"teleop replay frame {replay_index} has no integer sample number")

        if previous_frame is not None:
            previous_sample = int(previous_frame["sample"])
            if sample_value != previous_sample + 1:
                gaps.append(
                    {
                        "replay_index": replay_index,
                        "previous_sample": previous_sample,
                        "sample": sample_value,
                    }
                )
            previous_command = previous_frame.get("command_joints_rad")
            command = frame.get("command_joints_rad")
            recorded_velocity = frame.get("command_velocity_rad_s")
            if (
                isinstance(previous_command, dict)
                and isinstance(command, dict)
                and isinstance(recorded_velocity, dict)
                and all(name in previous_command for name in ARM_JOINTS)
                and all(name in command for name in ARM_JOINTS)
                and all(name in recorded_velocity for name in ARM_JOINTS)
            ):
                worst_joint = None
                worst_error = 0.0
                for name in ARM_JOINTS:
                    derived = (
                        float(command[name]) - float(previous_command[name])
                    ) / dt
                    error = abs(derived - float(recorded_velocity[name]))
                    if error > worst_error:
                        worst_error = error
                        worst_joint = name
                if worst_error > 1e-6:
                    velocity_mismatches.append(
                        {
                            "replay_index": replay_index,
                            "sample": sample_value,
                            "joint": worst_joint,
                            "error_rad_s": worst_error,
                        }
                    )
        previous_frame = frame

    return {
        "frame_count": len(frames),
        "first_sample": int(frames[0]["sample"]),
        "last_sample": int(frames[-1]["sample"]),
        "gap_count": len(gaps),
        "gaps": gaps,
        "velocity_mismatch_count": len(velocity_mismatches),
        "velocity_mismatches": velocity_mismatches[:20],
    }


def repair_single_missing_teleop_frames(
    frames: list[dict[str, Any]],
    *,
    frequency_hz: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Reconstruct isolated missing accepted commands from logged next-frame velocity.

    For a one-sample gap n -> n+2, frame n+2 records the exact limiter velocity
    used from command n+1 to command n+2. Therefore command n+1 is recoverable as
    command[n+2] - velocity[n+2] * dt. Larger gaps/restarts are not inferred.
    """
    if not frames:
        return [], []
    if not math.isfinite(frequency_hz) or frequency_hz <= 0:
        raise ValueError("frequency_hz must be positive and finite")

    dt = 1.0 / frequency_hz
    repaired: list[dict[str, Any]] = [dict(frames[0])]
    repairs: list[dict[str, Any]] = []

    for frame in frames[1:]:
        previous = repaired[-1]
        previous_sample = previous.get("sample")
        sample = frame.get("sample")
        if (
            isinstance(previous_sample, int)
            and isinstance(sample, int)
            and sample == previous_sample + 2
        ):
            command = frame.get("command_joints_rad")
            velocity = frame.get("command_velocity_rad_s")
            if (
                isinstance(command, dict)
                and isinstance(velocity, dict)
                and all(name in command for name in ARM_JOINTS)
                and all(name in velocity for name in ARM_JOINTS)
            ):
                reconstructed_command = {
                    name: float(command[name]) - float(velocity[name]) * dt
                    for name in ARM_JOINTS
                }
                reconstructed = dict(frame)
                reconstructed["sample"] = previous_sample + 1
                if frame.get("leader_timestamp") is not None:
                    reconstructed["leader_timestamp"] = (
                        float(frame["leader_timestamp"]) - dt
                    )
                if frame.get("monotonic_s") is not None:
                    reconstructed["monotonic_s"] = float(frame["monotonic_s"]) - dt
                reconstructed["interval_ms"] = dt * 1000.0
                reconstructed["command_joints_rad"] = reconstructed_command
                reconstructed["reconstructed_for_replay"] = True
                reconstructed["reconstruction_source_sample"] = sample
                reconstructed["command_velocity_rad_s"] = {
                    name: (
                        reconstructed_command[name]
                        - float(previous["command_joints_rad"][name])
                    )
                    / dt
                    for name in ARM_JOINTS
                }
                repaired.append(reconstructed)
                repairs.append(
                    {
                        "missing_sample": previous_sample + 1,
                        "source_sample": sample,
                    }
                )
        repaired.append(dict(frame))

    return repaired, repairs


def contiguous_teleop_segments(
    frames: list[dict[str, Any]],
) -> list[list[dict[str, Any]]]:
    """Split captured frames at teleop sample gaps/restarts."""
    if not frames:
        return []
    segments: list[list[dict[str, Any]]] = [[frames[0]]]
    for frame in frames[1:]:
        previous = segments[-1][-1]
        previous_sample = previous.get("sample")
        sample = frame.get("sample")
        if (
            isinstance(previous_sample, int)
            and isinstance(sample, int)
            and sample == previous_sample + 1
        ):
            segments[-1].append(frame)
        else:
            segments.append([frame])
    return segments


def preflight_stream_commands(
    commands: list[dict[str, float]],
    *,
    start: dict[str, float],
    frequency_hz: float,
    max_command_step_radians: float,
    max_joint_speed: float,
    max_joint_acceleration: float,
    limits: dict[str, tuple[float, float]],
) -> dict[str, float]:
    if not math.isfinite(frequency_hz) or frequency_hz <= 0:
        raise ValueError("frequency_hz must be positive and finite")

    dt = 1.0 / frequency_hz
    previous = dict(start)
    previous_velocity: dict[str, float] | None = None
    max_step = 0.0
    max_speed = 0.0
    max_acceleration = 0.0

    for index, command in enumerate(commands, start=1):
        if set(command) != set(ARM_JOINTS):
            raise ValueError(
                f"replay sample {index} does not contain exactly the five arm joints"
            )
        for name in ARM_JOINTS:
            value = float(command[name])
            if not math.isfinite(value):
                raise ValueError(f"replay sample {index} has non-finite {name}")
            lower, upper = limits[name]
            if value < lower or value > upper:
                raise ValueError(
                    f"replay sample {index} {name}={value:.6f} is outside "
                    f"{lower:.6f}..{upper:.6f} rad"
                )

        step = {name: command[name] - previous[name] for name in ARM_JOINTS}
        sample_step = max(abs(value) for value in step.values())
        max_step = max(max_step, sample_step)
        if sample_step > max_command_step_radians * 1.001:
            raise ValueError(
                f"replay sample {index} step {math.degrees(sample_step):.2f} deg exceeds "
                f"{math.degrees(max_command_step_radians):.2f} deg"
            )

        velocity = {name: step[name] / dt for name in ARM_JOINTS}
        sample_speed = max(abs(value) for value in velocity.values())
        max_speed = max(max_speed, sample_speed)
        if sample_speed > max_joint_speed * 1.001:
            raise ValueError(
                f"replay sample {index} speed {math.degrees(sample_speed):.2f} deg/s exceeds "
                f"{math.degrees(max_joint_speed):.2f} deg/s"
            )

        if previous_velocity is not None:
            acceleration = {
                name: (velocity[name] - previous_velocity[name]) / dt
                for name in ARM_JOINTS
            }
            sample_acceleration = max(abs(value) for value in acceleration.values())
            max_acceleration = max(max_acceleration, sample_acceleration)
            if sample_acceleration > max_joint_acceleration * 1.001:
                raise ValueError(
                    f"replay sample {index} acceleration "
                    f"{math.degrees(sample_acceleration):.2f} deg/s^2 exceeds "
                    f"{math.degrees(max_joint_acceleration):.2f} deg/s^2"
                )

        previous = dict(command)
        previous_velocity = velocity

    return {
        "max_step_deg": math.degrees(max_step),
        "max_speed_deg_s": math.degrees(max_speed),
        "max_acceleration_deg_s2": math.degrees(max_acceleration),
    }


def command_sequence(frames: list[dict[str, Any]]) -> list[dict[str, float]]:
    sequence: list[dict[str, float]] = []
    for frame in frames:
        command = frame.get("command_joints_rad")
        if not isinstance(command, dict) or not all(name in command for name in ARM_JOINTS):
            raise ValueError("teleop frame is missing a complete command_joints_rad target")
        sequence.append({name: float(command[name]) for name in ARM_JOINTS})
    return sequence
