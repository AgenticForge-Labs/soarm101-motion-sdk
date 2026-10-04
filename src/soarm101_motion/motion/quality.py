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


def command_sequence(frames: list[dict[str, Any]]) -> list[dict[str, float]]:
    sequence: list[dict[str, float]] = []
    for frame in frames:
        command = frame.get("command_joints_rad")
        if not isinstance(command, dict) or not all(name in command for name in ARM_JOINTS):
            raise ValueError("teleop frame is missing a complete command_joints_rad target")
        sequence.append({name: float(command[name]) for name in ARM_JOINTS})
    return sequence
