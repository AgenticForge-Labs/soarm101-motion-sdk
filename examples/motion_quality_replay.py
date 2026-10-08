"""Safely replay a captured teleop reference after the main motion-quality study."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import statistics
import subprocess
import time
from pathlib import Path
from typing import Any

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.constants import ARM_JOINTS, DEFAULT_TELEOP_STREAM_FREQUENCY_HZ
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.motion.quality import (
    analyze_teleop_frame_continuity,
    command_sequence,
    load_jsonl,
    preflight_stream_commands,
    recorded_teleop_settings,
    repair_single_missing_teleop_frames,
    teleop_frames,
)
from soarm101_motion.workstation import WorkstationProfileStore


def _git_sha() -> str | None:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception:
        return None
    value = completed.stdout.strip()
    return value or None


def _latest_study() -> Path:
    root = Path.home() / "soarm-motion-tests"
    candidates = sorted(
        (
            path
            for path in root.glob("motion-quality-study-*")
            if path.is_dir() and (path / "teleop-reference-frames.jsonl").exists()
        ),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not candidates:
        raise RuntimeError(
            "no completed motion-quality-study-* folder with "
            "teleop-reference-frames.jsonl was found"
        )
    return candidates[0]


def _recorded_timing(frames: list[dict[str, Any]]) -> tuple[float, list[float], str]:
    frequencies = [
        float(frame["frequency_hz"])
        for frame in frames
        if frame.get("frequency_hz") is not None
        and math.isfinite(float(frame["frequency_hz"]))
        and float(frame["frequency_hz"]) > 0
    ]
    frequency = (
        statistics.median(frequencies)
        if frequencies
        else DEFAULT_TELEOP_STREAM_FREQUENCY_HZ
    )

    timestamps: list[float] = []
    try:
        timestamps = [float(frame["leader_timestamp"]) for frame in frames]
    except (KeyError, TypeError, ValueError):
        timestamps = []
    if len(timestamps) == len(frames) and timestamps:
        origin = timestamps[0]
        offsets = [max(0.0, value - origin) for value in timestamps]
        return frequency, offsets, "recorded_leader_timestamps"

    period = 1.0 / frequency
    return (
        frequency,
        [index * period for index in range(len(frames))],
        "fixed_nominal_frequency",
    )


def _max_delta(first: dict[str, float], second: dict[str, float]) -> float:
    return max(abs(first[name] - second[name]) for name in ARM_JOINTS)


def _persist_result(
    study: Path,
    *,
    result: dict[str, Any],
    summary_path: Path,
) -> tuple[Path, Path]:
    summary_path.write_text(
        json.dumps(result, indent=2, default=str) + "\n",
        encoding="utf-8",
    )

    experiment_summary_path = study / "experiment-summary.json"
    experiment_summary: dict[str, Any] = {}
    if experiment_summary_path.exists():
        try:
            loaded = json.loads(experiment_summary_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                experiment_summary = loaded
        except Exception:
            pass
    experiment_summary["post_study_teleop_replay"] = result
    experiment_summary_path.write_text(
        json.dumps(experiment_summary, indent=2, default=str) + "\n",
        encoding="utf-8",
    )

    archive = Path(
        shutil.make_archive(
            str(study),
            "gztar",
            root_dir=study.parent,
            base_dir=study.name,
        )
    )
    return experiment_summary_path, archive


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Safely pre-position to the first captured teleop command, replay the exact "
            "accepted arm-joint command sequence, and append evidence to the study folder."
        )
    )
    parser.add_argument(
        "experiment_dir",
        nargs="?",
        type=Path,
        help="motion-quality-study-* directory; defaults to the latest completed study",
    )
    parser.add_argument("--preposition-speed-deg-s", type=float, default=8.0)
    parser.add_argument("--preposition-acceleration-deg-s2", type=float, default=25.0)
    return parser


def main() -> int:
    args = _parser().parse_args()
    study = (args.experiment_dir or _latest_study()).expanduser().resolve()
    frames_path = study / "teleop-reference-frames.jsonl"
    if not frames_path.exists():
        raise RuntimeError(f"missing teleop reference: {frames_path}")

    frames = teleop_frames(load_jsonl(frames_path))
    if len(frames) < 5:
        raise RuntimeError("teleop reference has fewer than five frames")

    session_path = study / "teleop-gui-session.jsonl"
    if not session_path.exists():
        raise RuntimeError(f"missing copied GUI session log: {session_path}")
    session_rows = load_jsonl(session_path)
    recorded_settings = recorded_teleop_settings(session_rows)
    frequency, _, _ = _recorded_timing(frames)
    continuity_before = analyze_teleop_frame_continuity(
        frames,
        frequency_hz=frequency,
    )
    repaired_frames, repairs = repair_single_missing_teleop_frames(
        frames,
        frequency_hz=frequency,
    )
    continuity_after = analyze_teleop_frame_continuity(
        repaired_frames,
        frequency_hz=frequency,
    )
    if continuity_after["gap_count"]:
        first_gap = continuity_after["gaps"][0]
        raise RuntimeError(
            "teleop capture is not one contiguous accepted-command stream after "
            "isolated-gap repair: replay index "
            f"{first_gap['replay_index']} jumps from original sample "
            f"{first_gap['previous_sample']} to {first_gap['sample']}"
        )
    frames = repaired_frames
    commands = command_sequence(frames)
    frequency, offsets, timing_source = _recorded_timing(frames)

    profile = WorkstationProfileStore().load()
    port = profile.follower.port
    robot_id = profile.follower.robot_id or "so101"
    if not port:
        raise RuntimeError("no follower port is configured")

    cfg = SOARM101Config(
        port=port,
        robot_id=robot_id,
        configure_motors_on_connect=False,
        disable_torque_on_disconnect=False,
        teleop_max_joint_speed=float(recorded_settings["max_joint_speed_rad_s"]),
        teleop_max_joint_acceleration=float(
            recorded_settings["max_joint_acceleration_rad_s2"]
        ),
        max_command_step_radians=float(recorded_settings["max_command_step_rad"]),
        following_error_limit_rad=float(
            recorded_settings["following_error_limit_rad"]
        ),
    )
    preposition_speed = math.radians(args.preposition_speed_deg_s)
    preposition_acceleration = math.radians(args.preposition_acceleration_deg_s2)
    if preposition_speed <= 0 or preposition_acceleration <= 0:
        raise ValueError("preposition speed and acceleration must be positive")

    trace_path = study / "teleop-command-replay-poststudy.jsonl"
    summary_path = study / "teleop-command-replay-poststudy.summary.json"

    print("\n" + "=" * 72)
    print("POST-STUDY EXACT TELEOP REPLAY")
    print("=" * 72)
    print(f"Study: {study}")
    print(f"Frames: {len(frames)}")
    print(f"Recorded cadence: {frequency:.2f} Hz")
    print(
        "Recorded teleop limits: "
        f"{math.degrees(cfg.stream_joint_speed_limit):.2f} deg/s, "
        f"{math.degrees(cfg.stream_joint_acceleration_limit):.2f} deg/s^2, "
        f"step {math.degrees(cfg.max_command_step_radians):.2f} deg"
    )
    if continuity_before["gap_count"]:
        print(
            f"Captured stream gaps before repair: {continuity_before['gap_count']} "
            f"(isolated frames reconstructed: {len(repairs)})"
        )
    else:
        print("Captured teleop sample numbers are contiguous.")
    print("\nThe follower will first move under the normal guarded joint-motion primitive")
    print("to the FIRST recorded teleop arm pose. That pre-positioning move keeps the")
    print("normal joint/workspace/fault/effort/following-error checks active.")
    print("Only after measured arrival is verified will exact teleop streaming begin.")
    print("The gripper is not replayed.")
    input("\nClear the workspace and press ENTER to continue, or Ctrl-C to stop: ")

    result: dict[str, Any] = {
        "status": "not_started",
        "study": str(study),
        "source_frames": str(frames_path),
        "trace": str(trace_path),
        "git_sha": _git_sha(),
        "robot_id": robot_id,
        "frequency_hz": frequency,
        "frame_count": len(frames),
        "timing_source": timing_source,
        "preposition_speed_deg_s": args.preposition_speed_deg_s,
        "preposition_acceleration_deg_s2": args.preposition_acceleration_deg_s2,
        "recorded_teleop_settings": {
            "max_joint_speed_rad_s": cfg.stream_joint_speed_limit,
            "max_joint_acceleration_rad_s2": cfg.stream_joint_acceleration_limit,
            "max_command_step_rad": cfg.max_command_step_radians,
            "following_error_limit_rad": cfg.following_error_limit_rad,
        },
        "continuity_before": continuity_before,
        "continuity_after": continuity_after,
        "reconstructed_frames": repairs,
    }

    try:
        with SOARM101(cfg) as arm:
            arm.enable()
            arm.hold()
            current = dict(arm.get_joint_positions().positions)
            first = commands[0]
            result["initial_to_recorded_start_deg"] = math.degrees(
                _max_delta(current, first)
            )

            with PassiveBackendTrace(
                arm,
                trace_path,
                metadata={
                    "kind": "poststudy_exact_teleop_command_replay",
                    "git_sha": _git_sha(),
                    "study": str(study),
                    "source_frames": str(frames_path),
                    "frequency_hz": frequency,
                    "frame_count": len(commands),
                    "timing_source": timing_source,
                },
            ) as trace:
                trace.mark(
                    "preposition_start",
                    initial_positions=current,
                    target_positions=first,
                    initial_delta_deg=result["initial_to_recorded_start_deg"],
                )
                print(
                    "Pre-positioning to first recorded arm pose "
                    f"({result['initial_to_recorded_start_deg']:.2f} deg "
                    "max joint delta)..."
                )
                preposition = arm.move_joints(
                    first,
                    speed=preposition_speed,
                    acceleration=preposition_acceleration,
                    workspace_check="full",
                )
                arm.hold()
                arrived = dict(preposition.final_positions)
                arrival_delta = _max_delta(arrived, first)
                result["preposition_arrival_delta_deg"] = math.degrees(arrival_delta)
                trace.mark(
                    "preposition_end",
                    completed=preposition.completed,
                    final_positions=arrived,
                    arrival_delta_deg=result["preposition_arrival_delta_deg"],
                )

                arrival_limit = min(
                    cfg.joint_position_tolerance_rad * 1.5,
                    cfg.stream_joint_speed_limit / frequency * 0.8,
                    cfg.max_command_step_radians * 0.8,
                )
                result["replay_start_tolerance_deg"] = math.degrees(arrival_limit)
                if arrival_delta > arrival_limit:
                    result["status"] = "blocked_preposition_not_close_enough"
                    raise RuntimeError(
                        "pre-positioning completed but the measured follower is still "
                        f"{math.degrees(arrival_delta):.2f} deg from the first recorded "
                        f"target; replay requires <= {math.degrees(arrival_limit):.2f} deg"
                    )

                limits = arm.get_joint_limits()
                preflight = preflight_stream_commands(
                    commands,
                    start=arrived,
                    frequency_hz=frequency,
                    max_command_step_radians=cfg.max_command_step_radians,
                    max_joint_speed=cfg.stream_joint_speed_limit,
                    max_joint_acceleration=cfg.stream_joint_acceleration_limit,
                    limits=limits,
                )
                result["preflight"] = preflight
                trace.mark("replay_preflight_passed", **preflight)

                print(
                    "Pre-position verified. Exact accepted teleop arm-command stream "
                    "will now replay automatically."
                )
                input("Press ENTER to start exact replay, or Ctrl-C to stop: ")

                arm.start_joint_stream(frequency_hz=frequency)
                started = time.perf_counter()
                trace.mark("replay_start")
                try:
                    for index, (command, offset_s) in enumerate(
                        zip(commands, offsets, strict=True),
                        start=1,
                    ):
                        deadline = started + offset_s
                        delay = deadline - time.perf_counter()
                        if delay > 0:
                            time.sleep(delay)
                        trace.mark("replay_sample", sample=index)
                        arm.stream_joint_target(command)
                finally:
                    arm.stop_joint_stream(hold=True)
                trace.mark("replay_end")
                arm.hold()
                result.update(trace.summary)

        result["status"] = "completed"
    except BaseException as exc:
        if result["status"] == "not_started":
            result["status"] = "failed"
        result["error"] = repr(exc)
        experiment_summary_path, archive = _persist_result(
            study,
            result=result,
            summary_path=summary_path,
        )
        print("\n" + "=" * 72)
        print("POST-STUDY REPLAY STOPPED SAFELY")
        print("=" * 72)
        print(f"Reason: {exc}")
        print(f"Replay summary: {summary_path}")
        print(f"Updated study summary: {experiment_summary_path}")
        print(f"Updated archive: {archive}")
        print("Follower remains torque-held if connection/hold was available.")
        return 2

    experiment_summary_path, archive = _persist_result(
        study,
        result=result,
        summary_path=summary_path,
    )

    print("\n" + "=" * 72)
    print("POST-STUDY REPLAY COMPLETE")
    print("=" * 72)
    print(f"Replay trace: {trace_path}")
    print(f"Replay summary: {summary_path}")
    print(f"Updated study summary: {experiment_summary_path}")
    print(f"Updated archive: {archive}")
    print("Follower remains torque-held.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
