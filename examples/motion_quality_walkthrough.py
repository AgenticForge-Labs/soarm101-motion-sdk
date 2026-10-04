"""Guided collection of comparable SO-ARM101 motion-quality evidence."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.constants import ARM_JOINTS, DEFAULT_TELEOP_STREAM_FREQUENCY_HZ
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.motion.quality import (
    command_sequence,
    load_jsonl,
    summarize_teleop_frames,
    teleop_frames,
    write_frame_extract,
)
from soarm101_motion.workstation import WorkstationProfileStore


GUI_LOG_DIR = Path.home() / ".local" / "state" / "soarm101" / "gui"


def _now_utc() -> datetime:
    return datetime.now(timezone.utc)


def _parse_utc(value: Any) -> datetime | None:
    if not isinstance(value, str):
        return None
    try:
        parsed = datetime.fromisoformat(value)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


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


def _default_experiment_dir() -> Path:
    stamp = _now_utc().strftime("%Y%m%dT%H%M%SZ")
    return Path.home() / "soarm-motion-tests" / f"motion-quality-study-{stamp}"


def _candidate_gui_logs(
    before: dict[Path, int],
    *,
    launched_at: datetime,
) -> list[Path]:
    if not GUI_LOG_DIR.exists():
        return []
    candidates: list[Path] = []
    launch_ts = launched_at.timestamp() - 2.0
    for path in GUI_LOG_DIR.glob("session-*.jsonl"):
        try:
            stat = path.stat()
        except OSError:
            continue
        if path not in before or before[path] != stat.st_mtime_ns or stat.st_mtime >= launch_ts:
            candidates.append(path)
    return sorted(candidates, key=lambda path: path.stat().st_mtime, reverse=True)


def _choose_session(
    candidates: list[Path],
    *,
    capture_start: datetime,
    capture_end: datetime,
) -> tuple[Path, list[dict[str, Any]]]:
    scored: list[tuple[int, Path, list[dict[str, Any]]]] = []
    for path in candidates:
        try:
            rows = load_jsonl(path)
        except Exception:
            continue
        frames: list[dict[str, Any]] = []
        for frame in teleop_frames(rows):
            timestamp = _parse_utc(frame.get("time_utc"))
            if timestamp is None or capture_start <= timestamp <= capture_end:
                frames.append(frame)
        scored.append((len(frames), path, frames))
    if not scored:
        raise RuntimeError("no readable GUI session log was created")
    count, path, frames = max(scored, key=lambda item: item[0])
    if count < 5:
        raise RuntimeError(
            "the new GUI session logs contain fewer than five teleop frames in the marked interval"
        )
    return path, frames


def _print_teleop_summary(summary: dict[str, Any]) -> None:
    print("\nTeleop reference captured:")
    print(f"  frames: {summary['frame_count']}")
    duration = summary.get("duration_s")
    if duration is not None:
        print(f"  duration: {duration:.2f} s")
    frequency = summary.get("nominal_frequency_hz")
    if frequency is not None:
        print(f"  nominal cadence: {frequency:.2f} Hz")
    raw = summary.get("raw_encoder_steps", {})
    median_ticks = raw.get("all_joints_median_abs_ticks")
    zero_fraction = raw.get("zero_complete_target_fraction")
    if median_ticks is not None:
        print(f"  median raw step across joints: {median_ticks:.2f} ticks")
    if zero_fraction is not None:
        print(f"  repeated complete raw targets: {100.0 * zero_fraction:.1f}%")


def _replay_teleop(
    frames: list[dict[str, Any]],
    *,
    output: Path,
    port: str,
    robot_id: str,
) -> dict[str, Any]:
    commands = command_sequence(frames)
    frequency_values = [
        float(frame["frequency_hz"])
        for frame in frames
        if frame.get("frequency_hz") is not None
        and math.isfinite(float(frame["frequency_hz"]))
        and float(frame["frequency_hz"]) > 0
    ]
    frequency = (
        statistics.median(frequency_values)
        if frequency_values
        else DEFAULT_TELEOP_STREAM_FREQUENCY_HZ
    )

    first_leader_timestamp = frames[0].get("leader_timestamp")
    recorded_offsets: list[float] = []
    if first_leader_timestamp is not None:
        try:
            origin = float(first_leader_timestamp)
            recorded_offsets = [
                max(0.0, float(frame["leader_timestamp"]) - origin)
                for frame in frames
            ]
        except (KeyError, TypeError, ValueError):
            recorded_offsets = []
    if len(recorded_offsets) != len(commands):
        period = 1.0 / frequency
        recorded_offsets = [index * period for index in range(len(commands))]

    cfg = SOARM101Config(
        port=port,
        robot_id=robot_id,
        configure_motors_on_connect=False,
        disable_torque_on_disconnect=False,
    )
    result: dict[str, Any] = {
        "status": "not_started",
        "frame_count": len(commands),
        "frequency_hz": frequency,
        "trace": str(output),
    }

    with SOARM101(cfg) as arm:
        arm.enable()
        arm.hold()
        current = dict(arm.get_joint_positions().positions)
        first = commands[0]
        start_delta = max(abs(first[name] - current[name]) for name in ARM_JOINTS)
        result["start_delta_deg"] = math.degrees(start_delta)
        if start_delta > cfg.max_command_step_radians:
            result["status"] = "skipped_start_too_far"
            result["max_stream_start_delta_deg"] = math.degrees(
                cfg.max_command_step_radians
            )
            arm.hold()
            return result

        with PassiveBackendTrace(
            arm,
            output,
            metadata={
                "kind": "exact_teleop_command_replay",
                "git_sha": _git_sha(),
                "frequency_hz": frequency,
                "frame_count": len(commands),
                "timing_source": (
                    "recorded_leader_timestamps"
                    if first_leader_timestamp is not None
                    else "fixed_nominal_frequency"
                ),
            },
        ) as trace:
            trace.mark("replay_start")
            arm.start_joint_stream(frequency_hz=frequency)
            started = time.perf_counter()
            try:
                for index, (command, offset_s) in enumerate(
                    zip(commands, recorded_offsets, strict=True)
                ):
                    deadline = started + offset_s
                    delay = deadline - time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)
                    trace.mark("replay_sample", sample=index + 1)
                    arm.stream_joint_target(command)
            finally:
                arm.stop_joint_stream(hold=True)
            trace.mark("replay_end")
            result.update(trace.summary)

        result["status"] = "completed"
        arm.hold()
    return result


def _run_program_trace(
    *,
    script: Path,
    output: Path,
    speed_deg_s: float,
    acceleration_deg_s2: float,
    command_frequency_hz: float,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(script),
        "--speed-deg-s",
        str(speed_deg_s),
        "--acceleration-deg-s2",
        str(acceleration_deg_s2),
        "--command-frequency-hz",
        str(command_frequency_hz),
        "--pause-s",
        "0.5",
        "--output",
        str(output),
    ]
    completed = subprocess.run(command, check=False)
    summary_path = output.with_suffix(".summary.json")
    summary: dict[str, Any] = {
        "status": "completed" if completed.returncode == 0 else "failed",
        "returncode": completed.returncode,
        "trace": str(output),
        "summary": str(summary_path),
        "command_frequency_hz": command_frequency_hz,
    }
    if summary_path.exists():
        try:
            summary["program_summary"] = json.loads(
                summary_path.read_text(encoding="utf-8")
            )
        except Exception:
            pass
    return summary


def _write_readme(
    path: Path,
    *,
    sha: str | None,
    teleop_log: Path,
    capture_start: datetime,
    capture_end: datetime,
) -> None:
    path.write_text(
        "\n".join(
            [
                "SO-ARM101 motion-quality study",
                "",
                f"Git SHA: {sha}",
                f"Teleop source log: {teleop_log}",
                f"Marked teleop interval: {capture_start.isoformat()} -> {capture_end.isoformat()}",
                "",
                "Conditions:",
                "A. Live GUI teleoperation reference, manually moved slowly:",
                "   Sleep -> Overhead -> Left -> Right -> Sleep",
                "B. Exact accepted teleop arm-joint command sequence replayed through guarded streaming.",
                "C. Programmed saved-pose route at 8 deg/s, 25 deg/s^2, 50 Hz host cadence.",
                "D. Same programmed saved-pose route at 8 deg/s, 25 deg/s^2, 20 Hz host cadence.",
                "",
                "The passive backend traces do not add hardware reads during motion.",
                "Use the traces to compare raw encoder step sizes, cadence, command-versus-feedback",
                "lag, low-speed behavior, and whether the programmed path differs from smooth teleop.",
                "",
            ]
        ),
        encoding="utf-8",
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Walk through a controlled teleop/program motion-quality study."
    )
    parser.add_argument("--experiment-dir", type=Path)
    parser.add_argument("--speed-deg-s", type=float, default=8.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    parser.add_argument(
        "--skip-exact-replay",
        action="store_true",
        help="collect teleop and programmed traces but do not replay the teleop command sequence",
    )
    return parser


def main() -> int:
    args = _parser().parse_args()
    root = Path(__file__).resolve().parents[1]
    program_script = root / "examples" / "motion_quality_trace.py"
    experiment_dir = (args.experiment_dir or _default_experiment_dir()).expanduser()
    experiment_dir.mkdir(parents=True, exist_ok=False)

    profile = WorkstationProfileStore().load()
    port = profile.follower.port
    robot_id = profile.follower.robot_id or "so101"
    if not port:
        raise RuntimeError("no follower port is configured")

    sha = _git_sha()
    print("\n" + "=" * 72)
    print("SO-ARM101 MOTION QUALITY STUDY")
    print("=" * 72)
    print(f"Commit: {sha}")
    print(f"Experiment folder: {experiment_dir}")
    print("\nThis collects comparable evidence without changing PID or safety limits.")
    print("Keep physical power accessible and keep the workspace clear.")
    print("Close any other process that has the follower or leader serial port open.")
    print("\nWe will collect:")
    print("  A. slow live teleoperation")
    if not args.skip_exact_replay:
        print("  B. replay of the exact accepted teleop arm-joint commands")
    print("  C. current programmed route at 50 Hz")
    print("  D. the same programmed route at 20 Hz")
    input("\nPress ENTER when the workspace is ready, or Ctrl-C to stop: ")

    GUI_LOG_DIR.mkdir(parents=True, exist_ok=True)
    before = {
        path: path.stat().st_mtime_ns
        for path in GUI_LOG_DIR.glob("session-*.jsonl")
        if path.exists()
    }

    print("\n" + "-" * 72)
    print("STEP A — LIVE TELEOP REFERENCE")
    print("-" * 72)
    print("The GUI is about to open.")
    print("1. Connect the follower and leader normally.")
    print("2. Open Teleoperation and link the arms.")
    print("3. Move the follower to the folded Sleep-like starting position.")
    print("4. Leave teleoperation RUNNING, then return to this terminal.")
    print("Do not close the GUI yet.")
    input("\nPress ENTER to launch the GUI: ")

    launched_at = _now_utc()
    gui = subprocess.Popen(["soarm101-gui"])
    try:
        input(
            "\nWhen teleoperation is running and the follower is at the starting "
            "Sleep position, press ENTER HERE to mark the beginning of the reference run: "
        )
        capture_start = _now_utc()
        print("\nNow move the leader SLOWLY through:")
        print("  SLEEP -> OVERHEAD -> LEFT -> RIGHT -> SLEEP")
        print("\nTry to move at roughly the same slow speed that feels smooth in teleop.")
        print("Do not intentionally wiggle the leader; use ordinary smooth hand motion.")
        print("When the follower reaches the FINAL Sleep position:")
        print("  1. click Stop Teleoperation in the GUI")
        print("  2. then press ENTER here")
        input("\nPress ENTER after Stop Teleoperation: ")
        capture_end = _now_utc()
        print("\nReference interval marked. Now close the GUI window normally.")
        gui.wait()
    finally:
        if gui.poll() is None:
            gui.terminate()
            try:
                gui.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gui.kill()

    candidates = _candidate_gui_logs(before, launched_at=launched_at)
    source_log, frames = _choose_session(
        candidates,
        capture_start=capture_start,
        capture_end=capture_end,
    )
    copied_log = experiment_dir / "teleop-gui-session.jsonl"
    shutil.copy2(source_log, copied_log)
    extracted = write_frame_extract(
        frames,
        experiment_dir / "teleop-reference-frames.jsonl",
    )
    teleop_summary = summarize_teleop_frames(frames)
    (experiment_dir / "teleop-reference-summary.json").write_text(
        json.dumps(teleop_summary, indent=2) + "\n",
        encoding="utf-8",
    )
    _print_teleop_summary(teleop_summary)
    print(f"  source session: {source_log}")
    print(f"  extracted reference: {extracted}")

    results: dict[str, Any] = {
        "git_sha": sha,
        "experiment_dir": str(experiment_dir),
        "teleop": {
            "source_log": str(source_log),
            "copied_log": str(copied_log),
            "extract": str(extracted),
            "capture_start_utc": capture_start.isoformat(),
            "capture_end_utc": capture_end.isoformat(),
            "summary": teleop_summary,
        },
    }

    if not args.skip_exact_replay:
        print("\n" + "-" * 72)
        print("STEP B — EXACT TELEOP COMMAND REPLAY")
        print("-" * 72)
        print("This replays the accepted ARM-JOINT targets from the marked teleop run.")
        print("It uses the guarded streaming primitive; it does not bypass rate,")
        print("acceleration, following-error, fault, effort, or joint-limit checks.")
        print("The gripper is intentionally not replayed so this isolates arm motion.")
        print("The follower should still be at the final Sleep position.")
        input("\nPress ENTER to run the replay, or Ctrl-C to stop: ")
        replay_result = _replay_teleop(
            frames,
            output=experiment_dir / "teleop-command-replay.jsonl",
            port=port,
            robot_id=robot_id,
        )
        results["teleop_command_replay"] = replay_result
        if replay_result["status"] == "completed":
            print("Exact teleop arm-command replay completed; follower is holding.")
        else:
            print(
                "Exact teleop replay was safely skipped: "
                f"{replay_result['status']} "
                f"(start delta {replay_result.get('start_delta_deg'):.2f} deg)."
            )

    print("\n" + "-" * 72)
    print("STEP C — PROGRAMMED ROUTE AT CURRENT 50 Hz CADENCE")
    print("-" * 72)
    print(
        "This now runs automatically with NO per-move ENTER prompts:\n"
        "  SLEEP -> OVERHEAD -> LEFT -> RIGHT -> SLEEP\n"
        f"  {args.speed_deg_s:g} deg/s, {args.acceleration_deg_s2:g} deg/s^2, 50 Hz"
    )
    input("\nPress ENTER once to start the complete automatic 50 Hz route: ")
    result_50 = _run_program_trace(
        script=program_script,
        output=experiment_dir / "program-50hz.jsonl",
        speed_deg_s=args.speed_deg_s,
        acceleration_deg_s2=args.acceleration_deg_s2,
        command_frequency_hz=50.0,
    )
    results["program_50hz"] = result_50

    print("\n" + "-" * 72)
    print("STEP D — SAME PROGRAMMED ROUTE AT TELEOP-LIKE 20 Hz CADENCE")
    print("-" * 72)
    print(
        "Same route, same speed and acceleration, only host cadence changes to 20 Hz.\n"
        "Again there are NO per-move prompts."
    )
    input("\nPress ENTER once to start the complete automatic 20 Hz route: ")
    result_20 = _run_program_trace(
        script=program_script,
        output=experiment_dir / "program-20hz.jsonl",
        speed_deg_s=args.speed_deg_s,
        acceleration_deg_s2=args.acceleration_deg_s2,
        command_frequency_hz=20.0,
    )
    results["program_20hz"] = result_20

    _write_readme(
        experiment_dir / "README.txt",
        sha=sha,
        teleop_log=source_log,
        capture_start=capture_start,
        capture_end=capture_end,
    )
    (experiment_dir / "experiment-summary.json").write_text(
        json.dumps(results, indent=2, default=str) + "\n",
        encoding="utf-8",
    )

    archive = shutil.make_archive(
        str(experiment_dir),
        "gztar",
        root_dir=experiment_dir.parent,
        base_dir=experiment_dir.name,
    )

    print("\n" + "=" * 72)
    print("DATA COLLECTION COMPLETE")
    print("=" * 72)
    print(f"Experiment folder: {experiment_dir}")
    print(f"Summary: {experiment_dir / 'experiment-summary.json'}")
    print(f"Upload this archive back to the chat: {archive}")
    print("\nThe follower should remain torque-held after the final route.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
