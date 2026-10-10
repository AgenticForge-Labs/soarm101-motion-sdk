"""Compare slow programmed motion against teleoperation evidence.

This script runs the saved-pose route automatically without operator prompts and writes
passive JSONL evidence. The passive tracer records only backend calls the SDK already makes;
it does not add hardware polling during motion.

Suggested experiment:
1. Use the GUI Teleoperation tab to manually reproduce the route slowly. Detailed teleop
   logging is enabled by default and writes teleop_frame events into the GUI session JSONL.
2. Run this script at the same approximate speed.
3. Compare command cadence, command/feedback positions, following behavior, and timing.

The fixed route is:
Sleep -> agent_start_overhead -> agent_start_overhead_left ->
agent_start_overhead_right -> Sleep
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.poses import PoseLibrary
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


def _default_output() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path.home() / "soarm-motion-tests" / f"program-trace-{stamp}.jsonl"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the saved-pose motion-quality route with passive JSONL tracing."
    )
    parser.add_argument("--port")
    parser.add_argument("--robot-id")
    parser.add_argument("--speed-deg-s", type=float, default=8.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    parser.add_argument(
        "--command-frequency-hz",
        type=float,
        default=50.0,
        help="host planned-motion command cadence",
    )
    parser.add_argument(
        "--execution-mode",
        choices=("streamed", "final_target"),
        default="streamed",
        help="joint execution strategy: host-streamed trajectory or one synchronized final target",
    )
    parser.add_argument(
        "--pause-s",
        type=float,
        default=0.5,
        help="automatic hold time between route legs; no ENTER prompts are used",
    )
    parser.add_argument(
        "--joint-only",
        action="store_true",
        help="do not request gripper closure on Sleep legs",
    )
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if not math.isfinite(args.speed_deg_s) or args.speed_deg_s <= 0:
        raise ValueError("--speed-deg-s must be positive and finite")
    if not math.isfinite(args.acceleration_deg_s2) or args.acceleration_deg_s2 <= 0:
        raise ValueError("--acceleration-deg-s2 must be positive and finite")
    if not math.isfinite(args.command_frequency_hz) or args.command_frequency_hz <= 0:
        raise ValueError("--command-frequency-hz must be positive and finite")
    if not math.isfinite(args.pause_s) or args.pause_s < 0:
        raise ValueError("--pause-s must be finite and non-negative")

    profile = WorkstationProfileStore().load()
    port = args.port or profile.follower.port
    robot_id = args.robot_id or profile.follower.robot_id or "so101"
    if not port:
        raise RuntimeError("no follower port is configured")

    output = (args.output or _default_output()).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    summary_path = output.with_suffix(".summary.json")

    speed = math.radians(args.speed_deg_s)
    acceleration = math.radians(args.acceleration_deg_s2)

    cfg = SOARM101Config(
        port=port,
        robot_id=robot_id,
        configure_motors_on_connect=False,
        disable_torque_on_disconnect=False,
        command_frequency_hz=args.command_frequency_hz,
    )
    library = PoseLibrary(robot_id)
    pose_names = {
        "OVERHEAD": "agent_start_overhead",
        "LEFT": "agent_start_overhead_left",
        "RIGHT": "agent_start_overhead_right",
    }
    poses = {label: library.require(name) for label, name in pose_names.items()}

    print(f"Follower: {port}")
    print(f"Robot ID: {robot_id}")
    print(
        f"Program: Sleep -> Overhead -> Left -> Right -> Sleep "
        f"at {args.speed_deg_s:g} deg/s, {args.acceleration_deg_s2:g} deg/s^2, "
        f"{args.command_frequency_hz:g} Hz, mode={args.execution_mode}"
    )
    print(f"Trace: {output}")

    with SOARM101(cfg) as arm:
        arm.enable()
        arm.hold()

        for label, pose in poses.items():
            arm.require_artifact_calibration(
                {
                    "source_robot_id": pose.source_robot_id,
                    "source_calibration_id": pose.source_calibration_id,
                    "target_robot_id": pose.target_robot_id,
                    "target_calibration_id": pose.target_calibration_id,
                },
                artifact_label=f"{label} saved pose",
            )

        with PassiveBackendTrace(
            arm,
            output,
            metadata={
                "kind": "program_motion_quality",
                "git_sha": _git_sha(),
                "speed_deg_s": args.speed_deg_s,
                "acceleration_deg_s2": args.acceleration_deg_s2,
                "command_frequency_hz": cfg.command_frequency_hz,
                "execution_mode": args.execution_mode,
                "joint_only": bool(args.joint_only),
                "hardware_speed_raw": cfg.hardware_speed_raw,
                "hardware_acceleration_raw": cfg.hardware_acceleration_raw,
                "route": ["SLEEP", "OVERHEAD", "LEFT", "RIGHT", "SLEEP"],
                "pose_names": pose_names,
            },
        ) as trace:
            def run_sleep() -> None:
                trace.mark("leg_start", destination="SLEEP")
                result = arm.move_sleep(
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                    close_gripper=not args.joint_only,
                )
                arm.hold()
                trace.mark(
                    "leg_end",
                    destination="SLEEP",
                    completed=result.completed,
                    final_positions=dict(result.final_positions),
                )

            def run_pose(label: str) -> None:
                pose = poses[label]
                trace.mark("leg_start", destination=label, pose_name=pose_names[label])
                result = arm.move_joints_from_saved_pose(
                    pose.joints,
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                )
                arm.hold()
                trace.mark(
                    "leg_end",
                    destination=label,
                    completed=result.completed,
                    final_positions=dict(result.final_positions),
                )

            route = [
                ("SLEEP", run_sleep),
                ("OVERHEAD", lambda: run_pose("OVERHEAD")),
                ("LEFT", lambda: run_pose("LEFT")),
                ("RIGHT", lambda: run_pose("RIGHT")),
                ("SLEEP", run_sleep),
            ]

            for index, (label, operation) in enumerate(route, start=1):
                print(f"[{index}/{len(route)}] {label}")
                operation()
                if args.pause_s and index < len(route):
                    trace.mark("inter_leg_hold", duration_s=args.pause_s)
                    time.sleep(args.pause_s)

            arm.hold()
            summary = dict(trace.summary)

    summary.update(
        {
            "trace": str(output),
            "git_sha": _git_sha(),
            "robot_id": robot_id,
            "speed_deg_s": args.speed_deg_s,
            "acceleration_deg_s2": args.acceleration_deg_s2,
            "command_frequency_hz": cfg.command_frequency_hz,
            "execution_mode": args.execution_mode,
            "joint_only": bool(args.joint_only),
            "hardware_speed_raw": cfg.hardware_speed_raw,
            "hardware_acceleration_raw": cfg.hardware_acceleration_raw,
            "route": ["SLEEP", "OVERHEAD", "LEFT", "RIGHT", "SLEEP"],
        }
    )
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print("Finished; follower remains torque-held.")
    print(f"Trace: {output}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
