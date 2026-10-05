"""Supervised comparison of canonical versus open/staged Sleep geometry."""

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
    return completed.stdout.strip() or None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare direct, open-wrist, and staged-wrist approaches to Sleep."
    )
    parser.add_argument("--port")
    parser.add_argument("--robot-id")
    parser.add_argument("--speed-deg-s", type=float, default=24.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=150.0)
    parser.add_argument(
        "--execution-mode",
        choices=("streamed", "final_target"),
        default="streamed",
    )
    parser.add_argument("--output", type=Path)
    return parser


def main() -> int:
    args = _parser().parse_args()
    if not math.isfinite(args.speed_deg_s) or args.speed_deg_s <= 0:
        raise ValueError("--speed-deg-s must be positive and finite")
    if not math.isfinite(args.acceleration_deg_s2) or args.acceleration_deg_s2 <= 0:
        raise ValueError("--acceleration-deg-s2 must be positive and finite")

    profile = WorkstationProfileStore().load()
    port = args.port or profile.follower.port
    robot_id = args.robot_id or profile.follower.robot_id or "so101"
    if not port:
        raise RuntimeError("no follower port is configured")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = (
        args.output
        or Path.home()
        / "soarm-motion-tests"
        / f"sleep-geometry-comparison-{stamp}.jsonl"
    ).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    summary_path = output.with_suffix(".summary.json")

    speed = math.radians(args.speed_deg_s)
    acceleration = math.radians(args.acceleration_deg_s2)
    cfg = SOARM101Config(
        port=port,
        robot_id=robot_id,
        configure_motors_on_connect=False,
        disable_torque_on_disconnect=False,
    )
    library = PoseLibrary(robot_id)
    right = library.require("agent_start_overhead_right")

    print(f"Follower: {port}")
    print(f"Robot ID: {robot_id}")
    print(
        "Sleep geometry comparison at "
        f"{args.speed_deg_s:g} deg/s, {args.acceleration_deg_s2:g} deg/s^2, "
        f"mode={args.execution_mode}"
    )
    print(f"Trace: {output}")
    print()
    print("Conditions:")
    print("  A direct: RIGHT -> canonical Sleep")
    print("  B open_wrist: RIGHT -> Sleep shoulder/elbow geometry with wrist_flex neutral")
    print("  C staged_wrist: RIGHT -> open_wrist pre-Sleep -> canonical Sleep")
    print()

    with SOARM101(cfg) as arm:
        arm.enable()
        arm.hold()
        arm.require_artifact_calibration(
            {
                "source_robot_id": right.source_robot_id,
                "source_calibration_id": right.source_calibration_id,
                "target_robot_id": right.target_robot_id,
                "target_calibration_id": right.target_calibration_id,
            },
            artifact_label="RIGHT saved pose",
        )

        sleep_target = arm.get_sleep_joint_positions()
        limits = arm.get_joint_limits()
        open_wrist_target = dict(sleep_target)
        wrist_lower, wrist_upper = limits["wrist_flex"]
        open_wrist_target["wrist_flex"] = (wrist_lower + wrist_upper) / 2.0

        metadata = {
            "kind": "sleep_geometry_comparison",
            "git_sha": _git_sha(),
            "robot_id": robot_id,
            "speed_deg_s": args.speed_deg_s,
            "acceleration_deg_s2": args.acceleration_deg_s2,
            "execution_mode": args.execution_mode,
            "canonical_sleep": sleep_target,
            "open_wrist_target": open_wrist_target,
            "right_pose_name": "agent_start_overhead_right",
        }

        with PassiveBackendTrace(arm, output, metadata=metadata) as trace:

            def reset_right() -> None:
                trace.mark("reset_start", destination="RIGHT")
                result = arm.move_joints_from_saved_pose(
                    right.joints,
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                )
                arm.hold()
                trace.mark(
                    "reset_end",
                    destination="RIGHT",
                    completed=result.completed,
                    final_positions=dict(result.final_positions),
                )

            def run_direct() -> None:
                reset_right()
                input(
                    "\nA DIRECT: arm is at RIGHT. Press ENTER for canonical Sleep, "
                    "or Ctrl-C to stop: "
                )
                trace.mark("condition_start", condition="direct")
                result = arm.move_sleep(
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                )
                arm.hold()
                trace.mark(
                    "condition_end",
                    condition="direct",
                    completed=result.completed,
                    final_positions=dict(result.final_positions),
                )

            def run_open_wrist() -> None:
                reset_right()
                input(
                    "\nB OPEN_WRIST: arm is at RIGHT. Press ENTER for shoulder/elbow "
                    "Sleep geometry with wrist flex held neutral, or Ctrl-C to stop: "
                )
                trace.mark("condition_start", condition="open_wrist")
                result = arm.move_joints(
                    open_wrist_target,
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                    workspace_check="full",
                )
                arm.hold()
                trace.mark(
                    "condition_end",
                    condition="open_wrist",
                    completed=result.completed,
                    final_positions=dict(result.final_positions),
                )

            def run_staged_wrist() -> None:
                reset_right()
                input(
                    "\nC STAGED_WRIST: arm is at RIGHT. Press ENTER to lower/fold "
                    "the main arm while keeping wrist flex neutral: "
                )
                trace.mark("condition_start", condition="staged_wrist_main_arm")
                first = arm.move_joints(
                    open_wrist_target,
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                    workspace_check="full",
                )
                arm.hold()
                trace.mark(
                    "condition_end",
                    condition="staged_wrist_main_arm",
                    completed=first.completed,
                    final_positions=dict(first.final_positions),
                )
                input(
                    "Main arm is down with wrist flex neutral. Press ENTER to fold "
                    "only the remaining wrist into canonical Sleep: "
                )
                trace.mark("condition_start", condition="staged_wrist_final_fold")
                second = arm.move_sleep(
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                )
                arm.hold()
                trace.mark(
                    "condition_end",
                    condition="staged_wrist_final_fold",
                    completed=second.completed,
                    final_positions=dict(second.final_positions),
                )

            run_direct()
            run_open_wrist()
            run_staged_wrist()
            arm.hold()
            summary = dict(trace.summary)

    summary.update(metadata)
    summary["trace"] = str(output)
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print()
    print("Sleep geometry comparison complete; follower remains torque-held.")
    print(f"Trace: {output}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
