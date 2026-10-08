"""Capture the current pose as sleep2, then compare it against canonical Sleep."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.poses import PoseLibrary, SavedPose
from soarm101_motion.safety import (
    validate_joint_targets,
    validate_sleep_family_workspace_path,
)
from soarm101_motion.workstation import WorkstationProfileStore

SLEEP2_NAME = "sleep2"


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
        description="Record the current pose as sleep2 and compare it with canonical Sleep."
    )
    parser.add_argument("--port")
    parser.add_argument("--robot-id")
    parser.add_argument("--speed-deg-s", type=float, default=24.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=150.0)
    parser.add_argument("--output", type=Path)
    return parser


def _joint_path(
    start: dict[str, float],
    target: dict[str, float],
    *,
    step_rad: float,
) -> tuple[dict[str, float], ...]:
    max_delta = max(abs(float(target[name]) - float(start[name])) for name in ARM_JOINTS)
    steps = max(2, int(math.ceil(max_delta / step_rad)) + 1)
    return tuple(
        {
            name: float(start[name])
            + (float(target[name]) - float(start[name])) * (index / (steps - 1))
            for name in ARM_JOINTS
        }
        for index in range(steps)
    )


def _move_sleep2(
    arm: SOARM101,
    target: dict[str, float],
    *,
    speed: float,
    acceleration: float,
):
    start = dict(arm.get_joint_positions().positions)
    validate_joint_targets(target, limits=arm.get_joint_limits())
    validate_sleep_family_workspace_path(
        arm.model,
        _joint_path(
            start,
            target,
            step_rad=arm.config.workspace_check_step_rad,
        ),
        tcp=arm.active_tcp,
        **arm._workspace_kwargs(),
    )
    return arm.move_joints(
        target,
        speed=speed,
        acceleration=acceleration,
        execution_mode="streamed",
        workspace_check="off",
    )


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
        / f"sleep2-comparison-{stamp}.jsonl"
    ).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    summary_path = output.with_suffix(".summary.json")
    sleep2_path = output.with_name("sleep2.json")

    speed = math.radians(args.speed_deg_s)
    acceleration = math.radians(args.acceleration_deg_s2)
    cfg = SOARM101Config(
        port=port,
        robot_id=robot_id,
        configure_motors_on_connect=False,
        disable_torque_on_disconnect=False,
        auto_enable_torque=False,
    )
    library = PoseLibrary(robot_id)
    right = library.require("agent_start_overhead_right")

    print(f"Follower: {port}")
    print(f"Robot ID: {robot_id}")
    print(
        f"Sleep vs sleep2 comparison at {args.speed_deg_s:g} deg/s, "
        f"{args.acceleration_deg_s2:g} deg/s^2"
    )
    print()
    print("IMPORTANT: the pose the arm is in when this program starts will be saved as sleep2.")
    input("Press ENTER to capture the CURRENT pose as sleep2, or Ctrl-C to stop: ")

    with SOARM101(cfg) as arm:
        # Capture before issuing any arm motion or torque-latch command.
        sleep2_pose = SavedPose.capture(arm, source="manual_sleep2_capture")
        sleep2 = dict(sleep2_pose.joints)
        validate_joint_targets(sleep2, limits=arm.get_joint_limits())
        library.save(SLEEP2_NAME, sleep2_pose)

        sleep2_path.write_text(
            json.dumps(
                {
                    "pose_name": SLEEP2_NAME,
                    "pose": sleep2_pose.to_mapping(),
                    "canonical_sleep": arm.get_sleep_joint_positions(),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

        print()
        print("Captured sleep2:")
        for name in ARM_JOINTS:
            print(f"  {name}: {math.degrees(sleep2[name]):.2f} deg")
        print(f"Saved named pose: {SLEEP2_NAME}")
        print(f"Saved artifact: {sleep2_path}")

        # Latch the exact measured pose before beginning the A/B.
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

        canonical_sleep = arm.get_sleep_joint_positions()
        metadata = {
            "kind": "sleep2_comparison",
            "git_sha": _git_sha(),
            "speed_deg_s": args.speed_deg_s,
            "acceleration_deg_s2": args.acceleration_deg_s2,
            "canonical_sleep": canonical_sleep,
            "sleep2": sleep2,
            "sleep2_pose_name": SLEEP2_NAME,
            "right_pose_name": "agent_start_overhead_right",
        }

        with PassiveBackendTrace(arm, output, metadata=metadata) as trace:

            def reset_right() -> None:
                trace.mark("reset_start", destination="RIGHT")
                result = arm.move_joints_from_saved_pose(
                    right.joints,
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode="streamed",
                )
                arm.hold()
                trace.mark(
                    "reset_end",
                    destination="RIGHT",
                    completed=result.completed,
                    final_positions=dict(result.final_positions),
                )

            reset_right()
            input(
                "\nA NORMAL SLEEP: arm is at RIGHT. Press ENTER to move to canonical Sleep: "
            )
            trace.mark("condition_start", condition="canonical_sleep")
            normal = arm.move_sleep(
                speed=speed,
                acceleration=acceleration,
                execution_mode="streamed",
            )
            arm.hold()
            trace.mark(
                "condition_end",
                condition="canonical_sleep",
                completed=normal.completed,
                final_positions=dict(normal.final_positions),
            )

            reset_right()
            input(
                "\nB SLEEP2: arm is at RIGHT. Press ENTER to move to the recorded sleep2 pose: "
            )
            trace.mark("condition_start", condition="sleep2")
            alternate = _move_sleep2(
                arm,
                sleep2,
                speed=speed,
                acceleration=acceleration,
            )
            arm.hold()
            trace.mark(
                "condition_end",
                condition="sleep2",
                completed=alternate.completed,
                final_positions=dict(alternate.final_positions),
            )

            arm.hold()
            summary = dict(trace.summary)

    summary.update(metadata)
    summary["trace"] = str(output)
    summary["sleep2_file"] = str(sleep2_path)
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print()
    print("Sleep vs sleep2 comparison complete; follower remains torque-held.")
    print(f"Trace: {output}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
