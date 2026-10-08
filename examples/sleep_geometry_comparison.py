"""Teach a wrist-relaxed Sleep variant, then compare direct versus staged Sleep."""

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
from soarm101_motion.motion.quality import maximum_joint_drift
from soarm101_motion.poses import PoseLibrary, SavedPose
from soarm101_motion.safety import (
    validate_joint_targets,
    validate_sleep_family_workspace_path,
)
from soarm101_motion.workstation import WorkstationProfileStore

TAUGHT_POSE_NAME = "motion_test_sleep_wrist"


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
        description="Teach a manual wrist Sleep variant and compare direct/staged folds."
    )
    parser.add_argument("--port")
    parser.add_argument("--robot-id")
    parser.add_argument("--speed-deg-s", type=float, default=24.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=150.0)
    parser.add_argument(
        "--execution-mode",
        choices=("streamed",),
        default="streamed",
        help="manual Sleep-family geometry test currently uses guarded streamed execution",
    )
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


def _validate_sleep_family_path(
    arm: SOARM101,
    start: dict[str, float],
    target: dict[str, float],
) -> None:
    """Validate a Sleep-family path while ignoring only coarse self-clearance.

    Canonical Sleep already needs this narrow exception because its folded centerlines
    are intentionally closer than the generic 25 mm heuristic. Floor, reach, base
    keepout, calibrated joint limits, and all runtime motion guards remain active.
    """
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


def _move_sleep_family(
    arm: SOARM101,
    target: dict[str, float],
    *,
    speed: float,
    acceleration: float,
    execution_mode: str,
):
    start = dict(arm.get_joint_positions().positions)
    _validate_sleep_family_path(arm, start, target)
    # The path was just validated with only coarse self-clearance removed. Use
    # workspace_check="off" here solely to avoid running the generic 25 mm check
    # a second time inside move_joints; all non-workspace runtime guards remain.
    return arm.move_joints(
        target,
        speed=speed,
        acceleration=acceleration,
        execution_mode=execution_mode,
        workspace_check="off",
    )


def _teach_wrist(
    arm: SOARM101,
    library: PoseLibrary,
    *,
    speed: float,
    acceleration: float,
    execution_mode: str,
) -> tuple[SavedPose, dict[str, float]]:
    print()
    print("TEACH WRIST")
    print("Moving to canonical Sleep and holding all motors...")
    arm.move_sleep(
        speed=speed,
        acceleration=acceleration,
        execution_mode=execution_mode,
    )
    arm.hold()
    before = dict(arm.get_joint_positions().positions)

    print()
    print("Only wrist_flex will now be relaxed.")
    print("Shoulder pan/lift, elbow, wrist roll, and gripper remain torque-held.")
    input("Keep hands clear of the other joints and press ENTER to relax wrist_flex: ")

    arm.backend.disable_torque(["wrist_flex"])
    wrist_reenabled = False
    try:
        limits = arm.get_joint_limits()
        lower, upper = limits["wrist_flex"]
        while True:
            print()
            print("wrist_flex is RELAXED. Move only the wrist by hand to the pose you want.")
            input("When the wrist looks right, press ENTER to capture it: ")
            measured = dict(arm.get_joint_positions().positions)
            wrist = float(measured["wrist_flex"])
            if lower <= wrist <= upper:
                break
            print(
                "That wrist angle is outside the executable calibrated range: "
                f"{math.degrees(wrist):.2f} deg not within "
                f"{math.degrees(lower):.2f}..{math.degrees(upper):.2f} deg."
            )
            print("Move the relaxed wrist back inside the range and try again.")

        worst_other = maximum_joint_drift(
            before,
            measured,
            exclude=("wrist_flex",),
        )
        if worst_other > arm.config.joint_position_tolerance_rad:
            raise RuntimeError(
                "a held non-wrist joint moved too far during wrist teaching "
                f"({math.degrees(worst_other):.2f} deg); refusing to save the pose"
            )

        # Re-enable only the taught wrist. The hardware backend first latches the
        # freshly measured position, then enables torque, preventing a stale-goal jump.
        arm.backend.enable_torque(["wrist_flex"])
        wrist_reenabled = True
        arm.hold()

        taught = SavedPose.capture(arm, source="manual_wrist_sleep_teach")
        library.save(TAUGHT_POSE_NAME, taught)
        taught_joints = dict(taught.joints)
        print()
        print(
            f"Saved {TAUGHT_POSE_NAME!r}: wrist_flex="
            f"{math.degrees(taught_joints['wrist_flex']):.2f} deg"
        )
        return taught, taught_joints
    finally:
        if not wrist_reenabled:
            try:
                arm.backend.enable_torque(["wrist_flex"])
                arm.hold()
            except Exception:
                # Preserve the original failure. The operator still has physical
                # power control if relatching cannot be confirmed.
                pass


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
    taught_path = output.with_name("taught-sleep-wrist.json")

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
        "Manual wrist Sleep comparison at "
        f"{args.speed_deg_s:g} deg/s, {args.acceleration_deg_s2:g} deg/s^2, "
        f"mode={args.execution_mode}"
    )
    print(f"Trace: {output}")

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

        taught_pose, taught_sleep = _teach_wrist(
            arm,
            library,
            speed=speed,
            acceleration=acceleration,
            execution_mode=args.execution_mode,
        )
        taught_path.write_text(
            json.dumps(
                {
                    "pose_name": TAUGHT_POSE_NAME,
                    "pose": taught_pose.to_mapping(),
                    "canonical_sleep": arm.get_sleep_joint_positions(),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )

        canonical_sleep = arm.get_sleep_joint_positions()
        metadata = {
            "kind": "manual_sleep_wrist_comparison",
            "git_sha": _git_sha(),
            "speed_deg_s": args.speed_deg_s,
            "acceleration_deg_s2": args.acceleration_deg_s2,
            "execution_mode": args.execution_mode,
            "canonical_sleep": canonical_sleep,
            "taught_sleep": taught_sleep,
            "taught_pose_name": TAUGHT_POSE_NAME,
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

            reset_right()
            input(
                "\nA DIRECT: arm is at RIGHT. Press ENTER for canonical Sleep, "
                "or Ctrl-C to stop: "
            )
            trace.mark("condition_start", condition="direct_canonical_sleep")
            direct = arm.move_sleep(
                speed=speed,
                acceleration=acceleration,
                execution_mode=args.execution_mode,
            )
            arm.hold()
            trace.mark(
                "condition_end",
                condition="direct_canonical_sleep",
                completed=direct.completed,
                final_positions=dict(direct.final_positions),
            )

            reset_right()
            input(
                "\nB TAUGHT_WRIST: arm is at RIGHT. Press ENTER for the same folded "
                "Sleep arm geometry using your taught wrist angle: "
            )
            trace.mark("condition_start", condition="direct_taught_wrist_sleep")
            taught_result = _move_sleep_family(
                arm,
                taught_sleep,
                speed=speed,
                acceleration=acceleration,
                execution_mode=args.execution_mode,
            )
            arm.hold()
            trace.mark(
                "condition_end",
                condition="direct_taught_wrist_sleep",
                completed=taught_result.completed,
                final_positions=dict(taught_result.final_positions),
            )

            reset_right()
            input(
                "\nC STAGED: arm is at RIGHT. Press ENTER to fold the arm using "
                "your taught wrist angle first: "
            )
            trace.mark("condition_start", condition="staged_taught_wrist")
            staged_first = _move_sleep_family(
                arm,
                taught_sleep,
                speed=speed,
                acceleration=acceleration,
                execution_mode=args.execution_mode,
            )
            arm.hold()
            trace.mark(
                "condition_end",
                condition="staged_taught_wrist",
                completed=staged_first.completed,
                final_positions=dict(staged_first.final_positions),
            )

            input(
                "Taught-wrist Sleep reached. Press ENTER to move only wrist_flex "
                "into canonical Sleep: "
            )
            current = dict(arm.get_joint_positions().positions)
            final_target = dict(current)
            final_target["wrist_flex"] = canonical_sleep["wrist_flex"]
            trace.mark("condition_start", condition="staged_final_wrist_fold")
            staged_final = _move_sleep_family(
                arm,
                final_target,
                speed=speed,
                acceleration=acceleration,
                execution_mode=args.execution_mode,
            )
            arm.hold()
            trace.mark(
                "condition_end",
                condition="staged_final_wrist_fold",
                completed=staged_final.completed,
                final_positions=dict(staged_final.final_positions),
            )

            arm.hold()
            summary = dict(trace.summary)

    summary.update(metadata)
    summary["trace"] = str(output)
    summary["taught_pose_file"] = str(taught_path)
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print()
    print("Manual wrist Sleep comparison complete; follower remains torque-held.")
    print(f"Saved pose: {TAUGHT_POSE_NAME}")
    print(f"Taught pose artifact: {taught_path}")
    print(f"Trace: {output}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
