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
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.exceptions import SafetyViolationError
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.safety import (
    minimum_workspace_self_clearance,
    validate_workspace_path,
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
    return completed.stdout.strip() or None


def _interpolate_joints(
    start: dict[str, float],
    target: dict[str, float],
    fraction: float,
) -> dict[str, float]:
    return {
        name: float(start[name]) + (float(target[name]) - float(start[name])) * fraction
        for name in ARM_JOINTS
    }


def _joint_path(
    start: dict[str, float],
    target: dict[str, float],
    *,
    step_rad: float,
) -> tuple[dict[str, float], ...]:
    max_delta = max(abs(float(target[name]) - float(start[name])) for name in ARM_JOINTS)
    steps = max(2, int(math.ceil(max_delta / step_rad)) + 1)
    return tuple(
        _interpolate_joints(start, target, index / (steps - 1))
        for index in range(steps)
    )


def _deepest_safe_open_target(
    arm: SOARM101,
    start: dict[str, float],
    desired: dict[str, float],
    *,
    clearance_reserve_m: float = 0.002,
) -> tuple[dict[str, float], float, float]:
    """Find the deepest strict-workspace fold toward desired with a clearance reserve."""
    required = arm.config.minimum_self_clearance_m + clearance_reserve_m
    workspace = {
        **arm._workspace_kwargs(),  # diagnostic uses the same SDK workspace contract
        "minimum_self_clearance_m": required,
    }

    # Search from deepest to most open in 1% increments. This is diagnostic target
    # selection only; every powered move is independently validated again by the SDK.
    for step in range(100, 0, -1):
        fraction = step / 100.0
        candidate = _interpolate_joints(start, desired, fraction)
        path = _joint_path(
            start,
            candidate,
            step_rad=arm.config.workspace_check_step_rad,
        )
        try:
            validate_workspace_path(
                arm.model,
                path,
                tcp=arm.active_tcp,
                **workspace,
            )
        except SafetyViolationError:
            continue
        clearance = minimum_workspace_self_clearance(
            arm.model,
            candidate,
            tcp=arm.active_tcp,
        )
        return candidate, fraction, clearance

    raise RuntimeError(
        "could not find a nontrivial open pre-Sleep target that passes the strict "
        "workspace envelope with the requested self-clearance reserve"
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare direct Sleep with a validated open pre-Sleep and staged fold."
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
    print("  B open_pre_sleep: RIGHT -> deepest strict-workspace-valid neutral-wrist fold")
    print("  C staged_sleep: RIGHT -> open pre-Sleep -> canonical Sleep")
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
        wrist_lower, wrist_upper = limits["wrist_flex"]
        neutral_wrist = (wrist_lower + wrist_upper) / 2.0

        metadata = {
            "kind": "sleep_geometry_comparison",
            "git_sha": _git_sha(),
            "speed_deg_s": args.speed_deg_s,
            "acceleration_deg_s2": args.acceleration_deg_s2,
            "execution_mode": args.execution_mode,
            "canonical_sleep": sleep_target,
            "neutral_wrist_flex": neutral_wrist,
            "right_pose_name": "agent_start_overhead_right",
        }

        with PassiveBackendTrace(arm, output, metadata=metadata) as trace:

            def reset_right() -> dict[str, float]:
                trace.mark("reset_start", destination="RIGHT")
                result = arm.move_joints_from_saved_pose(
                    right.joints,
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                )
                arm.hold()
                measured = dict(arm.get_joint_positions().positions)
                trace.mark(
                    "reset_end",
                    destination="RIGHT",
                    completed=result.completed,
                    final_positions=dict(result.final_positions),
                    measured_positions=measured,
                )
                return measured

            def open_target_from_right(
                measured_right: dict[str, float],
            ) -> tuple[dict[str, float], float, float]:
                desired = dict(sleep_target)
                desired["wrist_flex"] = neutral_wrist
                target, fraction, clearance = _deepest_safe_open_target(
                    arm,
                    measured_right,
                    desired,
                )
                trace.mark(
                    "open_target_selected",
                    fraction_toward_sleep=fraction,
                    target=target,
                    self_clearance_m=clearance,
                )
                print(
                    "Selected open pre-Sleep at "
                    f"{fraction * 100.0:.0f}% of the RIGHT->neutral-wrist Sleep fold "
                    f"(coarse clearance {clearance * 1000.0:.1f} mm)."
                )
                return target, fraction, clearance

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
                measured_right = reset_right()
                open_target, fraction, clearance = open_target_from_right(measured_right)
                input(
                    "\nB OPEN_PRE_SLEEP: arm is at RIGHT. Press ENTER for the deepest "
                    "strict-workspace-valid neutral-wrist fold, or Ctrl-C to stop: "
                )
                trace.mark(
                    "condition_start",
                    condition="open_pre_sleep",
                    fraction_toward_sleep=fraction,
                    self_clearance_m=clearance,
                )
                result = arm.move_joints(
                    open_target,
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                    workspace_check="full",
                )
                arm.hold()
                trace.mark(
                    "condition_end",
                    condition="open_pre_sleep",
                    completed=result.completed,
                    final_positions=dict(result.final_positions),
                )

            def run_staged_wrist() -> None:
                measured_right = reset_right()
                open_target, fraction, clearance = open_target_from_right(measured_right)
                input(
                    "\nC STAGED_SLEEP: arm is at RIGHT. Press ENTER for the deepest "
                    "strict-workspace-valid neutral-wrist pre-Sleep: "
                )
                trace.mark(
                    "condition_start",
                    condition="staged_sleep_open",
                    fraction_toward_sleep=fraction,
                    self_clearance_m=clearance,
                )
                first = arm.move_joints(
                    open_target,
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                    workspace_check="full",
                )
                arm.hold()
                trace.mark(
                    "condition_end",
                    condition="staged_sleep_open",
                    completed=first.completed,
                    final_positions=dict(first.final_positions),
                )
                input(
                    "Open pre-Sleep reached. Press ENTER to continue from that valid "
                    "configuration into canonical Sleep: "
                )
                trace.mark("condition_start", condition="staged_sleep_final_fold")
                second = arm.move_sleep(
                    speed=speed,
                    acceleration=acceleration,
                    execution_mode=args.execution_mode,
                )
                arm.hold()
                trace.mark(
                    "condition_end",
                    condition="staged_sleep_final_fold",
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
