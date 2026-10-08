"""Compare the new default Sleep posture against historical sleep_up."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.exceptions import MotionTimeoutError
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.motion.quality import maximum_joint_drift
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
        description="Compare RIGHT -> default Sleep against RIGHT -> historical sleep_up."
    )
    parser.add_argument("--port")
    parser.add_argument("--robot-id")
    parser.add_argument("--speed-deg-s", type=float, default=24.0)
    parser.add_argument("--acceleration-deg-s2", type=float, default=150.0)
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
        / f"sleep-vs-sleep-up-{stamp}.jsonl"
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
        auto_enable_torque=False,
    )
    right = PoseLibrary(robot_id).require("agent_start_overhead_right")

    print(f"Follower: {port}")
    print(f"Robot ID: {robot_id}")
    print(
        f"Sleep vs sleep_up at {args.speed_deg_s:g} deg/s, "
        f"{args.acceleration_deg_s2:g} deg/s^2"
    )

    with SOARM101(cfg) as arm:
        arm.require_artifact_calibration(
            {
                "source_robot_id": right.source_robot_id,
                "source_calibration_id": right.source_calibration_id,
                "target_robot_id": right.target_robot_id,
                "target_calibration_id": right.target_calibration_id,
            },
            artifact_label="RIGHT saved pose",
        )
        sleep = arm.get_sleep_joint_positions()
        sleep_up = arm.get_sleep_up_joint_positions()

        print("\nDerived calibrated postures:")
        print(
            "  default Sleep wrist_flex: "
            f"{math.degrees(sleep['wrist_flex']):.2f} deg"
        )
        print(
            "  sleep_up wrist_flex:      "
            f"{math.degrees(sleep_up['wrist_flex']):.2f} deg"
        )
        print("\nOther joint targets:")
        for name in ARM_JOINTS:
            if name == "wrist_flex":
                continue
            print(
                f"  {name}: Sleep={math.degrees(sleep[name]):.2f} deg, "
                f"sleep_up={math.degrees(sleep_up[name]):.2f} deg"
            )

        metadata = {
            "kind": "sleep_vs_sleep_up",
            "git_sha": _git_sha(),
            "speed_deg_s": args.speed_deg_s,
            "acceleration_deg_s2": args.acceleration_deg_s2,
            "sleep": sleep,
            "sleep_up": sleep_up,
            "right_pose_name": "agent_start_overhead_right",
        }

        arm.enable()
        arm.hold()

        with PassiveBackendTrace(arm, output, metadata=metadata) as trace:

            def reset_right() -> None:
                trace.mark("reset_start", destination="RIGHT")
                try:
                    result = arm.move_joints_from_saved_pose(
                        right.joints,
                        speed=speed,
                        acceleration=acceleration,
                        execution_mode="streamed",
                    )
                except MotionTimeoutError:
                    measured = dict(arm.get_joint_positions().positions)
                    worst_error = maximum_joint_drift(right.joints, measured)
                    retry_bound = 2.0 * arm.config.joint_position_tolerance_rad
                    trace.mark(
                        "reset_near_target_timeout",
                        destination="RIGHT",
                        worst_error_rad=worst_error,
                        retry_bound_rad=retry_bound,
                        measured_positions=measured,
                    )
                    if worst_error > retry_bound:
                        raise
                    print(
                        "RIGHT reset missed settle narrowly "
                        f"({math.degrees(worst_error):.2f} deg worst error); "
                        "retrying the same guarded target once."
                    )
                    result = arm.move_joints_from_saved_pose(
                        right.joints,
                        speed=speed,
                        acceleration=acceleration,
                        execution_mode="streamed",
                    )
                    trace.mark(
                        "reset_retry_completed",
                        destination="RIGHT",
                        completed=result.completed,
                        final_positions=dict(result.final_positions),
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
                "\nA DEFAULT SLEEP: arm is at RIGHT. "
                "Press ENTER to move to new default Sleep: "
            )
            trace.mark("condition_start", condition="sleep")
            result_sleep = arm.move_sleep(
                speed=speed,
                acceleration=acceleration,
                execution_mode="streamed",
            )
            arm.hold()
            trace.mark(
                "condition_end",
                condition="sleep",
                completed=result_sleep.completed,
                final_positions=dict(result_sleep.final_positions),
            )

            reset_right()
            input(
                "\nB SLEEP_UP: arm is at RIGHT. "
                "Press ENTER to move to historical wrist-up Sleep: "
            )
            trace.mark("condition_start", condition="sleep_up")
            result_sleep_up = arm.move_sleep_up(
                speed=speed,
                acceleration=acceleration,
                execution_mode="streamed",
            )
            arm.hold()
            trace.mark(
                "condition_end",
                condition="sleep_up",
                completed=result_sleep_up.completed,
                final_positions=dict(result_sleep_up.final_positions),
            )

            arm.hold()
            summary = dict(trace.summary)

    summary.update(metadata)
    summary["trace"] = str(output)
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print("\nSleep vs sleep_up comparison complete; follower remains torque-held.")
    print(f"Trace: {output}")
    print(f"Summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
