"""Attended joint-only A/B comparison of the optional post-settle HOLD.

The four trials use one session, identical saved-pose moves, and A/B/B/A order.
All motion uses guarded SDK primitives; exceptions abort the sequence and normal
SDK STOP/cleanup remains active. This is not a physical STOP qualification.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.workstation import WorkstationProfileStore

from motion_quality_trace import _git_sha, _simulation_fallback_poses


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulation", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    workstation = WorkstationProfileStore().load()
    robot_id = workstation.follower.robot_id or "so101"
    port = None if args.simulation else workstation.follower.port
    if not args.simulation and not port:
        raise RuntimeError("no follower port is configured")
    cfg = SOARM101Config(port=port, robot_id=robot_id, auto_enable_torque=False,
                         disable_torque_on_disconnect=False)
    library = PoseLibrary(robot_id)
    try:
        start = library.require("agent_start_overhead")
        target = library.require("agent_start_overhead_left")
    except KeyError:
        if not args.simulation:
            raise
        fallback = _simulation_fallback_poses()
        start, target = fallback["OVERHEAD"], fallback["LEFT"]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or Path.home() / "soarm-motion-tests" / f"hold-comparison-{stamp}.jsonl"
    arm_context = SOARM101.simulated(realtime=False, config=cfg) if args.simulation else SOARM101(cfg)
    with arm_context as arm:
        for pose in (start, target):
            arm.require_artifact_calibration(
                {key: getattr(pose, key) for key in (
                    "source_robot_id", "source_calibration_id", "target_robot_id", "target_calibration_id"
                )}, artifact_label="hold comparison saved pose"
            )
        with PassiveBackendTrace(arm, output, metadata={
            "kind": "hold_comparison", "git_sha": _git_sha(), "simulation": args.simulation,
            "profile_mode": "cosine_cruise", "speed_deg_s": 8.0,
            "acceleration_deg_s2": 25.0, "order": ["hold", "no_extra_hold", "no_extra_hold", "hold"],
        }) as trace:
            trace.mark("enable_start")
            arm.enable()
            trace.mark("enable_complete")
            options = {"speed": math.radians(8), "acceleration": math.radians(25),
                       "profile_mode": "cosine_cruise"}
            print("Moving to the starting pose.", flush=True)
            arm.move_joints_from_saved_pose(start.joints, **options)
            for index, extra_hold in enumerate((True, False, False, True), start=1):
                print(f"Trial {index}/4", flush=True)
                trace.mark("trial_start", trial=index, condition="hold" if extra_hold else "no_extra_hold")
                arm.move_joints_from_saved_pose(target.joints, **options)
                trace.mark("settled", trial=index)
                observation_reference = arm.backend.read_joint_positions()
                if extra_hold:
                    trace.mark("optional_hold_start", trial=index)
                    arm.hold()
                    trace.mark("optional_hold_complete", trial=index)
                trace.mark("observation_start", trial=index)
                for _ in range(20):
                    arm.motion._read_guarded_actual(observation_reference)
                    time.sleep(0.1)
                trace.mark("observation_end", trial=index)
                if index < 4:
                    trace.mark("return_start", trial=index)
                    arm.move_joints_from_saved_pose(start.joints, **options)
                    trace.mark("return_complete", trial=index)
            trace.mark("experiment_complete")
    print(json.dumps({"trace": str(output), "simulation": args.simulation, "completed": True}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
