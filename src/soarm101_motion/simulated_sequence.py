"""Guarded execution of an operator-pinned sequence in SDK simulation only."""
from __future__ import annotations

import math
import threading
import time

from soarm101_motion.asset_catalog import discover_assets
from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.broker_watchdog import BrokerWatchdog
from soarm101_motion.hardware import SimulationBackend
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.provenance import require_calibration_compatibility
from soarm101_motion.replay_policy import ReplayPolicy
from soarm101_motion.replay_preflight import ReplayPreflight
from soarm101_motion.replay_snapshot import load_pinned_recording
from soarm101_motion.sequences import SequenceLibrary
from soarm101_motion.simulated_replay import (
    ReplayExecutionError, _require_replay_authority, _stop_after_failure,
)
from soarm101_motion.tools.gripper import SO101Gripper
from soarm101_motion.trajectories import TrajectoryLibrary
from soarm101_motion.types import MotionResult
from soarm101_motion.motion.controller import MotionHandle


def _completed_result(result: MotionResult | MotionHandle[MotionResult]) -> MotionResult:
    """Resolve the SDK's wait=True result union to its completed value."""
    return result.wait() if isinstance(result, MotionHandle) else result


def execute_simulated_sequence(executor: SDKAgentExecutor, watchdog: BrokerWatchdog,
                               policy: ReplayPolicy, preflight: ReplayPreflight,
                               poses: PoseLibrary, trajectories: TrajectoryLibrary,
                               sequences: SequenceLibrary, *, speed_scale: float
                               ) -> dict[str, object]:
    """Execute validated ordered steps with the recording replay STOP/lease fence."""
    if not executor.simulation or not executor.simulation_actions:
        raise PermissionError("sequence execution requires simulation action mode")
    if (getattr(executor.motion_guard, "__self__", None) is not watchdog
            or watchdog._worker is None or not watchdog._worker.is_alive()):
        raise PermissionError("active broker watchdog is required")
    if (isinstance(speed_scale, bool) or not isinstance(speed_scale, (int, float))
            or not math.isfinite(speed_scale) or not 0 < speed_scale <= policy.max_speed_scale):
        raise ValueError("speed scale exceeds replay policy")
    if policy.robot_id != executor.robot_id or policy.calibration_id != "simulation":
        raise PermissionError("replay policy does not match simulation identity")
    with executor._dispatch_lock:
        executor._require_active()
        arm = executor._session
        if arm is None or not isinstance(arm.backend, SimulationBackend):
            raise PermissionError("existing simulated SDK session required")
        if not isinstance(arm.tool, SO101Gripper):
            raise PermissionError("stock simulated gripper required for sequence replay")
        gripper = arm.tool
        executor._assert_session_identity(arm)
        watchdog.check()
        _require_replay_authority(executor, arm)
        state = arm.get_state()
        if not state.connected or state.faulted or state.moving:
            raise PermissionError("healthy stationary simulation state required")
        current = {item.id: item.revision for item in discover_assets(
            poses, trajectories, sequences, current_robot_id=policy.robot_id,
            current_calibration_id=policy.calibration_id,
        )}
        for asset_id, revision in preflight.dependencies:
            if current.get(asset_id) != revision:
                raise RuntimeError("replay asset changed after sequence preflight")
            policy.require_asset(asset_id, revision)
        if preflight.known_duration_s > policy.max_duration_s:
            raise ValueError("sequence exceeds replay duration ceiling")
        sequence_descriptor = next((item for item in discover_assets(
            poses, trajectories, sequences) if item.id == preflight.sequence_id), None)
        if sequence_descriptor is None:
            raise ValueError("pinned sequence is missing")
        enable_attempted = False
        deadline_fired = threading.Event()
        final_lock = threading.Lock()
        finalized = False
        def on_deadline() -> None:
            with final_lock:
                if finalized:
                    return
                deadline_fired.set()
            _stop_after_failure(executor, watchdog)

        def require_execution_authority() -> None:
            """Fence every SDK submission against broker STOP and deadline."""
            if deadline_fired.is_set():
                raise RuntimeError("sequence deadline expired; completion unconfirmed")
            watchdog.check()
            _require_replay_authority(executor, arm)

        def gripper_rates(step_scale: float) -> tuple[int, int]:
            # Replay scaling applies to every actuator in the step. Keep the
            # operator-pinned broker rates as hard ceilings.
            return (
                max(1, math.floor(executor.rates.gripper_speed_raw * step_scale)),
                max(1, math.floor(executor.rates.gripper_acceleration_raw * step_scale)),
            )

        timer = threading.Timer(policy.max_duration_s, on_deadline)
        timer.daemon = True
        try:
            with watchdog._lock:
                watchdog.check()
                _require_replay_authority(executor, arm)
                enable_attempted = True
                arm.enable()
            timer.start()
            completed: list[dict[str, object]] = []
            for step in preflight.planned_steps:
                require_execution_authority()
                latest = {item.id: item.revision for item in discover_assets(
                    poses, trajectories, sequences, current_robot_id=policy.robot_id,
                    current_calibration_id=policy.calibration_id,
                )}
                if any(latest.get(asset_id) != revision
                       for asset_id, revision in preflight.dependencies):
                    raise RuntimeError("replay asset changed during sequence execution")
                if step.pose_target is not None:
                    from urllib.parse import unquote
                    pose_name = unquote(step.pose_target.asset_id.split("/", 3)[-1])
                    pose = poses.require(pose_name)
                    require_calibration_compatibility(
                        {"source_robot_id": pose.source_robot_id,
                         "source_calibration_id": pose.source_calibration_id,
                         "target_robot_id": pose.target_robot_id,
                         "target_calibration_id": pose.target_calibration_id},
                        current_robot_id=policy.robot_id,
                        current_calibration_id=policy.calibration_id, artifact_label="pose",
                    )
                    speed = math.radians(executor.rates.joint_speed_deg_s * step.speed_scale)
                    acceleration = math.radians(executor.rates.joint_acceleration_deg_s2 * step.speed_scale)
                    require_execution_authority()
                    result = _completed_result(arm.move_joints_from_saved_pose(
                        dict(step.pose_target.joints_rad), speed=speed,
                        acceleration=acceleration, wait=True,
                    ))
                    if not result.accepted or not result.completed:
                        raise RuntimeError("sequence pose step did not complete")
                    if step.kind in {"home", "rest"} and step.pose_target.gripper is not None:
                        # This is a second SDK action. STOP or lease revocation
                        # can race the pose completion, so re-admit independently.
                        require_execution_authority()
                        speed_raw, acceleration_raw = gripper_rates(step.speed_scale)
                        result_gripper = _completed_result(gripper.move(
                            step.pose_target.gripper,
                            speed_raw=speed_raw, acceleration_raw=acceleration_raw, wait=True,
                        ))
                        if not result_gripper.accepted or not result_gripper.completed:
                            raise RuntimeError("sequence pose gripper step did not complete")
                elif step.kind == "gripper":
                    if step.gripper_position is None:
                        raise ValueError("preflight omitted gripper target")
                    require_execution_authority()
                    speed_raw, acceleration_raw = gripper_rates(step.speed_scale)
                    result = _completed_result(gripper.move(
                        step.gripper_position,
                        speed_raw=speed_raw, acceleration_raw=acceleration_raw, wait=True,
                    ))
                    if not result.accepted or not result.completed:
                        raise RuntimeError("sequence gripper step did not complete")
                elif step.kind == "wait":
                    if step.wait_seconds is None:
                        raise ValueError("preflight omitted wait duration")
                    end = time.monotonic() + step.wait_seconds
                    while time.monotonic() < end:
                        watchdog.check()
                        _require_replay_authority(executor, arm)
                        if deadline_fired.is_set():
                            raise RuntimeError("sequence deadline expired; completion unconfirmed")
                        time.sleep(min(0.05, max(0.0, end - time.monotonic())))
                elif step.entry is not None:
                    if step.asset_id is None:
                        raise ValueError("preflight omitted recording asset ID")
                    snapshot = load_pinned_recording(policy, step.asset_id, trajectories)
                    trajectory = snapshot.trajectory()
                    require_calibration_compatibility(
                        trajectory.metadata, current_robot_id=policy.robot_id,
                        current_calibration_id=policy.calibration_id, artifact_label="recording",
                    )
                    for _ in range(step.loops):
                        require_execution_authority()
                        policy.check_start(arm.get_joint_positions().positions,
                                           dict(step.entry.joints_rad))
                        policy.check_gripper_start(gripper.get_position(),
                                                   step.entry.gripper)
                        plan = arm.motion.plan_recorded_trajectory(
                            trajectory, speed_scale=step.speed_scale,
                            move_to_start=False, tcp=arm.active_tcp,
                        )
                        arm.motion._validate_samples(
                            plan.command_samples,
                            speed_limit=math.radians(executor.rates.joint_speed_deg_s),
                            acceleration_limit=math.radians(executor.rates.joint_acceleration_deg_s2),
                        )
                        require_execution_authority()
                        speed_raw, acceleration_raw = gripper_rates(step.speed_scale)
                        replay = _completed_result(arm.play_trajectory(
                            trajectory, speed_scale=step.speed_scale, move_to_start=False,
                            gripper_speed_raw=speed_raw,
                            gripper_acceleration_raw=acceleration_raw,
                            wait=True,
                        ))
                        if not replay.accepted or not replay.completed:
                            raise RuntimeError("sequence recording step did not complete")
                require_execution_authority()
                completed.append({"index": step.index, "kind": step.kind,
                                  "asset_id": step.asset_id, "completed": True})
            with final_lock:
                if not completed or deadline_fired.is_set():
                    raise RuntimeError("sequence completion is unconfirmed")
                watchdog.check()
                _require_replay_authority(executor, arm)
                finalized = True
                return {"accepted": True, "completed": True, "mode": "simulation",
                        "sequence_id": preflight.sequence_id,
                        "revision": sequence_descriptor.revision,
                        "steps": completed, "broker_session": executor.session_evidence}
        except BaseException as exc:
            if not enable_attempted:
                raise
            stop = _stop_after_failure(executor, watchdog)
            raise ReplayExecutionError(exc, stop) from exc
        finally:
            with final_lock:
                finalized = True
            timer.cancel()
