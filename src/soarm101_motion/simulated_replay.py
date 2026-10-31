"""Supervised recording replay for an already-connected simulation session only.

Used by the explicitly granted simulation broker route and software tests.
This uses the existing SDK planner, controller and STOP; physical replay is
deliberately unavailable pending separate qualification.
"""
from __future__ import annotations

import math
import threading
from dataclasses import asdict

from soarm101_motion.broker_sdk import SDKAgentExecutor
from soarm101_motion.broker_watchdog import BrokerWatchdog
from soarm101_motion.hardware import SimulationBackend
from soarm101_motion.provenance import require_calibration_compatibility
from soarm101_motion.replay_policy import ReplayPolicy
from soarm101_motion.replay_snapshot import RecordingSnapshot
from soarm101_motion.tools.gripper import SO101Gripper


def replay_simulated_recording(
    executor: SDKAgentExecutor, watchdog: BrokerWatchdog, policy: ReplayPolicy,
    snapshot: RecordingSnapshot, *, speed_scale: float,
) -> dict[str, object]:
    """Play one pinned snapshot under broker grant, replay policy and watchdog.

    A deadline requests the existing independent STOP and makes late success
    invalid. Calling this helper does not grant broker authority or physical qualification.
    """
    if not executor.simulation or not executor.simulation_actions:
        raise PermissionError("recorded replay helper requires simulation action mode")
    if (getattr(executor.motion_guard, "__self__", None) is not watchdog
            or watchdog._worker is None or not watchdog._worker.is_alive()):
        raise PermissionError("active broker watchdog is required")
    if (isinstance(speed_scale, bool) or not isinstance(speed_scale, (int, float))
            or not math.isfinite(speed_scale) or not 0 < speed_scale <= policy.max_speed_scale):
        raise ValueError("speed scale exceeds replay policy")
    if policy.robot_id != executor.robot_id or policy.calibration_id != "simulation":
        raise PermissionError("replay policy does not match simulation identity")
    policy.require_asset(snapshot.asset_id, snapshot.revision)
    trajectory = snapshot.trajectory()
    require_calibration_compatibility(
        trajectory.metadata, current_robot_id=policy.robot_id,
        current_calibration_id=policy.calibration_id, artifact_label="recording",
    )
    if trajectory.duration_s / speed_scale > policy.max_duration_s:
        raise ValueError("recording exceeds replay duration ceiling")
    with executor._dispatch_lock:
        executor._require_active()
        arm = executor._session
        if arm is None or not isinstance(arm.backend, SimulationBackend):
            raise PermissionError("existing simulated SDK session required")
        if not isinstance(arm.tool, SO101Gripper):
            raise PermissionError("stock simulated gripper required for recording replay")
        executor._assert_session_identity(arm)
        watchdog.check()
        _require_replay_authority(executor, arm)
        state = arm.get_state()
        if not state.connected or state.faulted or state.moving:
            raise PermissionError("healthy stationary simulation state required")
        entry = trajectory.joint_mappings()[0]
        policy.check_start(arm.get_joint_positions().positions, entry)
        policy.check_gripper_start(arm.tool.get_position(), float(trajectory.gripper[0]))
        # A partially failed enable can still have changed device state, so
        # begin cleanup responsibility before attempting it.
        enable_attempted = False
        try:
            with watchdog._lock:
                watchdog.check()
                _require_replay_authority(executor, arm)
                enable_attempted = True
                arm.enable()
            return _execute_enabled(executor, watchdog, policy, snapshot, trajectory, arm, speed_scale)
        except BaseException as exc:
            if not enable_attempted:
                raise
            stop = _stop_after_failure(executor, watchdog)
            raise ReplayExecutionError(exc, stop) from exc


class ReplayExecutionError(RuntimeError):
    """Replay failed; stop evidence is separate and may remain unconfirmed."""

    def __init__(self, original: BaseException, stop: dict[str, object]) -> None:
        self.original = original
        self.stop = stop
        super().__init__(f"simulated replay failed ({type(original).__name__}): {str(original)[:512]}; "
                         f"stop outcome: {stop['outcome']}")


def _require_replay_authority(executor: SDKAgentExecutor, arm) -> None:
    """Simulation replay authority is the broker profile plus exact replay policy.

    Simulation actions intentionally have no human physical lease. Physical action
    sessions retain their existing human-lease check; replay dispatch itself is
    still only available to the simulation backend.
    """
    if executor.simulation and executor.simulation_actions:
        executor._require_active()
        executor._assert_session_identity(arm)
    else:
        executor._require_authority(arm)


def _stop_after_failure(executor: SDKAgentExecutor, watchdog: BrokerWatchdog) -> dict[str, object]:
    """Use the same independent STOP path once, preserving unconfirmed outcomes."""
    try:
        watchdog.begin_stop()
    except PermissionError:
        status = watchdog.status
        result = status.get("stop_result")
        if isinstance(result, dict) and result.get("completed") and result.get("holding"):
            return {"outcome": "confirmed", "result": result}
        return {"outcome": "unconfirmed", "detail": status.get("stop_error") or "STOP pending"}
    try:
        result = executor.execute("stop", {})
    except BaseException as exc:
        watchdog.finish_stop(error=f"{type(exc).__name__}: {exc}")
        return {"outcome": "unconfirmed", "detail": f"{type(exc).__name__}: {exc}"}
    watchdog.finish_stop(result=result)
    if result.get("completed") and result.get("holding"):
        return {"outcome": "confirmed", "result": result}
    return {"outcome": "unconfirmed", "detail": "STOP did not confirm holding"}


def _execute_enabled(executor, watchdog, policy, snapshot, trajectory, arm, speed_scale):
    deadline_fired = threading.Event()
    final_lock = threading.Lock()
    finalized = False

    def on_deadline() -> None:
        with final_lock:
            if finalized:
                return
            deadline_fired.set()
        _stop_after_failure(executor, watchdog)

    timer = threading.Timer(policy.max_duration_s, on_deadline)
    timer.daemon = True
    watchdog.check()
    _require_replay_authority(executor, arm)
    timer.start()
    try:
        plan = arm.motion.plan_recorded_trajectory(
            trajectory, speed_scale=speed_scale, move_to_start=False, tcp=arm.active_tcp,
        )
        arm.motion._validate_samples(
            plan.command_samples,
            speed_limit=math.radians(executor.rates.joint_speed_deg_s),
            acceleration_limit=math.radians(executor.rates.joint_acceleration_deg_s2),
        )
        # A delayed planner must not submit after deadline/lease/STOP fencing.
        watchdog.check()
        _require_replay_authority(executor, arm)
        result = arm.play_trajectory(
            trajectory, speed_scale=speed_scale, move_to_start=False,
            gripper_speed_raw=executor.rates.gripper_speed_raw,
            gripper_acceleration_raw=executor.rates.gripper_acceleration_raw, wait=True,
        )
        with final_lock:
            if deadline_fired.is_set():
                raise RuntimeError("replay deadline expired; completion unconfirmed")
            watchdog.check()
            _require_replay_authority(executor, arm)
            if not result.accepted or not result.completed:
                raise RuntimeError("recorded replay did not complete")
            finalized = True
            return {"asset_id": snapshot.asset_id, "revision": snapshot.revision,
                    "mode": "simulation", "completion": asdict(result)}
    finally:
        with final_lock:
            finalized = True
        timer.cancel()
