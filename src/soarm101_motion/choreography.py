"""Trusted, content-pinned, SDK-native deterministic joint choreography.

This is an SDK capability, not an agent route or a Puppeteer behavior library.
Only an already-connected, separately supervised trusted-host SDK executor may
run it. Motion, calibration, planner, workspace guards and STOP remain in the
existing SOARM101/controller/broker stack. The physical arm must already be
human-held and is NOT automatically torque-enabled.

A multi-keyframe program is one bounded, fully preflighted trajectory. It does
not loop, name emotions, interpret speech, invoke tools, or accept raw servos.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import threading
from dataclasses import dataclass
from math import radians
from typing import TYPE_CHECKING

import numpy as np

from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.hardware import SimulationBackend
from soarm101_motion.safety import validate_workspace_path_from_measured_start
from soarm101_motion.simulated_replay import (
    _require_replay_authority,
    _stop_after_failure,
)
from soarm101_motion.trajectories import Trajectory

# Joint targets have no operator-authored timestamps, so reserve explicit time
# for SDK completion reporting and host scheduling before the hard policy timer.
_JOINT_TARGET_DEADLINE_MARGIN_S = 1.0

if TYPE_CHECKING:
    from soarm101_motion.arm import SOARM101
    from soarm101_motion.broker_sdk import SDKAgentExecutor
    from soarm101_motion.broker_watchdog import BrokerWatchdog


def _require_choreography_authority(executor: SDKAgentExecutor, arm) -> None:
    """Exact trusted-host programs use the selected SDK supervision mode.

    The operator-only mode has no agent lease. This exception belongs only to
    pinned choreography/targets, never the broader recording/agent interfaces.
    """
    if getattr(executor, "operator_choreography_only", False):
        if not executor.physical_actions or executor.simulation:
            raise PermissionError("operator choreography requires its physical SDK session")
        executor.validate_supervision()
        executor._assert_session_identity(arm)
    else:
        _require_replay_authority(executor, arm)


_REV = re.compile(r"^sha256:[0-9a-f]{64}$")
_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$")


class ChoreographyExecutionError(RuntimeError):
    """A guarded choreography failed; STOP/HOLD evidence is kept separately."""

    def __init__(self, original: BaseException, stop: dict[str, object]) -> None:
        self.original = original
        self.stop = stop
        outcome = str(stop.get("outcome", "unconfirmed"))
        super().__init__(
            f"choreography execution failed ({type(original).__name__}): "
            f"{str(original)[:512]}; STOP/HOLD outcome: {outcome}"
        )


@dataclass(frozen=True)
class JointFrame:
    """Absolute joint target in radians at a relative monotonic timestamp."""

    at_s: float
    joints_rad: tuple[float, ...]

    def __post_init__(self) -> None:
        if (isinstance(self.at_s, bool) or not isinstance(self.at_s, (int, float))
                or not math.isfinite(self.at_s) or self.at_s < 0):
            raise ValueError("choreography frame time must be finite and nonnegative")
        if (not isinstance(self.joints_rad, tuple)
                or len(self.joints_rad) != len(ARM_JOINTS)
                or any(isinstance(x, bool) or not isinstance(x, (int, float))
                       or not math.isfinite(x) for x in self.joints_rad)):
            raise ValueError("frame requires all five finite arm joints in canonical order")


@dataclass(frozen=True)
class JointChoreography:
    """Immutable operator-reviewed segment, with no gripper or personality."""

    name: str
    robot_id: str
    calibration_id: str
    frames: tuple[JointFrame, ...]
    revision: str

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not _ID.fullmatch(self.name):
            raise ValueError("invalid choreography name")
        if any(not isinstance(x, str) or not x or len(x) > 256
               for x in (self.robot_id, self.calibration_id)):
            raise ValueError("robot and calibration identity required")
        if (not isinstance(self.frames, tuple) or not 2 <= len(self.frames) <= 256
                or any(not isinstance(frame, JointFrame) for frame in self.frames)):
            raise ValueError("choreography requires 2..256 joint frames")
        if self.frames[0].at_s != 0 or self.frames[-1].at_s > 30.0:
            raise ValueError("choreography must begin at zero and finish within 30s")
        if any(a.at_s >= b.at_s for a, b in zip(self.frames, self.frames[1:])):
            raise ValueError("choreography frame times must increase strictly")
        if not isinstance(self.revision, str) or not _REV.fullmatch(self.revision):
            raise ValueError("choreography requires exact SHA-256 revision")
        if self.revision != self.content_revision():
            raise ValueError("choreography content differs from pinned SHA-256 revision")

    def content_revision(self) -> str:
        payload = {
            "schema": "soarm101.choreography.v1",
            "name": self.name,
            "robot_id": self.robot_id,
            "calibration_id": self.calibration_id,
            "frames": [
                {"at_s": float(frame.at_s), "joints_rad": [float(x) for x in frame.joints_rad]}
                for frame in self.frames
            ],
        }
        return "sha256:" + hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        ).hexdigest()

    @classmethod
    def author(
        cls, name: str, robot_id: str, calibration_id: str,
        frames: tuple[JointFrame, ...],
    ) -> JointChoreography:
        provisional = cls.__new__(cls)
        object.__setattr__(provisional, "name", name)
        object.__setattr__(provisional, "robot_id", robot_id)
        object.__setattr__(provisional, "calibration_id", calibration_id)
        object.__setattr__(provisional, "frames", frames)
        return cls(name, robot_id, calibration_id, frames, provisional.content_revision())


@dataclass(frozen=True)
class JointTarget:
    """Immutable operator-reviewed absolute five-joint target asset."""

    name: str
    robot_id: str
    calibration_id: str
    joints_rad: tuple[float, ...]
    revision: str

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not _ID.fullmatch(self.name):
            raise ValueError("invalid joint target name")
        if any(not isinstance(x, str) or not x or len(x) > 256
               for x in (self.robot_id, self.calibration_id)):
            raise ValueError("robot and calibration identity required")
        if (not isinstance(self.joints_rad, tuple) or len(self.joints_rad) != len(ARM_JOINTS)
                or any(isinstance(x, bool) or not isinstance(x, (int, float))
                       or not math.isfinite(x) for x in self.joints_rad)):
            raise ValueError("target requires all five finite arm joints in canonical order")
        if not isinstance(self.revision, str) or not _REV.fullmatch(self.revision):
            raise ValueError("joint target requires exact SHA-256 revision")
        if self.revision != self.content_revision():
            raise ValueError("joint target content differs from pinned SHA-256 revision")

    def content_revision(self) -> str:
        payload = {
            "schema": "soarm101.joint_target.v1", "name": self.name,
            "robot_id": self.robot_id, "calibration_id": self.calibration_id,
            "joints_rad": [float(x) for x in self.joints_rad],
        }
        return "sha256:" + hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        ).hexdigest()

    @classmethod
    def author(cls, name: str, robot_id: str, calibration_id: str,
               joints_rad: tuple[float, ...]) -> JointTarget:
        payload = {"schema": "soarm101.joint_target.v1", "name": name,
                   "robot_id": robot_id, "calibration_id": calibration_id,
                   "joints_rad": [float(x) for x in joints_rad]}
        revision = "sha256:" + hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        ).hexdigest()
        return cls(name, robot_id, calibration_id, joints_rad, revision)


@dataclass(frozen=True)
class ChoreographyPolicy:
    """Trusted host's reviewed exact segment allowlist and rate ceilings."""

    robot_id: str
    calibration_id: str
    allowed_revisions: tuple[str, ...]
    max_duration_s: float = 30.0
    max_samples: int = 256
    max_start_error_rad: float = 0.02
    max_speed_deg_s: float = 8.0
    max_acceleration_deg_s2: float = 25.0
    allow_physical_trial: bool = False
    target_timing: str = "deadline"

    def __post_init__(self) -> None:
        if self.target_timing not in {"deadline", "native"}:
            raise ValueError("target timing must be deadline or native")
        if (not self.robot_id or not self.calibration_id
                or not isinstance(self.allowed_revisions, tuple)
                or not 1 <= len(self.allowed_revisions) <= 256
                or any(not isinstance(item, str) or not _REV.fullmatch(item)
                       for item in self.allowed_revisions)
                or len(set(self.allowed_revisions)) != len(self.allowed_revisions)):
            raise ValueError("invalid choreography policy identity or immutable allowlist")
        bounds = (
            (self.max_duration_s, 30.0),
            (self.max_start_error_rad, 0.05),
            (self.max_speed_deg_s, 100.0),
            (self.max_acceleration_deg_s2, 1000.0),
        )
        if any(isinstance(x, bool) or not isinstance(x, (int, float))
               or not math.isfinite(x) or not 0 < x <= maximum for x, maximum in bounds):
            raise ValueError("invalid trusted choreography bound")
        if type(self.max_samples) is not int or not 2 <= self.max_samples <= 256:
            raise ValueError("invalid choreography sample ceiling")
        if type(self.allow_physical_trial) is not bool:
            raise ValueError("physical trial permission must be explicit boolean")

    def require(self, pattern: JointChoreography) -> None:
        # Rebuild typed data to catch a caller replacing frozen dataclass fields.
        fresh = JointChoreography(
            pattern.name, pattern.robot_id, pattern.calibration_id,
            tuple(JointFrame(f.at_s, tuple(f.joints_rad)) for f in pattern.frames),
            pattern.revision,
        )
        if (fresh.robot_id != self.robot_id
                or fresh.calibration_id != self.calibration_id
                or fresh.revision not in self.allowed_revisions):
            raise PermissionError("choreography is not pinned for this robot and calibration")
        if fresh.frames[-1].at_s > self.max_duration_s or len(fresh.frames) > self.max_samples:
            raise PermissionError("choreography exceeds reviewed duration/frame ceiling")


@dataclass(frozen=True)
class ChoreographyPreflight:
    name: str
    revision: str
    robot_id: str
    calibration_id: str
    duration_s: float
    sample_count: int
    start_error_rad: float
    gripper_position: float
    trajectory: Trajectory

    def report(self) -> dict[str, object]:
        return {
            "name": self.name, "revision": self.revision,
            "robot_id": self.robot_id, "calibration_id": self.calibration_id,
            "duration_s": self.duration_s, "sample_count": self.sample_count,
            "start_error_rad": self.start_error_rad,
            "gripper": "held_at_measured_start",
            "status": "preflight_only_not_motion_authorization",
        }


def preflight_choreography(
    arm: SOARM101, pattern: JointChoreography, policy: ChoreographyPolicy,
    *, speed_scale: float = 1.0,
) -> ChoreographyPreflight:
    """Validate full path via the existing SDK planner; never enable torque."""
    policy.require(pattern)
    if isinstance(speed_scale, bool) or not isinstance(speed_scale, (float, int)) or (
        not math.isfinite(speed_scale) or not 0 < speed_scale <= 1
    ):
        raise ValueError("speed scale must be finite and within (0, 1]")
    if pattern.robot_id != arm.config.robot_id:
        raise PermissionError("active SDK robot does not match authored program")
    simulation = isinstance(arm.backend, SimulationBackend)
    calibration = "simulation" if simulation else arm.calibration_id
    if calibration != policy.calibration_id:
        raise PermissionError("active SDK calibration does not match reviewed program")
    if not simulation and not arm.config.enable_workspace_checks:
        raise PermissionError("physical choreography requires enabled workspace validation")
    state = arm.get_state()
    if not state.connected or state.faulted or state.moving:
        raise PermissionError("healthy, connected and stationary SDK state required")
    if not simulation and not state.torque_enabled:
        raise PermissionError("physical choreography requires existing human torque HOLD")
    if arm.tool.is_moving:
        raise PermissionError("tool is already moving; choreography never commands gripper")
    actual = arm.get_joint_positions().positions
    entry = dict(zip(ARM_JOINTS, pattern.frames[0].joints_rad, strict=True))
    start_error = max(abs(float(actual[name]) - entry[name]) for name in ARM_JOINTS)
    if start_error > policy.max_start_error_rad:
        raise PermissionError("measured start differs from approved choreography entry")
    gripper = float(arm.tool.get_position())
    if not math.isfinite(gripper) or not 0 <= gripper <= 1:
        raise ValueError("invalid measured gripper state")
    trajectory = Trajectory(
        timestamps_s=np.array([f.at_s for f in pattern.frames], dtype=float),
        joints_rad=np.array([f.joints_rad for f in pattern.frames], dtype=float),
        gripper=np.full(len(pattern.frames), gripper, dtype=float),
        metadata={
            "source_robot_id": pattern.robot_id,
            "source_calibration_id": pattern.calibration_id,
            "choreography_revision": pattern.revision,
        },
    )
    # Pure native SDK validation must work before simulated torque enable.
    # plan_recorded_trajectory() requires torque, so use the same SDK
    # retiming/resampling, joint validators and workspace validator here.
    # The guarded SDK executor performs its own full preflight again at play.
    samples = trajectory.retime(speed_scale).resample(
        arm.config.command_frequency_hz
    ).joint_mappings()
    # The arm is wherever it is: a pose resting just past a software limit (an
    # arm that went limp on its stop) is a valid start, and the path may only
    # reach as far out as that start, never further.
    arm.motion._validate_samples(
        samples,
        speed_limit=min(arm.config.max_joint_speed, radians(policy.max_speed_deg_s)),
        acceleration_limit=min(
            arm.config.max_joint_acceleration, radians(policy.max_acceleration_deg_s2)
        ),
        limits=arm.motion._limits_for_present(actual),
    )
    if arm.config.enable_workspace_checks:
        validate_workspace_path_from_measured_start(
            arm.model, samples, tcp=arm.active_tcp, **arm._workspace_kwargs()
        )
    duration = trajectory.duration_s / speed_scale
    if duration > policy.max_duration_s:
        raise PermissionError("retimed choreography exceeds reviewed deadline")
    return ChoreographyPreflight(
        pattern.name, pattern.revision, pattern.robot_id, pattern.calibration_id,
        duration, len(samples), start_error, gripper, trajectory,
    )


def preflight_joint_target(
    arm: SOARM101, target: JointTarget, policy: ChoreographyPolicy,
) -> ChoreographyPreflight:
    """Validate a pinned endpoint from measured joints through the native planner."""
    fresh = JointTarget(target.name, target.robot_id, target.calibration_id,
                        tuple(target.joints_rad), target.revision)
    if (fresh.robot_id != policy.robot_id or fresh.calibration_id != policy.calibration_id
            or fresh.revision not in policy.allowed_revisions):
        raise PermissionError("joint target is not pinned for this robot and calibration")
    if policy.max_duration_s > 30:
        raise PermissionError("joint target duration exceeds SDK ceiling")
    motion_duration = policy.max_duration_s - _JOINT_TARGET_DEADLINE_MARGIN_S
    if motion_duration <= 0:
        raise PermissionError(
            "joint target deadline must leave at least 1s for completion margin"
        )
    actual = arm.get_joint_positions().positions
    start = tuple(float(actual[name]) for name in ARM_JOINTS)
    if policy.target_timing == "native":
        # Reuse the SDK's actual joint planner. The deadline is a ceiling,
        # rather than the requested duration of every endpoint transition.
        baseline = JointChoreography.author("measured_target_start", fresh.robot_id,
            fresh.calibration_id, (JointFrame(0.0, start), JointFrame(.2, start)))
        baseline_policy = ChoreographyPolicy(policy.robot_id, policy.calibration_id,
            (baseline.revision,), max_duration_s=policy.max_duration_s,
            max_samples=policy.max_samples, max_start_error_rad=policy.max_start_error_rad,
            max_speed_deg_s=policy.max_speed_deg_s,
            max_acceleration_deg_s2=policy.max_acceleration_deg_s2,
            allow_physical_trial=policy.allow_physical_trial)
        checked = preflight_choreography(arm, baseline, baseline_policy)
        plan = arm.motion._plan_joint_motion(actual, dict(zip(ARM_JOINTS, fresh.joints_rad, strict=True)),
            speed=radians(policy.max_speed_deg_s), acceleration=radians(policy.max_acceleration_deg_s2))
        duration = max(plan.duration_s, 1.0 / arm.config.command_frequency_hz)
        if duration + arm.config.motion_completion_timeout_s > policy.max_duration_s:
            raise PermissionError("native target plan and completion margin exceed reviewed deadline")
        trajectory = Trajectory(
            timestamps_s=np.linspace(0., duration, len(plan.command_samples)),
            joints_rad=np.array([[row[n] for n in ARM_JOINTS] for row in plan.command_samples]),
            gripper=np.full(len(plan.command_samples), checked.gripper_position),
            metadata={"source_robot_id": fresh.robot_id, "source_calibration_id": fresh.calibration_id,
                      "joint_target_revision": fresh.revision})
        samples = trajectory.resample(arm.config.command_frequency_hz).joint_mappings()
        arm.motion._validate_samples(samples, speed_limit=radians(policy.max_speed_deg_s),
                                     acceleration_limit=radians(policy.max_acceleration_deg_s2),
                                     limits=arm.motion._limits_for_present(actual))
        if arm.config.enable_workspace_checks:
            validate_workspace_path_from_measured_start(
            arm.model, samples, tcp=arm.active_tcp, **arm._workspace_kwargs()
        )
        return ChoreographyPreflight(fresh.name, fresh.revision, fresh.robot_id, fresh.calibration_id,
            duration, len(samples), checked.start_error_rad, checked.gripper_position, trajectory)
    path = JointChoreography.author(
        "measured_target_path", fresh.robot_id, fresh.calibration_id,
        (JointFrame(0.0, start), JointFrame(motion_duration, fresh.joints_rad)),
    )
    # The transient path is created only inside the SDK from measured state;
    # the operator pin remains the immutable endpoint asset revision.
    internal_policy = ChoreographyPolicy(
        policy.robot_id, policy.calibration_id, (path.revision,),
        max_duration_s=policy.max_duration_s, max_samples=policy.max_samples,
        max_start_error_rad=policy.max_start_error_rad,
        max_speed_deg_s=policy.max_speed_deg_s,
        max_acceleration_deg_s2=policy.max_acceleration_deg_s2,
        allow_physical_trial=policy.allow_physical_trial,
    )
    result = preflight_choreography(arm, path, internal_policy)
    trajectory = result.trajectory
    trajectory.metadata["joint_target_revision"] = fresh.revision
    return ChoreographyPreflight(
        fresh.name, fresh.revision, result.robot_id, result.calibration_id,
        result.duration_s, result.sample_count, result.start_error_rad,
        result.gripper_position, trajectory,
    )


def execute_trusted_joint_target(
    executor: SDKAgentExecutor, watchdog: BrokerWatchdog,
    policy: ChoreographyPolicy, target: JointTarget,
) -> dict[str, object]:
    """Execute one exact pinned endpoint using the existing trusted SDK authority.

    The measured start and complete planned path are resolved while holding the
    existing dispatch lock. Physical torque is never enabled here.
    """
    if (getattr(executor.motion_guard, "__self__", None) is not watchdog
            or watchdog._worker is None or not watchdog._worker.is_alive()):
        raise PermissionError("active bound host watchdog required")
    if not (executor.simulation and executor.simulation_actions):
        if not (executor.physical_actions and policy.allow_physical_trial):
            raise PermissionError("physical joint target is not operator-qualified")
    if policy.robot_id != executor.robot_id:
        raise PermissionError("policy differs from active SDK identity")
    if (policy.max_speed_deg_s > executor.rates.joint_speed_deg_s
            or policy.max_acceleration_deg_s2 > executor.rates.joint_acceleration_deg_s2):
        raise PermissionError("joint target cannot widen trusted host motion rates")
    with executor._dispatch_lock:
        executor._require_active()
        arm = executor._session
        if arm is None:
            raise PermissionError("preexisting connected SDK session required")
        executor._assert_session_identity(arm)
        watchdog.check()
        _require_choreography_authority(executor, arm)
        start = preflight_joint_target(arm, target, policy)
        watchdog.check()
        _require_choreography_authority(executor, arm)
        sim = isinstance(arm.backend, SimulationBackend)
        if not sim and not arm.get_state().torque_enabled:
            raise PermissionError("operator must already enable physical torque HOLD")
        attempted = False
        deadline_tripped = threading.Event()
        finalize_lock = threading.Lock()
        finished = False

        def deadline() -> None:
            nonlocal finished
            with finalize_lock:
                if finished:
                    return
                deadline_tripped.set()
            _stop_after_failure(executor, watchdog)

        timer = threading.Timer(policy.max_duration_s, deadline)
        timer.daemon = True
        try:
            with watchdog._lock:
                watchdog.check()
                _require_choreography_authority(executor, arm)
                attempted = True
                if sim:
                    arm.enable()
                else:
                    executor._motion_action_attempted = True
                    executor._verified_physical_stop = False
            timer.start()
            result = arm.play_trajectory(
                start.trajectory, move_to_start=False, wait=True, joint_only=True,
            )
            with finalize_lock:
                if deadline_tripped.is_set():
                    raise RuntimeError("joint target deadline expired; completion unknown")
                watchdog.check()
                _require_choreography_authority(executor, arm)
                if not result.accepted or not result.completed:
                    raise RuntimeError("SDK joint target execution was not completed")
                final = result.final_positions
                if (not set(ARM_JOINTS) <= set(final)
                        or any(not math.isfinite(final[joint]) for joint in ARM_JOINTS)
                        or max(abs(final[joint] - target.joints_rad[index])
                               for index, joint in enumerate(ARM_JOINTS)) > policy.max_start_error_rad):
                    raise RuntimeError("SDK target completion is outside the reviewed choreography entry tolerance")
                finished = True
            return {
                "mode": "simulation" if sim else "operator_attended_physical_trial",
                "revision": start.revision, "name": start.name,
                "robot_id": start.robot_id, "calibration_id": start.calibration_id,
                "session_id": executor.session_id,
                "status": "sdk_execution_completed_physical_settling_not_certified",
                "preflight": start.report(),
                "final_joint_positions_rad": dict(result.final_positions),
            }
        except BaseException as exc:
            if not attempted:
                raise
            raise ChoreographyExecutionError(
                exc, _stop_after_failure(executor, watchdog),
            ) from exc
        finally:
            with finalize_lock:
                finished = True
            timer.cancel()


def execute_trusted_choreography(
    executor: SDKAgentExecutor, watchdog: BrokerWatchdog,
    policy: ChoreographyPolicy, pattern: JointChoreography,
    *, speed_scale: float = 1.0,
) -> dict[str, object]:
    """Run one reviewed segment under the existing trusted SDK broker authority.

    NOT registered in agent _ACTION_FIELDS, MCP or robotctl. For physical trial,
    human lease + preexisting torque + calibrated workspace + exact opt-in and
    independently running host watchdog are mandatory. Simulation alone is not
    evidence of physical STOP/HOLD or safe workspace.
    """
    if (getattr(executor.motion_guard, "__self__", None) is not watchdog
            or watchdog._worker is None or not watchdog._worker.is_alive()):
        raise PermissionError("active bound host watchdog required")
    if not (executor.simulation and executor.simulation_actions):
        if not (executor.physical_actions and policy.allow_physical_trial):
            raise PermissionError("physical choreography is not operator-qualified")
    if policy.robot_id != executor.robot_id:
        raise PermissionError("policy differs from active SDK identity")
    if (policy.max_speed_deg_s > executor.rates.joint_speed_deg_s
            or policy.max_acceleration_deg_s2 > executor.rates.joint_acceleration_deg_s2):
        raise PermissionError("choreography cannot widen trusted host motion rates")

    with executor._dispatch_lock:
        executor._require_active()
        arm = executor._session
        if arm is None:
            raise PermissionError("preexisting connected SDK session required")
        executor._assert_session_identity(arm)
        watchdog.check()
        _require_choreography_authority(executor, arm)
        start = preflight_choreography(arm, pattern, policy, speed_scale=speed_scale)
        watchdog.check()
        _require_choreography_authority(executor, arm)
        # Only simulator setup may enable torque; physical operator owns HOLD.
        sim = isinstance(arm.backend, SimulationBackend)
        arm_state = arm.get_state()
        if not sim and not arm_state.torque_enabled:
            raise PermissionError("operator must already enable physical torque HOLD")
        attempted = False
        deadline_tripped = threading.Event()
        finalize_lock = threading.Lock()
        finished = False

        def deadline() -> None:
            nonlocal finished
            with finalize_lock:
                if finished:
                    return
                deadline_tripped.set()
            _stop_after_failure(executor, watchdog)

        timer = threading.Timer(policy.max_duration_s, deadline)
        timer.daemon = True
        try:
            with watchdog._lock:
                watchdog.check()
                _require_choreography_authority(executor, arm)
                # A partially failed torque change is still our cleanup duty.
                attempted = True
                if sim:
                    arm.enable()
                else:
                    executor._motion_action_attempted = True
                    executor._verified_physical_stop = False
            timer.start()
            # Public SDK facade performs a fresh validation/preflight and uses
            # the canonical motion controller; no raw USB/servo writes.
            result = arm.play_trajectory(
                start.trajectory, speed_scale=speed_scale, move_to_start=False,
                wait=True, joint_only=True,
            )
            with finalize_lock:
                if deadline_tripped.is_set():
                    raise RuntimeError("choreography deadline expired; completion unknown")
                watchdog.check()
                _require_choreography_authority(executor, arm)
                if not result.accepted or not result.completed:
                    raise RuntimeError("SDK choreography execution was not completed")
                finished = True
            return {
                "mode": "simulation" if sim else "operator_attended_physical_trial",
                "revision": start.revision, "name": start.name,
                "robot_id": start.robot_id, "calibration_id": start.calibration_id,
                "session_id": executor.session_id,
                "status": "sdk_execution_completed_physical_settling_not_certified",
                "preflight": start.report(),
                "final_joint_positions_rad": dict(result.final_positions),
            }
        except BaseException as exc:
            if not attempted:
                raise
            # Preserve unconfirmed outcomes. The existing STOP path also
            # invalidates further motion and revokes physical authority.
            stop = _stop_after_failure(executor, watchdog)
            raise ChoreographyExecutionError(exc, stop) from exc
        finally:
            with finalize_lock:
                finished = True
            timer.cancel()
