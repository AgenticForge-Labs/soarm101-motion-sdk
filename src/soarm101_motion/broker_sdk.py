"""Preview of broker-owned persistent SDK dispatch (not hardware-enabled).

The broker is the sole HTTP authority. This executor consumes typed actions, not
CLI arguments or shell commands. Real robot execution is deliberately NOT enabled
by the production broker until local supervised STOP/workspace validation passes.
"""

from __future__ import annotations

import math
import threading
from dataclasses import asdict
from math import pi
from typing import TYPE_CHECKING, Callable, Mapping

import numpy as np

from soarm101_motion.agent_control import (
    AGENT_CAMERA_NAMES,
    AGENT_JOINT_MAX_DELTA_DEG,
    AGENT_POSE_PREFIX,
    AgentAuthorityStore,
    evaluate_agent_jog,
)
from soarm101_motion.arm import SOARM101
from soarm101_motion.control import relative_target_pose
from soarm101_motion.sdk_capabilities import SDK_CAPABILITIES
from soarm101_motion.workspace import WorkspaceCalibrationStore
from soarm101_motion.workstation import WorkstationProfileStore

if TYPE_CHECKING:
    from soarm101_motion.config import SOARM101Config


# HTTP fields are a strict, versioned interface. No host paths, arbitrary
# Python attributes, servo registers, rate overrides, or CLI flag forwarding.
_ACTION_FIELDS: Mapping[str, frozenset[str]] = {
    "capabilities": frozenset(),
    "state": frozenset(),
    "capture": frozenset({"camera"}),
    "go_pose": frozenset({"name"}),
    "joint": frozenset({"joint", "delta_deg"}),
    "jog": frozenset({"frame", "x_mm", "y_mm", "z_mm"}),
    "gripper": frozenset({"target"}),
    "sleep": frozenset(),
    "sleep_up": frozenset(),
    "stop": frozenset(),
}


class SDKAgentExecutor:
    """Single lazy SDK session owned by a trusted broker, with explicit dispatch.

    The production entrypoint permits ONLY simulated preview sessions. A
    fake-arm factory is supported exclusively for deterministic tests.
    Physical enabling requires a later, separately gated change.
    """

    def __init__(
        self,
        *,
        config: SOARM101Config,
        rates: object,
        simulation: bool = False,
        arm_factory: Callable[[], SOARM101] | None = None,
        authority_store: AgentAuthorityStore | None = None,
    ) -> None:
        if not simulation and arm_factory is None:
            raise ValueError(
                "direct-SDK physical sessions are disabled pending supervised "
                "STOP, ownership and workspace validation"
            )
        if config.auto_enable_torque:
            raise ValueError("broker SDK preview forbids automatic torque enable")
        self.config = config
        self.robot_id = config.robot_id
        self.rates = rates
        self.simulation = simulation
        self._arm_factory = arm_factory or (
            lambda: SOARM101.simulated(config=config, realtime=False)
        )
        self._authority = authority_store or AgentAuthorityStore()
        self._create_lock = threading.Lock()
        self._session: SOARM101 | None = None

    def _arm(self) -> SOARM101:
        with self._create_lock:
            if self._session is None:
                arm = self._arm_factory()
                if arm.config.robot_id != self.robot_id:
                    raise ValueError("SDK session robot ID differs from broker identity")
                arm.connect()  # Connection only. No automatic torque enable.
                self._session = arm
            return self._session

    def close(self) -> None:
        with self._create_lock:
            arm = self._session
            self._session = None
            if arm is not None:
                arm.disconnect()

    def _calibration_id(self, arm: SOARM101) -> str:
        identity = arm.calibration_id
        if identity:
            return identity
        if self.simulation:
            return "simulation"
        raise PermissionError("agent motion requires an active calibrated follower")

    def _require_authority(self, arm: SOARM101) -> dict[str, object]:
        lease = self._authority.require(
            robot_id=arm.config.robot_id,
            calibration_id=self._calibration_id(arm),
        )
        return lease.status_payload()

    @staticmethod
    def _number(request: Mapping[str, object], key: str, default: float = 0.0) -> float:
        raw = request.get(key, default)
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"{key} must be a finite number")
        value = float(raw)
        if not math.isfinite(value):
            raise ValueError(f"{key} must be a finite number")
        return value

    @staticmethod
    def _text(request: Mapping[str, object], key: str) -> str:
        raw = request.get(key)
        if not isinstance(raw, str) or not raw.strip():
            raise ValueError(f"{key} is required")
        return raw.strip()

    def _validate(self, action: str, request: Mapping[str, object]) -> None:
        fields = _ACTION_FIELDS.get(action)
        if fields is None:
            raise ValueError(f"unknown SDK broker action: {action}")
        unexpected = set(request) - fields
        if unexpected:
            raise ValueError(f"unknown {action} fields: {sorted(unexpected)!r}")

    def execute(self, action: str, request: Mapping[str, object]) -> dict[str, object]:
        """Typed action entrypoint; caller enforces pinned profile and concurrency.

        STOP may run in parallel with a motion call in order to signal the
        same MotionController. All other calls are serialized by the broker.
        """
        self._validate(action, request)
        if action == "capabilities":
            # Reuse the legacy pure projection until wrapper retirement in PR 3.
            from soarm101_motion.cli.main import _agent_capabilities_payload

            result = _agent_capabilities_payload(self.robot_id, config=self.config)
            # This opt-in preview must not advertise physical authority or
            # motion/camera tools that its HTTP boundary explicitly denies.
            result["actions"] = {"state": "read_only"}
            result["poses"] = []
            result["cameras"] = []
            result["world_directions"] = {
                "available": False,
                "reason": "simulation preview has no measured physical directions",
            }
            result["authority"] = {
                "armed": False,
                "robot_id": None,
                "calibration_id": None,
                "issued_at": None,
                "expires_at": None,
                "remaining_seconds": 0.0,
            }
            return result
        if action == "capture":
            name = self._text(request, "camera")
            if name not in AGENT_CAMERA_NAMES:
                raise PermissionError("agent camera is not permitted")
            profile = WorkstationProfileStore().load()
            if name not in profile.cameras:
                raise KeyError(f"agent camera {name!r} is not configured")
            return SDK_CAPABILITIES.dispatch(
                "capture_camera", None, {"name": name},
                camera_settings=profile.camera(name),
            )
        if action == "stop":
            # DO NOT wait for the motion lock or silently enable torque here.
            # A new session cannot be opened by STOP during an active command.
            arm = self._session
            if arm is None:
                raise RuntimeError("no connected SDK session to stop")
            SDK_CAPABILITIES.dispatch("stop", arm, {})
            joints = dict(arm.get_joint_positions().positions)
            return {
                "accepted": True, "completed": True, "action": "stop",
                "holding": bool(arm.get_state().torque_enabled),
                "joint_positions_rad": joints,
            }

        arm = self._arm()
        if action == "state":
            state = SDK_CAPABILITIES.dispatch("read_pose", arm, {})
            calibration_id = self._calibration_id(arm)
            height: float | None = None
            error: str | None = None
            if calibration_id != "simulation":
                try:
                    workspace = WorkspaceCalibrationStore(self.robot_id).load()
                    if workspace.arm_calibration_id != calibration_id:
                        raise ValueError("workspace calibration does not match active motor calibration")
                    physical = workspace.physical_position_from_model(
                        np.asarray(state["tcp_xyz_mm"], dtype=float) / 1000.0
                    )
                    height = float(physical[2] * 1000.0)
                except Exception as exc:
                    error = str(exc)
            return {
                "authority": {
                    "armed": False, "robot_id": None,
                    "calibration_id": None, "issued_at": None,
                    "expires_at": None, "remaining_seconds": 0.0,
                },
                "robot_id": self.robot_id,
                "calibration_id": calibration_id,
                **state,
                "gripper": float(arm.tool.get_position()),
                "physical_height_mm": height,
                "workspace_height_error": error,
            }

        # Physical state changes always re-evaluate the current human lease.
        # No preflight on a cached state or static profile grants motion.
        authority = self._require_authority(arm)
        if action == "go_pose":
            name = self._text(request, "name")
            if not name.startswith(AGENT_POSE_PREFIX):
                raise PermissionError("agent pose name must begin with 'agent_'")
            # Check provenance before enabling torque; replay checks it again.
            SDK_CAPABILITIES.dispatch(
                "validate_saved_pose", arm,
                {"robot_id": self.robot_id, "name": name},
            )
            arm.enable()
            moved, gripper = SDK_CAPABILITIES.dispatch(
                "replay_saved_pose", arm,
                {"robot_id": self.robot_id, "name": name},
            )
            if not (moved.accepted and moved.completed
                    and gripper.accepted and gripper.completed):
                raise RuntimeError("saved pose playback did not complete successfully")
            return {
                "accepted": True, "completed": True, "action": "go_pose",
                "pose": name, "holding": True, "authority": authority,
                "arm": asdict(moved), "gripper": asdict(gripper),
            }
        if action == "joint":
            joint = self._text(request, "joint")
            delta = self._number(request, "delta_deg")
            if abs(delta) <= 1e-9 or abs(delta) > AGENT_JOINT_MAX_DELTA_DEG:
                raise PermissionError("agent joint delta must be nonzero and within bounded policy")
            arguments = {
                "joint": joint, "delta_rad": delta * pi / 180.0,
                "speed_rad_s": float(getattr(self.rates, "joint_speed_deg_s")) * pi / 180.0,
                "acceleration_rad_s2": (
                    float(getattr(self.rates, "joint_acceleration_deg_s2")) * pi / 180.0
                ),
            }
            SDK_CAPABILITIES.get("jog_joint").validate(arguments)
            before = dict(arm.get_joint_positions().positions)
            arm.enable()
            result = SDK_CAPABILITIES.dispatch("jog_joint", arm, arguments)
            arm.hold()
            if not result.accepted or not result.completed:
                raise RuntimeError("joint motion did not complete successfully")
            after = dict(arm.get_joint_positions().positions)
            return {
                "accepted": result.accepted, "completed": result.completed,
                "action": "joint", "joint": joint, "delta_deg": delta,
                "before_deg": before[joint] * 180.0 / pi,
                "after_deg": after[joint] * 180.0 / pi,
                "holding": True, "authority": authority, "message": result.message,
            }
        if action == "jog":
            # Simulation has no measured physical workspace. Do not pretend it
            # proves physical-clearance policy, even if model FK succeeds.
            if self.simulation:
                raise PermissionError(
                    "agent jog requires a measured workspace calibration and "
                    "is not enabled in simulation"
                )
            frame = request.get("frame", "world")
            if frame not in ("world", "tool"):
                raise ValueError("frame must be 'world' or 'tool'")
            delta = [self._number(request, axis) for axis in ("x_mm", "y_mm", "z_mm")]
            current = arm.get_position()
            target = relative_target_pose(
                current, translation_m=np.asarray(delta, dtype=float) / 1000.0,
                frame=frame,
            )
            decision = evaluate_agent_jog(
                WorkspaceCalibrationStore(self.robot_id).load(),
                active_calibration_id=self._calibration_id(arm),
                current_model_position_m=current.position,
                delta_model_m=target.position - current.position,
            )
            x, y, z, roll, pitch, yaw = target.xyz_rpy()
            arm.enable()
            result = SDK_CAPABILITIES.dispatch("move_linear", arm, {
                "target_xyz_mm": [x * 1000.0, y * 1000.0, z * 1000.0],
                "target_rpy_deg": [roll * 180 / pi, pitch * 180 / pi, yaw * 180 / pi],
                "orientation_mode": "compatible",
                "speed_mm_s": getattr(self.rates, "cartesian_speed_mm_s"),
                "acceleration_mm_s2": getattr(self.rates, "cartesian_acceleration_mm_s2"),
            })
            arm.hold()
            if not result.accepted or not result.completed:
                raise RuntimeError("Cartesian jog did not complete successfully")
            return {
                "accepted": result.accepted, "completed": result.completed,
                "action": "jog", "holding": True, "authority": authority,
                "policy": decision.to_payload(),
                "final_positions": result.final_positions, "message": result.message,
            }
        if action == "gripper":
            target = self._text(request, "target")
            if target not in ("open", "close"):
                raise ValueError("gripper target must be 'open' or 'close'")
            closed = float(arm.get_sleep_gripper_position())
            position = closed if target == "close" else 1.0 - closed
            arm.enable()
            result = SDK_CAPABILITIES.dispatch(
                "move_gripper", arm, {"position": position}
            )
            arm.hold()
            if not result.accepted or not result.completed:
                raise RuntimeError("gripper command did not complete successfully")
            return {
                "accepted": result.accepted, "completed": result.completed,
                "action": "gripper", "target": target,
                "normalized_target": position, "holding": True,
                "authority": authority, "final_positions": result.final_positions,
                "message": result.message,
            }
        if action in ("sleep", "sleep_up"):
            arm.enable()
            result = SDK_CAPABILITIES.dispatch(
                action, arm, {
                    "speed_rad_s": 8.0 * pi / 180.0,
                    "acceleration_rad_s2": 25.0 * pi / 180.0,
                }
            )
            arm.hold()
            if not result.accepted or not result.completed:
                raise RuntimeError("Sleep operation did not complete successfully")
            return {**asdict(result), "action": action, "holding": True,
                    "authority": authority}
        raise AssertionError(f"missing SDK broker action handler for {action}")
