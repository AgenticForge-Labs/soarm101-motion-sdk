"""Persistent SDK sessions for simulation and explicitly attended physical use.

The default physical broker retains its bounded CLI path. Experimental native
physical joint actions require a pinned policy, existing human HOLD/authority
and private host-supervisor watchdog. Hardware qualification remains pending.
"""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import asdict
from pathlib import Path
from math import pi
from typing import TYPE_CHECKING, Callable, Mapping

import numpy as np

from soarm101_motion.agent_control import (
    AGENT_CAMERA_NAMES,
    AGENT_JOINT_MAX_DELTA_DEG,
    AGENT_POSE_PREFIX,
    AgentAuthority,
    AgentAuthorityStore,
    evaluate_agent_jog,
)
from soarm101_motion.arm import SOARM101
from soarm101_motion.control import relative_target_pose
from soarm101_motion.exceptions import (
    CommunicationError, HardwareFaultError, MotionTimeoutError, RobotConnectionError,
)
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.sdk_capabilities import SDK_CAPABILITIES
from soarm101_motion.workspace import WorkspaceCalibrationStore
from soarm101_motion.workstation import WorkstationProfileStore

if TYPE_CHECKING:
    from soarm101_motion.config import SOARM101Config
    from soarm101_motion.broker import AgentMotionRates


# An acknowledged software STOP must not mean merely that a goal write
# succeeded. A physical arm can continue moving after the servo was sent
# a present-position latch (observed 18.4° elbow travel within one second
# on 2026-10-11). Verify bounded post-latch encoder stability before ACK.
# These conservative detection bounds do NOT certify physical braking.
STOP_STABILITY_SAMPLE_DELAYS_S = (0.25, 0.25, 0.5, 0.5)
STOP_MAX_POST_LATCH_DRIFT_DEG = 1.0


# HTTP fields are a strict, versioned interface. No host paths, arbitrary
# Python attributes, servo registers, rate overrides, or CLI flag forwarding.
_ACTION_FIELDS: Mapping[str, frozenset[str]] = {
    "capabilities": frozenset(),
    "state": frozenset(),
    "capture": frozenset({"camera"}),
    "go_pose": frozenset({"name", "speed_deg_s", "acceleration_deg_s2",
                           "gripper_speed_raw", "gripper_acceleration_raw",
                           "joint_only"}),
    "joint": frozenset({"joint", "delta_deg", "speed_deg_s", "acceleration_deg_s2"}),
    "jog": frozenset({"frame", "x_mm", "y_mm", "z_mm",
                        "speed_mm_s", "acceleration_mm_s2"}),
    "gripper": frozenset({"target", "gripper_speed_raw", "gripper_acceleration_raw"}),
    "sleep": frozenset({"speed_deg_s", "acceleration_deg_s2",
                          "gripper_speed_raw", "gripper_acceleration_raw",
                          "joint_only"}),
    "sleep_up": frozenset({"speed_deg_s", "acceleration_deg_s2",
                             "gripper_speed_raw", "gripper_acceleration_raw"}),
    "stop": frozenset(),
}


class SDKAgentExecutor:
    """Single lazy SDK session owned by a trusted broker, with explicit dispatch.

    The default physical broker is CLI-backed. Native physical joint actions
    require explicit trusted-host provisioning and watchdog supervision; the
    separate attended qualification trial retains its fixed action protocol.
    """

    def __init__(
        self,
        *,
        config: SOARM101Config,
        rates: AgentMotionRates,
        simulation: bool = False,
        arm_factory: Callable[[], SOARM101] | None = None,
        authority_store: AgentAuthorityStore | None = None,
        physical_trial: bool = False,
        simulation_actions: bool = False,
        physical_actions: bool = False,
        trace_path: str | Path | None = None,
    ) -> None:
        if physical_actions and (simulation or physical_trial or not config.port):
            raise ValueError("physical actions require an explicit real port and exclusive mode")
        if physical_actions and (config.auto_enable_torque or config.configure_motors_on_connect
                                 or config.disable_torque_on_disconnect):
            raise ValueError("physical broker must preserve human torque/configuration ownership")
        if physical_trial and (simulation or arm_factory is None or not config.port):
            raise ValueError("physical trial requires a real port and explicit SDK arm factory")
        if trace_path is not None and not physical_trial:
            raise ValueError("passive broker motion trace requires physical trial mode")
        if simulation_actions and (not simulation or physical_trial):
            raise ValueError("SDK simulation action mode requires a simulated arm only")
        if not simulation and arm_factory is None and not physical_actions:
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
        self.simulation_actions = simulation_actions
        self.physical_trial = physical_trial
        self.physical_actions = physical_actions
        self._supervised_lease: AgentAuthority | None = None
        self.motion_guard: Callable[[], None] | None = None
        self.session_id = uuid.uuid4().hex
        self.connection_count = 0
        self.trace_path = Path(trace_path).expanduser() if trace_path else None
        self._trace: PassiveBackendTrace | None = None
        self._arm_factory = arm_factory or (
            lambda: (SOARM101(config) if physical_actions
                     else SOARM101.simulated(config=config, realtime=False))
        )
        self._authority = authority_store or AgentAuthorityStore()
        self._create_lock = threading.Lock()
        self._dispatch_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._lifecycle_lock = threading.RLock()
        self._lifecycle = "new"  # new -> open -> closing/closed; failure -> faulted
        self._fault_kind: str | None = None
        self._authority_revoke_error: str | None = None
        # Close after a verified physical STOP must not send another
        # all-motor position latch. Any later motion clears this flag.
        self._verified_physical_stop = False
        # Opening for read-only state/preflight does not transfer motion
        # ownership from an earlier human-armed CLI process. Closing such a
        # physical session must not issue a fresh all-motor position latch.
        self._motion_action_attempted = False
        self._bound_calibration_id: str | None = None
        self._session: SOARM101 | None = None

    def _require_active(self, *, stop: bool = False) -> None:
        """Never resurrect an explicitly closed or faulted hardware session."""
        with self._lifecycle_lock:
            if self._lifecycle == "closed" or (
                self._lifecycle == "closing"
                and not (stop and self._session is not None)
            ):
                raise RuntimeError("SDK broker session is closed")
            if self._lifecycle == "faulted" and not stop:
                raise RuntimeError("SDK broker session faulted; human restart required")

    def _revoke_authority(self) -> None:
        # A filesystem error cannot prevent the physical STOP attempt. The
        # executor remains fenced and STOP reports revocation failure below.
        try:
            self._authority.clear()
        except OSError as exc:
            self._authority_revoke_error = str(exc)

    def _fault(self, exc: BaseException) -> None:
        """Latch transport/servo faults; a new executor is required for recovery."""
        with self._lifecycle_lock:
            if self._lifecycle not in {"closing", "closed"}:
                self._lifecycle = "faulted"
                self._fault_kind = type(exc).__name__
        if self.motion_guard is not None and isinstance(self._session, SOARM101):
            self._session.motion.inhibit_new_motion("supervised SDK session faulted")
        if self.physical_trial or self.physical_actions:
            self._revoke_authority()

    def _assert_session_identity(self, arm: SOARM101) -> None:
        if getattr(arm, "is_connected", True) is False:
            error = RobotConnectionError("SDK broker device disconnected")
            self._fault(error)
            raise error
        if arm.config.robot_id != self.robot_id:
            error = RobotConnectionError("SDK broker robot identity changed")
            self._fault(error)
            raise error
        observed_calibration = self._calibration_id(arm)
        if (self._bound_calibration_id is not None
                and observed_calibration != self._bound_calibration_id):
            error = RobotConnectionError("SDK broker calibration identity changed")
            self._fault(error)
            raise error

    def _arm(self) -> SOARM101:
        with self._create_lock:
            with self._lifecycle_lock:
                self._require_active()
                if self._session is None:
                    try:
                        arm = self._arm_factory()
                    except BaseException as exc:
                        # Factory failure is not an invitation to reconnect on
                        # the next agent request.
                        self._fault(exc)
                        raise
                    try:
                        if arm.config.robot_id != self.robot_id:
                            raise RobotConnectionError(
                                "SDK session robot ID differs from broker identity"
                            )
                        arm.connect()  # Connection only; never auto-enable torque.
                        observed = self._calibration_id(arm)
                        if self.trace_path is not None:
                            trace = PassiveBackendTrace(
                                arm, self.trace_path,
                                metadata={
                                    "kind": "persistent_sdk_broker_physical_trial",
                                    "session_id": self.session_id,
                                    "joint_speed_deg_s": self.rates.joint_speed_deg_s,
                                    "joint_acceleration_deg_s2": self.rates.joint_acceleration_deg_s2,
                                },
                            )
                            trace.__enter__()
                            self._trace = trace
                    except BaseException as exc:
                        self._fault(exc)
                        try:
                            arm.disconnect()
                        except Exception:
                            pass  # Preserve the original connection failure.
                        raise
                    self.connection_count += 1
                    self._session = arm
                    self._bound_calibration_id = observed
                    self._lifecycle = "open"
                self._assert_session_identity(self._session)
                return self._session

    def close(self) -> None:
        """Terminal, idempotent shutdown; never permit implicit reconnection."""
        with self._lifecycle_lock:
            if self._lifecycle in {"closing", "closed"}:
                return
            had_unverified_fault = self._lifecycle == "faulted"
            self._lifecycle = "closing"  # Reject queued requests before waiting.
        try:
            # STOP may be independent of the ordinary dispatch lock. Do not
            # disconnect the transport while an in-flight STOP is accessing it.
            with self._dispatch_lock:
                with self._stop_lock:
                    with self._create_lock:
                        arm = self._session
                        try:
                            if arm is not None:
                                if (
                                    (self.physical_trial or self.physical_actions)
                                    and isinstance(arm, SOARM101)
                                    and (
                                        had_unverified_fault
                                        or self._verified_physical_stop
                                        or not self._motion_action_attempted
                                    )
                                ):
                                    # A failed OR a verified physical STOP
                                    # already sent its position latch. Cleanup
                                    # must not silently send another goal.
                                    arm.disconnect(stop_before_disconnect=False)
                                else:
                                    arm.disconnect()
                        finally:
                            self._session = None
                            if self._trace is not None:
                                trace = self._trace
                                self._trace = None
                                trace.__exit__(None, None, None)
        except BaseException as exc:
            with self._lifecycle_lock:
                self._lifecycle = "faulted"
                self._fault_kind = type(exc).__name__
            raise
        else:
            with self._lifecycle_lock:
                self._lifecycle = "closed"
        finally:
            if self.physical_trial or self.physical_actions:
                self._revoke_authority()

    @property
    def session_evidence(self) -> dict[str, object]:
        return {
            "id": self.session_id,
            "sdk_connect_count": self.connection_count,
            "mode": (
                "physical_actions_experimental" if self.physical_actions else
                "physical_trial" if self.physical_trial else
                "simulation_actions" if self.simulation_actions else
                "simulation_preview"
            ),
            "robot_id": self.robot_id,
            "lifecycle": self._lifecycle,
            "fault_kind": self._fault_kind,
            "authority_revoke_error": self._authority_revoke_error,
        }

    def _calibration_id(self, arm: SOARM101) -> str:
        identity = arm.calibration_id
        if identity:
            return identity
        if self.simulation:
            return "simulation"
        raise PermissionError("agent motion requires an active calibrated follower")

    def validate_supervision(self) -> None:
        """Read the existing lease only; never connect hardware or create authority."""
        self._require_active()
        arm = self._session
        if arm is None:
            raise PermissionError("connected SDK state preflight required before heartbeat")
        self._assert_session_identity(arm)
        if self._bound_calibration_id is None:
            raise PermissionError("supervision requires bound calibration identity")
        lease = self._authority.require(
            robot_id=self.robot_id, calibration_id=self._bound_calibration_id,
        )
        if self._supervised_lease is None:
            self._supervised_lease = lease
        elif lease != self._supervised_lease:
            raise PermissionError("human authority changed during supervised session")

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
        if "joint_only" in request:
            if not self.physical_trial or action not in {"go_pose", "sleep"}:
                raise PermissionError("joint-only is limited to local physical trial poses")
            if not isinstance(request["joint_only"], bool):
                raise ValueError("joint_only must be boolean")
        # Direct in-process calls receive the same trusted ceiling enforcement
        # as the HTTP/CLI adapter. Validation happens before device activity.
        self.rates.selected(action, request)

    def execute(self, action: str, request: Mapping[str, object]) -> dict[str, object]:
        """Validate and serialize ordinary SDK actions; do not block STOP."""
        self._validate(action, request)
        if self.physical_actions and action not in {"state", "capabilities", "capture", "joint", "stop"}:
            raise PermissionError("experimental physical SDK mode permits bounded joint actions only")
        def run() -> dict[str, object]:
            self._require_active(stop=action == "stop")
            if action in {"go_pose", "joint", "jog", "gripper", "sleep", "sleep_up"}:
                if self.motion_guard is not None:
                    self.motion_guard()
                elif self.physical_actions:
                    raise PermissionError("physical SDK actions require broker watchdog supervision")
                self._verified_physical_stop = False
                self._motion_action_attempted = True
            trace = self._trace
            if trace is not None:
                trace.mark(
                    "broker_action_start", action=action,
                    pose_name=request.get("name"), joint_only=request.get("joint_only"),
                    speed_deg_s=request.get("speed_deg_s", self.rates.joint_speed_deg_s),
                    acceleration_deg_s2=request.get(
                        "acceleration_deg_s2", self.rates.joint_acceleration_deg_s2
                    ),
                )
            try:
                result = self._execute_action(action, request)
                if action == "stop" and (self.physical_trial or self.physical_actions):
                    self._verified_physical_stop = bool(
                        result.get("completed")
                        and result.get("holding")
                        and result.get("post_stop_stability", {}).get("verified")
                    )
            except BaseException as exc:
                # Never re-open a silently disconnected/faulted transport on
                # the next request. STOP remains possible on an existing arm.
                if (
                    isinstance(exc, (
                        CommunicationError, RobotConnectionError, HardwareFaultError,
                        MotionTimeoutError, OSError, ConnectionError,
                    ))
                    # An attempted STOP that cannot confirm HOLD leaves
                    # physical state unknown even if its backend raised a
                    # plain RuntimeError. Latch failure and revoke authority.
                    or (action == "stop" and self._session is not None)
                ):
                    self._fault(exc)
                if self._trace is not None:
                    self._trace.mark("broker_action_error", action=action, error=repr(exc))
                raise
            if self._trace is not None:
                self._trace.mark("broker_action_end", action=action)
            return {**result, "broker_session": self.session_evidence}

        if action == "stop":
            with self._stop_lock:
                return run()
        with self._dispatch_lock:
            return run()

    def _execute_action(
        self, action: str, request: Mapping[str, object]
    ) -> dict[str, object]:
        """Perform one already validated trusted-host SDK operation."""
        if action == "capabilities":
            # Reuse the legacy pure projection until wrapper retirement in PR 3.
            from soarm101_motion.cli.main import _agent_capabilities_payload

            result = _agent_capabilities_payload(self.robot_id, config=self.config)
            # This opt-in preview must not advertise physical authority or
            # motion/camera tools that its HTTP boundary explicitly denies.
            if self.physical_actions:
                actions = result.get("actions")
                if not isinstance(actions, dict):
                    raise RuntimeError("SDK capabilities are missing actions")
                visible_actions = {
                    name: value for name, value in actions.items()
                    if name in {"state", "joint", "stop", "capture"}
                }
                joint_action = visible_actions.get("joint")
                if isinstance(joint_action, dict):
                    joint_action["max_abs_delta_deg"] = 2.0
                result["actions"] = visible_actions
                result["poses"] = []
                result["authority"] = self._authority.status()
            elif self.physical_trial:
                result["actions"] = {
                    "state": "read_only", "go_pose": True,
                    "sleep": True, "stop": True,
                }
                from soarm101_motion.poses import PoseLibrary
                names = PoseLibrary(self.robot_id).names()
                result["poses"] = [name for name in names if name.startswith(AGENT_POSE_PREFIX)]
                result["authority"] = self._authority.status()
            else:
                if self.simulation_actions:
                    # Actual supported simulated actions. The HTTP profile
                    # further intersects these; no fake Cartesian workspace
                    # or camera capture is advertised.
                    allowed = {
                        "state", "go_pose", "joint", "gripper",
                        "sleep", "sleep_up", "stop",
                    }
                    result["actions"] = {
                        k: v for k, v in result["actions"].items()
                        if k in allowed
                    }
                    result["authority"] = self._authority.status()
                else:
                    result["actions"] = {"state": "read_only"}
                    result["authority"] = {
                        "armed": False, "robot_id": None, "calibration_id": None,
                        "issued_at": None, "expires_at": None,
                        "remaining_seconds": 0.0,
                    }
                result["poses"] = []
                result["world_directions"] = {
                    "available": False,
                    "reason": "simulation preview has no measured physical directions",
                }
            if not self.physical_actions:
                result["cameras"] = []
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
            # Revocation occurs even if the physical/fake HOLD subsequently
            # fails. A human must issue fresh motion authority after STOP.
            self._revoke_authority()
            # DO NOT wait for the motion lock or silently enable torque here.
            # A new session cannot be opened by STOP during an active command.
            arm = self._session
            if arm is None:
                raise RuntimeError("no connected SDK session to stop")
            if getattr(arm, "is_connected", True) is False:
                raise RobotConnectionError(
                    "STOP/HOLD unverified: SDK device is disconnected"
                )
            if self.motion_guard is not None and isinstance(arm, SOARM101):
                arm.motion.inhibit_new_motion("supervised broker STOP requested")
            SDK_CAPABILITIES.dispatch("stop", arm, {})
            # STOP/HOLD was attempted. Verify the device is still connected
            # and not faulted before declaring completion. Never invent an
            # encoder position from the last issued command.
            state = arm.get_state()
            if getattr(state, "connected", True) is False:
                raise RobotConnectionError(
                    "STOP/HOLD unverified: device disconnected during STOP"
                )
            if getattr(state, "faulted", False):
                raise HardwareFaultError(
                    "STOP/HOLD unverified: " +
                    str(getattr(state, "fault_message", None) or "device is faulted")
                )
            joints = dict(arm.get_joint_positions().positions)
            stability: dict[str, object] = {"verified": False}
            if not self.simulation:
                # No new target writes are made during these observations.
                # Compare every measured sample against the immediate latch
                # position, not just the end points of an oscillation.
                if not state.torque_enabled:
                    raise HardwareFaultError(
                        "STOP/HOLD unverified: physical torque hold is disabled"
                    )
                first = dict(joints)
                max_drift_deg = 0.0
                samples = []
                elapsed = 0.0
                for pause in STOP_STABILITY_SAMPLE_DELAYS_S:
                    time.sleep(pause)
                    elapsed += pause
                    sampled = dict(arm.get_joint_positions().positions)
                    if set(sampled) != set(first):
                        raise HardwareFaultError(
                            "STOP/HOLD unverified: arm joint feedback changed shape"
                        )
                    worst_joint, drift = max(
                        (
                            (joint, abs(math.degrees(sampled[joint] - initial)))
                            for joint, initial in first.items()
                        ),
                        key=lambda item: item[1],
                    )
                    max_drift_deg = max(max_drift_deg, drift)
                    samples.append({
                        "elapsed_s": elapsed, "max_drift_deg": drift,
                        "worst_joint": worst_joint,
                    })
                    if self._trace is not None:
                        self._trace.mark(
                            "post_stop_stability_sample",
                            elapsed_s=elapsed, max_drift_deg=drift,
                            joints_rad=sampled,
                        )
                    if drift > STOP_MAX_POST_LATCH_DRIFT_DEG:
                        raise HardwareFaultError(
                            "STOP/HOLD unverified: "
                            f"{worst_joint} moved {drift:.2f}° after the "
                            "position latch, without a new movement command"
                        )
                    joints = sampled
                state = arm.get_state()
                if (
                    not state.connected or state.faulted
                    or not state.torque_enabled or state.moving
                ):
                    raise HardwareFaultError(
                        "STOP/HOLD unverified: motor state still moving, "
                        "faulted, disconnected, or torque-disabled after settling"
                    )
                stability = {
                    "verified": True,
                    "duration_s": elapsed,
                    "max_drift_deg": max_drift_deg,
                    "samples": samples,
                }
            if self._authority_revoke_error is not None:
                raise HardwareFaultError(
                    "STOP attempted but authority revocation failed: " + self._authority_revoke_error
                )
            return {
                "accepted": True, "completed": True, "action": "stop",
                "holding": bool(state.torque_enabled),
                "joint_positions_rad": joints,
                "post_stop_stability": stability,
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
                "authority": self._authority.status() if (
                    self.physical_trial or self.physical_actions or self.simulation_actions
                ) else {
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
        rates = self.rates.selected(action, request)
        if action == "go_pose":
            name = self._text(request, "name")
            if not name.startswith(AGENT_POSE_PREFIX):
                raise PermissionError("agent pose name must begin with 'agent_'")
            # Check provenance before enabling torque; replay checks it again.
            SDK_CAPABILITIES.dispatch(
                "validate_saved_pose", arm,
                {"robot_id": self.robot_id, "name": name},
            )
            joint_only = bool(request.get("joint_only", False))
            if self.physical_trial and joint_only:
                # A separately human-armed process may already own the servo
                # torque HOLD. A redundant enable_torque() would issue a new
                # all-motor 1/1 position latch before the approved Overhead
                # trajectory and change Feetech inner-loop tracking settings.
                # Preserve that verified live HOLD just as physical Sleep does.
                hold = arm.get_state()
                if (not hold.connected or hold.faulted
                        or not hold.torque_enabled or hold.moving):
                    raise HardwareFaultError(
                        "joint-only physical saved pose requires an already "
                        "healthy stationary torque HOLD"
                    )
            else:
                arm.enable()
            moved, gripper = SDK_CAPABILITIES.dispatch(
                "replay_saved_pose", arm,
                {"robot_id": self.robot_id, "name": name,
                 "include_gripper": not joint_only, **rates},
            )
            if not (moved.accepted and moved.completed
                    and (gripper is None or (gripper.accepted and gripper.completed))):
                raise RuntimeError("saved pose playback did not complete successfully")
            return {
                "accepted": True, "completed": True, "action": "go_pose",
                "pose": name, "holding": True, "authority": authority,
                "arm": asdict(moved),
                "gripper": asdict(gripper) if gripper is not None else None,
                "joint_only": joint_only,
            }
        if action == "joint":
            joint = self._text(request, "joint")
            delta = self._number(request, "delta_deg")
            delta_cap = 2.0 if self.physical_actions else AGENT_JOINT_MAX_DELTA_DEG
            if abs(delta) <= 1e-9 or abs(delta) > delta_cap:
                raise PermissionError("agent joint delta must be nonzero and within bounded policy")
            arguments = {
                "joint": joint, "delta_rad": delta * pi / 180.0,
                "speed_rad_s": float(rates["speed_deg_s"]) * pi / 180.0,
                "acceleration_rad_s2": float(rates["acceleration_deg_s2"]) * pi / 180.0,
            }
            SDK_CAPABILITIES.get("jog_joint").validate(arguments)
            before = dict(arm.get_joint_positions().positions)
            if self.physical_actions:
                self.validate_supervision()
                hold = arm.get_state()
                if (not hold.connected or hold.faulted or hold.moving
                        or not hold.torque_enabled):
                    raise HardwareFaultError("physical joint action requires healthy stationary torque HOLD")
            else:
                arm.enable()
            result = SDK_CAPABILITIES.dispatch("jog_joint", arm, arguments)
            if self.physical_actions:
                self._require_active()
                hold = arm.get_state()
                if (not hold.connected or hold.faulted or hold.moving
                        or not hold.torque_enabled):
                    raise HardwareFaultError("physical joint completion requires healthy stationary HOLD")
            else:
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
                "speed_mm_s": rates["speed_mm_s"],
                "acceleration_mm_s2": rates["acceleration_mm_s2"],
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
                "move_gripper", arm, {"position": position, **rates}
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
            if self.physical_trial and action == "sleep" and request.get("joint_only") is True:
                # The attended Sleep-first trial begins from a human-armed
                # torque HOLD established in an earlier CLI process. Our
                # fresh Feetech backend now reconciles the live servo torque
                # registers on connect. Do NOT blindly call arm.enable()
                # again here: enable_torque() issues a new all-motor 1/1
                # position latch, which could alter the servo inner loop.
                # Refuse an unsafe or unverified hold rather than changing it.
                initial_hold = arm.get_state()
                if (
                    not initial_hold.connected or initial_hold.faulted
                    or not initial_hold.torque_enabled or initial_hold.moving
                ):
                    raise HardwareFaultError(
                        "joint-only physical Sleep requires an already healthy "
                        "stationary torque HOLD"
                    )
            else:
                arm.enable()
            result = SDK_CAPABILITIES.dispatch(
                action, arm, {
                    "speed_rad_s": float(rates["speed_deg_s"]) * pi / 180.0,
                    "acceleration_rad_s2": float(rates["acceleration_deg_s2"]) * pi / 180.0,
                    "gripper_speed_raw": rates["gripper_speed_raw"],
                    "gripper_acceleration_raw": rates["gripper_acceleration_raw"],
                    **({"close_gripper": not request.get("joint_only", False)}
                       if action == "sleep" else {}),
                }
            )
            arm.hold()
            if not result.accepted or not result.completed:
                raise RuntimeError("Sleep operation did not complete successfully")
            return {**asdict(result), "action": action, "holding": True,
                    "authority": authority}
        raise AssertionError(f"missing SDK broker action handler for {action}")
