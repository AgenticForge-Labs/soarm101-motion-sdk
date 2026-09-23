"""Application-facing SO-ARM101 facade."""

from __future__ import annotations

import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import replace
from math import pi
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import (
    ARM_JOINTS,
    DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
    HOME_JOINTS,
    JOINT_LIMITS,
)
from soarm101_motion.exceptions import (
    ConfigurationError,
    InvalidCommandError,
    InvalidJointError,
    RobotConnectionError,
    SafetyViolationError,
)
from soarm101_motion.hardware import FeetechBackend, SO101HardwareBackend, SimulationBackend
from soarm101_motion.kinematics import IKOptions, IKSolver, OrientationMode, SO101KinematicModel
from soarm101_motion.motion import MotionController, MotionHandle
from soarm101_motion.provenance import require_calibration_compatibility
from soarm101_motion.safety import (
    validate_joint_targets,
    validate_workspace_configuration,
    validate_workspace_path,
)
from soarm101_motion.tools import RobotTool, SO101Gripper
from soarm101_motion.trajectories import Trajectory
from soarm101_motion.types import HardwareState, IKResult, JointState, MotionResult, Pose


class SOARM101:
    """High-level SO-ARM101 motion API."""

    def __init__(
        self,
        config: SOARM101Config | None = None,
        *,
        port: str | None = None,
        robot_id: str | None = None,
        calibration_path: str | Path | None = None,
        backend: SO101HardwareBackend | None = None,
        tool: RobotTool | None = None,
    ) -> None:
        if config is None:
            config = SOARM101Config(
                port=port,
                robot_id=robot_id or "so101",
                calibration_path=Path(calibration_path).expanduser() if calibration_path else None,
            )
        elif any(value is not None for value in (port, robot_id, calibration_path)):
            raise ConfigurationError("pass either SOARM101Config or constructor overrides, not both")
        if backend is None:
            if not config.port:
                ports = FeetechBackend.candidate_ports()
                if len(ports) != 1:
                    detail = "none found" if not ports else ", ".join(ports)
                    raise ConfigurationError(
                        "hardware mode requires a serial port when auto-detection is not unique; "
                        f"candidates: {detail}"
                    )
                config = replace(config, port=ports[0])
            backend = FeetechBackend(config)
        self.config = config
        self.backend = backend
        self.tool = tool or SO101Gripper()
        bind = getattr(self.tool, "bind", None)
        if callable(bind):
            bind(backend)
        self.model = SO101KinematicModel()
        self.ik = IKSolver(self.model)
        self.motion = MotionController(backend, config, self.model)
        self._active_tcp = "gripper" if "gripper" in self.tool.tcp_frames else "tool"

    @classmethod
    def simulated(
        cls,
        *,
        gui: bool = False,
        realtime: bool = False,
        initial_positions: Mapping[str, float] | None = None,
        tool: RobotTool | None = None,
    ) -> "SOARM101":
        config = SOARM101Config(auto_enable_torque=False)
        if gui:
            from soarm101_motion.simulation.pybullet import PyBulletSimulationBackend

            backend: SO101HardwareBackend = PyBulletSimulationBackend(
                initial_positions=initial_positions,
                gui=True,
                realtime=realtime,
            )
        else:
            backend = SimulationBackend(initial_positions, realtime=realtime)
        return cls(config, backend=backend, tool=tool)

    def __enter__(self) -> "SOARM101":
        self.connect()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        return self.backend.is_connected

    @property
    def is_moving(self) -> bool:
        tool_moving = bool(getattr(self.tool, "is_moving", False))
        return self.motion.is_moving or tool_moving or self.backend.get_hardware_state().moving

    @property
    def calibration_id(self) -> str | None:
        calibration = getattr(self.backend, "calibration", None)
        if calibration is None:
            return None
        return str(calibration.calibration_id)

    @property
    def calibration_source(self) -> str | None:
        calibration = getattr(self.backend, "calibration", None)
        if calibration is None:
            return None
        return str(calibration.source)

    def require_artifact_calibration(
        self,
        metadata: Mapping[str, object],
        *,
        artifact_label: str,
    ) -> None:
        """Require physical artifacts to match this robot's active calibration.

        Simulation backends do not expose a physical calibration object, so provenance
        gating is intentionally skipped there.
        """

        calibration = getattr(self.backend, "calibration", None)
        if calibration is None:
            return
        require_calibration_compatibility(
            metadata,
            current_robot_id=self.config.robot_id,
            current_calibration_id=calibration.calibration_id,
            artifact_label=artifact_label,
        )

    @property
    def active_tcp(self) -> Pose:
        frames = self.tool.tcp_frames
        if self._active_tcp not in frames:
            return self.model.tcp
        return frames[self._active_tcp]

    def set_active_tcp(self, name: str) -> None:
        if name not in self.tool.tcp_frames:
            raise KeyError(f"tool does not define TCP frame {name!r}")
        self._active_tcp = name

    def _stop_tool(self, *, wait: bool = True) -> None:
        stop = getattr(self.tool, "stop", None)
        if callable(stop):
            stop(wait=wait)

    def _workspace_kwargs(self) -> dict[str, float]:
        return {
            "minimum_z_m": self.config.minimum_workspace_z_m,
            "maximum_tcp_reach_m": self.config.maximum_tcp_reach_m,
            "minimum_self_clearance_m": self.config.minimum_self_clearance_m,
            "base_keepout_radius_m": self.config.base_keepout_radius_m,
            "base_keepout_height_m": self.config.base_keepout_height_m,
        }

    def _resolve_joint_target(
        self,
        positions: Mapping[str, float] | Sequence[float],
        *,
        relative: bool,
    ) -> tuple[dict[str, float], dict[str, float]]:
        current = self.backend.read_joint_positions()
        target = dict(current)
        if isinstance(positions, Mapping):
            for name, value in positions.items():
                if name not in ARM_JOINTS:
                    raise InvalidJointError(f"unknown arm joint: {name}")
                value = float(value)
                if not math.isfinite(value):
                    raise InvalidCommandError(f"joint {name} target must be finite")
                target[name] = current[name] + value if relative else value
        else:
            values = [float(value) for value in positions]
            if len(values) != len(ARM_JOINTS):
                raise InvalidCommandError(f"expected {len(ARM_JOINTS)} joint values")
            if not all(math.isfinite(value) for value in values):
                raise InvalidCommandError("all joint targets must be finite")
            target = {
                name: current[name] + value if relative else value
                for name, value in zip(ARM_JOINTS, values, strict=True)
            }
        validate_joint_targets(target)
        return current, target

    def _validate_joint_workspace_path(
        self,
        positions: Mapping[str, float] | Sequence[float],
        *,
        relative: bool,
    ) -> None:
        if not self.config.enable_workspace_checks:
            return
        current, target = self._resolve_joint_target(positions, relative=relative)
        max_delta = max(abs(target[name] - current[name]) for name in ARM_JOINTS)
        steps = max(2, int(math.ceil(max_delta / self.config.workspace_check_step_rad)) + 1)
        samples = tuple(
            {
                name: current[name] + (target[name] - current[name]) * fraction
                for name in ARM_JOINTS
            }
            for fraction in np.linspace(0.0, 1.0, steps)
        )
        validate_workspace_path(
            self.model,
            samples,
            tcp=self.active_tcp,
            **self._workspace_kwargs(),
        )

    def connect(self) -> None:
        connected = False
        try:
            self.backend.connect()
            connected = True
            if self.config.auto_enable_torque:
                self.enable()
        except Exception as exc:
            if connected:
                try:
                    self.backend.disconnect()
                except Exception:
                    pass
            if isinstance(exc, RobotConnectionError):
                raise
            raise RobotConnectionError("failed to connect to SO-ARM101") from exc

    def disconnect(self) -> None:
        self.motion.stop(wait=True)
        self._stop_tool(wait=True)
        self.backend.disconnect()

    def enable(self) -> None:
        """Latch present positions and enable torque without executing stale goals."""
        self.backend.enable_torque()

    def disable(self) -> None:
        self.motion.stop(wait=True)
        self._stop_tool(wait=True)
        self.backend.disable_torque()

    def hold(self) -> None:
        state = self.backend.get_hardware_state()
        if not state.connected:
            raise RobotConnectionError("robot is not connected")
        if state.torque_enabled:
            self.stop()
        else:
            self.enable()

    def relax(self) -> None:
        self.disable()

    def stop(self) -> None:
        """Cancel active host motion and hold the current arm/tool positions."""
        self.motion.stop(wait=True)
        self._stop_tool(wait=True)

    software_stop = stop

    def get_state(self) -> HardwareState:
        return self.backend.get_hardware_state()

    def get_effort_safety_status(self, *, refresh: bool = False) -> dict[str, object]:
        """Return backend effort-guard state when supported."""

        getter = getattr(self.backend, "get_effort_safety_status", None)
        if not callable(getter):
            return {"supported": False}
        return dict(getter(refresh=refresh))

    def configure_effort_safety(
        self,
        *,
        enabled: bool,
        current_trip_raw: int | None,
        load_trip_raw: int | None,
        consecutive_samples: int,
    ) -> None:
        """Apply session-only effort guard settings on supported hardware."""

        configure = getattr(self.backend, "configure_effort_safety", None)
        if not callable(configure):
            raise RuntimeError("active backend does not provide effort safety configuration")
        configure(
            enabled=enabled,
            current_trip_raw=current_trip_raw,
            load_trip_raw=load_trip_raw,
            consecutive_samples=consecutive_samples,
        )

    def clear_effort_trip(self) -> None:
        clear = getattr(self.backend, "clear_effort_trip", None)
        if not callable(clear):
            raise RuntimeError("active backend does not provide an effort safety interlock")
        clear()

    def reset_effort_peaks(self) -> None:
        reset = getattr(self.backend, "reset_effort_peaks", None)
        if not callable(reset):
            raise RuntimeError("active backend does not provide effort peak tracking")
        reset()

    def get_joint_limits(self) -> dict[str, tuple[float, float]]:
        """Return effective model/calibration limits for the five pose joints."""
        limits = dict(JOINT_LIMITS)
        calibration = getattr(self.backend, "calibration", None)
        if calibration is None:
            return limits
        for name in ARM_JOINTS:
            motor = calibration.motors.get(name)
            if motor is None:
                continue
            model_lower, model_upper = limits[name]
            calibrated_lower, calibrated_upper = motor.radians_limits
            lower = max(model_lower, calibrated_lower)
            upper = min(model_upper, calibrated_upper)
            if lower < upper:
                limits[name] = (lower, upper)
        return limits

    def get_joint_positions(self) -> JointState:
        return JointState(self.backend.read_joint_positions(), time.monotonic())

    def get_position(self, *, tcp: Pose | None = None) -> Pose:
        return self.model.forward(self.backend.read_joint_positions(), tcp=tcp or self.active_tcp)

    def move_joints(
        self,
        positions: Mapping[str, float] | Sequence[float],
        *,
        speed: float | None = None,
        acceleration: float | None = None,
        relative: bool = False,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        self._validate_joint_workspace_path(positions, relative=relative)
        return self.motion.move_joints(
            positions,
            speed=speed,
            acceleration=acceleration,
            relative=relative,
            wait=wait,
        )

    def move_home(
        self,
        *,
        speed: float | None = None,
        acceleration: float | None = None,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        return self.move_joints(
            HOME_JOINTS,
            speed=speed,
            acceleration=acceleration,
            wait=wait,
        )

    move_gohome = move_home

    def solve_ik(
        self,
        target: Pose,
        *,
        seed: Mapping[str, float] | None = None,
        orientation_mode: OrientationMode = "compatible",
        look_at: Sequence[float] | None = None,
        tcp: Pose | None = None,
    ) -> IKResult:
        active_tcp = tcp or self.active_tcp
        result = self.ik.solve(
            target,
            seed=seed or self.backend.read_joint_positions(),
            tcp=active_tcp,
            options=IKOptions(
                orientation_mode=orientation_mode,
                look_at=np.asarray(look_at, dtype=float) if look_at is not None else None,
            ),
        )
        if result.success and self.config.enable_workspace_checks:
            try:
                validate_workspace_configuration(
                    self.model,
                    result.joints,
                    tcp=active_tcp,
                    **self._workspace_kwargs(),
                )
            except SafetyViolationError as exc:
                return IKResult(
                    success=False,
                    joints=result.joints,
                    position_error_m=result.position_error_m,
                    orientation_error_rad=result.orientation_error_rad,
                    iterations=result.iterations,
                    message=f"IK solution violates workspace envelope: {exc}",
                )
        return result

    def move_pose(
        self,
        target: Pose,
        *,
        orientation_mode: OrientationMode = "compatible",
        look_at: Sequence[float] | None = None,
        speed: float | None = None,
        acceleration: float | None = None,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        solution = self.solve_ik(
            target,
            orientation_mode=orientation_mode,
            look_at=look_at,
        )
        if not solution.success:
            from soarm101_motion.exceptions import IKError

            raise IKError(solution.message)
        return self.move_joints(
            solution.joints,
            speed=speed,
            acceleration=acceleration,
            wait=wait,
        )

    def move_linear(
        self,
        target: Pose,
        *,
        orientation_mode: OrientationMode = "compatible",
        look_at: Sequence[float] | None = None,
        speed: float | None = None,
        acceleration: float | None = None,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        look_at_array = np.asarray(look_at, dtype=float) if look_at is not None else None
        if self.config.enable_workspace_checks:
            planned = self.motion.plan_linear(
                target,
                tcp=self.active_tcp,
                orientation_mode=orientation_mode,
                look_at=look_at_array,
                speed=speed,
                acceleration=acceleration,
            )
            validate_workspace_path(
                self.model,
                planned.command_samples,
                tcp=self.active_tcp,
                **self._workspace_kwargs(),
            )
        return self.motion.move_linear(
            target,
            tcp=self.active_tcp,
            orientation_mode=orientation_mode,
            look_at=look_at_array,
            speed=speed,
            acceleration=acceleration,
            wait=wait,
        )

    def get_servo_angle(self, *, is_radian: bool = True) -> list[float]:
        positions = self.backend.read_joint_positions()
        values = [positions[name] for name in ARM_JOINTS]
        return values if is_radian else [value * 180.0 / pi for value in values]

    def set_servo_angle(
        self,
        angle: Mapping[str, float] | Sequence[float],
        *,
        speed: float | None = None,
        mvacc: float | None = None,
        wait: bool = True,
        relative: bool = False,
        is_radian: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        scale = 1.0 if is_radian else pi / 180.0
        if isinstance(angle, Mapping):
            targets = {name: float(value) * scale for name, value in angle.items()}
        else:
            targets = [float(value) * scale for value in angle]
        speed_value = speed if is_radian or speed is None else speed * scale
        acceleration_value = mvacc if is_radian or mvacc is None else mvacc * scale
        return self.move_joints(
            targets,
            speed=speed_value,
            acceleration=acceleration_value,
            wait=wait,
            relative=relative,
        )

    def set_position(
        self,
        *,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        speed: float | None = None,
        mvacc: float | None = None,
        wait: bool = True,
        relative: bool = False,
        is_radian: bool = False,
        orientation_mode: OrientationMode = "compatible",
    ) -> MotionResult | MotionHandle[MotionResult]:
        current = self.get_position()
        cx, cy, cz, cr, cp, cyaw = current.xyz_rpy()
        distance_scale = 0.001
        angle_scale = 1.0 if is_radian else pi / 180.0
        supplied_position = np.array(
            [
                0.0 if x is None else x * distance_scale,
                0.0 if y is None else y * distance_scale,
                0.0 if z is None else z * distance_scale,
            ]
        )
        supplied_angles = np.array(
            [
                0.0 if roll is None else roll * angle_scale,
                0.0 if pitch is None else pitch * angle_scale,
                0.0 if yaw is None else yaw * angle_scale,
            ]
        )
        if relative:
            position = current.position + supplied_position
            rotation = current.rotation @ Rotation.from_euler("xyz", supplied_angles).as_matrix()
        else:
            position = np.array(
                [
                    cx if x is None else supplied_position[0],
                    cy if y is None else supplied_position[1],
                    cz if z is None else supplied_position[2],
                ]
            )
            angles = np.array(
                [
                    cr if roll is None else supplied_angles[0],
                    cp if pitch is None else supplied_angles[1],
                    cyaw if yaw is None else supplied_angles[2],
                ]
            )
            rotation = Rotation.from_euler("xyz", angles).as_matrix()
        target = Pose(position, rotation)
        speed_m = None if speed is None else speed * 0.001
        acceleration_m = None if mvacc is None else mvacc * 0.001
        return self.move_linear(
            target,
            orientation_mode=orientation_mode,
            speed=speed_m,
            acceleration=acceleration_m,
            wait=wait,
        )

    def start_joint_stream(
        self,
        *,
        frequency_hz: float = DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
    ) -> None:
        """Start guarded continuous joint streaming for teleoperation."""
        if self.tool.is_moving:
            self._stop_tool(wait=True)
        self.motion.start_joint_stream(
            frequency_hz=frequency_hz,
            tcp=self.active_tcp,
        )

    def stream_joint_target(
        self,
        positions: Mapping[str, float],
        *,
        gripper: float | None = None,
    ) -> MotionResult:
        """Send one guarded streaming target while a joint stream is active."""
        return self.motion.stream_joint_target(positions, gripper=gripper)

    def stop_joint_stream(self, *, hold: bool = True) -> None:
        """End continuous streaming and optionally hold the measured pose."""
        self.motion.stop_joint_stream(hold=hold)

    def play_trajectory(
        self,
        trajectory: Trajectory,
        *,
        speed_scale: float = 1.0,
        move_to_start: bool = True,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        """Replay an immutable recorded trajectory through the guarded motion stack."""
        self.require_artifact_calibration(
            trajectory.metadata,
            artifact_label="recorded trajectory",
        )
        if self.tool.is_moving:
            self._stop_tool(wait=True)
        return self.motion.play_trajectory(
            trajectory,
            speed_scale=speed_scale,
            move_to_start=move_to_start,
            tcp=self.active_tcp,
            wait=wait,
        )

    def diagnostics(self):
        return self.backend.diagnostics()
