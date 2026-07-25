"""Application-facing SO-ARM101 facade."""

from __future__ import annotations

import time
from collections.abc import Mapping, Sequence
from dataclasses import replace
from math import pi
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import ARM_JOINTS, HOME_JOINTS
from soarm101_motion.exceptions import ConfigurationError, RobotConnectionError
from soarm101_motion.hardware import FeetechBackend, SO101HardwareBackend, SimulationBackend
from soarm101_motion.kinematics import IKOptions, IKSolver, OrientationMode, SO101KinematicModel
from soarm101_motion.motion import MotionController, MotionHandle
from soarm101_motion.tools import RobotTool, SO101Gripper
from soarm101_motion.types import HardwareState, IKResult, JointState, MotionResult, Pose


class SOARM101:
    """High-level SO-ARM101 motion API.

    Hardware mode uses a direct Feetech backend. Use :meth:`simulated` for the
    in-memory simulator or ``SOARM101.simulated(gui=True)`` for optional PyBullet.
    """

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
        return self.motion.is_moving or self.backend.get_hardware_state().moving

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

    def connect(self) -> None:
        try:
            self.backend.connect()
            if self.config.auto_enable_torque:
                self.enable()
        except Exception as exc:
            if isinstance(exc, RobotConnectionError):
                raise
            raise RobotConnectionError("failed to connect to SO-ARM101") from exc

    def disconnect(self) -> None:
        self.motion.stop(wait=True)
        self.backend.disconnect()

    def enable(self) -> None:
        """Latch present positions and enable torque without executing stale goals."""
        self.backend.enable_torque()

    def disable(self) -> None:
        self.motion.stop(wait=True)
        self.backend.disable_torque()

    def hold(self) -> None:
        state = self.backend.get_hardware_state()
        if not state.connected:
            raise RobotConnectionError("robot is not connected")
        if state.torque_enabled:
            self.motion.stop(wait=True)
        else:
            self.enable()

    def relax(self) -> None:
        self.disable()

    def stop(self) -> None:
        self.motion.stop(wait=True)

    emergency_stop = stop

    def get_state(self) -> HardwareState:
        return self.backend.get_hardware_state()

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
        return self.ik.solve(
            target,
            seed=seed or self.backend.read_joint_positions(),
            tcp=tcp or self.active_tcp,
            options=IKOptions(
                orientation_mode=orientation_mode,
                look_at=np.asarray(look_at, dtype=float) if look_at is not None else None,
            ),
        )

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
        return self.motion.move_linear(
            target,
            tcp=self.active_tcp,
            orientation_mode=orientation_mode,
            look_at=np.asarray(look_at, dtype=float) if look_at is not None else None,
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

    def diagnostics(self):
        return self.backend.diagnostics()
