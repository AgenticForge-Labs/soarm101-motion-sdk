"""Optional PyBullet visualization for the common kinematic simulation backend."""

from __future__ import annotations

from collections.abc import Mapping
from importlib.resources import as_file, files
from typing import Any

from soarm101_motion.constants import ARM_JOINTS, STOCK_GRIPPER
from soarm101_motion.exceptions import MissingDependencyError
from soarm101_motion.hardware.simulation import SimulationBackend


class PyBulletSimulationBackend(SimulationBackend):
    def __init__(
        self,
        initial_positions: Mapping[str, float] | None = None,
        *,
        gui: bool = True,
        realtime: bool = True,
    ) -> None:
        super().__init__(initial_positions, realtime=realtime)
        self.gui = gui
        self._p: Any = None
        self._client: int | None = None
        self._robot: int | None = None
        self._joint_indices: dict[str, int] = {}
        self._resource_context: Any = None

    def connect(self) -> None:
        if self.is_connected:
            return
        try:
            import pybullet as p
        except ImportError as exc:
            raise MissingDependencyError(
                "visual simulation requires the 'simulation' extra: pip install 'soarm101-motion-sdk[simulation]'"
            ) from exc
        self._p = p
        mode = p.GUI if self.gui else p.DIRECT
        self._client = p.connect(mode)
        p.setGravity(0, 0, -9.81, physicsClientId=self._client)
        resource = files("soarm101_motion.models").joinpath("so101.urdf")
        self._resource_context = as_file(resource)
        urdf_path = self._resource_context.__enter__()
        self._robot = p.loadURDF(
            str(urdf_path),
            useFixedBase=True,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self._client,
        )
        for index in range(p.getNumJoints(self._robot, physicsClientId=self._client)):
            name = p.getJointInfo(self._robot, index, physicsClientId=self._client)[1].decode()
            self._joint_indices[name] = index
        super().connect()
        self._update_visual_state()

    def disconnect(self) -> None:
        try:
            super().disconnect()
        finally:
            if self._p is not None and self._client is not None:
                self._p.disconnect(self._client)
            if self._resource_context is not None:
                self._resource_context.__exit__(None, None, None)
            self._client = None
            self._robot = None

    def _update_visual_state(self) -> None:
        if self._p is None or self._robot is None or self._client is None:
            return
        for name in ARM_JOINTS:
            if name in self._joint_indices:
                self._p.resetJointState(
                    self._robot,
                    self._joint_indices[name],
                    self._positions[name],
                    physicsClientId=self._client,
                )
        if "gripper" in self._joint_indices:
            normalized = self._tool_positions[STOCK_GRIPPER]
            angle = -0.174533 + normalized * (1.74533 + 0.174533)
            self._p.resetJointState(
                self._robot,
                self._joint_indices["gripper"],
                angle,
                physicsClientId=self._client,
            )
        self._p.stepSimulation(physicsClientId=self._client)

    def write_joint_positions(self, positions: Mapping[str, float], **kwargs: int | None) -> None:
        super().write_joint_positions(positions, **kwargs)
        self._update_visual_state()

    def write_tool_position(self, actuator: str, position: float, **kwargs: int | None) -> None:
        super().write_tool_position(actuator, position, **kwargs)
        self._update_visual_state()
