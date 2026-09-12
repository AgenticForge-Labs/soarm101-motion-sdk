"""xArm-inspired convenience API for direct SO-ARM101 control.

This is intentionally a thin facade over :class:`SOARM101`; it does not emulate
xArm controller features the SO-ARM101 hardware does not possess. The goal is
to make common direct-control scripts familiar while preserving the SDK's
existing safety, planning, and feedback checks.
"""

from __future__ import annotations

from math import pi
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from soarm101_motion.arm import SOARM101
from soarm101_motion.kinematics import OrientationMode
from soarm101_motion.motion import MotionHandle
from soarm101_motion.types import MotionResult, Pose


class SOArmAPI(SOARM101):
    """Direct-control facade with names familiar from the UFactory xArm SDK."""

    def motion_enable(self, enable: bool = True) -> None:
        """Enable or disable motor torque while preserving safe position latching."""

        if enable:
            self.enable()
        else:
            self.disable()

    def set_gripper_position(
        self,
        position: float,
        *,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        """Move the stock gripper using normalized position ``0.0 .. 1.0``."""

        move = getattr(self.tool, "move", None)
        if not callable(move):
            raise RuntimeError("active tool does not provide position control")
        return move(float(position), wait=wait)

    def get_gripper_position(self) -> float:
        """Return the stock gripper's normalized position ``0.0 .. 1.0``."""

        getter = getattr(self.tool, "get_position", None)
        if not callable(getter):
            raise RuntimeError("active tool does not provide position feedback")
        return float(getter())

    def set_tool_position(
        self,
        *,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        speed: float | None = None,
        mvacc: float | None = None,
        wait: bool = True,
        is_radian: bool = False,
        orientation_mode: OrientationMode = "compatible",
    ) -> MotionResult | MotionHandle[MotionResult]:
        """Apply a relative Cartesian move in the current TCP/tool frame.

        Translation arguments use millimetres, matching ``set_position``. Angular
        arguments use degrees unless ``is_radian=True``.
        """

        current = self.get_position()
        angle_scale = 1.0 if is_radian else pi / 180.0
        translation_tool = np.asarray([x, y, z], dtype=float) / 1000.0
        rotation_delta = Rotation.from_euler(
            "xyz",
            np.asarray([roll, pitch, yaw], dtype=float) * angle_scale,
        ).as_matrix()
        target = Pose(
            current.position + current.rotation @ translation_tool,
            current.rotation @ rotation_delta,
        )
        return self.move_linear(
            target,
            orientation_mode=orientation_mode,
            speed=None if speed is None else speed / 1000.0,
            acceleration=None if mvacc is None else mvacc / 1000.0,
            wait=wait,
        )

    def get_position_values(self, *, is_radian: bool = False) -> list[float]:
        """Return ``[x, y, z, roll, pitch, yaw]`` in mm and degrees/radians."""

        x, y, z, roll, pitch, yaw = self.get_position().xyz_rpy()
        angles = [roll, pitch, yaw]
        if not is_radian:
            angles = [value * 180.0 / pi for value in angles]
        return [x * 1000.0, y * 1000.0, z * 1000.0, *angles]

    def set_position_values(
        self,
        values: Sequence[float],
        *,
        speed: float | None = None,
        mvacc: float | None = None,
        wait: bool = True,
        relative: bool = False,
        is_radian: bool = False,
        orientation_mode: OrientationMode = "compatible",
    ) -> MotionResult | MotionHandle[MotionResult]:
        """Sequence form of ``set_position`` for xArm-like scripting."""

        data = [float(value) for value in values]
        if len(data) != 6:
            raise ValueError("expected [x, y, z, roll, pitch, yaw]")
        return self.set_position(
            x=data[0],
            y=data[1],
            z=data[2],
            roll=data[3],
            pitch=data[4],
            yaw=data[5],
            speed=speed,
            mvacc=mvacc,
            wait=wait,
            relative=relative,
            is_radian=is_radian,
            orientation_mode=orientation_mode,
        )
