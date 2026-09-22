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
from soarm101_motion.constants import DEFAULT_TELEOP_STREAM_FREQUENCY_HZ
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

    def set_gripper_speed(self, speed: int) -> None:
        """Set the default gripper speed in raw STS3215 controller units."""

        resolved = int(speed)
        if not 1 <= resolved <= 4095:
            raise ValueError("gripper speed must be in [1, 4095]")
        self._gripper_speed_raw = resolved

    def get_gripper_speed(self) -> int:
        """Return the configured default gripper speed in raw controller units."""

        return int(getattr(self, "_gripper_speed_raw", self.config.hardware_speed_raw))

    def set_gripper_position(
        self,
        position: float,
        *,
        speed: int | None = None,
        acceleration: int | None = None,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        """Move the gripper to normalized position ``0.0 .. 1.0``.

        ``speed`` and ``acceleration`` are raw STS3215 controller units. If speed
        is omitted, the value set by :meth:`set_gripper_speed` is used.
        """

        move = getattr(self.tool, "move", None)
        if not callable(move):
            raise RuntimeError("active tool does not provide position control")
        speed_raw = self.get_gripper_speed() if speed is None else int(speed)
        if not 1 <= speed_raw <= 4095:
            raise ValueError("gripper speed must be in [1, 4095]")
        if acceleration is not None and not 1 <= int(acceleration) <= 254:
            raise ValueError("gripper acceleration must be in [1, 254]")
        return move(
            float(position),
            speed_raw=speed_raw,
            acceleration_raw=None if acceleration is None else int(acceleration),
            wait=wait,
        )

    def get_gripper_position(self) -> float:
        """Return the stock gripper's normalized position ``0.0 .. 1.0``."""

        getter = getattr(self.tool, "get_position", None)
        if not callable(getter):
            raise RuntimeError("active tool does not provide position feedback")
        return float(getter())

    def get_motor_effort(self, motor: str) -> dict[str, int]:
        """Return raw current and signed load feedback for one hardware motor."""

        reader = getattr(self.backend, "read_motor_effort", None)
        if not callable(reader):
            raise RuntimeError("active backend does not provide motor effort feedback")
        return dict(reader(motor))

    @property
    def effort_trip_message(self) -> str | None:
        """Return the latched motor-effort safety trip message, if any."""

        return getattr(self.backend, "effort_trip_message", None)

    def clear_effort_trip(self) -> None:
        """Clear a latched motor-effort interlock after removing the obstruction."""

        clear = getattr(self.backend, "clear_effort_trip", None)
        if not callable(clear):
            raise RuntimeError("active backend does not provide an effort safety interlock")
        clear()

    def servo_j_start(
        self,
        *,
        frequency_hz: float = DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
    ) -> None:
        """Begin guarded servo-joint style streaming at an explicit sample rate."""
        self.start_joint_stream(frequency_hz=frequency_hz)

    def servo_j(
        self,
        angles: Sequence[float],
        *,
        gripper: float | None = None,
        is_radian: bool = True,
    ) -> MotionResult:
        """Send one five-joint streaming sample.

        Unlike a normal point-to-point move this does not wait for settling.
        """
        values = [float(value) for value in angles]
        if len(values) != len(self.get_servo_angle()):
            raise ValueError("servo_j expects five joint values")
        scale = 1.0 if is_radian else pi / 180.0
        targets = {
            name: value * scale
            for name, value in zip(
                ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"),
                values,
                strict=True,
            )
        }
        return self.stream_joint_target(targets, gripper=gripper)

    def servo_j_stop(self, *, hold: bool = True) -> None:
        """End guarded servo-joint streaming."""
        self.stop_joint_stream(hold=hold)

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
