"""Stock SO-101 gripper tool."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Mapping

from soarm101_motion.constants import STOCK_GRIPPER
from soarm101_motion.hardware.base import SO101HardwareBackend
from soarm101_motion.kinematics import DEFAULT_GRIPPER_TCP
from soarm101_motion.tools.base import RobotTool
from soarm101_motion.types import MotionResult, Pose


@dataclass
class SO101Gripper(RobotTool):
    """Position-controlled stock moving-jaw gripper.

    The normalized convention is ``0.0 = closed`` and ``1.0 = open``. Motor
    direction and raw encoder ranges are handled by calibration.
    """

    backend: SO101HardwareBackend | None = None
    name: str = STOCK_GRIPPER
    open_position: float = 1.0
    closed_position: float = 0.0
    default_timeout_s: float = 3.0

    @property
    def tcp_frames(self) -> Mapping[str, Pose]:
        return {"gripper": DEFAULT_GRIPPER_TCP, "tool": DEFAULT_GRIPPER_TCP}

    def bind(self, backend: SO101HardwareBackend) -> None:
        self.backend = backend

    def _backend(self) -> SO101HardwareBackend:
        if self.backend is None:
            raise RuntimeError("gripper is not bound to a robot backend")
        return self.backend

    def get_position(self) -> float:
        return self._backend().read_tool_position(STOCK_GRIPPER)

    def move(
        self,
        position: float,
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
        wait: bool = True,
        timeout: float | None = None,
        tolerance: float = 0.03,
    ) -> MotionResult:
        target = min(1.0, max(0.0, float(position)))
        backend = self._backend()
        backend.write_tool_position(
            STOCK_GRIPPER,
            target,
            speed_raw=speed_raw,
            acceleration_raw=acceleration_raw,
        )
        if wait:
            deadline = time.monotonic() + (timeout or self.default_timeout_s)
            while time.monotonic() < deadline:
                if abs(backend.read_tool_position(STOCK_GRIPPER) - target) <= tolerance:
                    return MotionResult(True, True, final_positions={STOCK_GRIPPER: target})
                if not getattr(backend, "realtime", True):
                    break
                time.sleep(0.02)
        return MotionResult(True, not wait or abs(self.get_position() - target) <= tolerance)

    def open(self, **kwargs: object) -> MotionResult:
        return self.move(self.open_position, **kwargs)

    def close(self, **kwargs: object) -> MotionResult:
        return self.move(self.closed_position, **kwargs)

    @property
    def is_open(self) -> bool:
        return abs(self.get_position() - self.open_position) <= 0.05

    @property
    def is_closed(self) -> bool:
        return abs(self.get_position() - self.closed_position) <= 0.05
