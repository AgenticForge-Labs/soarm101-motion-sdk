"""Stock SO-101 gripper tool."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from math import isfinite
from typing import Mapping

from soarm101_motion.constants import STOCK_GRIPPER
from soarm101_motion.exceptions import (
    HardwareFaultError,
    InvalidCommandError,
    MotionCancelledError,
    MotionTimeoutError,
)
from soarm101_motion.hardware.base import SO101HardwareBackend
from soarm101_motion.kinematics import DEFAULT_GRIPPER_TCP
from soarm101_motion.motion.controller import MotionHandle
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
    default_timeout_s: float = 10.0
    _state_lock: threading.RLock = field(
        default_factory=threading.RLock, init=False, repr=False
    )
    _active_handle: MotionHandle[MotionResult] | None = field(
        default=None, init=False, repr=False
    )

    @property
    def tcp_frames(self) -> Mapping[str, Pose]:
        return {"gripper": DEFAULT_GRIPPER_TCP, "tool": DEFAULT_GRIPPER_TCP}

    @property
    def is_moving(self) -> bool:
        with self._state_lock:
            return self._active_handle is not None and not self._active_handle.done

    def bind(self, backend: SO101HardwareBackend) -> None:
        self.backend = backend

    def _backend(self) -> SO101HardwareBackend:
        if self.backend is None:
            raise RuntimeError("gripper is not bound to a robot backend")
        return self.backend

    def _clear_handle(self, handle: MotionHandle[MotionResult]) -> None:
        with self._state_lock:
            if self._active_handle is handle:
                self._active_handle = None

    def get_position(self) -> float:
        return self._backend().read_tool_position(STOCK_GRIPPER)

    def begin_opening(
        self,
        position: float,
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
    ) -> None:
        """Send one opening goal; callers must verify arrival before relying on it.

        This avoids a second serial polling loop while an arm trajectory runs.
        The backend's shared transport lock serializes this write with joint commands.
        """
        target = float(position)
        if not isfinite(target) or not 0.0 <= target <= 1.0:
            raise InvalidCommandError("gripper opening target must be within [0, 1]")
        if speed_raw is not None and (
            not isinstance(speed_raw, int) or not 0 <= speed_raw <= 4095
        ):
            raise InvalidCommandError("gripper speed must be an integer within [0, 4095]")
        if acceleration_raw is not None and (
            not isinstance(acceleration_raw, int) or not 0 <= acceleration_raw <= 254
        ):
            raise InvalidCommandError("gripper acceleration must be an integer within [0, 254]")
        if self.is_moving:
            raise InvalidCommandError("another gripper motion is already active")
        current = self.get_position()
        if target < current:
            raise InvalidCommandError("begin_opening cannot command the gripper to close")
        self._backend().write_tool_position(
            STOCK_GRIPPER,
            target,
            speed_raw=speed_raw,
            acceleration_raw=acceleration_raw,
        )

    def _execute_move(
        self,
        target: float,
        cancel_event: threading.Event,
        *,
        speed_raw: int | None,
        acceleration_raw: int | None,
        timeout: float,
        tolerance: float,
    ) -> MotionResult:
        backend = self._backend()
        try:
            if cancel_event.is_set():
                raise MotionCancelledError("gripper motion cancelled")
            backend.write_tool_position(
                STOCK_GRIPPER,
                target,
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
            )
            deadline = time.monotonic() + timeout
            last_progress = time.monotonic()
            best_error = float("inf")
            while time.monotonic() < deadline:
                if cancel_event.is_set():
                    raise MotionCancelledError("gripper motion cancelled")
                state = backend.get_hardware_state()
                if state.faulted:
                    raise HardwareFaultError(state.fault_message or "robot faulted during gripper move")
                actual = backend.read_tool_position(STOCK_GRIPPER)
                error = abs(actual - target)
                if error <= tolerance:
                    return MotionResult(True, True, final_positions={STOCK_GRIPPER: actual})
                if error < best_error - 0.005:
                    best_error = error
                    last_progress = time.monotonic()
                elif getattr(backend, "realtime", True) and time.monotonic() - last_progress > 1.5:
                    raise MotionTimeoutError(
                        f"gripper stopped making progress toward {target:.3f}; "
                        f"actual position is {actual:.3f}"
                    )
                if not getattr(backend, "realtime", True):
                    break
                time.sleep(0.02)
            actual = backend.read_tool_position(STOCK_GRIPPER)
            raise MotionTimeoutError(
                f"gripper did not reach {target:.3f} within {timeout:.2f}s; "
                f"actual position is {actual:.3f}"
            )
        except BaseException:
            try:
                backend.stop()
            except Exception:
                pass
            raise

    def move(
        self,
        position: float,
        *,
        speed_raw: int | None = None,
        acceleration_raw: int | None = None,
        wait: bool = True,
        timeout: float | None = None,
        tolerance: float = 0.03,
    ) -> MotionResult | MotionHandle[MotionResult]:
        target = float(position)
        if not 0.0 <= target <= 1.0:
            raise InvalidCommandError(f"gripper position {target} is outside [0, 1]")
        if tolerance <= 0:
            raise InvalidCommandError("gripper tolerance must be positive")
        resolved_timeout = self.default_timeout_s if timeout is None else float(timeout)
        if resolved_timeout <= 0:
            raise InvalidCommandError("gripper timeout must be positive")
        with self._state_lock:
            if self._active_handle is not None and not self._active_handle.done:
                raise InvalidCommandError("another gripper motion is already active")
            handle = MotionHandle(
                lambda event: self._execute_move(
                    target,
                    event,
                    speed_raw=speed_raw,
                    acceleration_raw=acceleration_raw,
                    timeout=resolved_timeout,
                    tolerance=tolerance,
                ),
                on_done=self._clear_handle,
            )
            self._active_handle = handle
            handle.start()
        return handle.wait() if wait else handle

    def stop(self, *, wait: bool = True, timeout: float = 2.0) -> None:
        with self._state_lock:
            handle = self._active_handle
            if handle is not None and not handle.done:
                handle.cancel()
        self._backend().stop()
        if wait and handle is not None and not handle.done:
            try:
                handle.wait(timeout)
            except MotionCancelledError:
                pass

    def open(self, **kwargs: object) -> MotionResult | MotionHandle[MotionResult]:
        return self.move(self.open_position, **kwargs)

    def close(self, **kwargs: object) -> MotionResult | MotionHandle[MotionResult]:
        return self.move(self.closed_position, **kwargs)

    @property
    def is_open(self) -> bool:
        return abs(self.get_position() - self.open_position) <= 0.05

    @property
    def is_closed(self) -> bool:
        return abs(self.get_position() - self.closed_position) <= 0.05
