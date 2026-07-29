"""Motion controller extensions for executing a validated linear plan once."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from soarm101_motion.kinematics import OrientationMode
from soarm101_motion.motion.controller import (
    MotionController as _BaseMotionController,
    MotionHandle,
    PlannedPath,
)
from soarm101_motion.types import MotionResult, Pose


class MotionController(_BaseMotionController):
    """Base controller plus safe reuse of the most recently inspected linear plan.

    ``SOARM101.move_linear`` asks for a plan first when workspace checks are enabled,
    validates every command sample, and then requests execution. The original base
    controller planned the same target again. This class retains exactly one inspected
    plan and reuses it only when all request parameters and measured starting joints
    still match.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._cached_linear: tuple[tuple[Any, ...], PlannedPath] | None = None

    @staticmethod
    def _pose_key(pose: Pose | None) -> tuple[float, ...] | None:
        if pose is None:
            return None
        return tuple(float(value) for value in pose.as_matrix().ravel())

    @classmethod
    def _linear_key(
        cls,
        target: Pose,
        *,
        tcp: Pose | None,
        orientation_mode: OrientationMode,
        look_at: np.ndarray | None,
        speed: float | None,
        acceleration: float | None,
    ) -> tuple[Any, ...]:
        return (
            cls._pose_key(target),
            cls._pose_key(tcp),
            orientation_mode,
            None if look_at is None else tuple(float(value) for value in look_at.ravel()),
            speed,
            acceleration,
        )

    def plan_linear(
        self,
        target: Pose,
        *,
        tcp: Pose | None = None,
        orientation_mode: OrientationMode = "compatible",
        look_at: np.ndarray | None = None,
        speed: float | None = None,
        acceleration: float | None = None,
    ) -> PlannedPath:
        with self._state_lock:
            self._ensure_idle_locked()
            plan = super().plan_linear(
                target,
                tcp=tcp,
                orientation_mode=orientation_mode,
                look_at=look_at,
                speed=speed,
                acceleration=acceleration,
            )
            key = self._linear_key(
                target,
                tcp=tcp,
                orientation_mode=orientation_mode,
                look_at=look_at,
                speed=speed,
                acceleration=acceleration,
            )
            self._cached_linear = (key, plan)
            return plan

    @staticmethod
    def _starts_at(
        plan: PlannedPath,
        present: Mapping[str, float],
        *,
        tolerance: float = 1e-6,
    ) -> bool:
        if not plan.joint_waypoints:
            return False
        start = plan.joint_waypoints[0]
        return all(abs(float(present[name]) - float(start[name])) <= tolerance for name in start)

    def move_linear(
        self,
        target: Pose,
        *,
        tcp: Pose | None = None,
        orientation_mode: OrientationMode = "compatible",
        look_at: np.ndarray | None = None,
        speed: float | None = None,
        acceleration: float | None = None,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        with self._state_lock:
            self._ensure_idle_locked()
            self._require_ready()
            key = self._linear_key(
                target,
                tcp=tcp,
                orientation_mode=orientation_mode,
                look_at=look_at,
                speed=speed,
                acceleration=acceleration,
            )
            cached = self._cached_linear
            self._cached_linear = None
            plan: PlannedPath | None = None
            if cached is not None and cached[0] == key:
                present = self.backend.read_joint_positions()
                if self._starts_at(cached[1], present):
                    plan = cached[1]
            if plan is None:
                plan = super().plan_linear(
                    target,
                    tcp=tcp,
                    orientation_mode=orientation_mode,
                    look_at=look_at,
                    speed=speed,
                    acceleration=acceleration,
                )
            handle = self._start_locked(
                lambda event: self._execute_plan(
                    plan,
                    event,
                    cancellation_message="linear motion cancelled",
                )
            )
        return handle.wait() if wait else handle
