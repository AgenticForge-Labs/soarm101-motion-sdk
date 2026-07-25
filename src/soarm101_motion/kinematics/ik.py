"""Bounded numerical inverse kinematics for the SO-ARM101."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Mapping

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from soarm101_motion.exceptions import IKError
from soarm101_motion.kinematics.model import SO101KinematicModel
from soarm101_motion.types import IKResult, Pose

FloatArray = NDArray[np.float64]
OrientationMode = Literal["exact", "compatible", "position_only", "look_at"]


@dataclass(frozen=True)
class IKOptions:
    orientation_mode: OrientationMode = "compatible"
    look_at: FloatArray | None = None
    position_tolerance_m: float = 0.002
    orientation_tolerance_rad: float = 0.06
    position_weight: float = 100.0
    orientation_weight: float = 1.0
    continuity_weight: float = 0.02
    joint_center_weight: float = 0.002
    max_evaluations: int = 500
    multi_start: bool = True


class IKSolver:
    def __init__(self, model: SO101KinematicModel | None = None) -> None:
        self.model = model or SO101KinematicModel()

    def _orientation_residual(
        self,
        actual: Pose,
        target: Pose,
        options: IKOptions,
    ) -> FloatArray:
        if options.orientation_mode == "position_only":
            return np.zeros(0)
        if options.orientation_mode == "exact":
            return Rotation.from_matrix(target.rotation @ actual.rotation.T).as_rotvec()
        if options.orientation_mode == "look_at":
            if options.look_at is None:
                raise ValueError("look_at mode requires a target point")
            direction = np.asarray(options.look_at, dtype=float).reshape(3) - actual.position
            norm = np.linalg.norm(direction)
            if norm < 1e-9:
                return np.zeros(3)
            desired_axis = direction / norm
            actual_axis = actual.rotation[:, 2]
            return np.cross(actual_axis, desired_axis)
        approach = np.cross(actual.rotation[:, 2], target.rotation[:, 2])
        lateral = 0.15 * np.cross(actual.rotation[:, 0], target.rotation[:, 0])
        return approach + lateral

    def solve(
        self,
        target: Pose,
        *,
        seed: Mapping[str, float] | FloatArray,
        tcp: Pose | None = None,
        options: IKOptions | None = None,
    ) -> IKResult:
        options = options or IKOptions()
        seed_vector = np.clip(
            self.model.vector(seed),
            self.model.lower_bounds,
            self.model.upper_bounds,
        )
        center = (self.model.lower_bounds + self.model.upper_bounds) / 2.0
        span = self.model.upper_bounds - self.model.lower_bounds

        def residual(q: FloatArray) -> FloatArray:
            actual = self.model.forward(q, tcp=tcp)
            position = (actual.position - target.position) * options.position_weight
            orientation = (
                self._orientation_residual(actual, target, options) * options.orientation_weight
            )
            continuity = (q - seed_vector) * options.continuity_weight
            centered = ((q - center) / span) * options.joint_center_weight
            return np.concatenate([position, orientation, continuity, centered])

        starts = [seed_vector]
        if options.multi_start:
            starts.extend(
                [
                    center,
                    np.clip(
                        seed_vector + np.array([0.2, -0.3, 0.3, 0.0, 0.0]),
                        self.model.lower_bounds,
                        self.model.upper_bounds,
                    ),
                    np.clip(
                        seed_vector + np.array([-0.2, 0.3, -0.3, 0.0, 0.0]),
                        self.model.lower_bounds,
                        self.model.upper_bounds,
                    ),
                    np.clip(
                        center + 0.20 * span * np.array([1, -1, 1, -1, 0]),
                        self.model.lower_bounds,
                        self.model.upper_bounds,
                    ),
                    np.clip(
                        center + 0.20 * span * np.array([-1, 1, -1, 1, 0]),
                        self.model.lower_bounds,
                        self.model.upper_bounds,
                    ),
                ]
            )

        best = None
        best_cost = float("inf")
        for start in starts:
            result = least_squares(
                residual,
                x0=start,
                bounds=(self.model.lower_bounds, self.model.upper_bounds),
                method="trf",
                xtol=1e-10,
                ftol=1e-10,
                gtol=1e-10,
                max_nfev=options.max_evaluations,
            )
            if result.cost < best_cost:
                best = result
                best_cost = float(result.cost)

        assert best is not None
        actual = self.model.forward(best.x, tcp=tcp)
        position_error = float(np.linalg.norm(actual.position - target.position))
        orientation_error = float(
            np.linalg.norm(self._orientation_residual(actual, target, options))
        )
        orientation_ok = (
            options.orientation_mode == "position_only"
            or orientation_error <= options.orientation_tolerance_rad
        )
        success = bool(
            best.success
            and position_error <= options.position_tolerance_m
            and orientation_ok
        )
        message = str(best.message)
        if not success:
            message = (
                f"IK did not meet tolerance: position={position_error:.6f} m, "
                f"orientation={orientation_error:.6f} rad; optimizer={best.message}"
            )
        return IKResult(
            success=success,
            joints=self.model.mapping(best.x),
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            iterations=int(best.nfev),
            message=message,
        )

    def solve_or_raise(self, *args: object, **kwargs: object) -> IKResult:
        result = self.solve(*args, **kwargs)  # type: ignore[arg-type]
        if not result.success:
            raise IKError(result.message)
        return result
