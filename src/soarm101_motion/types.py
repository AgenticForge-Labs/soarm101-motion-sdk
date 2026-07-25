"""Stable SDK-owned data types."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class Pose:
    """Rigid transform represented by position and a 3x3 rotation matrix."""

    position: FloatArray
    rotation: FloatArray

    def __post_init__(self) -> None:
        position = np.asarray(self.position, dtype=float).reshape(3)
        rotation = np.asarray(self.rotation, dtype=float).reshape(3, 3)
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "rotation", rotation)

    @classmethod
    def identity(cls) -> "Pose":
        return cls(np.zeros(3), np.eye(3))

    @classmethod
    def from_xyz_rpy(
        cls,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> "Pose":
        rotation = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
        return cls(np.array([x, y, z], dtype=float), rotation)

    @classmethod
    def from_matrix(cls, matrix: FloatArray) -> "Pose":
        matrix = np.asarray(matrix, dtype=float).reshape(4, 4)
        return cls(matrix[:3, 3], matrix[:3, :3])

    def as_matrix(self) -> FloatArray:
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.position
        return matrix

    def xyz_rpy(self) -> tuple[float, float, float, float, float, float]:
        roll, pitch, yaw = Rotation.from_matrix(self.rotation).as_euler("xyz")
        x, y, z = self.position
        return float(x), float(y), float(z), float(roll), float(pitch), float(yaw)

    def inverse(self) -> "Pose":
        rotation_t = self.rotation.T
        return Pose(-(rotation_t @ self.position), rotation_t)

    def compose(self, other: "Pose") -> "Pose":
        return Pose(
            self.position + self.rotation @ other.position,
            self.rotation @ other.rotation,
        )


@dataclass(frozen=True)
class JointState:
    positions: Mapping[str, float]
    timestamp: float
    velocities: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class HardwareState:
    connected: bool
    torque_enabled: bool
    moving: bool
    faulted: bool
    fault_message: str | None = None


@dataclass(frozen=True)
class MotionResult:
    accepted: bool
    completed: bool
    message: str | None = None
    final_positions: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class IKResult:
    success: bool
    joints: Mapping[str, float]
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    message: str


@dataclass(frozen=True)
class MotorDiagnostic:
    name: str
    motor_id: int
    model_number: int | None
    position_raw: int | None
    temperature_c: float | None
    voltage_v: float | None
    current_raw: int | None
    moving: bool | None
    status: int | None
    error: str | None = None
