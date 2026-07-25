"""Stable SDK-owned state types."""

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class JointState:
    positions: dict[str, float]
    timestamp: float


@dataclass(frozen=True, slots=True)
class HardwareState:
    connected: bool
    torque_enabled: bool
    moving: bool
    faulted: bool
    fault_message: str | None = None


@dataclass(frozen=True, slots=True)
class MotionResult:
    accepted: bool
    completed: bool
    message: str | None = None
