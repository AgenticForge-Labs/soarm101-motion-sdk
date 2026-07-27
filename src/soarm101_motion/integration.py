"""Stable metadata used by Agentic Forge consumer adapters."""

from __future__ import annotations

from dataclasses import dataclass

DIRECTOR_CONTRACT_FAMILY = "agenticforge.robotics"
DIRECTOR_CONTRACT_VERSION = "0.2.0"
DEFAULT_BASE_FRAME = "soarm101/base"
DEFAULT_RESOURCE_ID = "motion-platform:soarm101"
SOFTWARE_STOP_IS_CERTIFIED_EMERGENCY_STOP = False


@dataclass(frozen=True, slots=True)
class IntegrationMetadata:
    """Cross-repository facts that adapters must not infer independently."""

    robot_id: str
    base_frame: str
    resource_id: str
    position_unit: str = "meters"
    joint_angle_unit: str = "radians"
    duration_unit: str = "seconds"
    orientation_representation: str = "3x3 rotation matrix"
    nonblocking_motion_handles: bool = True
    certified_emergency_stop: bool = SOFTWARE_STOP_IS_CERTIFIED_EMERGENCY_STOP


def integration_metadata(
    robot_id: str = "soarm101",
    *,
    base_frame: str = DEFAULT_BASE_FRAME,
    resource_id: str | None = None,
) -> IntegrationMetadata:
    """Return metadata for one configured arm instance."""

    if not robot_id.strip():
        raise ValueError("robot_id must be non-empty")
    if not base_frame.strip():
        raise ValueError("base_frame must be non-empty")
    resolved_resource = resource_id or f"motion-platform:{robot_id}"
    if not resolved_resource.strip():
        raise ValueError("resource_id must be non-empty")
    return IntegrationMetadata(
        robot_id=robot_id,
        base_frame=base_frame,
        resource_id=resolved_resource,
    )
