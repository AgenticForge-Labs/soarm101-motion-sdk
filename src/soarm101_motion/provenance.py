"""Calibration provenance helpers for persisted motion artifacts."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from soarm101_motion.exceptions import CalibrationError

_PROVENANCE_KEYS = (
    "source_robot_id",
    "source_calibration_id",
    "target_robot_id",
    "target_calibration_id",
)


def provenance_subset(metadata: Mapping[str, Any]) -> dict[str, str]:
    """Return normalized calibration provenance keys present in metadata."""

    result: dict[str, str] = {}
    for key in _PROVENANCE_KEYS:
        value = metadata.get(key)
        if value is None:
            continue
        text = str(value).strip()
        if text:
            result[key] = text
    return result


def bind_target_calibration(
    metadata: Mapping[str, Any],
    *,
    robot_id: str,
    calibration_id: str,
) -> dict[str, Any]:
    """Return artifact metadata explicitly bound to one target robot calibration."""

    result = dict(metadata)
    result["target_robot_id"] = str(robot_id)
    result["target_calibration_id"] = str(calibration_id)
    return result


def require_calibration_compatibility(
    metadata: Mapping[str, Any],
    *,
    current_robot_id: str,
    current_calibration_id: str,
    artifact_label: str,
) -> None:
    """Fail closed when an artifact's calibration provenance is absent or stale.

    Same-robot artifacts can be validated from their source calibration alone. Cross-arm
    artifacts (for example, leader recordings replayed on a follower) must also carry an
    explicit target binding because the source and target calibrations are intentionally
    different physical measurements.
    """

    values = provenance_subset(metadata)
    source_robot = values.get("source_robot_id")
    source_calibration = values.get("source_calibration_id")
    target_robot = values.get("target_robot_id")
    target_calibration = values.get("target_calibration_id")

    if target_robot is not None or target_calibration is not None:
        if not target_robot or not target_calibration:
            raise CalibrationError(
                f"{artifact_label} has incomplete target calibration provenance; "
                "re-save or re-bind it before physical replay"
            )
        if target_robot != current_robot_id:
            raise CalibrationError(
                f"{artifact_label} is bound to robot {target_robot!r}, not "
                f"{current_robot_id!r}"
            )
        if target_calibration != current_calibration_id:
            raise CalibrationError(
                f"{artifact_label} was bound to calibration {target_calibration}, but "
                f"the active calibration is {current_calibration_id}; review and re-bind "
                "the artifact after recalibration"
            )
        return

    if source_robot == current_robot_id:
        if not source_calibration:
            raise CalibrationError(
                f"{artifact_label} predates calibration fingerprinting; re-capture or "
                "explicitly re-bind it before physical replay"
            )
        if source_calibration != current_calibration_id:
            raise CalibrationError(
                f"{artifact_label} was created under calibration {source_calibration}, "
                f"but the active calibration is {current_calibration_id}; review and "
                "re-bind it after recalibration"
            )
        return

    if source_robot and source_robot != current_robot_id:
        raise CalibrationError(
            f"{artifact_label} came from robot {source_robot!r} and has no target "
            f"calibration binding for {current_robot_id!r}; bind it before physical replay"
        )

    raise CalibrationError(
        f"{artifact_label} has no calibration provenance; re-capture or explicitly "
        "re-bind it before physical replay"
    )
