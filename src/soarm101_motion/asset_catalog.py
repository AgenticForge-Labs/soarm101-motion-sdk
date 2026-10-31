"""Read-only descriptors for explicitly selected SO-ARM asset libraries.

This catalog never connects to hardware and does not authorize replay. Revisions
are discovery identities only; a replay implementation must re-resolve and
validate the complete asset, its dependencies, calibration and broker authority.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal
from urllib.parse import quote

from soarm101_motion.exceptions import CalibrationError
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.provenance import provenance_subset, require_calibration_compatibility
from soarm101_motion.sequences import SequenceLibrary
from soarm101_motion.trajectories import TrajectoryLibrary


@dataclass(frozen=True)
class AssetDescriptor:
    id: str
    kind: Literal["pose", "recording", "sequence"]
    source: str
    revision: str
    description: str
    duration_s: float | None
    resources: tuple[str, ...]
    provenance: tuple[tuple[str, str], ...]
    compatibility: Literal["calibration_matched", "calibration_mismatch", "unchecked"]
    compatibility_detail: str


def _revision(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return "sha256:" + hashlib.sha256(encoded).hexdigest()


def _file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _compatibility(
    metadata: dict[str, Any], robot_id: str | None, calibration_id: str | None
) -> tuple[Literal["calibration_matched", "calibration_mismatch", "unchecked"], str]:
    if not robot_id or not calibration_id:
        return "unchecked", "current robot and calibration are not supplied"
    try:
        require_calibration_compatibility(
            metadata, current_robot_id=robot_id, current_calibration_id=calibration_id,
            artifact_label="catalog asset",
        )
    except CalibrationError as exc:
        return "calibration_mismatch", str(exc)
    return "calibration_matched", "calibration provenance matches supplied identity only"


def discover_assets(
    poses: PoseLibrary,
    trajectories: TrajectoryLibrary,
    sequences: SequenceLibrary,
    *,
    current_robot_id: str | None = None,
    current_calibration_id: str | None = None,
) -> tuple[AssetDescriptor, ...]:
    """Read a deterministic snapshot from caller-selected libraries, without hardware access."""
    if len({poses.robot_id, trajectories.robot_id, sequences.robot_id}) != 1:
        raise ValueError("asset libraries must belong to the same robot")
    pose_snapshot = poses.path.read_bytes() if poses.path.is_file() else None
    poses.reload()
    if (poses.path.read_bytes() if poses.path.is_file() else None) != pose_snapshot:
        raise RuntimeError("pose library changed while catalog was being read")
    raw_poses = json.loads(pose_snapshot).get("poses", {}) if pose_snapshot is not None else {}
    sequences.reload()
    result: list[AssetDescriptor] = []
    for name in poses.names():
        # Use persisted values for identity: legacy poses may lack created_at,
        # which SavedPose.from_mapping otherwise fills from the current clock.
        data = raw_poses[name]
        status, detail = _compatibility(data, current_robot_id, current_calibration_id)
        result.append(AssetDescriptor(
            id=f"soarm101/pose/{quote(name, safe='')}", kind="pose", source="pose-library",
            revision=_revision(data), description="Saved pose", duration_s=None,
            resources=("arm.motion", "gripper"),
            provenance=tuple(sorted(provenance_subset(data).items())),
            compatibility=status, compatibility_detail=detail,
        ))
    for entry in trajectories.entries():
        before = (_file_digest(entry.data_path), _file_digest(entry.metadata_path))
        trajectory = trajectories.load(entry.name, kind=entry.kind)
        after = (_file_digest(entry.data_path), _file_digest(entry.metadata_path))
        if before != after:
            raise RuntimeError("trajectory changed while catalog was being read")
        status, detail = _compatibility(
            dict(trajectory.metadata), current_robot_id, current_calibration_id
        )
        result.append(AssetDescriptor(
            id=f"soarm101/recording/{entry.kind}/{quote(entry.name, safe='')}",
            kind="recording", source=f"trajectory-library/{entry.kind}",
            revision=_revision({"data_sha256": before[0], "metadata_sha256": before[1]}),
            description="Recorded joint and gripper trajectory", duration_s=trajectory.duration_s,
            resources=("arm.motion", "gripper"),
            provenance=tuple(sorted(provenance_subset(trajectory.metadata).items())),
            compatibility=status, compatibility_detail=detail,
        ))
    for name in sequences.names():
        sequence = sequences.require(name)
        data = sequence.to_mapping()
        result.append(AssetDescriptor(
            id=f"soarm101/sequence/{quote(name, safe='')}", kind="sequence",
            source="sequence-library", revision=_revision(data), description=sequence.description,
            duration_s=None, resources=("arm.motion", "gripper"),
            provenance=tuple(sorted(provenance_subset(sequence.metadata).items())),
            compatibility="unchecked",
            compatibility_detail="sequence dependencies and individual steps require validation",
        ))
    return tuple(sorted(result, key=lambda item: item.id))
