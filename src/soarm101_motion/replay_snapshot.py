"""Pinned, bounded in-memory recording snapshots for future replay admission."""
from __future__ import annotations

import hashlib
import io
import json
import math
import zipfile
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import quote

import numpy as np

from soarm101_motion.asset_catalog import _revision
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.replay_policy import ReplayPolicy
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary

_MAX_DATA_BYTES = 32 * 1024 * 1024
_MAX_EXPANDED_BYTES = 128 * 1024 * 1024
_MAX_METADATA_BYTES = 1024 * 1024
_REQUIRED = {"timestamps_s.npy", "joints_rad.npy", "gripper.npy"}
_OPTIONAL = {"effort_current_raw.npy", "effort_load_raw.npy"}


def _read(path: Path, limit: int) -> bytes:
    if path.is_symlink() or not path.is_file():
        raise ValueError("recording files must be regular non-symlink files")
    with path.open("rb") as stream:
        data = stream.read(limit + 1)
    if len(data) > limit:
        raise ValueError("recording file exceeds size limit")
    return data


@dataclass(frozen=True)
class RecordingSnapshot:
    asset_id: str
    revision: str
    data_bytes: bytes
    metadata_bytes: bytes
    robot_id: str
    name: str
    kind: str

    def trajectory(self) -> Trajectory:
        """Construct a fresh validated trajectory from the exact pinned bytes."""
        revision = _revision({
            "data_sha256": hashlib.sha256(self.data_bytes).hexdigest(),
            "metadata_sha256": hashlib.sha256(self.metadata_bytes).hexdigest(),
        })
        if revision != self.revision:
            raise ValueError("snapshot content differs from pinned revision")
        if len(self.data_bytes) > _MAX_DATA_BYTES or len(self.metadata_bytes) > _MAX_METADATA_BYTES:
            raise ValueError("recording snapshot exceeds size limit")
        def reject_constant(value: str) -> None:
            raise ValueError(f"invalid non-finite metadata value: {value}")

        metadata = json.loads(self.metadata_bytes, parse_constant=reject_constant)
        expected_id = f"soarm101/recording/{self.kind}/{quote(self.name, safe='')}"
        if (self.asset_id != expected_id or not isinstance(metadata, dict)
                or type(metadata.get("schema_version")) is not int
                or metadata.get("schema_version") != 1
                or metadata.get("robot_id") != self.robot_id or metadata.get("name") != self.name
                or metadata.get("kind") != self.kind or metadata.get("joint_order") != list(ARM_JOINTS)
                or not isinstance(metadata.get("metadata"), dict)):
            raise ValueError("recording metadata identity or schema mismatch")
        with zipfile.ZipFile(io.BytesIO(self.data_bytes)) as archive:
            entries = archive.infolist()
            names = [item.filename for item in entries]
            if (len(names) != len(set(names)) or not _REQUIRED.issubset(names)
                    or set(names) - _REQUIRED - _OPTIONAL
                    or sum(item.file_size for item in entries) > _MAX_EXPANDED_BYTES):
                raise ValueError("recording archive has unsupported or oversized arrays")
        with np.load(io.BytesIO(self.data_bytes), allow_pickle=False) as arrays:
            trajectory = Trajectory(
                timestamps_s=arrays["timestamps_s"], joints_rad=arrays["joints_rad"],
                gripper=arrays["gripper"],
                effort_current_raw=arrays["effort_current_raw"] if "effort_current_raw" in arrays else None,
                effort_load_raw=arrays["effort_load_raw"] if "effort_load_raw" in arrays else None,
                metadata={**metadata["metadata"], "name": self.name, "kind": self.kind},
                created_at=str(metadata.get("created_at") or "unknown"),
            )
        duration, samples = metadata.get("duration_s"), metadata.get("sample_count")
        if (type(samples) is not int or samples != trajectory.sample_count
                or isinstance(duration, bool) or not isinstance(duration, (int, float))
                or not math.isfinite(duration)
                or not math.isclose(duration, trajectory.duration_s, rel_tol=1e-9, abs_tol=1e-9)):
            raise ValueError("recording metadata does not match trajectory samples")
        return trajectory


def load_pinned_recording(
    policy: ReplayPolicy, asset_id: str, library: TrajectoryLibrary,
) -> RecordingSnapshot:
    """Resolve only an enumerated operator library ID and verify its pinned bytes."""
    if library.robot_id != policy.robot_id:
        raise PermissionError("recording library does not match policy robot")
    entries = [entry for entry in library.entries()
               if f"soarm101/recording/{entry.kind}/{quote(entry.name, safe='')}" == asset_id]
    if len(entries) != 1:
        raise ValueError("recording identity is missing or ambiguous")
    entry = entries[0]
    root = library.root.resolve()
    for path in (entry.data_path, entry.metadata_path):
        if path.is_symlink():
            raise ValueError("recording files must be regular non-symlink files")
        if path.parent.is_symlink() or not path.resolve().is_relative_to(root):
            raise ValueError("recording must remain inside the operator-selected library")
    data, metadata = _read(entry.data_path, _MAX_DATA_BYTES), _read(entry.metadata_path, _MAX_METADATA_BYTES)
    revision = _revision({"data_sha256": hashlib.sha256(data).hexdigest(),
                          "metadata_sha256": hashlib.sha256(metadata).hexdigest()})
    policy.require_asset(asset_id, revision)
    snapshot = RecordingSnapshot(asset_id, revision, data, metadata,
                                 library.robot_id, entry.name, entry.kind)
    snapshot.trajectory()  # Validate the entire snapshot before returning it.
    return snapshot
