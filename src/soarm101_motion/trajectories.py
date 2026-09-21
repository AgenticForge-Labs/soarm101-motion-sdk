"""Immutable recorded trajectories and on-disk trajectory libraries."""

from __future__ import annotations

import json
import math
import re
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from soarm101_motion.constants import ALL_MOTORS, ARM_JOINTS

FloatArray = NDArray[np.float64]
_SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


def _utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def _readonly(values: Any, *, ndim: int | None = None) -> FloatArray:
    array = np.asarray(values, dtype=float).copy()
    if ndim is not None and array.ndim != ndim:
        raise ValueError(f"expected a {ndim}D array, got shape {array.shape}")
    if not np.all(np.isfinite(array) | np.isnan(array)):
        raise ValueError("trajectory arrays must contain finite values or NaN diagnostics")
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class Trajectory:
    """One time-indexed SO-ARM101 demonstration.

    Joint positions are radians in canonical ARM_JOINTS order. Gripper values
    use the stock normalized convention (0=closed, 1=open). Optional effort channels
    are diagnostics only and are never replayed.
    """

    timestamps_s: FloatArray
    joints_rad: FloatArray
    gripper: FloatArray
    effort_current_raw: FloatArray | None = None
    effort_load_raw: FloatArray | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)
    created_at: str = field(default_factory=_utc_timestamp)

    def __post_init__(self) -> None:
        timestamps = _readonly(self.timestamps_s, ndim=1)
        joints = _readonly(self.joints_rad, ndim=2)
        gripper = _readonly(self.gripper, ndim=1)
        if len(timestamps) < 2:
            raise ValueError("trajectory requires at least two samples")
        if joints.shape != (len(timestamps), len(ARM_JOINTS)):
            raise ValueError(
                f"joints must have shape ({len(timestamps)}, {len(ARM_JOINTS)}), "
                f"got {joints.shape}"
            )
        if gripper.shape != (len(timestamps),):
            raise ValueError("gripper sample count must match timestamps")
        if not np.all(np.isfinite(timestamps)) or timestamps[0] < 0:
            raise ValueError("trajectory timestamps must be finite and non-negative")
        if not np.all(np.diff(timestamps) > 0):
            raise ValueError("trajectory timestamps must be strictly increasing")
        if not np.all(np.isfinite(joints)):
            raise ValueError("joint samples must be finite")
        if not np.all(np.isfinite(gripper)) or np.any((gripper < 0.0) | (gripper > 1.0)):
            raise ValueError("gripper samples must be finite and within [0, 1]")

        timestamps = _readonly(timestamps - timestamps[0], ndim=1)
        current = self._validate_effort(self.effort_current_raw, len(timestamps), "current")
        load = self._validate_effort(self.effort_load_raw, len(timestamps), "load")

        object.__setattr__(self, "timestamps_s", timestamps)
        object.__setattr__(self, "joints_rad", joints)
        object.__setattr__(self, "gripper", gripper)
        object.__setattr__(self, "effort_current_raw", current)
        object.__setattr__(self, "effort_load_raw", load)
        object.__setattr__(self, "metadata", dict(self.metadata))

    @staticmethod
    def _validate_effort(values: Any, count: int, label: str) -> FloatArray | None:
        if values is None:
            return None
        array = _readonly(values, ndim=2)
        expected = (count, len(ALL_MOTORS))
        if array.shape != expected:
            raise ValueError(f"{label} effort must have shape {expected}, got {array.shape}")
        return array

    @property
    def duration_s(self) -> float:
        return float(self.timestamps_s[-1])

    @property
    def sample_count(self) -> int:
        return len(self.timestamps_s)

    @property
    def sample_rate_hz(self) -> float:
        intervals = np.diff(self.timestamps_s)
        return float(1.0 / np.median(intervals))

    def joint_mappings(self) -> tuple[dict[str, float], ...]:
        return tuple(
            {name: float(row[index]) for index, name in enumerate(ARM_JOINTS)}
            for row in self.joints_rad
        )

    def _interpolate_matrix(self, matrix: FloatArray, times: FloatArray) -> FloatArray:
        columns = [
            np.interp(times, self.timestamps_s, matrix[:, index])
            for index in range(matrix.shape[1])
        ]
        return np.column_stack(columns)

    def _interpolate_vector(self, vector: FloatArray, times: FloatArray) -> FloatArray:
        return np.interp(times, self.timestamps_s, vector)

    def resample(self, frequency_hz: float) -> "Trajectory":
        frequency = float(frequency_hz)
        if not math.isfinite(frequency) or frequency <= 0:
            raise ValueError("frequency_hz must be positive and finite")
        count = max(2, int(math.ceil(self.duration_s * frequency)) + 1)
        times = np.linspace(0.0, self.duration_s, count)
        return replace(
            self,
            timestamps_s=times,
            joints_rad=self._interpolate_matrix(self.joints_rad, times),
            gripper=self._interpolate_vector(self.gripper, times),
            effort_current_raw=(
                None
                if self.effort_current_raw is None
                else self._interpolate_matrix(self.effort_current_raw, times)
            ),
            effort_load_raw=(
                None
                if self.effort_load_raw is None
                else self._interpolate_matrix(self.effort_load_raw, times)
            ),
            metadata={**self.metadata, "resampled_hz": frequency},
            created_at=_utc_timestamp(),
        )

    def crop(self, start_s: float, end_s: float) -> "Trajectory":
        start = float(start_s)
        end = float(end_s)
        if not math.isfinite(start) or not math.isfinite(end):
            raise ValueError("crop bounds must be finite")
        if start < 0 or end > self.duration_s or end <= start:
            raise ValueError(
                f"crop must satisfy 0 <= start < end <= {self.duration_s:.6f}"
            )
        interior = self.timestamps_s[
            (self.timestamps_s > start) & (self.timestamps_s < end)
        ]
        times = np.concatenate(([start], interior, [end]))
        parent = self.metadata.get("name") or self.metadata.get("source_name")
        edits = list(self.metadata.get("edits", []))
        edits.append({"op": "crop", "start_s": start, "end_s": end})
        return replace(
            self,
            timestamps_s=times - start,
            joints_rad=self._interpolate_matrix(self.joints_rad, times),
            gripper=self._interpolate_vector(self.gripper, times),
            effort_current_raw=(
                None
                if self.effort_current_raw is None
                else self._interpolate_matrix(self.effort_current_raw, times)
            ),
            effort_load_raw=(
                None
                if self.effort_load_raw is None
                else self._interpolate_matrix(self.effort_load_raw, times)
            ),
            metadata={
                **self.metadata,
                "derived_from": self.metadata.get("derived_from") or parent,
                "edits": edits,
                "markers": [
                    {**marker, "time_s": float(marker["time_s"]) - start}
                    for marker in self._markers()
                    if start <= float(marker["time_s"]) <= end
                ],
            },
            created_at=_utc_timestamp(),
        )

    def _markers(self) -> list[dict[str, Any]]:
        markers: list[dict[str, Any]] = []
        for item in self.metadata.get("markers", []):
            value = dict(item)
            try:
                at_s = float(value["time_s"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(at_s):
                value["time_s"] = at_s
                markers.append(value)
        return markers

    def retime(self, speed_scale: float) -> "Trajectory":
        scale = float(speed_scale)
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("speed_scale must be positive and finite")
        edits = list(self.metadata.get("edits", []))
        edits.append({"op": "speed_scale", "scale": scale})
        markers = [
            {**marker, "time_s": float(marker["time_s"]) / scale}
            for marker in self._markers()
        ]
        return replace(
            self,
            timestamps_s=self.timestamps_s / scale,
            metadata={**self.metadata, "edits": edits, "markers": markers},
            created_at=_utc_timestamp(),
        )

    def smooth(self, window_samples: int = 5) -> "Trajectory":
        """Centered moving-average smoothing for joint/gripper channels.

        Endpoints are preserved exactly so derived motion keeps the same start/end
        poses. Diagnostics are left untouched because they are observations, not commands.
        """
        window = int(window_samples)
        if window < 3 or window % 2 == 0:
            raise ValueError("window_samples must be an odd integer >= 3")
        if window > self.sample_count:
            raise ValueError("window_samples cannot exceed trajectory sample count")

        def smooth_array(values: FloatArray) -> FloatArray:
            radius = window // 2
            padded = np.pad(values, ((radius, radius), (0, 0)), mode="edge")
            kernel = np.ones(window, dtype=float) / window
            result = np.column_stack(
                [
                    np.convolve(padded[:, column], kernel, mode="valid")
                    for column in range(values.shape[1])
                ]
            )
            result[0] = values[0]
            result[-1] = values[-1]
            return result

        joints = smooth_array(self.joints_rad)
        gripper_matrix = smooth_array(self.gripper.reshape(-1, 1))
        gripper = np.clip(gripper_matrix[:, 0], 0.0, 1.0)
        edits = list(self.metadata.get("edits", []))
        edits.append({"op": "smooth", "window_samples": window})
        return replace(
            self,
            joints_rad=joints,
            gripper=gripper,
            metadata={**self.metadata, "edits": edits},
            created_at=_utc_timestamp(),
        )

    def delete_region(self, start_s: float, end_s: float) -> "Trajectory":
        """Remove a time region and splice the remaining samples together.

        Replay validation remains authoritative: a splice that creates too large a
        step/speed/acceleration transition is rejected before hardware commands.
        """
        start = float(start_s)
        end = float(end_s)
        if start <= 0 or end >= self.duration_s or end <= start:
            raise ValueError("delete region must be strictly inside the trajectory")

        left_mask = self.timestamps_s < start
        right_mask = self.timestamps_s > end
        left_times = self.timestamps_s[left_mask]
        right_times = self.timestamps_s[right_mask] - (end - start)
        times = np.concatenate((left_times, right_times))
        if len(times) < 2:
            raise ValueError("delete would leave fewer than two samples")

        joints = np.vstack((self.joints_rad[left_mask], self.joints_rad[right_mask]))
        gripper = np.concatenate((self.gripper[left_mask], self.gripper[right_mask]))

        def splice_optional(values: FloatArray | None) -> FloatArray | None:
            if values is None:
                return None
            return np.vstack((values[left_mask], values[right_mask]))

        markers: list[dict[str, Any]] = []
        removed = end - start
        for marker in self._markers():
            time_s = float(marker["time_s"])
            if start <= time_s <= end:
                continue
            markers.append(
                {
                    **marker,
                    "time_s": time_s if time_s < start else time_s - removed,
                }
            )
        edits = list(self.metadata.get("edits", []))
        edits.append({"op": "delete_region", "start_s": start, "end_s": end})
        return replace(
            self,
            timestamps_s=times,
            joints_rad=joints,
            gripper=gripper,
            effort_current_raw=splice_optional(self.effort_current_raw),
            effort_load_raw=splice_optional(self.effort_load_raw),
            metadata={**self.metadata, "edits": edits, "markers": markers},
            created_at=_utc_timestamp(),
        )

    def insert_hold(self, at_s: float, duration_s: float) -> "Trajectory":
        at = float(at_s)
        duration = float(duration_s)
        if not 0.0 <= at <= self.duration_s:
            raise ValueError("hold insertion time is outside trajectory")
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("hold duration must be positive and finite")

        before = self.timestamps_s < at
        after = self.timestamps_s > at
        joint_at = self._interpolate_matrix(self.joints_rad, np.array([at]))[0]
        gripper_at = float(self._interpolate_vector(self.gripper, np.array([at]))[0])

        times = np.concatenate(
            (
                self.timestamps_s[before],
                np.array([at, at + duration]),
                self.timestamps_s[after] + duration,
            )
        )
        joints = np.vstack(
            (
                self.joints_rad[before],
                joint_at,
                joint_at,
                self.joints_rad[after],
            )
        )
        gripper = np.concatenate(
            (
                self.gripper[before],
                np.array([gripper_at, gripper_at]),
                self.gripper[after],
            )
        )

        def insert_optional(values: FloatArray | None) -> FloatArray | None:
            if values is None:
                return None
            value_at = self._interpolate_matrix(values, np.array([at]))[0]
            return np.vstack((values[before], value_at, value_at, values[after]))

        markers = [
            {
                **marker,
                "time_s": (
                    float(marker["time_s"])
                    if float(marker["time_s"]) < at
                    else float(marker["time_s"]) + duration
                ),
            }
            for marker in self._markers()
        ]
        edits = list(self.metadata.get("edits", []))
        edits.append({"op": "insert_hold", "at_s": at, "duration_s": duration})
        return replace(
            self,
            timestamps_s=times,
            joints_rad=joints,
            gripper=gripper,
            effort_current_raw=insert_optional(self.effort_current_raw),
            effort_load_raw=insert_optional(self.effort_load_raw),
            metadata={**self.metadata, "edits": edits, "markers": markers},
            created_at=_utc_timestamp(),
        )

    def set_keyframe(self, at_s: float, channel: str, value: float) -> "Trajectory":
        """Insert or replace one exact channel value at a selected time."""
        at = float(at_s)
        target = float(value)
        if not math.isfinite(at) or not 0.0 <= at <= self.duration_s:
            raise ValueError("keyframe time is outside trajectory")
        if not math.isfinite(target):
            raise ValueError("keyframe value must be finite")
        valid_channels = {*ARM_JOINTS, "gripper"}
        if channel not in valid_channels:
            raise ValueError(f"unknown trajectory channel {channel!r}")
        if channel == "gripper" and not 0.0 <= target <= 1.0:
            raise ValueError("gripper keyframe must be within [0, 1]")

        exact = np.where(np.isclose(self.timestamps_s, at, atol=1e-9))[0]
        if exact.size:
            times = self.timestamps_s.copy()
            joints = self.joints_rad.copy()
            gripper = self.gripper.copy()
            index = int(exact[0])
            current = self.effort_current_raw
            load = self.effort_load_raw
        else:
            index = int(np.searchsorted(self.timestamps_s, at))
            times = np.insert(self.timestamps_s, index, at)
            interpolated_joint = self._interpolate_matrix(
                self.joints_rad, np.array([at])
            )[0]
            joints = np.insert(self.joints_rad, index, interpolated_joint, axis=0)
            interpolated_gripper = float(
                self._interpolate_vector(self.gripper, np.array([at]))[0]
            )
            gripper = np.insert(self.gripper, index, interpolated_gripper)

            def insert_diag(values: FloatArray | None) -> FloatArray | None:
                if values is None:
                    return None
                row = self._interpolate_matrix(values, np.array([at]))[0]
                return np.insert(values, index, row, axis=0)

            current = insert_diag(self.effort_current_raw)
            load = insert_diag(self.effort_load_raw)

        if channel == "gripper":
            gripper[index] = target
        else:
            joints[index, ARM_JOINTS.index(channel)] = target

        edits = list(self.metadata.get("edits", []))
        edits.append({"op": "set_keyframe", "time_s": at, "channel": channel, "value": target})
        return replace(
            self,
            timestamps_s=times,
            joints_rad=joints,
            gripper=gripper,
            effort_current_raw=current,
            effort_load_raw=load,
            metadata={**self.metadata, "edits": edits},
            created_at=_utc_timestamp(),
        )

    def add_marker(self, at_s: float, label: str, *, marker_type: str = "cue") -> "Trajectory":
        at = float(at_s)
        text = str(label).strip()
        kind = str(marker_type).strip() or "cue"
        if not math.isfinite(at) or not 0.0 <= at <= self.duration_s:
            raise ValueError("marker time is outside trajectory")
        if not text:
            raise ValueError("marker label must not be empty")
        markers = self._markers()
        markers.append({"time_s": at, "label": text, "type": kind})
        markers.sort(key=lambda item: float(item["time_s"]))
        edits = list(self.metadata.get("edits", []))
        edits.append({"op": "add_marker", "time_s": at, "label": text, "type": kind})
        return replace(
            self,
            metadata={**self.metadata, "edits": edits, "markers": markers},
            created_at=_utc_timestamp(),
        )

    def repeat(self, count: int) -> "Trajectory":
        repeats = int(count)
        if repeats < 1:
            raise ValueError("repeat count must be >= 1")
        if repeats == 1:
            return self
        epsilon = max(1e-9, float(np.min(np.diff(self.timestamps_s))) * 0.001)
        times: list[FloatArray] = []
        joints: list[FloatArray] = []
        gripper: list[FloatArray] = []
        current: list[FloatArray] = []
        load: list[FloatArray] = []
        markers: list[dict[str, Any]] = []
        offset = 0.0
        for index in range(repeats):
            source_times = self.timestamps_s if index == 0 else self.timestamps_s[1:]
            shifted = source_times + offset
            if index > 0:
                shifted = shifted + epsilon
            times.append(shifted)
            joints.append(self.joints_rad if index == 0 else self.joints_rad[1:])
            gripper.append(self.gripper if index == 0 else self.gripper[1:])
            if self.effort_current_raw is not None:
                current.append(
                    self.effort_current_raw
                    if index == 0
                    else self.effort_current_raw[1:]
                )
            if self.effort_load_raw is not None:
                load.append(
                    self.effort_load_raw if index == 0 else self.effort_load_raw[1:]
                )
            for marker in self._markers():
                markers.append(
                    {
                        **marker,
                        "time_s": float(marker["time_s"]) + offset + (epsilon if index else 0.0),
                    }
                )
            offset = float(shifted[-1])
        edits = list(self.metadata.get("edits", []))
        edits.append({"op": "repeat", "count": repeats})
        return replace(
            self,
            timestamps_s=np.concatenate(times),
            joints_rad=np.vstack(joints),
            gripper=np.concatenate(gripper),
            effort_current_raw=(np.vstack(current) if current else None),
            effort_load_raw=(np.vstack(load) if load else None),
            metadata={**self.metadata, "edits": edits, "markers": markers},
            created_at=_utc_timestamp(),
        )


@dataclass(frozen=True)
class TrajectoryEntry:
    name: str
    kind: str
    data_path: Path
    metadata_path: Path


def default_trajectory_library_path(robot_id: str) -> Path:
    return Path.home() / ".config" / "soarm101" / "trajectories" / robot_id


class TrajectoryLibrary:
    """Non-destructive raw and edited trajectory storage."""

    schema_version = 1

    def __init__(self, robot_id: str, *, root: str | Path | None = None) -> None:
        self.robot_id = str(robot_id).strip() or "so101"
        self.root = (
            Path(root).expanduser()
            if root is not None
            else default_trajectory_library_path(self.robot_id)
        )

    @staticmethod
    def _validate_name(name: str) -> str:
        value = str(name).strip()
        if not _SAFE_NAME.fullmatch(value):
            raise ValueError(
                "trajectory name must start with a letter/number and contain only "
                "letters, numbers, '.', '_' or '-'"
            )
        return value

    @staticmethod
    def _validate_kind(kind: str) -> str:
        value = str(kind).strip().lower()
        if value not in {"raw", "edited"}:
            raise ValueError("trajectory kind must be 'raw' or 'edited'")
        return value

    def _paths(self, name: str, kind: str) -> tuple[Path, Path]:
        safe = self._validate_name(name)
        category = self._validate_kind(kind)
        folder = self.root / category
        return folder / f"{safe}.npz", folder / f"{safe}.json"

    def save(self, name: str, trajectory: Trajectory, *, kind: str) -> TrajectoryEntry:
        safe = self._validate_name(name)
        category = self._validate_kind(kind)
        data_path, metadata_path = self._paths(safe, category)
        if data_path.exists() or metadata_path.exists():
            raise FileExistsError(
                f"{category} trajectory {safe!r} already exists; choose a new name"
            )
        data_path.parent.mkdir(parents=True, exist_ok=True)
        arrays: dict[str, FloatArray] = {
            "timestamps_s": trajectory.timestamps_s,
            "joints_rad": trajectory.joints_rad,
            "gripper": trajectory.gripper,
        }
        if trajectory.effort_current_raw is not None:
            arrays["effort_current_raw"] = trajectory.effort_current_raw
        if trajectory.effort_load_raw is not None:
            arrays["effort_load_raw"] = trajectory.effort_load_raw
        np.savez_compressed(data_path, **arrays)

        metadata = {
            "schema_version": self.schema_version,
            "robot_id": self.robot_id,
            "name": safe,
            "kind": category,
            "created_at": trajectory.created_at,
            "duration_s": trajectory.duration_s,
            "sample_count": trajectory.sample_count,
            "joint_order": list(ARM_JOINTS),
            "effort_motor_order": list(ALL_MOTORS),
            "metadata": {**trajectory.metadata, "name": safe},
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        return TrajectoryEntry(safe, category, data_path, metadata_path)

    def entries(self) -> tuple[TrajectoryEntry, ...]:
        found: list[TrajectoryEntry] = []
        for kind in ("raw", "edited"):
            folder = self.root / kind
            if not folder.is_dir():
                continue
            for metadata_path in sorted(folder.glob("*.json")):
                name = metadata_path.stem
                data_path = folder / f"{name}.npz"
                if data_path.is_file():
                    found.append(TrajectoryEntry(name, kind, data_path, metadata_path))
        return tuple(found)

    def load(self, name: str, *, kind: str) -> Trajectory:
        safe = self._validate_name(name)
        category = self._validate_kind(kind)
        data_path, metadata_path = self._paths(safe, category)
        if not data_path.is_file() or not metadata_path.is_file():
            raise FileNotFoundError(f"{category} trajectory {safe!r} was not found")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        with np.load(data_path, allow_pickle=False) as arrays:
            return Trajectory(
                timestamps_s=arrays["timestamps_s"],
                joints_rad=arrays["joints_rad"],
                gripper=arrays["gripper"],
                effort_current_raw=(
                    arrays["effort_current_raw"]
                    if "effort_current_raw" in arrays.files
                    else None
                ),
                effort_load_raw=(
                    arrays["effort_load_raw"]
                    if "effort_load_raw" in arrays.files
                    else None
                ),
                metadata={**metadata.get("metadata", {}), "name": safe, "kind": category},
                created_at=str(metadata.get("created_at") or _utc_timestamp()),
            )
