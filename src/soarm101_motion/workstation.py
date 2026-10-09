"""Machine-local SO-ARM101 workstation profile shared by GUI, CLI, and agents.

The profile records how this workstation addresses its arms and cameras. Calibration
contents remain authoritative in the calibration files themselves; this profile stores
only the selected robot IDs, ports, and calibration paths.
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Mapping

from soarm101_motion.calibration import default_calibration_path
from soarm101_motion.camera import (
    DEFAULT_CAMERA_CONFIG_PATH,
    CameraSettings,
    CameraSettingsStore,
)


WORKSTATION_SCHEMA_VERSION = 1


def default_workstation_profile_path() -> Path:
    override = os.environ.get("SOARM101_WORKSTATION_CONFIG", "").strip()
    if override:
        return Path(override).expanduser()
    return Path.home() / ".config" / "soarm101" / "workstation.json"


@dataclass(frozen=True)
class ArmConnectionProfile:
    """Persistent addressing information for one arm role."""

    port: str = ""
    robot_id: str = "so101"
    calibration: str | None = None

    def validated(self) -> "ArmConnectionProfile":
        robot_id = str(self.robot_id).strip()
        if not robot_id:
            raise ValueError("arm robot_id cannot be empty")
        port = str(self.port).strip()
        calibration = self.calibration
        if calibration is None or not str(calibration).strip():
            calibration_path = default_calibration_path(robot_id)
        else:
            calibration_path = Path(str(calibration)).expanduser()
        return replace(
            self,
            port=port,
            robot_id=robot_id,
            calibration=str(calibration_path),
        )


@dataclass(frozen=True)
class WorkstationProfile:
    """Named local hardware profile for one SO-ARM101 workstation."""

    schema_version: int = WORKSTATION_SCHEMA_VERSION
    follower: ArmConnectionProfile = field(
        default_factory=lambda: ArmConnectionProfile(robot_id="so101").validated()
    )
    leader: ArmConnectionProfile = field(
        default_factory=lambda: ArmConnectionProfile(robot_id="so101-leader").validated()
    )
    cameras: Mapping[str, CameraSettings] = field(default_factory=dict)
    selected_camera: str | None = None

    def validated(self) -> "WorkstationProfile":
        if int(self.schema_version) != WORKSTATION_SCHEMA_VERSION:
            raise ValueError(
                f"unsupported workstation schema version {self.schema_version}; "
                f"expected {WORKSTATION_SCHEMA_VERSION}"
            )
        follower = self.follower.validated()
        leader = self.leader.validated()
        normalized: dict[str, CameraSettings] = {}
        devices: dict[str, str] = {}
        for raw_name, raw_settings in self.cameras.items():
            name = str(raw_name).strip()
            if not name:
                raise ValueError("camera profile names cannot be empty")
            if name in normalized:
                raise ValueError(f"duplicate camera profile name: {name}")
            settings = raw_settings.validated()
            device_text = settings.device.strip()
            device_key = (
                os.path.realpath(device_text)
                if device_text.startswith("/")
                else device_text
            )
            if device_key in devices:
                raise ValueError(
                    f"camera profiles {devices[device_key]!r} and {name!r} "
                    f"use the same device {device_key!r}"
                )
            devices[device_key] = name
            normalized[name] = settings
        selected = self.selected_camera
        if selected is not None:
            selected = str(selected).strip() or None
        if selected not in normalized:
            selected = next(iter(normalized), None)
        return WorkstationProfile(
            schema_version=WORKSTATION_SCHEMA_VERSION,
            follower=follower,
            leader=leader,
            cameras=normalized,
            selected_camera=selected,
        )

    def camera(self, name: str | None = None) -> CameraSettings:
        profile = self.validated()
        resolved = (name or profile.selected_camera or "").strip()
        if not resolved:
            raise KeyError("no camera profiles are configured")
        try:
            return profile.cameras[resolved]
        except KeyError as exc:
            available = ", ".join(profile.cameras) or "none"
            raise KeyError(
                f"unknown camera profile {resolved!r}; configured profiles: {available}"
            ) from exc

    def camera_payload(self, name: str | None = None) -> dict[str, object]:
        """One canonical projection for CLI/registry camera settings."""
        if name is not None:
            return {"name": name, **asdict(self.camera(name))}
        return {
            "selected_camera": self.selected_camera,
            "cameras": {
                camera_name: asdict(settings)
                for camera_name, settings in self.cameras.items()
            },
        }

    def with_camera(
        self,
        name: str,
        settings: CameraSettings,
        *,
        select: bool = True,
    ) -> "WorkstationProfile":
        name = str(name).strip()
        if not name:
            raise ValueError("camera profile name cannot be empty")
        cameras = dict(self.cameras)
        cameras[name] = settings.validated()
        return replace(
            self,
            cameras=cameras,
            selected_camera=name if select else self.selected_camera,
        ).validated()

    def without_camera(self, name: str) -> "WorkstationProfile":
        name = str(name).strip()
        cameras = dict(self.cameras)
        if name not in cameras:
            raise KeyError(f"unknown camera profile {name!r}")
        del cameras[name]
        selected = self.selected_camera
        if selected == name:
            selected = next(iter(cameras), None)
        return replace(self, cameras=cameras, selected_camera=selected).validated()

    def renamed_camera(self, old_name: str, new_name: str) -> "WorkstationProfile":
        old_name = str(old_name).strip()
        new_name = str(new_name).strip()
        if old_name not in self.cameras:
            raise KeyError(f"unknown camera profile {old_name!r}")
        if not new_name:
            raise ValueError("camera profile name cannot be empty")
        if new_name != old_name and new_name in self.cameras:
            raise ValueError(f"camera profile {new_name!r} already exists")
        cameras = dict(self.cameras)
        settings = cameras.pop(old_name)
        cameras[new_name] = settings
        selected = new_name if self.selected_camera == old_name else self.selected_camera
        return replace(self, cameras=cameras, selected_camera=selected).validated()


class WorkstationProfileStore:
    """Atomic JSON persistence with migration from the legacy single-camera file."""

    def __init__(
        self,
        path: str | Path | None = None,
        *,
        legacy_camera_path: str | Path | None = None,
    ) -> None:
        self.path = (
            Path(path).expanduser() if path is not None else default_workstation_profile_path()
        )
        self.legacy_camera_path = (
            Path(legacy_camera_path).expanduser()
            if legacy_camera_path is not None
            else DEFAULT_CAMERA_CONFIG_PATH
        )

    def _default(self) -> WorkstationProfile:
        profile = WorkstationProfile()
        if self.legacy_camera_path.exists():
            try:
                settings = CameraSettingsStore(self.legacy_camera_path).load()
            except Exception:
                return profile.validated()
            return profile.with_camera("camera", settings)
        return profile.validated()

    @staticmethod
    def _arm_from_payload(
        payload: object,
        *,
        default_robot_id: str,
    ) -> ArmConnectionProfile:
        if not isinstance(payload, dict):
            return ArmConnectionProfile(robot_id=default_robot_id).validated()
        return ArmConnectionProfile(
            port=str(payload.get("port", "")),
            robot_id=str(payload.get("robot_id", default_robot_id)),
            calibration=(
                None
                if payload.get("calibration") in (None, "")
                else str(payload.get("calibration"))
            ),
        ).validated()

    def load(self) -> WorkstationProfile:
        if not self.path.exists():
            return self._default()
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("workstation profile root must be a JSON object")
        cameras_payload = payload.get("cameras", {})
        cameras: dict[str, CameraSettings] = {}
        if isinstance(cameras_payload, dict):
            for name, values in cameras_payload.items():
                if not isinstance(values, dict):
                    continue
                cameras[str(name)] = CameraSettings(**values).validated()
        return WorkstationProfile(
            schema_version=int(payload.get("schema_version", WORKSTATION_SCHEMA_VERSION)),
            follower=self._arm_from_payload(
                payload.get("follower"),
                default_robot_id="so101",
            ),
            leader=self._arm_from_payload(
                payload.get("leader"),
                default_robot_id="so101-leader",
            ),
            cameras=cameras,
            selected_camera=(
                None
                if payload.get("selected_camera") in (None, "")
                else str(payload.get("selected_camera"))
            ),
        ).validated()

    def save(self, profile: WorkstationProfile) -> WorkstationProfile:
        profile = profile.validated()
        payload = {
            "schema_version": profile.schema_version,
            "follower": asdict(profile.follower),
            "leader": asdict(profile.leader),
            **profile.camera_payload(),
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, self.path)
        return profile
