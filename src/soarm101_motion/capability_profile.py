"""Trusted-host, immutable restrictions for bounded robot-broker experiments.

The profile removes permissions from the existing broker; it cannot authorize
motion, alter calibration, or introduce a new robot action. Keep route mapping
here shared with the optional MCP facade, not duplicated by client code.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping


# Tool names match the optional MCP facade. Routes are the already-shipped broker API.
TOOL_ROUTES: dict[str, tuple[str, str]] = {
    "robot_health": ("GET", "/v1/health"),
    "robot_capabilities": ("GET", "/v1/capabilities"),
    "robot_state": ("GET", "/v1/state"),
    "capture_camera": ("POST", "/v1/capture"),
    "go_pose": ("POST", "/v1/go-pose"),
    "jog_joint": ("POST", "/v1/joint"),
    "jog_cartesian": ("POST", "/v1/jog"),
    "move_gripper": ("POST", "/v1/gripper"),
    "sleep": ("POST", "/v1/sleep"),
    "sleep_up": ("POST", "/v1/sleep-up"),
    "stop": ("POST", "/v1/stop"),
}
ROUTE_TO_TOOL = {route: name for name, route in TOOL_ROUTES.items()}
# Retain the broker's existing alternate spelling under the same permissions.
ROUTE_TO_TOOL[("POST", "/v1/sleep_up")] = "sleep_up"
CAMERA_NAMES = frozenset({"overhead", "wrist"})
MAX_PROFILE_JOINT_DELTA_DEG = 30.0
MAX_PROFILE_MODEL_JOG_MM = 50.0


class ProfileError(ValueError):
    """A profile is invalid or requests capabilities not in the bounded API."""


def _unique_names(value: object, allowed: frozenset[str], field: str) -> frozenset[str]:
    if not isinstance(value, list) or any(not isinstance(x, str) for x in value):
        raise ProfileError(f"{field} must be an array of names")
    if len(value) != len(set(value)):
        raise ProfileError(f"{field} must not contain duplicates")
    unknown = set(value) - allowed
    if unknown:
        raise ProfileError(f"{field} contains unsupported names: {sorted(unknown)!r}")
    return frozenset(value)


def _positive_limit(value: object, name: str, ceiling: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ProfileError(f"{name} must be a positive number")
    limit = float(value)
    if not math.isfinite(limit) or limit <= 0 or limit > ceiling:
        raise ProfileError(f"{name} must be within (0, {ceiling:g}]")
    return limit


@dataclass(frozen=True, slots=True)
class CapabilityProfile:
    name: str
    allowed_tools: frozenset[str]
    allowed_cameras: frozenset[str]
    max_joint_delta_deg: float | None = None
    max_model_jog_mm: float | None = None

    @classmethod
    def full(cls) -> "CapabilityProfile":
        return cls(
            name="default-full",
            allowed_tools=frozenset(TOOL_ROUTES),
            allowed_cameras=CAMERA_NAMES,
        )

    @classmethod
    def from_document(cls, document: object) -> "CapabilityProfile":
        if not isinstance(document, dict):
            raise ProfileError("capability profile root must be a JSON object")
        expected = {"schema_version", "name", "tools", "cameras", "limits"}
        extra = set(document) - expected
        if extra:
            raise ProfileError(f"unknown capability profile fields: {sorted(extra)!r}")
        if type(document.get("schema_version")) is not int or document["schema_version"] != 1:
            raise ProfileError("capability profile schema_version must be 1")
        name = document.get("name")
        if not isinstance(name, str) or not re.fullmatch(r"[a-z0-9][a-z0-9._-]{0,63}", name):
            raise ProfileError("profile name must be a short lowercase identifier")
        tools = _unique_names(document.get("tools"), frozenset(TOOL_ROUTES), "tools")
        cameras = _unique_names(document.get("cameras"), CAMERA_NAMES, "cameras")
        if cameras and "capture_camera" not in tools:
            raise ProfileError("camera access requires capture_camera in tools")
        if "capture_camera" in tools and not cameras:
            raise ProfileError("capture_camera requires at least one permitted camera")
        limits = document.get("limits", {})
        if not isinstance(limits, dict):
            raise ProfileError("limits must be an object")
        unknown_limits = set(limits) - {"max_joint_delta_deg", "max_model_jog_mm"}
        if unknown_limits:
            raise ProfileError(f"unsupported limits: {sorted(unknown_limits)!r}")
        if "max_joint_delta_deg" in limits and "jog_joint" not in tools:
            raise ProfileError("joint limit requires jog_joint tool")
        if "max_model_jog_mm" in limits and "jog_cartesian" not in tools:
            raise ProfileError("jog limit requires jog_cartesian tool")
        return cls(
            name=name,
            allowed_tools=tools,
            allowed_cameras=cameras,
            max_joint_delta_deg=(
                _positive_limit(
                    limits["max_joint_delta_deg"],
                    "max_joint_delta_deg",
                    MAX_PROFILE_JOINT_DELTA_DEG,
                ) if "max_joint_delta_deg" in limits else None
            ),
            max_model_jog_mm=(
                _positive_limit(
                    limits["max_model_jog_mm"],
                    "max_model_jog_mm",
                    MAX_PROFILE_MODEL_JOG_MM,
                ) if "max_model_jog_mm" in limits else None
            ),
        )

    @classmethod
    def from_file(cls, path: str | Path) -> "CapabilityProfile":
        source = Path(path).expanduser()
        if source.stat().st_size > 16_384:
            raise ProfileError("capability profile exceeds 16 KiB")
        try:
            document = json.loads(source.read_text(encoding="utf-8"))
        except (ValueError, UnicodeError) as exc:
            raise ProfileError(f"invalid capability profile JSON: {exc}") from exc
        return cls.from_document(document)

    def as_dict(self) -> dict[str, object]:
        limits: dict[str, float] = {}
        if self.max_joint_delta_deg is not None:
            limits["max_joint_delta_deg"] = self.max_joint_delta_deg
        if self.max_model_jog_mm is not None:
            limits["max_model_jog_mm"] = self.max_model_jog_mm
        return {
            "schema_version": 1,
            "name": self.name,
            "tools": sorted(self.allowed_tools),
            "cameras": sorted(self.allowed_cameras),
            "limits": limits,
        }

    def public(self) -> dict[str, object]:
        document = self.as_dict()
        canonical = json.dumps(document, sort_keys=True, separators=(",", ":")).encode("utf-8")
        return {**document, "sha256": hashlib.sha256(canonical).hexdigest()}

    def check(self, method: str, path: str, payload: Mapping[str, object]) -> None:
        """Fail closed before reaching the bounded CLI.

        Health and the profile-descriptor endpoint stay available for client
        setup and local service checks; neither permits physical actions.
        Unknown paths are left to the broker's existing 404 behavior.
        """
        if (method, path) in {("GET", "/v1/health"), ("GET", "/v1/profile")}:
            return
        tool = ROUTE_TO_TOOL.get((method, path))
        if tool is None:
            return
        if tool not in self.allowed_tools:
            raise PermissionError(f"capability {tool} is not enabled by profile {self.name}")
        if tool == "capture_camera":
            if payload.get("camera") not in self.allowed_cameras:
                raise PermissionError("requested camera is not enabled by this profile")
        elif tool == "jog_joint" and self.max_joint_delta_deg is not None:
            try:
                delta = float(payload.get("delta_deg", 0.0))
            except (TypeError, ValueError) as exc:
                raise ProfileError("delta_deg must be a finite number") from exc
            if not math.isfinite(delta) or abs(delta) > self.max_joint_delta_deg:
                raise PermissionError("joint adjustment exceeds profile limit")
        elif tool == "jog_cartesian" and self.max_model_jog_mm is not None:
            try:
                coords = [float(payload.get(k, 0.0)) for k in ("x_mm", "y_mm", "z_mm")]
            except (TypeError, ValueError) as exc:
                raise ProfileError("jog coordinates must be finite numbers") from exc
            if any(not math.isfinite(x) for x in coords):
                raise ProfileError("jog coordinates must be finite numbers")
            if math.hypot(*coords) > self.max_model_jog_mm:
                raise PermissionError("Cartesian adjustment exceeds profile model-frame limit")
