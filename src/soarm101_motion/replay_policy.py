"""Fail-closed replay admission primitives; these do not execute robot motion.

Only a trusted operator can construct the allowlist. The broker must independently
recheck authority, identity, asset revisions, current state and STOP epoch at
execution. A compiled plan is not permission to move or a physical safety claim.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Mapping

_REVISION = re.compile(r"^sha256:[0-9a-f]{64}$")
_ID = re.compile(r"^soarm101/(?:pose|sequence|recording/(?:raw|edited))/[A-Za-z0-9._%~-]+$")


@dataclass(frozen=True)
class ReplayPolicy:
    """Operator-pinned asset identities and explicit bounded replay ceilings."""

    robot_id: str
    calibration_id: str
    allowed_assets: tuple[tuple[str, str], ...]
    max_speed_scale: float
    max_steps: int
    max_duration_s: float
    max_start_error_rad: float
    max_start_gripper_error: float

    def __post_init__(self) -> None:
        if any(not isinstance(v, str) or not v.strip() or len(v) > 256
               for v in (self.robot_id, self.calibration_id)):
            raise ValueError("invalid robot or calibration identity")
        if (not isinstance(self.allowed_assets, tuple) or not 1 <= len(self.allowed_assets) <= 1000
                or any(not isinstance(item, tuple) or len(item) != 2
                       or not isinstance(item[0], str) or not _ID.fullmatch(item[0])
                       or not isinstance(item[1], str) or not _REVISION.fullmatch(item[1])
                       for item in self.allowed_assets)
                or len({item[0] for item in self.allowed_assets}) != len(self.allowed_assets)):
            raise ValueError("invalid pinned asset identities")
        if type(self.max_steps) is not int or not 1 <= self.max_steps <= 1000:
            raise ValueError("invalid step limit")
        for value, ceiling in ((self.max_speed_scale, 1.0), (self.max_duration_s, 300.0),
                               (self.max_start_error_rad, 0.1),
                               (self.max_start_gripper_error, 0.1)):
            if (isinstance(value, bool) or not isinstance(value, (float, int))
                    or not math.isfinite(value) or not 0 < value <= ceiling):
                raise ValueError("invalid replay policy bound")

    @classmethod
    def from_document(cls, document: object) -> ReplayPolicy:
        if not isinstance(document, dict) or set(document) != {
            "schema_version", "robot_id", "calibration_id", "allowed_assets",
            "max_speed_scale", "max_steps", "max_duration_s", "max_start_error_rad",
            "max_start_gripper_error",
        }:
            raise ValueError("invalid replay policy fields")
        if type(document["schema_version"]) is not int or document["schema_version"] != 1:
            raise ValueError("unsupported replay policy version")
        robot, calibration = document["robot_id"], document["calibration_id"]
        if any(not isinstance(v, str) or not v.strip() or len(v) > 256 for v in (robot, calibration)):
            raise ValueError("robot and calibration identities must be nonempty strings")
        assets = document["allowed_assets"]
        if not isinstance(assets, dict) or not 1 <= len(assets) <= 1000:
            raise ValueError("allowed_assets must be a nonempty bounded object")
        for asset_id, revision in assets.items():
            if (not isinstance(asset_id, str) or not _ID.fullmatch(asset_id)
                    or not isinstance(revision, str) or not _REVISION.fullmatch(revision)):
                raise ValueError("invalid pinned asset ID or revision")
        steps = document["max_steps"]
        if type(steps) is not int or not 1 <= steps <= 1000:
            raise ValueError("max_steps must be an integer in 1..1000")
        def bounded(name: str, ceiling: float) -> float:
            value = document[name]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not 0 < value <= ceiling):
                raise ValueError(f"{name} is outside the permitted range")
            return float(value)
        return cls(
            robot_id=robot, calibration_id=calibration, allowed_assets=tuple(sorted(assets.items())),
            max_speed_scale=bounded("max_speed_scale", 1.0), max_steps=steps,
            max_duration_s=bounded("max_duration_s", 300.0),
            max_start_error_rad=bounded("max_start_error_rad", 0.1),
            max_start_gripper_error=bounded("max_start_gripper_error", 0.1),
        )

    def require_asset(self, asset_id: str, revision: str) -> None:
        """Reject unapproved or changed assets, including every sequence dependency."""
        allowed = dict(self.allowed_assets)
        if asset_id not in allowed:
            raise PermissionError("asset is not allowed by replay policy")
        if allowed[asset_id] != revision:
            raise PermissionError("asset revision differs from operator-pinned revision")

    def check_start(self, actual: Mapping[str, float], expected: Mapping[str, float]) -> None:
        """Reject a start state outside the operator-pinned joint tolerance."""
        from soarm101_motion.constants import ARM_JOINTS

        if set(actual) != set(ARM_JOINTS) or set(expected) != set(ARM_JOINTS):
            raise ValueError("start state must include exactly the arm joints")
        for joint in ARM_JOINTS:
            a, b = actual[joint], expected[joint]
            if (isinstance(a, bool) or isinstance(b, bool) or not isinstance(a, (int, float))
                    or not isinstance(b, (int, float)) or not math.isfinite(a) or not math.isfinite(b)):
                raise ValueError("start state must contain finite joint positions")
            if abs(a - b) > self.max_start_error_rad:
                raise PermissionError("start state exceeds replay tolerance")

    def check_gripper_start(self, actual: float, expected: float) -> None:
        for value in (actual, expected):
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not 0 <= value <= 1):
                raise ValueError("gripper start must be finite and within 0..1")
        if abs(actual - expected) > self.max_start_gripper_error:
            raise PermissionError("gripper start exceeds replay tolerance")
