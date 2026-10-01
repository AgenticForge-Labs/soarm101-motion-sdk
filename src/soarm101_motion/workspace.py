"""Persistent machine-local workspace calibration for an SO-ARM101 setup.

Motor calibration answers how encoder counts map to joint coordinates. Workspace
calibration is separate: it records how a particular physical work surface maps into
the SDK kinematic model on a particular setup.

The first supported source is a US-Letter-style paper calibration with four known
table-plane corners plus one manually measured point above the paper. The result is a
local affine mapping from physical paper coordinates into model/base coordinates.

Workspace calibration is evidence and geometry metadata. A freshly measured workspace
calibration is deliberately marked unvalidated for autonomous Cartesian motion until
the physical motion path has been separately verified.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]

WORKSPACE_CALIBRATION_SCHEMA_VERSION = 1


def default_workspace_calibration_path(robot_id: str) -> Path:
    robot_id = str(robot_id).strip()
    if not robot_id:
        raise ValueError("robot_id cannot be empty")
    root = os.environ.get("SOARM101_WORKSPACE_CALIBRATION_DIR", "").strip()
    base = Path(root).expanduser() if root else Path.home() / ".config" / "soarm101" / "workspace"
    return base / f"{robot_id}.json"


def _vector3(values: Sequence[float], *, label: str) -> tuple[float, float, float]:
    vector = tuple(float(value) for value in values)
    if len(vector) != 3 or any(not math.isfinite(value) for value in vector):
        raise ValueError(f"{label} must contain three finite values")
    return vector  # type: ignore[return-value]


def _matrix3(values: Sequence[Sequence[float]], *, label: str) -> tuple[tuple[float, ...], ...]:
    rows = tuple(tuple(float(value) for value in row) for row in values)
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError(f"{label} must be a 3x3 matrix")
    if any(not math.isfinite(value) for row in rows for value in row):
        raise ValueError(f"{label} must contain finite values")
    return rows


@dataclass(frozen=True)
class WorkspaceCalibration:
    """Local physical-workspace mapping tied to one arm calibration."""

    schema_version: int
    workspace_id: str
    robot_id: str
    arm_calibration_id: str
    source: str
    created_at: str
    physical_width_m: float
    physical_height_m: float
    reference_height_m: float
    model_origin_m: tuple[float, float, float]
    physical_to_model_linear: tuple[tuple[float, ...], ...]
    table_plane_point_m: tuple[float, float, float]
    table_plane_normal_model: tuple[float, float, float]
    affine_fit_rms_m: float
    table_plane_rms_m: float
    model_x_scale: float
    model_y_scale: float
    model_up_scale: float
    linear_condition_number: float
    model_xy_angle_deg: float
    up_vs_table_normal_angle_deg: float
    up_reference_plane_height_m: float
    up_reference_tangent_m: float
    corner_model_xyz_m: Mapping[str, tuple[float, float, float]]
    up_reference_model_xyz_m: tuple[float, float, float]
    motion_validation_status: str = "unvalidated"

    def validated(self) -> "WorkspaceCalibration":
        if int(self.schema_version) != WORKSPACE_CALIBRATION_SCHEMA_VERSION:
            raise ValueError(
                f"unsupported workspace calibration schema {self.schema_version}; "
                f"expected {WORKSPACE_CALIBRATION_SCHEMA_VERSION}"
            )
        robot_id = str(self.robot_id).strip()
        arm_calibration_id = str(self.arm_calibration_id).strip()
        source = str(self.source).strip()
        workspace_id = str(self.workspace_id).strip()
        if not robot_id or not arm_calibration_id or not source or not workspace_id:
            raise ValueError("workspace calibration identity fields cannot be empty")
        for name in (
            "physical_width_m",
            "physical_height_m",
            "reference_height_m",
            "model_x_scale",
            "model_y_scale",
            "model_up_scale",
            "linear_condition_number",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        for name in (
            "affine_fit_rms_m",
            "table_plane_rms_m",
            "model_xy_angle_deg",
            "up_vs_table_normal_angle_deg",
            "up_reference_tangent_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be non-negative and finite")
        if not math.isfinite(float(self.up_reference_plane_height_m)):
            raise ValueError("up_reference_plane_height_m must be finite")

        origin = _vector3(self.model_origin_m, label="model_origin_m")
        table_point = _vector3(self.table_plane_point_m, label="table_plane_point_m")
        normal = np.asarray(
            _vector3(self.table_plane_normal_model, label="table_plane_normal_model"),
            dtype=float,
        )
        if not math.isclose(float(np.linalg.norm(normal)), 1.0, abs_tol=1e-6):
            raise ValueError("table_plane_normal_model must be unit length")
        linear = _matrix3(self.physical_to_model_linear, label="physical_to_model_linear")
        up_reference = _vector3(
            self.up_reference_model_xyz_m,
            label="up_reference_model_xyz_m",
        )
        corners: dict[str, tuple[float, float, float]] = {}
        for name in ("A", "B", "C", "D"):
            if name not in self.corner_model_xyz_m:
                raise ValueError(f"workspace calibration is missing corner {name}")
            corners[name] = _vector3(
                self.corner_model_xyz_m[name],
                label=f"corner_model_xyz_m[{name}]",
            )
        status = str(self.motion_validation_status).strip()
        if status not in {"unvalidated", "validated", "failed"}:
            raise ValueError(
                "motion_validation_status must be unvalidated, validated, or failed"
            )
        return replace(
            self,
            schema_version=WORKSPACE_CALIBRATION_SCHEMA_VERSION,
            workspace_id=workspace_id,
            robot_id=robot_id,
            arm_calibration_id=arm_calibration_id,
            source=source,
            model_origin_m=origin,
            physical_to_model_linear=linear,
            table_plane_point_m=table_point,
            table_plane_normal_model=tuple(float(value) for value in normal),
            corner_model_xyz_m=corners,
            up_reference_model_xyz_m=up_reference,
            motion_validation_status=status,
        )

    def model_position_from_physical(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
    ) -> FloatArray:
        calibration = self.validated()
        linear = np.asarray(calibration.physical_to_model_linear, dtype=float)
        origin = np.asarray(calibration.model_origin_m, dtype=float)
        physical = np.asarray([x_m, y_m, z_m], dtype=float)
        return origin + linear @ physical

    def physical_position_from_model(self, model_position_m: Sequence[float]) -> FloatArray:
        """Invert the local affine map back into measured physical paper coordinates."""
        calibration = self.validated()
        linear = np.asarray(calibration.physical_to_model_linear, dtype=float)
        origin = np.asarray(calibration.model_origin_m, dtype=float)
        model_position = np.asarray(
            _vector3(model_position_m, label="model_position_m"),
            dtype=float,
        )
        return np.linalg.solve(linear, model_position - origin)


def _workspace_fingerprint(payload: Mapping[str, object]) -> str:
    serialized = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return "sha256:" + hashlib.sha256(serialized).hexdigest()


def fit_paper_workspace(
    corners_model_m: Mapping[str, Sequence[float]],
    up_reference_model_m: Sequence[float],
    *,
    robot_id: str,
    arm_calibration_id: str,
    width_m: float,
    height_m: float,
    reference_height_m: float,
    source: str = "paper-four-corner-plus-up",
) -> WorkspaceCalibration:
    """Fit a local physical-paper -> model affine transform.

    Physical coordinates are defined as:
      A=(0,0,0), B=(width,0,0), C=(width,height,0),
      D=(0,height,0), UP=(0,height,reference_height).

    The UP point must be manually measured in physical space. It is what determines
    which model-space direction corresponds to physical "away from the table".
    """

    for name, value in (
        ("width_m", width_m),
        ("height_m", height_m),
        ("reference_height_m", reference_height_m),
    ):
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError(f"{name} must be positive and finite")

    model_points: list[FloatArray] = []
    normalized_corners: dict[str, tuple[float, float, float]] = {}
    for name in ("A", "B", "C", "D"):
        if name not in corners_model_m:
            raise ValueError(f"missing paper corner {name}")
        point = np.asarray(
            _vector3(corners_model_m[name], label=f"corner {name}"),
            dtype=float,
        )
        normalized_corners[name] = tuple(float(value) for value in point)
        model_points.append(point)
    up_reference = np.asarray(
        _vector3(up_reference_model_m, label="up_reference_model_m"),
        dtype=float,
    )
    model_points.append(up_reference)
    model = np.stack(model_points)

    physical = np.array(
        [
            [0.0, 0.0, 0.0, 1.0],
            [width_m, 0.0, 0.0, 1.0],
            [width_m, height_m, 0.0, 1.0],
            [0.0, height_m, 0.0, 1.0],
            [0.0, height_m, reference_height_m, 1.0],
        ],
        dtype=float,
    )
    coefficients, _, _, _ = np.linalg.lstsq(physical, model, rcond=None)
    predicted = physical @ coefficients
    affine_fit_rms_m = float(np.sqrt(np.mean(np.sum((predicted - model) ** 2, axis=1))))

    x_column = np.asarray(coefficients[0, :], dtype=float)
    y_column = np.asarray(coefficients[1, :], dtype=float)
    up_column = np.asarray(coefficients[2, :], dtype=float)
    origin = np.asarray(coefficients[3, :], dtype=float)
    model_x_scale = float(np.linalg.norm(x_column))
    model_y_scale = float(np.linalg.norm(y_column))
    model_up_scale = float(np.linalg.norm(up_column))
    linear = np.asarray(coefficients[:3, :].T, dtype=float)
    linear_condition_number = float(np.linalg.cond(linear))
    if (
        min(model_x_scale, model_y_scale, model_up_scale) <= 1e-9
        or not math.isfinite(linear_condition_number)
    ):
        raise ValueError("paper workspace fit is degenerate")

    x_unit = x_column / model_x_scale
    y_unit = y_column / model_y_scale
    up_unit = up_column / model_up_scale
    xy_cosine = float(np.clip(np.dot(x_unit, y_unit), -1.0, 1.0))
    model_xy_angle_deg = float(np.degrees(np.arccos(xy_cosine)))

    table_points = model[:4]
    table_plane_point = np.mean(table_points, axis=0)
    _, _, vh = np.linalg.svd(table_points - table_plane_point)
    table_normal = np.asarray(vh[-1], dtype=float)
    table_normal /= np.linalg.norm(table_normal)
    if float(np.dot(table_normal, up_unit)) < 0.0:
        table_normal = -table_normal
    signed = (table_points - table_plane_point) @ table_normal
    table_plane_rms_m = float(np.sqrt(np.mean(signed**2)))

    normal_cosine = float(np.clip(np.dot(table_normal, up_unit), -1.0, 1.0))
    up_vs_table_normal_angle_deg = float(np.degrees(np.arccos(normal_cosine)))
    up_delta = up_reference - np.asarray(normalized_corners["D"], dtype=float)
    up_reference_plane_height_m = float(
        np.dot(up_reference - table_plane_point, table_normal)
    )
    up_reference_tangent_m = float(
        np.linalg.norm(up_delta - np.dot(up_delta, table_normal) * table_normal)
    )

    core_payload: dict[str, object] = {
        "schema_version": WORKSPACE_CALIBRATION_SCHEMA_VERSION,
        "robot_id": str(robot_id).strip(),
        "arm_calibration_id": str(arm_calibration_id).strip(),
        "source": str(source).strip(),
        "physical_width_m": float(width_m),
        "physical_height_m": float(height_m),
        "reference_height_m": float(reference_height_m),
        "model_origin_m": tuple(float(value) for value in origin),
        "physical_to_model_linear": tuple(
            tuple(float(value) for value in row)
            for row in coefficients[:3, :].T
        ),
        "table_plane_point_m": tuple(float(value) for value in table_plane_point),
        "table_plane_normal_model": tuple(float(value) for value in table_normal),
        "affine_fit_rms_m": affine_fit_rms_m,
        "table_plane_rms_m": table_plane_rms_m,
        "model_x_scale": model_x_scale,
        "model_y_scale": model_y_scale,
        "model_up_scale": model_up_scale,
        "linear_condition_number": linear_condition_number,
        "model_xy_angle_deg": model_xy_angle_deg,
        "up_vs_table_normal_angle_deg": up_vs_table_normal_angle_deg,
        "up_reference_plane_height_m": up_reference_plane_height_m,
        "up_reference_tangent_m": up_reference_tangent_m,
        "corner_model_xyz_m": normalized_corners,
        "up_reference_model_xyz_m": tuple(float(value) for value in up_reference),
        "motion_validation_status": "unvalidated",
    }
    workspace_id = _workspace_fingerprint(core_payload)
    return WorkspaceCalibration(
        created_at=datetime.now(timezone.utc).isoformat(),
        workspace_id=workspace_id,
        **core_payload,
    ).validated()


class WorkspaceCalibrationStore:
    """Atomic JSON persistence for one robot's machine-local workspace calibration."""

    def __init__(self, robot_id: str, path: str | Path | None = None) -> None:
        self.robot_id = str(robot_id).strip()
        if not self.robot_id:
            raise ValueError("robot_id cannot be empty")
        self.path = (
            Path(path).expanduser()
            if path is not None
            else default_workspace_calibration_path(self.robot_id)
        )

    def save(self, calibration: WorkspaceCalibration) -> WorkspaceCalibration:
        calibration = calibration.validated()
        if calibration.robot_id != self.robot_id:
            raise ValueError(
                f"workspace robot_id {calibration.robot_id!r} does not match "
                f"store robot_id {self.robot_id!r}"
            )
        payload = asdict(calibration)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        os.replace(temporary, self.path)
        return calibration

    def load(self) -> WorkspaceCalibration:
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("workspace calibration root must be a JSON object")
        calibration = WorkspaceCalibration(**payload).validated()
        if calibration.robot_id != self.robot_id:
            raise ValueError(
                f"workspace robot_id {calibration.robot_id!r} does not match "
                f"store robot_id {self.robot_id!r}"
            )
        return calibration
