"""Read-only three-corner paper geometry check for a physical SO-ARM101.

The operator manually points the same physical probe point at three corners of a known
rectangle while torque is disabled. The script reports how the current FK maps those
physical points and predicts the fourth corner in model coordinates.

It intentionally performs NO autonomous Cartesian arm motion. Hardware testing showed
that a model/paper-normal direction that looked valid numerically did not correspond to
physical up on the tested arm/setup. Use paper_four_corner_hover_demo.py (now a
manual workspace-calibration workflow) to teach four table corners plus a measured
physical-UP reference before any later Cartesian motion-validation stage.

US Letter defaults are 215.9 x 279.4 mm.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from math import acos, degrees
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.workstation import WorkstationProfileStore

FloatArray = NDArray[np.float64]

LETTER_WIDTH_MM = 215.9
LETTER_HEIGHT_MM = 279.4


@dataclass(frozen=True)
class CornerSample:
    name: str
    tcp_xyz_mm: tuple[float, float, float]
    tcp_rpy_deg: tuple[float, float, float]
    rotation_matrix: tuple[tuple[float, float, float], ...]
    joints_rad: dict[str, float]

    @property
    def position_m(self) -> FloatArray:
        return np.asarray(self.tcp_xyz_mm, dtype=float) / 1000.0

    @property
    def rotation(self) -> FloatArray:
        return np.asarray(self.rotation_matrix, dtype=float)


@dataclass(frozen=True)
class PaperGeometry:
    origin_m: FloatArray
    x_axis: FloatArray
    y_axis: FloatArray
    normal: FloatArray
    predicted_far_corner_m: FloatArray
    raw_parallelogram_corner_m: FloatArray
    measured_width_mm: float
    measured_height_mm: float
    measured_corner_angle_deg: float
    width_error_mm: float
    height_error_mm: float


def _unit(vector: FloatArray, *, label: str) -> FloatArray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-9:
        raise ValueError(f"{label} is too small to define a direction")
    return vector / norm


def derive_paper_geometry(
    a_m: FloatArray,
    b_m: FloatArray,
    c_m: FloatArray,
    *,
    width_mm: float,
    height_mm: float,
) -> PaperGeometry:
    """Derive an orthonormal model-space paper frame from three physical corners.

    A is the origin. B is adjacent to A along the known width. C is adjacent to
    A along the known height. The normal sign is chosen toward model base +Z for
    deterministic reporting only. It is NOT treated as demonstrated physical up.
    """

    a = np.asarray(a_m, dtype=float).reshape(3)
    b = np.asarray(b_m, dtype=float).reshape(3)
    c = np.asarray(c_m, dtype=float).reshape(3)
    ab = b - a
    ac = c - a
    measured_width_mm = float(np.linalg.norm(ab) * 1000.0)
    measured_height_mm = float(np.linalg.norm(ac) * 1000.0)

    x_axis = _unit(ab, label="A->B")
    ac_orthogonal = ac - np.dot(ac, x_axis) * x_axis
    y_axis = _unit(ac_orthogonal, label="A->C component perpendicular to A->B")
    if float(np.dot(y_axis, ac)) < 0.0:
        y_axis = -y_axis
    normal = _unit(np.cross(x_axis, y_axis), label="paper normal")
    if normal[2] < 0.0:
        normal = -normal

    cosine = float(
        np.clip(
            np.dot(_unit(ab, label="A->B"), _unit(ac, label="A->C")),
            -1.0,
            1.0,
        )
    )
    corner_angle_deg = degrees(acos(cosine))

    predicted_far_corner_m = (
        a + x_axis * (width_mm / 1000.0) + y_axis * (height_mm / 1000.0)
    )
    raw_parallelogram_corner_m = b + c - a
    return PaperGeometry(
        origin_m=a,
        x_axis=x_axis,
        y_axis=y_axis,
        normal=normal,
        predicted_far_corner_m=predicted_far_corner_m,
        raw_parallelogram_corner_m=raw_parallelogram_corner_m,
        measured_width_mm=measured_width_mm,
        measured_height_mm=measured_height_mm,
        measured_corner_angle_deg=corner_angle_deg,
        width_error_mm=measured_width_mm - width_mm,
        height_error_mm=measured_height_mm - height_mm,
    )


def orientation_delta_deg(reference: FloatArray, other: FloatArray) -> float:
    delta = Rotation.from_matrix(other @ reference.T)
    return float(np.linalg.norm(delta.as_rotvec()) * 180.0 / np.pi)


def _resolve_config(args: argparse.Namespace) -> SOARM101Config:
    if args.port:
        robot_id = args.robot_id or "so101"
        calibration = Path(args.calibration).expanduser() if args.calibration else None
        return SOARM101Config(
            port=args.port,
            robot_id=robot_id,
            calibration_path=calibration,
            configure_motors_on_connect=False,
            auto_enable_torque=False,
        )

    follower = WorkstationProfileStore().load().follower
    if not follower.port:
        raise ValueError(
            "No --port was supplied and the workstation profile has no saved follower port"
        )
    robot_id = args.robot_id or follower.robot_id
    calibration: Path | None = None
    if args.calibration:
        calibration = Path(args.calibration).expanduser()
    elif robot_id == follower.robot_id and follower.calibration:
        calibration = Path(follower.calibration).expanduser()
    return SOARM101Config(
        port=follower.port,
        robot_id=robot_id,
        calibration_path=calibration,
        configure_motors_on_connect=False,
        auto_enable_torque=False,
    )


def _capture_corner(arm: SOARM101, name: str, instruction: str) -> CornerSample:
    print(f"\n{name}: {instruction}")
    input("Press Enter when the fixed lower finger is exactly on the corner... ")
    pose = arm.get_position()
    joints = dict(arm.get_joint_positions().positions)
    xyz_rpy = pose.xyz_rpy()
    xyz_mm = tuple(float(value * 1000.0) for value in xyz_rpy[:3])
    rpy_deg = tuple(float(value * 180.0 / np.pi) for value in xyz_rpy[3:])
    sample = CornerSample(
        name=name,
        tcp_xyz_mm=xyz_mm,
        tcp_rpy_deg=rpy_deg,
        rotation_matrix=tuple(tuple(float(value) for value in row) for row in pose.rotation),
        joints_rad=joints,
    )
    print(
        f"  model TCP = ({xyz_mm[0]:.1f}, {xyz_mm[1]:.1f}, {xyz_mm[2]:.1f}) mm; "
        f"RPY = ({rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f}) deg"
    )
    return sample


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", help="follower serial port; defaults to saved workstation follower"
    )
    parser.add_argument("--robot-id", help="robot ID; defaults to saved workstation follower")
    parser.add_argument("--calibration", help="explicit motor calibration file")
    parser.add_argument("--width-mm", type=float, default=LETTER_WIDTH_MM)
    parser.add_argument("--height-mm", type=float, default=LETTER_HEIGHT_MM)
    parser.add_argument("--max-edge-error-mm", type=float, default=25.0)
    parser.add_argument("--max-angle-error-deg", type=float, default=10.0)
    parser.add_argument("--max-orientation-drift-deg", type=float, default=5.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("paper-cartesian-validation.json"),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.width_mm <= 0 or args.height_mm <= 0:
        raise SystemExit("paper width and height must be positive")

    config = _resolve_config(args)
    report: dict[str, object] = {
        "schema_version": 2,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "robot_id": config.robot_id,
        "port": config.port,
        "paper": {"width_mm": args.width_mm, "height_mm": args.height_mm},
        "probe": "fixed lower gripper finger; stock modeled gripper TCP used for FK",
        "autonomous_motion_attempted": False,
    }

    print("Paper Cartesian geometry check — READ ONLY AFTER GRIPPER OPEN")
    print(f"Paper size: {args.width_mm:.1f} x {args.height_mm:.1f} mm")
    print("Required three-corner layout:")
    print()
    print("  C ---------------- D  (D is predicted; do not touch it)")
    print("  |                  |")
    print("  |                  |")
    print("  A ---------------- B")
    print()
    print("  A = reference corner")
    print("  B = adjacent to A along the SHORT/WIDTH edge")
    print("  C = adjacent to A along the LONG/HEIGHT edge, on the SAME SIDE as A")
    print("Do NOT go A->B and then up the right side: that far corner is D, not C.")
    print()
    print("No Cartesian motion will be commanded by this script.")
    print(
        "A model-space paper normal is reported for diagnosis only; it is not accepted "
        "as physical up without a separate manually measured UP reference."
    )

    with SOARM101(config) as arm:
        try:
            input("\nPress Enter to enable torque briefly and open the moving jaw... ")
            arm.enable()
            arm.tool.open()
            print("Moving jaw is open. Relaxing all motors for manual corner pointing.")
            arm.relax()

            a = _capture_corner(arm, "A", "touch the reference corner")
            b = _capture_corner(
                arm,
                "B",
                "touch the adjacent corner along the paper WIDTH",
            )
            c = _capture_corner(
                arm,
                "C",
                "touch the adjacent corner along the paper HEIGHT",
            )
            samples = [a, b, c]
            report["corners"] = [asdict(sample) for sample in samples]

            geometry = derive_paper_geometry(
                a.position_m,
                b.position_m,
                c.position_m,
                width_mm=args.width_mm,
                height_mm=args.height_mm,
            )
            drift_b = orientation_delta_deg(a.rotation, b.rotation)
            drift_c = orientation_delta_deg(a.rotation, c.rotation)
            max_drift = max(drift_b, drift_c)

            d_mm = geometry.predicted_far_corner_m * 1000.0
            d_raw_mm = geometry.raw_parallelogram_corner_m * 1000.0
            prediction_difference_mm = float(
                np.linalg.norm(
                    geometry.predicted_far_corner_m
                    - geometry.raw_parallelogram_corner_m
                )
                * 1000.0
            )

            print("\nMeasured paper geometry from FK")
            print(
                f"  A->B width:  {geometry.measured_width_mm:.1f} mm "
                f"(expected {args.width_mm:.1f}, error {geometry.width_error_mm:+.1f})"
            )
            print(
                f"  A->C height: {geometry.measured_height_mm:.1f} mm "
                f"(expected {args.height_mm:.1f}, error {geometry.height_error_mm:+.1f})"
            )
            print(
                f"  corner angle: {geometry.measured_corner_angle_deg:.2f} deg "
                "(expected 90.00)"
            )
            print(
                "  model-space paper normal (DIAGNOSTIC ONLY): "
                f"({geometry.normal[0]:+.4f}, "
                f"{geometry.normal[1]:+.4f}, "
                f"{geometry.normal[2]:+.4f})"
            )
            print(
                "  predicted D (known paper size): "
                f"({d_mm[0]:.1f}, {d_mm[1]:.1f}, {d_mm[2]:.1f}) mm"
            )
            print(
                "  D from raw parallelogram B+C-A: "
                f"({d_raw_mm[0]:.1f}, {d_raw_mm[1]:.1f}, {d_raw_mm[2]:.1f}) mm"
            )
            print(f"  known-size vs raw D difference: {prediction_difference_mm:.1f} mm")
            print(
                f"  lower-finger probe orientation drift: B={drift_b:.2f} deg, "
                f"C={drift_c:.2f} deg, max={max_drift:.2f} deg"
            )

            edge_error = max(
                abs(geometry.width_error_mm),
                abs(geometry.height_error_mm),
            )
            angle_error = abs(geometry.measured_corner_angle_deg - 90.0)
            geometry_ok = (
                edge_error <= args.max_edge_error_mm
                and angle_error <= args.max_angle_error_deg
                and max_drift <= args.max_orientation_drift_deg
            )

            report["geometry"] = {
                "paper_origin_xyz_mm": (geometry.origin_m * 1000.0).tolist(),
                "paper_x_axis_base": geometry.x_axis.tolist(),
                "paper_y_axis_base": geometry.y_axis.tolist(),
                "paper_z_axis_base_diagnostic_only": geometry.normal.tolist(),
                "measured_width_mm": geometry.measured_width_mm,
                "measured_height_mm": geometry.measured_height_mm,
                "width_error_mm": geometry.width_error_mm,
                "height_error_mm": geometry.height_error_mm,
                "measured_corner_angle_deg": geometry.measured_corner_angle_deg,
                "predicted_far_corner_xyz_mm": d_mm.tolist(),
                "raw_parallelogram_far_corner_xyz_mm": d_raw_mm.tolist(),
                "prediction_difference_mm": prediction_difference_mm,
                "orientation_drift_deg": {
                    "B_from_A": drift_b,
                    "C_from_A": drift_c,
                },
                "geometry_within_configured_tolerances": geometry_ok,
            }
            args.output.write_text(
                json.dumps(report, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"\nReport written to {args.output}")
            print(
                "No powered Cartesian motion was attempted. For table height and physical-UP "
                "mapping, run examples/paper_four_corner_hover_demo.py next."
            )
            return 0 if geometry_ok else 2
        finally:
            try:
                arm.relax()
                print("Motors relaxed.")
            except Exception as exc:
                print(f"WARNING: could not confirm relax during cleanup: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
