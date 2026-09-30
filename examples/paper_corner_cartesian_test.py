"""Interactive paper-corner Cartesian validation for a physical SO-ARM101.

The operator manually points the same physical probe point at three corners of a
known rectangle while torque is disabled. The script derives an orthonormal
paper frame, predicts the fourth corner, then (after an explicit confirmation)
uses the normal Motion SDK to lift, traverse, and point at the predicted corner.
It finally visits several heights above that far corner along the paper normal.

US Letter defaults are 215.9 x 279.4 mm.

Important: the stock SDK TCP is the modeled gripper TCP, not the lower finger
tip. If the lower finger is used as the probe, keep the wrist/tool orientation
as constant as practical for all three manual corner captures. The script
reports orientation drift and refuses autonomous motion when that drift exceeds
--max-orientation-drift-deg unless --allow-orientation-drift is supplied.
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
from soarm101_motion.types import Pose
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
    """Derive an orthonormal paper frame from three adjacent rectangle corners.

    A is the origin. B is adjacent to A along the known width. C is adjacent to
    A along the known height. The returned +Z normal is chosen to point generally
    along the robot base +Z axis for a sheet lying on a table.
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
    input("Press Enter when the lower finger is exactly on the corner... ")
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


def _pose_at(position_m: FloatArray, rotation: FloatArray) -> Pose:
    return Pose(np.asarray(position_m, dtype=float), np.asarray(rotation, dtype=float))


def _preflight_exact(arm: SOARM101, name: str, pose: Pose) -> None:
    solution = arm.solve_ik(pose, orientation_mode="exact")
    if not solution.success:
        raise RuntimeError(
            f"{name} is not reachable with the captured tool orientation: "
            f"{solution.message}"
        )
    print(
        f"  preflight {name}: IK position error {solution.position_error_m * 1000.0:.2f} mm, "
        f"orientation error {solution.orientation_error_rad * 180.0 / np.pi:.2f} deg"
    )


def _move_exact(
    arm: SOARM101,
    name: str,
    pose: Pose,
    *,
    speed_mm_s: float,
    acceleration_mm_s2: float,
) -> None:
    print(f"\nMoving: {name}")
    result = arm.move_linear(
        pose,
        orientation_mode="exact",
        speed=speed_mm_s / 1000.0,
        acceleration=acceleration_mm_s2 / 1000.0,
    )
    if not result.accepted or not result.completed:
        raise RuntimeError(f"motion did not complete: {result}")
    actual = arm.get_position().position * 1000.0
    print(
        "  measured model TCP after move = "
        f"({actual[0]:.1f}, {actual[1]:.1f}, {actual[2]:.1f}) mm"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", help="follower serial port; defaults to saved workstation follower"
    )
    parser.add_argument("--robot-id", help="robot ID; defaults to saved workstation follower")
    parser.add_argument("--calibration", help="explicit calibration file")
    parser.add_argument("--width-mm", type=float, default=LETTER_WIDTH_MM)
    parser.add_argument("--height-mm", type=float, default=LETTER_HEIGHT_MM)
    parser.add_argument(
        "--clearance-mm",
        type=float,
        default=30.0,
        help="lift above the paper before traversing to the predicted fourth corner",
    )
    parser.add_argument(
        "--point-height-mm",
        type=float,
        default=2.0,
        help=(
            "stop this far above the predicted fourth corner; "
            "use 0 only after confidence is established"
        ),
    )
    parser.add_argument(
        "--z-heights-mm",
        type=float,
        nargs="+",
        default=[25.0, 50.0, 100.0],
        help="paper-frame +Z heights to visit above the far corner after prediction",
    )
    parser.add_argument("--speed-mm-s", type=float, default=5.0)
    parser.add_argument("--acceleration-mm-s2", type=float, default=20.0)
    parser.add_argument("--max-edge-error-mm", type=float, default=25.0)
    parser.add_argument("--max-angle-error-deg", type=float, default=10.0)
    parser.add_argument("--max-orientation-drift-deg", type=float, default=5.0)
    parser.add_argument(
        "--allow-orientation-drift",
        action="store_true",
        help=(
            "allow autonomous motion even when lower-finger probe orientation "
            "changed substantially"
        ),
    )
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
    if args.clearance_mm <= 0:
        raise SystemExit("--clearance-mm must be positive")
    if args.point_height_mm < 0:
        raise SystemExit("--point-height-mm must be >= 0")
    if any(height <= 0 for height in args.z_heights_mm):
        raise SystemExit("all --z-heights-mm values must be positive")

    config = _resolve_config(args)
    report: dict[str, object] = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "robot_id": config.robot_id,
        "port": config.port,
        "paper": {"width_mm": args.width_mm, "height_mm": args.height_mm},
        "probe": "lower gripper finger; stock modeled gripper TCP used for FK",
    }

    print("Paper Cartesian validation")
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
    print("\nKeep the GUI disconnected from this follower port.")
    print("The lower finger is not the SDK's modeled TCP, so keep wrist/tool orientation")
    print("as constant as practical while touching A, B, and C.")

    with SOARM101(config) as arm:
        try:
            input("\nPress Enter to enable torque briefly and open the gripper fully... ")
            arm.enable()
            arm.tool.open()
            print("Gripper is open. Relaxing all motors for manual corner pointing.")
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
                "  paper normal: "
                f"({geometry.normal[0]:+.4f}, "
                f"{geometry.normal[1]:+.4f}, "
                f"{geometry.normal[2]:+.4f})"
            )
            d_mm = geometry.predicted_far_corner_m * 1000.0
            d_raw_mm = geometry.raw_parallelogram_corner_m * 1000.0
            print(
                "  predicted D (known paper size): "
                f"({d_mm[0]:.1f}, {d_mm[1]:.1f}, {d_mm[2]:.1f}) mm"
            )
            print(
                "  D from raw parallelogram B+C-A: "
                f"({d_raw_mm[0]:.1f}, {d_raw_mm[1]:.1f}, {d_raw_mm[2]:.1f}) mm"
            )
            prediction_difference_mm = float(
                np.linalg.norm(
                    geometry.predicted_far_corner_m - geometry.raw_parallelogram_corner_m
                )
                * 1000.0
            )
            print(f"  known-size vs raw D difference: {prediction_difference_mm:.1f} mm")
            print(
                f"  lower-finger probe orientation drift: B={drift_b:.2f} deg, "
                f"C={drift_c:.2f} deg, max={max_drift:.2f} deg"
            )

            report["geometry"] = {
                "paper_origin_xyz_mm": (geometry.origin_m * 1000.0).tolist(),
                "paper_x_axis_base": geometry.x_axis.tolist(),
                "paper_y_axis_base": geometry.y_axis.tolist(),
                "paper_z_axis_base": geometry.normal.tolist(),
                "measured_width_mm": geometry.measured_width_mm,
                "measured_height_mm": geometry.measured_height_mm,
                "width_error_mm": geometry.width_error_mm,
                "height_error_mm": geometry.height_error_mm,
                "measured_corner_angle_deg": geometry.measured_corner_angle_deg,
                "predicted_far_corner_xyz_mm": d_mm.tolist(),
                "raw_parallelogram_far_corner_xyz_mm": d_raw_mm.tolist(),
                "prediction_difference_mm": prediction_difference_mm,
                "orientation_drift_deg": {"B_from_A": drift_b, "C_from_A": drift_c},
            }
            args.output.write_text(
                json.dumps(report, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"\nIntermediate report written to {args.output}")

            edge_error = max(
                abs(geometry.width_error_mm),
                abs(geometry.height_error_mm),
            )
            angle_error = abs(geometry.measured_corner_angle_deg - 90.0)
            if edge_error > args.max_edge_error_mm or angle_error > args.max_angle_error_deg:
                print(
                    "\nSTOP: the three captured corners do not look enough like the known "
                    "paper rectangle for autonomous motion."
                )
                print(
                    f"  max edge error={edge_error:.1f} mm "
                    f"(limit {args.max_edge_error_mm:.1f})"
                )
                print(
                    f"  corner-angle error={angle_error:.1f} deg "
                    f"(limit {args.max_angle_error_deg:.1f})"
                )
                return 2

            if max_drift > args.max_orientation_drift_deg and not args.allow_orientation_drift:
                print(
                    f"\nSTOP: probe orientation changed by {max_drift:.2f} deg, above the "
                    f"{args.max_orientation_drift_deg:.2f} deg limit. Because the lower finger is "
                    "offset from the modeled TCP, that can corrupt the corner geometry. Repeat the "
                    "three touches with a steadier wrist, or deliberately pass "
                    "--allow-orientation-drift."
                )
                return 2

            reference_rotation = c.rotation
            clearance_m = args.clearance_mm / 1000.0
            point_height_m = args.point_height_mm / 1000.0
            lift_c = _pose_at(
                c.position_m + geometry.normal * clearance_m,
                reference_rotation,
            )
            above_d = _pose_at(
                geometry.predicted_far_corner_m + geometry.normal * clearance_m,
                reference_rotation,
            )
            point_d = _pose_at(
                geometry.predicted_far_corner_m + geometry.normal * point_height_m,
                reference_rotation,
            )
            z_targets = [
                (
                    height,
                    _pose_at(
                        geometry.predicted_far_corner_m
                        + geometry.normal * (height / 1000.0),
                        reference_rotation,
                    ),
                )
                for height in args.z_heights_mm
            ]

            print("\nPreflighting exact-orientation IK with torque still OFF...")
            _preflight_exact(arm, "lift above C", lift_c)
            _preflight_exact(arm, "above predicted D", above_d)
            _preflight_exact(arm, "predicted D point", point_d)
            for height, pose in z_targets:
                _preflight_exact(arm, f"D + {height:.1f} mm paper-Z", pose)

            print("\nAUTONOMOUS MOTION PHASE")
            print("The arm will preserve the captured C tool orientation, lift from C,")
            print(
                "traverse above D, descend to the predicted D point, "
                "then test paper-frame Z."
            )
            print("Keep the workspace clear and physical power immediately reachable.")
            input("Press Enter to enable torque and start the predicted-corner move... ")
            arm.enable()

            _move_exact(
                arm,
                f"lift {args.clearance_mm:.1f} mm along paper +Z above C",
                lift_c,
                speed_mm_s=args.speed_mm_s,
                acceleration_mm_s2=args.acceleration_mm_s2,
            )
            _move_exact(
                arm,
                "traverse above predicted D",
                above_d,
                speed_mm_s=args.speed_mm_s,
                acceleration_mm_s2=args.acceleration_mm_s2,
            )
            _move_exact(
                arm,
                f"point at predicted D ({args.point_height_mm:.1f} mm above paper)",
                point_d,
                speed_mm_s=min(args.speed_mm_s, 3.0),
                acceleration_mm_s2=args.acceleration_mm_s2,
            )
            input(
                "\nInspect alignment with the fourth corner. "
                "Press Enter for the Z-height test... "
            )

            for height, pose in sorted(z_targets, key=lambda item: item[0]):
                _move_exact(
                    arm,
                    (
                        f"paper (x={args.width_mm:.1f}, "
                        f"y={args.height_mm:.1f}, z={height:.1f}) mm"
                    ),
                    pose,
                    speed_mm_s=args.speed_mm_s,
                    acceleration_mm_s2=args.acceleration_mm_s2,
                )
                input("Press Enter for the next height... ")

            final_pose = arm.get_position()
            final_xyz = (final_pose.position * 1000.0).tolist()
            report["completed"] = True
            report["final_tcp_xyz_mm"] = final_xyz
            args.output.write_text(
                json.dumps(report, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"\nComplete. Final report written to {args.output}")
            return 0
        finally:
            try:
                arm.relax()
                print("Motors relaxed.")
            except Exception as exc:
                print(f"WARNING: could not confirm relax during cleanup: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
