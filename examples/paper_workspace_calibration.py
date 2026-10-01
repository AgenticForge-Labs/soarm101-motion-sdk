"""Manual paper/workspace calibration for a physical SO-ARM101.

This workflow intentionally performs no autonomous Cartesian arm motion.

The operator teaches all four paper corners on the table, then manually teaches one
known physical height above corner D. Those five physical correspondences are used to
fit a local affine map from paper/workspace coordinates into the SDK kinematic model.

The resulting calibration records:
- where the taught table plane lies in model coordinates;
- how the physical paper X/Y directions map into model coordinates; and
- most importantly, how a manually demonstrated physical-UP direction maps into the
  model on this exact arm/setup.

A measured calibration is evidence only and remains unvalidated for autonomous Cartesian
motion until a separate supervised motion-validation stage passes.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from math import hypot
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.workstation import WorkstationProfileStore
from soarm101_motion.workspace import WorkspaceCalibrationStore, fit_paper_workspace

LETTER_WIDTH_MM = 215.9
LETTER_HEIGHT_MM = 279.4


@dataclass(frozen=True)
class Sample:
    name: str
    tcp_xyz_mm: tuple[float, float, float]
    tcp_rpy_deg: tuple[float, float, float]
    rotation_matrix: tuple[tuple[float, float, float], ...]
    joints_rad: dict[str, float]

    @property
    def position_m(self) -> np.ndarray:
        return np.asarray(self.tcp_xyz_mm, dtype=float) / 1000.0

    @property
    def rotation(self) -> np.ndarray:
        return np.asarray(self.rotation_matrix, dtype=float)


def perimeter_distances_mm(points_m: dict[str, np.ndarray]) -> dict[str, float]:
    def distance(first: str, second: str) -> float:
        return float(np.linalg.norm(points_m[second] - points_m[first]) * 1000.0)

    return {
        "AB": distance("A", "B"),
        "BC": distance("B", "C"),
        "CD": distance("C", "D"),
        "DA": distance("D", "A"),
        "AC": distance("A", "C"),
        "BD": distance("B", "D"),
    }


def orientation_delta_deg(first: np.ndarray, second: np.ndarray) -> float:
    delta = Rotation.from_matrix(second @ first.T)
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


def _capture(arm: SOARM101, name: str, instruction: str) -> Sample:
    print(f"\n{name}: {instruction}")
    input("Press Enter when the fixed lower finger is at the requested physical point... ")
    pose = arm.get_position()
    joints = dict(arm.get_joint_positions().positions)
    xyz_rpy = pose.xyz_rpy()
    xyz_mm = tuple(float(value * 1000.0) for value in xyz_rpy[:3])
    rpy_deg = tuple(float(value * 180.0 / np.pi) for value in xyz_rpy[3:])
    sample = Sample(
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
    parser.add_argument(
        "--reference-height-mm",
        "--hover-height-mm",
        dest="reference_height_mm",
        type=float,
        default=50.0,
        help=(
            "manually measured physical height above D used to demonstrate physical UP; "
            "--hover-height-mm is retained as a compatibility alias and no longer commands motion"
        ),
    )
    parser.add_argument(
        "--max-table-fit-rms-mm",
        type=float,
        default=5.0,
        help="maximum table-plane residual for activating the saved workspace calibration",
    )
    parser.add_argument(
        "--max-affine-fit-rms-mm",
        type=float,
        default=10.0,
        help="maximum correspondence RMS for accepting the measured workspace calibration",
    )
    parser.add_argument(
        "--max-linear-condition-number",
        type=float,
        default=20.0,
        help="maximum condition number for the local 3-D physical-to-model mapping",
    )
    parser.add_argument(
        "--max-up-orientation-drift-deg",
        type=float,
        default=10.0,
        help=(
            "maximum D->UP tool-orientation change; larger changes confound the lower-finger "
            "probe with the modeled TCP"
        ),
    )
    parser.add_argument(
        "--min-up-scale",
        type=float,
        default=0.25,
        help="minimum model displacement / physical displacement scale for the UP reference",
    )
    parser.add_argument(
        "--max-up-scale",
        type=float,
        default=2.0,
        help="maximum model displacement / physical displacement scale for the UP reference",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("paper-workspace-calibration-report.json"),
        help="diagnostic report path; written whether or not activation succeeds",
    )
    parser.add_argument(
        "--workspace-output",
        type=Path,
        help=(
            "authoritative workspace calibration path; defaults to "
            "~/.config/soarm101/workspace/<robot-id>.json"
        ),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    for name in ("width_mm", "height_mm", "reference_height_mm"):
        if getattr(args, name) <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} must be positive")
    if args.max_table_fit_rms_mm <= 0.0:
        raise SystemExit("--max-table-fit-rms-mm must be positive")
    if args.max_affine_fit_rms_mm <= 0.0:
        raise SystemExit("--max-affine-fit-rms-mm must be positive")
    if args.max_linear_condition_number <= 1.0:
        raise SystemExit("--max-linear-condition-number must be greater than 1")
    if args.max_up_orientation_drift_deg <= 0.0:
        raise SystemExit("--max-up-orientation-drift-deg must be positive")
    if not 0.0 < args.min_up_scale < args.max_up_scale:
        raise SystemExit("--min-up-scale must be positive and below --max-up-scale")

    config = _resolve_config(args)

    print("Paper workspace calibration — MANUAL / TORQUE-OFF GEOMETRY")
    print(f"Reference sheet: {args.width_mm:.1f} x {args.height_mm:.1f} mm")
    print("No autonomous Cartesian arm motion is performed by this workflow.")
    print()
    print("Teach the table corners clockwise:")
    print()
    print("  D ---------------- C")
    print("  |                  |")
    print("  |                  |")
    print("  A ---------------- B")
    print()
    print("A->B and C->D are the SHORT/WIDTH edges.")
    print("B->C and D->A are the LONG/HEIGHT edges.")
    print()
    print(
        f"After D, you will MANUALLY place the same fixed finger exactly "
        f"{args.reference_height_mm:.1f} mm physically above D."
    )
    print(
        "Use a ruler, rigid spacer, gauge block, or another physical reference. "
        "Keep the wrist/tool orientation as close to D as practical."
    )
    print(
        "That manual UP point is required because model +Z is not assumed to mean physical up."
    )
    print("Keep the GUI disconnected from this follower port.")

    report: dict[str, object] = {
        "schema_version": 2,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "robot_id": config.robot_id,
        "port": config.port,
        "paper": {"width_mm": args.width_mm, "height_mm": args.height_mm},
        "reference_height_mm": args.reference_height_mm,
        "autonomous_motion_attempted": False,
        "probe": "fixed lower gripper finger; stock modeled gripper TCP used for FK",
    }

    with SOARM101(config) as arm:
        try:
            input("\nPress Enter to enable torque briefly and open the moving jaw... ")
            arm.enable()
            arm.tool.open()
            print("Moving jaw is open. Relaxing all motors for manual teaching.")
            arm.relax()

            a = _capture(arm, "A", "touch the lower-left / first paper corner")
            b = _capture(arm, "B", "touch the adjacent SHORT/WIDTH corner")
            c = _capture(arm, "C", "from B, touch the next LONG/HEIGHT corner")
            d = _capture(arm, "D", "touch the remaining SHORT/WIDTH corner from C")
            samples = [a, b, c, d]

            print(
                f"\nUP: manually raise the SAME fixed finger {args.reference_height_mm:.1f} mm "
                "physically straight away from the table above D."
            )
            print(
                "Do not infer direction from the SDK axes. Measure the physical height directly."
            )
            up = _capture(
                arm,
                "UP",
                f"touch/hold the measured point {args.reference_height_mm:.1f} mm above D",
            )

            report["samples"] = [asdict(sample) for sample in [*samples, up]]
            points = {sample.name: sample.position_m for sample in samples}
            distances = perimeter_distances_mm(points)
            report["model_distances_mm"] = distances
            diagonal_mm = hypot(args.width_mm, args.height_mm)

            print("\nModel distances for the physically taught paper:")
            print(f"  A->B {distances['AB']:.1f} mm (physical {args.width_mm:.1f})")
            print(f"  B->C {distances['BC']:.1f} mm (physical {args.height_mm:.1f})")
            print(f"  C->D {distances['CD']:.1f} mm (physical {args.width_mm:.1f})")
            print(f"  D->A {distances['DA']:.1f} mm (physical {args.height_mm:.1f})")
            print(
                f"  diagonals A->C={distances['AC']:.1f}, B->D={distances['BD']:.1f} mm "
                f"(physical {diagonal_mm:.1f})"
            )

            calibration_id = arm.calibration_id
            if not calibration_id:
                raise RuntimeError(
                    "physical workspace calibration requires an active motor calibration ID"
                )

            calibration = fit_paper_workspace(
                {sample.name: sample.position_m for sample in samples},
                up.position_m,
                robot_id=config.robot_id,
                arm_calibration_id=calibration_id,
                width_m=args.width_mm / 1000.0,
                height_m=args.height_mm / 1000.0,
                reference_height_m=args.reference_height_mm / 1000.0,
            )
            up_orientation_drift = orientation_delta_deg(d.rotation, up.rotation)

            metrics = {
                "workspace_id": calibration.workspace_id,
                "table_plane_point_mm": [
                    value * 1000.0 for value in calibration.table_plane_point_m
                ],
                "table_plane_normal_model": list(calibration.table_plane_normal_model),
                "table_plane_rms_mm": calibration.table_plane_rms_m * 1000.0,
                "affine_fit_rms_mm": calibration.affine_fit_rms_m * 1000.0,
                "model_x_scale_mm_per_mm": calibration.model_x_scale,
                "model_y_scale_mm_per_mm": calibration.model_y_scale,
                "model_up_scale_mm_per_mm": calibration.model_up_scale,
                "linear_condition_number": calibration.linear_condition_number,
                "model_xy_angle_deg": calibration.model_xy_angle_deg,
                "up_vs_table_normal_angle_deg": calibration.up_vs_table_normal_angle_deg,
                "up_reference_plane_height_mm": (
                    calibration.up_reference_plane_height_m * 1000.0
                ),
                "up_reference_tangent_mm": calibration.up_reference_tangent_m * 1000.0,
                "up_orientation_drift_deg": up_orientation_drift,
                "physical_to_model_linear": [
                    list(row) for row in calibration.physical_to_model_linear
                ],
            }
            report["workspace_fit"] = metrics

            print("\nWorkspace fit")
            print(
                "  table normal in model coordinates: "
                f"({calibration.table_plane_normal_model[0]:+.4f}, "
                f"{calibration.table_plane_normal_model[1]:+.4f}, "
                f"{calibration.table_plane_normal_model[2]:+.4f})"
            )
            print(
                f"  table-plane RMS residual: {calibration.table_plane_rms_m * 1000.0:.2f} mm"
            )
            print(
                f"  physical-UP vs model-space table normal: "
                f"{calibration.up_vs_table_normal_angle_deg:.2f} deg "
                "(diagnostic skew; not required to be near zero)"
            )
            print(
                f"  affine linear condition number: "
                f"{calibration.linear_condition_number:.2f}"
            )
            print(
                f"  measured UP model displacement normal to table: "
                f"{calibration.up_reference_plane_height_m * 1000.0:.1f} mm"
            )
            print(
                f"  measured UP model displacement tangent to table: "
                f"{calibration.up_reference_tangent_m * 1000.0:.1f} mm"
            )
            print(
                f"  D->UP tool-orientation drift: {up_orientation_drift:.2f} deg"
            )
            print(
                "  local model scale (model mm / physical mm): "
                f"X={calibration.model_x_scale:.3f}, "
                f"Y={calibration.model_y_scale:.3f}, "
                f"UP={calibration.model_up_scale:.3f}"
            )

            table_ok = (
                calibration.table_plane_rms_m * 1000.0 <= args.max_table_fit_rms_mm
            )
            affine_ok = (
                calibration.affine_fit_rms_m * 1000.0 <= args.max_affine_fit_rms_mm
            )
            condition_ok = (
                calibration.linear_condition_number <= args.max_linear_condition_number
            )
            orientation_ok = up_orientation_drift <= args.max_up_orientation_drift_deg
            up_scale_ok = args.min_up_scale <= calibration.model_up_scale <= args.max_up_scale
            measurement_ok = (
                table_ok and affine_ok and condition_ok and orientation_ok and up_scale_ok
            )

            report["acceptance_checks"] = {
                "table_fit_ok": table_ok,
                "affine_fit_ok": affine_ok,
                "linear_mapping_well_conditioned": condition_ok,
                "up_probe_orientation_ok": orientation_ok,
                "up_scale_plausible": up_scale_ok,
                "measurement_accepted": measurement_ok,
                "diagnostic_up_vs_table_normal_angle_deg": (
                    calibration.up_vs_table_normal_angle_deg
                ),
                "limits": {
                    "max_table_fit_rms_mm": args.max_table_fit_rms_mm,
                    "max_affine_fit_rms_mm": args.max_affine_fit_rms_mm,
                    "max_linear_condition_number": args.max_linear_condition_number,
                    "max_up_orientation_drift_deg": args.max_up_orientation_drift_deg,
                    "min_up_scale": args.min_up_scale,
                    "max_up_scale": args.max_up_scale,
                },
            }

            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(f"\nDiagnostic report written to {args.output}")

            if not measurement_ok:
                print("\nWORKSPACE MEASUREMENT NOT ACCEPTED.")
                if not table_ok:
                    print(
                        f"  table fit RMS exceeds {args.max_table_fit_rms_mm:.1f} mm"
                    )
                if not affine_ok:
                    print(
                        f"  affine fit RMS exceeds {args.max_affine_fit_rms_mm:.1f} mm"
                    )
                if not condition_ok:
                    print(
                        "  the local physical-to-model mapping is too ill-conditioned "
                        "for reliable inversion"
                    )
                if not orientation_ok:
                    print(
                        "  D->UP tool orientation changed too much for the lower-finger "
                        "probe approximation"
                    )
                if not up_scale_ok:
                    print(
                        "  the model displacement for the measured UP height is implausibly "
                        "small or large"
                    )
                print(
                    "No workspace calibration was saved, and no powered Cartesian motion "
                    "was attempted."
                )
                return 2

            store = WorkspaceCalibrationStore(
                config.robot_id,
                path=args.workspace_output,
            )
            saved = store.save(calibration)
            report["workspace_calibration_path"] = str(store.path)
            report["workspace_id"] = saved.workspace_id
            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(
                f"\nMeasured workspace calibration saved to {store.path}\n"
                f"workspace_id={saved.workspace_id}"
            )
            print(
                "Status remains UNVALIDATED for autonomous Cartesian motion. "
                "This workflow has only measured the workspace mapping."
            )
            print("No powered Cartesian motion was attempted.")
            return 0
        finally:
            try:
                arm.relax()
                print("Motors relaxed.")
            except Exception as exc:
                print(f"WARNING: could not confirm relax during cleanup: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
