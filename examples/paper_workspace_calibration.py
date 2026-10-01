"""Manual paper/workspace calibration for a physical SO-ARM101.

The operator teaches all four paper corners on the table, then manually teaches one
known physical height above corner D. Those five physical correspondences are used to
fit a local affine map from paper/workspace coordinates into the SDK kinematic model.

After the final UP teaching sample, the script counts down and enables torque so the arm
holds that exact pose instead of sagging. Elevated Cartesian endpoints are then built
from known-reachable joint configurations inferred from the taught corners plus the
taught D->UP posture change. The actual motion between those endpoints is still executed
with Cartesian move_linear() and position-only IK.

A saved teaching can also be replayed later from any ordinary resting pose with --replay.
The first Cartesian move to D_UP validates the destination but relaxes the coarse
workspace-path model for the unknown starting pose; subsequent paper segments use the
normal full workspace path checks.

The resulting calibration records:
- where the taught table plane lies in model coordinates;
- how the physical paper X/Y directions map into model coordinates; and
- most importantly, how a manually demonstrated physical-UP direction maps into the
  model on this exact arm/setup.

The powered traversal is a supervised workspace demonstration and its result is recorded
as evidence. A saved workspace remains unvalidated for broader autonomous Cartesian use
until that demonstration succeeds and the resulting evidence is reviewed.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from math import hypot
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from soarm101_motion import Pose, SOARM101, SOARM101Config
from soarm101_motion.workstation import WorkstationProfileStore
from soarm101_motion.workspace import WorkspaceCalibration, WorkspaceCalibrationStore, fit_paper_workspace

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
            joint_position_tolerance_rad=np.deg2rad(args.settle_tolerance_deg),
            motion_completion_timeout_s=args.settle_timeout_s,
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
        joint_position_tolerance_rad=np.deg2rad(args.settle_tolerance_deg),
        motion_completion_timeout_s=args.settle_timeout_s,
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


def evaluate_measurement_acceptance(
    calibration: WorkspaceCalibration,
    *,
    up_orientation_drift_deg: float,
    max_table_fit_rms_mm: float,
    max_affine_fit_rms_mm: float,
    max_linear_condition_number: float,
    min_up_scale: float,
    max_up_scale: float,
) -> tuple[bool, dict[str, object]]:
    table_ok = calibration.table_plane_rms_m * 1000.0 <= max_table_fit_rms_mm
    affine_ok = calibration.affine_fit_rms_m * 1000.0 <= max_affine_fit_rms_mm
    condition_ok = calibration.linear_condition_number <= max_linear_condition_number
    up_scale_ok = min_up_scale <= calibration.model_up_scale <= max_up_scale
    measurement_ok = table_ok and affine_ok and condition_ok and up_scale_ok

    checks: dict[str, object] = {
        "table_fit_ok": table_ok,
        "affine_fit_ok": affine_ok,
        "linear_mapping_well_conditioned": condition_ok,
        "up_scale_plausible": up_scale_ok,
        "measurement_accepted": measurement_ok,
        "diagnostic_up_orientation_drift_deg": up_orientation_drift_deg,
        "diagnostic_up_vs_table_normal_angle_deg": (
            calibration.up_vs_table_normal_angle_deg
        ),
        "limits": {
            "max_table_fit_rms_mm": max_table_fit_rms_mm,
            "max_affine_fit_rms_mm": max_affine_fit_rms_mm,
            "max_linear_condition_number": max_linear_condition_number,
            "min_up_scale": min_up_scale,
            "max_up_scale": max_up_scale,
        },
    }
    return measurement_ok, checks


def _joint_delta(first: dict[str, float], second: dict[str, float]) -> dict[str, float]:
    return {name: float(second[name] - first[name]) for name in first}


def _offset_joints(
    base: dict[str, float],
    delta: dict[str, float],
) -> dict[str, float]:
    return {name: float(base[name] + delta[name]) for name in base}


def reachable_demo_targets(
    arm: SOARM101,
    *,
    corners: dict[str, Sample],
    up_sample: Sample,
) -> tuple[dict[str, np.ndarray], dict[str, dict[str, float]]]:
    """Build Cartesian endpoints from known-reachable inferred joint configurations.

    The taught D->UP joint delta is applied to each taught paper corner. FK of those
    joint configurations defines the Cartesian endpoints. The demonstration still uses
    move_linear() between endpoints; this construction only makes each endpoint itself
    reachable by definition.
    """

    required = {"A", "B", "C", "D"}
    if set(corners) != required:
        raise ValueError(f"expected corners {sorted(required)}, got {sorted(corners)}")

    lift_delta = _joint_delta(corners["D"].joints_rad, up_sample.joints_rad)
    elevated_joints = {
        "D_UP": dict(up_sample.joints_rad),
        "A_UP": _offset_joints(corners["A"].joints_rad, lift_delta),
        "B_UP": _offset_joints(corners["B"].joints_rad, lift_delta),
        "C_UP": _offset_joints(corners["C"].joints_rad, lift_delta),
        "D_UP_RETURN": dict(up_sample.joints_rad),
    }
    center_base = {
        name: float(
            np.mean([corners[label].joints_rad[name] for label in ("A", "B", "C", "D")])
        )
        for name in up_sample.joints_rad
    }
    elevated_joints["CENTER_UP"] = _offset_joints(center_base, lift_delta)

    limits = arm.get_joint_limits()
    for target_name, joints in elevated_joints.items():
        for joint_name, value in joints.items():
            lower, upper = limits[joint_name]
            if not lower <= value <= upper:
                raise RuntimeError(
                    f"{target_name} inferred {joint_name}={value:.4f} rad is outside "
                    f"{lower:.4f}..{upper:.4f}"
                )

    positions = {
        name: arm.model.forward(joints, tcp=arm.active_tcp).position.copy()
        for name, joints in elevated_joints.items()
    }
    return positions, elevated_joints


def preflight_demo_targets(
    arm: SOARM101,
    positions: dict[str, np.ndarray],
    *,
    preferred_seeds: dict[str, dict[str, float]],
    rotation: np.ndarray,
) -> list[dict[str, object]]:
    """Solve each reachable Cartesian endpoint read-only before powered motion."""

    results: list[dict[str, object]] = []
    previous_solution: dict[str, float] | None = None
    for name, position in positions.items():
        seeds = [preferred_seeds[name]]
        if previous_solution is not None:
            seeds.append(previous_solution)

        attempts: list[dict[str, object]] = []
        successes: list[tuple[float, object, int]] = []
        fingerprints: set[tuple[tuple[str, float], ...]] = set()
        for seed_index, seed in enumerate(seeds):
            fingerprint = tuple(
                sorted((joint, round(value, 10)) for joint, value in seed.items())
            )
            if fingerprint in fingerprints:
                continue
            fingerprints.add(fingerprint)
            solution = arm.solve_ik(
                Pose(position, rotation),
                seed=seed,
                orientation_mode="position_only",
            )
            attempts.append(
                {
                    "seed_index": seed_index,
                    "success": solution.success,
                    "position_error_mm": float(solution.position_error_m * 1000.0),
                    "message": solution.message,
                }
            )
            if solution.success:
                successes.append((float(solution.position_error_m), solution, seed_index))

        if not successes:
            best = min(attempts, key=lambda item: float(item["position_error_mm"]))
            raise RuntimeError(
                f"{name} preflight IK failed; best position error="
                f"{float(best['position_error_mm']):.2f} mm; {best['message']}"
            )

        _, solution, chosen_seed_index = min(successes, key=lambda item: item[0])
        previous_solution = dict(solution.joints)
        results.append(
            {
                "name": name,
                "target_model_xyz_mm": [
                    float(value * 1000.0) for value in position
                ],
                "position_error_mm": float(solution.position_error_m * 1000.0),
                "joints_rad": dict(solution.joints),
                "chosen_seed_index": chosen_seed_index,
                "attempts": attempts,
            }
        )
    return results


def run_demo_targets(
    arm: SOARM101,
    positions: dict[str, np.ndarray],
    *,
    rotation: np.ndarray,
    speed_mm_s: float,
    acceleration_mm_s2: float,
    report_moves: list[dict[str, object]],
) -> None:
    for name, position in positions.items():
        print(f"\nMoving to {name}...")
        result = arm.move_linear(
            Pose(position, rotation),
            orientation_mode="position_only",
            speed=speed_mm_s / 1000.0,
            acceleration=acceleration_mm_s2 / 1000.0,
            workspace_check="target_only",
        )
        if not result.accepted or not result.completed:
            raise RuntimeError(f"{name} motion did not complete: {result}")
        actual = arm.get_position()
        actual_xyz_mm = [float(value * 1000.0) for value in actual.position]
        print(
            "  model TCP after move = "
            f"({actual_xyz_mm[0]:.1f}, {actual_xyz_mm[1]:.1f}, "
            f"{actual_xyz_mm[2]:.1f}) mm"
        )
        report_moves.append(
            {
                "name": name,
                "target_model_xyz_mm": [
                    float(value * 1000.0) for value in position
                ],
                "actual_model_xyz_mm": actual_xyz_mm,
            }
        )


def _countdown_hold(arm: SOARM101, seconds: int = 3) -> None:
    print("\nTeaching complete. Keep the arm still; torque will engage and hold this pose.")
    for remaining in range(seconds, 0, -1):
        print(f"  holding in {remaining}...")
        time.sleep(1.0)
    arm.enable()
    print("Torque enabled. Arm is holding its current pose.")


def _hold_until_operator_release(arm: SOARM101, message: str) -> None:
    """Keep torque on after a run result until the operator deliberately releases it."""
    arm.stop()
    print(f"\n{message}")
    print("Arm is HOLDING its current pose; it will not automatically relax and drop.")
    input("Support the arm, then press Enter when you are ready to relax and exit... ")


def _sample_from_payload(payload: dict[str, object], name: str) -> Sample:
    samples = payload.get("samples")
    if not isinstance(samples, list):
        raise ValueError("saved report has no samples")
    for item in samples:
        if not isinstance(item, dict) or item.get("name") != name:
            continue
        return Sample(
            name=name,
            tcp_xyz_mm=tuple(float(value) for value in item["tcp_xyz_mm"]),
            tcp_rpy_deg=tuple(float(value) for value in item["tcp_rpy_deg"]),
            rotation_matrix=tuple(
                tuple(float(value) for value in row)
                for row in item["rotation_matrix"]
            ),
            joints_rad={
                str(joint): float(value)
                for joint, value in dict(item["joints_rad"]).items()
            },
        )
    raise ValueError(f"saved report is missing sample {name}")


def load_saved_teaching(path: Path) -> tuple[dict[str, object], list[Sample], Sample]:
    payload_obj = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload_obj, dict):
        raise ValueError("saved report root must be a JSON object")
    payload = dict(payload_obj)
    corners = [_sample_from_payload(payload, name) for name in ("A", "B", "C", "D")]
    up = _sample_from_payload(payload, "UP")
    return payload, corners, up


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
    parser.add_argument(
        "--speed-mm-s",
        type=float,
        default=5.0,
        help="supervised elevated-paper demonstration speed",
    )
    parser.add_argument(
        "--acceleration-mm-s2",
        type=float,
        default=20.0,
        help="supervised elevated-paper demonstration acceleration",
    )
    parser.add_argument(
        "--settle-tolerance-deg",
        type=float,
        default=3.0,
        help=(
            "paper-demo joint completion tolerance in degrees; default 3.0 is intentionally "
            "looser than the SDK-wide precision default for this supervised hobby-arm test"
        ),
    )
    parser.add_argument(
        "--settle-timeout-s",
        type=float,
        default=8.0,
        help="paper-demo final settle timeout per Cartesian segment",
    )
    parser.add_argument(
        "--measure-only",
        action="store_true",
        help="save the workspace measurement but skip the powered paper demonstration",
    )
    parser.add_argument(
        "--replay",
        action="store_true",
        help=(
            "reuse the saved A/B/C/D/UP teaching in --output and run the Cartesian "
            "linear sequence from the arm's current resting pose"
        ),
    )
    return parser


def run_saved_replay(args: argparse.Namespace, config: SOARM101Config) -> int:
    report, samples, up = load_saved_teaching(args.output)
    corner_samples = {sample.name: sample for sample in samples}
    reference_height = float(report.get("reference_height_mm", 0.0))
    if reference_height <= 0.0:
        raise RuntimeError("saved report has no valid reference_height_mm")

    with SOARM101(config) as arm:
        try:
            store = WorkspaceCalibrationStore(config.robot_id, path=args.workspace_output)
            saved = store.load()
            if arm.calibration_id and saved.arm_calibration_id != arm.calibration_id:
                raise RuntimeError(
                    "saved workspace calibration does not match the active motor calibration"
                )

            print("Paper Cartesian linear replay")
            print(
                f"Loaded saved teaching at {reference_height:.1f} mm reference height from "
                f"{args.output}."
            )
            print(
                f"Paper-demo settle criterion: {args.settle_tolerance_deg:.1f} deg for "
                f"{args.settle_timeout_s:.1f} s."
            )
            print(
                "The arm may start from any ordinary resting pose. This supervised paper "
                "validation uses target-only coarse workspace checking for each segment; "
                "dynamic, joint, effort, fault, following-error, and timeout guards remain active."
            )
            _countdown_hold(arm)
            arm.tool.open()

            demo_positions, preferred_seeds = reachable_demo_targets(
                arm,
                corners=corner_samples,
                up_sample=up,
            )
            print("\nRead-only endpoint preflight while holding the current pose...")
            preflight = preflight_demo_targets(
                arm,
                demo_positions,
                preferred_seeds=preferred_seeds,
                rotation=up.rotation,
            )
            for item in preflight:
                xyz = item["target_model_xyz_mm"]
                assert isinstance(xyz, list)
                print(
                    f"  {item['name']}: "
                    f"({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}) mm; "
                    f"IK error {item['position_error_mm']:.2f} mm"
                )

            report["replayed_at"] = datetime.now(timezone.utc).isoformat()
            report["demo_target_strategy"] = (
                "reachable_FK_endpoints_from_taught_corner_joints_plus_taught_lift_delta"
            )
            report["demo_preflight"] = preflight
            report["demo_targets_model_xyz_mm"] = {
                name: [float(value * 1000.0) for value in position]
                for name, position in demo_positions.items()
            }
            report["demo_moves"] = []
            report["autonomous_motion_attempted"] = False
            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

            input(
                "\nPress Enter to run D_UP -> A_UP -> B_UP -> C_UP -> "
                "D_UP -> CENTER_UP with Cartesian move_linear(), or Ctrl+C to stop... "
            )
            report["autonomous_motion_attempted"] = True
            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

            moves = report["demo_moves"]
            assert isinstance(moves, list)
            try:
                run_demo_targets(
                    arm,
                    demo_positions,
                    rotation=up.rotation,
                    speed_mm_s=args.speed_mm_s,
                    acceleration_mm_s2=args.acceleration_mm_s2,
                    report_moves=moves,
                )
            except Exception as exc:
                report["demo_completed"] = False
                report["demo_error"] = f"{type(exc).__name__}: {exc}"
                args.output.write_text(
                    json.dumps(report, indent=2) + "\n",
                    encoding="utf-8",
                )
                _hold_until_operator_release(
                    arm,
                    f"Motion stopped: {type(exc).__name__}: {exc}",
                )
                arm.relax()
                print("Motors relaxed.")
                return 2

            report["demo_completed"] = True
            report.pop("demo_error", None)
            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            _hold_until_operator_release(
                arm,
                "Cartesian linear replay completed at CENTER_UP.",
            )
            arm.relax()
            print("Motors relaxed.")
            return 0
        finally:
            try:
                arm.relax()
            except Exception as exc:
                print(f"WARNING: could not confirm relax during cleanup: {exc}")


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
    if not 0.0 < args.min_up_scale < args.max_up_scale:
        raise SystemExit("--min-up-scale must be positive and below --max-up-scale")
    if args.speed_mm_s <= 0.0:
        raise SystemExit("--speed-mm-s must be positive")
    if args.acceleration_mm_s2 <= 0.0:
        raise SystemExit("--acceleration-mm-s2 must be positive")
    if not 0.1 <= args.settle_tolerance_deg <= 10.0:
        raise SystemExit("--settle-tolerance-deg must be between 0.1 and 10")
    if args.settle_timeout_s <= 0.0:
        raise SystemExit("--settle-timeout-s must be positive")
    if args.replay and args.measure_only:
        raise SystemExit("--replay and --measure-only cannot be used together")

    config = _resolve_config(args)
    if args.replay:
        return run_saved_replay(args, config)

    print("Paper workspace calibration — MANUAL / TORQUE-OFF GEOMETRY")
    print(f"Reference sheet: {args.width_mm:.1f} x {args.height_mm:.1f} mm")
    print(
        "After an accepted measurement, the default workflow performs one supervised "
        "elevated-paper demonstration unless --measure-only is supplied."
    )
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
        "Use a ruler, rigid spacer, paper edge, gauge block, or another physical reference. "
        "Allow the wrist/tool orientation to change naturally as needed to place the fixed "
        "finger at the measured physical point."
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
        "demo_requested": not args.measure_only,
        "demo_speed_mm_s": args.speed_mm_s,
        "demo_acceleration_mm_s2": args.acceleration_mm_s2,
        "probe": "fixed lower gripper finger; stock modeled gripper TCP used for FK",
        "demo_moves": [],
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
            _countdown_hold(arm)

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

            measurement_ok, acceptance_checks = evaluate_measurement_acceptance(
                calibration,
                up_orientation_drift_deg=up_orientation_drift,
                max_table_fit_rms_mm=args.max_table_fit_rms_mm,
                max_affine_fit_rms_mm=args.max_affine_fit_rms_mm,
                max_linear_condition_number=args.max_linear_condition_number,
                min_up_scale=args.min_up_scale,
                max_up_scale=args.max_up_scale,
            )
            report["acceptance_checks"] = acceptance_checks

            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(f"\nDiagnostic report written to {args.output}")

            if not measurement_ok:
                print("\nWORKSPACE MEASUREMENT NOT ACCEPTED.")
                if not bool(acceptance_checks["table_fit_ok"]):
                    print(
                        f"  table fit RMS exceeds {args.max_table_fit_rms_mm:.1f} mm"
                    )
                if not bool(acceptance_checks["affine_fit_ok"]):
                    print(
                        f"  affine fit RMS exceeds {args.max_affine_fit_rms_mm:.1f} mm"
                    )
                if not bool(acceptance_checks["linear_mapping_well_conditioned"]):
                    print(
                        "  the local physical-to-model mapping is too ill-conditioned "
                        "for reliable inversion"
                    )
                if not bool(acceptance_checks["up_scale_plausible"]):
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

            if args.measure_only:
                print(
                    "Measurement saved. Powered workspace demonstration skipped by "
                    "--measure-only."
                )
                print(
                    "Status remains UNVALIDATED for broader autonomous Cartesian motion."
                )
                return 0

            corner_samples = {sample.name: sample for sample in samples}
            demo_positions, preferred_seeds = reachable_demo_targets(
                arm,
                corners=corner_samples,
                up_sample=up,
            )
            report["demo_target_strategy"] = (
                "reachable_FK_endpoints_from_taught_corner_joints_plus_taught_lift_delta"
            )
            report["measured_up_delta_model_mm"] = [
                float(value * 1000.0)
                for value in (up.position_m - d.position_m)
            ]
            report["demo_targets_model_xyz_mm"] = {
                name: [float(value * 1000.0) for value in position]
                for name, position in demo_positions.items()
            }

            print(
                f"\nRead-only endpoint preflight at the taught "
                f"{args.reference_height_mm:.1f} mm reference height while holding UP..."
            )
            preflight = preflight_demo_targets(
                arm,
                demo_positions,
                preferred_seeds=preferred_seeds,
                rotation=up.rotation,
            )
            report["demo_preflight"] = preflight
            for item in preflight:
                xyz = item["target_model_xyz_mm"]
                assert isinstance(xyz, list)
                print(
                    f"  {item['name']}: model target "
                    f"({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}) mm; "
                    f"IK error {item['position_error_mm']:.2f} mm"
                )
            args.output.write_text(
                json.dumps(report, indent=2) + "\n",
                encoding="utf-8",
            )

            print(
                "\nThe arm is holding the manually taught UP pose."
            )
            print(
                "The Cartesian endpoints come from known-reachable FK poses inferred from "
                "the taught corners plus the taught D->UP joint change:"
            )
            print("  D_UP -> A_UP -> B_UP -> C_UP -> D_UP -> CENTER_UP")
            print(
                f"The D->UP posture change was taught at {args.reference_height_mm:.1f} mm "
                "physical height. Each endpoint is reachable by construction; the motion "
                "between endpoints is still Cartesian move_linear()."
            )
            input(
                "\nPress Enter to run the full Cartesian linear path, "
                "or Ctrl+C to stop while holding... "
            )

            report["autonomous_motion_attempted"] = True
            args.output.write_text(
                json.dumps(report, indent=2) + "\n",
                encoding="utf-8",
            )
            moves = report["demo_moves"]
            assert isinstance(moves, list)
            try:
                run_demo_targets(
                    arm,
                    demo_positions,
                    rotation=up.rotation,
                    speed_mm_s=args.speed_mm_s,
                    acceleration_mm_s2=args.acceleration_mm_s2,
                    report_moves=moves,
                )
            except Exception as exc:
                report["demo_completed"] = False
                report["demo_error"] = f"{type(exc).__name__}: {exc}"
                args.output.write_text(
                    json.dumps(report, indent=2) + "\n",
                    encoding="utf-8",
                )
                _hold_until_operator_release(
                    arm,
                    f"Motion stopped: {type(exc).__name__}: {exc}",
                )
                arm.relax()
                print("Motors relaxed.")
                return 2

            report["demo_completed"] = True
            args.output.write_text(
                json.dumps(report, indent=2) + "\n",
                encoding="utf-8",
            )
            _hold_until_operator_release(
                arm,
                "Elevated paper demonstration completed at CENTER_UP.",
            )
            arm.relax()
            print("Motors relaxed.")
            return 0
        finally:
            if arm.is_enabled:
                try:
                    arm.relax()
                    print("Motors relaxed during cleanup.")
                except Exception as exc:
                    print(f"WARNING: could not confirm relax during cleanup: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
