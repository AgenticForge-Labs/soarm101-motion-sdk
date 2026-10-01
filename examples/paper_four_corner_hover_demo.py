"""Supervised four-corner paper hover demo for a physical SO-ARM101.

The operator manually teaches all four corners of a rectangular sheet while torque is
disabled. The script then creates one hover target 50 mm (configurable) along SDK base +Z
from each captured model TCP and, only after explicit confirmations, moves slowly around
the taught perimeter.

This is a motion diagnostic, not a kinematic calibration and not a substitute for the
three-corner paper validation. No fourth corner is predicted and no measured paper
geometry is used to modify a target.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from math import hypot
from pathlib import Path
from typing import Mapping

import numpy as np
from numpy.typing import NDArray

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


def perimeter_distances_mm(points_m: Mapping[str, FloatArray]) -> dict[str, float]:
    """Return perimeter and diagonal distances for clockwise A/B/C/D points."""

    required = ("A", "B", "C", "D")
    missing = [name for name in required if name not in points_m]
    if missing:
        raise ValueError(f"missing paper corners: {missing}")

    def distance(first: str, second: str) -> float:
        return float(
            np.linalg.norm(
                np.asarray(points_m[second], dtype=float)
                - np.asarray(points_m[first], dtype=float)
            )
            * 1000.0
        )

    return {
        "AB": distance("A", "B"),
        "BC": distance("B", "C"),
        "CD": distance("C", "D"),
        "DA": distance("D", "A"),
        "AC": distance("A", "C"),
        "BD": distance("B", "D"),
    }


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
    input("Press Enter when the fixed lower finger is exactly on this corner... ")
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


def _hover_pose(sample: CornerSample, hover_height_mm: float) -> Pose:
    return Pose(
        sample.position_m + np.array([0.0, 0.0, hover_height_mm / 1000.0]),
        sample.rotation,
    )


def _preflight_hover(
    arm: SOARM101,
    sample: CornerSample,
    target: Pose,
) -> None:
    solution = arm.solve_ik(
        target,
        seed=sample.joints_rad,
        orientation_mode="compatible",
    )
    if not solution.success:
        raise RuntimeError(
            f"hover above {sample.name} is not reachable from that taught corner pose: "
            f"{solution.message}"
        )
    print(
        f"  {sample.name}: IK position error "
        f"{solution.position_error_m * 1000.0:.2f} mm; orientation error "
        f"{solution.orientation_error_rad * 180.0 / np.pi:.2f} deg"
    )


def _move_hover(
    arm: SOARM101,
    sample: CornerSample,
    target: Pose,
    *,
    speed_mm_s: float,
    acceleration_mm_s2: float,
    allow_floor_recovery: bool = False,
) -> dict[str, object]:
    print(f"\nMoving to {sample.name} hover...")
    result = arm.move_linear(
        target,
        orientation_mode="compatible",
        speed=speed_mm_s / 1000.0,
        acceleration=acceleration_mm_s2 / 1000.0,
        allow_floor_recovery=allow_floor_recovery,
    )
    if not result.accepted or not result.completed:
        raise RuntimeError(f"motion to {sample.name} hover did not complete: {result}")
    actual = arm.get_position()
    actual_xyz_mm = tuple(float(value * 1000.0) for value in actual.position)
    print(
        "  model TCP after move = "
        f"({actual_xyz_mm[0]:.1f}, {actual_xyz_mm[1]:.1f}, {actual_xyz_mm[2]:.1f}) mm"
    )
    return {
        "corner": sample.name,
        "actual_tcp_xyz_mm": actual_xyz_mm,
    }


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
        "--hover-height-mm",
        type=float,
        default=50.0,
        help="SDK base +Z offset above every manually captured corner",
    )
    parser.add_argument("--speed-mm-s", type=float, default=5.0)
    parser.add_argument("--acceleration-mm-s2", type=float, default=20.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("paper-four-corner-hover.json"),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.width_mm <= 0 or args.height_mm <= 0:
        raise SystemExit("paper width and height must be positive")
    if args.hover_height_mm <= 0:
        raise SystemExit("--hover-height-mm must be positive")
    if args.speed_mm_s <= 0:
        raise SystemExit("--speed-mm-s must be positive")
    if args.acceleration_mm_s2 <= 0:
        raise SystemExit("--acceleration-mm-s2 must be positive")

    config = _resolve_config(args)
    print("Paper four-corner hover demo")
    print(f"Reference sheet: {args.width_mm:.1f} x {args.height_mm:.1f} mm")
    print("Teach ALL FOUR corners clockwise in this order:")
    print()
    print("  D ---------------- C")
    print("  |                  |")
    print("  |                  |")
    print("  A ---------------- B")
    print()
    print("A->B and C->D are the SHORT/WIDTH edges.")
    print("B->C and D->A are the LONG/HEIGHT edges.")
    print("This matches A->B left-to-right, then C on the right side.")
    print()
    print(
        f"After teaching, targets are {args.hover_height_mm:.1f} mm along SDK base +Z "
        "from each captured model TCP."
    )
    print("No fourth corner is predicted and measured paper geometry does not alter the targets.")
    print("Keep the GUI disconnected, clear the whole workspace, and keep physical power reachable.")

    report: dict[str, object] = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "robot_id": config.robot_id,
        "port": config.port,
        "paper": {"width_mm": args.width_mm, "height_mm": args.height_mm},
        "hover_height_mm": args.hover_height_mm,
        "orientation_mode": "compatible",
        "capture_order": ["A", "B", "C", "D"],
        "probe": "fixed lower gripper finger; stock modeled gripper TCP used for FK",
        "moves": [],
    }

    with SOARM101(config) as arm:
        try:
            input("\nPress Enter to enable torque briefly and open the moving jaw... ")
            arm.enable()
            arm.tool.open()
            print("Moving jaw is open. Relaxing all motors for manual corner teaching.")
            arm.relax()

            samples = [
                _capture_corner(arm, "A", "touch the lower-left / first corner"),
                _capture_corner(
                    arm,
                    "B",
                    "touch the adjacent corner along the SHORT/WIDTH edge",
                ),
                _capture_corner(
                    arm,
                    "C",
                    "from B, touch the next corner along the LONG/HEIGHT edge",
                ),
                _capture_corner(
                    arm,
                    "D",
                    "touch the remaining corner along the SHORT/WIDTH edge from C",
                ),
            ]
            report["corners"] = [asdict(sample) for sample in samples]

            points = {sample.name: sample.position_m for sample in samples}
            distances = perimeter_distances_mm(points)
            report["model_distances_mm"] = distances
            diagonal_mm = hypot(args.width_mm, args.height_mm)

            print("\nModel distances for the manually taught physical paper:")
            print(
                f"  A->B {distances['AB']:.1f} mm "
                f"(physical width {args.width_mm:.1f})"
            )
            print(
                f"  B->C {distances['BC']:.1f} mm "
                f"(physical height {args.height_mm:.1f})"
            )
            print(
                f"  C->D {distances['CD']:.1f} mm "
                f"(physical width {args.width_mm:.1f})"
            )
            print(
                f"  D->A {distances['DA']:.1f} mm "
                f"(physical height {args.height_mm:.1f})"
            )
            print(
                f"  diagonals A->C={distances['AC']:.1f}, B->D={distances['BD']:.1f} mm "
                f"(physical {diagonal_mm:.1f})"
            )
            print("These distances are diagnostic only; they do not move or rescale a corner.")

            targets = {
                sample.name: _hover_pose(sample, args.hover_height_mm)
                for sample in samples
            }
            report["hover_targets_xyz_mm"] = {
                name: tuple(float(value * 1000.0) for value in target.position)
                for name, target in targets.items()
            }
            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(f"\nIntermediate report written to {args.output}")

            print("\nPreflighting all four hover targets with torque OFF...")
            for sample in samples:
                _preflight_hover(arm, sample, targets[sample.name])

            print(
                "\nThe first lift may start below the generic base-Z floor in model coordinates. "
                "For this lift only, the workspace validator permits a monotonic upward escape "
                "that must finish back inside the normal workspace envelope; reach, base keep-out, "
                "self-clearance, joint, effort, fault, and following-error guards remain active."
            )
            input(
                f"\nPress Enter to enable torque and make ONLY the first "
                f"{args.hover_height_mm:.0f} mm lift above D... "
            )
            arm.enable()
            moves = report["moves"]
            assert isinstance(moves, list)
            moves.append(
                _move_hover(
                    arm,
                    samples[3],
                    targets["D"],
                    speed_mm_s=args.speed_mm_s,
                    acceleration_mm_s2=args.acceleration_mm_s2,
                    allow_floor_recovery=True,
                )
            )
            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

            input(
                "\nVerify the fixed finger moved UP and is safely above the table. "
                "Press Enter to traverse the taught perimeter D->A->B->C->D, "
                "or Ctrl+C to stop and relax... "
            )
            for name in ("A", "B", "C", "D"):
                sample = next(item for item in samples if item.name == name)
                moves.append(
                    _move_hover(
                        arm,
                        sample,
                        targets[name],
                        speed_mm_s=args.speed_mm_s,
                        acceleration_mm_s2=args.acceleration_mm_s2,
                    )
                )
                args.output.write_text(
                    json.dumps(report, indent=2) + "\n",
                    encoding="utf-8",
                )

            print("\nFour-corner hover traversal completed.")
            return 0
        finally:
            try:
                arm.relax()
                print("Motors relaxed.")
            except Exception as exc:
                print(f"WARNING: could not confirm relax during cleanup: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
