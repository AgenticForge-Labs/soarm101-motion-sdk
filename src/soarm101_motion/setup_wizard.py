"""Guided first-run setup for an assembled SO-ARM101."""

from __future__ import annotations

import argparse
import sys
from dataclasses import asdict
from pathlib import Path

from soarm101_motion.calibration import default_calibration_path
from soarm101_motion.config import SOARM101Config
from soarm101_motion.exceptions import RobotConnectionError
from soarm101_motion.hardware import FeetechBackend
from soarm101_motion.kinematics import SO101KinematicModel


def _resolve_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    candidates = FeetechBackend.candidate_ports()
    if len(candidates) == 1:
        print(f"Using detected serial port: {candidates[0]}")
        return candidates[0]
    if not candidates:
        raise RobotConnectionError(
            "no serial adapter found; connect the controller board or pass --port"
        )
    raise RobotConnectionError(
        "multiple serial adapters found; pass --port with one of: " + ", ".join(candidates)
    )


def _lerobot_export_path(robot_id: str) -> Path:
    return (
        Path.home()
        / ".cache"
        / "huggingface"
        / "lerobot"
        / "calibration"
        / "robots"
        / "so101_follower"
        / f"{robot_id}.json"
    )


def _confirm(skip: bool) -> bool:
    if skip:
        return True
    print("SO-ARM101 SETUP")
    print("- Match the servo and power-supply voltage.")
    print("- Secure the base, remove payloads, and clear the workspace.")
    print("- Keep physical power immediately accessible.")
    answer = input("Type SETUP to continue: ").strip()
    return answer == "SETUP"


def _print_diagnostics(items: list[object]) -> None:
    print("name             id model raw   temp C voltage current moving status/error")
    for diagnostic in items:
        item = asdict(diagnostic)
        if item["error"]:
            print(f"{item['name']:16s} {item['motor_id']:2d} ERROR {item['error']}")
        else:
            print(
                f"{item['name']:16s} {item['motor_id']:2d} {item['model_number']:5d} "
                f"{item['position_raw']:4d} {item['temperature_c']:6.1f} "
                f"{item['voltage_v']:7.2f} {item['current_raw']:7d} "
                f"{str(item['moving']):6s} 0x{item['status']:02x}"
            )


def _run(args: argparse.Namespace) -> int:
    if not _confirm(args.yes):
        print("Setup cancelled.")
        return 1

    port = _resolve_port(args.port)
    config = SOARM101Config(
        port=port,
        robot_id=args.robot_id,
        allow_uncalibrated=True,
        use_stored_calibration=False,
        verify_calibration_on_connect=False,
        configure_motors_on_connect=False,
    )
    backend = FeetechBackend(config)
    backend.connect()
    try:
        diagnostics = backend.diagnostics()
        _print_diagnostics(diagnostics)
        bad = [item for item in diagnostics if item.error or item.status]
        if bad:
            raise RuntimeError(
                "diagnostics are not clean; fix wiring, voltage, or servo faults before setup"
            )

        backend.disable_torque()
        backend.configure_motors()
        print("Recommended motor settings applied.")

        calibration = backend.calibration
        needs_calibration = (
            args.recalibrate
            or calibration is None
            or bool(calibration.uncalibrated_motors)
        )
        if needs_calibration:
            input(
                "Place every joint and the gripper near the middle of its usable range, "
                "then press ENTER."
            )
            print(
                f"For {args.seconds:.1f} seconds, move every joint and the gripper smoothly "
                "through the full safe range. Do not force mechanical stops."
            )
            calibration = backend.interactive_calibration(record_seconds=args.seconds)
        else:
            print("Existing motor EEPROM calibration looks usable; keeping it.")

        assert calibration is not None
        output = default_calibration_path(args.robot_id)
        lerobot_path = _lerobot_export_path(args.robot_id) if args.export_lerobot else None
        saved = backend.save_calibration(
            calibration,
            path=output,
            lerobot_path=lerobot_path,
        )
        print(f"Calibration saved to {saved}")
        if lerobot_path is not None:
            print(f"LeRobot-compatible copy saved to {lerobot_path}")

        joints = backend.read_joint_positions()
        pose = SO101KinematicModel().forward(joints)
        print("Current joints (rad):", {name: round(value, 4) for name, value in joints.items()})
        print("Predicted TCP (m, rad):", " ".join(f"{value:.4f}" for value in pose.xyz_rpy()))
    finally:
        backend.disconnect()

    print("\nSetup complete. Torque is off.")
    print("Run the first supervised movement with:")
    print(
        "  soarm101 smoke-test "
        f"--port {port} --robot-id {args.robot_id} --joint shoulder_pan"
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="soarm101-setup",
        description="Configure, diagnose, and calibrate an assembled SO-ARM101 in one guided run.",
    )
    parser.add_argument("--port", help="serial port; auto-detected when exactly one is connected")
    parser.add_argument("--robot-id", default="so101", help="calibration name used by later commands")
    parser.add_argument("--seconds", type=float, default=30.0, help="calibration sweep duration")
    parser.add_argument("--recalibrate", action="store_true", help="replace an existing EEPROM calibration")
    parser.add_argument("--export-lerobot", action="store_true")
    parser.add_argument("--yes", action="store_true", help="skip only the initial typed confirmation")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return _run(args)
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except RobotConnectionError as exc:
        print(f"error: {exc}", file=sys.stderr)
        print(
            "If this is a new kit whose motor IDs were never assigned, run "
            "'soarm101 setup-motors' first.",
            file=sys.stderr,
        )
        return 1
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
