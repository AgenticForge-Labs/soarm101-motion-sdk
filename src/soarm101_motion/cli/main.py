"""Command-line interface for hardware setup, diagnostics, and simulation."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from math import pi
from pathlib import Path

from soarm101_motion import SOARM101, SOARM101Config, __version__
from soarm101_motion.calibration import default_calibration_path
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.hardware import FeetechBackend


def _hardware_config(args: argparse.Namespace, **overrides: object) -> SOARM101Config:
    values: dict[str, object] = {
        "port": args.port,
        "robot_id": getattr(args, "robot_id", "so101"),
    }
    calibration = getattr(args, "calibration", None)
    if calibration:
        values["calibration_path"] = Path(calibration)
    values.update(overrides)
    return SOARM101Config(**values)


def _cmd_info(_: argparse.Namespace) -> int:
    print(f"soarm101-motion-sdk {__version__}")
    print("runtime: direct Feetech STS3215 control; LeRobot is not required")
    print("arm: 5 pose joints + SO101Gripper tool actuator")
    return 0


def _cmd_ports(_: argparse.Namespace) -> int:
    ports = FeetechBackend.candidate_ports()
    if not ports:
        print("No serial ports found.")
        return 1
    for port in ports:
        print(port)
    return 0


def _cmd_read(args: argparse.Namespace) -> int:
    with SOARM101(_hardware_config(args)) as arm:
        positions = arm.get_joint_positions().positions
        pose = arm.get_position().xyz_rpy()
        print("Joint positions (radians):")
        for name in ARM_JOINTS:
            print(f"  {name:15s} {positions[name]: .6f}")
        print("TCP pose (m, rad):", " ".join(f"{value:.6f}" for value in pose))
    return 0


def _cmd_diagnose(args: argparse.Namespace) -> int:
    with SOARM101(_hardware_config(args, allow_uncalibrated=args.allow_uncalibrated)) as arm:
        diagnostics = [asdict(item) for item in arm.diagnostics()]
    if args.json:
        print(json.dumps(diagnostics, indent=2))
    else:
        print("name             id model raw   temp C voltage current moving status/error")
        for item in diagnostics:
            if item["error"]:
                print(f"{item['name']:16s} {item['motor_id']:2d} ERROR {item['error']}")
            else:
                print(
                    f"{item['name']:16s} {item['motor_id']:2d} {item['model_number']:5d} "
                    f"{item['position_raw']:4d} {item['temperature_c']:6.1f} "
                    f"{item['voltage_v']:7.2f} {item['current_raw']:7d} "
                    f"{str(item['moving']):6s} 0x{item['status']:02x}"
                )
    return 0 if all(item["error"] is None for item in diagnostics) else 2


def _lerobot_export_path(robot_id: str) -> Path:
    return Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots" / "so101_follower" / f"{robot_id}.json"


def _cmd_calibrate(args: argparse.Namespace) -> int:
    print("CALIBRATION SAFETY")
    print("- Remove payloads and clear the workspace.")
    print("- Keep power accessible. Torque will be disabled.")
    print("- Do not force a joint past its mechanical stop.")
    if not args.yes:
        answer = input("Type CALIBRATE to continue: ").strip()
        if answer != "CALIBRATE":
            print("Calibration cancelled.")
            return 1
    config = _hardware_config(
        args,
        allow_uncalibrated=True,
        use_stored_calibration=False,
        verify_calibration_on_connect=False,
        configure_motors_on_connect=True,
    )
    backend = FeetechBackend(config)
    backend.connect()
    try:
        backend.disable_torque()
        input(
            "Place every joint near the middle of its usable range, including the gripper, "
            "then press ENTER."
        )
        print(
            f"For the next {args.seconds:.1f} seconds, move every joint and the gripper "
            "smoothly through its full safe range."
        )
        calibration = backend.interactive_calibration(record_seconds=args.seconds)
        output = Path(args.output) if args.output else default_calibration_path(args.robot_id)
        lerobot_path = _lerobot_export_path(args.robot_id) if args.export_lerobot else None
        saved = backend.save_calibration(calibration, path=output, lerobot_path=lerobot_path)
        print(f"Calibration saved to {saved}")
        if lerobot_path:
            print(f"LeRobot-compatible copy saved to {lerobot_path}")
    finally:
        backend.disconnect()
    return 0


def _cmd_move_joints(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move hardware without --yes.", file=sys.stderr)
        return 2
    values = [value * pi / 180.0 if args.degrees else value for value in args.joints]
    with SOARM101(_hardware_config(args)) as arm:
        arm.enable()
        result = arm.move_joints(values, speed=args.speed, acceleration=args.acceleration)
        print(result)
    return 0


def _cmd_sim_demo(args: argparse.Namespace) -> int:
    arm = SOARM101.simulated(gui=args.gui, realtime=args.realtime)
    with arm:
        arm.enable()
        arm.move_joints([0.25, -0.45, 0.65, 0.20, -0.15])
        arm.tool.close()
        arm.tool.open()
        current = arm.get_position()
        target = type(current)(current.position + [0.015, 0.0, 0.010], current.rotation)
        arm.move_linear(target, orientation_mode="position_only", speed=0.02)
        print("Final joints:", dict(arm.get_joint_positions().positions))
        print("Final TCP:", arm.get_position().xyz_rpy())
        if args.gui:
            input("Press ENTER to close the simulator.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="soarm101", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    info = sub.add_parser("info", help="show package and architecture information")
    info.set_defaults(func=_cmd_info)
    ports = sub.add_parser("ports", help="list candidate serial ports")
    ports.set_defaults(func=_cmd_ports)

    def add_hardware_options(command: argparse.ArgumentParser) -> None:
        command.add_argument("--port", required=True, help="serial port, for example /dev/ttyACM0")
        command.add_argument("--robot-id", default="so101")
        command.add_argument("--calibration")

    read = sub.add_parser("read", help="read calibrated joints and TCP pose")
    add_hardware_options(read)
    read.set_defaults(func=_cmd_read)
    diagnose = sub.add_parser("diagnose", help="read motor model, voltage, temperature, and status")
    add_hardware_options(diagnose)
    diagnose.add_argument("--json", action="store_true")
    diagnose.add_argument("--allow-uncalibrated", action="store_true")
    diagnose.set_defaults(func=_cmd_diagnose)
    calibrate = sub.add_parser("calibrate", help="interactively calibrate all six STS3215 motors")
    add_hardware_options(calibrate)
    calibrate.add_argument("--seconds", type=float, default=20.0)
    calibrate.add_argument("--output")
    calibrate.add_argument("--export-lerobot", action="store_true")
    calibrate.add_argument("--yes", action="store_true")
    calibrate.set_defaults(func=_cmd_calibrate)
    move = sub.add_parser("move-joints", help="perform one guarded five-joint move")
    add_hardware_options(move)
    move.add_argument("joints", nargs=5, type=float)
    move.add_argument("--degrees", action="store_true")
    move.add_argument("--speed", type=float)
    move.add_argument("--acceleration", type=float)
    move.add_argument("--yes", action="store_true")
    move.set_defaults(func=_cmd_move_joints)
    simulation = sub.add_parser("sim-demo", help="run joint, gripper, IK, and linear motion in simulation")
    simulation.add_argument("--gui", action="store_true", help="use optional PyBullet GUI")
    simulation.add_argument("--realtime", action="store_true")
    simulation.set_defaults(func=_cmd_sim_demo)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
