"""Command-line interface for setup, motion, diagnostics, GUI, and simulation."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from math import pi
from pathlib import Path

import numpy as np

from soarm101_motion import SOARM101, SOARM101Config, __version__
from soarm101_motion.calibration import default_calibration_path
from soarm101_motion.constants import ALL_MOTORS, ARM_JOINTS, MOTOR_IDS
from soarm101_motion.control import jog_linear_cli_units
from soarm101_motion.hardware import FeetechBackend, FeetechMotorSetup


def _hardware_config(args: argparse.Namespace, **overrides: object) -> SOARM101Config:
    values: dict[str, object] = {
        "port": args.port,
        "robot_id": getattr(args, "robot_id", "so101"),
        "configure_motors_on_connect": False,
    }
    calibration = getattr(args, "calibration", None)
    if calibration:
        values["calibration_path"] = Path(calibration)
    values.update(overrides)
    return SOARM101Config(**values)


def _confirm(args: argparse.Namespace, word: str, message: str) -> bool:
    if getattr(args, "yes", False):
        return True
    answer = input(f"{message}\nType {word} to continue: ").strip()
    return answer == word


def _cmd_info(_: argparse.Namespace) -> int:
    print(f"soarm101-motion-sdk {__version__}")
    print("runtime: direct Feetech STS3215 control; LeRobot and ROS are not required")
    print("arm: 5 pose joints + SO101Gripper tool actuator")
    print("normal connections do not rewrite motor configuration")
    return 0


def _cmd_ports(_: argparse.Namespace) -> int:
    ports = FeetechBackend.candidate_ports()
    if not ports:
        print("No serial ports found.")
        return 1
    for port in ports:
        print(port)
    return 0


def _cmd_setup_motors(args: argparse.Namespace) -> int:
    if not _confirm(
        args,
        "SETUP",
        "MOTOR SETUP WRITES ID AND BAUD RATE. Connect exactly one motor at a time and keep torque unloaded.",
    ):
        print("Motor setup cancelled.")
        return 1

    motors = (args.motor,) if args.motor else tuple(reversed(ALL_MOTORS))
    for name in motors:
        target_id = MOTOR_IDS[name]
        input(
            f"Disconnect all servos, connect only '{name}' to the controller, power it, "
            "then press ENTER."
        )
        setup = FeetechMotorSetup(
            args.port,
            target_baudrate=args.target_baudrate,
            verify_model_number=not args.skip_model_check,
            exhaustive_scan=args.exhaustive_scan,
        )
        result = setup.setup(
            target_id=target_id,
            initial_id=args.initial_id,
            initial_baudrate=args.initial_baudrate,
        )
        print(
            f"{name}: ID {result.original_id} @ {result.original_baudrate} -> "
            f"ID {result.target_id} @ {result.target_baudrate}; model {result.model_number}"
        )
    print("Motor setup complete. Reconnect the six-motor daisy chain and run 'soarm101-setup'.")
    return 0


def _cmd_configure(args: argparse.Namespace) -> int:
    if not _confirm(
        args,
        "CONFIGURE",
        "This one-time command writes recommended position/PID, phase, and gripper protection settings.",
    ):
        print("Configuration cancelled.")
        return 1
    config = _hardware_config(
        args,
        allow_uncalibrated=True,
        use_stored_calibration=False,
        verify_calibration_on_connect=False,
    )
    backend = FeetechBackend(config)
    backend.connect()
    try:
        backend.disable_torque()
        backend.configure_motors()
        print("Recommended motor settings written successfully.")
    finally:
        backend.disconnect()
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
    config = _hardware_config(
        args,
        allow_uncalibrated=args.allow_uncalibrated,
        configure_motors_on_connect=False,
    )
    with SOARM101(config) as arm:
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


def _cmd_calibrate(args: argparse.Namespace) -> int:
    print("CALIBRATION SAFETY")
    print("- Remove payloads and clear the workspace.")
    print("- Keep power accessible. Torque will be disabled.")
    print("- Do not force a joint past its mechanical stop.")
    if not _confirm(args, "CALIBRATE", "Calibration writes motor homing and range EEPROM values."):
        print("Calibration cancelled.")
        return 1
    config = _hardware_config(
        args,
        allow_uncalibrated=True,
        use_stored_calibration=False,
        verify_calibration_on_connect=False,
    )
    backend = FeetechBackend(config)
    backend.connect()
    try:
        backend.disable_torque()
        backend.configure_motors()
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


def _cmd_jog(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move hardware without --yes.", file=sys.stderr)
        return 2
    with SOARM101(_hardware_config(args)) as arm:
        arm.enable()
        result = jog_linear_cli_units(
            arm,
            frame=args.frame,
            translation_mm=(args.x_mm, args.y_mm, args.z_mm),
            rotation_rpy_deg=(args.roll_deg, args.pitch_deg, args.yaw_deg),
            orientation_mode=args.orientation_mode,
            speed_mm_s=args.speed_mm_s,
            acceleration_mm_s2=args.acceleration_mm_s2,
        )
        print(result)
    return 0


def _cmd_gripper(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move hardware without --yes.", file=sys.stderr)
        return 2
    if args.target == "open":
        position = 1.0
    elif args.target == "close":
        position = 0.0
    else:
        position = float(args.target)
    if not 0.0 <= position <= 1.0:
        raise ValueError("gripper target must be 'open', 'close', or a value in [0, 1]")
    with SOARM101(_hardware_config(args)) as arm:
        arm.enable()
        result = arm.tool.move(position)
        print(result)
    return 0


def _cmd_smoke_test(args: argparse.Namespace) -> int:
    if not _confirm(
        args,
        "MOVE",
        "The arm will enable torque, move one joint a small relative amount, then return. "
        "Remove payloads and keep physical power accessible.",
    ):
        print("Smoke test cancelled.")
        return 1
    delta = args.degrees * pi / 180.0
    with SOARM101(_hardware_config(args)) as arm:
        diagnostics = arm.diagnostics()
        bad = [item for item in diagnostics if item.error or item.status]
        if bad:
            raise RuntimeError("diagnostics are not clean; refusing powered smoke test")
        print("Starting joints:", dict(arm.get_joint_positions().positions))
        arm.enable()
        try:
            time.sleep(args.latch_seconds)
            arm.move_joints(
                {args.joint: delta},
                relative=True,
                speed=args.speed,
                acceleration=args.acceleration,
            )
            arm.move_joints(
                {args.joint: -delta},
                relative=True,
                speed=args.speed,
                acceleration=args.acceleration,
            )
            print("Smoke test completed and returned to the starting position.")
        finally:
            arm.relax()
    return 0


def _cmd_kinematics_check(args: argparse.Namespace) -> int:
    measured = np.array([args.x_mm, args.y_mm, args.z_mm], dtype=float) / 1000.0
    with SOARM101(_hardware_config(args)) as arm:
        joints = dict(arm.get_joint_positions().positions)
        predicted_pose = arm.get_position()
    error = measured - predicted_pose.position
    magnitude = float(np.linalg.norm(error))
    record = {
        "sample": args.sample,
        "timestamp": time.time(),
        "joint_positions_rad": joints,
        "predicted_tcp_m": predicted_pose.position.tolist(),
        "measured_tcp_m": measured.tolist(),
        "error_m": error.tolist(),
        "error_norm_m": magnitude,
    }
    print(json.dumps(record, indent=2))
    if args.output:
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record) + "\n")
        print(f"Appended sample to {output}")
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


def _cmd_gui(args: argparse.Namespace) -> int:
    try:
        from soarm101_motion.gui.app import run_gui
    except ImportError as exc:
        raise RuntimeError("GUI support requires: pip install -e '.[gui]'") from exc
    argv: list[str] = ["--robot-id", args.robot_id]
    if args.port:
        argv.extend(("--port", args.port))
    if args.simulation:
        argv.append("--simulation")
    return run_gui(argv)


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

    setup = sub.add_parser("setup-motors", help="assign IDs and baud rate one isolated motor at a time")
    setup.add_argument("--port", required=True)
    setup.add_argument("--motor", choices=ALL_MOTORS)
    setup.add_argument("--initial-id", type=int)
    setup.add_argument("--initial-baudrate", type=int)
    setup.add_argument("--target-baudrate", type=int, default=1_000_000)
    setup.add_argument("--skip-model-check", action="store_true")
    setup.add_argument("--exhaustive-scan", action="store_true")
    setup.add_argument("--yes", action="store_true")
    setup.set_defaults(func=_cmd_setup_motors)

    configure = sub.add_parser("configure", help="write recommended motor settings explicitly")
    add_hardware_options(configure)
    configure.add_argument("--yes", action="store_true")
    configure.set_defaults(func=_cmd_configure)

    read = sub.add_parser("read", help="read calibrated joints and TCP pose without configuration writes")
    add_hardware_options(read)
    read.set_defaults(func=_cmd_read)

    diagnose = sub.add_parser(
        "diagnose", help="read motor model, voltage, temperature, and status without configuration writes"
    )
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

    move = sub.add_parser("move-joints", help="perform one guarded five-joint absolute move")
    add_hardware_options(move)
    move.add_argument("joints", nargs=5, type=float)
    move.add_argument("--degrees", action="store_true")
    move.add_argument("--speed", type=float)
    move.add_argument("--acceleration", type=float)
    move.add_argument("--yes", action="store_true")
    move.set_defaults(func=_cmd_move_joints)

    jog = sub.add_parser("jog", help="perform one guarded world- or tool-frame Cartesian linear jog")
    add_hardware_options(jog)
    jog.add_argument("--frame", choices=("world", "tool"), default="world")
    jog.add_argument("--x-mm", type=float, default=0.0)
    jog.add_argument("--y-mm", type=float, default=0.0)
    jog.add_argument("--z-mm", type=float, default=0.0)
    jog.add_argument("--roll-deg", type=float, default=0.0)
    jog.add_argument("--pitch-deg", type=float, default=0.0)
    jog.add_argument("--yaw-deg", type=float, default=0.0)
    jog.add_argument(
        "--orientation-mode",
        choices=("compatible", "position_only", "exact"),
        default="compatible",
    )
    jog.add_argument("--speed-mm-s", type=float, default=10.0)
    jog.add_argument("--acceleration-mm-s2", type=float, default=40.0)
    jog.add_argument("--yes", action="store_true")
    jog.set_defaults(func=_cmd_jog)

    gripper = sub.add_parser("gripper", help="open, close, or position the stock gripper")
    add_hardware_options(gripper)
    gripper.add_argument("target", help="open, close, or normalized position 0..1")
    gripper.add_argument("--yes", action="store_true")
    gripper.set_defaults(func=_cmd_gripper)

    smoke = sub.add_parser("smoke-test", help="perform a tiny supervised relative one-joint test")
    add_hardware_options(smoke)
    smoke.add_argument("--joint", choices=ARM_JOINTS, required=True)
    smoke.add_argument("--degrees", type=float, default=2.0)
    smoke.add_argument("--speed", type=float, default=0.05)
    smoke.add_argument("--acceleration", type=float, default=0.20)
    smoke.add_argument("--latch-seconds", type=float, default=1.0)
    smoke.add_argument("--yes", action="store_true")
    smoke.set_defaults(func=_cmd_smoke_test)

    check = sub.add_parser(
        "kinematics-check", help="compare predicted TCP against one manually measured TCP sample"
    )
    add_hardware_options(check)
    check.add_argument("--sample", default="sample")
    check.add_argument("--x-mm", type=float, required=True)
    check.add_argument("--y-mm", type=float, required=True)
    check.add_argument("--z-mm", type=float, required=True)
    check.add_argument("--output", help="optional JSON Lines output file")
    check.set_defaults(func=_cmd_kinematics_check)

    simulation = sub.add_parser("sim-demo", help="run joint, gripper, IK, and linear motion in simulation")
    simulation.add_argument("--gui", action="store_true", help="use optional PyBullet GUI")
    simulation.add_argument("--realtime", action="store_true")
    simulation.set_defaults(func=_cmd_sim_demo)

    gui = sub.add_parser("gui", help="open the optional PySide6 robot controller")
    gui.add_argument("--port")
    gui.add_argument("--robot-id", default="so101")
    gui.add_argument("--simulation", action="store_true")
    gui.set_defaults(func=_cmd_gui)
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
