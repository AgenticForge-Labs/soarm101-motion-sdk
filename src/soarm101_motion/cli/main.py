"""Command-line interface for setup, motion, diagnostics, GUI, and simulation."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, replace
from math import pi
from pathlib import Path

import numpy as np

from soarm101_motion import SOARM101, SOARM101Config, __version__
from soarm101_motion.calibration import SO101Calibration, default_calibration_path
from soarm101_motion.camera import (
    CameraCapture,
    CameraSettings,
    discover_camera_devices,
)
from soarm101_motion.constants import (
    ALL_MOTORS,
    ARM_JOINTS,
    JOINT_LIMITS,
    MOTOR_IDS,
    STOCK_GRIPPER,
)
from soarm101_motion.control import jog_linear_cli_units
from soarm101_motion.discovery import discover_so101_arms
from soarm101_motion.hardware import FeetechBackend, FeetechMotorSetup
from soarm101_motion.poses import PoseLibrary, SavedPose, sleep_joint_positions
from soarm101_motion.primitives import MotionPrimitiveLibrary
from soarm101_motion.safety import resolve_effective_joint_limits
from soarm101_motion.sequences import SequenceLibrary, SequenceRunner
from soarm101_motion.tools import SO101Gripper
from soarm101_motion.trajectories import TrajectoryLibrary
from soarm101_motion.types import MotionResult, Pose
from soarm101_motion.workstation import (
    ArmConnectionProfile,
    WorkstationProfile,
    WorkstationProfileStore,
)


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


def _arm_from_args(
    args: argparse.Namespace,
    **config_overrides: object,
) -> SOARM101:
    if getattr(args, "simulation", False):
        return SOARM101.simulated(realtime=True)
    if args.port:
        return SOARM101(_hardware_config(args))

    follower = WorkstationProfileStore().load().follower
    if not follower.port:
        raise ValueError(
            "--port is required because no follower port is saved in the workstation profile"
        )
    requested_robot_id = getattr(args, "robot_id", None)
    robot_id = (
        follower.robot_id
        if requested_robot_id in (None, "", "so101")
        else str(requested_robot_id)
    )
    overrides: dict[str, object] = {
        "port": follower.port,
        "robot_id": robot_id,
    }
    if (
        not getattr(args, "calibration", None)
        and robot_id == follower.robot_id
        and follower.calibration
    ):
        overrides["calibration_path"] = Path(follower.calibration)
    overrides.update(config_overrides)
    return SOARM101(_hardware_config(args, **overrides))


def _confirm(args: argparse.Namespace, word: str, message: str) -> bool:
    if getattr(args, "yes", False):
        return True
    answer = input(f"{message}\nType {word} to continue: ").strip()
    return answer == word


def _print_motion_result(result: MotionResult, *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(asdict(result), indent=2))
    else:
        print(result)


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


def _cmd_discover(args: argparse.Namespace) -> int:
    results = discover_so101_arms(args.ports or None)
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        for item in results:
            if item["status"] == "ok":
                print(
                    f"{item['port']}: {item['role']} ({item['voltage_v']:.1f} V, "
                    f"{item['motor_count']}/{item['motor_total']} motors)"
                )
            else:
                print(f"{item['port']}: unavailable ({item['error']})")
    return 0 if results and all(item["status"] == "ok" for item in results) else 1


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
    with _arm_from_args(args) as arm:
        positions = dict(arm.get_joint_positions().positions)
        pose = arm.get_position().xyz_rpy()
    if args.json:
        payload = {
            "joint_positions_rad": positions,
            "tcp_xyz_mm": [value * 1000.0 for value in pose[:3]],
            "tcp_rpy_deg": [value * 180.0 / pi for value in pose[3:]],
        }
        print(json.dumps(payload, indent=2))
    else:
        print("Joint positions (radians):")
        for name in ARM_JOINTS:
            print(f"  {name:15s} {positions[name]: .6f}")
        print("TCP pose (m, rad):", " ".join(f"{value:.6f}" for value in pose))
    return 0



def _cmd_limits(args: argparse.Namespace) -> int:
    """Report model, calibration, and effective joint/workspace limits without hardware."""

    robot_id = str(args.robot_id)
    if args.calibration:
        calibration_path = Path(args.calibration).expanduser()
    else:
        profile = WorkstationProfileStore().load()
        if profile.follower.robot_id == robot_id and profile.follower.calibration:
            calibration_path = Path(profile.follower.calibration).expanduser()
        else:
            calibration_path = default_calibration_path(robot_id)

    calibration = SO101Calibration.load(calibration_path)
    config = SOARM101Config(robot_id=robot_id)
    calibrated_limits = {
        name: calibration.motors[name].radians_limits
        for name in ARM_JOINTS
    }
    effective_limits = resolve_effective_joint_limits(
        calibrated_limits,
        calibrated_joint_stop_margin_rad=config.calibrated_joint_stop_margin_rad,
    )
    gripper_motor = calibration.motors[STOCK_GRIPPER]
    sleep_gripper_position = SO101Gripper.closed_position_from_calibration(
        gripper_motor,
        stop_margin_rad=config.calibrated_gripper_stop_margin_rad,
    )
    sleep_gripper_raw = gripper_motor.normalized_to_raw(sleep_gripper_position)
    joints: dict[str, object] = {}
    for name in ARM_JOINTS:
        calibrated_lower, calibrated_upper = calibrated_limits[name]
        model_lower, model_upper = JOINT_LIMITS[name]
        effective_lower, effective_upper = effective_limits[name]
        joints[name] = {
            "model_rad": [float(model_lower), float(model_upper)],
            "model_deg": [
                float(model_lower * 180.0 / pi),
                float(model_upper * 180.0 / pi),
            ],
            "calibrated_rad": [float(calibrated_lower), float(calibrated_upper)],
            "calibrated_deg": [
                float(calibrated_lower * 180.0 / pi),
                float(calibrated_upper * 180.0 / pi),
            ],
            "effective_rad": [float(effective_lower), float(effective_upper)],
            "effective_deg": [
                float(effective_lower * 180.0 / pi),
                float(effective_upper * 180.0 / pi),
            ],
        }

    payload = {
        "robot_id": robot_id,
        "calibration_path": str(calibration_path),
        "calibration_id": calibration.calibration_id,
        "calibrated_joint_stop_margin_deg": float(
            config.calibrated_joint_stop_margin_rad * 180.0 / pi
        ),
        "calibrated_gripper_stop_margin_deg": float(
            config.calibrated_gripper_stop_margin_rad * 180.0 / pi
        ),
        "joints": joints,
        "sleep_pose_rad": sleep_joint_positions(effective_limits),
        "sleep_pose_deg": {
            name: float(value * 180.0 / pi)
            for name, value in sleep_joint_positions(effective_limits).items()
        },
        "sleep_gripper": {
            "normalized": float(sleep_gripper_position),
            "raw": int(sleep_gripper_raw),
            "drive_mode": int(gripper_motor.drive_mode),
            "calibrated_raw": [
                int(gripper_motor.range_min),
                int(gripper_motor.range_max),
            ],
        },
        "coarse_cartesian_envelope_mm": {
            "minimum_model_z": float(config.minimum_workspace_z_m * 1000.0),
            "maximum_tcp_reach": float(config.maximum_tcp_reach_m * 1000.0),
            "minimum_self_clearance": float(config.minimum_self_clearance_m * 1000.0),
            "base_keepout_radius": float(config.base_keepout_radius_m * 1000.0),
            "base_keepout_height": float(config.base_keepout_height_m * 1000.0),
        },
        "notes": [
            "URDF/model joint limits are the generic fallback/reference; calibrated real arms use measured pose-joint travel with the configured stop margin",
            "calibration remains the physical authority if a measured range is narrower than the model range",
            "Sleep closes the stock gripper to the calibrated closed stop inset by the configured gripper margin",
            "maximum_tcp_reach is a coarse radial envelope, not a guarantee that every XYZ point is reachable",
            "normal Cartesian CLI coordinates are in the soarm101/base model frame",
        ],
    }
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        print(f"robot_id: {robot_id}")
        print(f"calibration: {calibration_path}")
        print(
            "calibrated joint stop margin: "
            f"{payload['calibrated_joint_stop_margin_deg']:.1f} deg"
        )
        print(
            "calibrated gripper stop margin: "
            f"{payload['calibrated_gripper_stop_margin_deg']:.1f} deg; "
            f"Sleep normalized={payload['sleep_gripper']['normalized']:.4f}, "
            f"raw={payload['sleep_gripper']['raw']}"
        )
        print("joint                         calibrated                   model               effective")
        for name in ARM_JOINTS:
            item = joints[name]
            assert isinstance(item, dict)
            calibrated = item["calibrated_deg"]
            model = item["model_deg"]
            effective = item["effective_deg"]
            print(
                f"{name:16s} "
                f"{calibrated[0]:7.1f}..{calibrated[1]:7.1f}  "
                f"{model[0]:7.1f}..{model[1]:7.1f}  "
                f"{effective[0]:7.1f}..{effective[1]:7.1f}"
            )
        print(
            "coarse TCP reach: "
            f"{payload['coarse_cartesian_envelope_mm']['maximum_tcp_reach']:.1f} mm"
        )
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


def _print_calibration(calibration: object) -> None:
    motors = getattr(calibration, "motors")
    print("Calibration result:")
    for name, motor in motors.items():
        travel = motor.range_max - motor.range_min
        print(
            f"  {name:16s} limits={motor.range_min:4d}..{motor.range_max:4d} "
            f"travel={travel:4d} offset={motor.homing_offset:+5d}"
        )


def _cmd_calibrate(args: argparse.Namespace) -> int:
    print("CALIBRATION SAFETY")
    print("- Remove payloads and clear the workspace.")
    print("- Keep power accessible. Torque will be disabled.")
    print("- Gently reach the printed stops; never force a joint against a stop.")
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
        print("\nLIVE MECHANICAL-STOP CALIBRATION")
        print("During the live sweep:")
        print("- Move every joint and the gripper fully from stop to stop and back.")
        print("- Two end-to-end traversals are required for every motor.")
        print("- The SDK calculates zero halfway between the observed extrema.")
        print("- Crossing the encoder 4095/0 seam is handled automatically.")
        input("Press ENTER when you are ready to begin the live sweep. ")
        print(f"Recording extrema for up to {args.seconds:.1f} seconds; finishing when all six complete two traversals...")
        calibration = backend.interactive_calibration(record_seconds=args.seconds)
        _print_calibration(calibration)
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
        print("Refusing to move without --yes.", file=sys.stderr)
        return 2
    values = [value * pi / 180.0 if args.degrees else value for value in args.joints]
    with _arm_from_args(args) as arm:
        arm.enable()
        result = arm.move_joints(values, speed=args.speed, acceleration=args.acceleration)
        _print_motion_result(result, as_json=args.json)
    return 0


def _cmd_sleep(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move without --yes.", file=sys.stderr)
        return 2
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        arm.enable()
        result = arm.move_sleep(
            speed=args.speed_deg_s * pi / 180.0,
            acceleration=args.acceleration_deg_s2 * pi / 180.0,
        )
        _print_motion_result(result, as_json=args.json)
        print(
            "Sleep complete and holding. Press ENTER to relax the arm.",
            file=sys.stderr,
        )
        input()
        arm.relax()
    return 0


def _cmd_jog(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move without --yes.", file=sys.stderr)
        return 2
    with _arm_from_args(args) as arm:
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
        _print_motion_result(result, as_json=args.json)
    return 0


def _cmd_gripper(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move without --yes.", file=sys.stderr)
        return 2
    if args.target == "open":
        position = 1.0
    elif args.target == "close":
        position = 0.0
    else:
        position = float(args.target)
    if not 0.0 <= position <= 1.0:
        raise ValueError("gripper target must be 'open', 'close', or a value in [0, 1]")
    with _arm_from_args(args) as arm:
        arm.enable()
        result = arm.tool.move(position)
        _print_motion_result(result, as_json=args.json)
    return 0


def _cmd_ik(args: argparse.Namespace) -> int:
    target = Pose.from_xyz_rpy(
        args.x_mm / 1000.0,
        args.y_mm / 1000.0,
        args.z_mm / 1000.0,
        *(value * pi / 180.0 for value in (args.roll_deg, args.pitch_deg, args.yaw_deg)),
    )
    with _arm_from_args(args) as arm:
        result = arm.solve_ik(target, orientation_mode=args.orientation_mode)
    payload = {
        "success": result.success,
        "joints_rad": dict(result.joints),
        "joints_deg": {name: value * 180.0 / pi for name, value in result.joints.items()},
        "position_error_mm": result.position_error_m * 1000.0,
        "orientation_error_deg": result.orientation_error_rad * 180.0 / pi,
        "iterations": result.iterations,
        "message": result.message,
    }
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        print(f"success={result.success} position_error_mm={payload['position_error_mm']:.3f}")
        print(f"orientation_error_deg={payload['orientation_error_deg']:.3f}")
        print("Joint solution (degrees):")
        for name in ARM_JOINTS:
            print(f"  {name:15s} {payload['joints_deg'][name]: .3f}")
        print(result.message)
    return 0 if result.success else 2


def _cmd_move_linear(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move without --yes.", file=sys.stderr)
        return 2
    target = Pose.from_xyz_rpy(
        args.x_mm / 1000.0,
        args.y_mm / 1000.0,
        args.z_mm / 1000.0,
        *(value * pi / 180.0 for value in (args.roll_deg, args.pitch_deg, args.yaw_deg)),
    )
    with _arm_from_args(args) as arm:
        arm.enable()
        result = arm.move_linear(
            target,
            orientation_mode=args.orientation_mode,
            speed=args.speed_mm_s / 1000.0,
            acceleration=args.acceleration_mm_s2 / 1000.0,
        )
        _print_motion_result(result, as_json=args.json)
    return 0


def _cmd_pose_list(args: argparse.Namespace) -> int:
    library = PoseLibrary(args.robot_id)
    for name in library.names():
        pose = library.require(name)
        print(f"{name}\t{pose.source}\t{pose.created_at}")
    return 0


def _cmd_pose_capture(args: argparse.Namespace) -> int:
    with _arm_from_args(args) as arm:
        pose = SavedPose.capture(arm, source=args.source)
    path = PoseLibrary(args.robot_id).save(args.name, pose)
    print(f"Saved {args.name} to {path}")
    return 0


def _cmd_pose_go(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move hardware without --yes.", file=sys.stderr)
        return 2
    pose = PoseLibrary(args.robot_id).require(args.name)
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        arm.require_artifact_calibration(
            {
                "source_robot_id": pose.source_robot_id,
                "source_calibration_id": pose.source_calibration_id,
                "target_robot_id": pose.target_robot_id,
                "target_calibration_id": pose.target_calibration_id,
            },
            artifact_label=f"saved pose {args.name!r}",
        )
        arm.enable()
        if args.mode == "joint":
            result = arm.move_joints_from_saved_pose(
                pose.joints,
                speed=args.speed_deg_s * pi / 180.0,
                acceleration=args.acceleration_deg_s2 * pi / 180.0,
            )
        else:
            result = arm.move_linear(
                Pose.from_xyz_rpy(*pose.tcp_xyz_rpy),
                orientation_mode=args.orientation_mode,
                speed=args.speed_mm_s / 1000.0,
                acceleration=args.acceleration_mm_s2 / 1000.0,
            )
        print(result)
        print(arm.tool.move(pose.gripper))
        arm.hold()
        print("Pose reached; follower remains torque-held.", file=sys.stderr)
    return 0

def _cmd_relax(args: argparse.Namespace) -> int:
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        print(
            "Relax will disable follower torque. Press ENTER to confirm relax.",
            file=sys.stderr,
        )
        input()
        arm.relax()
    print("Follower relaxed.")
    return 0


def _cmd_trajectory_list(args: argparse.Namespace) -> int:
    for entry in TrajectoryLibrary(args.robot_id).entries():
        print(f"{entry.kind}\t{entry.name}")
    return 0


def _cmd_trajectory_play(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move hardware without --yes.", file=sys.stderr)
        return 2
    trajectory = TrajectoryLibrary(args.robot_id).load(args.name, kind=args.kind)
    with _arm_from_args(args) as arm:
        arm.enable()
        print(arm.play_trajectory(trajectory, speed_scale=args.speed_scale, move_to_start=True))
    return 0


def _cmd_sequence_list(args: argparse.Namespace) -> int:
    library = SequenceLibrary(args.robot_id)
    for name in library.names():
        sequence = library.require(name)
        print(f"{name}\t{len(sequence.steps)} steps")
    return 0


def _cmd_sequence_run(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move hardware without --yes.", file=sys.stderr)
        return 2
    sequence = SequenceLibrary(args.robot_id).require(args.name)
    with _arm_from_args(args) as arm:
        arm.enable()
        runner = SequenceRunner(
            arm,
            pose_library=PoseLibrary(args.robot_id),
            trajectory_library=TrajectoryLibrary(args.robot_id),
            primitive_library=MotionPrimitiveLibrary(args.robot_id),
        )
        print(
            runner.run(
                sequence,
                repeat=args.repeat,
                speed_scale=args.speed_scale,
                start_index=args.start_index,
                stop_index=args.stop_index,
            )
        )
    return 0


def _cmd_effort_status(args: argparse.Namespace) -> int:
    with _arm_from_args(args) as arm:
        print(json.dumps(arm.get_effort_safety_status(refresh=args.refresh), indent=2))
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



def _camera_profile_name(args: argparse.Namespace, profile: WorkstationProfile) -> str:
    requested = str(getattr(args, "name", "") or "").strip()
    if requested:
        return requested
    if profile.selected_camera:
        return profile.selected_camera
    if profile.cameras:
        return next(iter(profile.cameras))
    return "camera"


def _camera_settings_from_args(
    args: argparse.Namespace,
    *,
    allow_new: bool = False,
) -> tuple[WorkstationProfileStore, WorkstationProfile, str, CameraSettings]:
    store = WorkstationProfileStore()
    profile = store.load()
    name = _camera_profile_name(args, profile)
    if name in profile.cameras:
        settings = profile.cameras[name]
    elif allow_new:
        settings = CameraSettings().validated()
    else:
        available = ", ".join(profile.cameras) or "none"
        raise KeyError(f"unknown camera profile {name!r}; configured profiles: {available}")
    overrides: dict[str, object] = {}
    for key in ("device", "width", "height", "fps", "fourcc", "snapshot_dir"):
        value = getattr(args, key, None)
        if value is not None:
            overrides[key] = value
    mirror = getattr(args, "mirror", None)
    if mirror is not None:
        overrides["mirror"] = bool(mirror)
    auto_start = getattr(args, "auto_start", None)
    if auto_start is not None:
        overrides["auto_start"] = bool(auto_start)
    if overrides:
        settings = settings.with_overrides(**overrides)
    return store, profile, name, settings


def _camera_profile_payload(profile: WorkstationProfile) -> dict[str, object]:
    return {
        "selected_camera": profile.selected_camera,
        "cameras": {
            name: asdict(settings)
            for name, settings in profile.cameras.items()
        },
    }


def _cmd_camera_list(args: argparse.Namespace) -> int:
    devices = discover_camera_devices()
    profile = WorkstationProfileStore().load()
    payload = {
        "devices": devices,
        **_camera_profile_payload(profile),
    }
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        if devices:
            print("Discovered devices:")
            for device in devices:
                print(f"  {device}")
        else:
            print("No camera devices found.")
        print("Configured camera profiles:")
        for name, settings in profile.cameras.items():
            selected = " *" if name == profile.selected_camera else ""
            print(f"  {name}{selected}: {settings.device}")
    return 0 if devices or profile.cameras else 1


def _cmd_camera_show(args: argparse.Namespace) -> int:
    profile = WorkstationProfileStore().load()
    requested = str(getattr(args, "name", "") or "").strip()
    if requested:
        payload: object = {
            "name": requested,
            **asdict(profile.camera(requested)),
        }
    else:
        payload = _camera_profile_payload(profile)
    if args.json:
        print(json.dumps(payload, indent=2))
    elif isinstance(payload, dict):
        for key, value in payload.items():
            print(f"{key}: {value}")
    return 0


def _cmd_camera_configure(args: argparse.Namespace) -> int:
    store, profile, name, settings = _camera_settings_from_args(args, allow_new=True)
    old_name = str(getattr(args, "rename_from", "") or "").strip()
    if old_name and old_name != name:
        profile = profile.renamed_camera(old_name, name)
    profile = profile.with_camera(name, settings, select=True)
    profile = store.save(profile)
    payload = {"name": name, **asdict(profile.camera(name))}
    print(
        json.dumps(payload, indent=2)
        if args.json
        else f"Saved camera profile {name!r} for {settings.device}"
    )
    return 0


def _cmd_camera_select(args: argparse.Namespace) -> int:
    store = WorkstationProfileStore()
    profile = store.load()
    profile.camera(args.name)
    profile = replace(profile, selected_camera=args.name).validated()
    store.save(profile)
    print(args.name)
    return 0


def _cmd_camera_remove(args: argparse.Namespace) -> int:
    store = WorkstationProfileStore()
    profile = store.load().without_camera(args.name)
    store.save(profile)
    print(args.name)
    return 0


def _capture_named_camera(
    name: str,
    settings: CameraSettings,
    output: str | None = None,
) -> dict[str, object]:
    with CameraCapture(settings) as camera:
        path, metadata = camera.snapshot(output)
    return {"name": name, **metadata}


def _cmd_camera_capture(args: argparse.Namespace) -> int:
    store = WorkstationProfileStore()
    profile = store.load()
    if args.all:
        if args.output:
            raise ValueError("--output cannot be combined with --all")
        if not profile.cameras:
            raise ValueError("no camera profiles are configured")
        captures = [
            _capture_named_camera(name, settings)
            for name, settings in profile.cameras.items()
        ]
        if args.json:
            print(json.dumps({"captures": captures}, indent=2))
        else:
            for item in captures:
                print(f"{item['name']}: {item['path']}")
        return 0

    _store, _profile, name, settings = _camera_settings_from_args(
        args,
        allow_new=bool(getattr(args, "device", None)),
    )
    metadata = _capture_named_camera(name, settings, args.output)
    if args.json:
        print(json.dumps(metadata, indent=2))
    else:
        print(metadata["path"])
    return 0


def _workstation_payload(profile: WorkstationProfile) -> dict[str, object]:
    return {
        "schema_version": profile.schema_version,
        "follower": asdict(profile.follower),
        "leader": asdict(profile.leader),
        **_camera_profile_payload(profile),
    }


def _cmd_workstation_show(args: argparse.Namespace) -> int:
    profile = WorkstationProfileStore().load()
    payload = _workstation_payload(profile)
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        print(f"Follower: {profile.follower.port or 'unset'} · {profile.follower.robot_id}")
        print(f"Leader:   {profile.leader.port or 'unset'} · {profile.leader.robot_id}")
        print(f"Selected camera: {profile.selected_camera or 'none'}")
        for name, settings in profile.cameras.items():
            print(f"  {name}: {settings.device}")
    return 0


def _cmd_workstation_arm(args: argparse.Namespace) -> int:
    store = WorkstationProfileStore()
    profile = store.load()
    current = profile.follower if args.role == "follower" else profile.leader
    robot_id = args.robot_id or current.robot_id
    arm = ArmConnectionProfile(
        port=current.port if args.port is None else args.port,
        robot_id=robot_id,
        calibration=(
            current.calibration
            if args.calibration is None
            else args.calibration
        ),
    ).validated()
    if args.role == "follower":
        profile = replace(profile, follower=arm).validated()
    else:
        profile = replace(profile, leader=arm).validated()
    store.save(profile)
    payload = {"role": args.role, **asdict(arm)}
    print(json.dumps(payload, indent=2) if args.json else f"Saved {args.role} profile")
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
        raise RuntimeError("GUI support requires: pip install -e '.[gui]'" ) from exc
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
    discover = sub.add_parser("discover", help="identify connected SO-101 arms without enabling torque")
    discover.add_argument("ports", nargs="*")
    discover.add_argument("--json", action="store_true")
    discover.set_defaults(func=_cmd_discover)

    def add_hardware_options(command: argparse.ArgumentParser) -> None:
        command.add_argument("--port", required=True, help="serial port, for example /dev/ttyACM0")
        command.add_argument("--robot-id", default="so101")
        command.add_argument("--calibration")

    def add_session_options(command: argparse.ArgumentParser) -> None:
        command.add_argument(
            "--port",
            help="physical serial port; defaults to saved workstation follower without --simulation",
        )
        command.add_argument("--robot-id", default="so101")
        command.add_argument("--calibration")
        command.add_argument("--simulation", action="store_true")

    def add_linear_options(command: argparse.ArgumentParser) -> None:
        command.add_argument("--orientation-mode", choices=("compatible", "position_only", "exact"), default="compatible")
        command.add_argument("--speed-mm-s", type=float, default=10.0)
        command.add_argument("--acceleration-mm-s2", type=float, default=40.0)

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
    add_session_options(read)
    read.add_argument("--json", action="store_true")
    read.set_defaults(func=_cmd_read)

    limits = sub.add_parser(
        "limits",
        help="show saved calibrated, model, and effective joint/workspace limits",
    )
    limits.add_argument("--robot-id", default="so101")
    limits.add_argument("--calibration")
    limits.add_argument("--json", action="store_true")
    limits.set_defaults(func=_cmd_limits)

    diagnose = sub.add_parser(
        "diagnose", help="read motor model, voltage, temperature, and status without configuration writes"
    )
    add_hardware_options(diagnose)
    diagnose.add_argument("--json", action="store_true")
    diagnose.add_argument("--allow-uncalibrated", action="store_true")
    diagnose.set_defaults(func=_cmd_diagnose)

    calibrate = sub.add_parser(
        "calibrate",
        help="calibrate all six motors with two full stop-to-stop traversals each",
    )
    add_hardware_options(calibrate)
    calibrate.add_argument("--seconds", type=float, default=90.0)
    calibrate.add_argument("--output")
    calibrate.add_argument("--export-lerobot", action="store_true")
    calibrate.add_argument("--yes", action="store_true")
    calibrate.set_defaults(func=_cmd_calibrate)

    move = sub.add_parser("move-joints", help="perform one guarded five-joint absolute move")
    add_session_options(move)
    move.add_argument("joints", nargs=5, type=float)
    move.add_argument("--degrees", action="store_true")
    move.add_argument("--speed", type=float)
    move.add_argument("--acceleration", type=float)
    move.add_argument("--yes", action="store_true")
    move.add_argument("--json", action="store_true")
    move.set_defaults(func=_cmd_move_joints)

    sleep = sub.add_parser(
        "sleep",
        help="move to the calibration-derived natural Sleep posture through normal safety guards",
    )
    add_session_options(sleep)
    sleep.add_argument("--speed-deg-s", type=float, default=8.0)
    sleep.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    sleep.add_argument("--yes", action="store_true")
    sleep.add_argument("--json", action="store_true")
    sleep.set_defaults(func=_cmd_sleep)

    jog = sub.add_parser("jog", help="perform one guarded world- or tool-frame Cartesian linear jog")
    add_session_options(jog)
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
    jog.add_argument("--json", action="store_true")
    jog.set_defaults(func=_cmd_jog)

    gripper = sub.add_parser("gripper", help="open, close, or position the stock gripper")
    add_session_options(gripper)
    gripper.add_argument("target", help="open, close, or normalized position 0..1")
    gripper.add_argument("--yes", action="store_true")
    gripper.add_argument("--json", action="store_true")
    gripper.set_defaults(func=_cmd_gripper)

    ik = sub.add_parser("ik", help="solve a world TCP target without commanding motion")
    add_session_options(ik)
    for axis in ("x", "y", "z"):
        ik.add_argument(f"--{axis}-mm", type=float, required=True)
    for axis in ("roll", "pitch", "yaw"):
        ik.add_argument(f"--{axis}-deg", type=float, default=0.0)
    ik.add_argument(
        "--orientation-mode",
        choices=("compatible", "position_only", "exact"),
        default="compatible",
    )
    ik.add_argument("--json", action="store_true")
    ik.set_defaults(func=_cmd_ik)

    linear = sub.add_parser("move-linear", help="move to an absolute world TCP pose")
    add_session_options(linear)
    for axis in ("x", "y", "z"):
        linear.add_argument(f"--{axis}-mm", type=float, required=True)
    for axis in ("roll", "pitch", "yaw"):
        linear.add_argument(f"--{axis}-deg", type=float, default=0.0)
    add_linear_options(linear)
    linear.add_argument("--yes", action="store_true")
    linear.add_argument("--json", action="store_true")
    linear.set_defaults(func=_cmd_move_linear)

    relax = sub.add_parser(
        "relax",
        help="disable follower torque after explicit ENTER confirmation",
    )
    add_session_options(relax)
    relax.set_defaults(func=_cmd_relax)

    pose = sub.add_parser("pose", help="list, capture, and replay GUI named poses")
    pose_sub = pose.add_subparsers(dest="pose_command", required=True)
    pose_list = pose_sub.add_parser("list")
    pose_list.add_argument("--robot-id", default="so101")
    pose_list.set_defaults(func=_cmd_pose_list)
    pose_capture = pose_sub.add_parser("capture")
    add_session_options(pose_capture)
    pose_capture.add_argument("name")
    pose_capture.add_argument("--source", choices=("follower", "leader"), default="follower")
    pose_capture.set_defaults(func=_cmd_pose_capture)
    pose_go = pose_sub.add_parser("go")
    add_session_options(pose_go)
    pose_go.add_argument("name")
    pose_go.add_argument("--mode", choices=("joint", "linear"), default="joint")
    pose_go.add_argument("--speed-deg-s", type=float, default=8.0)
    pose_go.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    add_linear_options(pose_go)
    pose_go.add_argument("--yes", action="store_true")
    pose_go.set_defaults(func=_cmd_pose_go)

    trajectory = sub.add_parser("trajectory", help="list and replay GUI trajectories")
    trajectory_sub = trajectory.add_subparsers(dest="trajectory_command", required=True)
    trajectory_list = trajectory_sub.add_parser("list")
    trajectory_list.add_argument("--robot-id", default="so101")
    trajectory_list.set_defaults(func=_cmd_trajectory_list)
    trajectory_play = trajectory_sub.add_parser("play")
    add_session_options(trajectory_play)
    trajectory_play.add_argument("name")
    trajectory_play.add_argument("--kind", choices=("raw", "edited"), default="edited")
    trajectory_play.add_argument("--speed-scale", type=float, default=1.0)
    trajectory_play.add_argument("--yes", action="store_true")
    trajectory_play.set_defaults(func=_cmd_trajectory_play)

    sequence = sub.add_parser("sequence", help="list and run GUI sequences")
    sequence_sub = sequence.add_subparsers(dest="sequence_command", required=True)
    sequence_list = sequence_sub.add_parser("list")
    sequence_list.add_argument("--robot-id", default="so101")
    sequence_list.set_defaults(func=_cmd_sequence_list)
    sequence_run = sequence_sub.add_parser("run")
    add_session_options(sequence_run)
    sequence_run.add_argument("name")
    sequence_run.add_argument("--repeat", type=int, default=1)
    sequence_run.add_argument("--speed-scale", type=float, default=1.0)
    sequence_run.add_argument("--start-index", type=int, default=0)
    sequence_run.add_argument("--stop-index", type=int)
    sequence_run.add_argument("--yes", action="store_true")
    sequence_run.set_defaults(func=_cmd_sequence_run)

    camera = sub.add_parser("camera", help="discover, configure, and capture named USB cameras")
    camera_sub = camera.add_subparsers(dest="camera_command", required=True)

    camera_list = camera_sub.add_parser(
        "list",
        help="list discovered devices and configured camera profiles",
    )
    camera_list.add_argument("--json", action="store_true")
    camera_list.set_defaults(func=_cmd_camera_list)

    camera_show = camera_sub.add_parser("show", help="show named persisted camera settings")
    camera_show.add_argument("--name", help="camera profile name; default shows all profiles")
    camera_show.add_argument("--json", action="store_true")
    camera_show.set_defaults(func=_cmd_camera_show)

    def add_camera_settings_options(command: argparse.ArgumentParser) -> None:
        command.add_argument("--name", help="camera profile name")
        command.add_argument("--device", help="camera index or path, for example /dev/video0")
        command.add_argument("--width", type=int)
        command.add_argument("--height", type=int)
        command.add_argument("--fps", type=float)
        command.add_argument("--fourcc", help="four-character capture codec such as MJPG")
        command.add_argument("--snapshot-dir")
        mirror_group = command.add_mutually_exclusive_group()
        mirror_group.add_argument("--mirror", dest="mirror", action="store_true")
        mirror_group.add_argument("--no-mirror", dest="mirror", action="store_false")
        command.set_defaults(mirror=None)

    camera_configure = camera_sub.add_parser(
        "configure",
        help="create or update a named camera profile",
    )
    add_camera_settings_options(camera_configure)
    camera_configure.add_argument(
        "--rename-from",
        help="rename an existing profile while saving these settings",
    )
    start_group = camera_configure.add_mutually_exclusive_group()
    start_group.add_argument("--auto-start", dest="auto_start", action="store_true")
    start_group.add_argument("--no-auto-start", dest="auto_start", action="store_false")
    camera_configure.set_defaults(auto_start=None)
    camera_configure.add_argument("--json", action="store_true")
    camera_configure.set_defaults(func=_cmd_camera_configure)

    camera_select = camera_sub.add_parser("select", help="select the default camera profile")
    camera_select.add_argument("name")
    camera_select.set_defaults(func=_cmd_camera_select)

    camera_remove = camera_sub.add_parser("remove", help="remove a named camera profile")
    camera_remove.add_argument("name")
    camera_remove.set_defaults(func=_cmd_camera_remove)

    camera_capture = camera_sub.add_parser(
        "capture",
        help="acquire one named camera or all configured cameras",
    )
    add_camera_settings_options(camera_capture)
    camera_capture.add_argument("--all", action="store_true", help="capture every configured camera")
    camera_capture.add_argument("--output", help="output image path; single camera only")
    camera_capture.add_argument("--json", action="store_true")
    camera_capture.set_defaults(func=_cmd_camera_capture)

    workstation = sub.add_parser(
        "workstation",
        help="show or configure persistent local arm/camera addressing",
    )
    workstation_sub = workstation.add_subparsers(dest="workstation_command", required=True)
    workstation_show = workstation_sub.add_parser("show")
    workstation_show.add_argument("--json", action="store_true")
    workstation_show.set_defaults(func=_cmd_workstation_show)
    workstation_arm = workstation_sub.add_parser("arm")
    workstation_arm.add_argument("role", choices=("follower", "leader"))
    workstation_arm.add_argument("--port")
    workstation_arm.add_argument("--robot-id")
    workstation_arm.add_argument("--calibration")
    workstation_arm.add_argument("--json", action="store_true")
    workstation_arm.set_defaults(func=_cmd_workstation_arm)

    effort = sub.add_parser("effort", help="read session motor-effort safety status")
    effort_sub = effort.add_subparsers(dest="effort_command", required=True)
    effort_status = effort_sub.add_parser("status")
    add_session_options(effort_status)
    effort_status.add_argument("--refresh", action="store_true")
    effort_status.set_defaults(func=_cmd_effort_status)

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
