"""Command-line interface for setup, motion, diagnostics, GUI, and simulation."""

from __future__ import annotations

import argparse
import json
import sys
import time
from contextlib import nullcontext
from dataclasses import asdict, replace
from math import pi
from pathlib import Path

import numpy as np

from soarm101_motion import SOARM101, SOARM101Config, __version__
from soarm101_motion.agent_control import (
    AGENT_CAMERA_NAMES,
    AGENT_JOG_HEIGHT_THRESHOLD_M,
    AGENT_JOG_HIGH_MAX_DISTANCE_M,
    AGENT_JOG_LOW_MAX_DISTANCE_M,
    AGENT_JOG_MINIMUM_TARGET_HEIGHT_M,
    AGENT_JOINT_MAX_DELTA_DEG,
    AGENT_POSE_PREFIX,
    DEFAULT_AUTHORITY_MINUTES,
    MAX_AUTHORITY_MINUTES,
    AgentAuthorityStore,
    evaluate_agent_jog,
)
from soarm101_motion.calibration import SO101Calibration, default_calibration_path
from soarm101_motion.camera import (
    CameraSettings,
    discover_camera_devices,
)
from soarm101_motion.constants import (
    ALL_MOTORS,
    ARM_JOINTS,
    JOINT_LIMITS,
    MOTOR_IDS,
    STOCK_GRIPPER,
    TELEOP_SERVO_ACCELERATION_RAW,
    TELEOP_SERVO_SPEED_RAW,
)
from soarm101_motion.control import jog_linear_cli_units, relative_target_pose
from soarm101_motion.discovery import discover_so101_arms
from soarm101_motion.hardware import FeetechBackend, FeetechMotorSetup
from soarm101_motion.motion import PassiveBackendTrace
from soarm101_motion.motion.trace import summarize_agent_jog_trace
from soarm101_motion.poses import (
    PoseLibrary,
    sleep_joint_positions,
    sleep_up_joint_positions,
)
from soarm101_motion.primitives import MotionPrimitiveLibrary
from soarm101_motion.safety import resolve_effective_joint_limits
from soarm101_motion.sdk_capabilities import SDK_CAPABILITIES
from soarm101_motion.sequences import SequenceLibrary, SequenceRunner
from soarm101_motion.tools import SO101Gripper
from soarm101_motion.trajectories import TrajectoryLibrary
from soarm101_motion.types import MotionResult
from soarm101_motion.workspace import WorkspaceCalibrationStore
from soarm101_motion.workstation import (
    ArmConnectionProfile,
    WorkstationProfile,
    WorkstationProfileStore,
)


def _motion_limit_overrides(args: argparse.Namespace) -> dict[str, float]:
    """Resolve optional CLI motion-envelope overrides into internal SI units."""

    values: dict[str, float] = {}
    conversions = {
        "max_joint_speed_deg_s": ("max_joint_speed", pi / 180.0),
        "max_joint_acceleration_deg_s2": ("max_joint_acceleration", pi / 180.0),
        "max_linear_speed_mm_s": ("max_linear_speed", 1.0 / 1000.0),
        "max_linear_acceleration_mm_s2": (
            "max_linear_acceleration",
            1.0 / 1000.0,
        ),
        "max_tool_angular_speed_deg_s": ("max_angular_speed", pi / 180.0),
        "max_tool_angular_acceleration_deg_s2": (
            "max_angular_acceleration",
            pi / 180.0,
        ),
    }
    for cli_name, (config_name, scale) in conversions.items():
        raw = getattr(args, cli_name, None)
        if raw is not None:
            values[config_name] = float(raw) * scale
    return values


def _hardware_config(args: argparse.Namespace, **overrides: object) -> SOARM101Config:
    values: dict[str, object] = {
        "port": args.port,
        "robot_id": getattr(args, "robot_id", "so101"),
        "configure_motors_on_connect": False,
    }
    calibration = getattr(args, "calibration", None)
    if calibration:
        values["calibration_path"] = Path(calibration)
    values.update(_motion_limit_overrides(args))
    values.update(overrides)
    return SOARM101Config(**values)


def _arm_from_args(
    args: argparse.Namespace,
    **config_overrides: object,
) -> SOARM101:
    if getattr(args, "simulation", False):
        return SOARM101.simulated(
            realtime=True,
            config=_hardware_config(args, **config_overrides),
        )
    if args.port:
        return SOARM101(_hardware_config(args, **config_overrides))

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
        payload = SDK_CAPABILITIES.dispatch("read_pose", arm, {})
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        print("Joint positions (radians):")
        for name in ARM_JOINTS:
            print(f"  {name:15s} {payload['joint_positions_rad'][name]: .6f}")
        xyz_m = [value / 1000.0 for value in payload["tcp_xyz_mm"]]
        rpy_rad = [value * pi / 180.0 for value in payload["tcp_rpy_deg"]]
        print("TCP pose (m, rad):", " ".join(f"{value:.6f}" for value in (*xyz_m, *rpy_rad)))
    return 0


def _cmd_sdk_capabilities(args: argparse.Namespace) -> int:
    """Report authoritative typed SDK actions without opening hardware."""
    actions = SDK_CAPABILITIES.describe()
    if args.json:
        print(json.dumps({"schema_version": 1, "actions": actions}, indent=2))
    else:
        for action in actions:
            print(f"{action['name']}: {action['description']} ({action['effect']})")
    return 0

def _cmd_motion_envelope(args: argparse.Namespace) -> int:
    """Report the effective host motion envelope without requiring calibration."""

    config = SOARM101Config(
        robot_id=str(args.robot_id),
        **_motion_limit_overrides(args),
    )
    human = config.motion_limits_human
    payload = {
        "robot_id": config.robot_id,
        "host_envelope": human,
        "host_envelope_si": {
            "max_joint_speed_rad_s": config.max_joint_speed,
            "max_joint_acceleration_rad_s2": config.max_joint_acceleration,
            "max_linear_speed_m_s": config.max_linear_speed,
            "max_linear_acceleration_m_s2": config.max_linear_acceleration,
            "max_tool_angular_speed_rad_s": config.max_angular_speed,
            "max_tool_angular_acceleration_rad_s2": config.max_angular_acceleration,
        },
        "servo_tracking": {
            "goal_velocity_raw": TELEOP_SERVO_SPEED_RAW,
            "acceleration_raw": TELEOP_SERVO_ACCELERATION_RAW,
            "ownership": (
                "responsive inner-loop tracking; host trajectory remains the commanded "
                "speed/acceleration authority"
            ),
        },
    }
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        print(
            "joint: "
            f"{human['max_joint_speed_deg_s']:g} deg/s, "
            f"{human['max_joint_acceleration_deg_s2']:g} deg/s^2"
        )
        print(
            "TCP linear: "
            f"{human['max_linear_speed_mm_s']:g} mm/s, "
            f"{human['max_linear_acceleration_mm_s2']:g} mm/s^2"
        )
        print(
            "TCP angular: "
            f"{human['max_tool_angular_speed_deg_s']:g} deg/s, "
            f"{human['max_tool_angular_acceleration_deg_s2']:g} deg/s^2"
        )
        print(
            "servo tracking: Goal_Velocity raw "
            f"{TELEOP_SERVO_SPEED_RAW}, acceleration raw "
            f"{TELEOP_SERVO_ACCELERATION_RAW}"
        )
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
    config = SOARM101Config(
        robot_id=robot_id,
        **_motion_limit_overrides(args),
    )
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
        "sleep_up_pose_rad": sleep_up_joint_positions(effective_limits),
        "sleep_up_pose_deg": {
            name: float(value * 180.0 / pi)
            for name, value in sleep_up_joint_positions(effective_limits).items()
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
        "motion_envelope": {
            **config.motion_limits_human,
            "servo_tracking": {
                "goal_velocity_raw": TELEOP_SERVO_SPEED_RAW,
                "acceleration_raw": TELEOP_SERVO_ACCELERATION_RAW,
                "note": "responsive inner servo tracking; host trajectory owns speed/acceleration",
            },
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
            "Sleep uses wrist_flex at 75% of its executable calibrated range; sleep_up preserves the historical wrist-at-lower-limit posture",
            "Sleep and sleep_up close the stock gripper to the calibrated closed stop inset by the configured gripper margin",
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
        envelope = config.motion_limits_human
        print(
            "motion envelope: "
            f"joint {envelope['max_joint_speed_deg_s']:g} deg/s / "
            f"{envelope['max_joint_acceleration_deg_s2']:g} deg/s^2; "
            f"TCP {envelope['max_linear_speed_mm_s']:g} mm/s / "
            f"{envelope['max_linear_acceleration_mm_s2']:g} mm/s^2; "
            f"tool angular {envelope['max_tool_angular_speed_deg_s']:g} deg/s / "
            f"{envelope['max_tool_angular_acceleration_deg_s2']:g} deg/s^2"
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
    speed = (
        args.speed_deg_s * pi / 180.0
        if args.speed_deg_s is not None
        else args.speed
    )
    acceleration = (
        args.acceleration_deg_s2 * pi / 180.0
        if args.acceleration_deg_s2 is not None
        else args.acceleration
    )
    request: dict[str, object] = {"positions_rad": values}
    if speed is not None:
        request["speed_rad_s"] = speed
    if acceleration is not None:
        request["acceleration_rad_s2"] = acceleration
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        arm.enable()
        result = SDK_CAPABILITIES.dispatch("move_joints", arm, request)
        arm.hold()
        _print_motion_result(result, as_json=args.json)
        print("Joint move complete; follower remains torque-held.", file=sys.stderr)
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


def _cmd_sleep_up(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move without --yes.", file=sys.stderr)
        return 2
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        arm.enable()
        result = arm.move_sleep_up(
            speed=args.speed_deg_s * pi / 180.0,
            acceleration=args.acceleration_deg_s2 * pi / 180.0,
        )
        _print_motion_result(result, as_json=args.json)
        print(
            "sleep_up complete and holding. Press ENTER to relax the arm.",
            file=sys.stderr,
        )
        input()
        arm.relax()
    return 0


def _cmd_jog_joint(args: argparse.Namespace) -> int:
    """Single-joint relative operator jog using shared SDK dispatch."""
    if not args.yes:
        print("Refusing to move without --yes.", file=sys.stderr)
        return 2
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        arm.enable()
        result = SDK_CAPABILITIES.dispatch(
            "jog_joint", arm, {
                "joint": args.joint,
                "delta_rad": args.delta_deg * pi / 180.0,
                "speed_rad_s": args.speed_deg_s * pi / 180.0,
                "acceleration_rad_s2": args.acceleration_deg_s2 * pi / 180.0,
            },
        )
        arm.hold()
        _print_motion_result(result, as_json=args.json)
        print("Joint jog complete; follower remains torque-held.", file=sys.stderr)
    return 0


def _cmd_jog(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move without --yes.", file=sys.stderr)
        return 2
    with _arm_from_args(args) as arm:
        arm.enable()
        result = SDK_CAPABILITIES.dispatch(
            "jog_cartesian", arm, {
                "frame": args.frame,
                "translation_mm": [args.x_mm, args.y_mm, args.z_mm],
                "rotation_rpy_deg": [args.roll_deg, args.pitch_deg, args.yaw_deg],
                "orientation_mode": args.orientation_mode,
                "speed_mm_s": args.speed_mm_s,
                "acceleration_mm_s2": args.acceleration_mm_s2,
            },
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
        result = SDK_CAPABILITIES.dispatch("move_gripper", arm, {"position": position})
        _print_motion_result(result, as_json=args.json)
    return 0


def _cmd_ik(args: argparse.Namespace) -> int:
    with _arm_from_args(args) as arm:
        result = SDK_CAPABILITIES.dispatch(
            "solve_ik", arm, {
                "target_xyz_mm": [args.x_mm, args.y_mm, args.z_mm],
                "target_rpy_deg": [args.roll_deg, args.pitch_deg, args.yaw_deg],
                "orientation_mode": args.orientation_mode,
            },
        )
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
    with _arm_from_args(args) as arm:
        arm.enable()
        result = SDK_CAPABILITIES.dispatch(
            "move_linear", arm, {
                "target_xyz_mm": [args.x_mm, args.y_mm, args.z_mm],
                "target_rpy_deg": [args.roll_deg, args.pitch_deg, args.yaw_deg],
                "orientation_mode": args.orientation_mode,
                "speed_mm_s": args.speed_mm_s,
                "acceleration_mm_s2": args.acceleration_mm_s2,
            },
        )
        _print_motion_result(result, as_json=args.json)
    return 0

def _cmd_pose_list(args: argparse.Namespace) -> int:
    poses = SDK_CAPABILITIES.dispatch(
        "list_saved_poses", None, {"robot_id": args.robot_id},
    )
    for pose in poses:
        print(f"{pose['name']}\t{pose['source']}\t{pose['created_at']}")
    return 0


def _cmd_pose_capture(args: argparse.Namespace) -> int:
    with _arm_from_args(args) as arm:
        path = SDK_CAPABILITIES.dispatch(
            "capture_saved_pose", arm, {
                "robot_id": args.robot_id,
                "name": args.name,
                "source": args.source,
            },
        )
    print(f"Saved {args.name} to {path}")
    return 0

def _cmd_pose_go(args: argparse.Namespace) -> int:
    if not args.yes:
        print("Refusing to move hardware without --yes.", file=sys.stderr)
        return 2
    request = {
        "robot_id": args.robot_id,
        "name": args.name,
        "mode": args.mode,
        "orientation_mode": args.orientation_mode,
        "speed_deg_s": args.speed_deg_s,
        "acceleration_deg_s2": args.acceleration_deg_s2,
        "speed_mm_s": args.speed_mm_s,
        "acceleration_mm_s2": args.acceleration_mm_s2,
    }
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        # Preserve existing gate: verify saved-pose calibration *before*
        # enabling torque; replay rechecks it immediately before the move.
        SDK_CAPABILITIES.dispatch(
            "validate_saved_pose", arm, {
                "robot_id": args.robot_id,
                "name": args.name,
            },
        )
        arm.enable()
        result, gripper_result = SDK_CAPABILITIES.dispatch(
            "replay_saved_pose", arm, request,
        )
        print(result)
        print(gripper_result)
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
        status = SDK_CAPABILITIES.dispatch(
            "read_effort_status", arm, {"refresh": args.refresh},
        )
        print(json.dumps(status, indent=2))
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


def _cmd_camera_list(args: argparse.Namespace) -> int:
    devices = discover_camera_devices()
    payload = {
        "devices": devices,
        **SDK_CAPABILITIES.dispatch("camera_profiles", None, {}),
    }
    profile = WorkstationProfileStore().load()
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
    requested = str(getattr(args, "name", "") or "").strip()
    payload: object = SDK_CAPABILITIES.dispatch(
        "camera_profiles", None,
        {"name": requested} if requested else {},
    )
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
    request: dict[str, object] = {"name": name}
    if output is not None:
        request["output"] = output
    return SDK_CAPABILITIES.dispatch(
        "capture_camera", None, request, camera_settings=settings,
    )

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



def _agent_calibration_id(arm: SOARM101, *, simulation: bool) -> str:
    calibration_id = arm.calibration_id
    if calibration_id:
        return calibration_id
    if simulation:
        return "simulation"
    raise RuntimeError("agent motion requires an active calibrated follower")


def _agent_pose_names(robot_id: str) -> list[str]:
    return [
        name
        for name in PoseLibrary(robot_id).names()
        if name.startswith(AGENT_POSE_PREFIX)
    ]


def _agent_camera_names(profile: WorkstationProfile) -> list[str]:
    return [name for name in AGENT_CAMERA_NAMES if name in profile.cameras]


def _agent_require_authority(args: argparse.Namespace, arm: SOARM101) -> dict[str, object]:
    calibration_id = _agent_calibration_id(
        arm,
        simulation=bool(getattr(args, "simulation", False)),
    )
    authority = AgentAuthorityStore().require(
        robot_id=arm.config.robot_id,
        calibration_id=calibration_id,
    )
    return authority.status_payload()


def _agent_world_direction_payload(robot_id: str) -> dict[str, object]:
    try:
        workspace = WorkspaceCalibrationStore(robot_id).load()
    except Exception as exc:
        return {
            "available": False,
            "reason": str(exc),
        }

    authority = AgentAuthorityStore().load()
    if (
        authority is not None
        and authority.active()
        and authority.robot_id == robot_id
        and authority.calibration_id != workspace.arm_calibration_id
    ):
        return {
            "available": False,
            "reason": (
                "workspace calibration does not match the currently armed motor calibration"
            ),
            "workspace_id": workspace.workspace_id,
            "workspace_arm_calibration_id": workspace.arm_calibration_id,
            "authority_calibration_id": authority.calibration_id,
        }

    linear = np.asarray(workspace.physical_to_model_linear, dtype=float)

    def delta_for_physical_axis(axis: tuple[float, float, float]) -> list[float]:
        physical_mm = np.asarray(axis, dtype=float) / 1000.0
        model_delta_mm = linear @ physical_mm * 1000.0
        return [float(value) for value in model_delta_mm]

    return {
        "available": True,
        "source": "paper/workspace calibration",
        "workspace_id": workspace.workspace_id,
        "arm_calibration_id": workspace.arm_calibration_id,
        "convention": {
            "right": "+physical_x (A->B)",
            "left": "-physical_x (B->A)",
            "forward": "+physical_y (A->D / B->C)",
            "back": "-physical_y (D->A / C->B)",
            "up": "+physical_z (D->UP)",
            "down": "-physical_z (UP->D)",
        },
        "model_delta_mm_per_physical_mm": {
            "right": delta_for_physical_axis((1.0, 0.0, 0.0)),
            "left": delta_for_physical_axis((-1.0, 0.0, 0.0)),
            "forward": delta_for_physical_axis((0.0, 1.0, 0.0)),
            "back": delta_for_physical_axis((0.0, -1.0, 0.0)),
            "up": delta_for_physical_axis((0.0, 0.0, 1.0)),
            "down": delta_for_physical_axis((0.0, 0.0, -1.0)),
        },
        "usage": (
            "multiply the chosen direction vector by the requested physical distance in mm "
            "and pass the resulting XYZ values to 'soarm101 agent jog'"
        ),
    }


def _agent_capabilities_payload(
    robot_id: str,
    *,
    config: SOARM101Config | None = None,
) -> dict[str, object]:
    profile = WorkstationProfileStore().load()
    config = config or SOARM101Config(robot_id=robot_id)
    return {
        "authority": AgentAuthorityStore().status(),
        "poses": _agent_pose_names(robot_id),
        "cameras": _agent_camera_names(profile),
        "world_directions": _agent_world_direction_payload(robot_id),
        "actions": {
            "state": "read_only",
            "poses": "read_only",
            "cameras": "read_only",
            "capture": list(AGENT_CAMERA_NAMES),
            "go_pose": f"saved poses beginning with {AGENT_POSE_PREFIX!r}",
            "joint": {
                "mode": "single-joint relative angle",
                "joints": list(ARM_JOINTS),
                "max_abs_delta_deg": AGENT_JOINT_MAX_DELTA_DEG,
            },
            "jog": {
                "frames": ["world", "tool"],
                "translation_only": True,
            },
            "gripper": ["open", "close"],
            "sleep": True,
            "sleep_up": True,
            "stop": "always_available",
        },
        "motion_envelope": {
            **config.motion_limits_human,
            "servo_tracking": {
                "goal_velocity_raw": TELEOP_SERVO_SPEED_RAW,
                "acceleration_raw": TELEOP_SERVO_ACCELERATION_RAW,
            },
            "ownership": "trusted host / Motion SDK; broker clients cannot widen it",
        },
        "jog_policy": {
            "physical_height_threshold_mm": AGENT_JOG_HEIGHT_THRESHOLD_M * 1000.0,
            "max_distance_above_threshold_mm": AGENT_JOG_HIGH_MAX_DISTANCE_M * 1000.0,
            "max_distance_at_or_below_threshold_mm": AGENT_JOG_LOW_MAX_DISTANCE_M * 1000.0,
            "minimum_target_height_mm": AGENT_JOG_MINIMUM_TARGET_HEIGHT_M * 1000.0,
            "workspace_height_source": "saved measured workspace calibration",
        },
        "relax": "not exposed; human-only via 'soarm101 relax'",
    }


def _confirm_agent_arm_interactive(minutes: float) -> None:
    if not sys.stdin.isatty():
        raise PermissionError(
            "agent motion can only be armed by a human from an interactive terminal"
        )
    input(
        "AGENT MOTION AUTHORITY\n"
        f"This will park/hold the follower and authorize bounded agent motion for "
        f"{minutes:g} minutes. Keep physical power accessible.\n"
        "Press ENTER to arm the agent session, or Ctrl-C to cancel: "
    )


def _cmd_agent_arm(args: argparse.Namespace) -> int:
    minutes = float(args.minutes)
    if minutes <= 0.0 or minutes > MAX_AUTHORITY_MINUTES:
        raise ValueError(
            f"--minutes must be within (0, {MAX_AUTHORITY_MINUTES:g}]"
        )
    _confirm_agent_arm_interactive(minutes)
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        arm.enable()
        arm.hold()
        calibration_id = _agent_calibration_id(
            arm,
            simulation=bool(getattr(args, "simulation", False)),
        )
        authority = AgentAuthorityStore().issue(
            robot_id=arm.config.robot_id,
            calibration_id=calibration_id,
            minutes=minutes,
        )
    payload = authority.status_payload()
    payload["holding"] = True
    print(json.dumps(payload, indent=2))
    return 0


def _cmd_agent_disarm(_: argparse.Namespace) -> int:
    AgentAuthorityStore().clear()
    print(
        json.dumps(
            {
                "armed": False,
                "holding": "unchanged",
                "note": "motion authority removed; use human-confirmed 'soarm101 relax' to release torque",
            },
            indent=2,
        )
    )
    return 0


def _cmd_agent_capabilities(args: argparse.Namespace) -> int:
    config = SOARM101Config(
        robot_id=args.robot_id,
        **_motion_limit_overrides(args),
    )
    print(
        json.dumps(
            _agent_capabilities_payload(args.robot_id, config=config),
            indent=2,
        )
    )
    return 0


def _cmd_agent_poses(args: argparse.Namespace) -> int:
    print(json.dumps({"poses": _agent_pose_names(args.robot_id)}, indent=2))
    return 0


def _cmd_agent_cameras(_: argparse.Namespace) -> int:
    profile = WorkstationProfileStore().load()
    print(json.dumps({"cameras": _agent_camera_names(profile)}, indent=2))
    return 0


def _cmd_agent_capture(args: argparse.Namespace) -> int:
    name = str(args.name).strip()
    if name not in AGENT_CAMERA_NAMES:
        raise PermissionError(
            f"agent camera {name!r} is not allowed; allowed names: "
            + ", ".join(AGENT_CAMERA_NAMES)
        )
    profile = WorkstationProfileStore().load()
    if name not in profile.cameras:
        raise KeyError(f"agent camera {name!r} is not configured")
    print(
        json.dumps(
            _capture_named_camera(name, profile.camera(name), args.output),
            indent=2,
        )
    )
    return 0


def _cmd_agent_state(args: argparse.Namespace) -> int:
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        joints = dict(arm.get_joint_positions().positions)
        tcp = arm.get_position().xyz_rpy()
        gripper = float(arm.tool.get_position())
        calibration_id = _agent_calibration_id(
            arm,
            simulation=bool(getattr(args, "simulation", False)),
        )
        physical_height_mm: float | None = None
        workspace_error: str | None = None
        if calibration_id != "simulation":
            try:
                workspace = WorkspaceCalibrationStore(arm.config.robot_id).load()
                if workspace.arm_calibration_id != calibration_id:
                    raise ValueError(
                        "workspace calibration does not match active motor calibration"
                    )
                physical = workspace.physical_position_from_model(tcp[:3])
                physical_height_mm = float(physical[2] * 1000.0)
            except Exception as exc:
                workspace_error = str(exc)
    print(
        json.dumps(
            {
                "authority": AgentAuthorityStore().status(),
                "robot_id": arm.config.robot_id,
                "calibration_id": calibration_id,
                "joint_positions_rad": joints,
                "tcp_xyz_mm": [float(value * 1000.0) for value in tcp[:3]],
                "tcp_rpy_deg": [float(value * 180.0 / pi) for value in tcp[3:]],
                "gripper": gripper,
                "physical_height_mm": physical_height_mm,
                "workspace_height_error": workspace_error,
            },
            indent=2,
        )
    )
    return 0


def _cmd_agent_go_pose(args: argparse.Namespace) -> int:
    name = str(args.name).strip()
    if not name.startswith(AGENT_POSE_PREFIX):
        raise PermissionError(
            f"agent pose names must begin with {AGENT_POSE_PREFIX!r}"
        )
    pose = PoseLibrary(args.robot_id).require(name)
    rates = _agent_validated_pose_rates(args)
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        authority = _agent_require_authority(args, arm)
        arm.require_artifact_calibration(
            {
                "source_robot_id": pose.source_robot_id,
                "source_calibration_id": pose.source_calibration_id,
                "target_robot_id": pose.target_robot_id,
                "target_calibration_id": pose.target_calibration_id,
            },
            artifact_label=f"agent saved pose {name!r}",
        )
        arm.enable()
        arm_result = arm.move_joints_from_saved_pose(
            pose.joints,
            speed=rates[0] * pi / 180.0,
            acceleration=rates[1] * pi / 180.0,
        )
        gripper_result = arm.tool.move(
            pose.gripper, speed_raw=rates[2], acceleration_raw=rates[3],
        )
        arm.hold()
    print(
        json.dumps(
            {
                "accepted": True,
                "completed": True,
                "action": "go_pose",
                "pose": name,
                "holding": True,
                "authority": authority,
                "arm": asdict(arm_result),
                "gripper": asdict(gripper_result),
            },
            indent=2,
        )
    )
    return 0


def _agent_requested_rate(value: float, name: str, ceiling: float) -> float:
    """Reject unsafe requested rates before opening hardware."""
    speed = float(value)
    if not np.isfinite(speed) or speed <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    if speed > ceiling:
        raise ValueError(f"{name} exceeds the configured motion envelope ({ceiling:g})")
    return speed


def _agent_gripper_rates(args: argparse.Namespace) -> tuple[int, int]:
    speed = args.gripper_speed_raw
    acceleration = args.gripper_acceleration_raw
    if type(speed) is not int or not 1 <= speed <= 3400:
        raise ValueError("gripper speed raw must be in [1, 3400]")
    if type(acceleration) is not int or not 1 <= acceleration <= 254:
        raise ValueError("gripper acceleration raw must be in [1, 254]")
    return speed, acceleration


def _agent_validated_pose_rates(args: argparse.Namespace) -> tuple[float, float, int, int]:
    limits = SOARM101Config(
        robot_id=args.robot_id, **_motion_limit_overrides(args)
    ).motion_limits_human
    speed = _agent_requested_rate(
        args.speed_deg_s, "joint speed deg/s", limits["max_joint_speed_deg_s"]
    )
    acceleration = _agent_requested_rate(
        args.acceleration_deg_s2, "joint acceleration deg/s^2",
        limits["max_joint_acceleration_deg_s2"]
    )
    return speed, acceleration, *_agent_gripper_rates(args)


def _cmd_agent_joint(args: argparse.Namespace) -> int:
    delta_deg = float(args.delta_deg)
    if not np.isfinite(delta_deg) or abs(delta_deg) <= 1e-9:
        raise ValueError("agent joint delta must be finite and non-zero")
    if abs(delta_deg) > AGENT_JOINT_MAX_DELTA_DEG:
        raise PermissionError(
            f"agent joint delta {delta_deg:.1f} deg exceeds "
            f"{AGENT_JOINT_MAX_DELTA_DEG:.1f} deg per-command limit"
        )
    delta_rad = delta_deg * pi / 180.0
    limits = SOARM101Config(
        robot_id=args.robot_id, **_motion_limit_overrides(args)
    ).motion_limits_human
    requested_speed = _agent_requested_rate(
        args.speed_deg_s, "joint speed deg/s", limits["max_joint_speed_deg_s"]
    )
    requested_acceleration = _agent_requested_rate(
        args.acceleration_deg_s2, "joint acceleration deg/s^2",
        limits["max_joint_acceleration_deg_s2"],
    )
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        authority = _agent_require_authority(args, arm)
        before = dict(arm.get_joint_positions().positions)
        arm.enable()
        result = arm.move_joints(
            {args.joint: delta_rad},
            relative=True,
            speed=requested_speed * pi / 180.0,
            acceleration=requested_acceleration * pi / 180.0,
        )
        arm.hold()
        after = dict(arm.get_joint_positions().positions)
    print(json.dumps({
        "accepted": result.accepted,
        "completed": result.completed,
        "action": "joint",
        "joint": args.joint,
        "delta_deg": delta_deg,
        "before_deg": before[args.joint] * 180.0 / pi,
        "after_deg": after[args.joint] * 180.0 / pi,
        "holding": True,
        "authority": authority,
        "message": result.message,
    }, indent=2))
    return 0


def _cmd_agent_jog(args: argparse.Namespace) -> int:
    delta_mm = np.asarray([args.x_mm, args.y_mm, args.z_mm], dtype=float)
    if not np.all(np.isfinite(delta_mm)):
        raise ValueError("agent jog deltas must be finite")
    limits = SOARM101Config(
        robot_id=args.robot_id, **_motion_limit_overrides(args)
    ).motion_limits_human
    requested_speed = _agent_requested_rate(
        args.speed_mm_s, "Cartesian speed mm/s", limits["max_linear_speed_mm_s"]
    )
    requested_acceleration = _agent_requested_rate(
        args.acceleration_mm_s2, "Cartesian acceleration mm/s^2",
        limits["max_linear_acceleration_mm_s2"],
    )
    trace_summary: dict[str, int] | None = None
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        authority = _agent_require_authority(args, arm)
        calibration_id = _agent_calibration_id(
            arm,
            simulation=bool(getattr(args, "simulation", False)),
        )
        if calibration_id == "simulation":
            raise PermissionError(
                "agent jog requires a measured workspace calibration and is not enabled in simulation"
            )
        workspace = WorkspaceCalibrationStore(arm.config.robot_id).load()
        trace_context = (
            PassiveBackendTrace(
                arm,
                args.trace_file,
                metadata={
                    "action": "agent_jog",
                    "frame": args.frame,
                    "delta_model_mm": [float(v) for v in delta_mm],
                    "requested_speed_mm_s": requested_speed,
                    "requested_acceleration_mm_s2": requested_acceleration,
                },
            )
            if args.trace_file is not None
            else nullcontext()
        )
        with trace_context as trace:
            current = arm.get_position()
            target = relative_target_pose(
                current,
                translation_m=delta_mm / 1000.0,
                frame=args.frame,
            )
            decision = evaluate_agent_jog(
                workspace,
                active_calibration_id=calibration_id,
                current_model_position_m=current.position,
                delta_model_m=target.position - current.position,
            )
            if trace is not None:
                trace.mark(
                    "preflight",
                    start_model_xyz_mm=[float(v * 1000.0) for v in current.position],
                    target_model_xyz_mm=[float(v * 1000.0) for v in target.position],
                    physical_policy=decision.to_payload(),
                )
            arm.enable()
            if trace is not None:
                trace.mark("motion_start")
            result = jog_linear_cli_units(
                arm,
                frame=args.frame,
                translation_mm=tuple(float(value) for value in delta_mm),
                rotation_rpy_deg=(0.0, 0.0, 0.0),
                orientation_mode="compatible",
                speed_mm_s=requested_speed,
                acceleration_mm_s2=requested_acceleration,
            )
            if trace is not None:
                trace.mark(
                    "motion_completed_before_hold",
                    accepted=result.accepted,
                    completed=result.completed,
                    joints_rad=dict(result.final_positions),
                )
                # Every planned joint command is recorded as a "command" event;
                # the subsequent HOLD latch is recorded separately below.
                trace.mark("hold_start")
            arm.hold()
            if trace is not None:
                trace.mark("hold_complete")
                trace.mark(
                    "post_hold_immediate",
                    joints_rad=dict(arm.get_joint_positions().positions),
                )
                # Deliberate post-motion read only, not trajectory-time polling.
                time.sleep(2.0)
                trace.mark(
                    "post_hold_2s",
                    joints_rad=dict(arm.get_joint_positions().positions),
                )
                trace_summary = dict(trace.summary)
    print(
        json.dumps(
            {
                "accepted": result.accepted,
                "completed": result.completed,
                "action": "jog",
                "holding": True,
                "authority": authority,
                "policy": decision.to_payload(),
                "final_positions": result.final_positions,
                "message": result.message,
                **(
                    {
                        "trace_file": str(Path(args.trace_file).expanduser()),
                        "trace_summary": trace_summary,
                    }
                    if args.trace_file is not None
                    else {}
                ),
            },
            indent=2,
        )
    )
    return 0


def _cmd_agent_trace_summary(args: argparse.Namespace) -> int:
    """Summarize a previously recorded JSONL jog; no hardware access."""
    print(json.dumps(summarize_agent_jog_trace(args.path), indent=2))
    return 0


def _cmd_agent_gripper(args: argparse.Namespace) -> int:
    gripper_speed, gripper_accel = _agent_gripper_rates(args)
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        authority = _agent_require_authority(args, arm)
        arm.enable()
        closed = float(arm.get_sleep_gripper_position())
        target = closed if args.target == "close" else 1.0 - closed
        result = arm.tool.move(
            target, speed_raw=gripper_speed, acceleration_raw=gripper_accel,
        )
        arm.hold()
    print(
        json.dumps(
            {
                "accepted": result.accepted,
                "completed": result.completed,
                "action": "gripper",
                "target": args.target,
                "normalized_target": target,
                "holding": True,
                "authority": authority,
                "final_positions": result.final_positions,
                "message": result.message,
            },
            indent=2,
        )
    )
    return 0


def _cmd_agent_sleep(args: argparse.Namespace) -> int:
    speed, acceleration, gripper_speed, gripper_accel = _agent_validated_pose_rates(args)
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        authority = _agent_require_authority(args, arm)
        arm.enable()
        result = arm.move_sleep(
            speed=speed * pi / 180.0,
            acceleration=acceleration * pi / 180.0,
            gripper_speed_raw=gripper_speed,
            gripper_acceleration_raw=gripper_accel,
        )
        arm.hold()
    payload = asdict(result)
    payload.update(
        {
            "action": "sleep",
            "holding": True,
            "authority": authority,
        }
    )
    print(json.dumps(payload, indent=2))
    return 0


def _cmd_agent_sleep_up(args: argparse.Namespace) -> int:
    speed, acceleration, gripper_speed, gripper_accel = _agent_validated_pose_rates(args)
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        authority = _agent_require_authority(args, arm)
        arm.enable()
        result = arm.move_sleep_up(
            speed=speed * pi / 180.0,
            acceleration=acceleration * pi / 180.0,
            gripper_speed_raw=gripper_speed,
            gripper_acceleration_raw=gripper_accel,
        )
        arm.hold()
    payload = asdict(result)
    payload.update(
        {
            "action": "sleep_up",
            "holding": True,
            "authority": authority,
        }
    )
    print(json.dumps(payload, indent=2))
    return 0


def _cmd_agent_stop(args: argparse.Namespace) -> int:
    with _arm_from_args(args, disable_torque_on_disconnect=False) as arm:
        arm.enable()
        arm.stop()
        joints = dict(arm.get_joint_positions().positions)
    print(
        json.dumps(
            {
                "accepted": True,
                "completed": True,
                "action": "stop",
                "holding": True,
                "joint_positions_rad": joints,
            },
            indent=2,
        )
    )
    return 0


def _workstation_payload(profile: WorkstationProfile) -> dict[str, object]:
    return {
        "schema_version": profile.schema_version,
        "follower": asdict(profile.follower),
        "leader": asdict(profile.leader),
        **profile.camera_payload(),
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


def _cmd_agent_sandbox_agents(args: argparse.Namespace) -> int:
    from soarm101_motion.agent_adapters import agent_catalog

    payload = {"agents": agent_catalog()}
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        for item in payload["agents"]:
            model = item["default_model"] or "<agent default>"
            auth_summary = "; ".join(
                (
                    f"{mode['name']} -> "
                    f"{mode['provider'] or '<native-login>'} "
                    f"({', '.join(mode['credential_env_vars']) or ('login state' if mode['uses_login_state'] else 'no credential')})"
                )
                for mode in item["auth_modes"]
            )
            print(
                f"{item['name']}: model={model} default_auth={item['default_auth']} "
                f"auth=[{auth_summary}]"
            )
    return 0


def _cmd_agent_sandbox_doctor(args: argparse.Namespace) -> int:
    from soarm101_motion.agent_sandbox import doctor

    manifest = Path(args.adapter_manifest) if args.adapter_manifest else None
    result = doctor(
        agent=args.agent,
        auth=args.auth,
        image=args.image,
        provider=args.provider,
        adapter_manifest=manifest,
    )
    payload = result.as_dict()
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        state = "READY" if result.ready else "NOT READY"
        print(f"SO-ARM101 {result.agent} agent sandbox: {state}")
        for key in ("openshell", "gateway", "docker", "image", "provider"):
            print(f"  {key:10s} {'ok' if payload[key] else 'missing'}")
        if result.details:
            print("Details:")
            for key, value in result.details.items():
                if value:
                    print(f"  {key}: {value}")
    return 0 if result.ready else 2


def _cmd_agent_sandbox_setup(args: argparse.Namespace) -> int:
    from soarm101_motion.agent_adapters import resolve_agent_adapter
    from soarm101_motion.agent_sandbox import setup

    manifest = Path(args.adapter_manifest) if args.adapter_manifest else None
    adapter = resolve_agent_adapter(args.agent, manifest)
    setup(
        agent=adapter.name,
        auth=args.auth,
        image=args.image,
        provider=args.provider,
        reauth=args.reauth,
        adapter_manifest=manifest,
    )
    auth_suffix = f" --auth {args.auth}" if args.auth else ""
    manifest_suffix = (
        f" --adapter-manifest {args.adapter_manifest}" if args.adapter_manifest else ""
    )
    print(
        f"SO-ARM101 {adapter.name} agent sandbox setup complete. "
        f"Run 'soarm101 agent sandbox doctor --agent {adapter.name}{auth_suffix}"
        f"{manifest_suffix}' to verify readiness."
    )
    return 0


def _cmd_agent_sandbox_run(args: argparse.Namespace) -> int:
    from soarm101_motion.agent_adapters import resolve_agent_adapter
    from soarm101_motion.agent_sandbox import run_agent

    manifest = Path(args.adapter_manifest) if args.adapter_manifest else None
    adapter = resolve_agent_adapter(args.agent, manifest)
    output_dir = (
        Path(args.output)
        if args.output
        else Path("soarm101-agent-runs")
        / f"{time.strftime('%Y%m%d-%H%M%S')}-{adapter.name}"
    )
    task = Path(args.task) if args.task else None
    result = run_agent(
        agent=adapter.name,
        auth=args.auth,
        task=task,
        output_dir=output_dir,
        model=args.model,
        image=args.image,
        provider=args.provider,
        broker_port=args.broker_port,
        max_turns=args.max_turns,
        timeout=args.timeout,
        read_only=args.read_only,
        interface=args.interface,
        capability_profile=Path(args.capability_profile) if args.capability_profile else None,
        adapter_manifest=manifest,
    )
    payload = result.as_dict()
    print(json.dumps(payload, indent=2))
    return 0 if result.exit_code == 0 else result.exit_code


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

    sdk_capabilities = sub.add_parser(
        "sdk-capabilities", help="inspect SDK action contracts without hardware access",
    )
    sdk_capabilities.add_argument("--json", action="store_true")
    sdk_capabilities.set_defaults(func=_cmd_sdk_capabilities)

    def add_motion_limit_options(command: argparse.ArgumentParser) -> None:
        group = command.add_argument_group("motion envelope")
        group.add_argument(
            "--max-joint-speed-deg-s",
            type=float,
            help="absolute joint-speed ceiling in deg/s (SDK default: 100)",
        )
        group.add_argument(
            "--max-joint-acceleration-deg-s2",
            type=float,
            help="absolute joint-acceleration ceiling in deg/s^2 (SDK default: 1000)",
        )
        group.add_argument(
            "--max-linear-speed-mm-s",
            type=float,
            help="absolute TCP linear-speed ceiling in mm/s (SDK default: 100)",
        )
        group.add_argument(
            "--max-linear-acceleration-mm-s2",
            type=float,
            help="absolute TCP linear-acceleration ceiling in mm/s^2 (SDK default: 1000)",
        )
        group.add_argument(
            "--max-tool-angular-speed-deg-s",
            type=float,
            help="absolute TCP orientation-speed ceiling in deg/s (SDK default: 100)",
        )
        group.add_argument(
            "--max-tool-angular-acceleration-deg-s2",
            type=float,
            help="absolute TCP orientation-acceleration ceiling in deg/s^2 (SDK default: 1000)",
        )

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
        add_motion_limit_options(command)

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

    motion_envelope = sub.add_parser(
        "motion-envelope",
        aliases=["motion_settings", "motion-settings"],
        help="show or evaluate the host motion envelope in human-friendly units",
    )
    motion_envelope.add_argument("--robot-id", default="so101")
    add_motion_limit_options(motion_envelope)
    motion_envelope.add_argument("--json", action="store_true")
    motion_envelope.set_defaults(func=_cmd_motion_envelope)

    limits = sub.add_parser(
        "limits",
        help="show saved calibrated, model, and effective joint/workspace limits",
    )
    limits.add_argument("--robot-id", default="so101")
    limits.add_argument("--calibration")
    add_motion_limit_options(limits)
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
    move_speed = move.add_mutually_exclusive_group()
    move_speed.add_argument(
        "--speed",
        type=float,
        help="legacy joint speed in rad/s",
    )
    move_speed.add_argument(
        "--speed-deg-s",
        type=float,
        help="joint speed in deg/s",
    )
    move_acceleration = move.add_mutually_exclusive_group()
    move_acceleration.add_argument(
        "--acceleration",
        type=float,
        help="legacy joint acceleration in rad/s^2",
    )
    move_acceleration.add_argument(
        "--acceleration-deg-s2",
        type=float,
        help="joint acceleration in deg/s^2",
    )
    move.add_argument("--yes", action="store_true")
    move.add_argument("--json", action="store_true")
    move.set_defaults(func=_cmd_move_joints)

    sleep = sub.add_parser(
        "sleep",
        help="move to the calibration-derived smoother default Sleep posture",
    )
    add_session_options(sleep)
    sleep.add_argument("--speed-deg-s", type=float, default=8.0)
    sleep.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    sleep.add_argument("--yes", action="store_true")
    sleep.add_argument("--json", action="store_true")
    sleep.set_defaults(func=_cmd_sleep)

    sleep_up = sub.add_parser(
        "sleep-up",
        aliases=["sleep_up"],
        help="move to the historical fully folded wrist-up Sleep posture",
    )
    add_session_options(sleep_up)
    sleep_up.add_argument("--speed-deg-s", type=float, default=8.0)
    sleep_up.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    sleep_up.add_argument("--yes", action="store_true")
    sleep_up.add_argument("--json", action="store_true")
    sleep_up.set_defaults(func=_cmd_sleep_up)

    joint_jog = sub.add_parser(
        "jog-joint", help="perform one guarded relative operator joint jog",
    )
    add_session_options(joint_jog)
    joint_jog.add_argument("joint", choices=ARM_JOINTS)
    joint_jog.add_argument("--delta-deg", type=float, required=True)
    joint_jog.add_argument("--speed-deg-s", type=float, default=8.0)
    joint_jog.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    joint_jog.add_argument("--yes", action="store_true")
    joint_jog.add_argument("--json", action="store_true")
    joint_jog.set_defaults(func=_cmd_jog_joint)

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

    agent = sub.add_parser(
        "agent",
        help="bounded robot/camera capabilities for external reasoning agents",
    )
    agent_sub = agent.add_subparsers(dest="agent_command", required=True)

    agent_arm = agent_sub.add_parser(
        "arm",
        help="human-authorize a time-bounded agent motion session and park the follower",
    )
    add_session_options(agent_arm)
    agent_arm.add_argument(
        "--minutes",
        type=float,
        default=DEFAULT_AUTHORITY_MINUTES,
        help=f"motion authority duration; maximum {MAX_AUTHORITY_MINUTES:g} minutes",
    )
    agent_arm.set_defaults(func=_cmd_agent_arm)

    agent_disarm = agent_sub.add_parser(
        "disarm",
        help="remove agent motion authority without relaxing the follower",
    )
    agent_disarm.set_defaults(func=_cmd_agent_disarm)

    agent_capabilities = agent_sub.add_parser(
        "capabilities",
        help="show the bounded actions, named poses/cameras, and authority state",
    )
    agent_capabilities.add_argument("--robot-id", default="so101")
    add_motion_limit_options(agent_capabilities)
    agent_capabilities.set_defaults(func=_cmd_agent_capabilities)

    agent_state = agent_sub.add_parser("state", help="read robot state without commanding motion")
    add_session_options(agent_state)
    agent_state.set_defaults(func=_cmd_agent_state)

    agent_poses = agent_sub.add_parser("poses", help="list saved agent_* poses")
    agent_poses.add_argument("--robot-id", default="so101")
    agent_poses.set_defaults(func=_cmd_agent_poses)

    agent_cameras = agent_sub.add_parser(
        "cameras",
        help="list configured agent-visible cameras",
    )
    agent_cameras.set_defaults(func=_cmd_agent_cameras)

    agent_capture = agent_sub.add_parser(
        "capture",
        help="capture a fresh still from the overhead or wrist camera",
    )
    agent_capture.add_argument("name", choices=AGENT_CAMERA_NAMES)
    agent_capture.add_argument("--output")
    agent_capture.set_defaults(func=_cmd_agent_capture)

    agent_pose = agent_sub.add_parser(
        "go-pose",
        help="move to a calibration-bound saved agent_* pose and remain holding",
    )
    add_session_options(agent_pose)
    agent_pose.add_argument("name")
    agent_pose.add_argument("--speed-deg-s", type=float, default=8.0)
    agent_pose.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    agent_pose.add_argument("--gripper-speed-raw", type=int, default=250)
    agent_pose.add_argument("--gripper-acceleration-raw", type=int, default=20)
    agent_pose.set_defaults(func=_cmd_agent_go_pose)

    agent_joint = agent_sub.add_parser(
        "joint",
        help="bounded relative adjustment of one named arm joint",
    )
    add_session_options(agent_joint)
    agent_joint.add_argument("joint", choices=ARM_JOINTS)
    agent_joint.add_argument("--delta-deg", type=float, required=True)
    agent_joint.add_argument("--speed-deg-s", type=float, default=8.0)
    agent_joint.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    agent_joint.set_defaults(func=_cmd_agent_joint)

    agent_jog = agent_sub.add_parser(
        "jog",
        help="bounded world- or tool-frame translation using measured physical-height limits",
    )
    add_session_options(agent_jog)
    agent_jog.add_argument("--frame", choices=("world", "tool"), default="world")
    agent_jog.add_argument("--x-mm", type=float, default=0.0)
    agent_jog.add_argument("--y-mm", type=float, default=0.0)
    agent_jog.add_argument("--z-mm", type=float, default=0.0)
    agent_jog.add_argument("--speed-mm-s", type=float, default=10.0)
    agent_jog.add_argument("--acceleration-mm-s2", type=float, default=40.0)
    agent_jog.add_argument(
        "--trace-file",
        type=Path,
        help="trusted-host JSONL motion trace: command/feedback, HOLD latch and 2s settling",
    )
    agent_jog.set_defaults(func=_cmd_agent_jog)

    agent_trace_summary = agent_sub.add_parser(
        "trace-summary",
        help="summarize model trajectory, encoder feedback and raw HOLD goals from a local JSONL trace",
    )
    agent_trace_summary.add_argument("path", type=Path)
    agent_trace_summary.set_defaults(func=_cmd_agent_trace_summary)

    agent_gripper = agent_sub.add_parser(
        "gripper",
        help="open or close the stock gripper using calibration-inset endpoints",
    )
    add_session_options(agent_gripper)
    agent_gripper.add_argument("target", choices=("open", "close"))
    agent_gripper.add_argument("--gripper-speed-raw", type=int, default=250)
    agent_gripper.add_argument("--gripper-acceleration-raw", type=int, default=20)
    agent_gripper.set_defaults(func=_cmd_agent_gripper)

    agent_sleep = agent_sub.add_parser(
        "sleep",
        help="move to calibrated default Sleep and remain holding",
    )
    add_session_options(agent_sleep)
    agent_sleep.add_argument("--speed-deg-s", type=float, default=8.0)
    agent_sleep.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    agent_sleep.add_argument("--gripper-speed-raw", type=int, default=250)
    agent_sleep.add_argument("--gripper-acceleration-raw", type=int, default=20)
    agent_sleep.set_defaults(func=_cmd_agent_sleep)

    agent_sleep_up = agent_sub.add_parser(
        "sleep-up",
        aliases=["sleep_up"],
        help="move to the historical calibrated wrist-up Sleep and remain holding",
    )
    add_session_options(agent_sleep_up)
    agent_sleep_up.add_argument("--speed-deg-s", type=float, default=8.0)
    agent_sleep_up.add_argument("--acceleration-deg-s2", type=float, default=25.0)
    agent_sleep_up.add_argument("--gripper-speed-raw", type=int, default=250)
    agent_sleep_up.add_argument("--gripper-acceleration-raw", type=int, default=20)
    agent_sleep_up.set_defaults(func=_cmd_agent_sleep_up)

    agent_stop = agent_sub.add_parser(
        "stop",
        help="STOP/HOLD the follower; available even without active agent authority",
    )
    add_session_options(agent_stop)
    agent_stop.set_defaults(func=_cmd_agent_stop)

    agent_sandbox = agent_sub.add_parser(
        "sandbox",
        help="self-contained OpenShell environment for reasoning agents",
    )
    agent_sandbox_sub = agent_sandbox.add_subparsers(
        dest="agent_sandbox_command",
        required=True,
    )

    agent_sandbox_agents = agent_sandbox_sub.add_parser(
        "agents",
        help="list packaged agent adapters and their default provider/model requirements",
    )
    agent_sandbox_agents.add_argument("--json", action="store_true")
    agent_sandbox_agents.set_defaults(func=_cmd_agent_sandbox_agents)

    agent_sandbox_doctor = agent_sandbox_sub.add_parser(
        "doctor",
        help="check OpenShell, Docker, image, and provider readiness for one agent",
    )
    agent_sandbox_doctor.add_argument(
        "--agent",
        default="hermes",
        help="built-in agent name or the name declared by --adapter-manifest",
    )
    agent_sandbox_doctor.add_argument(
        "--auth",
        help="agent authentication mode; defaults to the selected adapter's default",
    )
    agent_sandbox_doctor.add_argument(
        "--adapter-manifest",
        help="JSON manifest for an external OpenShell-compatible CLI agent",
    )
    agent_sandbox_doctor.add_argument(
        "--image",
        help="override the selected agent's canonical sandbox image",
    )
    agent_sandbox_doctor.add_argument(
        "--provider",
        help="override the selected agent's canonical OpenShell provider",
    )
    agent_sandbox_doctor.add_argument("--json", action="store_true")
    agent_sandbox_doctor.set_defaults(func=_cmd_agent_sandbox_doctor)

    agent_sandbox_setup = agent_sandbox_sub.add_parser(
        "setup",
        help="build one agent image and install/update its provider",
    )
    agent_sandbox_setup.add_argument(
        "--agent",
        default="hermes",
        help="built-in agent name or the name declared by --adapter-manifest",
    )
    agent_sandbox_setup.add_argument(
        "--auth",
        help=(
            "agent authentication mode; normal Codex choices are api-key or installed; "
            "chatgpt is an advanced separate device-login mode; external manifests default "
            "to external"
        ),
    )
    agent_sandbox_setup.add_argument(
        "--adapter-manifest",
        help="JSON manifest for an external OpenShell-compatible CLI agent",
    )
    agent_sandbox_setup.add_argument("--image")
    agent_sandbox_setup.add_argument("--provider")
    agent_sandbox_setup.add_argument(
        "--reauth",
        action="store_true",
        help=(
            "force a fresh dedicated Codex ChatGPT device login; not used by api-key "
            "or installed modes"
        ),
    )
    agent_sandbox_setup.set_defaults(func=_cmd_agent_sandbox_setup)

    agent_sandbox_run = agent_sandbox_sub.add_parser(
        "run",
        help="run a supported agent in OpenShell through the bounded robot/camera broker",
    )
    agent_sandbox_run.add_argument(
        "--agent",
        default="hermes",
        help="built-in agent name or the name declared by --adapter-manifest",
    )
    agent_sandbox_run.add_argument(
        "--auth",
        help=(
            "agent authentication mode; normal Codex choices are api-key or installed; "
            "chatgpt is an advanced separate device-login mode; external manifests default "
            "to external"
        ),
    )
    agent_sandbox_run.add_argument(
        "--adapter-manifest",
        help="JSON manifest for an external OpenShell-compatible CLI agent",
    )
    agent_sandbox_run.add_argument(
        "--task",
        help=(
            "task markdown; required for full-control runs. "
            "Read-only runs use the packaged validation task when omitted."
        ),
    )
    agent_sandbox_run.add_argument(
        "--output",
        help=(
            "new or empty host directory for traces/captures/evidence; "
            "default: soarm101-agent-runs/<timestamp>"
        ),
    )
    agent_sandbox_run.add_argument(
        "--model",
        help=(
            "agent model override; Hermes defaults to its packaged OpenRouter model, "
            "Codex uses its CLI default when omitted"
        ),
    )
    agent_sandbox_run.add_argument(
        "--image",
        help="override the selected agent's canonical sandbox image",
    )
    agent_sandbox_run.add_argument(
        "--provider",
        help="override the selected agent's canonical OpenShell provider",
    )
    agent_sandbox_run.add_argument(
        "--interface", choices=("robotctl", "mcp"), default="robotctl",
        help="bounded robot tool interface; robotctl remains the default baseline",
    )
    agent_sandbox_run.add_argument(
        "--capability-profile", help="trusted host JSON tool/camera/limit profile for this run",
    )
    agent_sandbox_run.add_argument("--broker-port", type=int, default=8765)
    agent_sandbox_run.add_argument("--max-turns", type=int, default=100)
    agent_sandbox_run.add_argument("--timeout", type=int, default=1800)
    agent_sandbox_run.add_argument(
        "--read-only",
        action="store_true",
        help=(
            "omit all motion/gripper/STOP routes from the OpenShell broker policy "
            "and do not require human motion authority"
        ),
    )
    agent_sandbox_run.set_defaults(func=_cmd_agent_sandbox_run)

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
    add_motion_limit_options(smoke)
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
