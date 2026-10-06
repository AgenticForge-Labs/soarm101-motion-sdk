"""Configuration for hardware and simulation operation."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

from soarm101_motion.constants import (
    ALL_MOTORS,
    DEFAULT_COMMAND_FREQUENCY_HZ,
    DEFAULT_JOINT_ACCEL_RAD_S2,
    DEFAULT_JOINT_SPEED_RAD_S,
    DEFAULT_LINEAR_ACCEL_M_S2,
    DEFAULT_LINEAR_SPEED_M_S,
    DEFAULT_MAX_COMMAND_STEP_RAD,
    DEFAULT_MAX_JOINT_ACCEL_DEG_S2,
    DEFAULT_MAX_JOINT_ACCEL_RAD_S2,
    DEFAULT_MAX_JOINT_SPEED_DEG_S,
    DEFAULT_MAX_JOINT_SPEED_RAD_S,
    DEFAULT_MAX_LINEAR_ACCEL_M_S2,
    DEFAULT_MAX_LINEAR_ACCEL_MM_S2,
    DEFAULT_MAX_LINEAR_SPEED_M_S,
    DEFAULT_MAX_LINEAR_SPEED_MM_S,
    DEFAULT_MAX_TOOL_ANGULAR_ACCEL_DEG_S2,
    DEFAULT_MAX_TOOL_ANGULAR_ACCEL_RAD_S2,
    DEFAULT_MAX_TOOL_ANGULAR_SPEED_DEG_S,
    DEFAULT_MAX_TOOL_ANGULAR_SPEED_RAD_S,
    STS3215_MAX_POSITION_SPEED_RAW,
    TELEOP_SERVO_ACCELERATION_RAW,
    TELEOP_SERVO_SPEED_RAW,
)
from soarm101_motion.exceptions import ConfigurationError


@dataclass(frozen=True)
class SOARM101Config:
    port: str | None = None
    robot_id: str = "so101"
    baudrate: int = 1_000_000
    calibration_path: Path | None = None
    use_stored_calibration: bool = True
    verify_calibration_on_connect: bool = True
    command_frequency_hz: float = DEFAULT_COMMAND_FREQUENCY_HZ
    default_joint_speed: float = DEFAULT_JOINT_SPEED_RAD_S
    default_joint_acceleration: float = DEFAULT_JOINT_ACCEL_RAD_S2
    default_linear_speed: float = DEFAULT_LINEAR_SPEED_M_S
    default_linear_acceleration: float = DEFAULT_LINEAR_ACCEL_M_S2
    default_angular_speed: float = 0.8
    default_angular_acceleration: float = 2.0

    # Absolute host-side motion envelope. Public/CLI presentation uses degrees
    # and millimeters; deterministic internals remain SI.
    max_joint_speed: float = DEFAULT_MAX_JOINT_SPEED_RAD_S
    max_joint_acceleration: float = DEFAULT_MAX_JOINT_ACCEL_RAD_S2
    # Optional live-stream ceilings. None keeps the planned-motion ceiling.
    teleop_max_joint_speed: float | None = None
    teleop_max_joint_acceleration: float | None = None
    max_linear_speed: float = DEFAULT_MAX_LINEAR_SPEED_M_S
    max_linear_acceleration: float = DEFAULT_MAX_LINEAR_ACCEL_M_S2
    max_angular_speed: float = DEFAULT_MAX_TOOL_ANGULAR_SPEED_RAD_S
    max_angular_acceleration: float = DEFAULT_MAX_TOOL_ANGULAR_ACCEL_RAD_S2

    max_command_step_radians: float = DEFAULT_MAX_COMMAND_STEP_RAD
    max_ik_waypoint_jump_radians: float = 0.50
    # The official URDF is the generic fallback. Once a real arm has a
    # mechanical-stop calibration, that measured range becomes the pose-joint
    # authority, inset from each measured stop by this margin.
    calibrated_joint_stop_margin_rad: float = math.radians(1.0)
    calibrated_gripper_stop_margin_rad: float = math.radians(1.0)
    cartesian_waypoint_spacing_m: float = 0.005
    cartesian_waypoint_spacing_rad: float = 0.08
    cartesian_position_tolerance_m: float = 0.0005
    joint_position_tolerance_rad: float = 0.025
    following_error_limit_rad: float = 0.30
    unexpected_direction_threshold_rad: float = 0.05
    # Live streams may reverse faster than a physical servo can brake. Permit
    # only a short, bounded carry-through after a commanded reversal; ordinary
    # planned motion keeps the strict unexpected-direction rule.
    stream_reversal_grace_s: float = 0.10
    stream_reversal_decay_tolerance_rad: float = math.radians(0.5)
    trajectory_feedback_interval_s: float = 0.10
    feedback_poll_interval_s: float = 0.02
    settle_time_s: float = 0.08
    motion_completion_timeout_s: float = 5.0
    stop_timeout_s: float = 2.0
    max_command_lateness_s: float = 0.05

    # Current/load based contact and collision guard. Present_Load is a signed
    # magnitude value with a maximum magnitude of 1023. Present_Current is kept
    # in the STS3215 raw register units because that is what the servo reports.
    # These defaults are deliberately soft-stop guardrails, not calibrated force.
    effort_safety_enabled: bool = True
    effort_current_trip_raw: int | None = 250
    effort_load_trip_raw: int | None = 850
    effort_trip_consecutive_samples: int = 2
    motor_current_trip_raw: Mapping[str, int] = field(default_factory=dict)
    motor_load_trip_raw: Mapping[str, int] = field(default_factory=dict)

    # Coarse hobby-arm geometry envelope. This is intentionally conservative
    # and is not a substitute for measured collision geometry.
    enable_workspace_checks: bool = True
    # Live leader/follower streaming uses calibrated joint, step, rate,
    # acceleration, effort, and stale-sample guards by default. The coarse
    # floor/reach/self-clearance model is opt-in because it depends on a
    # robot-specific table frame and tool geometry calibration.
    teleop_workspace_checks: bool = False
    workspace_check_step_rad: float = 0.05
    minimum_workspace_z_m: float = 0.0
    maximum_tcp_reach_m: float = 0.50
    minimum_self_clearance_m: float = 0.025
    base_keepout_radius_m: float = 0.055
    base_keepout_height_m: float = 0.11

    auto_enable_torque: bool = False
    disable_torque_on_disconnect: bool = True
    allow_uncalibrated: bool = False
    verify_model_numbers: bool = True

    # Normal connections are read/configuration neutral. Use the explicit
    # ``soarm101 configure`` command for one-time recommended motor settings.
    configure_motors_on_connect: bool = False

    # Low-level arm position-write fallback. Keep the inner servo tracker more
    # responsive than the host trajectory so it follows host-planned motion
    # instead of becoming a slower competing trajectory generator.
    hardware_speed_raw: int = TELEOP_SERVO_SPEED_RAW
    hardware_acceleration_raw: int = TELEOP_SERVO_ACCELERATION_RAW
    # STS3215 factory position P gain. P=16 has been observed to leave a
    # several-degree static/gravity deadband on SO-family follower arms.
    position_p_coefficient: int = 32
    position_i_coefficient: int = 0
    position_d_coefficient: int = 32

    @classmethod
    def from_motion_limits(
        cls,
        *,
        max_joint_speed_deg_s: float = DEFAULT_MAX_JOINT_SPEED_DEG_S,
        max_joint_acceleration_deg_s2: float = DEFAULT_MAX_JOINT_ACCEL_DEG_S2,
        max_linear_speed_mm_s: float = DEFAULT_MAX_LINEAR_SPEED_MM_S,
        max_linear_acceleration_mm_s2: float = DEFAULT_MAX_LINEAR_ACCEL_MM_S2,
        max_tool_angular_speed_deg_s: float = DEFAULT_MAX_TOOL_ANGULAR_SPEED_DEG_S,
        max_tool_angular_acceleration_deg_s2: float = DEFAULT_MAX_TOOL_ANGULAR_ACCEL_DEG_S2,
        **kwargs: object,
    ) -> "SOARM101Config":
        """Build a config using human-friendly motion-envelope units."""

        return cls(
            **kwargs,
            max_joint_speed=math.radians(float(max_joint_speed_deg_s)),
            max_joint_acceleration=math.radians(float(max_joint_acceleration_deg_s2)),
            max_linear_speed=float(max_linear_speed_mm_s) / 1000.0,
            max_linear_acceleration=float(max_linear_acceleration_mm_s2) / 1000.0,
            max_angular_speed=math.radians(float(max_tool_angular_speed_deg_s)),
            max_angular_acceleration=math.radians(
                float(max_tool_angular_acceleration_deg_s2)
            ),
        )

    @property
    def motion_limits_human(self) -> dict[str, float]:
        """Return the configured absolute motion envelope in CLI-facing units."""

        return {
            "max_joint_speed_deg_s": math.degrees(self.max_joint_speed),
            "max_joint_acceleration_deg_s2": math.degrees(self.max_joint_acceleration),
            "max_linear_speed_mm_s": self.max_linear_speed * 1000.0,
            "max_linear_acceleration_mm_s2": self.max_linear_acceleration * 1000.0,
            "max_tool_angular_speed_deg_s": math.degrees(self.max_angular_speed),
            "max_tool_angular_acceleration_deg_s2": math.degrees(
                self.max_angular_acceleration
            ),
        }

    @property
    def stream_joint_speed_limit(self) -> float:
        return self.teleop_max_joint_speed or self.max_joint_speed

    @property
    def stream_joint_acceleration_limit(self) -> float:
        return self.teleop_max_joint_acceleration or self.max_joint_acceleration

    def __post_init__(self) -> None:
        positive = {
            "command_frequency_hz": self.command_frequency_hz,
            "default_joint_speed": self.default_joint_speed,
            "default_joint_acceleration": self.default_joint_acceleration,
            "default_linear_speed": self.default_linear_speed,
            "default_linear_acceleration": self.default_linear_acceleration,
            "default_angular_speed": self.default_angular_speed,
            "default_angular_acceleration": self.default_angular_acceleration,
            "max_joint_speed": self.max_joint_speed,
            "max_joint_acceleration": self.max_joint_acceleration,
            "max_linear_speed": self.max_linear_speed,
            "max_linear_acceleration": self.max_linear_acceleration,
            "max_angular_speed": self.max_angular_speed,
            "max_angular_acceleration": self.max_angular_acceleration,
            "max_command_step_radians": self.max_command_step_radians,
            "max_ik_waypoint_jump_radians": self.max_ik_waypoint_jump_radians,
            "calibrated_joint_stop_margin_rad": self.calibrated_joint_stop_margin_rad,
            "calibrated_gripper_stop_margin_rad": self.calibrated_gripper_stop_margin_rad,
            "cartesian_waypoint_spacing_m": self.cartesian_waypoint_spacing_m,
            "cartesian_waypoint_spacing_rad": self.cartesian_waypoint_spacing_rad,
            "cartesian_position_tolerance_m": self.cartesian_position_tolerance_m,
            "joint_position_tolerance_rad": self.joint_position_tolerance_rad,
            "following_error_limit_rad": self.following_error_limit_rad,
            "unexpected_direction_threshold_rad": self.unexpected_direction_threshold_rad,
            "stream_reversal_grace_s": self.stream_reversal_grace_s,
            "stream_reversal_decay_tolerance_rad": self.stream_reversal_decay_tolerance_rad,
            "trajectory_feedback_interval_s": self.trajectory_feedback_interval_s,
            "feedback_poll_interval_s": self.feedback_poll_interval_s,
            "settle_time_s": self.settle_time_s,
            "motion_completion_timeout_s": self.motion_completion_timeout_s,
            "stop_timeout_s": self.stop_timeout_s,
            "max_command_lateness_s": self.max_command_lateness_s,
            "workspace_check_step_rad": self.workspace_check_step_rad,
            "maximum_tcp_reach_m": self.maximum_tcp_reach_m,
            "minimum_self_clearance_m": self.minimum_self_clearance_m,
            "base_keepout_radius_m": self.base_keepout_radius_m,
            "base_keepout_height_m": self.base_keepout_height_m,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0:
                raise ConfigurationError(f"{name} must be a positive finite value")
        for name in ("teleop_max_joint_speed", "teleop_max_joint_acceleration"):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ConfigurationError(f"{name} must be a positive finite value")
        if not math.isfinite(self.minimum_workspace_z_m):
            raise ConfigurationError("minimum_workspace_z_m must be finite")
        ceilings = {
            "default_joint_speed": (self.default_joint_speed, self.max_joint_speed),
            "default_joint_acceleration": (
                self.default_joint_acceleration,
                self.max_joint_acceleration,
            ),
            "default_linear_speed": (self.default_linear_speed, self.max_linear_speed),
            "default_linear_acceleration": (
                self.default_linear_acceleration,
                self.max_linear_acceleration,
            ),
            "default_angular_speed": (self.default_angular_speed, self.max_angular_speed),
            "default_angular_acceleration": (
                self.default_angular_acceleration,
                self.max_angular_acceleration,
            ),
        }
        for name, (value, maximum) in ceilings.items():
            if value > maximum:
                raise ConfigurationError(f"{name} must not exceed its configured maximum {maximum}")

        if not 0 <= self.hardware_speed_raw <= STS3215_MAX_POSITION_SPEED_RAW:
            raise ConfigurationError(
                "hardware_speed_raw must be in "
                f"[0, {STS3215_MAX_POSITION_SPEED_RAW}]"
            )
        if not 0 <= self.hardware_acceleration_raw <= 254:
            raise ConfigurationError("hardware_acceleration_raw must be in [0, 254]")

        if self.effort_current_trip_raw is not None and self.effort_current_trip_raw <= 0:
            raise ConfigurationError("effort_current_trip_raw must be positive or None")
        if self.effort_load_trip_raw is not None and not 1 <= self.effort_load_trip_raw <= 1023:
            raise ConfigurationError("effort_load_trip_raw must be in [1, 1023] or None")
        if self.effort_trip_consecutive_samples < 1:
            raise ConfigurationError("effort_trip_consecutive_samples must be at least 1")

        current_overrides = dict(self.motor_current_trip_raw)
        load_overrides = dict(self.motor_load_trip_raw)
        for mapping_name, overrides, maximum in (
            ("motor_current_trip_raw", current_overrides, None),
            ("motor_load_trip_raw", load_overrides, 1023),
        ):
            unknown = set(overrides) - set(ALL_MOTORS)
            if unknown:
                raise ConfigurationError(
                    f"{mapping_name} contains unknown motors: {', '.join(sorted(unknown))}"
                )
            for motor, value in overrides.items():
                if value <= 0 or (maximum is not None and value > maximum):
                    suffix = "" if maximum is None else f" and <= {maximum}"
                    raise ConfigurationError(
                        f"{mapping_name}[{motor!r}] must be positive{suffix}"
                    )
        object.__setattr__(self, "motor_current_trip_raw", current_overrides)
        object.__setattr__(self, "motor_load_trip_raw", load_overrides)

        if self.calibration_path is not None:
            object.__setattr__(self, "calibration_path", Path(self.calibration_path).expanduser())
