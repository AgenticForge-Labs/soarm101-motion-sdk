"""Host-side smooth joint and Cartesian motion controller."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Generic, TypeVar

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import (
    ARM_JOINTS,
    DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
    JOINT_LIMITS,
    STOCK_GRIPPER,
    STS3215_MAX_POSITION_SPEED_RAW,
    TELEOP_SERVO_ACCELERATION_RAW,
    TELEOP_SERVO_SPEED_RAW,
)
from soarm101_motion.exceptions import (
    HardwareFaultError,
    IKError,
    InvalidCommandError,
    MotionCancelledError,
    MotionTimeoutError,
    RobotConnectionError,
    SafetyViolationError,
)
from soarm101_motion.hardware.base import SO101HardwareBackend
from soarm101_motion.kinematics import IKOptions, IKSolver, OrientationMode, SO101KinematicModel
from soarm101_motion.safety import (
    validate_command_step,
    validate_joint_targets,
    validate_workspace_path,
)
from soarm101_motion.trajectories import Trajectory
from soarm101_motion.types import MotionResult, Pose

T = TypeVar("T")


class MotionHandle(Generic[T]):
    """Cancelable result handle returned by all asynchronous operations."""

    def __init__(
        self,
        operation: Callable[[threading.Event], T],
        *,
        on_done: Callable[["MotionHandle[T]"], None] | None = None,
    ) -> None:
        self._cancel_event = threading.Event()
        self._done_event = threading.Event()
        self._result: T | None = None
        self._exception: BaseException | None = None
        self._on_done = on_done
        self._thread = threading.Thread(target=self._run, args=(operation,), daemon=True)
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._thread.start()

    def _run(self, operation: Callable[[threading.Event], T]) -> None:
        try:
            self._result = operation(self._cancel_event)
        except BaseException as exc:
            self._exception = exc
        finally:
            self._done_event.set()
            if self._on_done is not None:
                self._on_done(self)

    @property
    def done(self) -> bool:
        return self._done_event.is_set()

    @property
    def cancelled(self) -> bool:
        return self._cancel_event.is_set()

    def cancel(self) -> None:
        self._cancel_event.set()

    def wait(self, timeout: float | None = None) -> T:
        if not self._done_event.wait(timeout):
            raise TimeoutError("motion did not complete before timeout")
        if self._exception is not None:
            raise self._exception
        assert self._result is not None
        return self._result

    def result(self, timeout: float | None = None) -> T:
        return self.wait(timeout)

    def exception(self, timeout: float | None = None) -> BaseException | None:
        if not self._done_event.wait(timeout):
            raise TimeoutError("motion did not complete before timeout")
        return self._exception


@dataclass(frozen=True)
class PlannedPath:
    joint_waypoints: tuple[dict[str, float], ...]
    cartesian_waypoints: tuple[Pose, ...]
    command_samples: tuple[dict[str, float], ...]
    duration_s: float


@dataclass(frozen=True)
class RecordedPlan:
    command_samples: tuple[dict[str, float], ...]
    gripper_samples: tuple[float, ...]
    duration_s: float
    pre_roll: PlannedPath | None = None


@dataclass
class JointStreamState:
    last_command: dict[str, float]
    last_velocity: dict[str, float] | None
    previous_actual: dict[str, float]
    frequency_hz: float
    limits: dict[str, tuple[float, float]]
    tcp: Pose | None = None


class MotionController:
    def __init__(
        self,
        backend: SO101HardwareBackend,
        config: SOARM101Config,
        model: SO101KinematicModel | None = None,
    ) -> None:
        self.backend = backend
        self.config = config
        self.model = model or SO101KinematicModel()
        self.ik = IKSolver(self.model)
        self._state_lock = threading.RLock()
        self._active_handle: MotionHandle[MotionResult] | None = None
        self._joint_stream: JointStreamState | None = None

    @property
    def is_moving(self) -> bool:
        with self._state_lock:
            active_motion = self._active_handle is not None and not self._active_handle.done
            return active_motion or self._joint_stream is not None

    @property
    def is_streaming(self) -> bool:
        with self._state_lock:
            return self._joint_stream is not None

    def _clear_handle(self, handle: MotionHandle[MotionResult]) -> None:
        with self._state_lock:
            if self._active_handle is handle:
                self._active_handle = None

    def _ensure_idle_locked(self) -> None:
        if self._active_handle is not None and not self._active_handle.done:
            raise InvalidCommandError("another motion is already active")
        if self._joint_stream is not None:
            raise InvalidCommandError("joint streaming is active")

    def _require_ready(self) -> None:
        state = self.backend.get_hardware_state()
        if not state.connected:
            raise RobotConnectionError("robot is not connected")
        if state.faulted:
            raise HardwareFaultError(state.fault_message or "robot is faulted")
        if not state.torque_enabled:
            raise InvalidCommandError("torque is disabled; call enable() before motion")

    def _effective_limits(self) -> dict[str, tuple[float, float]]:
        limits = dict(JOINT_LIMITS)
        calibration = getattr(self.backend, "calibration", None)
        if calibration is None:
            return limits
        for name in ARM_JOINTS:
            motor = calibration.motors.get(name)
            if motor is None:
                continue
            calibrated_lower, calibrated_upper = motor.radians_limits
            model_lower, model_upper = limits[name]
            lower = max(model_lower, calibrated_lower)
            upper = min(model_upper, calibrated_upper)
            if lower >= upper:
                raise SafetyViolationError(
                    f"calibrated range for {name} does not overlap the model limits"
                )
            limits[name] = (lower, upper)
        return limits

    def _limits_for_present(self, present: Mapping[str, float]) -> dict[str, tuple[float, float]]:
        limits = self._effective_limits()
        for name in ARM_JOINTS:
            lower, upper = limits[name]
            limits[name] = (min(lower, present[name]), max(upper, present[name]))
        return limits

    @staticmethod
    def _minimum_duration(delta: float, speed: float, acceleration: float) -> float:
        if delta < 1e-12:
            return 0.0
        return max(1.875 * delta / speed, math.sqrt(5.774 * delta / acceleration))

    @staticmethod
    def _minimum_jerk(alpha: float) -> float:
        alpha = min(1.0, max(0.0, alpha))
        return 10 * alpha**3 - 15 * alpha**4 + 6 * alpha**5

    @staticmethod
    def _cosine_cruise_profile(
        delta: float,
        speed: float,
        acceleration: float,
    ) -> tuple[float, float, float]:
        """Return peak speed, ramp time, and cruise time for a smooth S-curve.

        Velocity uses half-cosine acceleration/deceleration ramps with zero
        acceleration at the transitions to and from constant-speed cruise. The
        requested speed and acceleration are true ceilings rather than minimum-jerk
        peak values.
        """

        if delta < 1e-12:
            return 0.0, 0.0, 0.0
        requested_speed = max(float(speed), 1e-12)
        requested_acceleration = max(float(acceleration), 1e-12)
        full_ramp_time = math.pi * requested_speed / (2.0 * requested_acceleration)
        full_ramp_distance = requested_speed * full_ramp_time
        if delta >= full_ramp_distance:
            cruise_time = (delta - full_ramp_distance) / requested_speed
            return requested_speed, full_ramp_time, cruise_time

        peak_speed = math.sqrt(2.0 * requested_acceleration * delta / math.pi)
        ramp_time = math.pi * peak_speed / (2.0 * requested_acceleration)
        return peak_speed, ramp_time, 0.0

    @classmethod
    def _cosine_cruise_duration(
        cls,
        delta: float,
        speed: float,
        acceleration: float,
    ) -> float:
        peak_speed, ramp_time, cruise_time = cls._cosine_cruise_profile(
            delta,
            speed,
            acceleration,
        )
        if peak_speed <= 0.0:
            return 0.0
        return 2.0 * ramp_time + cruise_time

    @classmethod
    def _cosine_cruise_progress(
        cls,
        delta: float,
        speed: float,
        acceleration: float,
        elapsed: float,
    ) -> float:
        if delta < 1e-12:
            return 1.0
        peak_speed, ramp_time, cruise_time = cls._cosine_cruise_profile(
            delta,
            speed,
            acceleration,
        )
        total = 2.0 * ramp_time + cruise_time
        if elapsed <= 0.0:
            return 0.0
        if elapsed >= total:
            return 1.0

        ramp_distance = peak_speed * ramp_time / 2.0
        if elapsed < ramp_time:
            position = 0.5 * peak_speed * (
                elapsed
                - ramp_time / math.pi * math.sin(math.pi * elapsed / ramp_time)
            )
        elif elapsed <= ramp_time + cruise_time:
            position = ramp_distance + peak_speed * (elapsed - ramp_time)
        else:
            tau = elapsed - ramp_time - cruise_time
            position = (
                ramp_distance
                + peak_speed * cruise_time
                + 0.5
                * peak_speed
                * (
                    tau
                    + ramp_time / math.pi * math.sin(math.pi * tau / ramp_time)
                )
            )
        return min(1.0, max(0.0, position / delta))

    @staticmethod
    def _positive(value: float, name: str) -> float:
        value = float(value)
        if not math.isfinite(value) or value <= 0:
            raise InvalidCommandError(f"{name} must be a positive finite value")
        return value

    def _bounded(self, value: float | None, default: float, maximum: float, name: str) -> float:
        resolved = self._positive(default if value is None else value, name)
        if resolved > maximum:
            raise SafetyViolationError(
                f"{name} {resolved:.4f} exceeds configured safety maximum {maximum:.4f}"
            )
        return resolved

    def _sleep_until(self, deadline: float) -> float:
        if not getattr(self.backend, "realtime", True):
            return 0.0
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        return max(0.0, time.perf_counter() - deadline)

    def _trajectory_metrics(
        self,
        samples: Sequence[Mapping[str, float]],
    ) -> tuple[float, float, float]:
        if len(samples) < 2:
            return 0.0, 0.0, 0.0
        matrix = np.array([[sample[name] for name in ARM_JOINTS] for sample in samples])
        dt = 1.0 / self.config.command_frequency_hz
        velocity = np.diff(matrix, axis=0) / dt
        acceleration = np.diff(velocity, axis=0) / dt if len(velocity) > 1 else np.zeros((0, 5))
        max_speed = float(np.max(np.abs(velocity))) if velocity.size else 0.0
        max_acceleration = float(np.max(np.abs(acceleration))) if acceleration.size else 0.0
        max_step = float(np.max(np.abs(np.diff(matrix, axis=0)))) if len(matrix) > 1 else 0.0
        return max_speed, max_acceleration, max_step

    @staticmethod
    def _joint_path_roughness(samples: Sequence[Mapping[str, float]]) -> float:
        """Return RMS discrete joint jerk for comparing equivalent IK paths."""

        if len(samples) < 4:
            return 0.0
        matrix = np.array([[sample[name] for name in ARM_JOINTS] for sample in samples])
        jerk = np.diff(matrix, n=3, axis=0)
        return float(np.sqrt(np.mean(jerk**2))) if jerk.size else 0.0

    def _smooth_position_only_cartesian_samples(
        self,
        samples: tuple[dict[str, float], ...],
        cartesian: tuple[Pose, ...],
        *,
        tcp: Pose | None,
        limits: Mapping[str, tuple[float, float]],
    ) -> tuple[dict[str, float], ...]:
        """Reproject a smoothed redundant IK path back onto the same Cartesian samples.

        Position-only IK leaves tool orientation free, so sequential numerical solves can
        wander slightly in redundant joint directions even when the Cartesian samples are
        smooth. Use a five-tap binomial filter only to create better IK seeds, then solve
        every interior Cartesian sample again at the unchanged hard position tolerance.
        The Cartesian path therefore remains authoritative. The refined path is accepted
        only when it is continuous and has lower discrete joint jerk.
        """

        if len(samples) < 5 or len(samples) != len(cartesian):
            return samples

        matrix = np.array([[sample[name] for name in ARM_JOINTS] for sample in samples])
        padded = np.pad(matrix, ((2, 2), (0, 0)), mode="edge")
        kernel = np.array([1.0, 4.0, 6.0, 4.0, 1.0]) / 16.0
        smooth_matrix = np.vstack(
            [
                np.sum(padded[index : index + 5] * kernel[:, None], axis=0)
                for index in range(len(matrix))
            ]
        )

        refined: list[dict[str, float]] = [dict(samples[0])]
        for index in range(1, len(samples) - 1):
            smooth_seed = {
                name: float(smooth_matrix[index, joint_index])
                for joint_index, name in enumerate(ARM_JOINTS)
            }
            try:
                solution = self.ik.solve_or_raise(
                    cartesian[index],
                    seed=smooth_seed,
                    tcp=tcp,
                    options=IKOptions(
                        orientation_mode="position_only",
                        position_tolerance_m=self.config.cartesian_position_tolerance_m,
                        multi_start=False,
                    ),
                )
            except IKError:
                return samples
            candidate = validate_joint_targets(solution.joints, limits=limits)
            jump = max(
                abs(candidate[name] - refined[-1][name])
                for name in ARM_JOINTS
            )
            if jump > self.config.max_ik_waypoint_jump_radians:
                return samples
            refined.append(candidate)

        final_sample = dict(samples[-1])
        final_jump = max(
            abs(final_sample[name] - refined[-1][name])
            for name in ARM_JOINTS
        )
        if final_jump > self.config.max_ik_waypoint_jump_radians:
            return samples
        refined.append(final_sample)
        refined_tuple = tuple(refined)

        if self._joint_path_roughness(refined_tuple) < self._joint_path_roughness(samples):
            return refined_tuple
        return samples

    def _validate_samples(
        self,
        samples: Sequence[Mapping[str, float]],
        *,
        speed_limit: float,
        acceleration_limit: float,
        limits: Mapping[str, tuple[float, float]] | None = None,
    ) -> None:
        limits = limits or self._effective_limits()
        for sample in samples:
            validate_joint_targets(sample, limits=limits)
        for previous, command in zip(samples, samples[1:]):
            validate_command_step(previous, command, self.config.max_command_step_radians)
        max_speed, max_acceleration, _ = self._trajectory_metrics(samples)
        if max_speed > speed_limit * 1.001:
            raise SafetyViolationError(
                f"planned joint speed {max_speed:.4f} rad/s exceeds {speed_limit:.4f} rad/s"
            )
        if max_acceleration > acceleration_limit * 1.001:
            raise SafetyViolationError(
                f"planned joint acceleration {max_acceleration:.4f} rad/s² exceeds "
                f"{acceleration_limit:.4f} rad/s²"
            )

    def _retime(
        self,
        builder: Callable[[float], tuple[dict[str, float], ...]],
        initial_duration: float,
        *,
        speed_limit: float,
        acceleration_limit: float,
        limits: Mapping[str, tuple[float, float]] | None = None,
    ) -> tuple[tuple[dict[str, float], ...], float]:
        duration = max(initial_duration, 1.0 / self.config.command_frequency_hz)
        for _ in range(12):
            samples = builder(duration)
            max_speed, max_acceleration, max_step = self._trajectory_metrics(samples)
            scale = max(
                1.0,
                max_speed / speed_limit if speed_limit else 1.0,
                math.sqrt(max_acceleration / acceleration_limit)
                if max_acceleration and acceleration_limit
                else 1.0,
                max_step / self.config.max_command_step_radians if max_step else 1.0,
            )
            if scale <= 1.001:
                self._validate_samples(
            samples,
            speed_limit=speed_limit,
            acceleration_limit=acceleration_limit,
            limits=limits,
                )
                actual_duration = (len(samples) - 1) / self.config.command_frequency_hz
                return samples, actual_duration
            duration *= scale * 1.05
        raise SafetyViolationError("could not time-parameterize trajectory within configured limits")

    def _plan_joint_motion(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        *,
        speed: float,
        acceleration: float,
        limits: Mapping[str, tuple[float, float]] | None = None,
    ) -> PlannedPath:
        max_delta = max(abs(target[name] - start[name]) for name in ARM_JOINTS)
        initial = self._minimum_duration(max_delta, speed, acceleration)

        def builder(duration: float) -> tuple[dict[str, float], ...]:
            steps = max(2, int(math.ceil(duration * self.config.command_frequency_hz)) + 1)
            return tuple(
                {
                    name: start[name]
                    + (target[name] - start[name])
                    * self._minimum_jerk(index / (steps - 1))
                    for name in ARM_JOINTS
                }
                for index in range(steps)
            )

        samples, duration = self._retime(
            builder,
            initial,
            speed_limit=speed,
            acceleration_limit=acceleration,
            limits=limits,
        )
        return PlannedPath((dict(start), dict(target)), (), samples, duration)

    def _solve_cartesian_command_samples(
        self,
        *,
        start_joints: Mapping[str, float],
        start_pose: Pose,
        target: Pose,
        tcp: Pose | None,
        orientation_mode: OrientationMode,
        look_at: np.ndarray | None,
        duration: float,
        profile_duration: float,
        normalized_speed: float,
        normalized_acceleration: float,
        limits: Mapping[str, tuple[float, float]],
        target_seed: Mapping[str, float] | None = None,
    ) -> tuple[tuple[dict[str, float], ...], tuple[Pose, ...]]:
        """Solve the smooth Cartesian trajectory directly at command-rate samples.

        The scalar path uses symmetric half-cosine acceleration/deceleration ramps with
        an optional constant-speed cruise. Every command sample gets its own sequential
        IK solution; there is no secondary piecewise-linear interpolation between sparse
        joint-space knots.

        Position-only paths normally solve from the measured start toward the target. If
        that forward continuation hits a numerical IK pocket, retry from the reachable
        target endpoint and solve the same Cartesian samples backward. A caller-provided
        target_seed is only a boundary-condition hint; every sample still has to pass
        the unchanged hard Cartesian tolerance, joint limits, and continuity checks.
        """

        steps = max(2, int(math.ceil(duration * self.config.command_frequency_hz)) + 1)
        fractions = np.linspace(0.0, 1.0, steps)
        slerp = Slerp(
            [0.0, 1.0],
            Rotation.from_matrix(np.stack([start_pose.rotation, target.rotation])),
        )
        cartesian_samples: list[Pose] = [start_pose]
        for time_fraction in fractions[1:]:
            progress = self._cosine_cruise_progress(
                1.0,
                normalized_speed,
                normalized_acceleration,
                float(time_fraction) * profile_duration,
            )
            pose_rotation = (
                start_pose.rotation
                if orientation_mode == "position_only"
                else slerp([progress]).as_matrix()[0]
            )
            cartesian_samples.append(
                Pose(
                    start_pose.position + (target.position - start_pose.position) * progress,
                    pose_rotation,
                )
            )
        solved_cartesian = tuple(cartesian_samples)

        def solve_sample(
            pose: Pose,
            *,
            seed: Mapping[str, float],
            multi_start: bool,
            enforce_seed_jump: bool = True,
        ) -> dict[str, float]:
            options = IKOptions(
                orientation_mode=orientation_mode,
                look_at=look_at,
                position_tolerance_m=self.config.cartesian_position_tolerance_m,
                multi_start=multi_start,
            )
            try:
                solution = self.ik.solve_or_raise(
                    pose,
                    seed=seed,
                    tcp=tcp,
                    options=options,
                )
            except IKError:
                if multi_start:
                    raise
                solution = self.ik.solve_or_raise(
                    pose,
                    seed=seed,
                    tcp=tcp,
                    options=IKOptions(
                        orientation_mode=orientation_mode,
                        look_at=look_at,
                        position_tolerance_m=self.config.cartesian_position_tolerance_m,
                        multi_start=True,
                    ),
                )
            candidate = validate_joint_targets(solution.joints, limits=limits)
            if enforce_seed_jump:
                jump = max(abs(candidate[name] - seed[name]) for name in ARM_JOINTS)
                if jump > self.config.max_ik_waypoint_jump_radians:
                    raise InvalidCommandError(
                        f"IK path discontinuity of {jump:.3f} rad exceeds "
                        f"{self.config.max_ik_waypoint_jump_radians:.3f} rad"
                    )
            return candidate

        def solve_forward() -> tuple[dict[str, float], ...]:
            joint_samples: list[dict[str, float]] = [dict(start_joints)]
            seed = dict(start_joints)
            for index, pose in enumerate(solved_cartesian[1:], start=1):
                candidate = solve_sample(
                    pose,
                    seed=seed,
                    multi_start=index == 1,
                )
                seed = candidate
                joint_samples.append(candidate)
            return tuple(joint_samples)

        try:
            solved_samples = solve_forward()
        except IKError as forward_error:
            if orientation_mode != "position_only":
                raise

            boundary_seed: Mapping[str, float] = start_joints
            if target_seed is not None:
                boundary_seed = validate_joint_targets(target_seed, limits=limits)

            try:
                endpoint = solve_sample(
                    solved_cartesian[-1],
                    seed=boundary_seed,
                    multi_start=True,
                    # A boundary seed is a target-side solver hint, not the previous
                    # command sample. Adjacent continuity is checked while walking
                    # backward and again when reconnecting to the measured start.
                    enforce_seed_jump=False,
                )
                reverse_samples: list[dict[str, float]] = [endpoint]
                seed = endpoint
                for pose in reversed(solved_cartesian[1:-1]):
                    candidate = solve_sample(
                        pose,
                        seed=seed,
                        multi_start=False,
                    )
                    seed = candidate
                    reverse_samples.append(candidate)

                ordered_tail = list(reversed(reverse_samples))
                first_jump = max(
                    abs(ordered_tail[0][name] - start_joints[name])
                    for name in ARM_JOINTS
                )
                if first_jump > self.config.max_ik_waypoint_jump_radians:
                    raise InvalidCommandError(
                        f"reverse IK path cannot connect to measured start: "
                        f"{first_jump:.3f} rad exceeds "
                        f"{self.config.max_ik_waypoint_jump_radians:.3f} rad"
                    )
                solved_samples = (dict(start_joints), *ordered_tail)
            except (IKError, InvalidCommandError) as reverse_error:
                raise IKError(
                    "forward Cartesian IK failed and endpoint-seeded reverse fallback "
                    f"also failed; forward={forward_error}; reverse={reverse_error}"
                ) from reverse_error

        if orientation_mode == "position_only":
            solved_samples = self._smooth_position_only_cartesian_samples(
                solved_samples,
                solved_cartesian,
                tcp=tcp,
                limits=limits,
            )
        return solved_samples, solved_cartesian

    def plan_linear(
        self,
        target: Pose,
        *,
        tcp: Pose | None = None,
        orientation_mode: OrientationMode = "compatible",
        look_at: np.ndarray | None = None,
        speed: float | None = None,
        acceleration: float | None = None,
        target_seed: Mapping[str, float] | None = None,
    ) -> PlannedPath:
        self._require_ready()
        linear_speed = self._bounded(
            speed,
            self.config.default_linear_speed,
            self.config.max_linear_speed,
            "linear speed",
        )
        linear_acceleration = self._bounded(
            acceleration,
            self.config.default_linear_acceleration,
            self.config.max_linear_acceleration,
            "linear acceleration",
        )
        start_joints = self.backend.read_joint_positions()
        start_pose = self.model.forward(start_joints, tcp=tcp)
        distance = float(np.linalg.norm(target.position - start_pose.position))
        angular_distance = (
            0.0
            if orientation_mode == "position_only"
            else float(
                np.linalg.norm(
                    Rotation.from_matrix(
                        target.rotation @ start_pose.rotation.T
                    ).as_rotvec()
                )
            )
        )

        normalized_speed_limits: list[float] = []
        normalized_acceleration_limits: list[float] = []
        if distance > 1e-12:
            normalized_speed_limits.append(linear_speed / distance)
            normalized_acceleration_limits.append(linear_acceleration / distance)
        if angular_distance > 1e-12:
            normalized_speed_limits.append(
                self.config.default_angular_speed / angular_distance
            )
            normalized_acceleration_limits.append(
                self.config.default_angular_acceleration / angular_distance
            )
        if normalized_speed_limits:
            normalized_speed = min(normalized_speed_limits)
            normalized_acceleration = min(normalized_acceleration_limits)
            profile_duration = self._cosine_cruise_duration(
                1.0,
                normalized_speed,
                normalized_acceleration,
            )
        else:
            normalized_speed = 1.0
            normalized_acceleration = 1.0
            profile_duration = 0.0

        # Preserve the configured Cartesian planning density as a lower bound on
        # duration/sample count, while command-rate IK remains the actual trajectory.
        minimum_segments = max(
            1,
            int(
                math.ceil(
                    max(
                        distance / self.config.cartesian_waypoint_spacing_m,
                        angular_distance / self.config.cartesian_waypoint_spacing_rad,
                    )
                )
            ),
        )
        density_duration = minimum_segments / self.config.command_frequency_hz
        duration = max(
            profile_duration,
            density_duration,
            1.0 / self.config.command_frequency_hz,
        )
        limits = self._limits_for_present(start_joints)

        samples: tuple[dict[str, float], ...] = ()
        cartesian: tuple[Pose, ...] = ()
        for _ in range(12):
            samples, cartesian = self._solve_cartesian_command_samples(
                start_joints=start_joints,
                start_pose=start_pose,
                target=target,
                tcp=tcp,
                orientation_mode=orientation_mode,
                look_at=look_at,
                duration=duration,
                profile_duration=profile_duration,
                normalized_speed=normalized_speed,
                normalized_acceleration=normalized_acceleration,
                limits=limits,
                target_seed=target_seed,
            )
            max_speed, max_acceleration, max_step = self._trajectory_metrics(samples)
            scale = max(
                1.0,
                max_speed / self.config.default_joint_speed
                if self.config.default_joint_speed
                else 1.0,
                math.sqrt(max_acceleration / self.config.default_joint_acceleration)
                if max_acceleration and self.config.default_joint_acceleration
                else 1.0,
                max_step / self.config.max_command_step_radians if max_step else 1.0,
            )
            if scale <= 1.001:
                self._validate_samples(
                    samples,
                    speed_limit=self.config.default_joint_speed,
                    acceleration_limit=self.config.default_joint_acceleration,
                    limits=limits,
                )
                actual_duration = (len(samples) - 1) / self.config.command_frequency_hz
                return PlannedPath(
                    tuple(dict(sample) for sample in samples),
                    cartesian,
                    samples,
                    actual_duration,
                )
            duration *= scale * 1.05

        raise SafetyViolationError(
            "could not time-parameterize Cartesian trajectory within configured limits"
        )

    def _synchronized_servo_speed_raw(
        self,
        previous: Mapping[str, float],
        target: Mapping[str, float],
        *,
        interval_s: float,
    ) -> int | dict[str, int]:
        """Choose per-joint STS3215 speed limits for synchronized sample arrival.

        On calibrated hardware, convert each planned joint increment to encoder ticks
        and choose a speed that reaches the next sample in about 80% of the command
        interval. This leaves headroom for the servo acceleration ramp while preventing
        lightly loaded joints from racing ahead of gravity-loaded joints. Simulation
        and backends without motor calibration retain the ordinary unrestricted profile.
        """

        calibration = getattr(self.backend, "calibration", None)
        if calibration is None:
            return TELEOP_SERVO_SPEED_RAW

        arrival_time = max(float(interval_s) * 0.80, 1e-4)
        speeds: dict[str, int] = {}
        for name in ARM_JOINTS:
            motor = calibration.motors.get(name)
            if motor is None:
                return TELEOP_SERVO_SPEED_RAW
            previous_raw = int(motor.radians_to_raw(float(previous[name])))
            target_raw = int(motor.radians_to_raw(float(target[name])))
            raw_delta = abs(target_raw - previous_raw)
            if raw_delta == 0:
                speeds[name] = 1
                continue
            required = int(math.ceil(raw_delta / arrival_time))
            speeds[name] = min(
                STS3215_MAX_POSITION_SPEED_RAW,
                max(1, required),
            )
        return speeds

    def _workspace_kwargs(self) -> dict[str, float]:
        return {
            "minimum_z_m": self.config.minimum_workspace_z_m,
            "maximum_tcp_reach_m": self.config.maximum_tcp_reach_m,
            "minimum_self_clearance_m": self.config.minimum_self_clearance_m,
            "base_keepout_radius_m": self.config.base_keepout_radius_m,
            "base_keepout_height_m": self.config.base_keepout_height_m,
        }

    def plan_recorded_trajectory(
        self,
        trajectory: Trajectory,
        *,
        speed_scale: float = 1.0,
        move_to_start: bool = True,
        tcp: Pose | None = None,
    ) -> RecordedPlan:
        """Validate, retime, and resample a recorded demonstration for execution."""
        self._require_ready()
        scale = self._positive(speed_scale, "trajectory speed scale")
        prepared = trajectory.retime(scale).resample(self.config.command_frequency_hz)
        samples = prepared.joint_mappings()
        gripper = tuple(float(value) for value in prepared.gripper)
        if any(value < 0.0 or value > 1.0 for value in gripper):
            raise InvalidCommandError("recorded gripper values must stay within [0, 1]")

        self._validate_samples(
            samples,
            speed_limit=self.config.max_joint_speed,
            acceleration_limit=self.config.max_joint_acceleration,
        )
        if self.config.enable_workspace_checks:
            validate_workspace_path(
                self.model,
                samples,
                tcp=tcp,
                **self._workspace_kwargs(),
            )

        present = self.backend.read_joint_positions()
        pre_roll: PlannedPath | None = None
        if move_to_start:
            pre_roll = self._plan_joint_motion(
                present,
                samples[0],
                speed=self.config.default_joint_speed,
                acceleration=self.config.default_joint_acceleration,
            )
            if self.config.enable_workspace_checks:
                validate_workspace_path(
                    self.model,
                    pre_roll.command_samples,
                    tcp=tcp,
                    **self._workspace_kwargs(),
                )
        else:
            start_error = max(
                abs(float(present[name]) - float(samples[0][name]))
                for name in ARM_JOINTS
            )
            if start_error > self.config.joint_position_tolerance_rad:
                raise InvalidCommandError(
                    "robot is not at the recorded start pose; enable move_to_start "
                    "or move to the first pose before replay"
                )

        return RecordedPlan(
            command_samples=samples,
            gripper_samples=gripper,
            duration_s=prepared.duration_s,
            pre_roll=pre_roll,
        )

    def _check_cancelled(self, cancel_event: threading.Event, message: str) -> None:
        if cancel_event.is_set():
            raise MotionCancelledError(message)

    def _monitor_motion(
        self,
        command: Mapping[str, float],
        previous_command: Mapping[str, float],
        previous_actual: Mapping[str, float],
    ) -> dict[str, float]:
        state = self.backend.get_hardware_state()
        if state.faulted:
            raise HardwareFaultError(state.fault_message or "robot faulted during motion")
        actual = self.backend.read_joint_positions()
        worst_joint = max(ARM_JOINTS, key=lambda name: abs(actual[name] - command[name]))
        following_error = abs(actual[worst_joint] - command[worst_joint])
        if following_error > self.config.following_error_limit_rad:
            raise SafetyViolationError(
                f"{worst_joint} following error {following_error:.3f} rad exceeds "
                f"{self.config.following_error_limit_rad:.3f} rad "
                f"(target {command[worst_joint]:.3f}, measured {actual[worst_joint]:.3f})"
            )
        for name in ARM_JOINTS:
            command_delta = command[name] - previous_command[name]
            actual_delta = actual[name] - previous_actual[name]
            if (
                abs(command_delta) >= self.config.joint_position_tolerance_rad
                and abs(actual_delta) >= self.config.unexpected_direction_threshold_rad
                and command_delta * actual_delta < 0.0
            ):
                raise SafetyViolationError(
                    f"{name} moved {actual_delta:+.3f} rad opposite the commanded direction"
                )
        return actual

    def _wait_for_settle(
        self,
        target: Mapping[str, float],
        cancel_event: threading.Event,
    ) -> MotionResult:
        deadline = time.monotonic() + self.config.motion_completion_timeout_s
        stable_since: float | None = None
        while True:
            self._check_cancelled(cancel_event, "motion cancelled while settling")
            actual = self.backend.read_joint_positions()
            error = max(abs(actual[name] - target[name]) for name in ARM_JOINTS)
            state = self.backend.get_hardware_state()
            if state.faulted:
                raise HardwareFaultError(state.fault_message or "robot faulted during motion")
            now = time.monotonic()
            # HardwareState.moving includes the tool actuator. A gripper that is
            # still moving must not prevent a five-joint path from completing.
            if error <= self.config.joint_position_tolerance_rad:
                if not getattr(self.backend, "realtime", True):
                    return MotionResult(True, True, final_positions=actual)
                stable_since = stable_since or now
                if now - stable_since >= self.config.settle_time_s:
                    return MotionResult(True, True, final_positions=actual)
            else:
                stable_since = None
            if now >= deadline:
                errors = {
                    name: float(target[name] - actual[name])
                    for name in ARM_JOINTS
                }
                worst_joint = max(ARM_JOINTS, key=lambda name: abs(errors[name]))
                detail = ", ".join(
                    f"{name}={errors[name]:+.4f}"
                    for name in ARM_JOINTS
                )
                diagnostic_detail = ""
                try:
                    diagnostics = {
                        item.name: item
                        for item in self.backend.diagnostics()
                        if item.name in ARM_JOINTS
                    }
                    parts = []
                    for name in ARM_JOINTS:
                        item = diagnostics.get(name)
                        if item is None:
                            continue
                        parts.append(
                            f"{name}[V={item.voltage_v},I={item.current_raw},"
                            f"moving={item.moving},status={item.status}]"
                        )
                    if parts:
                        diagnostic_detail = "; diagnostics: " + ", ".join(parts)
                except Exception:
                    diagnostic_detail = ""
                raise MotionTimeoutError(
                    f"motion did not settle within {self.config.motion_completion_timeout_s:.2f}s; "
                    f"worst={worst_joint} error={abs(errors[worst_joint]):.4f} rad; "
                    f"joint errors(target-measured): {detail}{diagnostic_detail}"
                )
            time.sleep(self.config.feedback_poll_interval_s)

    def _execute_plan(
        self,
        plan: PlannedPath,
        cancel_event: threading.Event,
        *,
        cancellation_message: str,
        servo_speed_raw: int | Mapping[str, int] | None = None,
        servo_acceleration_raw: int | None = None,
        synchronize_servo_arrival: bool = False,
    ) -> MotionResult:
        samples = plan.command_samples
        try:
            if len(samples) <= 1:
                return self._wait_for_settle(samples[-1], cancel_event)
            frequency = self.config.command_frequency_hz
            interval_s = 1.0 / frequency
            started = time.perf_counter()
            monitor_every = max(
                1,
                int(math.ceil(self.config.trajectory_feedback_interval_s * frequency)),
            )
            previous_command = samples[0]
            previous_actual = self.backend.read_joint_positions()
            last_command_sent = samples[0]
            for index, command in enumerate(samples[1:], start=1):
                self._check_cancelled(cancel_event, cancellation_message)
                lateness = self._sleep_until(started + index / frequency)
                if lateness > self.config.max_command_lateness_s:
                    raise MotionTimeoutError(
                        f"motion command deadline missed by {lateness:.3f}s"
                    )
                self._check_cancelled(cancel_event, cancellation_message)
                command_speed_raw = servo_speed_raw
                if synchronize_servo_arrival:
                    command_speed_raw = self._synchronized_servo_speed_raw(
                        last_command_sent,
                        command,
                        interval_s=interval_s,
                    )
                if command_speed_raw is None and servo_acceleration_raw is None:
                    self.backend.write_joint_positions(command)
                else:
                    self.backend.write_joint_positions(
                        command,
                        speed_raw=command_speed_raw,
                        acceleration_raw=servo_acceleration_raw,
                    )
                last_command_sent = command
                if index % monitor_every == 0 or index == len(samples) - 1:
                    previous_actual = self._monitor_motion(
                        command,
                        previous_command,
                        previous_actual,
                    )
                    previous_command = command
            return self._wait_for_settle(samples[-1], cancel_event)
        except BaseException:
            try:
                self.backend.stop()
            except Exception:
                pass
            raise

    def _execute_recorded(
        self,
        plan: RecordedPlan,
        cancel_event: threading.Event,
        *,
        gripper_speed_raw: int | None = None,
    ) -> MotionResult:
        samples = plan.command_samples
        gripper_samples = plan.gripper_samples
        try:
            self._check_cancelled(cancel_event, "recorded trajectory cancelled")
            self.backend.write_tool_position(
                STOCK_GRIPPER,
                gripper_samples[0],
                speed_raw=gripper_speed_raw,
            )
            if plan.pre_roll is not None:
                self._execute_plan(
                    plan.pre_roll,
                    cancel_event,
                    cancellation_message="recorded trajectory pre-roll cancelled",
                )

            frequency = self.config.command_frequency_hz
            started = time.perf_counter()
            monitor_every = max(
                1,
                int(math.ceil(self.config.trajectory_feedback_interval_s * frequency)),
            )
            previous_command = samples[0]
            previous_actual = self.backend.read_joint_positions()
            for index, command in enumerate(samples[1:], start=1):
                self._check_cancelled(cancel_event, "recorded trajectory cancelled")
                lateness = self._sleep_until(started + index / frequency)
                if lateness > self.config.max_command_lateness_s:
                    raise MotionTimeoutError(
                        f"recorded trajectory command deadline missed by {lateness:.3f}s"
                    )
                self._check_cancelled(cancel_event, "recorded trajectory cancelled")
                self.backend.write_joint_positions(command)
                self.backend.write_tool_position(
                    STOCK_GRIPPER,
                    gripper_samples[index],
                    speed_raw=gripper_speed_raw,
                )
                if index % monitor_every == 0 or index == len(samples) - 1:
                    previous_actual = self._monitor_motion(
                        command,
                        previous_command,
                        previous_actual,
                    )
                    previous_command = command

            settled = self._wait_for_settle(samples[-1], cancel_event)
            actual_gripper = self.backend.read_tool_position(STOCK_GRIPPER)
            if abs(actual_gripper - gripper_samples[-1]) > 0.05:
                raise MotionTimeoutError(
                    "recorded trajectory joints settled but gripper did not reach "
                    f"{gripper_samples[-1]:.3f}; actual is {actual_gripper:.3f}"
                )
            return MotionResult(
                settled.accepted,
                settled.completed,
                settled.message,
                {**settled.final_positions, STOCK_GRIPPER: actual_gripper},
            )
        except BaseException:
            try:
                self.backend.stop()
            except Exception:
                pass
            raise

    def _start_locked(
        self,
        operation: Callable[[threading.Event], MotionResult],
    ) -> MotionHandle[MotionResult]:
        handle = MotionHandle(operation, on_done=self._clear_handle)
        self._active_handle = handle
        handle.start()
        return handle

    def move_joints(
        self,
        positions: Mapping[str, float] | Sequence[float],
        *,
        speed: float | None = None,
        acceleration: float | None = None,
        relative: bool = False,
        wait: bool = True,
        servo_speed_raw: int | None = None,
        servo_acceleration_raw: int | None = None,
        synchronize_servo_arrival: bool = False,
    ) -> MotionResult | MotionHandle[MotionResult]:
        if synchronize_servo_arrival and servo_speed_raw is not None:
            raise InvalidCommandError(
                "servo_speed_raw cannot be combined with synchronize_servo_arrival"
            )
        if servo_speed_raw is not None and (
            not isinstance(servo_speed_raw, int) or not 0 <= servo_speed_raw <= 32767
        ):
            raise InvalidCommandError("servo speed must be an integer within [0, 32767]")
        if servo_acceleration_raw is not None and (
            not isinstance(servo_acceleration_raw, int)
            or not 0 <= servo_acceleration_raw <= 254
        ):
            raise InvalidCommandError("servo acceleration must be an integer within [0, 254]")
        with self._state_lock:
            self._ensure_idle_locked()
            self._require_ready()
            present = self.backend.read_joint_positions()
            limits = self._limits_for_present(present)
            if isinstance(positions, Mapping):
                provided = validate_joint_targets(positions, limits=limits)
            else:
                values = tuple(float(value) for value in positions)
                if len(values) != len(ARM_JOINTS):
                    raise InvalidCommandError(f"expected {len(ARM_JOINTS)} joint values")
                provided = validate_joint_targets(
                    dict(zip(ARM_JOINTS, values, strict=True)),
                    limits=limits,
                )
            target = dict(present)
            for name, value in provided.items():
                target[name] = present[name] + value if relative else value
            target = validate_joint_targets(target, limits=limits)
            joint_speed = self._bounded(
                speed,
                self.config.default_joint_speed,
                self.config.max_joint_speed,
                "joint speed",
            )
            joint_acceleration = self._bounded(
                acceleration,
                self.config.default_joint_acceleration,
                self.config.max_joint_acceleration,
                "joint acceleration",
            )
            plan = self._plan_joint_motion(
                present,
                target,
                speed=joint_speed,
                acceleration=joint_acceleration,
                limits=limits,
            )
            handle = self._start_locked(
                lambda event: self._execute_plan(
                    plan,
                    event,
                    cancellation_message="joint motion cancelled",
                    servo_speed_raw=servo_speed_raw,
                    servo_acceleration_raw=servo_acceleration_raw,
                    synchronize_servo_arrival=synchronize_servo_arrival,
                )
            )
        return handle.wait() if wait else handle

    def move_linear(
        self,
        target: Pose,
        *,
        tcp: Pose | None = None,
        orientation_mode: OrientationMode = "compatible",
        look_at: np.ndarray | None = None,
        speed: float | None = None,
        acceleration: float | None = None,
        wait: bool = True,
        target_seed: Mapping[str, float] | None = None,
    ) -> MotionResult | MotionHandle[MotionResult]:
        with self._state_lock:
            self._ensure_idle_locked()
            plan = self.plan_linear(
                target,
                tcp=tcp,
                orientation_mode=orientation_mode,
                look_at=look_at,
                speed=speed,
                acceleration=acceleration,
                target_seed=target_seed,
            )
            handle = self._start_locked(
                lambda event: self._execute_plan(
                    plan,
                    event,
                    cancellation_message="linear motion cancelled",
                    # Host-side trajectory generation owns Cartesian/joint
                    # speed and acceleration. Match smooth teleoperation on calibrated
                    # hardware instead of adding per-sample servo-side speed throttling.
                    servo_speed_raw=TELEOP_SERVO_SPEED_RAW,
                    servo_acceleration_raw=TELEOP_SERVO_ACCELERATION_RAW,
                )
            )
        return handle.wait() if wait else handle

    def start_joint_stream(
        self,
        *,
        frequency_hz: float = DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
        tcp: Pose | None = None,
    ) -> None:
        """Begin guarded continuous joint streaming from the current measured pose.

        Streaming has its own explicit clock.  It defaults below the normal 50 Hz
        trajectory command clock because hardware teleoperation currently performs
        synchronous feedback/fault/effort checks on every accepted sample.
        """
        frequency = self._positive(frequency_hz, "stream frequency")
        if frequency > self.config.command_frequency_hz:
            raise SafetyViolationError(
                f"stream frequency {frequency:.1f} Hz exceeds configured command "
                f"frequency ceiling {self.config.command_frequency_hz:.1f} Hz"
            )
        with self._state_lock:
            self._ensure_idle_locked()
            self._require_ready()
            present = dict(self.backend.read_joint_positions())
            # A manually placed, calibrated rest pose can sit just outside the
            # generic URDF joint limits. Accept that measured baseline, but do
            # not let a stream travel farther out on that side. Motion back
            # toward and through the model's normal range remains available.
            stream_limits = self._effective_limits()
            calibration = getattr(self.backend, "calibration", None)
            measured_limits = {
                name: calibration.motors[name].radians_limits
                if calibration is not None and name in calibration.motors
                else stream_limits[name]
                for name in ARM_JOINTS
            }
            validate_joint_targets(present, limits=measured_limits)
            for name, position in present.items():
                lower, upper = stream_limits[name]
                stream_limits[name] = (min(lower, position), max(upper, position))
            self._joint_stream = JointStreamState(
                last_command=present,
                last_velocity=None,
                previous_actual=present.copy(),
                frequency_hz=frequency,
                limits=stream_limits,
                tcp=tcp,
            )

    def stream_joint_target(
        self,
        positions: Mapping[str, float],
        *,
        gripper: float | None = None,
        gripper_speed_raw: int | None = None,
    ) -> MotionResult:
        """Accept one guarded sample in an active joint stream.

        This is intended for leader/follower teleoperation. It does not wait for
        settling; successful return means the sample passed checks and was written.
        """
        with self._state_lock:
            state = self._joint_stream
            if state is None:
                raise InvalidCommandError("joint streaming is not active")
            self._require_ready()
            if set(positions) != set(ARM_JOINTS):
                raise InvalidCommandError(
                    "stream target must provide exactly the five canonical arm joints"
                )
            target = validate_joint_targets(positions, limits=state.limits)
            validate_command_step(
                state.last_command,
                target,
                self.config.max_command_step_radians,
            )

            dt = 1.0 / state.frequency_hz
            velocity = {
                name: (target[name] - state.last_command[name]) / dt
                for name in ARM_JOINTS
            }
            max_speed = max(abs(value) for value in velocity.values())
            if max_speed > self.config.stream_joint_speed_limit * 1.001:
                raise SafetyViolationError(
                    f"streamed joint speed {max_speed:.4f} rad/s exceeds "
                    f"{self.config.stream_joint_speed_limit:.4f} rad/s"
                )
            if state.last_velocity is not None:
                acceleration = {
                    name: (velocity[name] - state.last_velocity[name]) / dt
                    for name in ARM_JOINTS
                }
                max_acceleration = max(abs(value) for value in acceleration.values())
                if max_acceleration > self.config.stream_joint_acceleration_limit * 1.001:
                    raise SafetyViolationError(
                        f"streamed joint acceleration {max_acceleration:.4f} rad/s² exceeds "
                        f"{self.config.stream_joint_acceleration_limit:.4f} rad/s²"
                    )

            if self.config.enable_workspace_checks and self.config.teleop_workspace_checks:
                max_delta = max(
                    abs(target[name] - state.last_command[name])
                    for name in ARM_JOINTS
                )
                segments = max(
                    1,
                    int(
                        math.ceil(
                            max_delta / self.config.workspace_check_step_rad
                        )
                    ),
                )
                samples = tuple(
                    {
                        name: state.last_command[name]
                        + (target[name] - state.last_command[name]) * fraction
                        for name in ARM_JOINTS
                    }
                    for fraction in np.linspace(0.0, 1.0, segments + 1)
                )
                validate_workspace_path(
                    self.model,
                    samples,
                    tcp=state.tcp,
                    **self._workspace_kwargs(),
                )

            if gripper is not None:
                gripper_value = float(gripper)
                if not math.isfinite(gripper_value) or not 0.0 <= gripper_value <= 1.0:
                    raise InvalidCommandError("streamed gripper must be within [0, 1]")
            else:
                gripper_value = None
            if gripper_speed_raw is not None and (
                not isinstance(gripper_speed_raw, int)
                or not 1 <= gripper_speed_raw <= 32767
            ):
                raise InvalidCommandError("streamed gripper speed must be within [1, 32767]")

            try:
                # Match the direct position-command behavior used by LeRobot's
                # Feetech follower: the host-side stream limiter shapes targets,
                # while the servo is not given a second, much slower speed cap.
                # Goal speed 0 means the servo's maximum speed; acceleration 254
                # is LeRobot's configured maximum acceleration profile.
                self.backend.write_joint_positions(
                    target,
                    speed_raw=TELEOP_SERVO_SPEED_RAW,
                    acceleration_raw=TELEOP_SERVO_ACCELERATION_RAW,
                )
                if gripper_value is not None:
                    if gripper_speed_raw is None:
                        self.backend.write_tool_position(STOCK_GRIPPER, gripper_value)
                    else:
                        self.backend.write_tool_position(
                            STOCK_GRIPPER, gripper_value, speed_raw=gripper_speed_raw
                        )
                actual = self._monitor_motion(
                    target,
                    state.last_command,
                    state.previous_actual,
                )
            except BaseException:
                self._joint_stream = None
                try:
                    self.backend.stop()
                except Exception:
                    pass
                raise

            state.last_command = dict(target)
            state.last_velocity = velocity
            state.previous_actual = dict(actual)
            return MotionResult(
                True,
                False,
                message="stream target accepted",
                final_positions=actual,
            )

    def stop_joint_stream(self, *, hold: bool = True) -> None:
        with self._state_lock:
            self._joint_stream = None
        # A failed stream write clears its state before control returns to the GUI.
        # Still issue an explicit hold when asked: the last servo goal may otherwise
        # continue executing if the first best-effort stop failed.
        if hold:
            self.backend.stop()

    def play_trajectory(
        self,
        trajectory: Trajectory,
        *,
        speed_scale: float = 1.0,
        move_to_start: bool = True,
        tcp: Pose | None = None,
        gripper_speed_raw: int | None = None,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        with self._state_lock:
            self._ensure_idle_locked()
            plan = self.plan_recorded_trajectory(
                trajectory,
                speed_scale=speed_scale,
                move_to_start=move_to_start,
                tcp=tcp,
            )
            handle = self._start_locked(
                lambda event: self._execute_recorded(
                    plan,
                    event,
                    gripper_speed_raw=gripper_speed_raw,
                )
            )
        return handle.wait() if wait else handle

    def stop(self, *, wait: bool = True) -> None:
        with self._state_lock:
            handle = self._active_handle
            self._joint_stream = None
            if handle is not None and not handle.done:
                handle.cancel()
        self.backend.stop()
        if wait and handle is not None and not handle.done:
            try:
                handle.wait(self.config.stop_timeout_s)
            except MotionCancelledError:
                pass
