"""Host-side smooth joint and Cartesian motion controller."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Generic, Literal, TypeVar

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from soarm101_motion.config import SOARM101Config
from soarm101_motion.constants import (
    ARM_JOINTS,
    DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
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
    resolve_effective_joint_limits,
    validate_command_step,
    validate_joint_targets,
    validate_workspace_path,
    validate_workspace_path_from_measured_start,
)
from soarm101_motion.trajectories import Trajectory
from soarm101_motion.types import MotionResult, Pose

T = TypeVar("T")
JointExecutionMode = Literal["streamed", "final_target"]


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
class StreamReversalGrace:
    expires_sample: int
    previous_wrong_way_delta_abs: float


@dataclass
class JointStreamState:
    last_command: dict[str, float]
    last_velocity: dict[str, float] | None
    previous_actual: dict[str, float]
    frequency_hz: float
    limits: dict[str, tuple[float, float]]
    speed_limits: dict[str, float]
    acceleration_limits: dict[str, float]
    sample_index: int
    last_nonzero_command_direction: dict[str, int]
    last_nonzero_command_sample: dict[str, int]
    reversal_grace: dict[str, StreamReversalGrace]
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
        calibration = getattr(self.backend, "calibration", None)
        calibrated_limits = None
        if calibration is not None:
            calibrated_limits = {
                name: motor.radians_limits
                for name, motor in calibration.motors.items()
                if name in ARM_JOINTS
            }
        return resolve_effective_joint_limits(
            calibrated_limits,
            calibrated_joint_stop_margin_rad=self.config.calibrated_joint_stop_margin_rad,
        )

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
        # Human-unit CLI/API values are converted to SI at the boundary. Exact
        # advertised ceilings such as 1000 deg/s^2 can differ from the stored
        # SI ceiling by a few floating-point ULPs depending on conversion order.
        if resolved > maximum and not math.isclose(
            resolved,
            maximum,
            rel_tol=1e-12,
            abs_tol=1e-12,
        ):
            raise SafetyViolationError(
                f"{name} {resolved:.4f} exceeds configured safety maximum {maximum:.4f}"
            )
        return min(resolved, maximum)

    def _resolve_stream_joint_limits(
        self,
        value: float | Mapping[str, float] | None,
        *,
        default: float,
        maximum: float,
        name: str,
    ) -> dict[str, float]:
        if isinstance(value, Mapping):
            if set(value) != set(ARM_JOINTS):
                raise InvalidCommandError(
                    f"{name} must provide exactly the five canonical arm joints"
                )
            return {
                joint: self._bounded(float(value[joint]), default, maximum, f"{name} {joint}")
                for joint in ARM_JOINTS
            }
        resolved = self._bounded(value, default, maximum, name)
        return {joint: resolved for joint in ARM_JOINTS}

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
                        joint_limits=limits,
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
            samples = [
                {
                    name: start[name]
                    + (target[name] - start[name])
                    * self._minimum_jerk(index / (steps - 1))
                    for name in ARM_JOINTS
                }
                for index in range(steps)
            ]
            # Preserve the already-validated endpoints exactly. Reconstructing the
            # final sample as start + (target - start) can round one ULP beyond a
            # target that intentionally sits on an executable joint-limit boundary.
            samples[0] = {name: float(start[name]) for name in ARM_JOINTS}
            samples[-1] = {name: float(target[name]) for name in ARM_JOINTS}
            return tuple(samples)

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
        """Solve the Cartesian trajectory directly at command-rate samples."""

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
        line_delta = target.position - start_pose.position
        line_length_sq = float(np.dot(line_delta, line_delta))

        def solve_sample(
            pose: Pose,
            *,
            seed: Mapping[str, float],
            multi_start: bool,
            sample_index: int,
            direction: str,
            enforce_seed_jump: bool = True,
        ) -> dict[str, float]:
            options = IKOptions(
                orientation_mode=orientation_mode,
                look_at=look_at,
                position_tolerance_m=self.config.cartesian_position_tolerance_m,
                multi_start=multi_start,
                joint_limits=limits,
            )
            solution = self.ik.solve(
                pose,
                seed=seed,
                tcp=tcp,
                options=options,
            )
            if not solution.success and not multi_start:
                solution = self.ik.solve(
                    pose,
                    seed=seed,
                    tcp=tcp,
                    options=IKOptions(
                        orientation_mode=orientation_mode,
                        look_at=look_at,
                        position_tolerance_m=self.config.cartesian_position_tolerance_m,
                        multi_start=True,
                        joint_limits=limits,
                    ),
                )
            if not solution.success:
                actual = self.model.forward(solution.joints, tcp=tcp)
                residual_mm = (actual.position - pose.position) * 1000.0
                jacobian = self.model.jacobian(solution.joints, tcp=tcp)[:3, :]
                singular = np.linalg.svd(jacobian, compute_uv=False)
                sigma_min = float(np.min(singular)) if singular.size else 0.0
                sigma_max = float(np.max(singular)) if singular.size else 0.0
                condition = float("inf") if sigma_min <= 1e-12 else sigma_max / sigma_min
                margins = {
                    name: min(
                        float(solution.joints[name]) - float(limits[name][0]),
                        float(limits[name][1]) - float(solution.joints[name]),
                    )
                    for name in ARM_JOINTS
                }
                margin_joint = min(margins, key=margins.get)
                progress = (
                    float(np.dot(pose.position - start_pose.position, line_delta) / line_length_sq)
                    if line_length_sq > 1e-18
                    else 1.0
                )
                target_mm = pose.position * 1000.0
                joints = ", ".join(
                    f"{name}={float(solution.joints[name]):+.4f}" for name in ARM_JOINTS
                )
                raise IKError(
                    f"{direction} Cartesian IK failed at sample "
                    f"{sample_index}/{len(solved_cartesian) - 1} "
                    f"(line progress={progress:.4f}, "
                    f"target=({target_mm[0]:.2f},{target_mm[1]:.2f},{target_mm[2]:.2f}) mm): "
                    f"best position error={solution.position_error_m * 1000.0:.3f} mm "
                    f"with residual=({residual_mm[0]:+.3f},{residual_mm[1]:+.3f},"
                    f"{residual_mm[2]:+.3f}) mm; "
                    f"position-Jacobian sigma_min={sigma_min:.6g}, condition={condition:.2f}; "
                    f"nearest effective joint limit={margin_joint} margin="
                    f"{margins[margin_joint]:+.4f} rad; best joints: {joints}; "
                    f"optimizer={solution.message}"
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
                    sample_index=index,
                    direction="forward",
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
                endpoint_index = len(solved_cartesian) - 1
                endpoint = solve_sample(
                    solved_cartesian[-1],
                    seed=boundary_seed,
                    multi_start=True,
                    sample_index=endpoint_index,
                    direction="reverse",
                    enforce_seed_jump=False,
                )
                reverse_samples: list[dict[str, float]] = [endpoint]
                seed = endpoint
                for sample_index in range(len(solved_cartesian) - 2, 0, -1):
                    candidate = solve_sample(
                        solved_cartesian[sample_index],
                        seed=seed,
                        multi_start=False,
                        sample_index=sample_index,
                        direction="reverse",
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

    def _plan_linear_from_start(
        self,
        start_joints: Mapping[str, float],
        target: Pose,
        *,
        tcp: Pose | None,
        orientation_mode: OrientationMode,
        look_at: np.ndarray | None,
        speed: float | None,
        acceleration: float | None,
        target_seed: Mapping[str, float] | None,
        limits: Mapping[str, tuple[float, float]],
    ) -> PlannedPath:
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
                max_speed / self.config.max_joint_speed
                if self.config.max_joint_speed
                else 1.0,
                math.sqrt(max_acceleration / self.config.max_joint_acceleration)
                if max_acceleration and self.config.max_joint_acceleration
                else 1.0,
                max_step / self.config.max_command_step_radians if max_step else 1.0,
            )
            if scale <= 1.001:
                self._validate_samples(
                    samples,
                    speed_limit=self.config.max_joint_speed,
                    acceleration_limit=self.config.max_joint_acceleration,
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

    def plan_linear_from(
        self,
        start_joints: Mapping[str, float],
        target: Pose,
        *,
        tcp: Pose | None = None,
        orientation_mode: OrientationMode = "compatible",
        look_at: np.ndarray | None = None,
        speed: float | None = None,
        acceleration: float | None = None,
        target_seed: Mapping[str, float] | None = None,
        limits_override: Mapping[str, tuple[float, float]] | None = None,
    ) -> PlannedPath:
        """Read-only Cartesian planning from an explicit validated start configuration.

        ``limits_override`` is diagnostic-only. It may widen normal executable
        bounds for read-only planning, but never beyond the active motor calibration.
        Executable motion uses the centrally resolved nominal-plus-calibrated-extension
        limits and configured mechanical-stop margin.
        """

        state = self.backend.get_hardware_state()
        if not state.connected:
            raise RobotConnectionError("robot is not connected")
        if state.faulted:
            raise HardwareFaultError(state.fault_message or "robot is faulted")
        if limits_override is None:
            limits = self._effective_limits()
        else:
            missing = set(ARM_JOINTS) - set(limits_override)
            if missing:
                raise InvalidCommandError(
                    "limits_override is missing joints: " + ", ".join(sorted(missing))
                )
            limits = {
                name: (
                    float(limits_override[name][0]),
                    float(limits_override[name][1]),
                )
                for name in ARM_JOINTS
            }
            calibration = getattr(self.backend, "calibration", None)
            for name in ARM_JOINTS:
                lower, upper = limits[name]
                if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                    raise InvalidCommandError(
                        f"invalid read-only limits_override for {name}: {lower}..{upper}"
                    )
                if calibration is not None:
                    calibrated_lower, calibrated_upper = (
                        calibration.motors[name].radians_limits
                    )
                    if (
                        lower < calibrated_lower - 1e-12
                        or upper > calibrated_upper + 1e-12
                    ):
                        raise SafetyViolationError(
                            f"read-only limits_override for {name} exceeds calibrated "
                            f"range {calibrated_lower:.4f}..{calibrated_upper:.4f} rad"
                        )
        start = validate_joint_targets(start_joints, limits=limits)
        return self._plan_linear_from_start(
            start,
            target,
            tcp=tcp,
            orientation_mode=orientation_mode,
            look_at=look_at,
            speed=speed,
            acceleration=acceleration,
            target_seed=target_seed,
            limits=limits,
        )

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
        start_joints = self.backend.read_joint_positions()
        limits = self._limits_for_present(start_joints)
        return self._plan_linear_from_start(
            start_joints,
            target,
            tcp=tcp,
            orientation_mode=orientation_mode,
            look_at=look_at,
            speed=speed,
            acceleration=acceleration,
            target_seed=target_seed,
            limits=limits,
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

    def _encoder_target_key(
        self,
        positions: Mapping[str, float],
    ) -> tuple[int, ...] | None:
        """Return the calibrated raw encoder target for one arm command.

        Planned host trajectories are continuous in radians, but calibrated Feetech
        execution ultimately resolves each joint to an integer encoder target. Near
        zero velocity, many adjacent host samples can therefore address the exact
        same hardware position. Returning None keeps non-calibrated/simulation
        backends on the existing write-per-sample behavior.
        """

        calibration = getattr(self.backend, "calibration", None)
        motors = getattr(calibration, "motors", None)
        if motors is None:
            return None

        raw: list[int] = []
        for name in ARM_JOINTS:
            motor = motors.get(name)
            converter = getattr(motor, "radians_to_raw", None)
            if not callable(converter):
                return None
            raw.append(int(converter(float(positions[name]))))
        return tuple(raw)

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

    def _read_guarded_actual(
        self,
        command: Mapping[str, float],
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
        return actual

    def _monitor_motion(
        self,
        command: Mapping[str, float],
        previous_command: Mapping[str, float],
        previous_actual: Mapping[str, float],
    ) -> dict[str, float]:
        actual = self._read_guarded_actual(command)
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

    def _monitor_stream_motion(
        self,
        command: Mapping[str, float],
        previous_command: Mapping[str, float],
        stream_state: JointStreamState,
    ) -> dict[str, float]:
        """Monitor a live stream while distinguishing braking lag from runaway motion."""
        actual = self._read_guarded_actual(command)
        current_sample = stream_state.sample_index + 1
        total_grace_samples = max(
            1,
            math.ceil(
                self.config.stream_reversal_grace_s
                * stream_state.frequency_hz
            ),
        )
        for name in ARM_JOINTS:
            command_delta = command[name] - previous_command[name]
            actual_delta = actual[name] - stream_state.previous_actual[name]
            grace = stream_state.reversal_grace.get(name)
            if grace is not None and current_sample > grace.expires_sample:
                stream_state.reversal_grace.pop(name, None)
                grace = None

            if abs(command_delta) < self.config.joint_position_tolerance_rad:
                continue
            if abs(actual_delta) < self.config.unexpected_direction_threshold_rad:
                stream_state.reversal_grace.pop(name, None)
                continue
            if command_delta * actual_delta >= 0.0:
                stream_state.reversal_grace.pop(name, None)
                continue

            current_direction = 1 if command_delta > 0.0 else -1
            previous_direction = stream_state.last_nonzero_command_direction[name]
            previous_direction_sample = stream_state.last_nonzero_command_sample[name]
            recent_reversal = (
                previous_direction != 0
                and current_direction != previous_direction
                and current_sample - previous_direction_sample <= total_grace_samples
            )
            if recent_reversal:
                stream_state.reversal_grace[name] = StreamReversalGrace(
                    expires_sample=current_sample + total_grace_samples - 1,
                    previous_wrong_way_delta_abs=abs(actual_delta),
                )
                continue

            if grace is not None:
                if (
                    abs(actual_delta)
                    <= grace.previous_wrong_way_delta_abs
                    + self.config.stream_reversal_decay_tolerance_rad
                ):
                    grace.previous_wrong_way_delta_abs = abs(actual_delta)
                    continue
                raise SafetyViolationError(
                    f"{name} opposite-direction carry-through grew during reversal braking "
                    f"({actual_delta:+.3f} rad)"
                )

            raise SafetyViolationError(
                f"{name} moved {actual_delta:+.3f} rad opposite the commanded direction"
            )
        return actual

    def _monitor_final_target_motion(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        previous_actual: Mapping[str, float],
    ) -> dict[str, float]:
        """Monitor one-shot joint motion without treating expected target lag as failure.

        A final-target command intentionally asks each servo to traverse the whole
        move internally, so ordinary endpoint following error and cross-joint phase
        matching are not valid transit guards. Keep each joint inside a bounded
        start-to-target corridor while preserving fault and unexpected-direction checks.
        """
        state = self.backend.get_hardware_state()
        if state.faulted:
            raise HardwareFaultError(state.fault_message or "robot faulted during motion")
        actual = self.backend.read_joint_positions()

        for name in ARM_JOINTS:
            delta = float(target[name] - start[name])
            actual_delta = float(actual[name] - previous_actual[name])
            lower = min(float(start[name]), float(target[name])) - self.config.following_error_limit_rad
            upper = max(float(start[name]), float(target[name])) + self.config.following_error_limit_rad
            if actual[name] < lower or actual[name] > upper:
                raise SafetyViolationError(
                    f"{name} left final-target motion corridor: measured {actual[name]:.3f} rad "
                    f"outside {lower:.3f}..{upper:.3f} rad"
                )
            if abs(delta) >= self.config.joint_position_tolerance_rad:
                if (
                    abs(actual_delta) >= self.config.unexpected_direction_threshold_rad
                    and actual_delta * delta < 0.0
                ):
                    raise SafetyViolationError(
                        f"{name} moved {actual_delta:+.3f} rad opposite the final target"
                    )
            elif abs(float(actual[name]) - float(start[name])) > self.config.following_error_limit_rad:
                raise SafetyViolationError(
                    f"{name} drifted {abs(float(actual[name]) - float(start[name])):.3f} rad "
                    "during final-target motion"
                )
        return actual

    def _execute_final_target_plan(
        self,
        plan: PlannedPath,
        cancel_event: threading.Event,
        *,
        cancellation_message: str,
        servo_speed_raw: int | Mapping[str, int] | None = None,
        servo_acceleration_raw: int | None = None,
        monitor_workspace: bool = False,
        tcp: Pose | None = None,
    ) -> MotionResult:
        """Execute a validated joint plan with exactly one endpoint command.

        The existing host plan still owns endpoint/path validation and timing. The
        actuator command strategy differs only at execution: one synchronized final
        target is written, then the controller observes guarded progress until settle.
        """
        samples = plan.command_samples
        start = samples[0]
        target = samples[-1]
        try:
            if len(samples) <= 1:
                return self._wait_for_settle(target, cancel_event)

            self._check_cancelled(cancel_event, cancellation_message)
            previous_actual = self.backend.read_joint_positions()
            command_speed_raw = servo_speed_raw
            if command_speed_raw is None:
                command_speed_raw = self._synchronized_servo_speed_raw(
                    start,
                    target,
                    interval_s=max(
                        plan.duration_s,
                        1.0 / self.config.command_frequency_hz,
                    ),
                )
            command_acceleration_raw = (
                servo_acceleration_raw
                if servo_acceleration_raw is not None
                else TELEOP_SERVO_ACCELERATION_RAW
            )
            self.backend.write_joint_positions(
                target,
                speed_raw=command_speed_raw,
                acceleration_raw=command_acceleration_raw,
            )

            deadline = (
                time.monotonic()
                + plan.duration_s
                + self.config.motion_completion_timeout_s
            )
            # Workspace escape semantics are anchored to the fresh measured
            # configuration immediately before the endpoint write, not the earlier
            # planner snapshot.
            observed_path: list[Mapping[str, float]] = [dict(previous_actual)]
            while True:
                self._check_cancelled(cancel_event, cancellation_message)
                actual = self._monitor_final_target_motion(
                    start,
                    target,
                    previous_actual,
                )
                if monitor_workspace:
                    observed_path.append(dict(actual))
                    validate_workspace_path_from_measured_start(
                        self.model,
                        observed_path,
                        tcp=tcp,
                        **self._workspace_kwargs(),
                    )
                error = max(abs(float(actual[name]) - float(target[name])) for name in ARM_JOINTS)
                if error <= self.config.joint_position_tolerance_rad:
                    return self._wait_for_settle(target, cancel_event)
                if time.monotonic() >= deadline:
                    errors = {
                        name: float(target[name] - actual[name])
                        for name in ARM_JOINTS
                    }
                    worst_joint = max(ARM_JOINTS, key=lambda name: abs(errors[name]))
                    raise MotionTimeoutError(
                        "final-target motion did not reach the destination within "
                        f"{plan.duration_s + self.config.motion_completion_timeout_s:.2f}s; "
                        f"worst={worst_joint} error={abs(errors[worst_joint]):.4f} rad"
                    )
                previous_actual = actual
                time.sleep(self.config.trajectory_feedback_interval_s)
        except BaseException:
            try:
                self.backend.stop()
            except Exception:
                pass
            raise

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
            last_encoder_target = self._encoder_target_key(samples[0])
            for index, command in enumerate(samples[1:], start=1):
                self._check_cancelled(cancel_event, cancellation_message)
                lateness = self._sleep_until(started + index / frequency)
                if lateness > self.config.max_command_lateness_s:
                    raise MotionTimeoutError(
                        f"motion command deadline missed by {lateness:.3f}s"
                    )
                self._check_cancelled(cancel_event, cancellation_message)

                command_encoder_target = self._encoder_target_key(command)
                is_final_sample = index == len(samples) - 1
                duplicate_encoder_target = (
                    not is_final_sample
                    and command_encoder_target is not None
                    and command_encoder_target == last_encoder_target
                )
                if not duplicate_encoder_target:
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
                    last_encoder_target = command_encoder_target

                if index % monitor_every == 0 or is_final_sample:
                    previous_actual = self._monitor_motion(
                        last_command_sent,
                        previous_command,
                        previous_actual,
                    )
                    previous_command = last_command_sent
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
                    servo_speed_raw=TELEOP_SERVO_SPEED_RAW,
                    servo_acceleration_raw=TELEOP_SERVO_ACCELERATION_RAW,
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
                self.backend.write_joint_positions(
                    command,
                    speed_raw=TELEOP_SERVO_SPEED_RAW,
                    acceleration_raw=TELEOP_SERVO_ACCELERATION_RAW,
                )
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
        execution_mode: JointExecutionMode = "streamed",
        monitor_workspace: bool = False,
        tcp: Pose | None = None,
    ) -> MotionResult | MotionHandle[MotionResult]:
        if execution_mode not in {"streamed", "final_target"}:
            raise InvalidCommandError(
                "execution_mode must be 'streamed' or 'final_target'"
            )
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
            if execution_mode == "final_target":
                handle = self._start_locked(
                    lambda event: self._execute_final_target_plan(
                        plan,
                        event,
                        cancellation_message="joint motion cancelled",
                        servo_speed_raw=servo_speed_raw,
                        servo_acceleration_raw=servo_acceleration_raw,
                        monitor_workspace=monitor_workspace,
                        tcp=tcp,
                    )
                )
            else:
                # Host-streamed planned motion owns the requested joint speed and
                # acceleration profile. Do not impose the backend's much slower
                # default Feetech Goal_Velocity/Acceleration profile on top of that
                # trajectory: it can make a perfectly valid fast host plan outrun
                # the servos and create artificial following-error trips. Match the
                # responsive profile already used by live teleoperation.
                streamed_speed_raw = servo_speed_raw
                if streamed_speed_raw is None and not synchronize_servo_arrival:
                    streamed_speed_raw = TELEOP_SERVO_SPEED_RAW
                streamed_acceleration_raw = (
                    servo_acceleration_raw
                    if servo_acceleration_raw is not None
                    else TELEOP_SERVO_ACCELERATION_RAW
                )
                handle = self._start_locked(
                    lambda event: self._execute_plan(
                        plan,
                        event,
                        cancellation_message="joint motion cancelled",
                        servo_speed_raw=streamed_speed_raw,
                        servo_acceleration_raw=streamed_acceleration_raw,
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
                    # The host trajectory owns Cartesian/joint speed and
                    # acceleration. Give each calibrated servo a proportional position-
                    # mode speed for the next command interval so lightly loaded joints
                    # do not race ahead of gravity-loaded joints.
                    servo_acceleration_raw=TELEOP_SERVO_ACCELERATION_RAW,
                    synchronize_servo_arrival=True,
                )
            )
        return handle.wait() if wait else handle

    def start_joint_stream(
        self,
        *,
        frequency_hz: float = DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
        max_speed: float | Mapping[str, float] | None = None,
        max_acceleration: float | Mapping[str, float] | None = None,
        tcp: Pose | None = None,
    ) -> None:
        """Begin guarded continuous joint streaming from the current measured pose.

        Streaming has its own explicit clock.  It defaults below the normal 50 Hz
        trajectory command clock because hardware teleoperation currently performs
        synchronous feedback/fault/effort checks on every accepted sample.
        """
        frequency = self._positive(frequency_hz, "stream frequency")
        speed_limits = self._resolve_stream_joint_limits(
            max_speed,
            default=self.config.stream_joint_speed_limit,
            maximum=self.config.stream_joint_speed_limit,
            name="stream joint speed",
        )
        acceleration_limits = self._resolve_stream_joint_limits(
            max_acceleration,
            default=self.config.stream_joint_acceleration_limit,
            maximum=self.config.stream_joint_acceleration_limit,
            name="stream joint acceleration",
        )
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
                speed_limits=speed_limits,
                acceleration_limits=acceleration_limits,
                sample_index=0,
                last_nonzero_command_direction={name: 0 for name in ARM_JOINTS},
                last_nonzero_command_sample={name: 0 for name in ARM_JOINTS},
                reversal_grace={},
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
            for name, value in velocity.items():
                if abs(value) > state.speed_limits[name] * 1.001:
                    raise SafetyViolationError(
                        f"streamed {name} speed {abs(value):.4f} rad/s exceeds "
                        f"{state.speed_limits[name]:.4f} rad/s"
                    )
            if state.last_velocity is not None:
                acceleration = {
                    name: (velocity[name] - state.last_velocity[name]) / dt
                    for name in ARM_JOINTS
                }
                for name, value in acceleration.items():
                    if abs(value) > state.acceleration_limits[name] * 1.001:
                        raise SafetyViolationError(
                            f"streamed {name} acceleration {abs(value):.4f} rad/s² exceeds "
                            f"{state.acceleration_limits[name]:.4f} rad/s²"
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
                actual = self._monitor_stream_motion(
                    target,
                    state.last_command,
                    state,
                )
            except BaseException:
                self._joint_stream = None
                try:
                    self.backend.stop()
                except Exception:
                    pass
                raise

            state.sample_index += 1
            for name, value in velocity.items():
                command_delta = target[name] - state.last_command[name]
                if abs(command_delta) >= self.config.joint_position_tolerance_rad:
                    state.last_nonzero_command_direction[name] = 1 if value > 0.0 else -1
                    state.last_nonzero_command_sample[name] = state.sample_index
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
