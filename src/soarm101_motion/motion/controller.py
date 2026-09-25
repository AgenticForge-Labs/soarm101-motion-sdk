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
    TELEOP_SERVO_ACCELERATION_RAW,
    TELEOP_SERVO_SPEED_RAW,
)
from soarm101_motion.exceptions import (
    HardwareFaultError,
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

    @staticmethod
    def _sample_waypoint_path(
        waypoints: Sequence[Mapping[str, float]],
        progress: float,
    ) -> dict[str, float]:
        if progress >= 1.0:
            return dict(waypoints[-1])
        coordinate = progress * (len(waypoints) - 1)
        index = min(int(math.floor(coordinate)), len(waypoints) - 2)
        local = coordinate - index
        first, second = waypoints[index], waypoints[index + 1]
        return {
            name: first[name] + (second[name] - first[name]) * local
            for name in ARM_JOINTS
        }

    def plan_linear(
        self,
        target: Pose,
        *,
        tcp: Pose | None = None,
        orientation_mode: OrientationMode = "compatible",
        look_at: np.ndarray | None = None,
        speed: float | None = None,
        acceleration: float | None = None,
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
        angular_distance = float(
            np.linalg.norm(Rotation.from_matrix(target.rotation @ start_pose.rotation.T).as_rotvec())
        )
        segments = max(
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
        fractions = np.linspace(0.0, 1.0, segments + 1)
        slerp = Slerp(
            [0.0, 1.0],
            Rotation.from_matrix(np.stack([start_pose.rotation, target.rotation])),
        )
        cartesian: list[Pose] = []
        joints: list[dict[str, float]] = [dict(start_joints)]
        seed = dict(start_joints)
        limits = self._effective_limits()
        for index, fraction in enumerate(fractions):
            pose = Pose(
                start_pose.position + (target.position - start_pose.position) * fraction,
                slerp([fraction]).as_matrix()[0],
            )
            cartesian.append(pose)
            if index == 0:
                continue
            solution = self.ik.solve_or_raise(
                pose,
                seed=seed,
                tcp=tcp,
                options=IKOptions(
                    orientation_mode=orientation_mode,
                    look_at=look_at,
                    position_tolerance_m=self.config.cartesian_position_tolerance_m,
                    multi_start=index == 1,
                ),
            )
            candidate = validate_joint_targets(solution.joints, limits=limits)
            jump = max(abs(candidate[name] - seed[name]) for name in ARM_JOINTS)
            if jump > self.config.max_ik_waypoint_jump_radians:
                raise InvalidCommandError(
                    f"IK path discontinuity of {jump:.3f} rad exceeds "
                    f"{self.config.max_ik_waypoint_jump_radians:.3f} rad"
                )
            seed = candidate
            joints.append(candidate)

        linear_duration = self._minimum_duration(distance, linear_speed, linear_acceleration)
        angular_duration = self._minimum_duration(
            angular_distance,
            self.config.default_angular_speed,
            self.config.default_angular_acceleration,
        )
        joint_path_delta = max(
            sum(abs(second[name] - first[name]) for first, second in zip(joints, joints[1:]))
            for name in ARM_JOINTS
        )
        joint_duration = self._minimum_duration(
            joint_path_delta,
            self.config.default_joint_speed,
            self.config.default_joint_acceleration,
        )
        initial_duration = max(linear_duration, angular_duration, joint_duration)

        def builder(duration: float) -> tuple[dict[str, float], ...]:
            steps = max(2, int(math.ceil(duration * self.config.command_frequency_hz)) + 1)
            return tuple(
                self._sample_waypoint_path(
                    joints,
                    self._minimum_jerk(index / (steps - 1)),
                )
                for index in range(steps)
            )

        samples, duration = self._retime(
            builder,
            initial_duration,
            speed_limit=self.config.default_joint_speed,
            acceleration_limit=self.config.default_joint_acceleration,
        )
        return PlannedPath(tuple(joints), tuple(cartesian), samples, duration)

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
                raise MotionTimeoutError(
                    f"motion did not settle within {self.config.motion_completion_timeout_s:.2f}s; "
                    f"maximum joint error is {error:.4f} rad"
                )
            time.sleep(self.config.feedback_poll_interval_s)

    def _execute_plan(
        self,
        plan: PlannedPath,
        cancel_event: threading.Event,
        *,
        cancellation_message: str,
        servo_speed_raw: int | None = None,
        servo_acceleration_raw: int | None = None,
    ) -> MotionResult:
        samples = plan.command_samples
        try:
            if len(samples) <= 1:
                return self._wait_for_settle(samples[-1], cancel_event)
            frequency = self.config.command_frequency_hz
            started = time.perf_counter()
            monitor_every = max(
                1,
                int(math.ceil(self.config.trajectory_feedback_interval_s * frequency)),
            )
            previous_command = samples[0]
            previous_actual = self.backend.read_joint_positions()
            for index, command in enumerate(samples[1:], start=1):
                self._check_cancelled(cancel_event, cancellation_message)
                lateness = self._sleep_until(started + index / frequency)
                if lateness > self.config.max_command_lateness_s:
                    raise MotionTimeoutError(
                        f"motion command deadline missed by {lateness:.3f}s"
                    )
                self._check_cancelled(cancel_event, cancellation_message)
                if servo_speed_raw is None and servo_acceleration_raw is None:
                    self.backend.write_joint_positions(command)
                else:
                    self.backend.write_joint_positions(
                        command,
                        speed_raw=servo_speed_raw,
                        acceleration_raw=servo_acceleration_raw,
                    )
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
    ) -> MotionResult:
        samples = plan.command_samples
        gripper_samples = plan.gripper_samples
        try:
            self._check_cancelled(cancel_event, "recorded trajectory cancelled")
            self.backend.write_tool_position(STOCK_GRIPPER, gripper_samples[0])
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
                self.backend.write_tool_position(STOCK_GRIPPER, gripper_samples[index])
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
    ) -> MotionResult | MotionHandle[MotionResult]:
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
            )
            handle = self._start_locked(
                lambda event: self._execute_plan(
                    plan,
                    event,
                    cancellation_message="linear motion cancelled",
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
                lambda event: self._execute_recorded(plan, event)
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
