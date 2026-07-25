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
from soarm101_motion.constants import ARM_JOINTS, JOINT_LIMITS
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
from soarm101_motion.safety import validate_command_step, validate_joint_targets
from soarm101_motion.types import MotionResult, Pose

T = TypeVar("T")


class MotionHandle(Generic[T]):
    """Cancelable result handle returned by all motion commands internally."""

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

    @property
    def is_moving(self) -> bool:
        with self._state_lock:
            return self._active_handle is not None and not self._active_handle.done

    def _clear_handle(self, handle: MotionHandle[MotionResult]) -> None:
        with self._state_lock:
            if self._active_handle is handle:
                self._active_handle = None

    def _ensure_idle_locked(self) -> None:
        if self._active_handle is not None and not self._active_handle.done:
            raise InvalidCommandError("another motion is already active")

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

    def _sleep_until(self, deadline: float) -> None:
        if not getattr(self.backend, "realtime", True):
            return
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)

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
    ) -> None:
        limits = self._effective_limits()
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
    ) -> PlannedPath:
        max_delta = max(abs(target[name] - start[name]) for name in ARM_JOINTS)
        initial = self._minimum_duration(max_delta, speed, acceleration)

        def builder(duration: float) -> tuple[dict[str, float], ...]:
            steps = max(2, int(math.ceil(duration * self.config.command_frequency_hz)) + 1)
            samples: list[dict[str, float]] = []
            for index in range(steps):
                fraction = self._minimum_jerk(index / (steps - 1))
                samples.append(
                    {
                        name: start[name] + (target[name] - start[name]) * fraction
                        for name in ARM_JOINTS
                    }
                )
            return tuple(samples)

        samples, duration = self._retime(
            builder,
            initial,
            speed_limit=speed,
            acceleration_limit=acceleration,
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
        linear_speed = self._positive(
            speed if speed is not None else self.config.default_linear_speed,
            "linear speed",
        )
        linear_acceleration = self._positive(
            acceleration if acceleration is not None else self.config.default_linear_acceleration,
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

    def _check_cancelled(self, cancel_event: threading.Event, message: str) -> None:
        if cancel_event.is_set():
            self.backend.stop()
            raise MotionCancelledError(message)

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
            if error <= self.config.joint_position_tolerance_rad and not state.moving:
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
    ) -> MotionResult:
        samples = plan.command_samples
        if len(samples) <= 1:
            return self._wait_for_settle(samples[-1], cancel_event)
        frequency = self.config.command_frequency_hz
        started = time.perf_counter()
        for index, command in enumerate(samples[1:], start=1):
            self._check_cancelled(cancel_event, cancellation_message)
            self.backend.write_joint_positions(command)
            self._sleep_until(started + index / frequency)
        return self._wait_for_settle(samples[-1], cancel_event)

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
    ) -> MotionResult | MotionHandle[MotionResult]:
        with self._state_lock:
            self._ensure_idle_locked()
            self._require_ready()
            present = self.backend.read_joint_positions()
            limits = self._effective_limits()
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
            joint_speed = self._positive(
                speed if speed is not None else self.config.default_joint_speed,
                "joint speed",
            )
            joint_acceleration = self._positive(
                acceleration
                if acceleration is not None
                else self.config.default_joint_acceleration,
                "joint acceleration",
            )
            plan = self._plan_joint_motion(
                present,
                target,
                speed=joint_speed,
                acceleration=joint_acceleration,
            )
            handle = self._start_locked(
                lambda event: self._execute_plan(
                    plan,
                    event,
                    cancellation_message="joint motion cancelled",
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

    def stop(self, *, wait: bool = True) -> None:
        with self._state_lock:
            handle = self._active_handle
            if handle is not None and not handle.done:
                handle.cancel()
        self.backend.stop()
        if wait and handle is not None and not handle.done:
            try:
                handle.wait(self.config.stop_timeout_s)
            except MotionCancelledError:
                pass
