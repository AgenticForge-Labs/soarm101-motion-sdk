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
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.exceptions import InvalidCommandError, MotionCancelledError
from soarm101_motion.hardware.base import SO101HardwareBackend
from soarm101_motion.kinematics import IKOptions, IKSolver, OrientationMode, SO101KinematicModel
from soarm101_motion.safety import validate_command_step, validate_joint_targets
from soarm101_motion.types import MotionResult, Pose

T = TypeVar("T")


class MotionHandle(Generic[T]):
    """Cancelable result handle returned by nonblocking motion commands."""

    def __init__(self, operation: Callable[[threading.Event], T]) -> None:
        self._cancel_event = threading.Event()
        self._done_event = threading.Event()
        self._result: T | None = None
        self._exception: BaseException | None = None
        self._thread = threading.Thread(target=self._run, args=(operation,), daemon=True)
        self._thread.start()

    def _run(self, operation: Callable[[threading.Event], T]) -> None:
        try:
            self._result = operation(self._cancel_event)
        except BaseException as exc:
            self._exception = exc
        finally:
            self._done_event.set()

    @property
    def done(self) -> bool:
        return self._done_event.is_set()

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
        self._motion_lock = threading.Lock()
        self._active_handle: MotionHandle[MotionResult] | None = None

    @property
    def is_moving(self) -> bool:
        return self._active_handle is not None and not self._active_handle.done

    def _minimum_duration(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        speed: float,
        acceleration: float,
    ) -> float:
        max_delta = max(abs(target[name] - start[name]) for name in ARM_JOINTS)
        if max_delta < 1e-12:
            return 0.0
        return max(
            1.875 * max_delta / speed,
            math.sqrt(5.774 * max_delta / acceleration),
            1.0 / self.config.command_frequency_hz,
        )

    @staticmethod
    def _minimum_jerk(alpha: float) -> float:
        alpha = min(1.0, max(0.0, alpha))
        return 10 * alpha**3 - 15 * alpha**4 + 6 * alpha**5

    def _sleep_until(self, deadline: float) -> None:
        if not getattr(self.backend, "realtime", True):
            return
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)

    def _run_joint_motion(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        duration: float,
        cancel_event: threading.Event,
    ) -> MotionResult:
        with self._motion_lock:
            if duration <= 0:
                return MotionResult(True, True, final_positions=dict(target))
            frequency = self.config.command_frequency_hz
            steps = max(2, int(math.ceil(duration * frequency)) + 1)
            previous = dict(start)
            started = time.perf_counter()
            for index in range(1, steps):
                if cancel_event.is_set():
                    self.backend.stop()
                    raise MotionCancelledError("joint motion cancelled")
                fraction = self._minimum_jerk(index / (steps - 1))
                command = {
                    name: start[name] + (target[name] - start[name]) * fraction
                    for name in ARM_JOINTS
                }
                validate_command_step(previous, command, self.config.max_command_step_radians)
                self.backend.write_joint_positions(command)
                previous = command
                self._sleep_until(started + index / frequency)
            return MotionResult(True, True, final_positions=dict(target))

    def move_joints(
        self,
        positions: Mapping[str, float] | Sequence[float],
        *,
        speed: float | None = None,
        acceleration: float | None = None,
        relative: bool = False,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        if self.is_moving:
            raise InvalidCommandError("another motion is already active")
        present = self.backend.read_joint_positions()
        if isinstance(positions, Mapping):
            provided = validate_joint_targets(positions)
        else:
            values = tuple(float(value) for value in positions)
            if len(values) != len(ARM_JOINTS):
                raise InvalidCommandError(f"expected {len(ARM_JOINTS)} joint values")
            provided = validate_joint_targets(dict(zip(ARM_JOINTS, values, strict=True)))
        target = dict(present)
        for name, value in provided.items():
            target[name] = present[name] + value if relative else value
        target = validate_joint_targets(target)
        duration = self._minimum_duration(
            present,
            target,
            speed or self.config.default_joint_speed,
            acceleration or self.config.default_joint_acceleration,
        )
        operation = lambda event: self._run_joint_motion(present, target, duration, event)
        if wait:
            return operation(threading.Event())
        handle = MotionHandle(operation)
        self._active_handle = handle
        return handle

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
            candidate = dict(solution.joints)
            jump = max(abs(candidate[name] - seed[name]) for name in ARM_JOINTS)
            if jump > self.config.max_ik_waypoint_jump_radians:
                raise InvalidCommandError(
                    f"IK path discontinuity of {jump:.3f} rad exceeds "
                    f"{self.config.max_ik_waypoint_jump_radians:.3f} rad"
                )
            seed = candidate
            joints.append(candidate)

        linear_speed = speed or self.config.default_linear_speed
        linear_acceleration = acceleration or self.config.default_linear_acceleration
        linear_duration = 0.0
        if distance > 1e-12:
            linear_duration = max(
                1.875 * distance / linear_speed,
                math.sqrt(5.774 * distance / linear_acceleration),
            )
        orientation_duration = 1.875 * angular_distance / 0.8 if angular_distance else 0.0
        path_distance = max(
            sum(abs(second[name] - first[name]) for first, second in zip(joints, joints[1:]))
            for name in ARM_JOINTS
        )
        max_segment_delta = max(
            (
                abs(second[name] - first[name])
                for first, second in zip(joints, joints[1:])
                for name in ARM_JOINTS
            ),
            default=0.0,
        )
        joint_duration = max(
            1.875 * path_distance / self.config.default_joint_speed if path_distance else 0.0,
            math.sqrt(5.774 * max_segment_delta / self.config.default_joint_acceleration)
            if max_segment_delta
            else 0.0,
        )
        duration = max(
            linear_duration,
            orientation_duration,
            joint_duration,
            1.0 / self.config.command_frequency_hz,
        )
        return PlannedPath(tuple(joints), tuple(cartesian), duration)

    @staticmethod
    def _sample_joint_path(
        waypoints: tuple[dict[str, float], ...],
        progress: float,
    ) -> dict[str, float]:
        if progress >= 1.0:
            return dict(waypoints[-1])
        coordinate = progress * (len(waypoints) - 1)
        index = min(int(math.floor(coordinate)), len(waypoints) - 2)
        local = coordinate - index
        first, second = waypoints[index], waypoints[index + 1]
        return {name: first[name] + (second[name] - first[name]) * local for name in ARM_JOINTS}

    def _run_waypoints(self, plan: PlannedPath, cancel_event: threading.Event) -> MotionResult:
        with self._motion_lock:
            waypoints = plan.joint_waypoints
            if len(waypoints) <= 1:
                return MotionResult(True, True, final_positions=waypoints[-1])
            frequency = self.config.command_frequency_hz
            steps = max(2, int(math.ceil(plan.duration_s * frequency)) + 1)
            started = time.perf_counter()
            previous = waypoints[0]
            for index in range(1, steps):
                if cancel_event.is_set():
                    self.backend.stop()
                    raise MotionCancelledError("linear motion cancelled")
                progress = self._minimum_jerk(index / (steps - 1))
                command = self._sample_joint_path(waypoints, progress)
                validate_command_step(previous, command, self.config.max_command_step_radians)
                self.backend.write_joint_positions(command)
                previous = command
                self._sleep_until(started + index / frequency)
            return MotionResult(True, True, final_positions=waypoints[-1])

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
        if self.is_moving:
            raise InvalidCommandError("another motion is already active")
        plan = self.plan_linear(
            target,
            tcp=tcp,
            orientation_mode=orientation_mode,
            look_at=look_at,
            speed=speed,
            acceleration=acceleration,
        )
        operation = lambda event: self._run_waypoints(plan, event)
        if wait:
            return operation(threading.Event())
        handle = MotionHandle(operation)
        self._active_handle = handle
        return handle

    def stop(self) -> None:
        if self._active_handle is not None and not self._active_handle.done:
            self._active_handle.cancel()
        self.backend.stop()
