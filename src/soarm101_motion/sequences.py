"""Persistent robot motion sequences and a cancelable sequence runner."""

from __future__ import annotations

import json
import math
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable

from soarm101_motion.exceptions import MotionCancelledError
from soarm101_motion.motion import MotionHandle
from soarm101_motion.primitives import MotionPrimitiveLibrary
from soarm101_motion.poses import HOME_POSE_NAME, REST_POSE_NAME, PoseLibrary
from soarm101_motion.trajectories import TrajectoryLibrary
from soarm101_motion.types import MotionResult, Pose

if TYPE_CHECKING:
    from soarm101_motion.arm import SOARM101

_SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
_ALLOWED_KINDS = {"point", "home", "rest", "gripper", "wait", "trajectory", "primitive"}


@dataclass(frozen=True)
class SequenceStep:
    kind: str
    params: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        kind = str(self.kind).strip().lower()
        if kind not in _ALLOWED_KINDS:
            raise ValueError(f"unsupported sequence step kind {kind!r}")
        object.__setattr__(self, "kind", kind)
        object.__setattr__(self, "params", dict(self.params))

    def to_mapping(self) -> dict[str, Any]:
        return {"kind": self.kind, "params": self.params}

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "SequenceStep":
        return cls(kind=str(data["kind"]), params=dict(data.get("params", {})))


@dataclass(frozen=True)
class MotionSequence:
    name: str
    steps: tuple[SequenceStep, ...]
    description: str = ""
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not _SAFE_NAME.fullmatch(self.name):
            raise ValueError("sequence name contains unsupported characters")
        if not self.steps:
            raise ValueError("sequence requires at least one step")
        object.__setattr__(self, "steps", tuple(self.steps))
        object.__setattr__(self, "metadata", dict(self.metadata))

    def to_mapping(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "steps": [step.to_mapping() for step in self.steps],
            "metadata": self.metadata,
        }

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "MotionSequence":
        return cls(
            name=str(data["name"]),
            description=str(data.get("description", "")),
            steps=tuple(SequenceStep.from_mapping(dict(step)) for step in data["steps"]),
            metadata=dict(data.get("metadata", {})),
        )


def default_sequence_library_path(robot_id: str) -> Path:
    return Path.home() / ".config" / "soarm101" / "sequences" / f"{robot_id}.json"


class SequenceLibrary:
    schema_version = 1

    def __init__(self, robot_id: str, *, path: str | Path | None = None) -> None:
        self.robot_id = str(robot_id).strip() or "so101"
        self.path = Path(path).expanduser() if path is not None else default_sequence_library_path(self.robot_id)
        self._items: dict[str, MotionSequence] = {}
        self.reload()

    def reload(self) -> None:
        self._items = {}
        if not self.path.is_file():
            return
        data = json.loads(self.path.read_text(encoding="utf-8"))
        self._items = {
            str(name): MotionSequence.from_mapping(dict(value))
            for name, value in data.get("sequences", {}).items()
        }

    def _write(self) -> Path:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": self.schema_version,
            "robot_id": self.robot_id,
            "sequences": {
                name: sequence.to_mapping()
                for name, sequence in sorted(self._items.items())
            },
        }
        self.path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return self.path

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._items))

    def get(self, name: str) -> MotionSequence | None:
        return self._items.get(str(name).strip())

    def require(self, name: str) -> MotionSequence:
        key = str(name).strip()
        sequence = self.get(key)
        if sequence is None:
            raise KeyError(f"motion sequence {key!r} does not exist")
        return sequence

    def save(self, sequence: MotionSequence) -> Path:
        self._items[sequence.name] = sequence
        return self._write()

    def delete(self, name: str) -> Path:
        key = str(name).strip()
        if key not in self._items:
            raise KeyError(key)
        del self._items[key]
        return self._write()


ProgressCallback = Callable[[int, int, SequenceStep, str], None]


class SequenceRunner:
    """Execute persisted semantic steps using only public guarded arm operations."""

    def __init__(
        self,
        arm: "SOARM101",
        *,
        pose_library: PoseLibrary,
        trajectory_library: TrajectoryLibrary,
        primitive_library: MotionPrimitiveLibrary | None = None,
    ) -> None:
        self.arm = arm
        self.pose_library = pose_library
        self.trajectory_library = trajectory_library
        self.primitive_library = primitive_library
        self._pause_event = threading.Event()

    @property
    def is_paused(self) -> bool:
        return self._pause_event.is_set()

    def pause(self) -> None:
        self._pause_event.set()

    def resume(self) -> None:
        self._pause_event.clear()

    def _wait_if_paused(self, cancel_event: threading.Event) -> None:
        while self._pause_event.is_set():
            if cancel_event.wait(0.05):
                raise MotionCancelledError("sequence cancelled while paused")

    def _wait_seconds(self, seconds: float, cancel_event: threading.Event) -> None:
        remaining = seconds
        last = time.monotonic()
        while remaining > 0:
            if cancel_event.is_set():
                raise MotionCancelledError("sequence cancelled during wait")
            if self._pause_event.is_set():
                self._wait_if_paused(cancel_event)
                last = time.monotonic()
                continue
            interval = min(0.05, remaining)
            if cancel_event.wait(interval):
                raise MotionCancelledError("sequence cancelled during wait")
            now = time.monotonic()
            remaining -= max(0.0, now - last)
            last = now

    def _scaled(self, value: float, overall: float, local: float = 1.0) -> float:
        result = float(value) * float(overall) * float(local)
        if not math.isfinite(result) or result <= 0:
            raise ValueError("sequence speed scale must be positive and finite")
        return result

    def _move_pose(self, name: str, mode: str, speed_scale: float) -> MotionResult:
        pose = self.pose_library.require(name)
        self.arm.require_artifact_calibration(
            {
                "source_robot_id": pose.source_robot_id,
                "source_calibration_id": pose.source_calibration_id,
                "target_robot_id": pose.target_robot_id,
                "target_calibration_id": pose.target_calibration_id,
            },
            artifact_label=f"saved pose {name!r}",
        )
        if mode == "linear":
            target = Pose.from_xyz_rpy(*pose.tcp_xyz_rpy)
            return self.arm.move_linear(
                target,
                speed=self._scaled(self.arm.config.default_linear_speed, speed_scale),
                acceleration=self._scaled(
                    self.arm.config.default_linear_acceleration, speed_scale
                ),
                wait=True,
            )
        return self.arm.move_joints(
            pose.joints,
            speed=self._scaled(self.arm.config.default_joint_speed, speed_scale),
            acceleration=self._scaled(
                self.arm.config.default_joint_acceleration, speed_scale
            ),
            wait=True,
        )

    def _execute_step(
        self,
        step: SequenceStep,
        *,
        overall_speed_scale: float,
        cancel_event: threading.Event,
    ) -> MotionResult:
        params = step.params
        local_scale = float(params.get("speed_scale", 1.0))
        scale = overall_speed_scale * local_scale

        if step.kind == "point":
            return self._move_pose(
                str(params["name"]),
                str(params.get("mode", "joint")),
                scale,
            )
        if step.kind == "home":
            return self._move_pose(HOME_POSE_NAME, str(params.get("mode", "joint")), scale)
        if step.kind == "rest":
            return self._move_pose(REST_POSE_NAME, str(params.get("mode", "joint")), scale)
        if step.kind == "gripper":
            return self.arm.tool.move(float(params["position"]), wait=True)
        if step.kind == "wait":
            seconds = float(params["seconds"])
            if not math.isfinite(seconds) or seconds < 0:
                raise ValueError("wait seconds must be finite and non-negative")
            self._wait_seconds(seconds, cancel_event)
            return MotionResult(True, True, message=f"waited {seconds:.3f}s")
        if step.kind == "trajectory":
            trajectory = self.trajectory_library.load(
                str(params["name"]),
                kind=str(params.get("kind", "edited")),
            )
            self.arm.require_artifact_calibration(
                trajectory.metadata,
                artifact_label=f"trajectory {params['name']!r}",
            )
            loops = int(params.get("loops", 1))
            if loops < 1:
                raise ValueError("trajectory loops must be >= 1")
            result = MotionResult(True, True)
            for _ in range(loops):
                if cancel_event.is_set():
                    raise MotionCancelledError("sequence cancelled")
                result = self.arm.play_trajectory(
                    trajectory,
                    speed_scale=scale,
                    move_to_start=True,
                    wait=True,
                )
            return result
        if step.kind == "primitive":
            if self.primitive_library is None:
                raise RuntimeError("sequence runner has no primitive library")
            primitive = self.primitive_library.require(str(params["name"]))
            loops = int(params.get("loops", 1))
            if loops > 1 and not primitive.loopable:
                raise ValueError(f"primitive {primitive.name!r} is not marked loopable")
            trajectory = self.trajectory_library.load(
                primitive.trajectory_name,
                kind=primitive.trajectory_kind,
            )
            self.arm.require_artifact_calibration(
                trajectory.metadata,
                artifact_label=f"primitive {primitive.name!r}",
            )
            result = MotionResult(True, True)
            primitive_scale = scale * primitive.default_speed_scale
            for _ in range(max(1, loops)):
                if cancel_event.is_set():
                    raise RuntimeError("sequence cancelled")
                result = self.arm.play_trajectory(
                    trajectory,
                    speed_scale=primitive_scale,
                    move_to_start=True,
                    wait=True,
                )
            return result
        raise AssertionError(step.kind)

    def run(
        self,
        sequence: MotionSequence,
        *,
        repeat: int = 1,
        speed_scale: float = 1.0,
        start_index: int = 0,
        stop_index: int | None = None,
        on_progress: ProgressCallback | None = None,
        wait: bool = True,
    ) -> MotionResult | MotionHandle[MotionResult]:
        repeat_count = int(repeat)
        if repeat_count < 1:
            raise ValueError("repeat must be >= 1")
        overall = float(speed_scale)
        if not math.isfinite(overall) or overall <= 0:
            raise ValueError("speed_scale must be positive and finite")
        if start_index < 0 or start_index >= len(sequence.steps):
            raise IndexError("start_index is outside the sequence")
        end = len(sequence.steps) if stop_index is None else int(stop_index)
        if end <= start_index or end > len(sequence.steps):
            raise IndexError("stop_index is outside the sequence")

        selected = sequence.steps[start_index:end]

        def operation(cancel_event: threading.Event) -> MotionResult:
            result = MotionResult(True, True)
            total = repeat_count * len(selected)
            progress_index = 0
            for _cycle in range(repeat_count):
                for absolute_index, step in enumerate(selected, start=start_index):
                    if cancel_event.is_set():
                        raise MotionCancelledError("sequence cancelled")
                    self._wait_if_paused(cancel_event)
                    if on_progress is not None:
                        on_progress(absolute_index, total, step, "started")
                    try:
                        result = self._execute_step(
                            step,
                            overall_speed_scale=overall,
                            cancel_event=cancel_event,
                        )
                    except BaseException:
                        if on_progress is not None:
                            on_progress(absolute_index, total, step, "failed")
                        raise
                    progress_index += 1
                    if on_progress is not None:
                        on_progress(absolute_index, total, step, "completed")
            return MotionResult(
                result.accepted,
                result.completed,
                message=f"completed {progress_index} sequence step executions",
                final_positions=result.final_positions,
            )

        handle: MotionHandle[MotionResult] = MotionHandle(operation)
        handle.start()
        return handle.wait() if wait else handle
