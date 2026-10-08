"""Passive motion-quality tracing without adding hardware I/O."""

from __future__ import annotations

import json
import threading
import time
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from soarm101_motion.constants import ARM_JOINTS


class PassiveBackendTrace:
    """Record existing backend commands/reads without issuing additional I/O.

    The tracer wraps calls the SDK would already make. It does not poll the robot,
    change controller cadence, or request extra diagnostics while motion is active.
    This makes it suitable for comparing teleoperation and planned-motion behavior
    without turning the logger into another timing/load variable.
    """

    def __init__(
        self,
        arm: Any,
        path: str | Path,
        *,
        metadata: Mapping[str, Any] | None = None,
    ) -> None:
        self.arm = arm
        self.backend = arm.backend
        self.path = Path(path).expanduser()
        self.metadata = dict(metadata or {})
        self._started = 0.0
        self._events: list[dict[str, Any]] = []
        self._lock = threading.Lock()
        self._originals: dict[str, Any] = {}
        self._last_command_raw: dict[str, int] | None = None
        self._command_count = 0
        self._feedback_count = 0
        self._duplicate_raw_commands = 0

    def __enter__(self) -> "PassiveBackendTrace":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._events = []
        self._started = time.perf_counter()
        trace_metadata = dict(self.metadata)
        # Runtime provenance is authoritative. Diagnostic callers may use
        # arbitrary metadata without being able to duplicate or override the
        # active robot/calibration identity.
        trace_metadata["robot_id"] = getattr(self.arm.config, "robot_id", None)
        trace_metadata["calibration_id"] = getattr(self.arm, "calibration_id", None)
        self._record("trace_start", **trace_metadata)
        self._wrap("write_joint_positions", self._wrap_write_joint_positions)
        # Feetech STOP/HOLD latches the live raw encoder snapshot via this
        # lower-level method, bypassing write_joint_positions entirely.
        # Recording it is necessary to see whether HOLD changes servo goals.
        self._wrap("_write_raw_positions", self._wrap_write_raw_positions)
        self._wrap("read_joint_positions", self._wrap_read_joint_positions)
        self._wrap("get_hardware_state", self._wrap_get_hardware_state)
        self._wrap("write_tool_position", self._wrap_write_tool_position)
        self._wrap("read_tool_position", self._wrap_read_tool_position)
        if callable(getattr(self.backend, "read_motor_effort", None)):
            self._wrap("read_motor_effort", self._wrap_read_motor_effort)
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        try:
            self._record(
                "trace_end",
                error=None if exc is None else repr(exc),
                command_count=self._command_count,
                feedback_count=self._feedback_count,
                duplicate_raw_commands=self._duplicate_raw_commands,
            )
        finally:
            for name, original in self._originals.items():
                setattr(self.backend, name, original)
            self._originals.clear()
            self._flush()

    @property
    def summary(self) -> dict[str, int]:
        return {
            "command_count": self._command_count,
            "feedback_count": self._feedback_count,
            "duplicate_raw_commands": self._duplicate_raw_commands,
        }

    def mark(self, event: str, **fields: Any) -> None:
        self._record("marker", marker=event, **fields)

    def _wrap(self, name: str, factory: Any) -> None:
        original = getattr(self.backend, name, None)
        if not callable(original):
            return
        self._originals[name] = original
        setattr(self.backend, name, factory(original))

    def _record(self, event: str, **fields: Any) -> None:
        payload = {
            "time_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "monotonic_s": time.perf_counter() - self._started,
            "event": event,
            **fields,
        }
        with self._lock:
            self._events.append(payload)

    def _flush(self) -> None:
        events = list(self._events)
        for payload in events:
            joints = payload.get("joints_rad")
            if isinstance(joints, Mapping):
                payload["tcp_xyz_mm"] = self._tcp_xyz_mm(joints)
        with self.path.open("w", encoding="utf-8") as handle:
            for payload in events:
                handle.write(json.dumps(payload, default=str, ensure_ascii=False) + "\n")

    def _raw_positions(self, positions: Mapping[str, float]) -> dict[str, int] | None:
        calibration = getattr(self.backend, "calibration", None)
        motors = getattr(calibration, "motors", None)
        if motors is None:
            return None
        raw: dict[str, int] = {}
        for name in ARM_JOINTS:
            motor = motors.get(name)
            converter = getattr(motor, "radians_to_raw", None)
            if motor is None or not callable(converter) or name not in positions:
                return None
            raw[name] = int(converter(float(positions[name])))
        return raw

    def _tcp_xyz_mm(self, positions: Mapping[str, float]) -> list[float] | None:
        try:
            pose = self.arm.model.forward(positions, tcp=self.arm.active_tcp)
            return [float(value * 1000.0) for value in pose.position]
        except Exception:
            return None

    @staticmethod
    def _json_speed(speed_raw: Any) -> Any:
        if isinstance(speed_raw, Mapping):
            return {str(name): int(value) for name, value in speed_raw.items()}
        return None if speed_raw is None else int(speed_raw)

    def _wrap_write_joint_positions(self, original: Any) -> Any:
        def traced(
            positions: Mapping[str, float],
            *,
            speed_raw: Any = None,
            acceleration_raw: Any = None,
        ) -> Any:
            joints = {name: float(positions[name]) for name in ARM_JOINTS}
            raw = self._raw_positions(joints)
            duplicate_raw = raw is not None and raw == self._last_command_raw
            if duplicate_raw:
                self._duplicate_raw_commands += 1
            effective_speed_raw = (
                speed_raw
                if speed_raw is not None
                else getattr(getattr(self.backend, "config", None), "hardware_speed_raw", None)
            )
            effective_acceleration_raw = (
                acceleration_raw
                if acceleration_raw is not None
                else getattr(
                    getattr(self.backend, "config", None),
                    "hardware_acceleration_raw",
                    None,
                )
            )
            started = time.perf_counter()
            try:
                result = original(
                    positions,
                    speed_raw=speed_raw,
                    acceleration_raw=acceleration_raw,
                )
            except BaseException as exc:
                self._record(
                    "command_error",
                    joints_rad=joints,
                    joints_raw=raw,
                    requested_speed_raw=self._json_speed(speed_raw),
                    requested_acceleration_raw=acceleration_raw,
                    speed_raw=self._json_speed(effective_speed_raw),
                    acceleration_raw=effective_acceleration_raw,
                    duplicate_raw=duplicate_raw,
                    call_ms=(time.perf_counter() - started) * 1000.0,
                    error=repr(exc),
                )
                raise
            self._command_count += 1
            self._last_command_raw = raw
            self._record(
                "command",
                sequence=self._command_count,
                joints_rad=joints,
                joints_raw=raw,
                requested_speed_raw=self._json_speed(speed_raw),
                requested_acceleration_raw=acceleration_raw,
                speed_raw=self._json_speed(effective_speed_raw),
                acceleration_raw=effective_acceleration_raw,
                duplicate_raw=duplicate_raw,
                call_ms=(time.perf_counter() - started) * 1000.0,
            )
            return result

        return traced

    def _wrap_write_raw_positions(self, original: Any) -> Any:
        def traced(
            positions: Mapping[str, int],
            *,
            speed_raw: int | Mapping[str, int],
            acceleration_raw: int,
        ) -> Any:
            raw = {str(name): int(value) for name, value in positions.items()}
            started = time.perf_counter()
            try:
                result = original(
                    positions,
                    speed_raw=speed_raw,
                    acceleration_raw=acceleration_raw,
                )
            except BaseException as exc:
                self._record(
                    "raw_command_error",
                    joints_raw=raw,
                    speed_raw=self._json_speed(speed_raw),
                    acceleration_raw=acceleration_raw,
                    call_ms=(time.perf_counter() - started) * 1000.0,
                    error=repr(exc),
                )
                raise
            self._record(
                "raw_command",
                joints_raw=raw,
                speed_raw=self._json_speed(speed_raw),
                acceleration_raw=acceleration_raw,
                call_ms=(time.perf_counter() - started) * 1000.0,
            )
            return result

        return traced

    def _wrap_read_joint_positions(self, original: Any) -> Any:
        def traced() -> Any:
            started = time.perf_counter()
            result = original()
            joints = {name: float(result[name]) for name in ARM_JOINTS}
            self._feedback_count += 1
            self._record(
                "feedback",
                sequence=self._feedback_count,
                joints_rad=joints,
                joints_raw=self._raw_positions(joints),
                call_ms=(time.perf_counter() - started) * 1000.0,
            )
            return result

        return traced

    def _wrap_get_hardware_state(self, original: Any) -> Any:
        def traced() -> Any:
            started = time.perf_counter()
            result = original()
            self._record(
                "hardware_state",
                connected=bool(result.connected),
                torque_enabled=bool(result.torque_enabled),
                moving=bool(result.moving),
                faulted=bool(result.faulted),
                fault_message=result.fault_message,
                call_ms=(time.perf_counter() - started) * 1000.0,
            )
            return result

        return traced

    def _wrap_write_tool_position(self, original: Any) -> Any:
        def traced(
            actuator: str,
            position: float,
            *,
            speed_raw: int | None = None,
            acceleration_raw: int | None = None,
        ) -> Any:
            started = time.perf_counter()
            result = original(
                actuator,
                position,
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
            )
            self._record(
                "tool_command",
                actuator=actuator,
                position=float(position),
                speed_raw=speed_raw,
                acceleration_raw=acceleration_raw,
                call_ms=(time.perf_counter() - started) * 1000.0,
            )
            return result

        return traced

    def _wrap_read_tool_position(self, original: Any) -> Any:
        def traced(actuator: str) -> Any:
            started = time.perf_counter()
            result = original(actuator)
            self._record(
                "tool_feedback",
                actuator=actuator,
                position=float(result),
                call_ms=(time.perf_counter() - started) * 1000.0,
            )
            return result

        return traced

    def _wrap_read_motor_effort(self, original: Any) -> Any:
        def traced(name: str) -> Any:
            started = time.perf_counter()
            result = original(name)
            self._record(
                "motor_effort",
                motor=name,
                current_raw=result.get("current_raw"),
                load_raw=result.get("load_raw"),
                call_ms=(time.perf_counter() - started) * 1000.0,
            )
            return result

        return traced



def summarize_agent_jog_trace(path: str | Path) -> dict[str, Any]:
    """Summarize model-space trajectory, encoder feedback and HOLD re-latching.

    This is an offline JSONL analysis; it never connects to a robot or reads
    hardware. TCP coordinates are forward-kinematics estimates, not independent
    physical-position measurements.
    """
    trace_path = Path(path).expanduser()
    events = [json.loads(line) for line in trace_path.read_text(encoding="utf-8").splitlines()]
    if not events or events[0].get("event") != "trace_start":
        raise ValueError("not a passive motion trace")
    markers = {
        str(event.get("marker")): (index, event)
        for index, event in enumerate(events)
        if event.get("event") == "marker"
    }
    preflight = markers.get("preflight", (None, {}))[1]
    motion_end = markers.get("motion_completed_before_hold", (None, {}))[1]
    hold_start = markers.get("hold_start", (len(events), {}))[0]
    hold_complete = markers.get("hold_complete", (len(events), {}))[0]
    executed_commands = [
        event for event in events[:hold_start] if event.get("event") == "command"
    ]
    feedback = [
        event for event in events[:hold_start] if event.get("event") == "feedback"
    ]
    raw_before = [
        event for event in events[:hold_start] if event.get("event") == "raw_command"
    ]
    raw_hold = [
        event for event in events[hold_start:hold_complete]
        if event.get("event") == "raw_command"
    ]

    def tcp(event: Mapping[str, Any] | None) -> list[float] | None:
        value = event.get("tcp_xyz_mm") if event is not None else None
        if not isinstance(value, list) or len(value) != 3:
            return None
        return [float(v) for v in value]

    def delta_z(left: list[float] | None, right: list[float] | None) -> float | None:
        return None if left is None or right is None else right[2] - left[2]

    start_tcp = preflight.get("start_model_xyz_mm")
    target_tcp = preflight.get("target_model_xyz_mm")
    first_command = tcp(executed_commands[0]) if executed_commands else None
    last_command = tcp(executed_commands[-1]) if executed_commands else None
    first_feedback = tcp(feedback[0]) if feedback else None
    last_feedback = tcp(feedback[-1]) if feedback else None
    settled_before_hold = tcp(motion_end)
    immediate_after_hold = tcp(markers.get("post_hold_immediate", (None, {}))[1])
    after_2s = tcp(markers.get("post_hold_2s", (None, {}))[1])

    before_raw = raw_before[-1].get("joints_raw") if raw_before else None
    hold_raw = raw_hold[-1].get("joints_raw") if raw_hold else None
    raw_delta = None
    if isinstance(before_raw, dict) and isinstance(hold_raw, dict):
        raw_delta = {
            name: int(hold_raw[name]) - int(before_raw[name])
            for name in before_raw if name in hold_raw
        }

    return {
        "trace_file": str(trace_path),
        "robot_id": events[0].get("robot_id"),
        "calibration_id": events[0].get("calibration_id"),
        "request": {
            "frame": events[0].get("frame"),
            "delta_model_mm": events[0].get("delta_model_mm"),
            "requested_speed_mm_s": events[0].get("requested_speed_mm_s"),
            "start_model_xyz_mm": start_tcp,
            "target_model_xyz_mm": target_tcp,
        },
        "trajectory": {
            "joint_commands": len(executed_commands),
            "joint_feedback_reads": len(feedback),
            "first_command_model_tcp_xyz_mm": first_command,
            "last_command_model_tcp_xyz_mm": last_command,
            "commanded_model_z_change_mm": delta_z(first_command, last_command),
            "observed_model_z_change_mm": delta_z(first_feedback, last_feedback),
            "model_tcp_xyz_mm_at_completion": settled_before_hold,
        },
        "hold": {
            "raw_goal_before_hold": before_raw,
            "raw_goal_latched_by_hold": hold_raw,
            "hold_goal_delta_ticks": raw_delta,
            "model_tcp_xyz_mm_immediate_after_hold": immediate_after_hold,
            "model_tcp_xyz_mm_2s_after_hold": after_2s,
            "model_z_change_over_2s_after_hold_mm": delta_z(immediate_after_hold, after_2s),
        },
        "completed": bool(motion_end.get("completed")),
        "trace_error": events[-1].get("error") if events[-1].get("event") == "trace_end" else "incomplete trace",
        "coordinate_warning": (
            "All TCP coordinates are modeled forward-kinematics estimates, "
            "not direct measurements of physical table clearance."
        ),
    }
