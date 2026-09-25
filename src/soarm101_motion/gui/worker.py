"""Robot session worker that keeps all GUI hardware access off the UI thread."""

from __future__ import annotations

import threading
import time
from math import ceil, degrees, isfinite, radians, sqrt
from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal, Slot

from soarm101_motion import Pose, SOARM101, SOARM101Config
from soarm101_motion.constants import (
    ALL_MOTORS,
    ARM_JOINTS,
    DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
    STOCK_GRIPPER,
    TELEOP_SERVO_ACCELERATION_RAW,
    TELEOP_SERVO_SPEED_RAW,
)
from soarm101_motion.control import relative_target_pose
from soarm101_motion.discovery import discover_so101_arms
from soarm101_motion.exceptions import CommunicationError, MotionCancelledError
from soarm101_motion.motion import MotionHandle
from soarm101_motion.poses import PoseLibrary, SavedPose
from soarm101_motion.primitives import MotionPrimitiveLibrary
from soarm101_motion.sequences import MotionSequence, SequenceRunner
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary
from soarm101_motion.gui.session_log import record as record_session
from soarm101_motion.gui.teleop_rate import (
    GripperContactLatch,
    TELEOP_GRIPPER_SPEED_PER_S,
    gripper_speed_raw,
    limit_joint_target,
    plan_alignment_target,
    update_gripper_contact_latch,
)
from soarm101_motion.exceptions import CalibrationCancelledError, CalibrationError
from soarm101_motion.hardware.simulation import SimulationBackend

GUI_TELEOP_MAX_JOINT_SPEED_RAD_S = 1.2
GUI_TELEOP_MAX_JOINT_ACCELERATION_RAD_S2 = 6.0


class RobotWorker(QObject):
    state_changed = Signal(object)
    connected_changed = Signal(bool)
    busy_changed = Signal(bool)
    log_message = Signal(str)
    error_message = Signal(str)
    calibration_completed = Signal(object)
    calibration_progress = Signal(object)
    calibration_cancelled = Signal()
    recording_completed = Signal(object)
    recording_changed = Signal(bool)
    stream_sample = Signal(object)
    stream_readout_changed = Signal(bool)
    teleop_changed = Signal(bool)
    teleop_alignment_adjusted = Signal(object)
    sequence_progress = Signal(object)
    effort_changed = Signal(object)
    arm_discovery_completed = Signal(object)
    measured_pose_captured = Signal(object)
    teleop_start_pose = Signal(object)
    joint_measurements = Signal(object)
    jog_queue_changed = Signal(object)
    cartesian_jog_diagnostic = Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self.arm: SOARM101 | None = None
        self._timer: QTimer | None = None
        self._handles: list[tuple[str, MotionHandle[Any]]] = []
        self._simulation = False
        self._detailed_logging = True
        self._calibration_cancel = threading.Event()
        self._robot_id = "so101"
        self._record_timer: QTimer | None = None
        self._recording: dict[str, Any] | None = None
        self._record_started = 0.0
        self._stream_timer: QTimer | None = None
        self._stream_readout_active = False
        self._last_voltage_log_s = 0.0
        self._teleop: dict[str, Any] | None = None
        self._teleop_staging: dict[str, Any] | None = None
        self._sequence_runner: SequenceRunner | None = None
        self._last_poll_error: str | None = None
        self._jog_queue: list[dict[str, Any]] = []
        self._jog_active = False
        self._active_jog_start: Pose | None = None
        self._active_jog_target: Pose | None = None
        self._active_jog_command: dict[str, Any] | None = None

    @Slot()
    def start(self) -> None:
        self._timer = QTimer(self)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self.poll)
        self._timer.start()
        self._record_timer = QTimer(self)
        self._record_timer.timeout.connect(self._capture_recording_sample)
        self._stream_timer = QTimer(self)
        self._stream_timer.timeout.connect(self._emit_stream_sample)

    def _report_error(self, operation: str, exc: BaseException) -> None:
        details = [str(exc)]
        cause = exc.__cause__
        while cause is not None:
            if str(cause) not in details:
                details.append(str(cause))
            cause = cause.__cause__
        message = f"{operation}: " + ": ".join(details)
        record_session("error", worker=self._robot_id, operation=operation, message=message)
        self.error_message.emit(message)

    def _record_gripper_snapshot(self, phase: str, *, leader_gripper: float | None = None) -> None:
        """Log raw gripper feedback and goal around teleop staging."""
        arm = self.arm
        backend = None if arm is None else getattr(arm, "backend", None)
        read_raw = getattr(backend, "read_raw_position", None)
        if not callable(read_raw):
            return
        fields: dict[str, object] = {"worker": self._robot_id, "phase": phase}
        if leader_gripper is not None:
            fields["leader_gripper"] = leader_gripper
        try:
            fields["present_raw"] = int(read_raw(STOCK_GRIPPER))
            read_register = getattr(backend, "read_register", None)
            if callable(read_register):
                fields["goal_raw"] = int(read_register(STOCK_GRIPPER, "Goal_Position"))
            calibration = getattr(backend, "calibration", None)
            if calibration is not None and STOCK_GRIPPER in calibration.motors:
                motor = calibration.motors[STOCK_GRIPPER]
                fields["calibrated_closed_raw"] = motor.range_min
                fields["calibrated_open_raw"] = motor.range_max
        except BaseException as exc:
            fields["read_error"] = str(exc)
        record_session("teleop_gripper_snapshot", **fields)

    @Slot()
    def discover_arms(self) -> None:
        """Probe candidate serial ports without enabling torque or changing configuration."""

        if self.arm is not None and self.arm.is_connected:
            self._report_error("find arms", RuntimeError("disconnect the follower before scanning"))
            return
        try:
            self.busy_changed.emit(True)
            self.log_message.emit("Scanning serial ports for SO-101 arms...")
            results = discover_so101_arms()
            self.arm_discovery_completed.emit(results)
            found = sum(1 for item in results if item.get("status") == "ok")
            self.log_message.emit(
                f"Arm discovery complete: {found} SO-101 arm(s) found on {len(results)} candidate port(s)."
            )
        except BaseException as exc:
            self.arm_discovery_completed.emit([])
            self._report_error("find arms", exc)
        finally:
            self.busy_changed.emit(False)

    @Slot(object)
    def capture_measured_pose(self, request: object) -> None:
        """Capture a fresh measured pose for teaching or cross-arm handoff."""

        values = dict(request)  # type: ignore[arg-type]
        source = str(values.get("source") or "follower")
        try:
            pose = SavedPose.capture(self._require_arm(), source=source)
            self.measured_pose_captured.emit(
                {
                    "request": values,
                    "pose": pose,
                    "error": None,
                }
            )
        except BaseException as exc:
            self.measured_pose_captured.emit(
                {
                    "request": values,
                    "pose": None,
                    "error": str(exc),
                }
            )
            self._report_error(f"capture measured {source} pose", exc)

    def _require_arm(self) -> SOARM101:
        if self.arm is None or not self.arm.is_connected:
            raise RuntimeError("robot is not connected")
        return self.arm

    def _require_motion_available(self) -> SOARM101:
        if self._recording is not None:
            raise RuntimeError("stop trajectory recording before commanding this arm")
        arm = self._require_arm()
        if arm.motion.is_streaming:
            raise RuntimeError("stop live teleoperation before commanding this arm")
        return arm

    @staticmethod
    def _pose_diagnostic_payload(pose: Pose) -> dict[str, tuple[float, ...]]:
        xyz_rpy = pose.xyz_rpy()
        return {
            "xyz_mm": tuple(float(value) * 1000.0 for value in xyz_rpy[:3]),
            "rpy_deg": tuple(degrees(float(value)) for value in xyz_rpy[3:]),
        }

    def _emit_jog_queue_state(self) -> None:
        self.jog_queue_changed.emit(
            {
                "active": self._jog_active,
                "queued": len(self._jog_queue),
            }
        )

    def _reset_jog_queue(self) -> None:
        self._jog_queue.clear()
        self._jog_active = False
        self._active_jog_start = None
        self._active_jog_target = None
        self._active_jog_command = None
        self._emit_jog_queue_state()

    def _start_cartesian_jog(self, values: dict[str, Any]) -> None:
        arm = self._require_motion_available()
        current = arm.get_position()
        translation_mm = tuple(float(value) for value in values["translation_mm"])
        rotation_deg = tuple(float(value) for value in values["rotation_rpy_deg"])
        target = relative_target_pose(
            current,
            translation_m=tuple(value / 1000.0 for value in translation_mm),
            rotation_rpy_rad=tuple(radians(value) for value in rotation_deg),
            frame=values["frame"],
        )
        self._active_jog_start = current
        self._active_jog_target = target
        self._active_jog_command = dict(values)
        diagnostic = {
            "phase": "planned",
            "frame": values["frame"],
            "command": dict(values),
            "start": self._pose_diagnostic_payload(current),
            "target": self._pose_diagnostic_payload(target),
        }
        self.cartesian_jog_diagnostic.emit(diagnostic)
        record_session("cartesian_jog_planned", worker=self._robot_id, **diagnostic)
        result = arm.move_linear(
            target,
            orientation_mode=values["orientation_mode"],
            speed=float(values["speed_mm_s"]) / 1000.0,
            acceleration=float(values["acceleration_mm_s2"]) / 1000.0,
            wait=False,
        )
        self._track("cartesian jog", result)
        self._emit_jog_queue_state()

    def _track(self, label: str, result: Any) -> None:
        if isinstance(result, MotionHandle):
            self._handles.append((label, result))
            self.busy_changed.emit(True)
            self.log_message.emit(f"Started {label}.")
            record_session("motion_started", worker=self._robot_id, label=label)
        else:
            self.log_message.emit(f"Completed {label}.")
            record_session("motion_completed", worker=self._robot_id, label=label)
            self.poll()

    def _process_handles(self) -> bool:
        pending: list[tuple[str, MotionHandle[Any]]] = []
        begin_teleop: dict[str, Any] | None = None
        align_gripper: dict[str, Any] | None = None
        next_jog: dict[str, Any] | None = None
        for label, handle in self._handles:
            if not handle.done:
                pending.append((label, handle))
                continue
            try:
                exception = handle.exception(0)
            except BaseException as exc:
                exception = exc
            if exception is not None:
                self._report_error(label, exception)
            else:
                self.log_message.emit(f"Completed {label}.")
                record_session("motion_completed", worker=self._robot_id, label=label)
            if label == "teleop alignment":
                self._record_gripper_snapshot(
                    "joint_alignment_finished" if exception is None else "joint_alignment_failed",
                    leader_gripper=(
                        float(self._teleop_staging["leader_gripper"])
                        if self._teleop_staging is not None
                        and "leader_gripper" in self._teleop_staging else None
                    ),
                )
                options = self._teleop_staging
                if exception is None and options is not None:
                    if options.get("mirror_gripper", True):
                        align_gripper = options
                    else:
                        begin_teleop = {**options, "align_follower": False}
                        self._teleop_staging = None
                elif options is not None:
                    self._teleop_staging = None
                    self.teleop_changed.emit(False)
            if label == "teleop gripper alignment":
                options = self._teleop_staging
                self._teleop_staging = None
                if exception is None and options is not None:
                    begin_teleop = {**options, "align_follower": False}
                elif options is not None:
                    self.teleop_changed.emit(False)
            if label.startswith("sequence "):
                self._sequence_runner = None
                self.sequence_progress.emit(
                    {
                        "type": "sequence_control",
                        "status": "failed" if exception is not None else "completed",
                    }
                )
            if label == "cartesian jog":
                if exception is None and self.arm is not None:
                    achieved = self.arm.get_position()
                    diagnostic = {
                        "phase": "completed",
                        "command": dict(self._active_jog_command or {}),
                        "start": (
                            self._pose_diagnostic_payload(self._active_jog_start)
                            if self._active_jog_start is not None
                            else None
                        ),
                        "target": (
                            self._pose_diagnostic_payload(self._active_jog_target)
                            if self._active_jog_target is not None
                            else None
                        ),
                        "achieved": self._pose_diagnostic_payload(achieved),
                    }
                    self.cartesian_jog_diagnostic.emit(diagnostic)
                    record_session("cartesian_jog_completed", worker=self._robot_id, **diagnostic)
                if exception is not None:
                    self._reset_jog_queue()
                elif self._jog_queue:
                    next_jog = self._jog_queue.pop(0)
                    self._emit_jog_queue_state()
                else:
                    self._jog_active = False
                    self._active_jog_start = None
                    self._active_jog_target = None
                    self._active_jog_command = None
                    self._emit_jog_queue_state()
        self._handles = pending
        if next_jog is not None:
            try:
                self._start_cartesian_jog(next_jog)
            except BaseException as exc:
                self._reset_jog_queue()
                self._report_error("Cartesian jog", exc)
        busy = bool(self._handles)
        self.busy_changed.emit(busy)
        if align_gripper is not None:
            try:
                arm = self._require_arm()
                leader_gripper = float(align_gripper["leader_gripper"])
                follower_gripper = float(arm.tool.get_position())
                record_session(
                    "teleop_gripper_alignment_requested",
                    worker=self._robot_id,
                    leader_gripper=leader_gripper,
                    follower_gripper=follower_gripper,
                )
                if follower_gripper >= leader_gripper - 0.03:
                    self._teleop_staging = None
                    begin_teleop = {**align_gripper, "align_follower": False}
                    if follower_gripper > leader_gripper + 0.03:
                        self.log_message.emit(
                            "Follower gripper will close toward the leader under "
                            "the live contact guard."
                        )
                else:
                    self.log_message.emit(
                        f"Aligning follower gripper {follower_gripper:.3f} → "
                        f"leader {leader_gripper:.3f}…"
                    )
                    self._track(
                        "teleop gripper alignment",
                        arm.tool.move(
                            leader_gripper,
                            speed_raw=int(align_gripper["gripper_speed_raw"]),
                            wait=False,
                            timeout=5.0,
                        ),
                    )
            except BaseException as exc:
                self._teleop_staging = None
                self.teleop_changed.emit(False)
                self._report_error("teleop gripper alignment", exc)
        if begin_teleop is not None:
            self.start_teleop(begin_teleop)
        return bool(self._handles) or begin_teleop is not None

    @Slot(object)
    def connect_robot(self, options: object) -> None:
        try:
            if self.arm is not None:
                self.disconnect_robot()
            values = dict(options)  # type: ignore[arg-type]
            self._simulation = bool(values.get("simulation", False))
            self._robot_id = str(values.get("robot_id") or "so101")
            if self._simulation:
                self.arm = SOARM101(
                    SOARM101Config(
                        teleop_max_joint_speed=GUI_TELEOP_MAX_JOINT_SPEED_RAD_S,
                        teleop_max_joint_acceleration=GUI_TELEOP_MAX_JOINT_ACCELERATION_RAD_S2,
                    ),
                    backend=SimulationBackend(realtime=True),
                )
            else:
                self.arm = SOARM101(
                    SOARM101Config(
                        port=str(values.get("port") or "") or None,
                        robot_id=str(values.get("robot_id") or "so101"),
                        allow_uncalibrated=bool(values.get("allow_uncalibrated", False)),
                        use_stored_calibration=not bool(values.get("allow_uncalibrated", False)),
                        verify_calibration_on_connect=not bool(
                            values.get("allow_uncalibrated", False)
                        ),
                        configure_motors_on_connect=False,
                        enable_workspace_checks=False,
                        teleop_max_joint_speed=GUI_TELEOP_MAX_JOINT_SPEED_RAD_S,
                        teleop_max_joint_acceleration=GUI_TELEOP_MAX_JOINT_ACCELERATION_RAD_S2,
                    )
                )
            self.arm.connect()
            self.connected_changed.emit(True)
            self.log_message.emit(
                "Connected to simulation." if self._simulation else "Connected to SO-ARM101."
            )
            self.poll()
        except BaseException as exc:
            if self.arm is not None:
                try:
                    self.arm.disconnect()
                except Exception:
                    pass
            self.arm = None
            self.connected_changed.emit(False)
            self._report_error("connect", exc)

    @Slot()
    def disconnect_robot(self) -> None:
        arm = self.arm
        self.arm = None
        self._teleop_staging = None
        for _label, handle in self._handles:
            handle.cancel()
        self._handles.clear()
        self._reset_jog_queue()
        self._cancel_recording("recording cancelled by disconnect")
        self._stop_stream_readout()
        self._teleop = None
        self.teleop_changed.emit(False)
        if arm is not None:
            try:
                arm.stop()
            except Exception:
                pass
            try:
                arm.disconnect()
            except BaseException as exc:
                self._report_error("disconnect", exc)
        self.connected_changed.emit(False)
        self.busy_changed.emit(False)
        self.log_message.emit("Disconnected.")

    @Slot()
    def shutdown(self) -> None:
        self.disconnect_robot()
        if self._timer is not None:
            self._timer.stop()
        if self._record_timer is not None:
            self._record_timer.stop()
        if self._stream_timer is not None:
            self._stream_timer.stop()

    @Slot()
    def enable(self) -> None:
        try:
            arm = self._require_arm()
            arm.enable()
            adjustments = getattr(arm.backend, "last_torque_latch_adjustments", {})
            if adjustments:
                details = ", ".join(
                    f"{name} {measured}→{target} ticks "
                    f"({360.0 * abs(target - measured) / 4096.0:.2f}° inward)"
                    for name, (measured, target) in adjustments.items()
                )
                self.log_message.emit(
                    "Torque enabled with a small inward endpoint correction: "
                    f"{details}."
                )
                record_session(
                    "torque_latch_endpoint_adjustment",
                    worker=self._robot_id,
                    corrections=adjustments,
                )
            else:
                self.log_message.emit("Torque enabled after latching current positions.")
            self.poll()
        except BaseException as exc:
            self._report_error("enable", exc)

    @Slot()
    def relax(self) -> None:
        try:
            self._teleop_staging = None
            for _label, handle in self._handles:
                handle.cancel()
            arm = self._require_arm()
            if arm.motion.is_streaming:
                arm.stop_joint_stream(hold=True)
            self._teleop = None
            self.teleop_changed.emit(False)
            arm.relax()
            self._handles.clear()
            self._reset_jog_queue()
            self.log_message.emit("Torque disabled; arm relaxed.")
            self.poll()
        except BaseException as exc:
            self._report_error("relax", exc)

    @Slot()
    def stop(self) -> None:
        try:
            self._teleop_staging = None
            for _label, handle in self._handles:
                handle.cancel()
            arm = self._require_arm()
            self._teleop = None
            self.teleop_changed.emit(False)
            arm.stop()
            self._handles.clear()
            self._reset_jog_queue()
            self.busy_changed.emit(False)
            self.log_message.emit(
                "Software stop: current arm and gripper positions held; Cartesian jog queue cleared."
            )
            self.poll()
        except BaseException as exc:
            self._report_error("stop", exc)

    @Slot(float)
    def start_stream_readout(
        self,
        frequency_hz: float = DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
    ) -> None:
        try:
            self._require_arm()
            frequency = float(frequency_hz)
            if frequency <= 0:
                raise ValueError("stream readout frequency must be positive")
            assert self._stream_timer is not None
            self._stream_timer.setInterval(max(1, round(1000.0 / frequency)))
            self._stream_readout_active = True
            self._last_voltage_log_s = 0.0
            self._emit_stream_sample()
            self._stream_timer.start()
            self.stream_readout_changed.emit(True)
            self.log_message.emit(f"High-rate readout started at {frequency:.1f} Hz.")
        except BaseException as exc:
            self._stop_stream_readout()
            self._report_error("start high-rate readout", exc)

    @Slot()
    def stop_stream_readout(self) -> None:
        self._stop_stream_readout()

    @Slot()
    def read_teleop_start_pose(self) -> None:
        """Read the leader at Start, rather than using the slower GUI display cache."""
        try:
            arm = self._require_arm()
            joints = arm.get_joint_positions().positions
            self.teleop_start_pose.emit({
                "joints_rad": {name: float(joints[name]) for name in ARM_JOINTS},
                "gripper": float(arm.tool.get_position()),
            })
        except BaseException as exc:
            self._record_voltage_fault("read leader for teleoperation", exc)
            self.teleop_start_pose.emit({"error": str(exc)})
            self._report_error("read leader for teleoperation", exc)

    def _record_voltage_fault(self, operation: str, exc: BaseException) -> None:
        """Capture a fresh read-only snapshot after a servo voltage status error."""
        if "input voltage error" not in str(exc).lower() or self.arm is None:
            return
        reader = getattr(self.arm.backend, "read_voltage_snapshot", None)
        if not callable(reader):
            return
        motor = next((name for name in ALL_MOTORS if name in str(exc)), STOCK_GRIPPER)
        try:
            snapshot = reader(motor, include_limits=True)
        except Exception as snapshot_error:
            snapshot = {"motor": motor, "snapshot_error": str(snapshot_error)}
        record_session(
            "servo_voltage_fault",
            worker=self._robot_id,
            operation=operation,
            original_error=str(exc),
            snapshot_after_fault=snapshot,
        )
        voltage = snapshot.get("voltage_v")
        minimum = snapshot.get("minimum_voltage_v")
        maximum = snapshot.get("maximum_voltage_v")
        self.log_message.emit(
            f"Voltage fault snapshot after {operation}: {motor} "
            f"{voltage if voltage is not None else 'unavailable'} V "
            f"(configured {minimum if minimum is not None else '?'}–"
            f"{maximum if maximum is not None else '?'} V). "
            "This reading was taken after the fault; a brief dip may have passed."
        )

    def _stop_stream_readout(self) -> None:
        if self._stream_timer is not None:
            self._stream_timer.stop()
        was_active = self._stream_readout_active
        self._stream_readout_active = False
        if was_active:
            self.stream_readout_changed.emit(False)
            self.log_message.emit("High-rate readout stopped.")

    def _emit_stream_sample(self) -> None:
        if not self._stream_readout_active:
            return
        try:
            arm = self._require_arm()
            joints = arm.get_joint_positions().positions
            gripper = float(arm.tool.get_position())
            now = time.perf_counter()
            if now - self._last_voltage_log_s >= 1.0:
                reader = getattr(arm.backend, "read_voltage_snapshot", None)
                if callable(reader):
                    snapshot = reader(STOCK_GRIPPER)
                    record_session(
                        "leader_voltage_sample", worker=self._robot_id, snapshot=snapshot
                    )
                    reading = snapshot["readings"]["Present_Voltage"]
                    if not reading.get("comm_success") or reading.get("packet_error"):
                        raise RuntimeError(
                            "leader gripper voltage sample failed: "
                            + str(reading.get("error_text") or reading.get("read_error")
                                  or reading.get("comm"))
                        )
                self._last_voltage_log_s = now
            self.stream_sample.emit(
                {
                    "timestamp": time.perf_counter(),
                    "joints_rad": {name: float(joints[name]) for name in ARM_JOINTS},
                    "gripper": gripper,
                }
            )
        except BaseException as exc:
            self._stop_stream_readout()
            self._record_voltage_fault("high-rate readout", exc)
            self._report_error("high-rate readout", exc)

    def _enable_follower_for_teleop(self, arm: SOARM101) -> None:
        """Retry only a lost torque-enable reply, relatching the pose each time."""
        for attempt in range(1, 4):
            try:
                arm.enable()
                return
            except CommunicationError as exc:
                message = str(exc).lower()
                if "write torque_enable" not in message or "no status packet" not in message:
                    raise
                # The write may have reached a motor despite the lost reply.
                # Require all torque-off writes to succeed before relatching.
                arm.backend.disable_torque()
                if attempt == 3:
                    raise
                self.log_message.emit(
                    f"Follower torque-enable reply missing; retrying with fresh "
                    f"position latch ({attempt}/2 retries)."
                )
                record_session(
                    "teleop_torque_enable_retry",
                    worker=self._robot_id,
                    retry=attempt,
                    error=str(exc),
                )

    @Slot(object)
    def start_teleop(self, options: object) -> None:
        try:
            arm = self._require_motion_available()
            if self._handles:
                raise RuntimeError("wait for active motion to finish before teleoperation")
            values = dict(options)  # type: ignore[arg-type]
            selected_gripper_speed = gripper_speed_raw(
                arm.config.hardware_speed_raw,
                float(values.get("gripper_speed_multiplier", 2.0)),
            )
            values["gripper_speed_raw"] = selected_gripper_speed
            mode = str(values.get("mode") or "relative")
            if mode not in {"relative", "absolute"}:
                raise ValueError("teleoperation mode must be relative or absolute")
            frequency = float(
                values.get("frequency_hz", DEFAULT_TELEOP_STREAM_FREQUENCY_HZ)
            )
            if frequency <= 0:
                raise ValueError("teleoperation frequency must be positive")
            if frequency > arm.config.command_frequency_hz:
                raise ValueError(
                    "teleoperation frequency exceeds the configured command-frequency ceiling"
                )
            leader_origin = {
                name: float(values["leader_joints_rad"][name]) for name in ARM_JOINTS
            }
            if bool(values.get("align_follower", False)):
                if bool(values.get("mirror_gripper", True)):
                    leader_gripper = float(values["leader_gripper"])
                    if not isfinite(leader_gripper) or not 0.0 <= leader_gripper <= 1.0:
                        raise ValueError("leader gripper position must be within [0, 1]")
                calibration = getattr(arm.backend, "calibration", None)
                calibrated_limits = (
                    {
                        name: calibration.motors[name].radians_limits
                        for name in ARM_JOINTS
                        if name in calibration.motors
                    }
                    if calibration is not None else None
                )
                alignment_target, offset_rad = plan_alignment_target(
                    leader_origin, arm.get_joint_limits(), calibrated_limits
                )
                adjustments = {name: degrees(abs(value)) for name, value in offset_rad.items()}
                if adjustments:
                    values["mode"] = "relative"
                    self.teleop_alignment_adjusted.emit(adjustments)
                    self.log_message.emit(
                        "Leader extends beyond the conservative motion range: "
                        "aligning to a legal pose and using relative mapping for "
                        + ", ".join(f"{name} ({amount:.1f}°)" for name, amount in adjustments.items())
                        + "."
                    )
                if not arm.get_state().torque_enabled:
                    self._record_gripper_snapshot(
                        "before_torque_enable",
                        leader_gripper=float(values.get("leader_gripper", 0.0)),
                    )
                    self._enable_follower_for_teleop(arm)
                    self._record_gripper_snapshot(
                        "after_torque_enable",
                        leader_gripper=float(values.get("leader_gripper", 0.0)),
                    )
                    self.log_message.emit("Follower holding its current pose before alignment.")
                    self.poll()
                self.log_message.emit("Aligning follower with the measured leader pose…")
                record_session("teleop_alignment_requested", worker=self._robot_id,
                               leader_joints_rad=leader_origin,
                               follower_target_rad=alignment_target,
                               adjustments_deg=adjustments)
                opening_start: float | None = None
                opening_target: float | None = None
                opening_speed_raw: int | None = None
                opening_duration_s: float | None = None
                if bool(values.get("mirror_gripper", True)):
                    opening_target = float(values["leader_gripper"])
                    opening_start = float(arm.tool.get_position())
                    if opening_target <= opening_start + 0.03:
                        if opening_start > opening_target + 0.03:
                            self.log_message.emit(
                                "Follower gripper will close under the live contact guard "
                                "after joint alignment."
                            )
                        opening_target = None
                    else:
                        joint_start = arm.get_joint_positions().positions
                        max_joint_delta = max(
                            abs(alignment_target[name] - joint_start[name])
                            for name in ARM_JOINTS
                        )
                        speed = radians(25.0)
                        acceleration = radians(60.0)
                        estimated_duration = max(
                            0.25,
                            1.875 * max_joint_delta / speed,
                            sqrt(5.774 * max_joint_delta / acceleration),
                        )
                        opening_duration_s = estimated_duration
                        gripper_calibration = getattr(
                            getattr(arm.backend, "calibration", None), "motors", {}
                        ).get(STOCK_GRIPPER)
                        if gripper_calibration is not None:
                            span = gripper_calibration.range_max - gripper_calibration.range_min
                            opening_speed_raw = min(
                                selected_gripper_speed,
                                max(
                                    50,
                                    ceil(
                                        span * (opening_target - opening_start)
                                        / estimated_duration
                                    ),
                                ),
                            )
                result = arm.move_joints(
                    alignment_target,
                    speed=radians(25.0),
                    acceleration=radians(60.0),
                    wait=False,
                    servo_speed_raw=TELEOP_SERVO_SPEED_RAW,
                    servo_acceleration_raw=TELEOP_SERVO_ACCELERATION_RAW,
                )
                if opening_target is not None:
                    try:
                        if result.done:
                            alignment_error = result.exception(0)
                            if alignment_error is not None:
                                raise alignment_error
                        arm.tool.begin_opening(
                            opening_target,
                            speed_raw=opening_speed_raw,
                        )
                    except BaseException:
                        result.cancel()
                        try:
                            arm.stop()
                        except BaseException:
                            pass
                        raise
                    self.log_message.emit(
                        f"Opening follower gripper {opening_start:.3f} → "
                        f"{opening_target:.3f} alongside joint alignment."
                    )
                    record_session(
                        "teleop_gripper_opening_started",
                        worker=self._robot_id,
                        follower_gripper=opening_start,
                        leader_gripper=opening_target,
                        speed_raw=opening_speed_raw,
                        estimated_joint_duration_s=opening_duration_s,
                    )
                self._teleop_staging = values
                self._track("teleop alignment", result)
                return
            if bool(values.get("latch_follower_if_relaxed", False)):
                if not arm.get_state().torque_enabled:
                    self._enable_follower_for_teleop(arm)
                    self.log_message.emit(
                        "Follower parked at its current pose for no-motion relative relink."
                    )
            follower_origin = dict(arm.get_joint_positions().positions)
            follower_gripper = float(arm.tool.get_position())
            stream_joint_limits = arm.get_joint_limits()
            for name in ARM_JOINTS:
                lower, upper = stream_joint_limits[name]
                stream_joint_limits[name] = (
                    min(lower, follower_origin[name]),
                    max(upper, follower_origin[name]),
                )
            period_s = 1.0 / frequency
            self._teleop = {
                "mode": mode,
                # Relative mode latches the first fresh stream sample. Absolute mode
                # also waits for fresh stream data instead of commanding from the
                # slower GUI snapshot used only to verify leader availability.
                "leader_origin": None if mode == "relative" else leader_origin,
                "follower_origin": follower_origin,
                "joint_limits": stream_joint_limits,
                "last_command": dict(follower_origin),
                "last_velocity": {name: 0.0 for name in ARM_JOINTS},
                "gripper_contact": GripperContactLatch(
                    last_command=follower_gripper,
                    last_actual=follower_gripper,
                ),
                "mirror_gripper": bool(values.get("mirror_gripper", True)),
                "gripper_speed_raw": selected_gripper_speed,
                "samples": 0,
                "limited_samples": 0,
                "frequency_hz": frequency,
                "period_s": period_s,
                "overruns": 0,
                "last_processing_s": 0.0,
                "last_sample_timestamp": None,
                "last_frame": None,
            }
            arm.start_joint_stream(frequency_hz=frequency)
            self.teleop_changed.emit(True)
            self.busy_changed.emit(True)
            self.log_message.emit(
                f"Live teleoperation started in {mode} mapping mode at {frequency:.1f} Hz."
            )
            record_session("teleop_started", worker=self._robot_id, mode=mode, frequency_hz=frequency)
            record_session(
                "teleop_settings",
                worker=self._robot_id,
                max_joint_speed_rad_s=arm.config.stream_joint_speed_limit,
                max_joint_acceleration_rad_s2=arm.config.stream_joint_acceleration_limit,
                gripper_speed_per_s=TELEOP_GRIPPER_SPEED_PER_S,
                gripper_speed_raw=selected_gripper_speed,
                max_command_step_rad=arm.config.max_command_step_radians,
                following_error_limit_rad=arm.config.following_error_limit_rad,
                teleop_servo_speed_raw=TELEOP_SERVO_SPEED_RAW,
                teleop_servo_acceleration_raw=TELEOP_SERVO_ACCELERATION_RAW,
            )
        except BaseException as exc:
            self._teleop_staging = None
            self._teleop = None
            try:
                arm = self.arm
                if arm is not None and arm.motion.is_streaming:
                    arm.stop_joint_stream(hold=True)
            except Exception:
                pass
            self.teleop_changed.emit(False)
            self.busy_changed.emit(False)
            self._report_error("start teleoperation", exc)

    @Slot(object)
    def apply_teleop_sample(self, sample: object) -> None:
        teleop = self._teleop
        if teleop is None:
            return
        teleop["last_frame"] = None
        try:
            started = time.perf_counter()
            values = dict(sample)  # type: ignore[arg-type]
            sample_timestamp = float(values.get("timestamp", started))
            sample_age_s = max(0.0, started - sample_timestamp)
            stale_limit_s = max(0.15, 3.0 * float(teleop["period_s"]))
            if sample_age_s > stale_limit_s:
                raise RuntimeError(
                    f"leader sample is {sample_age_s * 1000.0:.0f} ms old; "
                    f"teleop stale limit is {stale_limit_s * 1000.0:.0f} ms. "
                    "Follower held to avoid executing a queued command backlog."
                )
            leader = {
                name: float(values["joints_rad"][name]) for name in ARM_JOINTS
            }
            if teleop["mode"] == "relative":
                if teleop["leader_origin"] is None:
                    teleop["leader_origin"] = dict(leader)
                target = {
                    name: teleop["follower_origin"][name]
                    + (leader[name] - teleop["leader_origin"][name])
                    for name in ARM_JOINTS
                }
            else:
                target = leader
            gripper = float(values["gripper"]) if teleop["mirror_gripper"] else None
            arm = self._require_arm()
            command, velocity, limited = limit_joint_target(
                target,
                teleop["last_command"],
                teleop["last_velocity"],
                joint_limits=teleop["joint_limits"],
                period_s=float(teleop["period_s"]),
                max_speed_rad_s=arm.config.stream_joint_speed_limit,
                max_acceleration_rad_s2=arm.config.stream_joint_acceleration_limit,
                max_step_rad=arm.config.max_command_step_radians,
            )
            gripper_actual = None
            gripper_desired = gripper
            gripper_contact_latched = False
            if gripper is not None:
                gripper_desired = gripper
                gripper_actual = float(arm.tool.get_position())
                (
                    gripper,
                    gripper_contact_latched,
                    newly_latched,
                    latch_released,
                ) = update_gripper_contact_latch(
                    teleop["gripper_contact"],
                    gripper_desired,
                    gripper_actual,
                    period_s=float(teleop["period_s"]),
                    max_speed_per_s=TELEOP_GRIPPER_SPEED_PER_S,
                )
                contact_hook = getattr(
                    getattr(arm, "backend", None),
                    "set_gripper_contact_latched",
                    None,
                )
                if newly_latched:
                    if callable(contact_hook):
                        contact_hook(True)
                    self.log_message.emit(
                        f"Gripper contact detected at {gripper_actual:.3f}; "
                        f"easing to {gripper:.3f} and holding. "
                        "Open the leader gripper to release."
                    )
                    record_session(
                        "teleop_gripper_contact_latched",
                        worker=self._robot_id,
                        aperture=gripper_actual,
                        hold_aperture=gripper,
                        leader_target=gripper_desired,
                    )
                elif latch_released:
                    if callable(contact_hook):
                        contact_hook(False)
                    self.log_message.emit("Leader gripper opened; contact hold released.")
                    record_session(
                        "teleop_gripper_contact_released",
                        worker=self._robot_id,
                        aperture=gripper_actual,
                        leader_target=gripper_desired,
                    )
            prior_timestamp = teleop["last_sample_timestamp"]
            frame = {
                "worker": self._robot_id,
                "sample": teleop["samples"] + 1,
                "leader_timestamp": sample_timestamp,
                "interval_ms": None if prior_timestamp is None else (sample_timestamp - prior_timestamp) * 1000.0,
                "sample_age_ms": sample_age_s * 1000.0,
                "leader_joints_rad": leader,
                "desired_joints_rad": target,
                "command_joints_rad": command,
                "command_velocity_rad_s": velocity,
                "gripper_command": gripper,
                "leader_gripper": float(values["gripper"]),
                "gripper_desired": gripper_desired,
                "gripper_actual": gripper_actual,
                "gripper_contact_latched": gripper_contact_latched,
                "limited": limited,
            }
            teleop["last_frame"] = frame
            if gripper is None:
                result = arm.stream_joint_target(command, gripper=None)
            else:
                result = arm.stream_joint_target(
                    command, gripper=gripper, gripper_speed_raw=teleop["gripper_speed_raw"]
                )
            self.joint_measurements.emit({
                name: degrees(float(result.final_positions[name])) for name in ARM_JOINTS
            })
            teleop["last_sample_timestamp"] = sample_timestamp
            teleop["last_command"] = command
            teleop["last_velocity"] = velocity
            if limited:
                teleop["limited_samples"] += 1
            processing_s = time.perf_counter() - started
            if self._detailed_logging:
                actual = dict(result.final_positions)
                record_session(
                    "teleop_frame",
                    **frame,
                    actual_joints_rad=actual,
                    following_error_rad={
                        name: actual[name] - command[name] for name in ARM_JOINTS
                    },
                    processing_ms=processing_s * 1000.0,
                )
            teleop["last_processing_s"] = processing_s
            teleop["samples"] += 1
            if processing_s > float(teleop["period_s"]):
                teleop["overruns"] += 1
            else:
                teleop["overruns"] = 0
            if teleop["overruns"] >= 3:
                raise RuntimeError(
                    "follower teleop processing exceeded the selected stream period "
                    "for three consecutive samples; reduce the teleop rate before retrying"
                )
            if teleop["samples"] % 5 == 0:
                self.sequence_progress.emit(
                    {
                        "type": "teleop",
                        "samples": teleop["samples"],
                        "message": result.message,
                        "frequency_hz": teleop["frequency_hz"],
                        "processing_ms": processing_s * 1000.0,
                        "sample_age_ms": sample_age_s * 1000.0,
                        "overruns": teleop["overruns"],
                        "limited_samples": teleop["limited_samples"],
                        "gripper_contact_latched": gripper_contact_latched,
                    }
                )
                record_session(
                    "teleop_sample",
                    worker=self._robot_id,
                    samples=teleop["samples"],
                    processing_ms=processing_s * 1000.0,
                    sample_age_ms=sample_age_s * 1000.0,
                    limited_samples=teleop["limited_samples"],
                )
        except BaseException as exc:
            if self._detailed_logging and teleop.get("last_frame") is not None:
                record_session(
                    "teleop_fault_context",
                    **teleop["last_frame"],
                    error=str(exc),
                )
            record_session("teleop_stopped_on_error", worker=self._robot_id, message=str(exc))
            self._stop_teleop_internal(hold=True)
            self._report_error("live teleoperation", exc)

    @Slot()
    def stop_teleop(self) -> None:
        if self._teleop_staging is not None:
            self._teleop_staging = None
            for label, handle in self._handles:
                if label in {"teleop alignment", "teleop gripper alignment"}:
                    handle.cancel()
            self.teleop_changed.emit(False)
            self.log_message.emit("Teleoperation alignment cancelled; follower holding.")
            return
        self._stop_teleop_internal(hold=True)

    def _stop_teleop_internal(self, *, hold: bool) -> None:
        arm = self.arm
        was_active = self._teleop is not None
        self._teleop = None
        if arm is not None and arm.is_connected and (was_active or arm.motion.is_streaming):
            try:
                arm.stop_joint_stream(hold=hold)
            except BaseException as exc:
                self._report_error("stop teleoperation", exc)
            finally:
                contact_hook = getattr(
                    getattr(arm, "backend", None),
                    "set_gripper_contact_latched",
                    None,
                )
                if callable(contact_hook):
                    contact_hook(False)
        if was_active:
            self.teleop_changed.emit(False)
            self.busy_changed.emit(bool(self._handles))
            self.log_message.emit("Live teleoperation stopped.")
            record_session("teleop_stopped", worker=self._robot_id, hold=hold)

    @Slot(object)
    def run_sequence(self, command: object) -> None:
        try:
            values = dict(command)  # type: ignore[arg-type]
            sequence = values["sequence"]
            if not isinstance(sequence, MotionSequence):
                raise TypeError("sequence command must contain a MotionSequence")
            arm = self._require_motion_available()
            arm.require_artifact_calibration(
                sequence.metadata,
                artifact_label=f"sequence {sequence.name!r}",
            )
            selected_gripper_speed = gripper_speed_raw(
                arm.config.hardware_speed_raw,
                float(values.get("gripper_speed_multiplier", 2.0)),
            )
            runner = SequenceRunner(
                arm,
                pose_library=PoseLibrary(self._robot_id),
                trajectory_library=TrajectoryLibrary(self._robot_id),
                primitive_library=MotionPrimitiveLibrary(self._robot_id),
                gripper_speed_raw=selected_gripper_speed,
            )

            def progress(index: int, total: int, step: object, status: str) -> None:
                self.sequence_progress.emit(
                    {
                        "type": "sequence",
                        "index": index,
                        "total": total,
                        "status": status,
                        "kind": getattr(step, "kind", "unknown"),
                    }
                )

            self._sequence_runner = runner
            result = runner.run(
                sequence,
                repeat=int(values.get("repeat", 1)),
                speed_scale=float(values.get("speed_scale", 1.0)),
                start_index=int(values.get("start_index", 0)),
                stop_index=values.get("stop_index"),
                on_progress=progress,
                wait=False,
            )
            self._track(f"sequence {sequence.name}", result)
        except BaseException as exc:
            self._report_error("run sequence", exc)

    @Slot()
    def pause_sequence(self) -> None:
        runner = self._sequence_runner
        if runner is None:
            return
        runner.pause()
        self.sequence_progress.emit({"type": "sequence_control", "status": "paused"})
        self.log_message.emit(
            "Sequence pause requested; it takes effect at the next step boundary."
        )

    @Slot()
    def resume_sequence(self) -> None:
        runner = self._sequence_runner
        if runner is None:
            return
        runner.resume()
        self.sequence_progress.emit({"type": "sequence_control", "status": "running"})
        self.log_message.emit("Sequence resumed.")

    @Slot(object)
    def move_saved_pose(self, command: object) -> None:
        try:
            values = dict(command)  # type: ignore[arg-type]
            pose = values["pose"]
            if not isinstance(pose, SavedPose):
                raise TypeError("pose command must contain a SavedPose")
            mode = str(values.get("mode") or "joint")
            arm = self._require_motion_available()
            arm.require_artifact_calibration(
                {
                    "source_robot_id": pose.source_robot_id,
                    "source_calibration_id": pose.source_calibration_id,
                    "target_robot_id": pose.target_robot_id,
                    "target_calibration_id": pose.target_calibration_id,
                },
                artifact_label="saved pose",
            )
            move_gripper = bool(values.get("move_gripper", False))

            def execute_pose(*, wait: bool) -> Any:
                if mode == "joint":
                    return arm.move_joints(
                        pose.joints,
                        speed=radians(float(values["speed_deg_s"])),
                        acceleration=radians(float(values["acceleration_deg_s2"])),
                        wait=wait,
                    )
                if mode == "linear":
                    target = Pose.from_xyz_rpy(*pose.tcp_xyz_rpy)
                    return arm.move_linear(
                        target,
                        orientation_mode=str(
                            values.get("orientation_mode") or "compatible"
                        ),
                        speed=float(values["speed_mm_s"]) / 1000.0,
                        acceleration=float(values["acceleration_mm_s2"]) / 1000.0,
                        wait=wait,
                    )
                raise ValueError("saved pose mode must be 'joint' or 'linear'")

            if not move_gripper:
                self._track(f"{mode} move to taught point", execute_pose(wait=False))
                return

            def operation(cancel_event: threading.Event) -> Any:
                if cancel_event.is_set():
                    raise MotionCancelledError("saved pose move cancelled before start")
                result = execute_pose(wait=True)
                if cancel_event.is_set():
                    raise MotionCancelledError(
                        "saved pose move cancelled before gripper command"
                    )
                selected_gripper_speed = gripper_speed_raw(
                    arm.config.hardware_speed_raw,
                    float(values.get("gripper_speed_multiplier", 2.0)),
                )
                return (
                    arm.tool.move(
                        pose.gripper,
                        speed_raw=selected_gripper_speed,
                        wait=True,
                    )
                    or result
                )

            handle: MotionHandle[Any] = MotionHandle(operation)
            handle.start()
            self._track(f"{mode} move to saved pose + gripper", handle)
        except BaseException as exc:
            self._report_error("taught point move", exc)

    @Slot(object)
    def play_trajectory(self, command: object) -> None:
        try:
            values = dict(command)  # type: ignore[arg-type]
            trajectory = values["trajectory"]
            if not isinstance(trajectory, Trajectory):
                raise TypeError("trajectory command must contain a Trajectory")
            arm = self._require_motion_available()
            arm.require_artifact_calibration(
                trajectory.metadata,
                artifact_label="recorded trajectory",
            )
            selected_gripper_speed = gripper_speed_raw(
                arm.config.hardware_speed_raw,
                float(values.get("gripper_speed_multiplier", 2.0)),
            )
            result = arm.play_trajectory(
                trajectory,
                speed_scale=float(values.get("speed_scale", 1.0)),
                move_to_start=bool(values.get("move_to_start", True)),
                gripper_speed_raw=selected_gripper_speed,
                wait=False,
            )
            self._track("recorded trajectory replay", result)
        except BaseException as exc:
            self._report_error("trajectory replay", exc)

    def _read_recording_effort(self, arm: SOARM101) -> tuple[list[float], list[float]]:
        backend = arm.backend
        reader = getattr(backend, "read_motor_effort", None)
        if callable(reader):
            current: list[float] = []
            load: list[float] = []
            for name in ALL_MOTORS:
                value = reader(name)
                current.append(float(value["current_raw"]))
                load.append(float(value["load_raw"]))
            return current, load

        diagnostics = {item.name: item for item in backend.diagnostics()}
        current = [
            float(diagnostics[name].current_raw)
            if diagnostics[name].current_raw is not None
            else float("nan")
            for name in ALL_MOTORS
        ]
        return current, [float("nan")] * len(ALL_MOTORS)

    @Slot(object)
    def start_recording(self, options: object) -> None:
        try:
            arm = self._require_arm()
            if arm.motion.is_streaming:
                raise RuntimeError("stop live teleoperation before recording")
            if self._recording is not None:
                raise RuntimeError("a trajectory recording is already active")
            if self._handles:
                raise RuntimeError("wait for active motion to finish before recording")
            values = dict(options)  # type: ignore[arg-type]
            frequency = float(values.get("frequency_hz", 50.0))
            if frequency <= 0:
                raise ValueError("recording frequency must be positive")
            self._recording = {
                "frequency_hz": frequency,
                "record_effort": bool(values.get("record_effort", False)),
                "source": str(values.get("source") or "unknown"),
                "source_robot_id": arm.config.robot_id,
                "source_calibration_id": arm.calibration_id,
                "timestamps_s": [],
                "joints_rad": [],
                "gripper": [],
                "effort_current_raw": [],
                "effort_load_raw": [],
            }
            self._record_started = time.perf_counter()
            assert self._record_timer is not None
            self._record_timer.setInterval(max(1, round(1000.0 / frequency)))
            self._capture_recording_sample()
            self._record_timer.start()
            self.recording_changed.emit(True)
            self.log_message.emit(
                f"Recording {self._recording['source']} trajectory at {frequency:.1f} Hz."
            )
        except BaseException as exc:
            self._recording = None
            self.recording_changed.emit(False)
            self._report_error("start trajectory recording", exc)

    @Slot()
    def stop_recording(self) -> None:
        recording = self._recording
        if recording is None:
            return
        try:
            if self._record_timer is not None:
                self._record_timer.stop()
            self._capture_recording_sample()
            recording = self._recording
            assert recording is not None
            if len(recording["timestamps_s"]) < 2:
                raise RuntimeError("trajectory recording captured fewer than two samples")
            trajectory = Trajectory(
                timestamps_s=recording["timestamps_s"],
                joints_rad=recording["joints_rad"],
                gripper=recording["gripper"],
                effort_current_raw=(
                    recording["effort_current_raw"]
                    if recording["record_effort"]
                    else None
                ),
                effort_load_raw=(
                    recording["effort_load_raw"]
                    if recording["record_effort"]
                    else None
                ),
                metadata={
                    "source": recording["source"],
                    "robot_id": self._robot_id,
                    "source_robot_id": recording["source_robot_id"],
                    "source_calibration_id": recording["source_calibration_id"],
                    "requested_sample_rate_hz": recording["frequency_hz"],
                    "effort_recorded": recording["record_effort"],
                },
            )
            self.recording_completed.emit(trajectory)
            self.log_message.emit(
                f"Recorded {trajectory.sample_count} samples over "
                f"{trajectory.duration_s:.3f} s "
                f"(median {trajectory.sample_rate_hz:.1f} Hz)."
            )
        except BaseException as exc:
            self._report_error("stop trajectory recording", exc)
        finally:
            self._recording = None
            self.recording_changed.emit(False)

    def _capture_recording_sample(self) -> None:
        recording = self._recording
        if recording is None:
            return
        try:
            arm = self._require_arm()
            now = time.perf_counter() - self._record_started
            if recording["timestamps_s"] and now <= recording["timestamps_s"][-1]:
                return
            joints = arm.get_joint_positions().positions
            recording["timestamps_s"].append(now)
            recording["joints_rad"].append([float(joints[name]) for name in ARM_JOINTS])
            recording["gripper"].append(float(arm.tool.get_position()))
            if recording["record_effort"]:
                current, load = self._read_recording_effort(arm)
                recording["effort_current_raw"].append(current)
                recording["effort_load_raw"].append(load)
        except BaseException as exc:
            self._cancel_recording(f"recording failed: {exc}")
            self._report_error("trajectory recording", exc)

    def _cancel_recording(self, message: str) -> None:
        if self._recording is None:
            return
        if self._record_timer is not None:
            self._record_timer.stop()
        self._recording = None
        self.recording_changed.emit(False)
        self.log_message.emit(message)

    @Slot(object)
    def move_joints(self, command: object) -> None:
        try:
            values = dict(command)  # type: ignore[arg-type]
            record_session("move_joints_requested", worker=self._robot_id, command=values)
            positions = {
                name: radians(float(values["joints_deg"][name])) for name in ARM_JOINTS
            }
            result = self._require_motion_available().move_joints(
                positions,
                speed=radians(float(values["speed_deg_s"])),
                acceleration=radians(float(values["acceleration_deg_s2"])),
                wait=False,
            )
            self._track("joint move", result)
        except BaseException as exc:
            self._report_error("joint move", exc)

    @Slot(object)
    def jog_cartesian(self, command: object) -> None:
        try:
            values = dict(command)  # type: ignore[arg-type]
            if self._jog_active:
                if len(self._jog_queue) >= 32:
                    raise RuntimeError("Cartesian jog queue is full (32 waiting commands)")
                self._jog_queue.append(values)
                self._emit_jog_queue_state()
                self.log_message.emit(
                    f"Queued Cartesian jog; {len(self._jog_queue)} waiting."
                )
                record_session(
                    "cartesian_jog_queued",
                    worker=self._robot_id,
                    queued=len(self._jog_queue),
                    command=values,
                )
                return
            if self._handles:
                raise RuntimeError("another motion is active; wait before starting Cartesian jogs")
            self._jog_active = True
            self._emit_jog_queue_state()
            self._start_cartesian_jog(values)
        except BaseException as exc:
            if not self._handles:
                self._reset_jog_queue()
            self._report_error("Cartesian jog", exc)

    @Slot(object)
    def move_absolute_pose(self, command: object) -> None:
        try:
            values = dict(command)  # type: ignore[arg-type]
            xyz = values["xyz_mm"]
            rpy = values["rpy_deg"]
            target = Pose.from_xyz_rpy(
                float(xyz[0]) / 1000.0,
                float(xyz[1]) / 1000.0,
                float(xyz[2]) / 1000.0,
                radians(float(rpy[0])),
                radians(float(rpy[1])),
                radians(float(rpy[2])),
            )
            result = self._require_motion_available().move_linear(
                target,
                orientation_mode=values["orientation_mode"],
                speed=float(values["speed_mm_s"]) / 1000.0,
                acceleration=float(values["acceleration_mm_s2"]) / 1000.0,
                wait=False,
            )
            self._track("absolute world linear move", result)
        except BaseException as exc:
            self._report_error("absolute Cartesian move", exc)

    @Slot(object)
    def synchronize_pose(self, request: object) -> None:
        """Move this arm to a fresh measured pose from the other arm.

        This is deliberately a live handoff operation, not artifact replay. The
        destination arm applies its own calibration, limits, workspace checks, and
        normal motion guards. A relaxed destination is first latched where it is,
        then moved under torque.
        """

        try:
            values = dict(request)  # type: ignore[arg-type]
            if self._handles:
                raise RuntimeError("wait for active motion to finish before matching poses")
            arm = self._require_motion_available()
            target = {
                name: radians(float(values["joints_deg"][name]))
                for name in ARM_JOINTS
            }
            include_gripper = bool(values.get("include_gripper", False))
            target_gripper = float(values.get("gripper", 0.0))
            if include_gripper and not 0.0 <= target_gripper <= 1.0:
                raise ValueError("synchronization gripper target must be within [0, 1]")

            speed = radians(float(values.get("speed_deg_s", 8.0)))
            acceleration = radians(float(values.get("acceleration_deg_s2", 25.0)))
            selected_gripper_speed = gripper_speed_raw(
                arm.config.hardware_speed_raw,
                float(values.get("gripper_speed_multiplier", 2.0)),
            )
            source = str(values.get("source") or "other arm")
            if not arm.get_state().torque_enabled:
                arm.enable()
                self.log_message.emit(
                    "Destination arm parked at its measured pose before synchronization."
                )

            record_session(
                "live_pose_sync_requested",
                worker=self._robot_id,
                source=source,
                target_joints_deg={
                    name: float(values["joints_deg"][name]) for name in ARM_JOINTS
                },
                include_gripper=include_gripper,
                target_gripper=target_gripper if include_gripper else None,
                gripper_speed_raw=selected_gripper_speed if include_gripper else None,
            )

            def operation(cancel_event: threading.Event) -> Any:
                if cancel_event.is_set():
                    raise MotionCancelledError("pose synchronization cancelled before start")
                result = arm.move_joints(
                    target,
                    speed=speed,
                    acceleration=acceleration,
                    wait=True,
                )
                if cancel_event.is_set():
                    raise MotionCancelledError(
                        "pose synchronization cancelled before gripper command"
                    )
                if include_gripper:
                    arm.tool.move(
                        target_gripper,
                        speed_raw=selected_gripper_speed,
                        wait=True,
                    )
                return result

            handle: MotionHandle[Any] = MotionHandle(operation)
            handle.start()
            self._track(f"match pose from {source}", handle)
        except BaseException as exc:
            self._report_error("match arm pose", exc)

    @Slot(object)
    def move_gripper(self, request: object) -> None:
        try:
            values = dict(request) if isinstance(request, dict) else {"position": request}
            position = float(values["position"])
            arm = self._require_motion_available()
            speed = gripper_speed_raw(
                arm.config.hardware_speed_raw,
                float(values.get("gripper_speed_multiplier", 2.0)),
            )
            record_session(
                "gripper_requested", worker=self._robot_id, position=position, speed_raw=speed
            )
            result = arm.tool.move(position, speed_raw=speed, wait=False)
            self._track("gripper move", result)
        except BaseException as exc:
            self._report_error("gripper", exc)

    @Slot(float)
    def run_calibration(self, seconds: float) -> None:
        last_progress: object | None = None
        last_progress_log = 0.0
        try:
            arm = self._require_arm()
            if self._simulation:
                raise RuntimeError("live mechanical-stop calibration requires physical hardware")
            if arm.get_state().torque_enabled:
                arm.relax()
            backend = arm.backend
            calibrate = getattr(backend, "interactive_calibration", None)
            save = getattr(backend, "save_calibration", None)
            if not callable(calibrate) or not callable(save):
                raise RuntimeError("active backend does not support live calibration")
            previous_motor_calibration = backend.read_calibration_from_motors()
            previous_runtime_calibration = getattr(backend, "calibration", None)
            duration = float(seconds)
            if duration <= 0:
                raise ValueError("calibration duration must be positive")
            self.busy_changed.emit(True)
            self.log_message.emit(
                "Preparing calibration with torque off; resetting temporary motor ranges."
            )
            self.calibration_progress.emit({"phase": "preparing", "duration_s": duration})
            recording_started: float | None = None

            def report_progress(progress: object) -> None:
                nonlocal recording_started, last_progress, last_progress_log
                last_progress = progress
                if recording_started is None:
                    recording_started = time.monotonic()
                    self.log_message.emit(
                        f"Calibration sweep recording started; time limit {duration:.1f} s. "
                        "It will finish early when all six actuators reach 2/2."
                    )
                self.calibration_progress.emit(
                    {
                        "phase": "recording",
                        "progress": progress,
                        "duration_s": duration,
                        "remaining_s": max(
                            0.0, duration - (time.monotonic() - recording_started)
                        ),
                    }
                )
                now = time.monotonic()
                if now - last_progress_log >= 1.0:
                    record_session(
                        "calibration_progress",
                        worker=self._robot_id,
                        elapsed_s=now - recording_started,
                        motors=progress,
                    )
                    last_progress_log = now

            calibration = calibrate(
                record_seconds=duration,
                progress_callback=report_progress,
                cancel_event=self._calibration_cancel,
            )
            try:
                path = save(calibration)
            except BaseException as save_exc:
                try:
                    backend.apply_calibration(previous_motor_calibration)
                    restored = backend.read_calibration_from_motors()
                    backend._verify_calibration_matches_motors(
                        previous_motor_calibration, restored
                    )
                    backend.calibration = (
                        previous_runtime_calibration or previous_motor_calibration
                    )
                except BaseException as rollback_exc:
                    raise CalibrationError(
                        "calibration save failed and EEPROM rollback also failed; "
                        "do not enable torque"
                    ) from rollback_exc
                raise CalibrationError(
                    "calibration save failed; previous motor calibration restored "
                    "and saved calibration file kept"
                ) from save_exc
            limits = arm.get_joint_limits()
            payload = {
                "path": str(path),
                "source": calibration.source,
                "robot_id": arm.config.robot_id,
                "calibration_id": calibration.calibration_id,
                "joint_limits_deg": {
                    name: (degrees(bounds[0]), degrees(bounds[1]))
                    for name, bounds in limits.items()
                },
            }
            self.calibration_completed.emit(payload)
            self.log_message.emit(f"Calibration saved to {path}.")
            self.poll()
        except CalibrationCancelledError:
            self.calibration_cancelled.emit()
            self.log_message.emit("Calibration cancelled; previous motor calibration restored.")
        except BaseException as exc:
            restored = "rollback also failed" not in str(exc)
            if last_progress is not None:
                record_session(
                    "calibration_failed_context",
                    worker=self._robot_id,
                    motors=last_progress,
                    error=str(exc),
                    previous_calibration_restored=restored,
                )
            if restored:
                self.log_message.emit(
                    "Calibration did not replace the saved file; previous motor settings restored."
                )
            self._report_error("calibration", exc)
        finally:
            self.busy_changed.emit(False)

    def prepare_calibration(self) -> None:
        """Reset cancellation before queuing a new sweep."""
        self._calibration_cancel.clear()

    def request_calibration_cancel(self) -> None:
        """Signal the active sweep directly; its worker event loop is blocked."""
        self._calibration_cancel.set()

    @Slot(bool)
    def set_detailed_logging(self, enabled: bool) -> None:
        self._detailed_logging = bool(enabled)

    def _emit_effort_status(self, *, refresh: bool = False) -> None:
        if self.arm is None or not self.arm.is_connected:
            return
        try:
            self.effort_changed.emit(self.arm.get_effort_safety_status(refresh=refresh))
        except BaseException as exc:
            self._report_error("read effort safety", exc)

    @Slot()
    def refresh_effort(self) -> None:
        try:
            arm = self._require_motion_available()
            self.effort_changed.emit(arm.get_effort_safety_status(refresh=True))
        except BaseException as exc:
            self._report_error("refresh effort", exc)

    @Slot()
    def clear_effort_trip(self) -> None:
        try:
            arm = self._require_arm()
            arm.clear_effort_trip()
            self.log_message.emit("Cleared latched motor-effort safety trip.")
            self.effort_changed.emit(arm.get_effort_safety_status(refresh=False))
            self.poll()
        except BaseException as exc:
            self._report_error("clear effort trip", exc)

    @Slot()
    def reset_effort_peaks(self) -> None:
        try:
            arm = self._require_arm()
            arm.reset_effort_peaks()
            self.log_message.emit("Reset session motor-effort peaks.")
            self.effort_changed.emit(arm.get_effort_safety_status(refresh=False))
        except BaseException as exc:
            self._report_error("reset effort peaks", exc)

    @Slot(object)
    def configure_effort_safety(self, options: object) -> None:
        try:
            values = dict(options)  # type: ignore[arg-type]
            arm = self._require_arm()
            arm.configure_effort_safety(
                enabled=bool(values.get("enabled", True)),
                current_trip_raw=(
                    None
                    if int(values.get("current_trip_raw", 0)) <= 0
                    else int(values["current_trip_raw"])
                ),
                load_trip_raw=(
                    None
                    if int(values.get("load_trip_raw", 0)) <= 0
                    else int(values["load_trip_raw"])
                ),
                consecutive_samples=int(values.get("consecutive_samples", 2)),
            )
            self.log_message.emit(
                "Applied session-only motor-effort safety settings (torque remains off)."
            )
            self.effort_changed.emit(arm.get_effort_safety_status(refresh=False))
        except BaseException as exc:
            self._report_error("configure effort safety", exc)

    @Slot()
    def poll(self) -> None:
        if self.arm is not None and self.arm.is_connected and self.arm.motion.is_streaming:
            self._emit_effort_status(refresh=False)
            return
        if self._process_handles():
            # The SDK's motion thread already owns feedback polling. Avoid competing
            # serial traffic that could cause host-side command deadline misses.
            self._emit_effort_status(refresh=False)
            return
        if self.arm is None or not self.arm.is_connected:
            return
        try:
            state = self.arm.get_state()
            joints = self.arm.get_joint_positions().positions
            pose = self.arm.get_position().xyz_rpy()
            gripper = self.arm.tool.get_position()
            payload = {
                "connected": state.connected,
                "torque_enabled": state.torque_enabled,
                "moving": state.moving,
                "faulted": state.faulted,
                "fault_message": state.fault_message,
                "simulation": self._simulation,
                "joints_deg": {name: degrees(joints[name]) for name in ARM_JOINTS},
                "pose_mm_deg": (
                    pose[0] * 1000.0,
                    pose[1] * 1000.0,
                    pose[2] * 1000.0,
                    degrees(pose[3]),
                    degrees(pose[4]),
                    degrees(pose[5]),
                ),
                "gripper": gripper,
                "robot_id": self.arm.config.robot_id,
                "calibration_id": self.arm.calibration_id,
                "calibration_source": self.arm.calibration_source,
                "joint_limits_deg": {
                    name: (degrees(bounds[0]), degrees(bounds[1]))
                    for name, bounds in self.arm.get_joint_limits().items()
                },
            }
            self.state_changed.emit(payload)
            self._emit_effort_status(refresh=False)
            self._last_poll_error = None
        except BaseException as exc:
            message = str(exc)
            if message != self._last_poll_error:
                self._last_poll_error = message
                self._report_error("read state", exc)
