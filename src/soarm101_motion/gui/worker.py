"""Robot session worker that keeps all GUI hardware access off the UI thread."""

from __future__ import annotations

import time
from math import degrees, radians
from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal, Slot

from soarm101_motion import Pose, SOARM101, SOARM101Config
from soarm101_motion.constants import ALL_MOTORS, ARM_JOINTS
from soarm101_motion.control import jog_linear_cli_units
from soarm101_motion.motion import MotionHandle
from soarm101_motion.poses import PoseLibrary, SavedPose
from soarm101_motion.primitives import MotionPrimitiveLibrary
from soarm101_motion.sequences import MotionSequence, SequenceRunner
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary


class RobotWorker(QObject):
    state_changed = Signal(object)
    connected_changed = Signal(bool)
    busy_changed = Signal(bool)
    log_message = Signal(str)
    error_message = Signal(str)
    calibration_completed = Signal(object)
    recording_completed = Signal(object)
    recording_changed = Signal(bool)
    stream_sample = Signal(object)
    stream_readout_changed = Signal(bool)
    teleop_changed = Signal(bool)
    sequence_progress = Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self.arm: SOARM101 | None = None
        self._timer: QTimer | None = None
        self._handles: list[tuple[str, MotionHandle[Any]]] = []
        self._simulation = False
        self._robot_id = "so101"
        self._record_timer: QTimer | None = None
        self._recording: dict[str, Any] | None = None
        self._record_started = 0.0
        self._stream_timer: QTimer | None = None
        self._stream_readout_active = False
        self._teleop: dict[str, Any] | None = None
        self._sequence_runner: SequenceRunner | None = None

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
        self.error_message.emit(f"{operation}: {exc}")
        self.log_message.emit(f"ERROR {operation}: {exc}")

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

    def _track(self, label: str, result: Any) -> None:
        if isinstance(result, MotionHandle):
            self._handles.append((label, result))
            self.busy_changed.emit(True)
            self.log_message.emit(f"Started {label}.")
        else:
            self.log_message.emit(f"Completed {label}.")
            self.poll()

    def _process_handles(self) -> bool:
        pending: list[tuple[str, MotionHandle[Any]]] = []
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
            if label.startswith("sequence "):
                self._sequence_runner = None
                self.sequence_progress.emit(
                    {
                        "type": "sequence_control",
                        "status": "failed" if exception is not None else "completed",
                    }
                )
        self._handles = pending
        busy = bool(pending)
        self.busy_changed.emit(busy)
        return busy

    @Slot(object)
    def connect_robot(self, options: object) -> None:
        try:
            if self.arm is not None:
                self.disconnect_robot()
            values = dict(options)  # type: ignore[arg-type]
            self._simulation = bool(values.get("simulation", False))
            self._robot_id = str(values.get("robot_id") or "so101")
            if self._simulation:
                self.arm = SOARM101.simulated(realtime=True)
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
        for _label, handle in self._handles:
            handle.cancel()
        self._handles.clear()
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
            self._require_arm().enable()
            self.log_message.emit("Torque enabled after latching current positions.")
            self.poll()
        except BaseException as exc:
            self._report_error("enable", exc)

    @Slot()
    def relax(self) -> None:
        try:
            for _label, handle in self._handles:
                handle.cancel()
            arm = self._require_arm()
            if arm.motion.is_streaming:
                arm.stop_joint_stream(hold=True)
            self._teleop = None
            self.teleop_changed.emit(False)
            arm.relax()
            self._handles.clear()
            self.log_message.emit("Torque disabled; arm relaxed.")
            self.poll()
        except BaseException as exc:
            self._report_error("relax", exc)

    @Slot()
    def stop(self) -> None:
        try:
            for _label, handle in self._handles:
                handle.cancel()
            arm = self._require_arm()
            self._teleop = None
            self.teleop_changed.emit(False)
            arm.stop()
            self._handles.clear()
            self.busy_changed.emit(False)
            self.log_message.emit("Software stop: current arm and gripper positions held.")
            self.poll()
        except BaseException as exc:
            self._report_error("stop", exc)

    @Slot(float)
    def start_stream_readout(self, frequency_hz: float = 50.0) -> None:
        try:
            self._require_arm()
            frequency = float(frequency_hz)
            if frequency <= 0:
                raise ValueError("stream readout frequency must be positive")
            assert self._stream_timer is not None
            self._stream_timer.setInterval(max(1, round(1000.0 / frequency)))
            self._stream_readout_active = True
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
            self.stream_sample.emit(
                {
                    "timestamp": time.perf_counter(),
                    "joints_rad": {name: float(joints[name]) for name in ARM_JOINTS},
                    "gripper": float(arm.tool.get_position()),
                }
            )
        except BaseException as exc:
            self._stop_stream_readout()
            self._report_error("high-rate readout", exc)

    @Slot(object)
    def start_teleop(self, options: object) -> None:
        try:
            arm = self._require_motion_available()
            if self._handles:
                raise RuntimeError("wait for active motion to finish before teleoperation")
            values = dict(options)  # type: ignore[arg-type]
            mode = str(values.get("mode") or "relative")
            if mode not in {"relative", "absolute"}:
                raise ValueError("teleoperation mode must be relative or absolute")
            leader_origin = {
                name: float(values["leader_joints_rad"][name]) for name in ARM_JOINTS
            }
            follower_origin = dict(arm.get_joint_positions().positions)
            follower_gripper = float(arm.tool.get_position())
            self._teleop = {
                "mode": mode,
                "leader_origin": None if mode == "relative" else leader_origin,
                "follower_origin": follower_origin,
                "leader_gripper_origin": None,
                "follower_gripper_origin": follower_gripper,
                "mirror_gripper": bool(values.get("mirror_gripper", True)),
                "samples": 0,
            }
            arm.start_joint_stream()
            if mode == "absolute":
                arm.stream_joint_target(
                    leader_origin,
                    gripper=(
                        float(values["leader_gripper"])
                        if self._teleop["mirror_gripper"]
                        else None
                    ),
                )
            self.teleop_changed.emit(True)
            self.busy_changed.emit(True)
            self.log_message.emit(
                f"Live teleoperation started in {mode} mapping mode."
            )
        except BaseException as exc:
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
        try:
            values = dict(sample)  # type: ignore[arg-type]
            leader = {
                name: float(values["joints_rad"][name]) for name in ARM_JOINTS
            }
            if teleop["mode"] == "relative":
                if teleop["leader_origin"] is None:
                    teleop["leader_origin"] = dict(leader)
                    teleop["leader_gripper_origin"] = float(values["gripper"])
                target = {
                    name: teleop["follower_origin"][name]
                    + leader[name]
                    - teleop["leader_origin"][name]
                    for name in ARM_JOINTS
                }
                gripper = None
                if teleop["mirror_gripper"]:
                    leader_gripper_origin = float(teleop["leader_gripper_origin"])
                    gripper = max(
                        0.0,
                        min(
                            1.0,
                            teleop["follower_gripper_origin"]
                            + float(values["gripper"])
                            - leader_gripper_origin,
                        ),
                    )
            else:
                target = leader
                gripper = (
                    float(values["gripper"]) if teleop["mirror_gripper"] else None
                )
            result = self._require_arm().stream_joint_target(target, gripper=gripper)
            teleop["samples"] += 1
            if teleop["samples"] % 10 == 0:
                self.sequence_progress.emit(
                    {
                        "type": "teleop",
                        "samples": teleop["samples"],
                        "message": result.message,
                    }
                )
        except BaseException as exc:
            self._stop_teleop_internal(hold=True)
            self._report_error("live teleoperation", exc)

    @Slot()
    def stop_teleop(self) -> None:
        self._stop_teleop_internal(hold=True)

    def _stop_teleop_internal(self, *, hold: bool) -> None:
        arm = self.arm
        was_active = self._teleop is not None
        self._teleop = None
        if arm is not None and arm.is_connected and arm.motion.is_streaming:
            try:
                arm.stop_joint_stream(hold=hold)
            except BaseException as exc:
                self._report_error("stop teleoperation", exc)
        if was_active:
            self.teleop_changed.emit(False)
            self.busy_changed.emit(bool(self._handles))
            self.log_message.emit("Live teleoperation stopped.")

    @Slot(object)
    def run_sequence(self, command: object) -> None:
        try:
            values = dict(command)  # type: ignore[arg-type]
            sequence = values["sequence"]
            if not isinstance(sequence, MotionSequence):
                raise TypeError("sequence command must contain a MotionSequence")
            arm = self._require_motion_available()
            runner = SequenceRunner(
                arm,
                pose_library=PoseLibrary(self._robot_id),
                trajectory_library=TrajectoryLibrary(self._robot_id),
                primitive_library=MotionPrimitiveLibrary(self._robot_id),
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
            if mode == "joint":
                result = arm.move_joints(
                    pose.joints,
                    speed=radians(float(values["speed_deg_s"])),
                    acceleration=radians(float(values["acceleration_deg_s2"])),
                    wait=False,
                )
            elif mode == "linear":
                target = Pose.from_xyz_rpy(*pose.tcp_xyz_rpy)
                result = arm.move_linear(
                    target,
                    orientation_mode=str(values.get("orientation_mode") or "compatible"),
                    speed=float(values["speed_mm_s"]) / 1000.0,
                    acceleration=float(values["acceleration_mm_s2"]) / 1000.0,
                    wait=False,
                )
            else:
                raise ValueError("saved pose mode must be 'joint' or 'linear'")
            self._track(f"{mode} move to taught point", result)
        except BaseException as exc:
            self._report_error("taught point move", exc)

    @Slot(object)
    def play_trajectory(self, command: object) -> None:
        try:
            values = dict(command)  # type: ignore[arg-type]
            trajectory = values["trajectory"]
            if not isinstance(trajectory, Trajectory):
                raise TypeError("trajectory command must contain a Trajectory")
            result = self._require_motion_available().play_trajectory(
                trajectory,
                speed_scale=float(values.get("speed_scale", 1.0)),
                move_to_start=bool(values.get("move_to_start", True)),
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
            result = jog_linear_cli_units(
                self._require_motion_available(),
                frame=values["frame"],
                translation_mm=values["translation_mm"],
                rotation_rpy_deg=values["rotation_rpy_deg"],
                orientation_mode=values["orientation_mode"],
                speed_mm_s=float(values["speed_mm_s"]),
                acceleration_mm_s2=float(values["acceleration_mm_s2"]),
                wait=False,
            )
            self._track(f"{values['frame']} linear jog", result)
        except BaseException as exc:
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

    @Slot(float)
    def move_gripper(self, position: float) -> None:
        try:
            result = self._require_motion_available().tool.move(float(position), wait=False)
            self._track("gripper move", result)
        except BaseException as exc:
            self._report_error("gripper", exc)

    @Slot(float)
    def run_calibration(self, seconds: float) -> None:
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
            duration = float(seconds)
            if duration <= 0:
                raise ValueError("calibration duration must be positive")
            self.busy_changed.emit(True)
            self.log_message.emit(
                f"Calibration recording started for {duration:.1f} s; torque is off."
            )
            calibration = calibrate(record_seconds=duration)
            path = save(calibration)
            limits = arm.get_joint_limits()
            payload = {
                "path": str(path),
                "source": calibration.source,
                "joint_limits_deg": {
                    name: (degrees(bounds[0]), degrees(bounds[1]))
                    for name, bounds in limits.items()
                },
            }
            self.calibration_completed.emit(payload)
            self.log_message.emit(f"Calibration saved to {path}.")
            self.poll()
        except BaseException as exc:
            self._report_error("calibration", exc)
        finally:
            self.busy_changed.emit(False)

    @Slot()
    def poll(self) -> None:
        if self.arm is not None and self.arm.is_connected and self.arm.motion.is_streaming:
            return
        if self._process_handles():
            # The SDK's motion thread already owns feedback polling. Avoid competing
            # serial traffic that could cause host-side command deadline misses.
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
                "joint_limits_deg": {
                    name: (degrees(bounds[0]), degrees(bounds[1]))
                    for name, bounds in self.arm.get_joint_limits().items()
                },
            }
            self.state_changed.emit(payload)
        except BaseException as exc:
            self._report_error("read state", exc)
