"""Robot session worker that keeps all GUI hardware access off the UI thread."""

from __future__ import annotations

from math import degrees, radians
from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal, Slot

from soarm101_motion import Pose, SOARM101, SOARM101Config
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.control import jog_linear_cli_units
from soarm101_motion.motion import MotionHandle


class RobotWorker(QObject):
    state_changed = Signal(object)
    connected_changed = Signal(bool)
    busy_changed = Signal(bool)
    log_message = Signal(str)
    error_message = Signal(str)
    calibration_completed = Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self.arm: SOARM101 | None = None
        self._timer: QTimer | None = None
        self._handles: list[tuple[str, MotionHandle[Any]]] = []
        self._simulation = False

    @Slot()
    def start(self) -> None:
        self._timer = QTimer(self)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self.poll)
        self._timer.start()

    def _report_error(self, operation: str, exc: BaseException) -> None:
        self.error_message.emit(f"{operation}: {exc}")
        self.log_message.emit(f"ERROR {operation}: {exc}")

    def _require_arm(self) -> SOARM101:
        if self.arm is None or not self.arm.is_connected:
            raise RuntimeError("robot is not connected")
        return self.arm

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
        self._handles.clear()
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
            self._require_arm().relax()
            self._handles.clear()
            self.log_message.emit("Torque disabled; arm relaxed.")
            self.poll()
        except BaseException as exc:
            self._report_error("relax", exc)

    @Slot()
    def stop(self) -> None:
        try:
            self._require_arm().stop()
            self._handles.clear()
            self.busy_changed.emit(False)
            self.log_message.emit("Software stop: current arm and gripper positions held.")
            self.poll()
        except BaseException as exc:
            self._report_error("stop", exc)

    @Slot(object)
    def move_joints(self, command: object) -> None:
        try:
            values = dict(command)  # type: ignore[arg-type]
            positions = {
                name: radians(float(values["joints_deg"][name])) for name in ARM_JOINTS
            }
            result = self._require_arm().move_joints(
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
                self._require_arm(),
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
            result = self._require_arm().move_linear(
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
            result = self._require_arm().tool.move(float(position), wait=False)
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
