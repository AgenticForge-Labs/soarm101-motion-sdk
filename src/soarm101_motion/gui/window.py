"""PySide6 control window for the SO-ARM101."""

from __future__ import annotations

import os
from dataclasses import replace
from math import ceil, degrees, radians
from typing import Any

from PySide6.QtCore import QMetaObject, Qt, QThread, QTimer, Signal, Slot
from PySide6.QtGui import QColor, QCloseEvent, QPalette, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QToolButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from soarm101_motion.calibration import SO101Calibration, default_calibration_path
from soarm101_motion.camera import CameraSettings, discover_camera_devices
from soarm101_motion.calibration_live import (
    PROVISIONAL_MINIMUM_TRAVEL_TICKS,
    display_travel_targets,
)
from soarm101_motion.constants import (
    ALL_MOTORS,
    ARM_JOINTS,
    DEFAULT_TELEOP_STREAM_FREQUENCY_HZ,
    JOINT_LIMITS,
)
from soarm101_motion.gui.arm_status import RobotStatusPanel
from soarm101_motion.gui.calibration_progress import CalibrationSweepPanel
from soarm101_motion.gui.camera_manager import CameraSessionManager
from soarm101_motion.gui.timeline import TrajectoryTimeline
from soarm101_motion.gui.worker import RobotWorker
from soarm101_motion.gui.teleop_rate import GRIPPER_SPEED_PRESETS
from soarm101_motion.hardware import FeetechBackend
from soarm101_motion.poses import HOME_POSE_NAME, REST_POSE_NAME, PoseLibrary, SavedPose
from soarm101_motion.primitives import MotionPrimitive, MotionPrimitiveLibrary
from soarm101_motion.provenance import bind_target_calibration, provenance_subset
from soarm101_motion.sequences import MotionSequence, SequenceLibrary, SequenceStep
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary
from soarm101_motion.gui.session_log import record as record_session
from soarm101_motion.gui.session_log import current_path as session_log_path
from soarm101_motion.workstation import (
    ArmConnectionProfile,
    WorkstationProfile,
    WorkstationProfileStore,
)


class MainWindow(QMainWindow):
    connect_requested = Signal(object)
    disconnect_requested = Signal()
    enable_requested = Signal()
    relax_requested = Signal()
    stop_requested = Signal()
    move_joints_requested = Signal(object)
    jog_requested = Signal(object)
    absolute_pose_requested = Signal(object)
    gripper_requested = Signal(object)
    calibration_requested = Signal(float)
    leader_calibration_requested = Signal(float)
    leader_connect_requested = Signal(object)
    leader_disconnect_requested = Signal()
    leader_park_requested = Signal()
    leader_release_requested = Signal()
    leader_sync_requested = Signal(object)
    follower_sync_requested = Signal(object)
    move_saved_pose_requested = Signal(object)
    trajectory_play_requested = Signal(object)
    recording_start_requested = Signal(object)
    recording_stop_requested = Signal()
    leader_recording_start_requested = Signal(object)
    leader_recording_stop_requested = Signal()
    leader_stream_start_requested = Signal(float)
    leader_stream_stop_requested = Signal()
    leader_teleop_pose_requested = Signal()
    teleop_start_requested = Signal(object)
    teleop_stop_requested = Signal()
    sequence_run_requested = Signal(object)
    sequence_pause_requested = Signal()
    sequence_resume_requested = Signal()
    effort_refresh_requested = Signal()
    effort_clear_requested = Signal()
    effort_reset_peaks_requested = Signal()
    effort_configure_requested = Signal(object)
    discover_arms_requested = Signal()
    capture_follower_pose_requested = Signal(object)
    capture_leader_pose_requested = Signal(object)
    diagnostic_logging_requested = Signal(bool)

    def __init__(
        self,
        *,
        port: str | None = None,
        robot_id: str = "so101",
        simulation: bool = False,
    ) -> None:
        super().__init__()
        self.setWindowTitle("SO-ARM101 Motion Studio")
        self.resize(1480, 900)
        self.setMinimumSize(1080, 720)

        self._workstation_store = WorkstationProfileStore()
        self._workstation_load_error: str | None = None
        try:
            self._workstation_profile = self._workstation_store.load()
        except Exception as exc:
            self._workstation_profile = WorkstationProfile().validated()
            self._workstation_load_error = str(exc)
        if not self._workstation_profile.cameras:
            self._workstation_profile = self._workstation_profile.with_camera(
                "camera",
                CameraSettings(),
            )
        if port is None and not simulation and self._workstation_profile.follower.port:
            port = self._workstation_profile.follower.port
            robot_id = self._workstation_profile.follower.robot_id

        self._connected = False
        self._follower_setup_session = False
        self._torque_enabled = False
        self._busy = False
        self._latest_state: dict[str, Any] | None = None
        self._joint_targets_initialized = False
        self._last_joint_limits: dict[str, tuple[float, float]] | None = None
        self._leader_connected = False
        self._leader_torque_enabled = False
        self._teleop_error: str | None = None
        self._teleop_fault_details: dict[str, Any] | None = None
        self._teleop_alignment_note: str | None = None
        self._follower_connecting = False
        self._leader_connecting = False
        self._discovered_arms: dict[str, dict[str, Any]] = {}
        self._leader_busy = False
        self._latest_leader_state: dict[str, Any] | None = None
        self._active_calibration_target: str | None = None
        self._calibration_cancelling = False
        self._pose_library_cache: tuple[str, PoseLibrary] | None = None
        self._trajectory_library_cache: tuple[str, TrajectoryLibrary] | None = None
        self._active_trajectory: Trajectory | None = None
        self._recording_source: str | None = None
        self._pending_recording_name: str | None = None
        self._teleop_active = False
        self._teleop_starting = False
        self._gripper_speed_multiplier = 2.0
        self._sequence_library_cache: tuple[str, SequenceLibrary] | None = None
        self._primitive_library_cache: tuple[str, MotionPrimitiveLibrary] | None = None
        self._sequence_steps: list[SequenceStep] = []
        self._sequence_paused = False
        self._latest_effort_status: dict[str, Any] = {"supported": False}
        self._effort_controls_initialized = False
        self._cartesian_jog_active = False
        self._cartesian_jog_queued = 0
        self._follower_status_panels: list[RobotStatusPanel] = []
        self._coordination_leader_labels: list[QLabel] = []
        self._coordination_relation_labels: list[QLabel] = []
        self._coordination_park_buttons: list[QPushButton] = []
        self._coordination_move_leader_buttons: list[QPushButton] = []
        self._coordination_move_follower_buttons: list[QPushButton] = []
        self._coordination_relink_buttons: list[QPushButton] = []
        self._coordination_gripper_checks: list[QCheckBox] = []
        self._sync_include_gripper = True
        self._sync_capture_pending: str | None = None
        self._camera_settings = self._workstation_profile.camera()
        self._camera_connected = False
        self._camera_status_by_name: dict[str, dict[str, object]] = {}
        self._camera_latest_images: dict[str, object] = {}
        self._loaded_camera_name = self._workstation_profile.selected_camera or "camera"

        self._thread = QThread(self)
        self._worker = RobotWorker()
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.start)
        self._thread.finished.connect(self._worker.deleteLater)

        self.connect_requested.connect(self._worker.connect_robot)
        self.disconnect_requested.connect(self._worker.disconnect_robot)
        self.enable_requested.connect(self._worker.enable)
        self.relax_requested.connect(self._worker.relax)
        self.stop_requested.connect(self._worker.stop)
        self.move_joints_requested.connect(self._worker.move_joints)
        self.jog_requested.connect(self._worker.jog_cartesian)
        self.absolute_pose_requested.connect(self._worker.move_absolute_pose)
        self.gripper_requested.connect(self._worker.move_gripper)
        self.calibration_requested.connect(self._worker.run_calibration)
        self.diagnostic_logging_requested.connect(self._worker.set_detailed_logging)
        self.move_saved_pose_requested.connect(self._worker.move_saved_pose)
        self.trajectory_play_requested.connect(self._worker.play_trajectory)
        self.recording_start_requested.connect(self._worker.start_recording)
        self.recording_stop_requested.connect(self._worker.stop_recording)
        self.teleop_start_requested.connect(self._worker.start_teleop)
        self.teleop_stop_requested.connect(self._worker.stop_teleop)
        self.sequence_run_requested.connect(self._worker.run_sequence)
        self.sequence_pause_requested.connect(self._worker.pause_sequence)
        self.sequence_resume_requested.connect(self._worker.resume_sequence)
        self.effort_refresh_requested.connect(self._worker.refresh_effort)
        self.effort_clear_requested.connect(self._worker.clear_effort_trip)
        self.effort_reset_peaks_requested.connect(self._worker.reset_effort_peaks)
        self.effort_configure_requested.connect(self._worker.configure_effort_safety)
        self.discover_arms_requested.connect(self._worker.discover_arms)
        self.capture_follower_pose_requested.connect(self._worker.capture_measured_pose)
        self.follower_sync_requested.connect(self._worker.synchronize_pose)

        self._worker.state_changed.connect(self._on_state)
        self._worker.joint_measurements.connect(self._on_follower_joint_measurements)
        self._worker.connected_changed.connect(self._on_connected)
        self._worker.busy_changed.connect(self._on_busy)
        self._worker.log_message.connect(self._log)
        self._worker.error_message.connect(self._on_error)
        self._worker.calibration_completed.connect(self._on_follower_calibration_completed)
        self._worker.calibration_progress.connect(self._on_follower_calibration_progress)
        self._worker.calibration_cancelled.connect(self._on_follower_calibration_cancelled)
        self._worker.recording_completed.connect(self._on_recording_completed)
        self._worker.recording_changed.connect(self._on_follower_recording_changed)
        self._worker.teleop_changed.connect(self._on_teleop_changed)
        self._worker.teleop_faulted.connect(self._on_teleop_faulted)
        self._worker.teleop_alignment_adjusted.connect(self._on_teleop_alignment_adjusted)
        self._worker.sequence_progress.connect(self._on_sequence_progress)
        self._worker.effort_changed.connect(self._on_effort_status)
        self._worker.arm_discovery_completed.connect(self._on_arm_discovery_completed)
        self._worker.measured_pose_captured.connect(self._on_measured_pose_captured)
        self._worker.jog_queue_changed.connect(self._on_jog_queue_changed)
        self._worker.cartesian_jog_diagnostic.connect(self._on_cartesian_jog_diagnostic)

        self._leader_thread = QThread(self)
        self._leader_worker = RobotWorker()
        self._leader_worker.moveToThread(self._leader_thread)
        self._leader_thread.started.connect(self._leader_worker.start)
        self._leader_thread.finished.connect(self._leader_worker.deleteLater)
        self.leader_connect_requested.connect(self._leader_worker.connect_robot)
        self.leader_disconnect_requested.connect(self._leader_worker.disconnect_robot)
        self.leader_park_requested.connect(self._leader_worker.enable)
        self.leader_release_requested.connect(self._leader_worker.relax)
        self.leader_sync_requested.connect(self._leader_worker.synchronize_pose)
        self.leader_calibration_requested.connect(self._leader_worker.run_calibration)
        self.diagnostic_logging_requested.connect(self._leader_worker.set_detailed_logging)
        self.leader_recording_start_requested.connect(self._leader_worker.start_recording)
        self.leader_recording_stop_requested.connect(self._leader_worker.stop_recording)
        self.leader_stream_start_requested.connect(self._leader_worker.start_stream_readout)
        self.leader_stream_stop_requested.connect(self._leader_worker.stop_stream_readout)
        self.leader_teleop_pose_requested.connect(self._leader_worker.read_teleop_start_pose)
        self.capture_leader_pose_requested.connect(self._leader_worker.capture_measured_pose)
        self._leader_worker.teleop_start_pose.connect(self._on_leader_teleop_start_pose)
        self._leader_worker.stream_sample.connect(self._worker.apply_teleop_sample)
        self._leader_worker.stream_readout_changed.connect(
            self._on_leader_stream_readout_changed
        )
        self._leader_worker.state_changed.connect(self._on_leader_state)
        self._leader_worker.connected_changed.connect(self._on_leader_connected)
        self._leader_worker.busy_changed.connect(self._on_leader_busy)
        self._leader_worker.calibration_completed.connect(self._on_leader_calibration_completed)
        self._leader_worker.calibration_progress.connect(self._on_leader_calibration_progress)
        self._leader_worker.calibration_cancelled.connect(self._on_leader_calibration_cancelled)
        self._leader_worker.log_message.connect(self._on_leader_log_message)
        self._leader_worker.error_message.connect(self._on_leader_error_message)
        self._leader_worker.recording_completed.connect(self._on_recording_completed)
        self._leader_worker.recording_changed.connect(self._on_leader_recording_changed)
        self._leader_worker.measured_pose_captured.connect(self._on_measured_pose_captured)

        self._build_ui(port=port, robot_id=robot_id, simulation=simulation)
        self._camera_manager = CameraSessionManager()
        self._camera_manager.frame_ready.connect(self._on_camera_frame)
        self._camera_manager.status_changed.connect(self._on_camera_status)
        self._camera_manager.error_message.connect(self._on_camera_error)
        self._camera_manager.snapshot_saved.connect(self._on_camera_snapshot_saved)
        self._camera_manager.sync(self._workstation_profile.cameras)
        self._camera_manager.start_auto()
        self._thread.start()
        self._leader_thread.start()
        self._refresh_ports()
        self._refresh_leader_ports()
        self._refresh_named_pose_status()
        self._refresh_point_list()
        self._refresh_trajectory_list()
        self._refresh_primitive_list()
        self._refresh_sequence_list()
        self._update_enabled_state()

    @staticmethod
    def _mix_color(first: QColor, second: QColor, weight: float) -> QColor:
        weight = max(0.0, min(1.0, float(weight)))
        inverse = 1.0 - weight
        return QColor(
            round(first.red() * inverse + second.red() * weight),
            round(first.green() * inverse + second.green() * weight),
            round(first.blue() * inverse + second.blue() * weight),
        )

    def _apply_modern_style(self) -> None:
        palette = self.palette()
        window = palette.color(QPalette.ColorRole.Window)
        text = palette.color(QPalette.ColorRole.WindowText)
        highlight = palette.color(QPalette.ColorRole.Highlight)
        highlighted_text = palette.color(QPalette.ColorRole.HighlightedText)
        dark = window.lightness() < 128

        surface = self._mix_color(window, text, 0.13 if dark else 0.075)
        surface_hover = self._mix_color(surface, highlight, 0.24 if dark else 0.16)
        surface_pressed = self._mix_color(surface, highlight, 0.38 if dark else 0.28)
        border = self._mix_color(window, text, 0.42 if dark else 0.32)
        disabled_surface = self._mix_color(window, text, 0.07 if dark else 0.035)
        disabled_border = self._mix_color(window, text, 0.20 if dark else 0.16)
        disabled_text = self._mix_color(window, text, 0.40)

        primary = highlight
        primary_hover = self._mix_color(highlight, text, 0.10 if not dark else 0.06)
        primary_pressed = self._mix_color(highlight, text, 0.18 if not dark else 0.12)
        danger = QColor("#b91c1c")
        danger_hover = QColor("#991b1b")

        self.setStyleSheet(
            f"""
            QMainWindow, QWidget {{
                font-size: 12px;
            }}
            QGroupBox {{
                font-weight: 700;
                border: 1px solid palette(midlight);
                border-radius: 12px;
                margin-top: 10px;
                padding-top: 8px;
                background: palette(base);
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
            }}
            QTabWidget::pane {{
                border: 1px solid palette(midlight);
                border-radius: 12px;
                top: -1px;
                background: palette(window);
            }}
            QTabBar::tab {{
                min-height: 28px;
                padding: 7px 13px;
                margin-right: 2px;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
            }}
            QTabBar::tab:selected {{
                font-weight: 700;
                background: palette(base);
            }}
            QPushButton {{
                min-height: 34px;
                padding: 7px 13px;
                border: 1px solid {border.name()};
                border-radius: 9px;
                background-color: {surface.name()};
                color: {text.name()};
                font-weight: 650;
            }}
            QPushButton:hover {{
                border: 1px solid {highlight.name()};
                background-color: {surface_hover.name()};
            }}
            QPushButton:focus {{
                border: 2px solid {highlight.name()};
                padding: 6px 12px;
            }}
            QPushButton:pressed {{
                background-color: {surface_pressed.name()};
            }}
            QPushButton:checked {{
                border: 2px solid {highlight.name()};
                padding: 6px 12px;
                background-color: {surface_pressed.name()};
            }}
            QPushButton:disabled {{
                color: {disabled_text.name()};
                border: 1px solid {disabled_border.name()};
                background-color: {disabled_surface.name()};
            }}
            QPushButton[buttonRole="primary"] {{
                border: 1px solid {primary.name()};
                background-color: {primary.name()};
                color: {highlighted_text.name()};
                font-weight: 750;
            }}
            QPushButton[buttonRole="primary"]:hover {{
                border-color: {primary_hover.name()};
                background-color: {primary_hover.name()};
            }}
            QPushButton[buttonRole="primary"]:pressed {{
                background-color: {primary_pressed.name()};
            }}
            QPushButton[buttonRole="primary"]:disabled {{
                border-color: {disabled_border.name()};
                background-color: {disabled_surface.name()};
                color: {disabled_text.name()};
            }}
            QPushButton[buttonRole="danger"] {{
                border: 1px solid {danger.name()};
                background-color: {danger.name()};
                color: white;
                font-weight: 800;
            }}
            QPushButton[buttonRole="danger"]:hover {{
                border-color: {danger_hover.name()};
                background-color: {danger_hover.name()};
            }}
            QPushButton[buttonRole="danger"]:disabled {{
                border-color: {disabled_border.name()};
                background-color: {disabled_surface.name()};
                color: {disabled_text.name()};
            }}
            QToolButton#helpButton {{
                min-width: 24px;
                max-width: 24px;
                min-height: 24px;
                max-height: 24px;
                border: 1px solid {border.name()};
                border-radius: 12px;
                background-color: {surface.name()};
                color: {text.name()};
                font-weight: 800;
            }}
            QToolButton#helpButton:hover {{
                border-color: {highlight.name()};
                background-color: {surface_hover.name()};
            }}
            QComboBox#cameraDeviceCombo {{
                min-height: 34px;
                padding: 5px 34px 5px 9px;
                border: 2px solid {border.name()};
                border-radius: 8px;
                background-color: {surface.name()};
                color: {text.name()};
                font-weight: 650;
            }}
            QComboBox#cameraDeviceCombo:hover {{
                border-color: {highlight.name()};
                background-color: {surface_hover.name()};
            }}
            QComboBox#cameraDeviceCombo::drop-down {{
                width: 30px;
                border-left: 1px solid {border.name()};
            }}
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QTextEdit {{
                min-height: 27px;
                padding: 3px 6px;
                border: 1px solid palette(midlight);
                border-radius: 7px;
                background: palette(base);
            }}
            QListWidget {{
                border: 1px solid palette(midlight);
                border-radius: 9px;
                background: palette(base);
                padding: 4px;
            }}
            QProgressBar {{
                min-height: 18px;
                border: 1px solid palette(midlight);
                border-radius: 7px;
                text-align: center;
                background: palette(alternate-base);
            }}
            QProgressBar::chunk {{
                border-radius: 6px;
                background: palette(highlight);
            }}
            QScrollArea {{
                border: 0;
                background: transparent;
            }}
            """
        )

    def _style_action_buttons(self) -> None:
        primary_names = (
            "connect_button",
            "leader_connect_button",
            "find_arms_button",
            "setup_find_arms_button",
            "setup_connect_button",
            "run_calibration_button",
            "camera_apply_button",
            "camera_toggle_button",
            "teleop_button",
            "save_point_button",
            "record_button",
            "save_edited_trajectory_button",
            "save_sequence_button",
            "run_sequence_button",
            "move_joints_button",
            "absolute_move_button",
        )
        for name in primary_names:
            button = getattr(self, name, None)
            if isinstance(button, QPushButton):
                button.setProperty("buttonRole", "primary")

        for name in ("stop_button", "stop_sequence_button"):
            button = getattr(self, name, None)
            if isinstance(button, QPushButton):
                button.setProperty("buttonRole", "danger")
                button.setStyleSheet("")

        for button in self.findChildren(QPushButton):
            button.setCursor(Qt.CursorShape.PointingHandCursor)

        # Dynamic-property selectors are evaluated when the stylesheet is applied.
        # Reapply after all tabs/buttons have been constructed and roles assigned.
        self._apply_modern_style()


    def _help_button(self, title: str, text: str) -> QToolButton:
        button = QToolButton()
        button.setObjectName("helpButton")
        button.setText("?")
        button.setToolTip(text)
        button.clicked.connect(
            lambda _checked=False, t=title, body=text: QMessageBox.information(
                self, t, body
            )
        )
        return button

    def _save_arm_connection_profile(self, role: str) -> None:
        if role == "leader":
            port = self.leader_port_combo.currentText().strip()
            robot_id = self.leader_robot_id_edit.text().strip() or "so101-leader"
        else:
            port = self.port_combo.currentText().strip()
            robot_id = self.robot_id_edit.text().strip() or "so101"
        arm = ArmConnectionProfile(
            port=port,
            robot_id=robot_id,
            calibration=str(default_calibration_path(robot_id)),
        ).validated()
        if role == "leader":
            self._workstation_profile = replace(
                self._workstation_profile,
                leader=arm,
            ).validated()
        else:
            self._workstation_profile = replace(
                self._workstation_profile,
                follower=arm,
            ).validated()
        self._workstation_profile = self._workstation_store.save(
            self._workstation_profile
        )

    def _sync_camera_manager(self) -> None:
        if hasattr(self, "_camera_manager"):
            self._camera_manager.sync(self._workstation_profile.cameras)

    def _camera_name(self) -> str:
        if hasattr(self, "camera_profile_combo"):
            name = self.camera_profile_combo.currentText().strip()
            if name:
                return name
        return self._workstation_profile.selected_camera or next(
            iter(self._workstation_profile.cameras)
        )

    def _teleop_camera_name(self) -> str:
        if hasattr(self, "teleop_camera_combo"):
            name = self.teleop_camera_combo.currentText().strip()
            if name:
                return name
        return self._camera_name()

    def _camera_is_connected(self, name: str) -> bool:
        return bool(self._camera_status_by_name.get(name, {}).get("connected"))

    def _camera_is_running(self, name: str) -> bool:
        status = self._camera_status_by_name.get(name, {})
        return bool(status.get("connected") or status.get("recovering"))

    @staticmethod
    def _camera_device_label(device: str) -> str:
        text = str(device).strip()
        name = text.rsplit("/", 1)[-1]
        if name.startswith("usb-"):
            name = name[4:]
        if name.endswith("-video-index0"):
            name = name[: -len("-video-index0")]
        friendly = name.replace("_", " ").strip()
        return f"{friendly}  ·  {text}" if friendly and friendly != text else text

    def _set_camera_device_choices(
        self,
        devices: list[str],
        *,
        selected_device: str | None = None,
    ) -> None:
        selected = str(selected_device or "").strip()
        self.camera_device_combo.blockSignals(True)
        self.camera_device_combo.clear()
        for device in devices:
            self.camera_device_combo.addItem(self._camera_device_label(device), device)
        if selected:
            index = self.camera_device_combo.findData(selected)
            if index < 0:
                self.camera_device_combo.addItem(self._camera_device_label(selected), selected)
                index = self.camera_device_combo.findData(selected)
            self.camera_device_combo.setCurrentIndex(index)
        elif devices:
            self.camera_device_combo.setCurrentIndex(0)
        self.camera_device_combo.blockSignals(False)

    def _camera_device_value(self) -> str:
        data = self.camera_device_combo.currentData()
        return str(data if data is not None else self.camera_device_combo.currentText()).strip()

    def _refresh_camera_profile_choices(self, selected: str | None = None) -> None:
        names = list(self._workstation_profile.cameras)
        selected = selected or self._workstation_profile.selected_camera
        for combo_name in ("camera_profile_combo", "teleop_camera_combo"):
            combo = getattr(self, combo_name, None)
            if combo is None:
                continue
            current = combo.currentText().strip()
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(names)
            target = selected or current
            if target and combo.findText(target) >= 0:
                combo.setCurrentText(target)
            combo.blockSignals(False)

    def _build_ui(self, *, port: str | None, robot_id: str, simulation: bool) -> None:
        self._apply_modern_style()
        root = QWidget(self)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)

        self.follower_connection_panel = self._build_connection_bar(port, robot_id, simulation)
        self.leader_connection_panel = self._build_leader_connection()

        self.alert_label = QLabel()
        self.alert_label.setTextFormat(Qt.TextFormat.PlainText)
        self.alert_label.setWordWrap(True)
        self.alert_label.setStyleSheet(
            "font-weight: 700; padding: 9px 11px; border-radius: 9px; "
            "background: #fee2e2; color: #7f1d1d;"
        )
        self.alert_label.hide()
        layout.addWidget(self.alert_label)

        self.robot_sidebar = self._build_persistent_robot_sidebar()
        self._follower_status_panels = [self.robot_sidebar]
        self.cartesian_view = self.robot_sidebar.view

        self.tabs = QTabWidget()
        # Build in dependency order, then display in task order.
        self.calibration_page = self._build_setup_tab()
        self.camera_page = self._build_camera_tab()
        self.manual_page = self._build_control_tab()
        self.teleop_page = self._build_teleop_tab()
        self.record_page = self._build_teach_tab()
        self.trajectory_page = self._build_trajectory_tab()
        self.run_page = self._build_run_tab()
        self.tabs.addTab(self.calibration_page, "Setup")
        self.tabs.addTab(self.camera_page, "Camera")
        self.tabs.addTab(self.manual_page, "Manual")
        self.tabs.addTab(self.teleop_page, "Teleoperation")
        self.tabs.addTab(self.record_page, "Teach / Record")
        self.tabs.addTab(self.trajectory_page, "Edit recordings")
        self.tabs.addTab(self.run_page, "Programs")
        self.log_page = QWidget()
        log_layout = QVBoxLayout(self.log_page)
        log_layout.setContentsMargins(10, 10, 10, 10)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(2000)
        self.log.setPlaceholderText("Connection, motion, and safety messages appear here.")
        log_layout.addWidget(self.log)
        self.tabs.addTab(self.log_page, "Log")
        self.tabs.currentChanged.connect(self._on_tab_changed)

        workspace = QSplitter(Qt.Orientation.Horizontal)
        workspace.setChildrenCollapsible(False)
        workspace.addWidget(self.tabs)

        workspace.addWidget(self.robot_sidebar_container)
        workspace.setStretchFactor(0, 1)
        workspace.setStretchFactor(1, 0)
        workspace.setSizes([1040, 360])
        layout.addWidget(workspace, 1)
        self.workspace_splitter = workspace

        self.setCentralWidget(root)
        self._style_action_buttons()
        self._refresh_sidebar_context()

    def _build_persistent_robot_sidebar(self) -> RobotStatusPanel:
        container = QWidget()
        container.setObjectName("robotSidebarContainer")
        container.setMinimumWidth(330)
        container.setMaximumWidth(430)
        layout = QVBoxLayout(container)
        layout.setContentsMargins(4, 2, 4, 4)
        layout.setSpacing(9)

        heading = QHBoxLayout()
        title = QLabel("ROBOT")
        title.setStyleSheet("font-size: 11px; font-weight: 800; color: palette(mid);")
        heading.addWidget(title)
        heading.addStretch(1)
        self.sidebar_mode_label = QLabel("FOLLOWER")
        self.sidebar_mode_label.setStyleSheet(
            "font-size: 10px; font-weight: 800; padding: 3px 7px; "
            "border-radius: 8px; background: palette(alternate-base);"
        )
        heading.addWidget(self.sidebar_mode_label)
        layout.addLayout(heading)

        panel = RobotStatusPanel(
            "Follower",
            subtitle="Measured follower state · solid arm is always the follower",
            compact=False,
        )
        panel_scroll = QScrollArea()
        panel_scroll.setWidgetResizable(True)
        panel_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        panel_scroll.setWidget(panel)
        layout.addWidget(panel_scroll, 1)
        self.robot_sidebar_scroll = panel_scroll

        controls = QGroupBox("Always available")
        controls_layout = QGridLayout(controls)
        self.status_label = QLabel("Disconnected")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("font-weight: 700; padding: 4px 0;")
        controls_layout.addWidget(self.status_label, 0, 0, 1, 3)

        self.sidebar_enable_button = QPushButton("Enable hold")
        self.sidebar_enable_button.setToolTip(
            "Latch the follower's measured pose, then enable torque."
        )
        self.sidebar_enable_button.clicked.connect(
            lambda _checked=False: self.enable_requested.emit()
        )
        controls_layout.addWidget(self.sidebar_enable_button, 1, 0)

        self.stop_button.setObjectName("stopButton")
        self.stop_button.setStyleSheet(
            "QPushButton#stopButton { font-weight: 800; min-height: 34px; "
            "border: 2px solid #b91c1c; border-radius: 9px; }"
        )
        controls_layout.addWidget(self.stop_button, 1, 1)

        self.sidebar_relax_button = QPushButton("Relax")
        self.sidebar_relax_button.setToolTip("Disable follower servo torque.")
        self.sidebar_relax_button.clicked.connect(
            lambda _checked=False: self.relax_requested.emit()
        )
        controls_layout.addWidget(self.sidebar_relax_button, 1, 2)

        self.pose_summary = QLabel("TCP: —")
        self.pose_summary.setWordWrap(True)
        self.pose_summary.setStyleSheet("color: palette(mid);")
        controls_layout.addWidget(self.pose_summary, 2, 0, 1, 3)
        layout.addWidget(controls)

        self.robot_sidebar_container = container
        return panel

    def _on_tab_changed(self, index: int) -> None:
        if self.tabs.widget(index) is self.log_page:
            self.tabs.setTabText(index, "Log")
        self._refresh_sidebar_context()

    def _refresh_sidebar_context(self) -> None:
        if not hasattr(self, "robot_sidebar") or not hasattr(self, "tabs"):
            return

        panel = self.robot_sidebar
        panel.set_title(
            "Follower",
            "Measured follower state · solid arm is always the follower",
        )
        panel.clear_secondary()
        page = self.tabs.currentWidget()

        if page is self.calibration_page:
            self.sidebar_mode_label.setText("SETUP")
            panel.set_context(
                "Setup and calibration. The solid arm remains the follower's live "
                "measured state whenever it is connected."
            )
            return

        if page is self.camera_page:
            self.sidebar_mode_label.setText("CAMERA")
            panel.set_context(
                "Camera configuration and capture. Camera settings are shared with "
                "Teleoperation, CLI capture, and higher-level agent workflows."
            )
            return

        if page is self.manual_page:
            self.sidebar_mode_label.setText("MANUAL")
            panel.set_context(
                "Manual control. The solid arm follows measured follower joints; "
                "Cartesian targets appear on this same view."
            )
            return

        if page is self.teleop_page:
            self.sidebar_mode_label.setText("TELEOP")
            panel.set_context(
                "Teleoperation. Follower is solid; connected leader is shown as a "
                "ghost so alignment and divergence stay visible."
            )
            if self._latest_leader_state is not None:
                panel.show_secondary_state(self._latest_leader_state, label="leader")
            return

        if page is self.record_page:
            self.sidebar_mode_label.setText("TEACH")
            source = (
                str(self.teaching_source_combo.currentData())
                if hasattr(self, "teaching_source_combo")
                else "follower"
            )
            selected = (
                self.point_combo.currentText().strip()
                if hasattr(self, "point_combo")
                else ""
            )
            panel.set_context(
                f"Teach / Record · source: {source}. Follower stays solid; a selected "
                "saved position is shown as a ghost."
            )
            if selected:
                self._preview_selected_taught_point(force=True)
            elif source == "leader" and self._latest_leader_state is not None:
                panel.show_secondary_state(self._latest_leader_state, label="leader source")
            return

        if page is self.trajectory_page:
            self.sidebar_mode_label.setText("RECORDING")
            panel.set_context(
                "Recording editor. Follower stays solid; the scrubbed trajectory pose "
                "is shown as a ghost."
            )
            if self._active_trajectory is not None:
                self._update_trajectory_arm_preview(self._cursor_seconds(), force=True)
            return

        if page is self.run_page:
            self.sidebar_mode_label.setText("PROGRAM")
            panel.set_context(
                "Program preview. Follower stays solid; the selected Move destination "
                "is shown as a ghost."
            )
            self._preview_selected_program_step(force=True)
            return

        self.sidebar_mode_label.setText("LOG")
        panel.set_context(
            "Live follower status remains visible while you inspect connection, motion, "
            "and safety events."
        )

    def _build_connection_bar(
        self,
        port: str | None,
        robot_id: str,
        simulation: bool,
    ) -> QGroupBox:
        box = QGroupBox("Follower — the arm that moves")
        layout = QGridLayout(box)

        self.simulation_check = QCheckBox("Simulation")
        self.simulation_check.setChecked(simulation)
        self.simulation_check.toggled.connect(lambda _checked: self._update_enabled_state())
        layout.addWidget(self.simulation_check, 0, 0)

        layout.addWidget(QLabel("Port"), 0, 1)
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.currentTextChanged.connect(lambda _: self._refresh_device_hints())
        if port:
            self.port_combo.addItem(port)
            self.port_combo.setCurrentText(port)
        layout.addWidget(self.port_combo, 0, 2)

        self.refresh_ports_button = QPushButton("Refresh ports")
        self.refresh_ports_button.clicked.connect(self._refresh_ports)
        layout.addWidget(self.refresh_ports_button, 0, 3)
        layout.addWidget(
            self._help_button(
                "Follower connection",
                "The follower is the powered arm that executes motion. Find Arms probes "
                "candidate serial devices read-only. Connect opens the session with torque "
                "off; Enable hold latches the measured pose before enabling torque. The "
                "selected port, robot ID, and calibration-file path are saved to the local "
                "workstation profile after a successful connection.",
            ),
            0,
            4,
        )

        layout.addWidget(QLabel("Calibration profile"), 1, 0, 1, 2)
        self.robot_id_edit = QLineEdit(robot_id)
        self.robot_id_edit.setToolTip(
            "Robot/calibration ID. Calibration file: "
            f"{default_calibration_path(robot_id)}"
        )
        self.robot_id_edit.textChanged.connect(
            lambda _text: self._on_robot_id_changed()
        )
        layout.addWidget(self.robot_id_edit, 1, 2)

        self.connect_button = QPushButton("Connect follower")
        self.connect_button.clicked.connect(self._toggle_connection)
        layout.addWidget(self.connect_button, 1, 3, 1, 2)

        self.enable_button = QPushButton("Enable hold")
        self.enable_button.setToolTip("Latch current positions, then enable torque.")
        self.enable_button.clicked.connect(lambda _checked=False: self.enable_requested.emit())
        layout.addWidget(self.enable_button, 2, 0)

        self.stop_button = QPushButton("STOP / HOLD")
        self.stop_button.setStyleSheet("font-weight: 800;")
        self.stop_button.setToolTip("Software stop only. Keep physical power accessible.")
        self.stop_button.clicked.connect(lambda _checked=False: self.stop_requested.emit())

        self.relax_button = QPushButton("Relax follower")
        self.relax_button.setToolTip("Disable servo torque.")
        self.relax_button.clicked.connect(lambda _checked=False: self.relax_requested.emit())
        layout.addWidget(self.relax_button, 2, 1)

        self.find_arms_button = QPushButton("Find Arms")
        self.find_arms_button.setToolTip(
            "Probe serial devices read-only, verify SO-101 servos, and identify leader/follower by voltage."
        )
        self.find_arms_button.clicked.connect(self._find_arms)
        layout.addWidget(self.find_arms_button, 2, 2)

        self.allow_uncalibrated_check = QCheckBox(
            "Allow uncalibrated setup connection"
        )
        self.allow_uncalibrated_check.setToolTip(
            "Calibration/setup only. Torque remains off until a valid calibration exists."
        )
        layout.addWidget(self.allow_uncalibrated_check, 2, 3, 1, 2)

        self.follower_session_status = QLabel("Disconnected")
        self.follower_session_status.setWordWrap(True)
        layout.addWidget(self.follower_session_status, 3, 0, 1, 2)
        self.follower_device_info = QLabel("Voltage: not measured")
        layout.addWidget(self.follower_device_info, 3, 2, 1, 3)

        self.arm_discovery_status = QLabel("Arm discovery: not run")
        self.arm_discovery_status.setWordWrap(True)
        self.arm_discovery_status.setToolTip(
            "Find Arms identifies likely leader/follower roles from measured voltage; "
            "confirm the physical hardware before connecting."
        )
        layout.addWidget(self.arm_discovery_status, 4, 0, 1, 5)
        return box

    @staticmethod
    def _spin(
        minimum: float,
        maximum: float,
        value: float,
        *,
        decimals: int = 2,
        step: float = 1.0,
        suffix: str = "",
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _build_setup_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        sessions = QHBoxLayout()
        sessions.addWidget(self.follower_connection_panel, 1)
        sessions.addWidget(self.leader_connection_panel, 1)
        layout.addLayout(sessions)

        heading = QLabel("Mechanical-stop calibration · two full sweeps")
        heading.setStyleSheet("font-size: 18px; font-weight: 700; padding: 6px 0;")
        layout.addWidget(heading)

        calibration = QGroupBox("Calibrate arm — mechanical stops")
        calibration.setStyleSheet("QGroupBox { font-weight: 700; }")
        grid = QGridLayout(calibration)
        explanation = QLabel("Select arm → connect torque off → run two full sweeps.")
        explanation.setWordWrap(True)
        grid.addWidget(explanation, 0, 0, 1, 3)
        grid.addWidget(
            self._help_button(
                "Mechanical-stop calibration",
                "Move each joint and the gripper gently from one printed/mechanical stop "
                "to the other and back twice. Never force or hold an actuator against a "
                "stop. All six channels must reach 2/2 before calibration is saved. A "
                "failed or cancelled sweep preserves the previous calibration.",
            ),
            0,
            3,
        )

        grid.addWidget(QLabel("Calibration target"), 1, 0)
        self.calibration_target_combo = QComboBox()
        self.calibration_target_combo.addItem("Follower", "follower")
        self.calibration_target_combo.addItem("Leader", "leader")
        self.calibration_target_combo.currentIndexChanged.connect(
            lambda _index: self._on_calibration_target_changed()
        )
        grid.addWidget(self.calibration_target_combo, 1, 1)

        grid.addWidget(QLabel("Time limit"), 1, 2)
        self.calibration_duration = self._spin(
            5.0, 180.0, 90.0, decimals=1, step=5.0, suffix=" s"
        )
        grid.addWidget(self.calibration_duration, 1, 3)

        self.setup_find_arms_button = QPushButton("1. Find Arms")
        self.setup_find_arms_button.clicked.connect(self._find_arms)
        grid.addWidget(self.setup_find_arms_button, 2, 0)

        self.setup_connect_button = QPushButton("2. Connect for calibration (torque off)")
        self.setup_connect_button.clicked.connect(self._connect_for_calibration)
        grid.addWidget(self.setup_connect_button, 2, 1, 1, 2)

        self.leader_allow_uncalibrated_check = QCheckBox(
            "Allow uncalibrated leader setup"
        )
        self.leader_allow_uncalibrated_check.setToolTip(
            "Used on the next leader connection for calibration/setup only. "
            "Keep leader torque off."
        )
        grid.addWidget(self.leader_allow_uncalibrated_check, 3, 0, 1, 4)

        self.run_calibration_button = QPushButton("3. Start calibration sweep")
        self.run_calibration_button.setMinimumHeight(36)
        self.run_calibration_button.setStyleSheet("font-weight: 700;")
        self.run_calibration_button.clicked.connect(self._start_calibration)
        grid.addWidget(self.run_calibration_button, 2, 3)

        self.cancel_calibration_button = QPushButton("Cancel sweep")
        self.cancel_calibration_button.clicked.connect(self._cancel_calibration)
        grid.addWidget(self.cancel_calibration_button, 3, 3)

        self.calibration_time_bar = QProgressBar()
        self.calibration_time_bar.setRange(0, 1000)
        self.calibration_time_bar.setValue(0)
        self.calibration_time_bar.setFormat("Recording has not started")
        grid.addWidget(self.calibration_time_bar, 4, 0, 1, 4)

        self.calibration_status = QLabel(
            "Simulation cannot perform encoder calibration."
            if self.simulation_check.isChecked()
            else "Find Arms, select Follower or Leader, then connect for calibration."
        )
        self.calibration_status.setWordWrap(True)
        self.calibration_status.setStyleSheet("font-weight: 700; padding: 8px;")
        grid.addWidget(self.calibration_status, 5, 0, 1, 4)

        self.calibration_target_note = QLabel()
        self.calibration_target_note.setWordWrap(True)

        self.calibration_sweep_panel = CalibrationSweepPanel()
        self.calibration_sweep_panel.reset(
            PROVISIONAL_MINIMUM_TRAVEL_TICKS,
            self._current_calibration_display_targets(),
        )
        self._update_calibration_guide()
        grid.addWidget(self.calibration_sweep_panel, 6, 0, 1, 4)
        grid.addWidget(self.calibration_target_note, 7, 0, 1, 4)
        layout.addWidget(calibration)

        logging_box = QGroupBox("Session logging")
        logging_layout = QVBoxLayout(logging_box)
        self.detailed_logging_check = QCheckBox("Record detailed motion diagnostics")
        self.detailed_logging_check.setChecked(True)
        self.detailed_logging_check.setToolTip(
            "Writes each teleop sample's leader, target, command, measured joints, and timing "
            "to the session file. Session events and errors are always recorded."
        )
        self.detailed_logging_check.toggled.connect(self._set_detailed_logging)
        logging_layout.addWidget(self.detailed_logging_check)
        path = session_log_path()
        self.detailed_logging_check.setEnabled(path is not None)
        self.session_log_path = QLineEdit(str(path) if path is not None else "No session file configured")
        self.session_log_path.setReadOnly(True)
        self.session_log_path.setToolTip("Select and copy this path to inspect the JSONL log.")
        logging_layout.addWidget(self.session_log_path)
        layout.addWidget(logging_box)

        effort = QGroupBox("Motor effort safety / characterization")
        effort_grid = QGridLayout(effort)
        effort_note = QLabel(
            "Current/load feedback is a raw safety signal, not calibrated force. "
            "Settings below are session-only and reset on reconnect. Change thresholds "
            "only with torque OFF; hard joint, calibration, workspace, following-error, "
            "fault, and timing protections are unaffected."
        )
        effort_note.setWordWrap(True)
        effort_grid.addWidget(effort_note, 0, 0, 1, 7)

        self.effort_guard_check = QCheckBox("Effort guard enabled")
        self.effort_guard_check.setChecked(True)
        effort_grid.addWidget(self.effort_guard_check, 1, 0, 1, 2)

        effort_grid.addWidget(QLabel("Current trip"), 1, 2)
        self.effort_current_spin = QSpinBox()
        self.effort_current_spin.setRange(0, 4095)
        self.effort_current_spin.setValue(250)
        self.effort_current_spin.setSpecialValueText("disabled")
        self.effort_current_spin.setToolTip("0 disables the global current threshold.")
        effort_grid.addWidget(self.effort_current_spin, 1, 3)

        effort_grid.addWidget(QLabel("|Load| trip"), 1, 4)
        self.effort_load_spin = QSpinBox()
        self.effort_load_spin.setRange(0, 1023)
        self.effort_load_spin.setValue(850)
        self.effort_load_spin.setSpecialValueText("disabled")
        self.effort_load_spin.setToolTip("0 disables the global load threshold.")
        effort_grid.addWidget(self.effort_load_spin, 1, 5)

        effort_grid.addWidget(QLabel("Consecutive"), 1, 6)
        self.effort_consecutive_spin = QSpinBox()
        self.effort_consecutive_spin.setRange(1, 20)
        self.effort_consecutive_spin.setValue(2)
        effort_grid.addWidget(self.effort_consecutive_spin, 1, 7)

        self.effort_apply_button = QPushButton("Apply session settings")
        self.effort_apply_button.clicked.connect(self._apply_effort_settings)
        effort_grid.addWidget(self.effort_apply_button, 2, 0, 1, 2)
        self.effort_refresh_button = QPushButton("Refresh readings")
        self.effort_refresh_button.clicked.connect(
            lambda _checked=False: self.effort_refresh_requested.emit()
        )
        effort_grid.addWidget(self.effort_refresh_button, 2, 2, 1, 2)
        self.effort_reset_peaks_button = QPushButton("Reset peaks")
        self.effort_reset_peaks_button.clicked.connect(
            lambda _checked=False: self.effort_reset_peaks_requested.emit()
        )
        effort_grid.addWidget(self.effort_reset_peaks_button, 2, 4, 1, 2)
        self.effort_clear_button = QPushButton("Clear latched trip")
        self.effort_clear_button.clicked.connect(self._clear_effort_trip)
        effort_grid.addWidget(self.effort_clear_button, 2, 6, 1, 2)

        headers = ("Motor", "Current", "Load", "Peak I", "Peak |Load|", "I limit", "|Load| limit")
        for column, header in enumerate(headers):
            label = QLabel(header)
            label.setStyleSheet("font-weight: 600;")
            effort_grid.addWidget(label, 3, column)

        self.effort_value_labels: dict[str, dict[str, QLabel]] = {}
        for row, motor in enumerate(ALL_MOTORS, start=4):
            effort_grid.addWidget(QLabel(motor.replace("_", " ").title()), row, 0)
            labels: dict[str, QLabel] = {}
            for column, key in enumerate(
                ("current", "load", "peak_current", "peak_load", "current_limit", "load_limit"),
                start=1,
            ):
                label = QLabel("—")
                labels[key] = label
                effort_grid.addWidget(label, row, column)
            self.effort_value_labels[motor] = labels

        self.effort_status_label = QLabel(
            "Effort telemetry unavailable until compatible hardware is connected."
        )
        self.effort_status_label.setWordWrap(True)
        effort_grid.addWidget(self.effort_status_label, 10, 0, 1, 8)
        layout.addWidget(effort)

        later = QLabel(
            "Physical validation is intentionally deferred. Follow TESTING.md when you are "
            "ready to use the real arm."
        )
        later.setWordWrap(True)
        layout.addWidget(later)
        layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        return scroll

    @Slot(bool)
    def _set_detailed_logging(self, enabled: bool) -> None:
        self.diagnostic_logging_requested.emit(enabled)
        self._log(f"Detailed motion diagnostics {'enabled' if enabled else 'disabled'}.")

    def _new_follower_status_panel(
        self,
        title: str,
        *,
        subtitle: str = "",
        compact: bool = True,
    ) -> RobotStatusPanel:
        panel = RobotStatusPanel(title, subtitle=subtitle, compact=compact)
        self._follower_status_panels.append(panel)
        if self._latest_state is not None:
            panel.update_state(self._latest_state)
        return panel

    def _build_control_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(9)
        layout.addWidget(self._build_coordination_panel())
        layout.addWidget(self._build_named_pose_controls())

        self.manual_mode_tabs = QTabWidget()
        self.manual_mode_tabs.addTab(self._build_joint_tab(), "Joint / angular")
        self.manual_mode_tabs.addTab(self._build_cartesian_tab(), "Cartesian")
        layout.addWidget(self.manual_mode_tabs, 1)

        # The gripper is a tool, not a sixth pose joint. Keep its controls visible
        # below both angular and Cartesian modes so switching arm representations
        # never hides the tool state or speed.
        layout.addWidget(self._build_gripper_panel())
        self.manual_arm_panel = self.robot_sidebar
        return page

    def _build_coordination_panel(self) -> QGroupBox:
        box = QGroupBox("Leader / follower coordination")
        grid = QGridLayout(box)

        leader_status = QLabel("Leader: disconnected")
        leader_status.setStyleSheet("font-weight: 700;")
        relation_status = QLabel("Pose relationship: unavailable")
        relation_status.setWordWrap(True)
        self._coordination_leader_labels.append(leader_status)
        self._coordination_relation_labels.append(relation_status)
        grid.addWidget(leader_status, 0, 0, 1, 2)
        grid.addWidget(relation_status, 0, 2, 1, 4)

        park = QPushButton("Park leader here")
        park.clicked.connect(self._toggle_leader_park)
        move_leader = QPushButton("Move leader → follower pose")
        move_leader.setToolTip(
            "Guarded joint move: the leader moves to the follower's fresh measured pose."
        )
        move_leader.clicked.connect(self._move_leader_to_follower)
        move_follower = QPushButton("Move follower → leader pose")
        move_follower.setToolTip(
            "Guarded joint move: the follower moves to the leader's fresh measured pose."
        )
        move_follower.clicked.connect(self._move_follower_to_leader)
        relink = QPushButton("Relink here — no motion")
        relink.setToolTip(
            "Start relative teleoperation using both arms' current poses as the new reference."
        )
        relink.clicked.connect(self._relink_here)

        self._coordination_park_buttons.append(park)
        self._coordination_move_leader_buttons.append(move_leader)
        self._coordination_move_follower_buttons.append(move_follower)
        self._coordination_relink_buttons.append(relink)

        grid.addWidget(park, 1, 0)
        grid.addWidget(move_leader, 1, 1, 1, 2)
        grid.addWidget(move_follower, 1, 3, 1, 2)
        grid.addWidget(relink, 1, 5)

        include_gripper = QCheckBox("Include gripper when matching poses")
        include_gripper.setChecked(self._sync_include_gripper)
        include_gripper.toggled.connect(
            lambda checked, source=include_gripper: self._set_sync_include_gripper(
                checked, source
            )
        )
        self._coordination_gripper_checks.append(include_gripper)
        grid.addWidget(include_gripper, 2, 0, 1, 3)

        hint = QLabel(
            "Park latches the leader exactly where it is. Matching moves only the "
            "selected destination arm. Relink preserves both current poses and starts "
            "relative leader control without an alignment move."
        )
        hint.setWordWrap(True)
        grid.addWidget(hint, 2, 3, 1, 3)
        return box

    def _build_named_pose_controls(self) -> QGroupBox:
        box = QGroupBox("Standard poses")
        grid = QGridLayout(box)
        self.home_status = QLabel("Home: not saved")
        self.rest_status = QLabel("Rest: not saved")
        self.save_home_button = QPushButton("Save current as Home")
        self.save_home_button.setToolTip(
            "Capture the follower's fresh measured pose, including during live teleoperation."
        )
        self.go_home_button = QPushButton("Go Home")
        self.save_rest_button = QPushButton("Save current as Rest")
        self.save_rest_button.setToolTip(
            "Capture the follower's fresh measured pose, including during live teleoperation."
        )
        self.go_rest_button = QPushButton("Go Rest")
        self.save_home_button.clicked.connect(
            lambda _checked=False: self._save_named_pose(HOME_POSE_NAME)
        )
        self.go_home_button.clicked.connect(
            lambda _checked=False: self._go_named_pose(HOME_POSE_NAME)
        )
        self.save_rest_button.clicked.connect(
            lambda _checked=False: self._save_named_pose(REST_POSE_NAME)
        )
        self.go_rest_button.clicked.connect(
            lambda _checked=False: self._go_named_pose(REST_POSE_NAME)
        )
        grid.addWidget(self.home_status, 0, 0)
        grid.addWidget(self.save_home_button, 0, 1)
        grid.addWidget(self.go_home_button, 0, 2)
        grid.addWidget(self.rest_status, 1, 0)
        grid.addWidget(self.save_rest_button, 1, 1)
        grid.addWidget(self.go_rest_button, 1, 2)
        return box

    def _build_leader_connection(self) -> QWidget:
        leader = QGroupBox("Leader — the arm you move by hand")
        grid = QGridLayout(leader)
        stored = self._workstation_profile.leader

        self.leader_simulation_check = QCheckBox("Simulation")
        self.leader_simulation_check.setChecked(self.simulation_check.isChecked())
        self.leader_simulation_check.toggled.connect(
            lambda _checked: self._update_enabled_state()
        )
        grid.addWidget(self.leader_simulation_check, 0, 0)

        grid.addWidget(QLabel("Port"), 0, 1)
        self.leader_port_combo = QComboBox()
        self.leader_port_combo.setEditable(True)
        self.leader_port_combo.currentTextChanged.connect(lambda _: self._refresh_device_hints())
        if stored.port:
            self.leader_port_combo.addItem(stored.port)
            self.leader_port_combo.setCurrentText(stored.port)
        grid.addWidget(self.leader_port_combo, 0, 2)

        self.leader_refresh_button = QPushButton("Refresh ports")
        self.leader_refresh_button.clicked.connect(self._refresh_leader_ports)
        grid.addWidget(self.leader_refresh_button, 0, 3)
        grid.addWidget(
            self._help_button(
                "Leader connection",
                "The leader is normally back-drivable with torque off. Park deliberately "
                "only when you want it to hold a pose. Starting or relinking teleoperation "
                "releases it again. The selected port, robot ID, and calibration-file path "
                "are saved after a successful connection.",
            ),
            0,
            4,
        )

        grid.addWidget(QLabel("Calibration profile"), 1, 0, 1, 2)
        self.leader_robot_id_edit = QLineEdit(stored.robot_id or "so101-leader")
        self.leader_robot_id_edit.setToolTip(
            "Robot/calibration ID. Calibration file: "
            f"{stored.calibration or default_calibration_path(stored.robot_id)}"
        )
        grid.addWidget(self.leader_robot_id_edit, 1, 2)

        self.leader_connect_button = QPushButton("Connect leader")
        self.leader_connect_button.clicked.connect(self._toggle_leader_connection)
        grid.addWidget(self.leader_connect_button, 1, 3, 1, 2)

        self.leader_session_status = QLabel("Disconnected")
        self.leader_session_status.setWordWrap(True)
        grid.addWidget(self.leader_session_status, 2, 0, 1, 2)

        self.leader_device_info = QLabel("Voltage: not measured")
        grid.addWidget(self.leader_device_info, 2, 2, 1, 3)
        return leader

    def _new_camera_preview_label(self, *, minimum_height: int = 240) -> QLabel:
        preview = QLabel("Camera preview is stopped.")
        preview.setAlignment(Qt.AlignmentFlag.AlignCenter)
        preview.setMinimumHeight(minimum_height)
        preview.setStyleSheet(
            "background: #111827; color: #d1d5db; border-radius: 10px; padding: 8px;"
        )
        return preview

    def _build_camera_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(9)

        settings_box = QGroupBox("Camera setup")
        settings_box.setMaximumWidth(820)
        self.camera_settings_box = settings_box
        grid = QGridLayout(settings_box)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)

        grid.addWidget(QLabel("Camera"), 0, 0)
        self.camera_profile_combo = QComboBox()
        self.camera_profile_combo.setMinimumWidth(180)
        self.camera_profile_combo.addItems(list(self._workstation_profile.cameras))
        if self._workstation_profile.selected_camera:
            self.camera_profile_combo.setCurrentText(
                self._workstation_profile.selected_camera
            )
        self.camera_profile_combo.currentTextChanged.connect(
            self._select_camera_profile
        )
        grid.addWidget(self.camera_profile_combo, 0, 1)

        grid.addWidget(QLabel("Name"), 0, 2)
        self.camera_name_edit = QLineEdit(self._loaded_camera_name)
        self.camera_name_edit.setMaximumWidth(240)
        self.camera_name_edit.setPlaceholderText("overhead, wrist, side...")
        grid.addWidget(self.camera_name_edit, 0, 3)

        camera_profile_actions = QHBoxLayout()
        self.camera_new_button = QPushButton("New camera")
        self.camera_new_button.clicked.connect(self._new_camera_profile)
        camera_profile_actions.addWidget(self.camera_new_button)
        self.camera_delete_button = QPushButton("Delete")
        self.camera_delete_button.clicked.connect(self._delete_camera_profile)
        camera_profile_actions.addWidget(self.camera_delete_button)
        camera_profile_actions.addWidget(
            self._help_button(
                "Named cameras",
                "Each name maps to one physical USB/UVC device and its capture settings. "
                "Use names such as overhead and wrist. Multiple named cameras may stream "
                "at the same time, but a physical device can belong to only one profile. "
                "The same names are available to CLI and agent camera commands.",
            )
        )
        grid.addLayout(camera_profile_actions, 0, 4, 1, 2)

        grid.addWidget(QLabel("USB device"), 1, 0)
        self.camera_device_combo = QComboBox()
        self.camera_device_combo.setObjectName("cameraDeviceCombo")
        self.camera_device_combo.setEditable(False)
        self.camera_device_combo.setMinimumContentsLength(28)
        self.camera_device_combo.setMaximumWidth(610)
        self.camera_device_combo.setToolTip(
            "Choose a discovered physical camera. Linux stable /dev/v4l/by-id paths "
            "are preferred so the same logical camera survives reboot/replugging."
        )
        if self._camera_settings.device:
            self._set_camera_device_choices(
                [self._camera_settings.device],
                selected_device=self._camera_settings.device,
            )
        grid.addWidget(self.camera_device_combo, 1, 1, 1, 4)
        self.camera_refresh_button = QPushButton("Find cameras")
        self.camera_refresh_button.setToolTip(
            "Refresh the device dropdown from cameras currently visible to Linux."
        )
        self.camera_refresh_button.clicked.connect(self._refresh_camera_devices)
        grid.addWidget(self.camera_refresh_button, 1, 5)

        grid.addWidget(QLabel("Resolution"), 2, 0)
        resolution = QHBoxLayout()
        self.camera_width_spin = QSpinBox()
        self.camera_width_spin.setRange(160, 7680)
        self.camera_width_spin.setMaximumWidth(120)
        resolution.addWidget(self.camera_width_spin)
        resolution.addWidget(QLabel("×"))
        self.camera_height_spin = QSpinBox()
        self.camera_height_spin.setRange(120, 4320)
        self.camera_height_spin.setMaximumWidth(120)
        resolution.addWidget(self.camera_height_spin)
        resolution.addStretch(1)
        grid.addLayout(resolution, 2, 1)

        grid.addWidget(QLabel("FPS"), 2, 2)
        self.camera_fps_spin = QDoubleSpinBox()
        self.camera_fps_spin.setRange(1.0, 240.0)
        self.camera_fps_spin.setDecimals(1)
        self.camera_fps_spin.setMaximumWidth(120)
        grid.addWidget(self.camera_fps_spin, 2, 3)

        grid.addWidget(QLabel("FourCC"), 2, 4)
        self.camera_fourcc_edit = QLineEdit()
        self.camera_fourcc_edit.setMaxLength(4)
        self.camera_fourcc_edit.setMaximumWidth(100)
        grid.addWidget(self.camera_fourcc_edit, 2, 5)

        camera_flags = QHBoxLayout()
        self.camera_mirror_check = QCheckBox("Mirror horizontally")
        camera_flags.addWidget(self.camera_mirror_check)
        self.camera_auto_start_check = QCheckBox("Auto-start with GUI")
        camera_flags.addWidget(self.camera_auto_start_check)
        camera_flags.addStretch(1)
        grid.addLayout(camera_flags, 3, 1, 1, 5)

        grid.addWidget(QLabel("Snapshot folder"), 4, 0)
        self.camera_snapshot_dir_edit = QLineEdit()
        grid.addWidget(self.camera_snapshot_dir_edit, 4, 1, 1, 5)

        action_row = QHBoxLayout()
        self.camera_apply_button = QPushButton("Save camera")
        self.camera_apply_button.clicked.connect(self._apply_camera_settings)
        action_row.addWidget(self.camera_apply_button)
        self.camera_toggle_button = QPushButton("Start selected")
        self.camera_toggle_button.clicked.connect(self._toggle_camera_stream)
        action_row.addWidget(self.camera_toggle_button)
        self.camera_start_all_button = QPushButton("Start all")
        self.camera_start_all_button.clicked.connect(self._start_all_cameras)
        action_row.addWidget(self.camera_start_all_button)
        self.camera_stop_all_button = QPushButton("Stop all")
        self.camera_stop_all_button.clicked.connect(self._stop_all_cameras)
        action_row.addWidget(self.camera_stop_all_button)
        self.camera_capture_button = QPushButton("Capture selected")
        self.camera_capture_button.clicked.connect(self._capture_camera_frame)
        action_row.addWidget(self.camera_capture_button)
        action_row.addStretch(1)
        grid.addLayout(action_row, 5, 0, 1, 6)

        self.camera_status = QLabel()
        self.camera_status.setWordWrap(True)
        grid.addWidget(self.camera_status, 6, 0, 1, 6)

        settings_row = QHBoxLayout()
        settings_row.addWidget(settings_box)
        settings_row.addStretch(1)
        layout.addLayout(settings_row)

        preview_heading = QHBoxLayout()
        preview_heading.addWidget(QLabel("Live camera views"))
        preview_heading.addStretch(1)
        preview_heading.addWidget(
            self._help_button(
                "Camera views",
                "Every saved camera gets its own preview card. One camera fills the "
                "preview area; two split side-by-side; three or more use a two-column "
                "grid. Start all cameras to watch the full bench at once.",
            )
        )
        layout.addLayout(preview_heading)

        self.camera_preview_container = QWidget()
        self.camera_preview_grid = QGridLayout(self.camera_preview_container)
        self.camera_preview_grid.setContentsMargins(0, 0, 0, 0)
        self.camera_preview_grid.setHorizontalSpacing(9)
        self.camera_preview_grid.setVerticalSpacing(9)
        self.camera_preview_labels: dict[str, QLabel] = {}
        self.camera_preview_cards: dict[str, QGroupBox] = {}
        self.camera_preview_status_labels: dict[str, QLabel] = {}
        layout.addWidget(self.camera_preview_container, 0)
        layout.addStretch(1)

        self._rebuild_camera_preview_grid()
        self._load_camera_profile_controls(self._camera_name())
        self._refresh_camera_devices()
        return page

    def _clear_layout(self, layout: QGridLayout) -> None:
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
            child = item.layout()
            if child is not None:
                while child.count():
                    nested = child.takeAt(0)
                    if nested.widget() is not None:
                        nested.widget().deleteLater()

    def _set_preview_image(self, preview: QLabel, image: object) -> None:
        if not hasattr(image, "isNull") or image.isNull():
            return
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(
            max(preview.width(), 1),
            max(preview.height(), 1),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        preview.setPixmap(scaled)

    def _camera_card_status(self, name: str) -> str:
        values = self._camera_status_by_name.get(name, {})
        settings = self._workstation_profile.cameras.get(name)
        if bool(values.get("recovering")):
            dropped = int(values.get("dropped_frames", 0) or 0)
            limit = int(values.get("drop_limit", 0) or 0)
            if dropped and limit:
                return f"Recovering · dropped frame {dropped}/{limit}"
            attempt = int(values.get("open_attempt", 0) or 0)
            attempt_limit = int(values.get("open_attempt_limit", 0) or 0)
            if attempt and attempt_limit:
                return f"Recovering · reopen {attempt}/{attempt_limit}"
            return "Recovering camera stream…"
        if bool(values.get("connected")):
            width = values.get("width", "?")
            height = values.get("height", "?")
            fps = float(values.get("fps", 0.0) or 0.0)
            return f"Live · {width}×{height} @ {fps:.1f} FPS"
        if values.get("error"):
            return f"Stopped · {values['error']}"
        device = settings.device if settings is not None else ""
        short_device = device.rsplit("/", 1)[-1] if device else "device not assigned"
        return f"Stopped · {short_device}"

    def _refresh_camera_preview_card(self, name: str) -> None:
        card = getattr(self, "camera_preview_cards", {}).get(name)
        status = getattr(self, "camera_preview_status_labels", {}).get(name)
        if card is not None:
            values = self._camera_status_by_name.get(name, {})
            state = (
                "RECOVERING"
                if bool(values.get("recovering"))
                else "LIVE"
                if self._camera_is_connected(name)
                else "STOPPED"
            )
            selected = " · selected" if name == self._camera_name() else ""
            card.setTitle(f"{name} · {state}{selected}")
        if status is not None:
            status.setText(self._camera_card_status(name))

    def _rebuild_camera_preview_grid(self) -> None:
        if not hasattr(self, "camera_preview_grid"):
            return
        self._clear_layout(self.camera_preview_grid)
        self.camera_preview_labels = {}
        self.camera_preview_cards = {}
        self.camera_preview_status_labels = {}

        names = list(self._workstation_profile.cameras)
        columns = 1 if len(names) <= 1 else 2
        if len(names) <= 1:
            minimum_height, maximum_height = 300, 360
        elif len(names) == 2:
            minimum_height, maximum_height = 170, 240
        else:
            minimum_height, maximum_height = 140, 190

        for index, name in enumerate(names):
            card = QGroupBox(name)
            card_layout = QVBoxLayout(card)
            card_layout.setContentsMargins(7, 8, 7, 7)
            card_layout.setSpacing(4)
            preview = self._new_camera_preview_label(minimum_height=minimum_height)
            preview.setMaximumHeight(maximum_height)
            preview.setText("Start camera")
            status = QLabel(self._camera_card_status(name))
            status.setWordWrap(False)
            status.setStyleSheet(
                "font-size: 10px; color: palette(mid); padding: 1px 3px;"
            )
            card_layout.addWidget(preview)
            card_layout.addWidget(status)
            card.setMaximumHeight(maximum_height + 70)
            row, column = divmod(index, columns)
            self.camera_preview_grid.addWidget(card, row, column)
            self.camera_preview_grid.setColumnStretch(column, 1)
            self.camera_preview_grid.setRowStretch(row, 0)
            self.camera_preview_labels[name] = preview
            self.camera_preview_cards[name] = card
            self.camera_preview_status_labels[name] = status
            cached = self._camera_latest_images.get(name)
            if cached is not None:
                self._set_preview_image(preview, cached)
            self._refresh_camera_preview_card(name)

        selected = self._camera_name() if names else ""
        selected_preview = self.camera_preview_labels.get(selected)
        if selected_preview is None and self.camera_preview_labels:
            selected_preview = next(iter(self.camera_preview_labels.values()))
        self.camera_preview = selected_preview or self._new_camera_preview_label()


    def _load_camera_profile_controls(self, name: str) -> None:
        if name not in self._workstation_profile.cameras:
            return
        settings = self._workstation_profile.cameras[name]
        self._loaded_camera_name = name
        self._camera_settings = settings
        self.camera_name_edit.setText(name)

        current_device = settings.device
        discovered = [
            str(self.camera_device_combo.itemData(index))
            for index in range(self.camera_device_combo.count())
            if self.camera_device_combo.itemData(index) is not None
        ]
        self._set_camera_device_choices(discovered, selected_device=current_device)

        self.camera_width_spin.setValue(settings.width)
        self.camera_height_spin.setValue(settings.height)
        self.camera_fps_spin.setValue(settings.fps)
        self.camera_fourcc_edit.setText(settings.fourcc)
        self.camera_mirror_check.setChecked(settings.mirror)
        self.camera_auto_start_check.setChecked(settings.auto_start)
        self.camera_snapshot_dir_edit.setText(settings.snapshot_dir)
        self._refresh_camera_display(name)

    def _select_camera_profile(self, name: str) -> None:
        name = str(name).strip()
        if not name or name not in self._workstation_profile.cameras:
            return
        self._workstation_profile = replace(
            self._workstation_profile,
            selected_camera=name,
        ).validated()
        self._workstation_profile = self._workstation_store.save(
            self._workstation_profile
        )
        self._load_camera_profile_controls(name)
        if hasattr(self, "camera_preview_labels"):
            self.camera_preview = self.camera_preview_labels.get(
                name, self.camera_preview
            )
            for camera_name in self.camera_preview_cards:
                self._refresh_camera_preview_card(camera_name)

    def _new_camera_profile(self) -> None:
        index = 1
        existing = set(self._workstation_profile.cameras)
        while f"camera-{index}" in existing:
            index += 1
        name = f"camera-{index}"
        used_devices = {
            settings.device for settings in self._workstation_profile.cameras.values()
        }
        try:
            discovered = discover_camera_devices()
        except Exception:
            discovered = []
        device = next(
            (candidate for candidate in discovered if candidate not in used_devices),
            "",
        )
        self._loaded_camera_name = ""
        self._camera_settings = CameraSettings(device=device)
        self.camera_name_edit.setText(name)
        discovered = [
            str(self.camera_device_combo.itemData(index))
            for index in range(self.camera_device_combo.count())
            if self.camera_device_combo.itemData(index) is not None
        ]
        if device and device not in discovered:
            discovered.append(device)
        self._set_camera_device_choices(discovered, selected_device=device)
        self.camera_width_spin.setValue(self._camera_settings.width)
        self.camera_height_spin.setValue(self._camera_settings.height)
        self.camera_fps_spin.setValue(self._camera_settings.fps)
        self.camera_fourcc_edit.setText(self._camera_settings.fourcc)
        self.camera_mirror_check.setChecked(False)
        self.camera_auto_start_check.setChecked(False)
        self.camera_snapshot_dir_edit.setText(self._camera_settings.snapshot_dir)
        self.camera_status.setText(
            "New camera profile · choose a device from the USB device dropdown and press Save camera."
        )

    def _delete_camera_profile(self) -> None:
        name = self._camera_name()
        if len(self._workstation_profile.cameras) <= 1:
            QMessageBox.information(
                self,
                "Keep one camera profile",
                "Rename the existing profile instead of deleting the final camera entry.",
            )
            return
        answer = QMessageBox.question(
            self,
            "Delete camera profile?",
            f"Delete the saved camera profile {name!r}? No image files are deleted.",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._workstation_profile = self._workstation_profile.without_camera(name)
        self._workstation_profile = self._workstation_store.save(
            self._workstation_profile
        )
        self._sync_camera_manager()
        self._refresh_camera_profile_choices(self._workstation_profile.selected_camera)
        self._rebuild_camera_preview_grid()
        self._load_camera_profile_controls(self._camera_name())

    def _refresh_camera_devices(self) -> None:
        current = self._camera_device_value()
        try:
            devices = discover_camera_devices()
        except Exception as exc:
            self._on_camera_error(self._camera_name(), str(exc))
            return

        migration_note = ""
        selected = current
        if current and current.startswith("/"):
            current_real = os.path.realpath(current)
            stable_match = next(
                (
                    device
                    for device in devices
                    if device.startswith("/")
                    and os.path.realpath(device) == current_real
                ),
                None,
            )
            if stable_match and stable_match != current:
                selected = stable_match
                migration_note = (
                    f"Mapped {current} to stable camera ID; press Save camera to keep it. "
                )

        if selected and selected not in devices:
            devices = [*devices, selected]
        self._set_camera_device_choices(devices, selected_device=selected)

        assigned_by_real = {
            os.path.realpath(settings.device): name
            for name, settings in self._workstation_profile.cameras.items()
            if settings.device.startswith("/")
        }
        if devices:
            labels = []
            for device in devices:
                owner = (
                    assigned_by_real.get(os.path.realpath(device), "unassigned")
                    if device.startswith("/")
                    else "unassigned"
                )
                short = self._camera_device_label(device).split("  ·  ", 1)[0]
                labels.append(f"{short} ({owner})")
            self.camera_status.setText(
                migration_note
                + f"Found {len(devices)} camera device(s): "
                + ", ".join(labels)
            )
        else:
            self.camera_status.setText(
                "No camera devices found. Reconnect the camera or configure a device path through the CLI."
            )

    def _camera_settings_from_controls(self) -> CameraSettings:
        return self._camera_settings.with_overrides(
            device=self._camera_device_value(),
            width=self.camera_width_spin.value(),
            height=self.camera_height_spin.value(),
            fps=self.camera_fps_spin.value(),
            fourcc=self.camera_fourcc_edit.text().strip(),
            mirror=self.camera_mirror_check.isChecked(),
            auto_start=self.camera_auto_start_check.isChecked(),
            snapshot_dir=self.camera_snapshot_dir_edit.text().strip(),
        )

    def _apply_camera_settings(self) -> None:
        try:
            name = self.camera_name_edit.text().strip()
            if not name:
                raise ValueError("camera name cannot be empty")
            settings = self._camera_settings_from_controls()
            profile = self._workstation_profile
            old_name = self._loaded_camera_name
            if old_name and old_name != name and old_name in profile.cameras:
                profile = profile.renamed_camera(old_name, name)
            profile = profile.with_camera(name, settings, select=True)
            self._workstation_profile = self._workstation_store.save(profile)
            self._camera_settings = settings
            self._loaded_camera_name = name
            self._sync_camera_manager()
            self._refresh_camera_profile_choices(name)
            self._rebuild_camera_preview_grid()
            if settings.auto_start and hasattr(self, "_camera_manager"):
                self._camera_manager.start(name)
        except Exception as exc:
            self._on_camera_error(self.camera_name_edit.text().strip() or "camera", str(exc))
            return
        self.camera_status.setText(
            f"Saved {name!r} · {settings.device} · {settings.width}×{settings.height} "
            f"@ {settings.fps:g} FPS · {settings.fourcc}"
        )
        self._log(f"Camera profile saved: {name} -> {settings.device}")

    def _camera_status_text(self, name: str) -> str:
        status = self._camera_status_by_name.get(name, {})
        settings = self._workstation_profile.cameras.get(name)
        if bool(status.get("connected")):
            return (
                f"Live · {name} · {status.get('device', settings.device if settings else '?')} · "
                f"{status.get('width', '?')}×{status.get('height', '?')} "
                f"@ {float(status.get('fps', 0.0)):.1f} FPS"
            )
        device = settings.device if settings is not None else "unconfigured"
        text = f"Stopped · {name} · {device}"
        if status.get("error"):
            text += f" · {status['error']}"
        return text

    def _refresh_camera_display(self, name: str) -> None:
        if hasattr(self, "camera_status") and name == self._camera_name():
            self.camera_status.setText(self._camera_status_text(name))
            self.camera_toggle_button.setText(
                "Stop selected" if self._camera_is_running(name) else "Start selected"
            )
        if hasattr(self, "teleop_camera_status") and name == self._teleop_camera_name():
            self.teleop_camera_status.setText(self._camera_status_text(name))
            self.teleop_camera_toggle_button.setText(
                "Stop camera" if self._camera_is_running(name) else "Start camera"
            )
        if hasattr(self, "camera_preview_cards") and name in self.camera_preview_cards:
            self._refresh_camera_preview_card(name)

    def _toggle_camera_stream(self) -> None:
        name = self._camera_name()
        if self._camera_is_running(name):
            self._camera_manager.stop(name)
            return
        self._apply_camera_settings()
        name = self._camera_name()
        self._camera_manager.start(name)
        self.camera_status.setText(f"Opening {name}…")

    def _toggle_teleop_camera_stream(self) -> None:
        name = self._teleop_camera_name()
        if self._camera_is_running(name):
            self._camera_manager.stop(name)
        else:
            self._camera_manager.start(name)
            self.teleop_camera_status.setText(f"Opening {name}…")

    def _start_all_cameras(self) -> None:
        self._apply_camera_settings()
        self._camera_manager.start_all()

    def _stop_all_cameras(self) -> None:
        self._camera_manager.stop_all()

    def _capture_camera_frame(self, name: str | None = None) -> None:
        selected = name or self._camera_name()
        if selected == self._camera_name():
            self._apply_camera_settings()
            selected = self._camera_name()
        self._camera_manager.request_snapshot(selected)
        if not self._camera_is_connected(selected):
            self._camera_manager.start(selected)
        if hasattr(self, "camera_status") and selected == self._camera_name():
            self.camera_status.setText(f"Capturing fresh frame from {selected}…")
        if hasattr(self, "teleop_camera_status") and selected == self._teleop_camera_name():
            self.teleop_camera_status.setText(
                f"Capturing fresh frame from {selected}…"
            )

    def _capture_teleop_camera_frame(self) -> None:
        self._capture_camera_frame(self._teleop_camera_name())

    @Slot(str, object)
    def _on_camera_frame(self, name: str, image: object) -> None:
        if not hasattr(image, "isNull") or image.isNull():
            return
        self._camera_latest_images[name] = image
        preview = getattr(self, "camera_preview_labels", {}).get(name)
        if preview is not None:
            self._set_preview_image(preview, image)
        if hasattr(self, "teleop_camera_preview") and name == self._teleop_camera_name():
            self._set_preview_image(self.teleop_camera_preview, image)

    @Slot(str, object)
    def _on_camera_status(self, name: str, status: object) -> None:
        values = dict(status)
        self._camera_status_by_name[name] = values
        self._camera_connected = self._camera_is_connected(self._camera_name())
        self._refresh_camera_display(name)
        if not bool(values.get("connected")) and not bool(values.get("recovering")):
            preview = getattr(self, "camera_preview_labels", {}).get(name)
            if preview is not None:
                preview.clear()
                preview.setText("Camera stopped")
            if hasattr(self, "teleop_camera_preview") and name == self._teleop_camera_name():
                self.teleop_camera_preview.clear()
                self.teleop_camera_preview.setText("Camera stopped")

    @Slot(str, str)
    def _on_camera_error(self, name: str, message: str) -> None:
        self._camera_status_by_name[name] = {
            "connected": False,
            "device": self._workstation_profile.cameras.get(
                name, self._camera_settings
            ).device,
            "error": message,
        }
        self._refresh_camera_display(name)
        if hasattr(self, "log"):
            self._log(f"Camera {name}: {message}")

    @Slot(str, str)
    def _on_camera_snapshot_saved(self, name: str, path: str) -> None:
        text = f"Captured {name}: {path}"
        if hasattr(self, "camera_status") and name == self._camera_name():
            self.camera_status.setText(text)
        if hasattr(self, "teleop_camera_status") and name == self._teleop_camera_name():
            self.teleop_camera_status.setText(text)
        self._log(text)

    def _on_teleop_camera_changed(self, name: str) -> None:
        name = str(name).strip()
        if not name:
            return
        self.teleop_camera_preview.clear()
        self.teleop_camera_preview.setText(
            "Live frames appear here when this camera is running."
        )
        self._refresh_camera_display(name)

    def _build_teleop_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(9)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.addWidget(self._build_coordination_panel())

        teleop = QGroupBox("Live leader → follower teleoperation")
        teleop_grid = QGridLayout(teleop)
        teleop_grid.addWidget(QLabel("Mapping"), 0, 0)
        self.teleop_mode_combo = QComboBox()
        self.teleop_mode_combo.addItem("Relative / clutch-safe", "relative")
        self.teleop_mode_combo.addItem("Absolute calibrated angles", "absolute")
        self.teleop_mode_combo.setCurrentIndex(self.teleop_mode_combo.findData("absolute"))
        teleop_grid.addWidget(self.teleop_mode_combo, 0, 1)

        teleop_grid.addWidget(QLabel("Rate"), 0, 2)
        self.teleop_rate_combo = QComboBox()
        self.teleop_rate_combo.addItem("5 Hz — slow check", 5.0)
        self.teleop_rate_combo.addItem("10 Hz", 10.0)
        self.teleop_rate_combo.addItem("20 Hz — default", 20.0)
        self.teleop_rate_combo.addItem("50 Hz — experimental", 50.0)
        default_rate_index = self.teleop_rate_combo.findData(
            DEFAULT_TELEOP_STREAM_FREQUENCY_HZ
        )
        if default_rate_index >= 0:
            self.teleop_rate_combo.setCurrentIndex(default_rate_index)
        teleop_grid.addWidget(self.teleop_rate_combo, 0, 3)

        self.teleop_gripper_check = QCheckBox("Mirror gripper; hold at contact")
        self.teleop_gripper_check.setChecked(True)
        self.teleop_gripper_check.setToolTip(
            "The follower closes gradually. If its gripper stops against an object, "
            "it holds that opening until you open the leader gripper."
        )
        teleop_grid.addWidget(self.teleop_gripper_check, 0, 4)
        teleop_grid.addWidget(QLabel("Gripper speed"), 1, 0)
        self.teleop_gripper_speed_combo = self._new_gripper_speed_combo()
        teleop_grid.addWidget(self.teleop_gripper_speed_combo, 1, 1)
        self.teleop_button = QPushButton("Align follower and start")
        self.teleop_button.clicked.connect(self._toggle_teleop)
        teleop_grid.addWidget(self.teleop_button, 0, 5)
        self.transfer_manual_button = QPushButton("Stop → Manual + park leader")
        self.transfer_manual_button.setToolTip(
            "Stop following, hold the follower, park the leader at its current pose, "
            "and open Manual for fine adjustment."
        )
        self.transfer_manual_button.clicked.connect(self._transfer_to_manual_and_park)
        teleop_grid.addWidget(self.transfer_manual_button, 1, 4, 1, 2)
        self.teleop_status = QLabel(
            "Connect both arms in Setup. Starting teleoperation reads the leader, "
            "holds the follower at its current pose, aligns its five joints, then follows live."
        )
        self.teleop_status.setWordWrap(True)
        teleop_grid.addWidget(self.teleop_status, 2, 0, 1, 6)
        left_layout.addWidget(teleop)

        camera_box = QGroupBox("Live camera")
        camera_layout = QVBoxLayout(camera_box)
        camera_selector = QHBoxLayout()
        camera_selector.addWidget(QLabel("View"))
        self.teleop_camera_combo = QComboBox()
        self.teleop_camera_combo.addItems(list(self._workstation_profile.cameras))
        if self._workstation_profile.selected_camera:
            self.teleop_camera_combo.setCurrentText(
                self._workstation_profile.selected_camera
            )
        self.teleop_camera_combo.currentTextChanged.connect(
            self._on_teleop_camera_changed
        )
        camera_selector.addWidget(self.teleop_camera_combo, 1)
        camera_selector.addWidget(
            self._help_button(
                "Teleoperation camera",
                "Select any saved camera by name. Cameras are configured in the Camera "
                "tab. More than one named camera may be streaming at the same time; this "
                "selector only chooses which stream is shown here.",
            )
        )
        camera_layout.addLayout(camera_selector)

        self.teleop_camera_preview = self._new_camera_preview_label(minimum_height=260)
        camera_layout.addWidget(self.teleop_camera_preview)
        camera_controls = QHBoxLayout()
        self.teleop_camera_status = QLabel(
            self._camera_status_text(self._teleop_camera_name())
        )
        self.teleop_camera_status.setWordWrap(True)
        camera_controls.addWidget(self.teleop_camera_status, 1)
        self.teleop_camera_toggle_button = QPushButton("Start camera")
        self.teleop_camera_toggle_button.clicked.connect(
            self._toggle_teleop_camera_stream
        )
        camera_controls.addWidget(self.teleop_camera_toggle_button)
        teleop_capture_button = QPushButton("Capture picture")
        teleop_capture_button.clicked.connect(self._capture_teleop_camera_frame)
        camera_controls.addWidget(teleop_capture_button)
        camera_layout.addLayout(camera_controls)
        left_layout.addWidget(camera_box)

        self.teleop_readout = QLabel("Connect both arms to see live measurements.")
        self.teleop_readout.setTextFormat(Qt.TextFormat.PlainText)
        self.teleop_readout.setStyleSheet(
            "font-family: monospace; padding: 12px; border-radius: 10px; "
            "background: palette(alternate-base);"
        )
        self.teleop_readout.setAlignment(Qt.AlignmentFlag.AlignTop)
        left_layout.addWidget(self.teleop_readout, 1)
        layout.addWidget(left, 1)
        self.teleop_arm_panel = self.robot_sidebar
        return page

    def _build_teach_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(9)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)

        teach_heading = QHBoxLayout()
        teach_heading.addWidget(QLabel("Save positions or record continuous motion."))
        teach_heading.addStretch(1)
        teach_heading.addWidget(
            self._help_button(
                "Teach / Record",
                "Saved positions are deterministic destinations for Programs. Continuous "
                "recordings preserve demonstration paths for replay, editing, and future "
                "Robo Puppeteer / learning workflows. Neither workflow replaces the other.",
            )
        )
        left_layout.addLayout(teach_heading)

        source = QGroupBox("Teaching source")
        source_grid = QGridLayout(source)
        source_grid.addWidget(QLabel("Capture/read from"), 0, 0)
        self.teaching_source_combo = QComboBox()
        self.teaching_source_combo.addItem("Follower", "follower")
        self.teaching_source_combo.addItem("Leader", "leader")
        self.teaching_source_combo.currentIndexChanged.connect(
            lambda _index: self._update_teach_readout()
        )
        source_grid.addWidget(self.teaching_source_combo, 0, 1)
        self.teach_source_status = QLabel("Follower state unavailable")
        source_grid.addWidget(self.teach_source_status, 0, 2, 1, 4)

        self.teach_joint_labels: dict[str, QLabel] = {}
        for row, name in enumerate(ARM_JOINTS, start=1):
            source_grid.addWidget(QLabel(name.replace("_", " ").title()), row, 0)
            label = QLabel("—")
            self.teach_joint_labels[name] = label
            source_grid.addWidget(label, row, 1)

        source_grid.addWidget(QLabel("Gripper"), 1, 3)
        self.teach_gripper_label = QLabel("—")
        source_grid.addWidget(self.teach_gripper_label, 1, 4)
        self.teach_pose_label = QLabel("TCP: —")
        self.teach_pose_label.setWordWrap(True)
        source_grid.addWidget(self.teach_pose_label, 2, 3, 2, 3)
        left_layout.addWidget(source)

        points = QGroupBox("Saved positions")
        point_grid = QGridLayout(points)
        point_grid.addWidget(QLabel("New position name"), 0, 0)
        self.point_name_edit = QLineEdit()
        self.point_name_edit.setPlaceholderText("pick, above_drop, inspect, park...")
        point_grid.addWidget(self.point_name_edit, 0, 1, 1, 2)
        self.save_point_button = QPushButton("Save current position")
        self.save_point_button.setToolTip(
            "Capture a fresh measured position from the selected teaching source."
        )
        self.save_point_button.clicked.connect(self._save_taught_point)
        point_grid.addWidget(self.save_point_button, 0, 3)

        point_grid.addWidget(QLabel("Saved position"), 1, 0)
        self.point_combo = QComboBox()
        self.point_combo.currentTextChanged.connect(
            lambda _text: self._preview_selected_taught_point()
        )
        point_grid.addWidget(self.point_combo, 1, 1)
        self.point_mode_combo = QComboBox()
        self.point_mode_combo.addItem("Joint / angular", "joint")
        self.point_mode_combo.addItem("Cartesian linear", "linear")
        point_grid.addWidget(self.point_mode_combo, 1, 2)
        self.move_point_button = QPushButton("Move follower here")
        self.move_point_button.clicked.connect(self._move_taught_point)
        point_grid.addWidget(self.move_point_button, 1, 3)

        self.add_point_to_program_button = QPushButton("Add position to current program")
        self.add_point_to_program_button.setToolTip(
            "Append a move to this saved position using the current point mode."
        )
        self.add_point_to_program_button.clicked.connect(
            self._add_selected_taught_point_to_program
        )
        point_grid.addWidget(self.add_point_to_program_button, 2, 0, 1, 3)
        self.delete_point_button = QPushButton("Delete position")
        self.delete_point_button.clicked.connect(self._delete_taught_point)
        point_grid.addWidget(self.delete_point_button, 2, 3)
        left_layout.addWidget(points)

        recording = QGroupBox("Trajectory recording / demonstration data")
        record_grid = QGridLayout(recording)
        record_grid.addWidget(QLabel("Name"), 0, 0)
        self.recording_name_edit = QLineEdit()
        self.recording_name_edit.setPlaceholderText("wave_raw_01")
        record_grid.addWidget(self.recording_name_edit, 0, 1, 1, 2)
        self.record_effort_check = QCheckBox("Record effort/current diagnostics")
        record_grid.addWidget(self.record_effort_check, 1, 0, 1, 2)
        self.recording_rate_label = QLabel("50 Hz")
        record_grid.addWidget(self.recording_rate_label, 1, 2)
        self.record_button = QPushButton("Start recording selected source")
        self.record_button.clicked.connect(self._toggle_recording)
        record_grid.addWidget(self.record_button, 0, 3, 2, 1)
        self.recording_status = QLabel(
            "Record the complete demonstrated motion when path/timing matters or when "
            "capturing motion data for later consumers. Raw recordings are immutable; "
            "editing creates a derived trajectory."
        )
        self.recording_status.setWordWrap(True)
        record_grid.addWidget(self.recording_status, 2, 0, 1, 4)
        left_layout.addWidget(recording)

        go_programs = QPushButton("Open Programs")
        go_programs.clicked.connect(lambda: self.tabs.setCurrentWidget(self.run_page))
        left_layout.addWidget(go_programs)
        left_layout.addStretch(1)
        layout.addWidget(left, 1)
        self.teach_arm_panel = self.robot_sidebar
        return page

    def _build_trajectory_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        library_box = QGroupBox("Trajectory library")
        library_grid = QGridLayout(library_box)
        library_grid.addWidget(QLabel("Trajectory"), 0, 0)
        self.trajectory_combo = QComboBox()
        library_grid.addWidget(self.trajectory_combo, 0, 1)
        self.refresh_trajectory_button = QPushButton("Refresh")
        self.refresh_trajectory_button.clicked.connect(self._refresh_trajectory_list)
        library_grid.addWidget(self.refresh_trajectory_button, 0, 2)
        self.load_trajectory_button = QPushButton("Load")
        self.load_trajectory_button.clicked.connect(self._load_selected_trajectory)
        library_grid.addWidget(self.load_trajectory_button, 0, 3)
        self.trajectory_stats = QLabel("No trajectory loaded")
        library_grid.addWidget(self.trajectory_stats, 1, 0, 1, 4)
        layout.addWidget(library_box)

        self.trajectory_timeline = TrajectoryTimeline()
        layout.addWidget(self.trajectory_timeline, 1)
        self.trajectory_arm_panel = self.robot_sidebar

        edit_box = QGroupBox("Selection and playback")
        edit_grid = QGridLayout(edit_box)
        edit_grid.addWidget(QLabel("Scrub"), 0, 0)
        self.trajectory_scrub = QSlider(Qt.Orientation.Horizontal)
        self.trajectory_scrub.setRange(0, 10000)
        self.trajectory_scrub.valueChanged.connect(self._trajectory_scrub_changed)
        edit_grid.addWidget(self.trajectory_scrub, 0, 1, 1, 5)
        self.trajectory_cursor_label = QLabel("0.000 s")
        edit_grid.addWidget(self.trajectory_cursor_label, 0, 6)

        edit_grid.addWidget(QLabel("Selection start"), 1, 0)
        self.selection_start = self._spin(0.0, 0.0, 0.0, decimals=3, step=0.02, suffix=" s")
        self.selection_start.valueChanged.connect(lambda _value: self._selection_changed())
        edit_grid.addWidget(self.selection_start, 1, 1)
        self.cursor_to_start_button = QPushButton("Start = cursor")
        self.cursor_to_start_button.clicked.connect(
            lambda _checked=False: self._set_selection_from_cursor("start")
        )
        edit_grid.addWidget(self.cursor_to_start_button, 1, 2)

        edit_grid.addWidget(QLabel("Selection end"), 1, 3)
        self.selection_end = self._spin(0.0, 0.0, 0.0, decimals=3, step=0.02, suffix=" s")
        self.selection_end.valueChanged.connect(lambda _value: self._selection_changed())
        edit_grid.addWidget(self.selection_end, 1, 4)
        self.cursor_to_end_button = QPushButton("End = cursor")
        self.cursor_to_end_button.clicked.connect(
            lambda _checked=False: self._set_selection_from_cursor("end")
        )
        edit_grid.addWidget(self.cursor_to_end_button, 1, 5)

        edit_grid.addWidget(QLabel("Speed scale"), 2, 0)
        self.trajectory_speed_scale = self._spin(
            0.1, 3.0, 1.0, decimals=2, step=0.1, suffix="×"
        )
        edit_grid.addWidget(self.trajectory_speed_scale, 2, 1)
        self.replay_full_button = QPushButton("Replay full")
        self.replay_full_button.clicked.connect(
            lambda _checked=False: self._replay_trajectory(selection=False)
        )
        edit_grid.addWidget(self.replay_full_button, 2, 2)
        self.replay_selection_button = QPushButton("Replay selection")
        self.replay_selection_button.clicked.connect(
            lambda _checked=False: self._replay_trajectory(selection=True)
        )
        edit_grid.addWidget(self.replay_selection_button, 2, 3)
        edit_grid.addWidget(QLabel("Gripper speed"), 2, 4)
        self.trajectory_gripper_speed_combo = self._new_gripper_speed_combo()
        edit_grid.addWidget(self.trajectory_gripper_speed_combo, 2, 5)

        edit_grid.addWidget(QLabel("Save selection as"), 3, 0)
        self.edited_trajectory_name = QLineEdit()
        self.edited_trajectory_name.setPlaceholderText("wave_trimmed_v1")
        edit_grid.addWidget(self.edited_trajectory_name, 3, 1, 1, 3)
        self.save_edited_trajectory_button = QPushButton("Save derived clip")
        self.save_edited_trajectory_button.clicked.connect(self._save_edited_trajectory)
        edit_grid.addWidget(self.save_edited_trajectory_button, 3, 4, 1, 2)

        advanced_box = QGroupBox("Advanced non-destructive editing and primitives")
        advanced_grid = QGridLayout(advanced_box)

        advanced_grid.addWidget(QLabel("Smooth window"), 0, 0)
        self.smooth_window_spin = QSpinBox()
        self.smooth_window_spin.setRange(3, 51)
        self.smooth_window_spin.setSingleStep(2)
        self.smooth_window_spin.setValue(5)
        advanced_grid.addWidget(self.smooth_window_spin, 0, 1)
        self.smooth_button = QPushButton("Apply smoothing")
        self.smooth_button.clicked.connect(self._apply_smoothing)
        advanced_grid.addWidget(self.smooth_button, 0, 2)

        self.delete_selection_button = QPushButton("Delete selected region")
        self.delete_selection_button.clicked.connect(self._delete_trajectory_selection)
        advanced_grid.addWidget(self.delete_selection_button, 0, 3)

        advanced_grid.addWidget(QLabel("Hold"), 1, 0)
        self.hold_duration_spin = self._spin(
            0.02, 30.0, 0.5, decimals=2, step=0.1, suffix=" s"
        )
        advanced_grid.addWidget(self.hold_duration_spin, 1, 1)
        self.insert_hold_button = QPushButton("Insert hold at cursor")
        self.insert_hold_button.clicked.connect(self._insert_trajectory_hold)
        advanced_grid.addWidget(self.insert_hold_button, 1, 2)

        advanced_grid.addWidget(QLabel("Keyframe channel"), 2, 0)
        self.keyframe_channel_combo = QComboBox()
        for name in ARM_JOINTS:
            self.keyframe_channel_combo.addItem(name.replace("_", " ").title(), name)
        self.keyframe_channel_combo.addItem("Gripper", "gripper")
        self.keyframe_channel_combo.currentIndexChanged.connect(
            lambda _index: self._update_keyframe_units()
        )
        advanced_grid.addWidget(self.keyframe_channel_combo, 2, 1)
        self.keyframe_value_spin = self._spin(
            -180.0, 180.0, 0.0, decimals=2, step=1.0, suffix="°"
        )
        advanced_grid.addWidget(self.keyframe_value_spin, 2, 2)
        self.set_keyframe_button = QPushButton("Set at cursor")
        self.set_keyframe_button.clicked.connect(self._set_trajectory_keyframe)
        advanced_grid.addWidget(self.set_keyframe_button, 2, 3)

        advanced_grid.addWidget(QLabel("Marker"), 3, 0)
        self.marker_label_edit = QLineEdit()
        self.marker_label_edit.setPlaceholderText("contact, beat, release...")
        advanced_grid.addWidget(self.marker_label_edit, 3, 1, 1, 2)
        self.add_marker_button = QPushButton("Add marker at cursor")
        self.add_marker_button.clicked.connect(self._add_trajectory_marker)
        advanced_grid.addWidget(self.add_marker_button, 3, 3)

        advanced_grid.addWidget(QLabel("Loop selection"), 4, 0)
        self.loop_count_spin = QSpinBox()
        self.loop_count_spin.setRange(2, 100)
        self.loop_count_spin.setValue(2)
        advanced_grid.addWidget(self.loop_count_spin, 4, 1)
        self.loop_selection_button = QPushButton("Make repeated clip")
        self.loop_selection_button.clicked.connect(self._loop_trajectory_selection)
        advanced_grid.addWidget(self.loop_selection_button, 4, 2)

        advanced_grid.addWidget(QLabel("Primitive name"), 5, 0)
        self.primitive_name_edit = QLineEdit()
        self.primitive_name_edit.setPlaceholderText("ember_wave")
        advanced_grid.addWidget(self.primitive_name_edit, 5, 1)
        self.primitive_tags_edit = QLineEdit()
        self.primitive_tags_edit.setPlaceholderText("greeting, happy")
        advanced_grid.addWidget(self.primitive_tags_edit, 5, 2)
        self.primitive_loopable_check = QCheckBox("Loopable")
        advanced_grid.addWidget(self.primitive_loopable_check, 5, 3)
        self.primitive_interruptible_check = QCheckBox("Interruptible")
        self.primitive_interruptible_check.setChecked(True)
        advanced_grid.addWidget(self.primitive_interruptible_check, 6, 0)
        self.promote_primitive_button = QPushButton("Promote saved trajectory to primitive")
        self.promote_primitive_button.clicked.connect(self._promote_trajectory_primitive)
        advanced_grid.addWidget(self.promote_primitive_button, 6, 1, 1, 3)

        editor_tabs = QTabWidget()
        editor_tabs.addTab(edit_box, "Trim / Replay")
        editor_tabs.addTab(advanced_box, "Advanced / Primitives")
        layout.addWidget(editor_tabs)
        return page

    def _build_run_tab(self) -> QWidget:
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(9)

        left = QWidget()
        layout = QVBoxLayout(left)
        layout.setContentsMargins(0, 0, 0, 0)

        intro = QLabel(
            "Build a deterministic program by arranging saved positions and simple "
            "actions. The rows execute from top to bottom; recorded trajectories remain "
            "available as an advanced step when a continuous path matters."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)

        library_box = QGroupBox("Program library")
        library_grid = QGridLayout(library_box)
        library_grid.addWidget(QLabel("Saved program"), 0, 0)
        self.sequence_combo = QComboBox()
        library_grid.addWidget(self.sequence_combo, 0, 1)
        self.load_sequence_button = QPushButton("Load")
        self.load_sequence_button.clicked.connect(self._load_sequence)
        library_grid.addWidget(self.load_sequence_button, 0, 2)
        self.delete_sequence_button = QPushButton("Delete")
        self.delete_sequence_button.clicked.connect(self._delete_sequence)
        library_grid.addWidget(self.delete_sequence_button, 0, 3)
        library_grid.addWidget(QLabel("Program name"), 1, 0)
        self.sequence_name_edit = QLineEdit()
        self.sequence_name_edit.setPlaceholderText("pick_and_place")
        library_grid.addWidget(self.sequence_name_edit, 1, 1, 1, 2)
        self.save_sequence_button = QPushButton("Save / replace program")
        self.save_sequence_button.clicked.connect(self._save_sequence)
        library_grid.addWidget(self.save_sequence_button, 1, 3)
        layout.addWidget(library_box)

        step_tabs = QTabWidget()

        simple = QWidget()
        simple_grid = QGridLayout(simple)
        simple_grid.addWidget(QLabel("Saved position"), 0, 0)
        self.run_point_combo = QComboBox()
        self.run_point_combo.currentTextChanged.connect(
            lambda _text: self._preview_program_position()
        )
        simple_grid.addWidget(self.run_point_combo, 0, 1)
        self.run_point_mode_combo = QComboBox()
        self.run_point_mode_combo.addItem("Joint / angular", "joint")
        self.run_point_mode_combo.addItem("Cartesian linear", "linear")
        simple_grid.addWidget(self.run_point_mode_combo, 0, 2)

        simple_grid.addWidget(QLabel("Move speed"), 0, 3)
        self.program_move_speed_spin = self._spin(
            0.1, 3.0, 1.0, decimals=2, step=0.1, suffix="×"
        )
        self.program_move_speed_spin.setToolTip(
            "Per-move speed multiplier. This is multiplied by the program's overall speed."
        )
        simple_grid.addWidget(self.program_move_speed_spin, 0, 4)
        self.add_point_step_button = QPushButton("+ Move to position")
        self.add_point_step_button.clicked.connect(self._add_point_sequence_step)
        simple_grid.addWidget(self.add_point_step_button, 0, 5)

        self.add_home_step_button = QPushButton("+ Home")
        self.add_home_step_button.clicked.connect(
            lambda _checked=False: self._add_named_pose_program_step("home")
        )
        simple_grid.addWidget(self.add_home_step_button, 1, 0)
        self.add_rest_step_button = QPushButton("+ Rest")
        self.add_rest_step_button.clicked.connect(
            lambda _checked=False: self._add_named_pose_program_step("rest")
        )
        simple_grid.addWidget(self.add_rest_step_button, 1, 1)

        self.program_close_gripper_button = QPushButton("+ Close gripper")
        self.program_close_gripper_button.clicked.connect(
            lambda _checked=False: self._add_gripper_sequence_step(0.0)
        )
        simple_grid.addWidget(self.program_close_gripper_button, 1, 2)
        self.program_open_gripper_button = QPushButton("+ Open gripper")
        self.program_open_gripper_button.clicked.connect(
            lambda _checked=False: self._add_gripper_sequence_step(1.0)
        )
        simple_grid.addWidget(self.program_open_gripper_button, 1, 3)

        self.sequence_gripper_spin = self._spin(
            0.0, 1.0, 0.5, decimals=3, step=0.05
        )
        self.sequence_gripper_spin.setToolTip(
            "Custom normalized gripper position: 0 closed, 1 open."
        )
        simple_grid.addWidget(self.sequence_gripper_spin, 1, 4)
        self.add_gripper_step_button = QPushButton("+ Custom gripper")
        self.add_gripper_step_button.clicked.connect(
            lambda _checked=False: self._add_gripper_sequence_step()
        )
        simple_grid.addWidget(self.add_gripper_step_button, 1, 5)

        simple_grid.addWidget(QLabel("Wait"), 2, 0)
        self.sequence_wait_spin = self._spin(
            0.0, 60.0, 0.25, decimals=2, step=0.25, suffix=" s"
        )
        simple_grid.addWidget(self.sequence_wait_spin, 2, 1)
        self.add_wait_step_button = QPushButton("+ Wait")
        self.add_wait_step_button.clicked.connect(self._add_wait_sequence_step)
        simple_grid.addWidget(self.add_wait_step_button, 2, 2)

        simple_hint = QLabel(
            "Typical program: MOVE above_pick → MOVE pick → CLOSE → MOVE above_pick "
            "→ MOVE above_drop → MOVE drop → OPEN."
        )
        simple_hint.setWordWrap(True)
        simple_hint.setStyleSheet("color: palette(mid); padding-top: 4px;")
        simple_grid.addWidget(simple_hint, 3, 0, 1, 6)

        pan_box = QGroupBox("Radial / shoulder-pan pattern")
        pan_grid = QGridLayout(pan_box)
        pan_note = QLabel(
            "Use the selected saved position as the base. Every generated Move keeps "
            "shoulder lift, elbow, wrists, and gripper unchanged and varies only "
            "shoulder pan by an offset from the saved angle."
        )
        pan_note.setWordWrap(True)
        pan_grid.addWidget(pan_note, 0, 0, 1, 6)
        pan_grid.addWidget(QLabel("Start offset"), 1, 0)
        self.radial_pan_start_spin = self._spin(
            -180.0, 180.0, -45.0, decimals=1, step=5.0, suffix="°"
        )
        pan_grid.addWidget(self.radial_pan_start_spin, 1, 1)
        pan_grid.addWidget(QLabel("End offset"), 1, 2)
        self.radial_pan_end_spin = self._spin(
            -180.0, 180.0, 45.0, decimals=1, step=5.0, suffix="°"
        )
        pan_grid.addWidget(self.radial_pan_end_spin, 1, 3)
        pan_grid.addWidget(QLabel("Increment"), 1, 4)
        self.radial_pan_step_spin = self._spin(
            0.5, 180.0, 15.0, decimals=1, step=5.0, suffix="°"
        )
        pan_grid.addWidget(self.radial_pan_step_spin, 1, 5)
        self.add_radial_pan_pattern_button = QPushButton("+ Append pan pattern")
        self.add_radial_pan_pattern_button.setToolTip(
            "Generate ordinary guarded joint Move steps. The base saved position is "
            "not modified and no new saved positions are created."
        )
        self.add_radial_pan_pattern_button.clicked.connect(self._add_radial_pan_pattern)
        pan_grid.addWidget(self.add_radial_pan_pattern_button, 2, 0, 1, 6)
        simple_grid.addWidget(pan_box, 4, 0, 1, 6)

        step_tabs.addTab(simple, "Position steps")

        advanced = QWidget()
        advanced_grid = QGridLayout(advanced)
        advanced_grid.addWidget(
            QLabel(
                "Insert a recorded trajectory or reusable motion primitive into the "
                "same deterministic Program. Recording/replay remains a separate "
                "first-class workflow in Teach / Record and Edit recordings."
            ),
            0,
            0,
            1,
            4,
        )
        self.run_trajectory_combo = QComboBox()
        advanced_grid.addWidget(self.run_trajectory_combo, 1, 0, 1, 2)
        self.add_trajectory_step_button = QPushButton("+ Recorded trajectory")
        self.add_trajectory_step_button.clicked.connect(self._add_trajectory_sequence_step)
        advanced_grid.addWidget(self.add_trajectory_step_button, 1, 2)

        self.run_primitive_combo = QComboBox()
        advanced_grid.addWidget(self.run_primitive_combo, 2, 0, 1, 2)
        self.add_primitive_step_button = QPushButton("+ Motion primitive")
        self.add_primitive_step_button.clicked.connect(self._add_primitive_sequence_step)
        advanced_grid.addWidget(self.add_primitive_step_button, 2, 2)
        step_tabs.addTab(advanced, "Trajectories / primitives")
        layout.addWidget(step_tabs)

        program_box = QGroupBox("Program steps · top to bottom")
        program_layout = QVBoxLayout(program_box)
        self.sequence_step_list = QListWidget()
        self.sequence_step_list.setAlternatingRowColors(True)
        self.sequence_step_list.itemSelectionChanged.connect(
            self._preview_selected_program_step
        )
        self.sequence_step_list.itemDoubleClicked.connect(
            lambda _item: self._run_sequence(step_only=True)
        )
        program_layout.addWidget(self.sequence_step_list, 1)

        edit_row = QHBoxLayout()
        self.sequence_up_button = QPushButton("↑ Move up")
        self.sequence_up_button.clicked.connect(lambda _checked=False: self._move_sequence_step(-1))
        edit_row.addWidget(self.sequence_up_button)
        self.sequence_down_button = QPushButton("↓ Move down")
        self.sequence_down_button.clicked.connect(lambda _checked=False: self._move_sequence_step(1))
        edit_row.addWidget(self.sequence_down_button)
        self.sequence_delete_step_button = QPushButton("Delete step")
        self.sequence_delete_step_button.clicked.connect(self._delete_sequence_step)
        edit_row.addWidget(self.sequence_delete_step_button)
        edit_row.addStretch(1)
        program_layout.addLayout(edit_row)
        layout.addWidget(program_box, 1)

        run_box = QGroupBox("Run program")
        run_grid = QGridLayout(run_box)
        run_grid.addWidget(QLabel("Repeat"), 0, 0)
        self.sequence_repeat_spin = QSpinBox()
        self.sequence_repeat_spin.setRange(1, 1000)
        self.sequence_repeat_spin.setValue(1)
        run_grid.addWidget(self.sequence_repeat_spin, 0, 1)
        run_grid.addWidget(QLabel("Overall speed"), 0, 2)
        self.sequence_speed_spin = self._spin(
            0.1, 3.0, 1.0, decimals=2, step=0.1, suffix="×"
        )
        run_grid.addWidget(self.sequence_speed_spin, 0, 3)
        run_grid.addWidget(QLabel("Gripper speed"), 0, 4)
        self.run_gripper_speed_combo = self._new_gripper_speed_combo()
        run_grid.addWidget(self.run_gripper_speed_combo, 0, 5)

        self.run_step_button = QPushButton("Run selected step")
        self.run_step_button.clicked.connect(
            lambda _checked=False: self._run_sequence(step_only=True)
        )
        run_grid.addWidget(self.run_step_button, 1, 0, 1, 2)
        self.run_sequence_button = QPushButton("Run full program")
        self.run_sequence_button.clicked.connect(
            lambda _checked=False: self._run_sequence(step_only=False)
        )
        run_grid.addWidget(self.run_sequence_button, 1, 2, 1, 2)
        self.pause_sequence_button = QPushButton("Pause after current step")
        self.pause_sequence_button.clicked.connect(self._toggle_sequence_pause)
        run_grid.addWidget(self.pause_sequence_button, 1, 4)
        self.stop_sequence_button = QPushButton("STOP / HOLD")
        self.stop_sequence_button.clicked.connect(
            lambda _checked=False: self.stop_requested.emit()
        )
        run_grid.addWidget(self.stop_sequence_button, 1, 5)
        self.sequence_status = QLabel("No program running")
        self.sequence_status.setWordWrap(True)
        run_grid.addWidget(self.sequence_status, 2, 0, 1, 6)
        layout.addWidget(run_box)

        outer.addWidget(left, 1)
        self.program_arm_panel = self.robot_sidebar
        return page

    def _build_joint_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.edit_joint_targets_check = QCheckBox("Edit joint targets")
        self.edit_joint_targets_check.setToolTip(
            "Off: targets follow the measured follower pose. On: set target angles "
            "with the sliders or numeric boxes, then press Move."
        )
        self.edit_joint_targets_check.toggled.connect(self._on_edit_joint_targets)
        layout.addWidget(self.edit_joint_targets_check)
        box = QGroupBox("Absolute joint targets")
        box.setMaximumWidth(700)
        grid = QGridLayout(box)
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Target"), 0, 1)
        grid.addWidget(QLabel(""), 0, 2)
        grid.addWidget(QLabel("Measured"), 0, 3)

        self.joint_sliders: dict[str, QSlider] = {}
        self.joint_spins: dict[str, QDoubleSpinBox] = {}
        self.joint_actual_labels: dict[str, QLabel] = {}

        for row, name in enumerate(ARM_JOINTS, start=1):
            lower, upper = JOINT_LIMITS[name]
            lower_deg = degrees(lower)
            upper_deg = degrees(upper)
            grid.addWidget(QLabel(name.replace("_", " ").title()), row, 0)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(round(lower_deg * 10), round(upper_deg * 10))
            slider.setSingleStep(1)
            slider.setPageStep(10)
            slider.setMaximumWidth(270)
            spin = self._spin(
                lower_deg,
                upper_deg,
                0.0,
                decimals=1,
                step=0.5,
                suffix="°",
            )
            actual = QLabel("—")
            actual.setMinimumWidth(75)
            spin.setMinimumWidth(95)

            slider.valueChanged.connect(
                lambda value, control=spin: self._sync_slider_to_spin(value, control)
            )
            spin.valueChanged.connect(
                lambda value, control=slider: self._sync_spin_to_slider(value, control)
            )

            self.joint_sliders[name] = slider
            self.joint_spins[name] = spin
            self.joint_actual_labels[name] = actual
            grid.addWidget(spin, row, 1)
            grid.addWidget(slider, row, 2)
            grid.addWidget(actual, row, 3)

        layout.addWidget(box, alignment=Qt.AlignmentFlag.AlignLeft)

        motion = QGroupBox("Joint motion")
        form = QFormLayout(motion)
        self.joint_speed = self._spin(0.5, 57.0, 8.0, decimals=1, step=1.0, suffix=" °/s")
        self.joint_acceleration = self._spin(
            1.0, 286.0, 25.0, decimals=1, step=5.0, suffix=" °/s²"
        )
        form.addRow("Speed", self.joint_speed)
        form.addRow("Acceleration", self.joint_acceleration)
        self.move_joints_button = QPushButton("Move to joint targets")
        self.move_joints_button.clicked.connect(self._move_joints)
        form.addRow(self.move_joints_button)
        layout.addWidget(motion)
        layout.addStretch(1)
        return page

    def _on_edit_joint_targets(self, editing: bool) -> None:
        if not editing:
            self._load_current_targets()
        self._update_enabled_state()

    @staticmethod
    def _sync_slider_to_spin(value: int, spin: QDoubleSpinBox) -> None:
        spin.blockSignals(True)
        spin.setValue(value / 10.0)
        spin.blockSignals(False)

    @staticmethod
    def _sync_spin_to_slider(value: float, slider: QSlider) -> None:
        slider.blockSignals(True)
        slider.setValue(round(value * 10.0))
        slider.blockSignals(False)

    def _build_cartesian_tab(self) -> QWidget:
        page = QWidget()
        outer = QHBoxLayout(page)

        controls = QWidget()
        layout = QVBoxLayout(controls)
        outer.addWidget(controls, 3)

        current_box = QGroupBox("Measured TCP in robot world/base frame")
        current_grid = QGridLayout(current_box)
        self.pose_value_labels: list[QLabel] = []
        for column, label in enumerate(("X mm", "Y mm", "Z mm", "Roll °", "Pitch °", "Yaw °")):
            current_grid.addWidget(QLabel(label), 0, column)
            value = QLabel("—")
            value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            value.setMinimumWidth(74)
            self.pose_value_labels.append(value)
            current_grid.addWidget(value, 1, column)
        layout.addWidget(current_box)

        absolute_box = QGroupBox("Absolute world pose — guarded linear move")
        absolute_grid = QGridLayout(absolute_box)
        self.absolute_spins: list[QDoubleSpinBox] = []
        absolute_specs = (
            ("X", -600.0, 600.0, " mm"),
            ("Y", -600.0, 600.0, " mm"),
            ("Z", -100.0, 600.0, " mm"),
            ("Roll", -180.0, 180.0, "°"),
            ("Pitch", -180.0, 180.0, "°"),
            ("Yaw", -180.0, 180.0, "°"),
        )
        for column, (label, minimum, maximum, suffix) in enumerate(absolute_specs):
            absolute_grid.addWidget(QLabel(label), 0, column)
            spin = self._spin(minimum, maximum, 0.0, decimals=2, step=1.0, suffix=suffix)
            self.absolute_spins.append(spin)
            absolute_grid.addWidget(spin, 1, column)

        self.use_current_pose_button = QPushButton("Use measured pose")
        self.use_current_pose_button.clicked.connect(self._load_current_pose)
        absolute_grid.addWidget(self.use_current_pose_button, 2, 0, 1, 2)
        self.absolute_move_button = QPushButton("Move linear to world pose")
        self.absolute_move_button.clicked.connect(self._move_absolute_pose)
        absolute_grid.addWidget(self.absolute_move_button, 2, 2, 1, 4)
        layout.addWidget(absolute_box)

        jog_box = QGroupBox("Relative linear jog")
        jog_grid = QGridLayout(jog_box)
        jog_grid.addWidget(QLabel("Frame"), 0, 0)
        self.frame_combo = QComboBox()
        self.frame_combo.addItem("World / base", "world")
        self.frame_combo.addItem("Tool / TCP", "tool")
        jog_grid.addWidget(self.frame_combo, 0, 1)

        jog_grid.addWidget(QLabel("Orientation"), 0, 2)
        self.orientation_combo = QComboBox()
        self.orientation_combo.addItem("5-axis compatible", "compatible")
        self.orientation_combo.addItem("Position only", "position_only")
        self.orientation_combo.addItem("Exact when reachable", "exact")
        jog_grid.addWidget(self.orientation_combo, 0, 3)

        jog_grid.addWidget(QLabel("Linear step"), 1, 0)
        self.linear_step = self._spin(0.1, 50.0, 2.0, decimals=1, step=0.5, suffix=" mm")
        jog_grid.addWidget(self.linear_step, 1, 1)
        jog_grid.addWidget(QLabel("Angular step"), 1, 2)
        self.angular_step = self._spin(0.1, 30.0, 2.0, decimals=1, step=1.0, suffix="°")
        jog_grid.addWidget(self.angular_step, 1, 3)

        jog_grid.addWidget(QLabel("Linear speed"), 2, 0)
        self.linear_speed = self._spin(0.5, 60.0, 10.0, decimals=1, step=1.0, suffix=" mm/s")
        jog_grid.addWidget(self.linear_speed, 2, 1)
        jog_grid.addWidget(QLabel("Linear acceleration"), 2, 2)
        self.linear_acceleration = self._spin(
            1.0, 200.0, 40.0, decimals=1, step=5.0, suffix=" mm/s²"
        )
        jog_grid.addWidget(self.linear_acceleration, 2, 3)

        axes = (("X", 0), ("Y", 1), ("Z", 2), ("Roll", 3), ("Pitch", 4), ("Yaw", 5))
        self.jog_buttons: list[QPushButton] = []
        for index, (label, axis) in enumerate(axes):
            row = 3 + index // 3
            column = (index % 3) * 2
            minus = QPushButton(f"{label} −")
            plus = QPushButton(f"{label} +")
            minus.clicked.connect(lambda _=False, item=axis: self._jog(item, -1.0))
            plus.clicked.connect(lambda _=False, item=axis: self._jog(item, 1.0))
            self.jog_buttons.extend((minus, plus))
            jog_grid.addWidget(minus, row, column)
            jog_grid.addWidget(plus, row, column + 1)

        help_label = QLabel(
            "Each click is a guarded Cartesian path. Repeated clicks are queued and run "
            "sequentially; STOP / HOLD cancels the active move and clears the queue. "
            "Tool-frame XYZ follows the current gripper axes."
        )
        help_label.setWordWrap(True)
        jog_grid.addWidget(help_label, 5, 0, 1, 6)

        self.jog_queue_label = QLabel("Jog queue: idle")
        self.jog_queue_label.setStyleSheet("font-weight: 600;")
        jog_grid.addWidget(self.jog_queue_label, 6, 0, 1, 2)
        self.cartesian_diag_label = QLabel(
            "Jog diagnostics: requested and achieved TCP will appear here."
        )
        self.cartesian_diag_label.setWordWrap(True)
        jog_grid.addWidget(self.cartesian_diag_label, 6, 2, 1, 4)
        layout.addWidget(jog_box)
        layout.addStretch(1)

        return page

    def _build_gripper_panel(self) -> QGroupBox:
        box = QGroupBox("Gripper — tool control")
        grid = QGridLayout(box)

        self.gripper_slider = QSlider(Qt.Orientation.Horizontal)
        self.gripper_slider.setRange(0, 1000)
        self.gripper_slider.setValue(1000)
        self.gripper_spin = self._spin(0.0, 1.0, 1.0, decimals=3, step=0.05)
        self.gripper_slider.valueChanged.connect(
            lambda value: self._set_gripper_spin(value / 1000.0)
        )
        self.gripper_spin.valueChanged.connect(
            lambda value: self._set_gripper_slider(value)
        )
        grid.addWidget(QLabel("Target · 0 closed · 1 open"), 0, 0)
        grid.addWidget(self.gripper_spin, 0, 1)
        grid.addWidget(self.gripper_slider, 0, 2, 1, 3)

        grid.addWidget(QLabel("Speed"), 1, 0)
        self.manual_gripper_speed_combo = self._new_gripper_speed_combo()
        grid.addWidget(self.manual_gripper_speed_combo, 1, 1)
        self.gripper_measured = QLabel("Measured: —")
        grid.addWidget(self.gripper_measured, 1, 2)

        self.close_gripper_button = QPushButton("Close")
        self.close_gripper_button.clicked.connect(lambda: self._request_gripper(0.0))
        self.move_gripper_button = QPushButton("Move to value")
        self.move_gripper_button.clicked.connect(
            lambda: self._request_gripper(self.gripper_spin.value())
        )
        self.open_gripper_button = QPushButton("Open")
        self.open_gripper_button.clicked.connect(lambda: self._request_gripper(1.0))
        grid.addWidget(self.close_gripper_button, 1, 3)
        grid.addWidget(self.move_gripper_button, 1, 4)
        grid.addWidget(self.open_gripper_button, 1, 5)
        return box

    def _new_gripper_speed_combo(self) -> QComboBox:
        combo = QComboBox()
        for label, multiplier in GRIPPER_SPEED_PRESETS:
            combo.addItem(label, multiplier)
        combo.setCurrentIndex(combo.findData(self._gripper_speed_multiplier))
        combo.setToolTip("Speed relative to the original gripper motor setting. Applies to the next move.")
        combo.currentIndexChanged.connect(
            lambda _index, source=combo: self._set_gripper_speed(source)
        )
        return combo

    def _set_gripper_speed(self, source: QComboBox) -> None:
        self._gripper_speed_multiplier = float(source.currentData())
        for combo in (
            getattr(self, "manual_gripper_speed_combo", None),
            getattr(self, "teleop_gripper_speed_combo", None),
            getattr(self, "trajectory_gripper_speed_combo", None),
            getattr(self, "run_gripper_speed_combo", None),
        ):
            if combo is not None and combo is not source:
                combo.blockSignals(True)
                combo.setCurrentIndex(combo.findData(self._gripper_speed_multiplier))
                combo.blockSignals(False)

    def _request_gripper(self, position: float) -> None:
        self.gripper_requested.emit({
            "position": position,
            "gripper_speed_multiplier": self._gripper_speed_multiplier,
        })

    def _set_gripper_spin(self, value: float) -> None:
        self.gripper_spin.blockSignals(True)
        self.gripper_spin.setValue(value)
        self.gripper_spin.blockSignals(False)

    def _set_gripper_slider(self, value: float) -> None:
        self.gripper_slider.blockSignals(True)
        self.gripper_slider.setValue(round(value * 1000.0))
        self.gripper_slider.blockSignals(False)

    @staticmethod
    def _set_combo_port(combo: QComboBox, port: str) -> None:
        if combo.findText(port) < 0:
            combo.addItem(port)
        combo.setCurrentText(port)

    def _find_arms(self) -> None:
        if self._connected or self._leader_connected:
            QMessageBox.information(
                self,
                "Disconnect before scanning",
                "Disconnect the follower and leader before running read-only arm discovery.",
            )
            return
        if self.simulation_check.isChecked():
            QMessageBox.information(
                self,
                "Hardware discovery only",
                "Find Arms probes physical serial devices and is unavailable in simulation.",
            )
            return
        self.arm_discovery_status.setText("Arm discovery: scanning serial ports…")
        self.discover_arms_requested.emit()

    def _on_arm_discovery_completed(self, result: object) -> None:
        results = [dict(item) for item in list(result)]  # type: ignore[arg-type]
        found = [item for item in results if item.get("status") == "ok"]
        self._discovered_arms = {str(item["port"]): item for item in found}

        for item in results:
            port = str(item.get("port") or "unknown")
            if item.get("status") == "ok":
                voltage = float(item.get("voltage_v") or 0.0)
                role = str(item.get("role") or "unknown")
                count = int(item.get("motor_count") or 0)
                total = int(item.get("motor_total") or 6)
                self._log(
                    f"Found SO-101 on {port}: {voltage:.1f} V, {count}/{total} servos, role={role}."
                )
            else:
                self._log(f"Skipped {port}: {item.get('error') or 'not an SO-101'}")

        followers = [item for item in found if item.get("role") == "follower"]
        leaders = [item for item in found if item.get("role") == "leader"]
        if len(followers) == 1:
            self._set_combo_port(self.port_combo, str(followers[0]["port"]))
        else:
            self.port_combo.setCurrentIndex(-1)
        if len(leaders) == 1:
            self._set_combo_port(self.leader_port_combo, str(leaders[0]["port"]))
        else:
            self.leader_port_combo.setCurrentIndex(-1)

        if not found:
            self.arm_discovery_status.setText(
                "Arm discovery: no SO-101 arms found. Check USB connections and arm power, then rescan."
            )
            return

        summaries: list[str] = []
        for item in found:
            role = str(item.get("role") or "unknown")
            role_label = role.title() if role != "unknown" else "SO-101"
            summaries.append(
                f"{role_label} {item['port']} {float(item['voltage_v']):.1f} V "
                f"({int(item.get('motor_count') or 0)}/{int(item.get('motor_total') or 6)} servos)"
            )
        suffix = ""
        if len(followers) > 1 or len(leaders) > 1:
            suffix = " Multiple arms share a voltage role; choose the intended port manually."
        elif not followers or not leaders:
            suffix = " Manual port selection remains available for unclassified or missing roles."
        self.arm_discovery_status.setText("Arm discovery: " + "; ".join(summaries) + "." + suffix)

    def _refresh_device_hints(self) -> None:
        for role, combo_name in (("follower", "port_combo"), ("leader", "leader_port_combo")):
            label = getattr(self, f"{role}_device_info", None)
            combo = getattr(self, combo_name, None)
            if label is None or combo is None:
                continue
            item = self._discovered_arms.get(combo.currentText().strip())
            if item:
                label.setText(
                    f"Detected {float(item['voltage_v']):.1f} V · "
                    f"{item.get('motor_count', 0)}/6 servos · "
                    f"{item.get('role', 'unknown')} voltage"
                )
            else:
                label.setText("Voltage: not measured · use Find Arms to identify this device")

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText().strip()
        try:
            ports = FeetechBackend.candidate_ports()
        except Exception as exc:
            self._log(f"Port discovery failed: {exc}")
            ports = []
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current:
            if self.port_combo.findText(current) < 0:
                self.port_combo.addItem(current)
            self.port_combo.setCurrentText(current)
        elif len(ports) == 1:
            self.port_combo.setCurrentText(ports[0])
        self.port_combo.blockSignals(False)

    def _toggle_connection(self) -> None:
        if self._connected:
            self.disconnect_requested.emit()
            return
        simulation = self.simulation_check.isChecked()
        port = self.port_combo.currentText().strip()
        if not simulation and not port:
            QMessageBox.warning(self, "Serial port required", "Choose a serial port first.")
            return
        if not simulation and not self.leader_simulation_check.isChecked() and port == self.leader_port_combo.currentText().strip():
            self._on_error("Choose different devices for Follower and Leader before connecting.")
            return
        self._follower_setup_session = not simulation and self.allow_uncalibrated_check.isChecked()
        self._follower_connecting = True
        self._update_enabled_state()
        self.connect_requested.emit(
            {
                "simulation": simulation,
                "port": port,
                "robot_id": self.robot_id_edit.text().strip() or "so101",
                "allow_uncalibrated": self.allow_uncalibrated_check.isChecked(),
            }
        )

    def _on_robot_id_changed(self) -> None:
        robot_id = self.robot_id_edit.text().strip() or "so101"
        self.robot_id_edit.setToolTip(
            f"Robot/calibration ID. Calibration file: {default_calibration_path(robot_id)}"
        )
        self._pose_library_cache = None
        self._trajectory_library_cache = None
        self._sequence_library_cache = None
        self._primitive_library_cache = None
        self._active_trajectory = None
        self._sequence_steps = []
        self._refresh_named_pose_status()
        self._refresh_point_list()
        self._refresh_trajectory_list()
        self._refresh_primitive_list()
        self._refresh_sequence_list()
        self._refresh_sequence_step_list()
        if hasattr(self, "save_home_button"):
            self._update_enabled_state()

    def _refresh_leader_ports(self) -> None:
        if not hasattr(self, "leader_port_combo"):
            return
        current = self.leader_port_combo.currentText().strip()
        try:
            ports = FeetechBackend.candidate_ports()
        except Exception as exc:
            self._log(f"Leader port discovery failed: {exc}")
            ports = []
        self.leader_port_combo.blockSignals(True)
        self.leader_port_combo.clear()
        self.leader_port_combo.addItems(ports)
        if current:
            if self.leader_port_combo.findText(current) < 0:
                self.leader_port_combo.addItem(current)
            self.leader_port_combo.setCurrentText(current)
        self.leader_port_combo.blockSignals(False)

    def _toggle_leader_connection(self) -> None:
        if self._leader_connected:
            self.leader_disconnect_requested.emit()
            return
        simulation = self.leader_simulation_check.isChecked()
        port = self.leader_port_combo.currentText().strip()
        if not simulation and not port:
            QMessageBox.warning(self, "Leader serial port required", "Choose a leader port first.")
            return
        if not simulation and not self.simulation_check.isChecked() and port == self.port_combo.currentText().strip():
            self._on_error("Choose different devices for Follower and Leader before connecting.")
            return
        self._leader_connecting = True
        self._update_enabled_state()
        self.leader_connect_requested.emit(
            {
                "simulation": simulation,
                "port": port,
                "robot_id": self.leader_robot_id_edit.text().strip() or "so101-leader",
                "allow_uncalibrated": self.leader_allow_uncalibrated_check.isChecked(),
            }
        )

    def _on_calibration_target_changed(self) -> None:
        target = str(self.calibration_target_combo.currentData())
        if self._active_calibration_target is None:
            self.calibration_sweep_panel.reset(
                PROVISIONAL_MINIMUM_TRAVEL_TICKS,
                self._current_calibration_display_targets(),
            )
            self._update_calibration_guide()
            self.calibration_time_bar.setValue(0)
            self.calibration_time_bar.setFormat("Recording has not started")
            self.calibration_status.setStyleSheet("font-weight: 700; padding: 8px;")
            self.calibration_status.setText(
                f"{target.title()} selected for mechanical-stop calibration."
            )
        self._update_enabled_state()

    def _current_calibration_display_targets(self) -> dict[str, int]:
        target = str(self.calibration_target_combo.currentData())
        robot_id = (
            self.leader_robot_id_edit.text().strip() or "so101-leader"
            if target == "leader"
            else self.robot_id_edit.text().strip() or "so101"
        )
        return display_travel_targets(robot_id)

    def _update_calibration_guide(self) -> None:
        gripper_ticks = self._current_calibration_display_targets()["so101_gripper"]
        self.calibration_target_note.setText(
            f"Target: 2/2 on every dial · gripper display ≈ {gripper_ticks} ticks."
        )
        self.calibration_target_note.setToolTip(
            "Inner ring = first sweep; outer ring = return sweep. Both must reach 2/2. "
            "The gripper display uses saved travel when available. Voltage or communication "
            "faults stop calibration regardless of dial progress."
        )

    def _connect_for_calibration(self) -> None:
        target = str(self.calibration_target_combo.currentData())
        if target == "leader":
            if self._leader_connected:
                self.calibration_status.setText("Leader is already connected for calibration.")
                return
            if self.leader_simulation_check.isChecked():
                self.calibration_status.setText("Turn off Leader Simulation in Teach first.")
                return
            if not self.leader_port_combo.currentText().strip():
                self.calibration_status.setText("Click Find Arms to select the leader port first.")
                return
            self.leader_allow_uncalibrated_check.setChecked(True)
            self._toggle_leader_connection()
        else:
            if self._connected:
                self.calibration_status.setText("Follower is already connected for calibration.")
                return
            if self.simulation_check.isChecked():
                self.calibration_status.setText("Turn off Simulation in Robot session first.")
                return
            if not self.port_combo.currentText().strip():
                self.calibration_status.setText("Click Find Arms to select the follower port first.")
                return
            self.allow_uncalibrated_check.setChecked(True)
            self._toggle_connection()
        self.calibration_status.setText(
            f"Connecting {target} for calibration with torque off…"
        )

    def _start_calibration(self) -> None:
        target = str(self.calibration_target_combo.currentData())
        if target == "leader":
            connected = self._leader_connected
            simulation = self.leader_simulation_check.isChecked()
        else:
            connected = self._connected
            simulation = self.simulation_check.isChecked()

        if not connected:
            QMessageBox.warning(
                self,
                f"{target.title()} not connected",
                f"Connect the {target} first.",
            )
            return
        if simulation:
            QMessageBox.information(
                self,
                "Hardware calibration only",
                "Simulation has no raw encoders or mechanical stops to calibrate.",
            )
            return
        answer = QMessageBox.question(
            self,
            f"Calibrate {target}?",
            f"{target.title()} torque will be off. Move each joint and the gripper "
            "gently between both stops twice. Watch for 2/2 on every dial, then "
            "wait for the saved result. A failed or cancelled sweep keeps the "
            "previous calibration. Start recording?",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        self._active_calibration_target = target
        self._calibration_cancelling = False
        (self._leader_worker if target == "leader" else self._worker).prepare_calibration()
        self.calibration_target_combo.setEnabled(False)
        self.calibration_sweep_panel.reset(
            PROVISIONAL_MINIMUM_TRAVEL_TICKS,
            self._current_calibration_display_targets(),
        )
        self.calibration_time_bar.setValue(0)
        self.calibration_time_bar.setFormat("Preparing motors — sweep has not started")
        self.calibration_status.setStyleSheet(
            "font-weight: 700; padding: 8px; background: #fff7ed; color: #7c2d12;"
        )
        self.calibration_status.setText(
            f"Preparing {target} motors. Wait for RECORDING, then begin the two sweeps. "
            "Recording ends automatically when all six reach DONE."
        )
        duration = self.calibration_duration.value()
        self._update_enabled_state()
        if target == "leader":
            self.leader_calibration_requested.emit(duration)
        else:
            self.calibration_requested.emit(duration)

    def _cancel_calibration(self) -> None:
        target = self._active_calibration_target
        if target is None:
            return
        (self._leader_worker if target == "leader" else self._worker).request_calibration_cancel()
        self._calibration_cancelling = True
        self.cancel_calibration_button.setEnabled(False)
        self.calibration_time_bar.setFormat("CANCELLING — restoring previous calibration")
        self.calibration_status.setText(
            f"CANCELLING {target.title()} — wait for previous motor settings to be restored."
        )
        self._log(f"{target.title()} calibration cancellation requested.")

    def _on_calibration_cancelled(self, target: str) -> None:
        if self._active_calibration_target != target:
            return
        self._active_calibration_target = None
        self._calibration_cancelling = False
        self.calibration_time_bar.setFormat("CALIBRATION CANCELLED")
        self.calibration_status.setText(
            f"{target.title()} calibration cancelled. Previous motor calibration restored; torque remains off."
        )
        self._update_enabled_state()

    @Slot(object)
    def _on_follower_calibration_progress(self, result: object) -> None:
        self._on_calibration_progress("follower", result)

    @Slot(object)
    def _on_leader_calibration_progress(self, result: object) -> None:
        self._on_calibration_progress("leader", result)

    @Slot(object)
    def _on_follower_calibration_completed(self, result: object) -> None:
        self._on_calibration_completed("follower", result)

    @Slot(object)
    def _on_leader_calibration_completed(self, result: object) -> None:
        self._on_calibration_completed("leader", result)

    @Slot()
    def _on_follower_calibration_cancelled(self) -> None:
        self._on_calibration_cancelled("follower")

    @Slot()
    def _on_leader_calibration_cancelled(self) -> None:
        self._on_calibration_cancelled("leader")

    @Slot(str)
    def _on_leader_log_message(self, message: str) -> None:
        self._log(f"Leader: {message}")

    @Slot(str)
    def _on_leader_error_message(self, message: str) -> None:
        self._on_error(f"Leader: {message}")

    @Slot(bool)
    def _on_follower_recording_changed(self, active: bool) -> None:
        self._on_recording_changed("follower", active)

    @Slot(bool)
    def _on_leader_recording_changed(self, active: bool) -> None:
        self._on_recording_changed("leader", active)

    def _on_calibration_progress(self, target: str, result: object) -> None:
        if self._active_calibration_target != target or self._calibration_cancelling:
            return
        payload = dict(result)  # type: ignore[arg-type]
        if payload.get("phase") == "preparing":
            self.calibration_time_bar.setFormat("Preparing motors — sweep has not started")
            return
        values = dict(payload.get("progress", payload))
        duration = float(payload.get("duration_s", self.calibration_duration.value()))
        remaining = max(0.0, float(payload.get("remaining_s", duration)))
        fraction = 1.0 - remaining / duration if duration > 0 else 0.0
        self.calibration_time_bar.setValue(round(1000 * max(0.0, min(1.0, fraction))))
        self.calibration_time_bar.setFormat(
            "Verifying and saving…" if remaining <= 0.0
            else f"Recording — {ceil(remaining)} s remaining"
        )
        self.calibration_sweep_panel.set_progress(values)
        passed = sum(
            1
            for item in values.values()
            if isinstance(item, dict) and bool(item.get("passed", False))
        )
        total = len(PROVISIONAL_MINIMUM_TRAVEL_TICKS)
        if passed == total:
            self.calibration_time_bar.setValue(1000)
            self.calibration_time_bar.setFormat("ALL SIX DONE — verifying and saving…")
        if remaining <= 0.0:
            self.calibration_status.setText(
                f"Recording ended · {passed}/{total} complete. "
                "Checking motor ranges and saving; wait for the result."
            )
        elif passed == total:
            self.calibration_status.setText(
                f"All {total} actuators reached 2/2. Stop moving the arm. "
                "Verifying and saving calibration now."
            )
        else:
            incomplete = [
                (name, item) for name, item in values.items()
                if isinstance(item, dict) and not bool(item.get("passed", False))
            ]
            if not incomplete:
                self.calibration_status.setText(
                    f"RECORDING {target.title()} · {ceil(remaining)} s left. "
                    "Waiting for motor sweep readings."
                )
                return
            next_name, next_item = min(
                incomplete,
                key=lambda pair: (
                    int(pair[1].get("traversals_completed", 0)),
                    float(pair[1].get("fraction", 0.0)),
                ),
            )
            next_label = next_name.replace("_", " ").title()
            span_shortfall = max(
                0,
                int(next_item.get("required_ticks", 0))
                - int(next_item.get("travel_ticks", 0)),
            )
            instruction = (
                f"{next_label} needs {span_shortfall} more ticks of range."
                if span_shortfall else
                f"Move {next_label} to the opposite stop for sweep "
                f"{int(next_item.get('sweep_number', 1))}/2."
            )
            self.calibration_status.setText(
                f"RECORDING {target.title()} · {passed}/{total} complete · "
                f"{ceil(remaining)} s left. {instruction}"
            )

    def _on_calibration_completed(self, target: str, result: object) -> None:
        values = dict(result)  # type: ignore[arg-type]
        self._active_calibration_target = None
        self._calibration_cancelling = False
        self.calibration_target_combo.setEnabled(True)
        calibration_id = str(values.get("calibration_id") or "unknown")
        short_id = (
            calibration_id.split(":", 1)[-1][:12]
            if calibration_id != "unknown"
            else "unknown"
        )
        self.calibration_time_bar.setValue(1000)
        self.calibration_time_bar.setFormat("CALIBRATION SAVED")
        self.calibration_status.setStyleSheet(
            "font-weight: 700; padding: 8px; background: #dcfce7; color: #14532d;"
        )
        self.calibration_status.setText(
            f"{target.title().upper()} CALIBRATION SAVED ({values.get('source', 'unknown')}); "
            f"ID {short_id}; saved to {values.get('path', 'unknown path')}. "
            "Disconnect, turn off 'allow uncalibrated connection', then reconnect normally."
        )
        limits = values.get("joint_limits_deg")
        if target == "follower" and limits:
            self._apply_joint_limits(dict(limits))
        self._log(
            f"{target.title()} mechanical-stop midpoint calibration completed "
            f"for {values.get('robot_id', 'unknown robot')}."
        )
        self._update_enabled_state()
        # Let the six completion pops finish before a dialog covers the gauges.
        QTimer.singleShot(
            500,
            self,
            lambda: QMessageBox.information(
                self,
                f"{target.title()} calibration saved",
                f"Calibration was verified and saved to {values.get('path', 'unknown path')}.\n\n"
                "Disconnect this arm, turn off 'allow uncalibrated connection', "
                "and reconnect normally.",
            ),
        )

    def _get_sequence_library(self) -> SequenceLibrary:
        robot_id = self.robot_id_edit.text().strip() or "so101"
        if self._sequence_library_cache is None or self._sequence_library_cache[0] != robot_id:
            self._sequence_library_cache = (robot_id, SequenceLibrary(robot_id))
        return self._sequence_library_cache[1]

    def _get_primitive_library(self) -> MotionPrimitiveLibrary:
        robot_id = self.robot_id_edit.text().strip() or "so101"
        if self._primitive_library_cache is None or self._primitive_library_cache[0] != robot_id:
            self._primitive_library_cache = (robot_id, MotionPrimitiveLibrary(robot_id))
        return self._primitive_library_cache[1]

    def _get_trajectory_library(self) -> TrajectoryLibrary:
        robot_id = self.robot_id_edit.text().strip() or "so101"
        if (
            self._trajectory_library_cache is None
            or self._trajectory_library_cache[0] != robot_id
        ):
            self._trajectory_library_cache = (robot_id, TrajectoryLibrary(robot_id))
        return self._trajectory_library_cache[1]

    def _get_pose_library(self) -> PoseLibrary:
        robot_id = self.robot_id_edit.text().strip() or "so101"
        if self._pose_library_cache is None or self._pose_library_cache[0] != robot_id:
            self._pose_library_cache = (robot_id, PoseLibrary(robot_id))
        return self._pose_library_cache[1]

    def _follower_calibration_binding(self) -> tuple[str, str] | None:
        robot_id = self.robot_id_edit.text().strip() or "so101"
        if self._latest_state is not None:
            calibration_id = self._latest_state.get("calibration_id")
            state_robot_id = str(self._latest_state.get("robot_id") or robot_id)
            if calibration_id:
                return state_robot_id, str(calibration_id)
        path = default_calibration_path(robot_id)
        if path.is_file():
            try:
                calibration = SO101Calibration.load(path)
            except Exception as exc:
                self._log(f"Could not read follower calibration binding from {path}: {exc}")
                return None
            return robot_id, calibration.calibration_id
        return None

    def _bind_pose_to_follower(self, pose: SavedPose) -> SavedPose:
        binding = self._follower_calibration_binding()
        if binding is None:
            return pose
        robot_id, calibration_id = binding
        return replace(
            pose,
            target_robot_id=robot_id,
            target_calibration_id=calibration_id,
        )

    def _bind_trajectory_to_follower(self, trajectory: Trajectory) -> Trajectory:
        binding = self._follower_calibration_binding()
        if binding is None:
            return trajectory
        robot_id, calibration_id = binding
        return replace(
            trajectory,
            metadata=bind_target_calibration(
                trajectory.metadata,
                robot_id=robot_id,
                calibration_id=calibration_id,
            ),
        )

    @staticmethod
    def _saved_pose_from_state(state: dict[str, Any], *, source: str) -> SavedPose:
        joints = {
            name: radians(float(state["joints_deg"][name]))
            for name in ARM_JOINTS
        }
        pose_values = [float(value) for value in state["pose_mm_deg"]]
        tcp = (
            pose_values[0] / 1000.0,
            pose_values[1] / 1000.0,
            pose_values[2] / 1000.0,
            radians(pose_values[3]),
            radians(pose_values[4]),
            radians(pose_values[5]),
        )
        return SavedPose(
            joints=joints,
            gripper=float(state["gripper"]),
            tcp_xyz_rpy=tcp,
            source=source,
            source_robot_id=(
                None if state.get("robot_id") is None else str(state["robot_id"])
            ),
            source_calibration_id=(
                None
                if state.get("calibration_id") is None
                else str(state["calibration_id"])
            ),
        )

    def _save_named_pose(self, name: str) -> None:
        if not self._connected:
            self._on_error("Cannot save pose: follower is not connected.")
            return
        self.capture_follower_pose_requested.emit(
            {
                "kind": "named",
                "name": name,
            }
        )
        self._log(f"Capturing fresh measured follower pose for {name}…")

    def _go_named_pose(self, name: str) -> None:
        try:
            pose = self._get_pose_library().require(name)
        except Exception as exc:
            self._on_error(f"Load {name}: {exc}")
            return
        for joint, value in pose.joints.items():
            self.joint_spins[joint].setValue(degrees(value))
        self.gripper_spin.setValue(pose.gripper)
        self.move_saved_pose_requested.emit(
            {
                "pose": pose,
                "mode": "joint",
                "move_gripper": True,
                "speed_deg_s": self.joint_speed.value(),
                "acceleration_deg_s2": self.joint_acceleration.value(),
                "speed_mm_s": self.linear_speed.value(),
                "acceleration_mm_s2": self.linear_acceleration.value(),
                "orientation_mode": self.orientation_combo.currentData(),
                "gripper_speed_multiplier": self._gripper_speed_multiplier,
            }
        )
        self._log(
            f"Moving to saved {name} pose; gripper command follows after arm settles."
        )

    def _refresh_named_pose_status(self) -> None:
        if not hasattr(self, "home_status"):
            return
        try:
            library = self._get_pose_library()
            home = library.get(HOME_POSE_NAME)
            rest = library.get(REST_POSE_NAME)
        except Exception as exc:
            self.home_status.setText(f"Home: error ({exc})")
            self.rest_status.setText(f"Rest: error ({exc})")
            return
        self.home_status.setText("Home: saved" if home else "Home: not saved")
        self.rest_status.setText("Rest: saved" if rest else "Rest: not saved")

    def _current_teaching_state(self) -> tuple[str, dict[str, Any] | None]:
        source = str(self.teaching_source_combo.currentData())
        state = self._latest_state if source == "follower" else self._latest_leader_state
        return source, state

    def _refresh_point_list(self) -> None:
        if not hasattr(self, "point_combo"):
            return
        current = self.point_combo.currentText()
        try:
            names = [
                name
                for name in self._get_pose_library().names()
                if name not in {HOME_POSE_NAME, REST_POSE_NAME}
            ]
        except Exception as exc:
            self._on_error(f"Refresh taught points: {exc}")
            names = []
        self.point_combo.blockSignals(True)
        self.point_combo.clear()
        self.point_combo.addItems(names)
        if current and current in names:
            self.point_combo.setCurrentText(current)
        self.point_combo.blockSignals(False)
        self._refresh_sequence_resources()
        self._preview_selected_taught_point()
        self._update_enabled_state()

    def _show_saved_pose_preview(
        self,
        panel: RobotStatusPanel,
        name: str,
        *,
        label: str | None = None,
    ) -> None:
        key = str(name).strip()
        if not key:
            panel.clear_secondary()
            return
        try:
            pose = self._get_pose_library().require(key)
        except Exception:
            panel.clear_secondary()
            return
        panel.show_saved_pose(
            joints_rad=pose.joints,
            gripper=pose.gripper,
            label=label or key,
        )

    def _preview_selected_taught_point(self, *, force: bool = False) -> None:
        if not hasattr(self, "teach_arm_panel"):
            return
        if not force and self.tabs.currentWidget() is not self.record_page:
            return
        self._show_saved_pose_preview(
            self.robot_sidebar,
            self.point_combo.currentText(),
            label="saved position",
        )

    def _preview_program_position(self, *, force: bool = False) -> None:
        if not hasattr(self, "program_arm_panel"):
            return
        if not force and self.tabs.currentWidget() is not self.run_page:
            return
        self._show_saved_pose_preview(
            self.robot_sidebar,
            self.run_point_combo.currentText(),
            label="selected position",
        )

    def _preview_selected_program_step(self, *, force: bool = False) -> None:
        if not hasattr(self, "program_arm_panel"):
            return
        if not force and self.tabs.currentWidget() is not self.run_page:
            return
        row = self.sequence_step_list.currentRow()
        if not 0 <= row < len(self._sequence_steps):
            self._preview_program_position(force=force)
            return
        step = self._sequence_steps[row]
        if step.kind == "point":
            name = str(step.params.get("name") or "")
            overrides = dict(step.params.get("joint_overrides_deg") or {})
            if overrides:
                try:
                    pose = self._get_pose_library().require(name)
                    joints = dict(pose.joints)
                    for joint, value_deg in overrides.items():
                        if joint in joints:
                            joints[joint] = radians(float(value_deg))
                    self.robot_sidebar.show_saved_pose(
                        joints_rad=joints,
                        gripper=pose.gripper,
                        label=f"step {row + 1}",
                    )
                except Exception:
                    self.robot_sidebar.clear_secondary()
            else:
                self._show_saved_pose_preview(
                    self.robot_sidebar,
                    name,
                    label=f"step {row + 1}",
                )
            return
        if step.kind in {"home", "rest"}:
            self._show_saved_pose_preview(
                self.robot_sidebar,
                step.kind,
                label=f"step {row + 1}",
            )
            return
        self.robot_sidebar.clear_secondary()

    def _add_selected_taught_point_to_program(self) -> None:
        name = self.point_combo.currentText().strip()
        if not name:
            return
        if self.run_point_combo.findText(name) < 0:
            self._refresh_sequence_resources()
        self.run_point_combo.setCurrentText(name)
        mode_index = self.run_point_mode_combo.findData(self.point_mode_combo.currentData())
        if mode_index >= 0:
            self.run_point_mode_combo.setCurrentIndex(mode_index)
        self._add_point_sequence_step()
        self._log(f"Added saved position {name!r} to the current program.")

    def _save_taught_point(self) -> None:
        name = self.point_name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Point name required", "Enter a name for the point.")
            return
        if name in {HOME_POSE_NAME, REST_POSE_NAME}:
            QMessageBox.warning(
                self,
                "Reserved point name",
                "Use the dedicated Home/Rest controls for those standard poses.",
            )
            return
        source, state = self._current_teaching_state()
        if source == "follower":
            if not self._connected:
                self._on_error("Cannot save point: follower is not connected.")
                return
            self.capture_follower_pose_requested.emit(
                {
                    "kind": "point",
                    "name": name,
                }
            )
            self._log(f"Capturing fresh measured follower pose for taught point {name!r}…")
            return

        if state is None:
            self._on_error(f"Cannot save point: {source} state is unavailable.")
            return
        self.capture_leader_pose_requested.emit(
            {
                "kind": "point",
                "name": name,
                "source": "leader",
            }
        )
        self._log(f"Capturing fresh measured leader pose for taught point {name!r}…")

    def _on_measured_pose_captured(self, result: object) -> None:
        values = dict(result)  # type: ignore[arg-type]
        request = dict(values.get("request") or {})
        source = str(request.get("source") or "follower")
        error = values.get("error")
        if error:
            if str(request.get("kind") or "") == "sync":
                self._sync_capture_pending = None
                self._update_enabled_state()
            return

        pose = values.get("pose")
        if not isinstance(pose, SavedPose):
            self._on_error(f"Capture measured {source} pose: worker returned no valid pose.")
            self._sync_capture_pending = None
            self._update_enabled_state()
            return

        kind = str(request.get("kind") or "")
        name = str(request.get("name") or "").strip()
        try:
            if kind == "sync":
                destination = str(request.get("destination") or "")
                if destination not in {"leader", "follower"}:
                    raise ValueError("pose synchronization destination is invalid")
                payload = {
                    "joints_deg": {
                        joint: degrees(float(value)) for joint, value in pose.joints.items()
                    },
                    "gripper": float(pose.gripper),
                    "include_gripper": self._sync_include_gripper,
                    # Cross-arm synchronization deliberately uses the worker's
                    # conservative 8°/s, 25°/s² defaults rather than inheriting a
                    # potentially aggressive Manual joint-speed setting.
                    "gripper_speed_multiplier": self._gripper_speed_multiplier,
                    "source": source,
                }
                if destination == "leader":
                    self.leader_sync_requested.emit(payload)
                else:
                    self.follower_sync_requested.emit(payload)
                self._log(
                    f"Fresh {source} pose captured; moving {destination} to match "
                    f"{'including' if self._sync_include_gripper else 'without'} gripper."
                )
                self._sync_capture_pending = None
                self._update_enabled_state()
                return

            if kind == "named":
                if source != "follower":
                    raise ValueError("standard Home/Rest poses must be captured from the follower")
                if name not in {HOME_POSE_NAME, REST_POSE_NAME}:
                    raise ValueError(f"unknown standard pose {name!r}")
                pose = self._bind_pose_to_follower(pose)
                path = self._get_pose_library().save(name, pose)
                self._log(f"Saved {name} from fresh measured follower pose to {path}.")
                self._refresh_named_pose_status()
                self._update_enabled_state()
                return

            if kind == "point":
                if not name:
                    raise ValueError("point name is empty")
                if source == "leader":
                    pose = self._bind_pose_to_follower(pose)
                path = self._get_pose_library().save(name, pose)
                self._log(
                    f"Saved taught point {name!r} from fresh measured {source} pose to {path}."
                )
                self.point_name_edit.clear()
                self._refresh_point_list()
                self.point_combo.setCurrentText(name)
                return

            raise ValueError(f"unknown measured-pose request {kind!r}")
        except Exception as exc:
            self._sync_capture_pending = None
            self._on_error(f"Use measured {source} pose: {exc}")
            self._update_enabled_state()

    def _delete_taught_point(self) -> None:
        name = self.point_combo.currentText().strip()
        if not name:
            return
        try:
            self._get_pose_library().delete(name)
            self._log(f"Deleted taught point {name!r}.")
            self._refresh_point_list()
        except Exception as exc:
            self._on_error(f"Delete taught point: {exc}")

    def _move_taught_point(self) -> None:
        name = self.point_combo.currentText().strip()
        if not name:
            return
        try:
            pose = self._get_pose_library().require(name)
        except Exception as exc:
            self._on_error(f"Load taught point: {exc}")
            return
        self.move_saved_pose_requested.emit(
            {
                "pose": pose,
                "mode": self.point_mode_combo.currentData(),
                "speed_deg_s": self.joint_speed.value(),
                "acceleration_deg_s2": self.joint_acceleration.value(),
                "speed_mm_s": self.linear_speed.value(),
                "acceleration_mm_s2": self.linear_acceleration.value(),
                "orientation_mode": self.orientation_combo.currentData(),
            }
        )

    def _toggle_recording(self) -> None:
        if self._recording_source is not None:
            if self._recording_source == "leader":
                self.leader_recording_stop_requested.emit()
            else:
                self.recording_stop_requested.emit()
            return

        name = self.recording_name_edit.text().strip()
        if not name:
            QMessageBox.warning(
                self, "Trajectory name required", "Enter a raw trajectory name first."
            )
            return
        try:
            library = self._get_trajectory_library()
            library._validate_name(name)
            if any(entry.kind == "raw" and entry.name == name for entry in library.entries()):
                raise FileExistsError(
                    f"raw trajectory {name!r} already exists; choose a new name"
                )
        except Exception as exc:
            QMessageBox.warning(self, "Invalid trajectory name", str(exc))
            return

        source, state = self._current_teaching_state()
        if state is None:
            self._on_error(f"Cannot record: {source} state is unavailable.")
            return
        options = {
            "frequency_hz": 50.0,
            "record_effort": self.record_effort_check.isChecked(),
            "source": source,
        }
        self._recording_source = source
        self._pending_recording_name = name
        if source == "leader":
            self.leader_recording_start_requested.emit(options)
        else:
            self.recording_start_requested.emit(options)
        self.record_button.setText("Stop recording")
        self.recording_status.setText(
            f"Recording {source} at 50 Hz. Move the selected arm through the motion."
        )
        self._update_enabled_state()

    def _on_recording_changed(self, source: str, active: bool) -> None:
        if active:
            return
        if self._recording_source == source:
            self._recording_source = None
            self.record_button.setText("Start recording selected source")
            self._update_enabled_state()

    def _on_recording_completed(self, trajectory: object) -> None:
        if not isinstance(trajectory, Trajectory):
            self._on_error("Recording returned an invalid trajectory object.")
            return
        name = self._pending_recording_name
        self._pending_recording_name = None
        if not name:
            self._on_error("Recording completed without a pending trajectory name.")
            return
        try:
            library = self._get_trajectory_library()
            trajectory = self._bind_trajectory_to_follower(trajectory)
            entry = library.save(name, trajectory, kind="raw")
            loaded = library.load(entry.name, kind=entry.kind)
            self.recording_status.setText(
                f"Saved raw trajectory {name!r}: {loaded.sample_count} samples, "
                f"{loaded.duration_s:.2f} s."
            )
            self._log(f"Saved immutable raw trajectory to {entry.data_path}.")
            self.recording_name_edit.clear()
            self._refresh_trajectory_list()
            self._set_active_trajectory(loaded)
            self.tabs.setCurrentWidget(self.trajectory_page)
        except Exception as exc:
            self._on_error(f"Save raw trajectory: {exc}")

    def _refresh_trajectory_list(self) -> None:
        if not hasattr(self, "trajectory_combo"):
            return
        current = self.trajectory_combo.currentData()
        try:
            entries = self._get_trajectory_library().entries()
        except Exception as exc:
            self._on_error(f"Refresh trajectory library: {exc}")
            entries = ()
        self.trajectory_combo.blockSignals(True)
        self.trajectory_combo.clear()
        for entry in entries:
            self.trajectory_combo.addItem(
                f"{entry.kind}: {entry.name}", (entry.kind, entry.name)
            )
        if current is not None:
            index = self.trajectory_combo.findData(current)
            if index >= 0:
                self.trajectory_combo.setCurrentIndex(index)
        self.trajectory_combo.blockSignals(False)
        self._refresh_sequence_resources()
        self._update_enabled_state()

    def _refresh_primitive_list(self) -> None:
        if not hasattr(self, "run_primitive_combo"):
            return
        current = self.run_primitive_combo.currentText()
        try:
            names = self._get_primitive_library().names()
        except Exception as exc:
            self._on_error(f"Refresh motion primitives: {exc}")
            names = ()
        self.run_primitive_combo.blockSignals(True)
        self.run_primitive_combo.clear()
        self.run_primitive_combo.addItems(names)
        if current and current in names:
            self.run_primitive_combo.setCurrentText(current)
        self.run_primitive_combo.blockSignals(False)
        self._update_enabled_state()

    def _refresh_sequence_list(self) -> None:
        if not hasattr(self, "sequence_combo"):
            return
        current = self.sequence_combo.currentText()
        try:
            names = self._get_sequence_library().names()
        except Exception as exc:
            self._on_error(f"Refresh sequence library: {exc}")
            names = ()
        self.sequence_combo.blockSignals(True)
        self.sequence_combo.clear()
        self.sequence_combo.addItems(names)
        if current and current in names:
            self.sequence_combo.setCurrentText(current)
        self.sequence_combo.blockSignals(False)
        self._update_enabled_state()

    def _refresh_sequence_resources(self) -> None:
        if not hasattr(self, "run_point_combo"):
            return
        try:
            point_names = [
                name
                for name in self._get_pose_library().names()
                if name not in {HOME_POSE_NAME, REST_POSE_NAME}
            ]
        except Exception:
            point_names = []
        current_point = self.run_point_combo.currentText()
        self.run_point_combo.clear()
        self.run_point_combo.addItems(point_names)
        if current_point in point_names:
            self.run_point_combo.setCurrentText(current_point)

        try:
            entries = self._get_trajectory_library().entries()
        except Exception:
            entries = ()
        current_trajectory = self.run_trajectory_combo.currentData()
        self.run_trajectory_combo.clear()
        for entry in entries:
            self.run_trajectory_combo.addItem(
                f"{entry.kind}: {entry.name}", (entry.kind, entry.name)
            )
        if current_trajectory is not None:
            index = self.run_trajectory_combo.findData(current_trajectory)
            if index >= 0:
                self.run_trajectory_combo.setCurrentIndex(index)

        self._refresh_primitive_list()

    def _load_selected_trajectory(self) -> None:
        data = self.trajectory_combo.currentData()
        if not data:
            return
        kind, name = data
        try:
            trajectory = self._get_trajectory_library().load(name, kind=kind)
            self._set_active_trajectory(trajectory)
        except Exception as exc:
            self._on_error(f"Load trajectory: {exc}")

    def _set_active_trajectory(self, trajectory: Trajectory) -> None:
        self._active_trajectory = trajectory
        duration = trajectory.duration_s
        self.trajectory_timeline.set_trajectory(trajectory)
        self.selection_start.blockSignals(True)
        self.selection_end.blockSignals(True)
        self.selection_start.setRange(0.0, duration)
        self.selection_end.setRange(0.0, duration)
        self.selection_start.setValue(0.0)
        self.selection_end.setValue(duration)
        self.selection_start.blockSignals(False)
        self.selection_end.blockSignals(False)
        self.trajectory_scrub.setValue(0)
        self._update_trajectory_arm_preview(0.0)
        diagnostics = " + effort" if trajectory.effort_current_raw is not None else ""
        self.trajectory_stats.setText(
            f"{trajectory.metadata.get('kind', 'unsaved')} / "
            f"{trajectory.metadata.get('name', 'recording')} — "
            f"{trajectory.sample_count} samples, {duration:.3f} s, "
            f"median {trajectory.sample_rate_hz:.1f} Hz{diagnostics}"
        )
        self._selection_changed()
        self._update_enabled_state()

    def _update_trajectory_arm_preview(
        self,
        cursor_s: float,
        *,
        force: bool = False,
    ) -> None:
        trajectory = self._active_trajectory
        if trajectory is None or not hasattr(self, "trajectory_arm_panel"):
            return
        if not force and self.tabs.currentWidget() is not self.trajectory_page:
            return
        index = min(
            range(trajectory.sample_count),
            key=lambda item: abs(float(trajectory.timestamps_s[item]) - float(cursor_s)),
        )
        joints = {
            name: float(trajectory.joints_rad[index, joint_index])
            for joint_index, name in enumerate(ARM_JOINTS)
        }
        self.robot_sidebar.show_saved_pose(
            joints_rad=joints,
            gripper=float(trajectory.gripper[index]),
            label=f"recorded {float(trajectory.timestamps_s[index]):.2f} s",
        )

    def _trajectory_scrub_changed(self, value: int) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        cursor = trajectory.duration_s * float(value) / 10000.0
        self.trajectory_timeline.set_cursor(cursor)
        self.trajectory_cursor_label.setText(f"{cursor:.3f} s")
        self._update_trajectory_arm_preview(cursor)

    def _cursor_seconds(self) -> float:
        trajectory = self._active_trajectory
        if trajectory is None:
            return 0.0
        return trajectory.duration_s * self.trajectory_scrub.value() / 10000.0

    def _set_selection_from_cursor(self, which: str) -> None:
        cursor = self._cursor_seconds()
        if which == "start":
            self.selection_start.setValue(cursor)
        else:
            self.selection_end.setValue(cursor)
        self._selection_changed()

    def _selection_changed(self) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        start = self.selection_start.value()
        end = self.selection_end.value()
        if end <= start:
            epsilon = min(0.02, max(0.001, trajectory.duration_s / 100.0))
            if start + epsilon <= trajectory.duration_s:
                self.selection_end.blockSignals(True)
                self.selection_end.setValue(start + epsilon)
                self.selection_end.blockSignals(False)
            else:
                self.selection_start.blockSignals(True)
                self.selection_start.setValue(max(0.0, end - epsilon))
                self.selection_start.blockSignals(False)
            start = self.selection_start.value()
            end = self.selection_end.value()
        self.trajectory_timeline.set_selection(start, end)

    def _selection_clip(self) -> Trajectory:
        trajectory = self._active_trajectory
        if trajectory is None:
            raise RuntimeError("no trajectory is loaded")
        return trajectory.crop(self.selection_start.value(), self.selection_end.value())

    def _replay_trajectory(self, *, selection: bool) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        try:
            clip = self._selection_clip() if selection else trajectory
            self.trajectory_play_requested.emit(
                {
                    "trajectory": clip,
                    "speed_scale": self.trajectory_speed_scale.value(),
                    "move_to_start": True,
                    "gripper_speed_multiplier": self._gripper_speed_multiplier,
                }
            )
        except Exception as exc:
            self._on_error(f"Replay trajectory: {exc}")

    def _save_edited_trajectory(self) -> None:
        name = self.edited_trajectory_name.text().strip()
        if not name:
            QMessageBox.warning(
                self, "Edited trajectory name required", "Enter a Save As name."
            )
            return
        try:
            clip = self._bind_trajectory_to_follower(
                self._selection_clip().retime(self.trajectory_speed_scale.value())
            )
            entry = self._get_trajectory_library().save(name, clip, kind="edited")
            loaded = self._get_trajectory_library().load(entry.name, kind=entry.kind)
            self._log(
                f"Saved derived trajectory {name!r}; raw source was not modified."
            )
            self.edited_trajectory_name.clear()
            self._refresh_trajectory_list()
            self._set_active_trajectory(loaded)
        except Exception as exc:
            self._on_error(f"Save edited trajectory: {exc}")

    def _update_keyframe_units(self) -> None:
        if not hasattr(self, "keyframe_value_spin"):
            return
        channel = self.keyframe_channel_combo.currentData()
        if channel == "gripper":
            self.keyframe_value_spin.setRange(0.0, 1.0)
            self.keyframe_value_spin.setDecimals(3)
            self.keyframe_value_spin.setSingleStep(0.05)
            self.keyframe_value_spin.setSuffix("")
        else:
            self.keyframe_value_spin.setRange(-180.0, 180.0)
            self.keyframe_value_spin.setDecimals(2)
            self.keyframe_value_spin.setSingleStep(1.0)
            self.keyframe_value_spin.setSuffix("°")

    def _apply_smoothing(self) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        try:
            window = self.smooth_window_spin.value()
            if window % 2 == 0:
                window += 1
                self.smooth_window_spin.setValue(window)
            self._set_active_trajectory(trajectory.smooth(window))
            self._log(f"Applied in-memory smoothing window {window}; use Save As to persist.")
        except Exception as exc:
            self._on_error(f"Smooth trajectory: {exc}")

    def _delete_trajectory_selection(self) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        try:
            edited = trajectory.delete_region(
                self.selection_start.value(), self.selection_end.value()
            )
            self._set_active_trajectory(edited)
            self._log("Deleted selected region in memory; use Save As to persist.")
        except Exception as exc:
            self._on_error(f"Delete trajectory region: {exc}")

    def _insert_trajectory_hold(self) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        try:
            edited = trajectory.insert_hold(
                self._cursor_seconds(), self.hold_duration_spin.value()
            )
            self._set_active_trajectory(edited)
            self._log("Inserted hold in memory; use Save As to persist.")
        except Exception as exc:
            self._on_error(f"Insert trajectory hold: {exc}")

    def _set_trajectory_keyframe(self) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        try:
            channel = str(self.keyframe_channel_combo.currentData())
            value = self.keyframe_value_spin.value()
            if channel != "gripper":
                value = radians(value)
            edited = trajectory.set_keyframe(self._cursor_seconds(), channel, value)
            self._set_active_trajectory(edited)
            self._log(f"Set {channel} keyframe in memory; use Save As to persist.")
        except Exception as exc:
            self._on_error(f"Set trajectory keyframe: {exc}")

    def _add_trajectory_marker(self) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        label = self.marker_label_edit.text().strip()
        if not label:
            QMessageBox.warning(self, "Marker label required", "Enter a marker label.")
            return
        try:
            edited = trajectory.add_marker(self._cursor_seconds(), label)
            self._set_active_trajectory(edited)
            self.marker_label_edit.clear()
            self._log(f"Added marker {label!r} at the cursor.")
        except Exception as exc:
            self._on_error(f"Add trajectory marker: {exc}")

    def _loop_trajectory_selection(self) -> None:
        try:
            edited = self._selection_clip().repeat(self.loop_count_spin.value())
            self._set_active_trajectory(edited)
            self._log(
                f"Created {self.loop_count_spin.value()}× repeated clip in memory."
            )
        except Exception as exc:
            self._on_error(f"Loop trajectory selection: {exc}")

    def _promote_trajectory_primitive(self) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        name = self.primitive_name_edit.text().strip()
        trajectory_name = str(trajectory.metadata.get("name") or "").strip()
        trajectory_kind = str(trajectory.metadata.get("kind") or "").strip()
        if not name:
            QMessageBox.warning(self, "Primitive name required", "Enter a primitive name.")
            return
        if trajectory_kind not in {"raw", "edited"} or not trajectory_name:
            QMessageBox.warning(
                self,
                "Save trajectory first",
                "Save the current derived trajectory before promoting it to a primitive.",
            )
            return
        try:
            tags = tuple(
                item.strip()
                for item in self.primitive_tags_edit.text().split(",")
                if item.strip()
            )
            primitive = MotionPrimitive(
                name=name,
                trajectory_name=trajectory_name,
                trajectory_kind=trajectory_kind,
                tags=tags,
                loopable=self.primitive_loopable_check.isChecked(),
                interruptible=self.primitive_interruptible_check.isChecked(),
                default_speed_scale=self.trajectory_speed_scale.value(),
                metadata={
                    **provenance_subset(trajectory.metadata),
                    "trajectory_created_at": trajectory.created_at,
                },
            )
            path = self._get_primitive_library().save(primitive)
            self._log(f"Saved motion primitive {name!r} to {path}.")
            self.primitive_name_edit.clear()
            self.primitive_tags_edit.clear()
            self._refresh_primitive_list()
        except Exception as exc:
            self._on_error(f"Promote motion primitive: {exc}")

    @staticmethod
    def _sequence_step_text(step: SequenceStep) -> str:
        params = step.params
        speed = float(params.get("speed_scale", 1.0))
        speed_text = f" · {speed:.2f}×" if abs(speed - 1.0) > 1e-9 else ""
        if step.kind == "point":
            overrides = dict(params.get("joint_overrides_deg") or {})
            if "shoulder_pan" in overrides:
                offset = params.get("radial_pan_offset_deg")
                offset_text = (
                    ""
                    if offset is None
                    else f" · {float(offset):+.1f}° offset"
                )
                return (
                    f"PAN  {params.get('name')} · "
                    f"shoulder {float(overrides['shoulder_pan']):+.1f}°"
                    f"{offset_text}{speed_text}"
                )
            mode = "linear" if params.get("mode", "joint") == "linear" else "joint"
            return f"MOVE  {params.get('name')} · {mode}{speed_text}"
        if step.kind in {"home", "rest"}:
            return f"MOVE  {step.kind.title()}{speed_text}"
        if step.kind == "gripper":
            position = float(params.get("position", 0.0))
            if position <= 0.001:
                return "GRIPPER  CLOSE"
            if position >= 0.999:
                return "GRIPPER  OPEN"
            return f"GRIPPER  {position:.3f}"
        if step.kind == "wait":
            return f"WAIT  {float(params.get('seconds', 0.0)):.2f} s"
        if step.kind == "trajectory":
            return (
                f"RECORDED  {params.get('kind', 'edited')}:{params.get('name')} "
                f"×{int(params.get('loops', 1))}{speed_text}"
            )
        if step.kind == "primitive":
            return (
                f"PRIMITIVE  {params.get('name')} ×{int(params.get('loops', 1))}"
                f"{speed_text}"
            )
        return step.kind.upper()

    def _refresh_sequence_step_list(self) -> None:
        if not hasattr(self, "sequence_step_list"):
            return
        current = self.sequence_step_list.currentRow()
        self.sequence_step_list.clear()
        for index, step in enumerate(self._sequence_steps, start=1):
            self.sequence_step_list.addItem(f"{index:02d}  {self._sequence_step_text(step)}")
        if self._sequence_steps:
            self.sequence_step_list.setCurrentRow(
                min(max(current, 0), len(self._sequence_steps) - 1)
            )
        else:
            if hasattr(self, "program_arm_panel"):
                self.program_arm_panel.clear_secondary()
        self._preview_selected_program_step()
        self._update_enabled_state()

    def _append_sequence_step(self, step: SequenceStep) -> None:
        self._sequence_steps.append(step)
        self._refresh_sequence_step_list()
        self.sequence_step_list.setCurrentRow(len(self._sequence_steps) - 1)

    def _current_program_move_speed(self) -> float:
        return (
            float(self.program_move_speed_spin.value())
            if hasattr(self, "program_move_speed_spin")
            else 1.0
        )

    def _add_point_sequence_step(self) -> None:
        name = self.run_point_combo.currentText().strip()
        if name:
            self._append_sequence_step(
                SequenceStep(
                    "point",
                    {
                        "name": name,
                        "mode": self.run_point_mode_combo.currentData(),
                        "speed_scale": self._current_program_move_speed(),
                    },
                )
            )

    def _shoulder_pan_limits_deg(self) -> tuple[float, float]:
        if self._last_joint_limits and "shoulder_pan" in self._last_joint_limits:
            return self._last_joint_limits["shoulder_pan"]
        lower, upper = JOINT_LIMITS["shoulder_pan"]
        return degrees(lower), degrees(upper)

    @staticmethod
    def _radial_offsets(start_deg: float, end_deg: float, increment_deg: float) -> list[float]:
        start = float(start_deg)
        end = float(end_deg)
        increment = abs(float(increment_deg))
        if increment <= 0:
            raise ValueError("radial pan increment must be positive")
        direction = 1.0 if end >= start else -1.0
        step = increment * direction
        values: list[float] = []
        current = start
        tolerance = increment * 1e-6 + 1e-9
        while (
            current <= end + tolerance
            if direction > 0
            else current >= end - tolerance
        ):
            values.append(round(current, 9))
            if len(values) > 361:
                raise ValueError("radial pan pattern is limited to 361 positions")
            current += step
        if not values or abs(values[-1] - end) > tolerance:
            values.append(end)
        return values

    def _add_radial_pan_pattern(self) -> None:
        name = self.run_point_combo.currentText().strip()
        if not name:
            QMessageBox.warning(
                self,
                "Saved position required",
                "Choose a saved position to use as the radial-pattern base.",
            )
            return
        try:
            pose = self._get_pose_library().require(name)
            base_pan_deg = degrees(float(pose.joints["shoulder_pan"]))
            offsets = self._radial_offsets(
                self.radial_pan_start_spin.value(),
                self.radial_pan_end_spin.value(),
                self.radial_pan_step_spin.value(),
            )
            lower, upper = self._shoulder_pan_limits_deg()
            targets = [base_pan_deg + offset for offset in offsets]
            outside = [
                target for target in targets
                if target < lower - 1e-9 or target > upper + 1e-9
            ]
            if outside:
                raise ValueError(
                    "generated shoulder-pan target "
                    f"{outside[0]:.1f}° is outside the current "
                    f"{lower:.1f}°..{upper:.1f}° limit"
                )

            speed = self._current_program_move_speed()
            for offset, target in zip(offsets, targets, strict=True):
                self._sequence_steps.append(
                    SequenceStep(
                        "point",
                        {
                            "name": name,
                            "mode": "joint",
                            "speed_scale": speed,
                            "joint_overrides_deg": {"shoulder_pan": target},
                            "radial_pan_offset_deg": offset,
                        },
                    )
                )
            self._refresh_sequence_step_list()
            if offsets:
                self.sequence_step_list.setCurrentRow(len(self._sequence_steps) - len(offsets))
            self._log(
                f"Appended {len(offsets)} radial pan moves from {name!r}; "
                f"all joints except shoulder_pan inherit the saved position."
            )
        except Exception as exc:
            self._on_error(f"Add radial pan pattern: {exc}")

    def _add_named_pose_program_step(self, kind: str) -> None:
        if kind not in {"home", "rest"}:
            raise ValueError(f"unknown named program pose {kind!r}")
        self._append_sequence_step(
            SequenceStep(kind, {"speed_scale": self._current_program_move_speed()})
        )

    def _add_gripper_sequence_step(self, position: float | None = None) -> None:
        target = self.sequence_gripper_spin.value() if position is None else float(position)
        self._append_sequence_step(
            SequenceStep("gripper", {"position": target})
        )

    def _add_wait_sequence_step(self) -> None:
        self._append_sequence_step(
            SequenceStep("wait", {"seconds": self.sequence_wait_spin.value()})
        )

    def _add_trajectory_sequence_step(self) -> None:
        data = self.run_trajectory_combo.currentData()
        if data:
            kind, name = data
            self._append_sequence_step(
                SequenceStep(
                    "trajectory",
                    {
                        "kind": kind,
                        "name": name,
                        "loops": 1,
                        "speed_scale": self._current_program_move_speed(),
                    },
                )
            )

    def _add_primitive_sequence_step(self) -> None:
        name = self.run_primitive_combo.currentText().strip()
        if name:
            self._append_sequence_step(
                SequenceStep(
                    "primitive",
                    {
                        "name": name,
                        "loops": 1,
                        "speed_scale": self._current_program_move_speed(),
                    },
                )
            )

    def _move_sequence_step(self, delta: int) -> None:
        row = self.sequence_step_list.currentRow()
        target = row + int(delta)
        if row < 0 or not 0 <= target < len(self._sequence_steps):
            return
        self._sequence_steps[row], self._sequence_steps[target] = (
            self._sequence_steps[target],
            self._sequence_steps[row],
        )
        self._refresh_sequence_step_list()
        self.sequence_step_list.setCurrentRow(target)

    def _delete_sequence_step(self) -> None:
        row = self.sequence_step_list.currentRow()
        if 0 <= row < len(self._sequence_steps):
            del self._sequence_steps[row]
            self._refresh_sequence_step_list()

    def _save_sequence(self) -> None:
        name = self.sequence_name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "Program name required", "Enter a program name.")
            return
        if not self._sequence_steps:
            QMessageBox.warning(self, "No steps", "Add at least one program step.")
            return
        try:
            binding = self._follower_calibration_binding()
            sequence_metadata: dict[str, Any] = {}
            if binding is not None:
                sequence_metadata = {
                    "target_robot_id": binding[0],
                    "target_calibration_id": binding[1],
                }
            sequence = MotionSequence(
                name,
                tuple(self._sequence_steps),
                metadata=sequence_metadata,
            )
            path = self._get_sequence_library().save(sequence)
            self._log(f"Saved program {name!r} to {path}.")
            self._refresh_sequence_list()
            self.sequence_combo.setCurrentText(name)
        except Exception as exc:
            self._on_error(f"Save program: {exc}")

    def _load_sequence(self) -> None:
        name = self.sequence_combo.currentText().strip()
        if not name:
            return
        try:
            sequence = self._get_sequence_library().require(name)
            self._sequence_steps = list(sequence.steps)
            self.sequence_name_edit.setText(sequence.name)
            self._refresh_sequence_step_list()
            self._log(f"Loaded program {name!r}.")
        except Exception as exc:
            self._on_error(f"Load program: {exc}")

    def _delete_sequence(self) -> None:
        name = self.sequence_combo.currentText().strip()
        if not name:
            return
        try:
            self._get_sequence_library().delete(name)
            self._log(f"Deleted program {name!r}.")
            self._refresh_sequence_list()
        except Exception as exc:
            self._on_error(f"Delete program: {exc}")

    def _run_sequence(self, *, step_only: bool) -> None:
        if not self._sequence_steps:
            return
        try:
            name = self.sequence_name_edit.text().strip() or "unsaved_sequence"
            sequence = MotionSequence(name, tuple(self._sequence_steps))
            row = self.sequence_step_list.currentRow()
            start_index = row if step_only and row >= 0 else 0
            stop_index = start_index + 1 if step_only else None
            self.sequence_run_requested.emit(
                {
                    "sequence": sequence,
                    "repeat": 1 if step_only else self.sequence_repeat_spin.value(),
                    "speed_scale": self.sequence_speed_spin.value(),
                    "gripper_speed_multiplier": self._gripper_speed_multiplier,
                    "start_index": start_index,
                    "stop_index": stop_index,
                }
            )
            self.sequence_status.setText(
                f"Running {'step ' + str(start_index + 1) if step_only else sequence.name}…"
            )
        except Exception as exc:
            self._on_error(f"Run program: {exc}")

    def _toggle_sequence_pause(self) -> None:
        if self._sequence_paused:
            self.sequence_resume_requested.emit()
        else:
            self.sequence_pause_requested.emit()

    def _on_sequence_progress(self, payload: object) -> None:
        values = dict(payload)  # type: ignore[arg-type]
        if values.get("type") == "sequence_control":
            status = str(values.get("status", ""))
            self._sequence_paused = status == "paused"
            self.pause_sequence_button.setText(
                "Resume sequence" if self._sequence_paused else "Pause after current step"
            )
            labels = {
                "paused": "Program paused",
                "running": "Program running",
                "completed": "Program complete",
                "failed": "Program stopped with an error",
            }
            self.sequence_status.setText(labels.get(status, f"Program {status}"))
            return
        if values.get("type") == "teleop":
            if self._teleop_active:
                self.teleop_status.setText(
                    f"Live teleop {float(values.get('frequency_hz', 0.0)):.0f} Hz — "
                    f"{int(values.get('samples', 0))} samples; follower cycle "
                    f"{float(values.get('processing_ms', 0.0)):.0f} ms; queued age "
                    f"{float(values.get('sample_age_ms', 0.0)):.0f} ms; "
                    f"{int(values.get('limited_samples', 0))} samples smoothed; "
                    + (
                        "gripper holding at contact. Open the leader gripper to release."
                        if values.get("gripper_contact_latched")
                        else "gripper tracking."
                    )
                )
            return
        if values.get("type") != "sequence":
            return
        index = int(values.get("index", -1))
        status = str(values.get("status", ""))
        if 0 <= index < self.sequence_step_list.count():
            self.sequence_step_list.setCurrentRow(index)
        self.sequence_status.setText(
            f"Step {index + 1}: {values.get('kind', 'unknown')} — {status}"
        )

    def _apply_joint_limits(self, limits: dict[str, object]) -> None:
        normalized: dict[str, tuple[float, float]] = {}
        for name in ARM_JOINTS:
            bounds = limits.get(name)
            if bounds is None:
                continue
            lower, upper = bounds  # type: ignore[misc]
            lower_f, upper_f = float(lower), float(upper)
            normalized[name] = (lower_f, upper_f)
            slider = self.joint_sliders[name]
            spin = self.joint_spins[name]
            slider.setRange(round(lower_f * 10), round(upper_f * 10))
            spin.setRange(lower_f, upper_f)
        if normalized:
            self._last_joint_limits = normalized

    def _set_sync_include_gripper(
        self,
        checked: bool,
        source: QCheckBox | None = None,
    ) -> None:
        self._sync_include_gripper = bool(checked)
        for checkbox in self._coordination_gripper_checks:
            if checkbox is source:
                continue
            checkbox.blockSignals(True)
            checkbox.setChecked(self._sync_include_gripper)
            checkbox.blockSignals(False)
        self._update_coordination_panels()

    def _coordination_relation_text(self) -> str:
        if not self._connected or not self._leader_connected:
            return "Pose relationship: connect both arms"
        if self._latest_state is None or self._latest_leader_state is None:
            return "Pose relationship: waiting for measurements"

        joint_delta = max(
            abs(
                float(self._latest_state["joints_deg"][name])
                - float(self._latest_leader_state["joints_deg"][name])
            )
            for name in ARM_JOINTS
        )
        follower_xyz = tuple(float(v) for v in self._latest_state["pose_mm_deg"][:3])
        leader_xyz = tuple(float(v) for v in self._latest_leader_state["pose_mm_deg"][:3])
        tcp_delta = sum(
            (follower_xyz[index] - leader_xyz[index]) ** 2 for index in range(3)
        ) ** 0.5
        gripper_delta = abs(
            float(self._latest_state["gripper"])
            - float(self._latest_leader_state["gripper"])
        )
        aligned = (
            joint_delta <= 2.0
            and tcp_delta <= 5.0
            and (not self._sync_include_gripper or gripper_delta <= 0.03)
        )
        state = "ALIGNED" if aligned else "DIFFERENT"
        gripper_note = (
            f" · gripper Δ {gripper_delta:.3f}"
            if self._sync_include_gripper
            else " · gripper ignored"
        )
        return (
            f"Pose relationship: {state} · max joint Δ {joint_delta:.1f}° "
            f"· TCP Δ {tcp_delta:.1f} mm{gripper_note}"
        )

    def _update_coordination_panels(self) -> None:
        if self._leader_connected:
            leader_text = (
                "Leader: PARKED · torque on"
                if self._leader_torque_enabled
                else "Leader: FREE · torque off"
            )
        else:
            leader_text = "Leader: disconnected"
        relation = self._coordination_relation_text()
        for label in self._coordination_leader_labels:
            label.setText(leader_text)
        for label in self._coordination_relation_labels:
            label.setText(relation)

        calibration_leader = (
            hasattr(self, "leader_allow_uncalibrated_check")
            and self.leader_allow_uncalibrated_check.isChecked()
        )
        idle_pair = (
            self._connected
            and self._leader_connected
            and self._latest_state is not None
            and self._latest_leader_state is not None
            and not self._busy
            and not self._leader_busy
            and not self._teleop_active
            and not self._teleop_starting
            and self._recording_source is None
            and self._sync_capture_pending is None
        )
        can_park = (
            self._leader_connected
            and not self._leader_busy
            and not self._teleop_active
            and not self._teleop_starting
            and self._recording_source != "leader"
            and not calibration_leader
        )
        for button in self._coordination_park_buttons:
            button.setText(
                "Release leader" if self._leader_torque_enabled else "Park leader here"
            )
            button.setEnabled(can_park)
        for button in self._coordination_move_leader_buttons:
            button.setEnabled(idle_pair and not calibration_leader)
        for button in self._coordination_move_follower_buttons:
            button.setEnabled(idle_pair and not self._follower_setup_session)
        for button in self._coordination_relink_buttons:
            button.setEnabled(
                idle_pair
                and not calibration_leader
                and not self._follower_setup_session
            )
        for checkbox in self._coordination_gripper_checks:
            checkbox.setEnabled(not self._teleop_active and not self._teleop_starting)

    def _toggle_leader_park(self) -> None:
        if not self._leader_connected:
            return
        if self._leader_torque_enabled:
            self.leader_release_requested.emit()
            self._log("Releasing leader torque; leader will be FREE for hand movement.")
        else:
            self.leader_park_requested.emit()
            self._log("Parking leader at its freshly latched measured pose.")

    def _capture_pose_for_sync(self, *, source: str, destination: str) -> None:
        if self._sync_capture_pending is not None:
            return
        self._sync_capture_pending = f"{source}_to_{destination}"
        request = {
            "kind": "sync",
            "source": source,
            "destination": destination,
        }
        if source == "leader":
            self.capture_leader_pose_requested.emit(request)
        else:
            self.capture_follower_pose_requested.emit(request)
        self._log(f"Reading fresh {source} pose before moving {destination}…")
        self._update_enabled_state()

    def _move_leader_to_follower(self) -> None:
        self._capture_pose_for_sync(source="follower", destination="leader")

    def _move_follower_to_leader(self) -> None:
        self._capture_pose_for_sync(source="leader", destination="follower")

    def _relink_here(self) -> None:
        self._begin_teleop(align_follower=False, force_relative=True)

    def _transfer_to_manual_and_park(self) -> None:
        if self._teleop_active or self._teleop_starting:
            self._teleop_starting = False
            self.teleop_stop_requested.emit()
        self.leader_stream_stop_requested.emit()
        if self._leader_connected:
            self.leader_park_requested.emit()
        self.tabs.setCurrentWidget(self.manual_page)
        self._log(
            "Transferred to Manual: follower holds its current pose and leader is parking."
        )

    def _begin_teleop(
        self,
        *,
        align_follower: bool,
        force_relative: bool = False,
    ) -> None:
        self._teleop_error = None
        self._teleop_fault_details = None
        self._teleop_alignment_note = None
        if hasattr(self, "teleop_status"):
            self.teleop_status.setStyleSheet("")
        if not self._connected:
            QMessageBox.warning(
                self,
                "Follower not ready",
                "Connect the follower in Setup before starting live teleoperation.",
            )
            return
        if not self._leader_connected or self._latest_leader_state is None:
            QMessageBox.warning(
                self,
                "Leader not ready",
                "Connect the leader/controller arm before starting live teleoperation.",
            )
            return
        frequency_hz = float(
            self.teleop_rate_combo.currentData() or DEFAULT_TELEOP_STREAM_FREQUENCY_HZ
        )
        if not self.simulation_check.isChecked() and frequency_hz > 20.0:
            answer = QMessageBox.question(
                self,
                "Experimental teleop rate",
                f"{frequency_hz:.0f} Hz has not been physically validated with the "
                "current per-sample serial safety monitoring. Start at 20 Hz and "
                "only increase after timing/STOP tests pass. Continue anyway?",
            )
            if answer != QMessageBox.StandardButton.Yes:
                return

        # A parked leader must become back-drivable before live teaching. These
        # queued calls target the same worker, so release is processed before the
        # fresh pose read that seeds teleoperation.
        if self._leader_torque_enabled:
            self.leader_release_requested.emit()

        mode = "relative" if force_relative else self.teleop_mode_combo.currentData()
        if force_relative:
            self.teleop_mode_combo.setCurrentIndex(
                self.teleop_mode_combo.findData("relative")
            )
        self._teleop_starting = True
        self._teleop_start_options = {
            "mode": mode,
            "frequency_hz": frequency_hz,
            "mirror_gripper": self.teleop_gripper_check.isChecked(),
            "gripper_speed_multiplier": self._gripper_speed_multiplier,
            "align_follower": align_follower,
            "latch_follower_if_relaxed": not align_follower,
        }
        self.teleop_button.setText(
            "Cancel alignment" if align_follower else "Cancel relink"
        )
        self.teleop_status.setText(
            "Reading the leader's current pose for alignment…"
            if align_follower
            else "Reading both current poses for no-motion relative relink…"
        )
        self._update_enabled_state()
        self.leader_teleop_pose_requested.emit()

    def _toggle_teleop(self) -> None:
        if self._teleop_active or self._teleop_starting:
            self._teleop_starting = False
            self.teleop_stop_requested.emit()
            self.teleop_button.setText("Align follower and start")
            return
        self._begin_teleop(align_follower=True)

    @Slot(object)
    def _on_leader_teleop_start_pose(self, result: object) -> None:
        if not self._teleop_starting:
            return
        values = dict(result)  # type: ignore[arg-type]
        if "error" in values:
            self._teleop_starting = False
            self.teleop_button.setText("Align follower and start")
            self.teleop_status.setText(f"Could not read leader: {values['error']}")
            self._update_enabled_state()
            return
        if bool(self._teleop_start_options.get("align_follower", True)):
            self.teleop_status.setText(
                "Aligning follower with leader. Keep leader still until live following starts. "
                "STOP/HOLD or Cancel alignment stops this move."
            )
        else:
            self.teleop_status.setText(
                "Relinking current leader/follower poses with no alignment move. "
                "Keep the leader still until live following starts."
            )
        self.teleop_start_requested.emit({
            **self._teleop_start_options,
            "leader_joints_rad": values["joints_rad"],
            "leader_gripper": values["gripper"],
        })

    @Slot(object)
    def _on_teleop_alignment_adjusted(self, adjustments: object) -> None:
        values = dict(adjustments)  # type: ignore[arg-type]
        self.teleop_mode_combo.setCurrentIndex(self.teleop_mode_combo.findData("relative"))
        names = ", ".join(
            f"{str(name).replace('_', ' ')} {float(amount):.1f}°"
            for name, amount in values.items()
        )
        self._teleop_alignment_note = f"Alignment offset: {names}; relative mapping active."
        self.teleop_status.setText(
            f"Leader extends beyond the conservative motion range. Aligning with "
            f"{names} offset to a legal pose, then following in relative mode."
        )

    @Slot(object)
    def _on_teleop_faulted(self, details: object) -> None:
        values = dict(details)  # type: ignore[arg-type]
        reason = str(values.get("reason") or "teleoperation fault")
        self._teleop_error = reason
        self._teleop_fault_details = values
        self._teleop_active = False
        self._teleop_starting = False
        self.teleop_button.setText("Realign and restart")
        self.teleop_status.setStyleSheet(
            "font-weight: 800; padding: 9px; border-radius: 9px; "
            "background: #fee2e2; color: #7f1d1d;"
        )
        self.teleop_status.setText(self._teleop_fault_message(values))
        self.leader_stream_stop_requested.emit()
        self._update_enabled_state()

    @staticmethod
    def _teleop_fault_message(values: dict[str, Any]) -> str:
        reason = str(values.get("reason") or "teleoperation fault")
        frequency = float(values.get("frequency_hz", 0.0) or 0.0)
        recommended = float(values.get("recommended_frequency_hz", 0.0) or 0.0)
        processing_ms = float(values.get("processing_ms", 0.0) or 0.0)
        timing = (
            f" Last follower iteration: {processing_ms:.0f} ms at {frequency:.0f} Hz."
            if frequency > 0.0 and processing_ms > 0.0
            else ""
        )
        retry = (
            f" Select {recommended:.0f} Hz before retrying."
            if recommended > 0.0 and frequency > recommended
            else ""
        )
        return (
            "TELEOP STOPPED — follower is holding and leader/follower are DELINKED. "
            "No further leader motion will be sent until you explicitly realign or relink. "
            f"Cause: {reason}.{timing}{retry}"
        )

    def _on_teleop_changed(self, active: bool) -> None:
        self._teleop_active = bool(active)
        self._teleop_starting = False
        if active:
            frequency_hz = float(
                self.teleop_rate_combo.currentData()
                or DEFAULT_TELEOP_STREAM_FREQUENCY_HZ
            )
            self._teleop_fault_details = None
            self._teleop_error = None
            self.teleop_status.setStyleSheet(
                "font-weight: 700; padding: 7px; border-radius: 8px; "
                "background: #dcfce7; color: #14532d;"
            )
            self.teleop_button.setText("Stop live teleop")
            self.teleop_status.setText(
                f"LIVE / LINKED — {self.teleop_mode_combo.currentText()} "
                f"at {frequency_hz:.0f} Hz. "
                f"{self._teleop_alignment_note or ''}"
            )
            self.leader_stream_start_requested.emit(frequency_hz)
        else:
            self.leader_stream_stop_requested.emit()
            if self._teleop_fault_details is None:
                self.teleop_status.setStyleSheet("")
                self.teleop_button.setText("Align follower and start")
                self.teleop_status.setText(
                    "Teleoperation stopped normally · follower holding · leader/follower delinked."
                )
        self._update_enabled_state()

    def _on_leader_stream_readout_changed(self, active: bool) -> None:
        if not active and self._teleop_active:
            self.teleop_stop_requested.emit()

    def _on_leader_busy(self, busy: bool) -> None:
        self._leader_busy = busy
        if not busy and self._active_calibration_target == "leader":
            self._active_calibration_target = None
            self.calibration_status.setText(
                "Leader calibration ended without a completion result; see the log."
            )
        self._update_enabled_state()

    def _on_leader_connected(self, connected: bool) -> None:
        self._leader_connecting = False
        self._leader_connected = connected
        if connected and self.leader_allow_uncalibrated_check.isChecked():
            self.calibration_status.setText(
                "Leader connected for calibration with torque off. Start the sweep when ready."
            )
        if not connected:
            if self._teleop_active:
                self.teleop_stop_requested.emit()
            self._latest_leader_state = None
            self._leader_busy = False
            self._leader_torque_enabled = False
            self._refresh_sidebar_context()
        elif not self.leader_simulation_check.isChecked():
            self._save_arm_connection_profile("leader")
        self.leader_connect_button.setText("Disconnect leader" if connected else "Connect leader")
        self._update_teach_readout()
        self._refresh_sidebar_context()
        self._update_enabled_state()

    def _on_leader_state(self, state: object) -> None:
        self._latest_leader_state = dict(state)  # type: ignore[arg-type]
        self._leader_torque_enabled = bool(self._latest_leader_state.get("torque_enabled"))
        self._refresh_sidebar_context()
        self._update_teach_readout()
        self._update_enabled_state()

    def _update_teach_readout(self) -> None:
        if hasattr(self, "teleop_readout"):
            lines = [f"{'Joint':<20} {'Follower':>12} {'Leader':>12}"]
            for name in ARM_JOINTS:
                measured = []
                for state in (
                    self._latest_state if self._connected else None,
                    self._latest_leader_state if self._leader_connected else None,
                ):
                    measured.append(
                        f"{float(state['joints_deg'][name]):.1f}°" if state else "—"
                    )
                lines.append(
                    f"{name.replace('_', ' ').title():<20} "
                    f"{measured[0]:>12} {measured[1]:>12}"
                )
            follower_gripper = (
                f"{float(self._latest_state['gripper']):.3f}"
                if self._connected and self._latest_state else "—"
            )
            leader_gripper = (
                f"{float(self._latest_leader_state['gripper']):.3f}"
                if self._leader_connected and self._latest_leader_state else "—"
            )
            lines.append(
                f"{'Gripper':<20} {follower_gripper:>12} {leader_gripper:>12}"
            )
            self.teleop_readout.setText("\n".join(lines))
        if not hasattr(self, "teaching_source_combo"):
            return
        source = str(self.teaching_source_combo.currentData())
        state = self._latest_state if source == "follower" else self._latest_leader_state
        if not state:
            self.teach_source_status.setText(f"{source.title()} state unavailable")
            for label in self.teach_joint_labels.values():
                label.setText("—")
            self.teach_gripper_label.setText("—")
            self.teach_pose_label.setText("TCP: —")
            self._refresh_sidebar_context()
            return
        self.teach_source_status.setText(
            f"{source.title()} connected"
            + (" — simulation" if state.get("simulation") else "")
        )
        for name in ARM_JOINTS:
            self.teach_joint_labels[name].setText(
                f"{float(state['joints_deg'][name]):.1f}°"
            )
        self.teach_gripper_label.setText(f"{float(state['gripper']):.3f}")
        pose = [float(value) for value in state["pose_mm_deg"]]
        self.teach_pose_label.setText(
            f"TCP: X {pose[0]:.1f}, Y {pose[1]:.1f}, Z {pose[2]:.1f} mm\n"
            f"RPY: {pose[3]:.1f}°, {pose[4]:.1f}°, {pose[5]:.1f}°"
        )
        self._refresh_sidebar_context()

    def _move_joints(self) -> None:
        self.move_joints_requested.emit(
            {
                "joints_deg": {name: self.joint_spins[name].value() for name in ARM_JOINTS},
                "speed_deg_s": self.joint_speed.value(),
                "acceleration_deg_s2": self.joint_acceleration.value(),
            }
        )

    def _jog(self, axis: int, sign: float) -> None:
        if axis >= 3 and self.orientation_combo.currentData() == "position_only":
            self.orientation_combo.setCurrentIndex(0)
            self._log("Rotation jog switched orientation mode to 5-axis compatible.")
        translation = [0.0, 0.0, 0.0]
        rotation = [0.0, 0.0, 0.0]
        if axis < 3:
            translation[axis] = sign * self.linear_step.value()
        else:
            rotation[axis - 3] = sign * self.angular_step.value()
        self.jog_requested.emit(
            {
                "frame": self.frame_combo.currentData(),
                "translation_mm": translation,
                "rotation_rpy_deg": rotation,
                "orientation_mode": self.orientation_combo.currentData(),
                "speed_mm_s": self.linear_speed.value(),
                "acceleration_mm_s2": self.linear_acceleration.value(),
            }
        )

    @Slot(object)
    def _on_jog_queue_changed(self, state: object) -> None:
        values = dict(state)  # type: ignore[arg-type]
        self._cartesian_jog_active = bool(values.get("active", False))
        self._cartesian_jog_queued = int(values.get("queued", 0))
        if hasattr(self, "jog_queue_label"):
            if self._cartesian_jog_active:
                self.jog_queue_label.setText(
                    f"Jog queue: {self._cartesian_jog_queued} waiting"
                )
            else:
                self.jog_queue_label.setText("Jog queue: idle")
        if hasattr(self, "cartesian_view"):
            self.cartesian_view.set_queue_state(
                active=self._cartesian_jog_active,
                queued=self._cartesian_jog_queued,
            )
            if not self._cartesian_jog_active:
                self.cartesian_view.clear_target()
        self._update_enabled_state()

    @Slot(object)
    def _on_cartesian_jog_diagnostic(self, diagnostic: object) -> None:
        values = dict(diagnostic)  # type: ignore[arg-type]
        phase = str(values.get("phase") or "")
        target = values.get("target")
        if phase == "planned" and isinstance(target, dict):
            xyz = tuple(float(value) for value in target.get("xyz_mm", ()))
            if len(xyz) == 3 and hasattr(self, "cartesian_view"):
                self.cartesian_view.set_target_xyz_mm(xyz)
            start = values.get("start")
            if isinstance(start, dict) and len(xyz) == 3:
                start_xyz = tuple(float(value) for value in start.get("xyz_mm", ()))
                if len(start_xyz) == 3:
                    self.cartesian_diag_label.setText(
                        "Requested TCP: "
                        f"({start_xyz[0]:.1f}, {start_xyz[1]:.1f}, {start_xyz[2]:.1f}) → "
                        f"({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}) mm"
                    )
        elif phase == "completed":
            achieved = values.get("achieved")
            if isinstance(target, dict) and isinstance(achieved, dict):
                requested_xyz = tuple(float(value) for value in target.get("xyz_mm", ()))
                achieved_xyz = tuple(float(value) for value in achieved.get("xyz_mm", ()))
                if len(requested_xyz) == 3 and len(achieved_xyz) == 3:
                    error = sum(
                        (requested_xyz[index] - achieved_xyz[index]) ** 2
                        for index in range(3)
                    ) ** 0.5
                    self.cartesian_diag_label.setText(
                        "Achieved TCP: "
                        f"({achieved_xyz[0]:.1f}, {achieved_xyz[1]:.1f}, {achieved_xyz[2]:.1f}) mm "
                        f"· position error {error:.2f} mm"
                    )

    def _move_absolute_pose(self) -> None:
        values = [spin.value() for spin in self.absolute_spins]
        self.absolute_pose_requested.emit(
            {
                "xyz_mm": values[:3],
                "rpy_deg": values[3:],
                "orientation_mode": self.orientation_combo.currentData(),
                "speed_mm_s": self.linear_speed.value(),
                "acceleration_mm_s2": self.linear_acceleration.value(),
            }
        )

    def _load_current_targets(self) -> None:
        state = self._latest_state
        if not state:
            return
        for name in ARM_JOINTS:
            self.joint_spins[name].setValue(float(state["joints_deg"][name]))
        if not self._joint_targets_initialized:
            self._load_current_pose()
            self.gripper_spin.setValue(float(state["gripper"]))
        self._joint_targets_initialized = True

    def _load_current_pose(self) -> None:
        state = self._latest_state
        if not state:
            return
        for spin, value in zip(self.absolute_spins, state["pose_mm_deg"], strict=True):
            spin.setValue(float(value))

    def _on_connected(self, connected: bool) -> None:
        self._follower_connecting = False
        self._connected = connected
        if not connected:
            self._follower_setup_session = False
            self._torque_enabled = False
            self._busy = False
            self._joint_targets_initialized = False
            self.edit_joint_targets_check.setChecked(False)
            self._latest_effort_status = {"supported": False}
            self._effort_controls_initialized = False
            for panel in self._follower_status_panels:
                panel.clear_state()
        elif self._follower_setup_session:
            self.calibration_status.setText(
                "Follower connected for calibration with torque off. Start the sweep when ready."
            )
        if connected and not self.simulation_check.isChecked():
            self._save_arm_connection_profile("follower")
        self.connect_button.setText("Disconnect follower" if connected else "Connect follower")
        self._refresh_named_pose_status()
        self._refresh_point_list()
        self._refresh_sidebar_context()
        self._update_enabled_state()

    def _on_busy(self, busy: bool) -> None:
        self._busy = busy
        if self._latest_state is not None and hasattr(self, "robot_sidebar"):
            live = dict(self._latest_state)
            live["moving"] = bool(live.get("moving")) or busy
            self.robot_sidebar.update_state(live)
        if not busy:
            if self._active_calibration_target == "follower":
                self._active_calibration_target = None
                self.calibration_status.setText(
                    "Follower calibration ended without a completion result; see the log."
                )
            self._sequence_paused = False
            if hasattr(self, "pause_sequence_button"):
                self.pause_sequence_button.setText("Pause after current step")
        self._update_enabled_state()

    @Slot(object)
    def _on_follower_joint_measurements(self, joints: object) -> None:
        measured = {name: float(value) for name, value in dict(joints).items()}
        if self._latest_state is not None:
            self._latest_state["joints_deg"] = measured
        for name in ARM_JOINTS:
            self.joint_actual_labels[name].setText(f"{measured[name]:.1f}°")
            if not self.edit_joint_targets_check.isChecked():
                self.joint_spins[name].setValue(measured[name])
        if hasattr(self, "robot_sidebar"):
            self.robot_sidebar.update_joint_degrees(measured)
        self._update_teach_readout()

    def _on_state(self, state: object) -> None:
        values = dict(state)  # type: ignore[arg-type]
        self._latest_state = values
        self._connected = bool(values["connected"])
        self._torque_enabled = bool(values["torque_enabled"])
        if values.get("joint_limits_deg"):
            self._apply_joint_limits(dict(values["joint_limits_deg"]))
        moving = bool(values["moving"]) or self._busy
        faulted = bool(values["faulted"])

        for name in ARM_JOINTS:
            angle = float(values["joints_deg"][name])
            self.joint_actual_labels[name].setText(f"{angle:.1f}°")
        pose = tuple(float(value) for value in values["pose_mm_deg"])
        for label, value in zip(self.pose_value_labels, pose, strict=True):
            label.setText(f"{value:.2f}")
        live_sidebar_state = dict(values)
        live_sidebar_state["moving"] = moving
        for panel in self._follower_status_panels:
            panel.update_state(live_sidebar_state)
        self.pose_summary.setText(
            f"TCP: X {pose[0]:.1f}  Y {pose[1]:.1f}  Z {pose[2]:.1f} mm"
        )
        gripper = float(values["gripper"])
        self.gripper_measured.setText(f"Measured: {gripper:.3f}")

        if not self.edit_joint_targets_check.isChecked():
            self._load_current_targets()

        if faulted:
            status = f"FAULT: {values.get('fault_message') or 'unknown motor fault'}"
            self.status_label.setStyleSheet(
                "font-weight: 700; padding: 5px; background: #7f1d1d; color: white;"
            )
        elif moving:
            status = "Moving"
            self.status_label.setStyleSheet(
                "font-weight: 700; padding: 5px; background: #854d0e; color: white;"
            )
        elif self._torque_enabled:
            status = "Enabled / holding"
            self.status_label.setStyleSheet(
                "font-weight: 700; padding: 5px; background: #166534; color: white;"
            )
        else:
            status = "Connected / torque off"
            self.status_label.setStyleSheet("font-weight: 600; padding: 5px;")
        if values.get("simulation"):
            status += " — simulation"
        self.status_label.setText(status)
        self._update_teach_readout()
        self._update_enabled_state()

    def _on_effort_status(self, status: object) -> None:
        values = dict(status)  # type: ignore[arg-type]
        self._latest_effort_status = values
        supported = bool(values.get("supported", False))
        if not supported:
            self.effort_status_label.setText(
                "Effort telemetry is not available for this backend (simulation included)."
            )
            for labels in self.effort_value_labels.values():
                for label in labels.values():
                    label.setText("—")
            self._update_enabled_state()
            return

        enabled = bool(values.get("enabled", True))
        if not self._effort_controls_initialized:
            self.effort_guard_check.setChecked(enabled)
            current_trip = values.get("current_trip_raw")
            load_trip = values.get("load_trip_raw")
            self.effort_current_spin.setValue(
                0 if current_trip is None else int(current_trip)
            )
            self.effort_load_spin.setValue(
                0 if load_trip is None else int(load_trip)
            )
            self.effort_consecutive_spin.setValue(
                int(values.get("consecutive_samples", 2))
            )
            self._effort_controls_initialized = True

        readings = dict(values.get("readings") or {})
        peaks = dict(values.get("peaks") or {})
        limits = dict(values.get("effective_limits") or {})
        for motor, labels in self.effort_value_labels.items():
            reading = dict(readings.get(motor) or {})
            peak = dict(peaks.get(motor) or {})
            limit = dict(limits.get(motor) or {})
            labels["current"].setText(
                "—" if "current_raw" not in reading else str(int(reading["current_raw"]))
            )
            labels["load"].setText(
                "—" if "load_raw" not in reading else str(int(reading["load_raw"]))
            )
            labels["peak_current"].setText(str(int(peak.get("current_raw", 0))))
            labels["peak_load"].setText(str(int(peak.get("abs_load_raw", 0))))
            labels["current_limit"].setText(
                "off" if limit.get("current_raw") is None else str(int(limit["current_raw"]))
            )
            labels["load_limit"].setText(
                "off" if limit.get("load_raw") is None else str(int(limit["load_raw"]))
            )

        trip = values.get("trip_message")
        if trip:
            self.effort_status_label.setText(f"LATCHED TRIP: {trip}")
            self.effort_status_label.setStyleSheet(
                "font-weight: 700; padding: 4px; background: #7f1d1d; color: white;"
            )
        else:
            state = "enabled" if enabled else "DISABLED"
            self.effort_status_label.setText(
                f"Effort guard {state}. Raw current/load are diagnostic signals; "
                "session peaks accumulate until Reset peaks."
            )
            self.effort_status_label.setStyleSheet("padding: 4px;")
        self._update_enabled_state()

    def _apply_effort_settings(self) -> None:
        if self._torque_enabled:
            QMessageBox.warning(
                self,
                "Torque must be off",
                "Relax the follower before changing motor-effort safety thresholds.",
            )
            return
        enabled = self.effort_guard_check.isChecked()
        if not enabled:
            answer = QMessageBox.question(
                self,
                "Disable effort guard?",
                "This disables only current/load collision guarding for this session. "
                "Other SDK safety checks remain active. Continue?",
            )
            if answer != QMessageBox.StandardButton.Yes:
                self.effort_guard_check.setChecked(True)
                return
        self.effort_configure_requested.emit(
            {
                "enabled": enabled,
                "current_trip_raw": self.effort_current_spin.value(),
                "load_trip_raw": self.effort_load_spin.value(),
                "consecutive_samples": self.effort_consecutive_spin.value(),
            }
        )

    def _clear_effort_trip(self) -> None:
        trip = self._latest_effort_status.get("trip_message")
        if not trip:
            return
        answer = QMessageBox.question(
            self,
            "Clear effort trip?",
            "Only clear the latched trip after the obstruction/contact is removed. "
            "Clearing does not move the arm. Continue?",
        )
        if answer == QMessageBox.StandardButton.Yes:
            self.effort_clear_requested.emit()

    def _update_enabled_state(self) -> None:
        if hasattr(self, "follower_session_status"):
            follower_status = (
                "Connected · holding" if self._torque_enabled else "Connected · relaxed"
            ) if self._connected else "Disconnected · press Connect follower"
            if self._follower_setup_session and self._connected:
                follower_status = "Connected for calibration · movement disabled"
            if self._follower_connecting:
                follower_status = "Connecting follower…"
            self.follower_session_status.setText(follower_status)
            self.leader_session_status.setText(
                "Connecting leader…" if self._leader_connecting else
                "Connected · PARKED · torque on" if self._leader_connected and self._leader_torque_enabled
                else "Connected · FREE · move by hand" if self._leader_connected
                else "Disconnected · press Connect leader"
            )
        if hasattr(self, "teleop_status") and self._teleop_starting:
            pass
        elif hasattr(self, "teleop_status") and not self._teleop_active and self._teleop_error:
            if self._teleop_fault_details is not None:
                self.teleop_button.setText("Realign and restart")
                self.teleop_status.setStyleSheet(
                    "font-weight: 800; padding: 9px; border-radius: 9px; "
                    "background: #fee2e2; color: #7f1d1d;"
                )
                self.teleop_status.setText(
                    self._teleop_fault_message(self._teleop_fault_details)
                )
            else:
                self.teleop_status.setText(f"Teleoperation stopped: {self._teleop_error}")
        elif hasattr(self, "teleop_status") and not self._teleop_active:
            if not self._connected or not self._leader_connected:
                message = "Connect Follower and Leader using the panels above."
            elif self._follower_setup_session:
                message = "Complete follower calibration before teleoperation."
            elif self._recording_source:
                message = "Stop recording before starting teleoperation."
            else:
                message = "Ready. Align follower and start; keep the leader still until live following begins."
            self.teleop_status.setText(message)
        self._update_coordination_panels()
        simulation = self.simulation_check.isChecked()
        session_editable = not self._connected and not self._follower_connecting and not self._busy
        self.connect_button.setEnabled(not self._follower_connecting and not self._busy)
        self.simulation_check.setEnabled(session_editable)
        self.port_combo.setEnabled(session_editable and not simulation)
        self.refresh_ports_button.setEnabled(session_editable and not simulation)
        self.robot_id_edit.setEnabled(session_editable)
        self.allow_uncalibrated_check.setEnabled(session_editable and not simulation)
        self.find_arms_button.setEnabled(
            session_editable
            and not simulation
            and not self._leader_connected
            and not self._busy
        )
        self.setup_find_arms_button.setEnabled(self.find_arms_button.isEnabled())

        can_enable_hold = (
            self._connected
            and not self._torque_enabled
            and not self._busy
            and not self._follower_setup_session
        )
        self.enable_button.setEnabled(can_enable_hold)
        self.sidebar_enable_button.setEnabled(can_enable_hold)
        can_relax = self._connected and self._torque_enabled
        self.relax_button.setEnabled(can_relax)
        self.sidebar_relax_button.setEnabled(can_relax)
        self.stop_button.setEnabled(self._connected)
        self.edit_joint_targets_check.setEnabled(self._connected and not self._busy)
        for control in (*self.joint_sliders.values(), *self.joint_spins.values()):
            control.setEnabled(self._connected and self.edit_joint_targets_check.isChecked()
                               and not self._busy)

        motion_ready = (
            self._connected
            and self._torque_enabled
            and not self._follower_setup_session
        )
        can_move = motion_ready and not self._busy
        can_jog = motion_ready and (not self._busy or self._cartesian_jog_active)
        self.move_joints_button.setEnabled(can_move and self.edit_joint_targets_check.isChecked())
        self.absolute_move_button.setEnabled(can_move)
        self.use_current_pose_button.setEnabled(self._connected and not self._busy)
        for button in self.jog_buttons:
            button.setEnabled(can_jog)
        self.close_gripper_button.setEnabled(can_move)
        self.move_gripper_button.setEnabled(can_move)
        self.open_gripper_button.setEnabled(can_move)

        calibration_target = str(self.calibration_target_combo.currentData())
        if calibration_target == "leader":
            calibration_connected = self._leader_connected
            calibration_simulation = self.leader_simulation_check.isChecked()
            calibration_busy = self._leader_busy
        else:
            calibration_connected = self._connected
            calibration_simulation = simulation
            calibration_busy = self._busy
        any_calibration_busy = self._busy or self._leader_busy
        self.setup_connect_button.setEnabled(
            self._active_calibration_target is None
            and not calibration_connected
            and not calibration_simulation
            and not any_calibration_busy
        )
        self.calibration_target_combo.setEnabled(
            self._active_calibration_target is None and not any_calibration_busy
        )
        self.calibration_duration.setEnabled(not any_calibration_busy)
        self.run_calibration_button.setEnabled(
            self._active_calibration_target is None
            and calibration_connected
            and not calibration_simulation
            and not calibration_busy
            and not any_calibration_busy
            and self._recording_source is None
            and not self._teleop_active
        )
        self.cancel_calibration_button.setEnabled(
            self._active_calibration_target is not None and not self._calibration_cancelling
        )
        self.leader_allow_uncalibrated_check.setEnabled(
            not self._leader_connected
            and not self.leader_simulation_check.isChecked()
            and not any_calibration_busy
        )

        effort_supported = bool(self._latest_effort_status.get("supported", False))
        effort_editable = (
            self._connected
            and effort_supported
            and not self._torque_enabled
            and not self._busy
        )
        self.effort_guard_check.setEnabled(effort_editable)
        self.effort_current_spin.setEnabled(effort_editable)
        self.effort_load_spin.setEnabled(effort_editable)
        self.effort_consecutive_spin.setEnabled(effort_editable)
        self.effort_apply_button.setEnabled(effort_editable)
        self.effort_refresh_button.setEnabled(
            self._connected and effort_supported and not self._busy and not self._teleop_active
        )
        self.effort_reset_peaks_button.setEnabled(self._connected and effort_supported)
        self.effort_clear_button.setEnabled(
            self._connected
            and effort_supported
            and not self._busy
            and not self._teleop_active
            and bool(self._latest_effort_status.get("trip_message"))
        )

        can_save_pose = (
            self._connected
            and self._recording_source is None
            and (not self._busy or self._teleop_active)
        )
        self.save_home_button.setEnabled(can_save_pose)
        self.save_rest_button.setEnabled(can_save_pose)
        try:
            library = self._get_pose_library()
            has_home = library.get(HOME_POSE_NAME) is not None
            has_rest = library.get(REST_POSE_NAME) is not None
        except Exception:
            has_home = has_rest = False
        self.go_home_button.setEnabled(can_move and has_home)
        self.go_rest_button.setEnabled(can_move and has_rest)

        leader_editable = not self._leader_connected and not self._leader_connecting and not self._leader_busy and not self._busy
        self.leader_connect_button.setEnabled(not self._leader_connecting and not self._leader_busy and not self._busy)
        self.leader_simulation_check.setEnabled(leader_editable)
        self.leader_port_combo.setEnabled(
            leader_editable and not self.leader_simulation_check.isChecked()
        )
        self.leader_refresh_button.setEnabled(
            leader_editable and not self.leader_simulation_check.isChecked()
        )
        self.leader_robot_id_edit.setEnabled(leader_editable)

        source, source_state = self._current_teaching_state()
        can_save_point = (
            self._recording_source is None
            and (
                (source == "follower" and self._connected)
                or (source != "follower" and source_state is not None)
            )
        )
        self.save_point_button.setEnabled(can_save_point)
        has_point = bool(self.point_combo.currentText())
        self.move_point_button.setEnabled(can_move and has_point)
        self.add_point_to_program_button.setEnabled(has_point and not self._busy)
        self.delete_point_button.setEnabled(has_point and self._recording_source is None)

        source_available = source_state is not None
        self.record_button.setEnabled(
            self._recording_source is not None
            or (source_available and not self._busy and not self._teleop_active)
        )

        can_start_teleop = (
            self._connected
            and not self._follower_setup_session
            and self._leader_connected
            and self._latest_leader_state is not None
            and self._recording_source is None
            and not self._busy
            and not self._leader_busy
            and self._sync_capture_pending is None
        )
        self.teleop_button.setEnabled(
            self._teleop_active or self._teleop_starting or can_start_teleop
        )
        self.transfer_manual_button.setEnabled(
            self._connected
            and self._leader_connected
            and not self._leader_busy
            and self._recording_source is None
        )
        self.teleop_mode_combo.setEnabled(not self._teleop_active and not self._teleop_starting)
        self.teleop_rate_combo.setEnabled(not self._teleop_active and not self._teleop_starting)
        self.teleop_gripper_check.setEnabled(not self._teleop_active and not self._teleop_starting)
        for combo in (
            self.teleop_gripper_speed_combo,
            self.manual_gripper_speed_combo,
            self.trajectory_gripper_speed_combo,
            self.run_gripper_speed_combo,
        ):
            combo.setEnabled(not self._teleop_active and not self._teleop_starting)
        follower_recording = self._recording_source == "follower"
        if follower_recording:
            self.move_joints_button.setEnabled(False)
            self.absolute_move_button.setEnabled(False)
            for button in self.jog_buttons:
                button.setEnabled(False)
            self.close_gripper_button.setEnabled(False)
            self.move_gripper_button.setEnabled(False)
            self.open_gripper_button.setEnabled(False)
            self.move_point_button.setEnabled(False)

        has_trajectory = self._active_trajectory is not None
        self.replay_full_button.setEnabled(can_move and has_trajectory)
        self.replay_selection_button.setEnabled(can_move and has_trajectory)
        self.save_edited_trajectory_button.setEnabled(has_trajectory)
        self.load_trajectory_button.setEnabled(self.trajectory_combo.count() > 0)
        for control in (
            self.smooth_button,
            self.delete_selection_button,
            self.insert_hold_button,
            self.set_keyframe_button,
            self.add_marker_button,
            self.loop_selection_button,
        ):
            control.setEnabled(has_trajectory)
        self.promote_primitive_button.setEnabled(has_trajectory)

        has_steps = bool(self._sequence_steps)
        can_run_sequence = can_move and has_steps
        self.run_sequence_button.setEnabled(can_run_sequence)
        self.run_step_button.setEnabled(
            can_run_sequence and self.sequence_step_list.currentRow() >= 0
        )
        self.stop_sequence_button.setEnabled(self._connected)
        self.pause_sequence_button.setEnabled(self._busy and not self._teleop_active)
        self.sequence_up_button.setEnabled(
            self.sequence_step_list.currentRow() > 0 and not self._busy
        )
        self.sequence_down_button.setEnabled(
            0 <= self.sequence_step_list.currentRow() < len(self._sequence_steps) - 1
            and not self._busy
        )
        self.sequence_delete_step_button.setEnabled(
            self.sequence_step_list.currentRow() >= 0 and not self._busy
        )
        self.save_sequence_button.setEnabled(has_steps and not self._busy)
        self.load_sequence_button.setEnabled(
            self.sequence_combo.count() > 0 and not self._busy
        )
        self.delete_sequence_button.setEnabled(
            self.sequence_combo.count() > 0 and not self._busy
        )
        self.add_point_step_button.setEnabled(
            self.run_point_combo.count() > 0 and not self._busy
        )
        self.add_home_step_button.setEnabled(has_home and not self._busy)
        self.add_rest_step_button.setEnabled(has_rest and not self._busy)
        self.add_gripper_step_button.setEnabled(not self._busy)
        self.program_open_gripper_button.setEnabled(not self._busy)
        self.program_close_gripper_button.setEnabled(not self._busy)
        self.program_move_speed_spin.setEnabled(not self._busy)
        self.run_point_mode_combo.setEnabled(not self._busy)
        self.radial_pan_start_spin.setEnabled(not self._busy)
        self.radial_pan_end_spin.setEnabled(not self._busy)
        self.radial_pan_step_spin.setEnabled(not self._busy)
        self.add_radial_pan_pattern_button.setEnabled(
            self.run_point_combo.count() > 0 and not self._busy
        )
        self.add_wait_step_button.setEnabled(not self._busy)
        self.add_trajectory_step_button.setEnabled(
            self.run_trajectory_combo.count() > 0 and not self._busy
        )
        self.add_primitive_step_button.setEnabled(
            self.run_primitive_combo.count() > 0 and not self._busy
        )

    @Slot(str)
    def _on_error(self, message: str) -> None:
        self._log(message)
        self.alert_label.setText(message)
        self.alert_label.show()
        self.tabs.setTabText(self.tabs.indexOf(self.log_page), "Log •")
        if any(prefix in message.lower() for prefix in (
            "live teleoperation:", "teleop alignment:", "start teleoperation:",
        )):
            self._teleop_error = message.partition(":")[2].strip()
            if message.lower().startswith("live teleoperation:"):
                if self._teleop_fault_details is None:
                    self._teleop_fault_details = {
                        "reason": self._teleop_error,
                        "requires_relink": True,
                        "follower_holding": True,
                    }
                prominent = (
                    "TELEOP STOPPED — follower holding; leader/follower delinked. "
                    + self._teleop_error
                )
                self.alert_label.setText(prominent)
            self._update_enabled_state()
        self.statusBar().showMessage(self.alert_label.text(), 8000)
        if "calibration:" in message.lower():
            self._active_calibration_target = None
            self._calibration_cancelling = False
            self.tabs.setCurrentWidget(self.calibration_page)
            voltage_fault = "input voltage error" in message.lower()
            self.calibration_time_bar.setFormat(
                "STOPPED — SERVO VOLTAGE FAULT" if voltage_fault else "CALIBRATION FAILED"
            )
            self.calibration_status.setStyleSheet(
                "font-weight: 700; padding: 8px; background: #fee2e2; color: #7f1d1d;"
            )
            rollback_failed = "rollback also failed" in message.lower()
            preservation = (
                "Motor settings could not be restored; keep torque off."
                if rollback_failed else
                "Previous motor calibration restored. Saved calibration file unchanged."
            )
            self.calibration_status.setText(
                "Calibration stopped because a servo reported an input-voltage fault. "
                "Check the supply and gripper cable before retrying. " + preservation + " " + message
                if voltage_fault else f"Calibration failed: {message} {preservation}"
            )
            QMessageBox.warning(self, "Calibration failed", f"{message}\n\n{preservation}")
        elif "motors appear uncalibrated" in message:
            self.tabs.setCurrentWidget(self.calibration_page)
            self.calibration_status.setText(
                "The arm has uncalibrated motors. Select it here, then click "
                "'Connect for calibration (torque off)' before starting the sweep."
            )

    @Slot(str)
    def _log(self, message: str) -> None:
        self.log.append(message)
        record_session("gui_log", message=message)

    def closeEvent(self, event: QCloseEvent) -> None:
        try:
            if hasattr(self, "_camera_manager"):
                self._camera_manager.shutdown()
            if self._active_calibration_target is not None:
                # A sweep blocks its worker event loop, so request rollback
                # before waiting for the queued shutdown slot.
                (self._leader_worker if self._active_calibration_target == "leader"
                 else self._worker).request_calibration_cancel()
            if self._leader_thread.isRunning():
                QMetaObject.invokeMethod(
                    self._leader_worker,
                    "shutdown",
                    Qt.ConnectionType.BlockingQueuedConnection,
                )
                self._leader_thread.quit()
                self._leader_thread.wait(3000)
            if self._thread.isRunning():
                QMetaObject.invokeMethod(
                    self._worker,
                    "shutdown",
                    Qt.ConnectionType.BlockingQueuedConnection,
                )
                self._thread.quit()
                self._thread.wait(3000)
        finally:
            event.accept()
