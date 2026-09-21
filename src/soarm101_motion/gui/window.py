"""PySide6 control window for the SO-ARM101."""

from __future__ import annotations

from math import degrees, radians
from typing import Any

from PySide6.QtCore import QMetaObject, Qt, QThread, Signal
from PySide6.QtGui import QCloseEvent
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
    QSlider,
    QSpinBox,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from soarm101_motion.constants import ARM_JOINTS, JOINT_LIMITS
from soarm101_motion.gui.timeline import TrajectoryTimeline
from soarm101_motion.gui.worker import RobotWorker
from soarm101_motion.hardware import FeetechBackend
from soarm101_motion.poses import HOME_POSE_NAME, REST_POSE_NAME, PoseLibrary, SavedPose
from soarm101_motion.primitives import MotionPrimitive, MotionPrimitiveLibrary
from soarm101_motion.sequences import MotionSequence, SequenceLibrary, SequenceStep
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary


class MainWindow(QMainWindow):
    connect_requested = Signal(object)
    disconnect_requested = Signal()
    enable_requested = Signal()
    relax_requested = Signal()
    stop_requested = Signal()
    move_joints_requested = Signal(object)
    jog_requested = Signal(object)
    absolute_pose_requested = Signal(object)
    gripper_requested = Signal(float)
    calibration_requested = Signal(float)
    leader_connect_requested = Signal(object)
    leader_disconnect_requested = Signal()
    move_saved_pose_requested = Signal(object)
    trajectory_play_requested = Signal(object)
    recording_start_requested = Signal(object)
    recording_stop_requested = Signal()
    leader_recording_start_requested = Signal(object)
    leader_recording_stop_requested = Signal()
    leader_stream_start_requested = Signal(float)
    leader_stream_stop_requested = Signal()
    teleop_start_requested = Signal(object)
    teleop_stop_requested = Signal()
    sequence_run_requested = Signal(object)
    sequence_pause_requested = Signal()
    sequence_resume_requested = Signal()

    def __init__(
        self,
        *,
        port: str | None = None,
        robot_id: str = "so101",
        simulation: bool = False,
    ) -> None:
        super().__init__()
        self.setWindowTitle("SO-ARM101 Control")
        self.resize(1050, 780)

        self._connected = False
        self._torque_enabled = False
        self._busy = False
        self._latest_state: dict[str, Any] | None = None
        self._joint_targets_initialized = False
        self._last_joint_limits: dict[str, tuple[float, float]] | None = None
        self._leader_connected = False
        self._latest_leader_state: dict[str, Any] | None = None
        self._pose_library_cache: tuple[str, PoseLibrary] | None = None
        self._trajectory_library_cache: tuple[str, TrajectoryLibrary] | None = None
        self._active_trajectory: Trajectory | None = None
        self._recording_source: str | None = None
        self._pending_recording_name: str | None = None
        self._teleop_active = False
        self._sequence_library_cache: tuple[str, SequenceLibrary] | None = None
        self._primitive_library_cache: tuple[str, MotionPrimitiveLibrary] | None = None
        self._sequence_steps: list[SequenceStep] = []
        self._sequence_paused = False

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
        self.move_saved_pose_requested.connect(self._worker.move_saved_pose)
        self.trajectory_play_requested.connect(self._worker.play_trajectory)
        self.recording_start_requested.connect(self._worker.start_recording)
        self.recording_stop_requested.connect(self._worker.stop_recording)
        self.teleop_start_requested.connect(self._worker.start_teleop)
        self.teleop_stop_requested.connect(self._worker.stop_teleop)
        self.sequence_run_requested.connect(self._worker.run_sequence)
        self.sequence_pause_requested.connect(self._worker.pause_sequence)
        self.sequence_resume_requested.connect(self._worker.resume_sequence)

        self._worker.state_changed.connect(self._on_state)
        self._worker.connected_changed.connect(self._on_connected)
        self._worker.busy_changed.connect(self._on_busy)
        self._worker.log_message.connect(self._log)
        self._worker.error_message.connect(self._on_error)
        self._worker.calibration_completed.connect(self._on_calibration_completed)
        self._worker.recording_completed.connect(self._on_recording_completed)
        self._worker.recording_changed.connect(
            lambda active: self._on_recording_changed("follower", active)
        )
        self._worker.teleop_changed.connect(self._on_teleop_changed)
        self._worker.sequence_progress.connect(self._on_sequence_progress)

        self._leader_thread = QThread(self)
        self._leader_worker = RobotWorker()
        self._leader_worker.moveToThread(self._leader_thread)
        self._leader_thread.started.connect(self._leader_worker.start)
        self._leader_thread.finished.connect(self._leader_worker.deleteLater)
        self.leader_connect_requested.connect(self._leader_worker.connect_robot)
        self.leader_disconnect_requested.connect(self._leader_worker.disconnect_robot)
        self.leader_recording_start_requested.connect(self._leader_worker.start_recording)
        self.leader_recording_stop_requested.connect(self._leader_worker.stop_recording)
        self.leader_stream_start_requested.connect(self._leader_worker.start_stream_readout)
        self.leader_stream_stop_requested.connect(self._leader_worker.stop_stream_readout)
        self._leader_worker.stream_sample.connect(self._worker.apply_teleop_sample)
        self._leader_worker.stream_readout_changed.connect(
            self._on_leader_stream_readout_changed
        )
        self._leader_worker.state_changed.connect(self._on_leader_state)
        self._leader_worker.connected_changed.connect(self._on_leader_connected)
        self._leader_worker.log_message.connect(lambda message: self._log(f"Leader: {message}"))
        self._leader_worker.error_message.connect(
            lambda message: self._on_error(f"Leader: {message}")
        )
        self._leader_worker.recording_completed.connect(self._on_recording_completed)
        self._leader_worker.recording_changed.connect(
            lambda active: self._on_recording_changed("leader", active)
        )

        self._build_ui(port=port, robot_id=robot_id, simulation=simulation)
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

    def _build_ui(self, *, port: str | None, robot_id: str, simulation: bool) -> None:
        root = QWidget(self)
        layout = QVBoxLayout(root)
        layout.addWidget(self._build_connection_bar(port, robot_id, simulation))

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_setup_tab(), "Setup")
        self.tabs.addTab(self._build_control_tab(), "Control")
        self.tabs.addTab(self._build_teach_tab(), "Teach")
        self.tabs.addTab(self._build_trajectory_tab(), "Trajectories")
        self.tabs.addTab(self._build_run_tab(), "Run")
        layout.addWidget(self.tabs, 1)

        status_row = QHBoxLayout()
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("font-weight: 600; padding: 5px;")
        status_row.addWidget(self.status_label, 1)
        self.pose_summary = QLabel("TCP: —")
        status_row.addWidget(self.pose_summary)
        layout.addLayout(status_row)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(150)
        self.log.setPlaceholderText("Connection, motion, and safety messages appear here.")
        layout.addWidget(self.log)
        self.setCentralWidget(root)

    def _build_connection_bar(
        self,
        port: str | None,
        robot_id: str,
        simulation: bool,
    ) -> QGroupBox:
        box = QGroupBox("Robot session")
        layout = QGridLayout(box)

        self.simulation_check = QCheckBox("Simulation")
        self.simulation_check.setChecked(simulation)
        self.simulation_check.toggled.connect(lambda _checked: self._update_enabled_state())
        layout.addWidget(self.simulation_check, 0, 0)

        layout.addWidget(QLabel("Port"), 0, 1)
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        if port:
            self.port_combo.addItem(port)
            self.port_combo.setCurrentText(port)
        layout.addWidget(self.port_combo, 0, 2)

        self.refresh_ports_button = QPushButton("Refresh")
        self.refresh_ports_button.clicked.connect(self._refresh_ports)
        layout.addWidget(self.refresh_ports_button, 0, 3)

        layout.addWidget(QLabel("Robot ID"), 0, 4)
        self.robot_id_edit = QLineEdit(robot_id)
        self.robot_id_edit.textChanged.connect(
            lambda _text: self._on_robot_id_changed()
        )
        layout.addWidget(self.robot_id_edit, 0, 5)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self._toggle_connection)
        layout.addWidget(self.connect_button, 0, 6)

        self.enable_button = QPushButton("Enable")
        self.enable_button.setToolTip("Latch current positions, then enable torque.")
        self.enable_button.clicked.connect(lambda _checked=False: self.enable_requested.emit())
        layout.addWidget(self.enable_button, 1, 0)

        self.stop_button = QPushButton("STOP / HOLD")
        self.stop_button.setStyleSheet("font-weight: 700; padding: 7px;")
        self.stop_button.setToolTip("Software stop only. Keep physical power accessible.")
        self.stop_button.clicked.connect(lambda _checked=False: self.stop_requested.emit())
        layout.addWidget(self.stop_button, 1, 1, 1, 3)

        self.relax_button = QPushButton("Relax")
        self.relax_button.setToolTip("Disable servo torque.")
        self.relax_button.clicked.connect(lambda _checked=False: self.relax_requested.emit())
        layout.addWidget(self.relax_button, 1, 4)

        self.read_targets_button = QPushButton("Load current into controls")
        self.read_targets_button.clicked.connect(self._load_current_targets)
        layout.addWidget(self.read_targets_button, 1, 5, 1, 2)

        self.allow_uncalibrated_check = QCheckBox("Setup: allow uncalibrated connection")
        self.allow_uncalibrated_check.setToolTip(
            "For calibration/setup only. Keep torque off until calibration is complete."
        )
        layout.addWidget(self.allow_uncalibrated_check, 2, 0, 1, 7)
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

        calibration = QGroupBox("Mechanical-stop calibration")
        grid = QGridLayout(calibration)
        explanation = QLabel(
            "Calibration keeps torque off while you sweep every joint and the gripper "
            "between both mechanical stops. Zero is computed from the midpoint of the "
            "observed extrema; no visual midpoint placement is required."
        )
        explanation.setWordWrap(True)
        grid.addWidget(explanation, 0, 0, 1, 3)
        grid.addWidget(QLabel("Sweep duration"), 1, 0)
        self.calibration_duration = self._spin(
            5.0, 120.0, 30.0, decimals=1, step=5.0, suffix=" s"
        )
        grid.addWidget(self.calibration_duration, 1, 1)
        self.run_calibration_button = QPushButton("Start live calibration")
        self.run_calibration_button.clicked.connect(self._start_calibration)
        grid.addWidget(self.run_calibration_button, 1, 2)
        self.calibration_status = QLabel(
            "Not run in this session. Simulation cannot perform encoder calibration."
        )
        self.calibration_status.setWordWrap(True)
        grid.addWidget(self.calibration_status, 2, 0, 1, 3)
        layout.addWidget(calibration)

        later = QLabel(
            "Physical validation is intentionally deferred. Follow TESTING.md when you are "
            "ready to use the real arm."
        )
        later.setWordWrap(True)
        layout.addWidget(later)
        layout.addStretch(1)
        return page

    def _build_control_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(self._build_named_pose_controls())
        subtabs = QTabWidget()
        subtabs.addTab(self._build_joint_tab(), "Joints")
        subtabs.addTab(self._build_cartesian_tab(), "Cartesian")
        subtabs.addTab(self._build_gripper_tab(), "Gripper")
        layout.addWidget(subtabs, 1)
        return page

    def _build_named_pose_controls(self) -> QGroupBox:
        box = QGroupBox("Standard poses")
        grid = QGridLayout(box)
        self.home_status = QLabel("Home: not saved")
        self.rest_status = QLabel("Rest: not saved")
        self.save_home_button = QPushButton("Save current as Home")
        self.go_home_button = QPushButton("Go Home")
        self.save_rest_button = QPushButton("Save current as Rest")
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

    def _build_teach_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        leader = QGroupBox("Leader / controller arm")
        grid = QGridLayout(leader)
        self.leader_simulation_check = QCheckBox("Simulation")
        self.leader_simulation_check.setChecked(self.simulation_check.isChecked())
        self.leader_simulation_check.toggled.connect(
            lambda _checked: self._update_enabled_state()
        )
        grid.addWidget(self.leader_simulation_check, 0, 0)
        grid.addWidget(QLabel("Port"), 0, 1)
        self.leader_port_combo = QComboBox()
        self.leader_port_combo.setEditable(True)
        grid.addWidget(self.leader_port_combo, 0, 2)
        self.leader_refresh_button = QPushButton("Refresh")
        self.leader_refresh_button.clicked.connect(self._refresh_leader_ports)
        grid.addWidget(self.leader_refresh_button, 0, 3)
        grid.addWidget(QLabel("Robot ID"), 0, 4)
        self.leader_robot_id_edit = QLineEdit(
            (self.robot_id_edit.text().strip() or "so101") + "-leader"
        )
        grid.addWidget(self.leader_robot_id_edit, 0, 5)
        self.leader_connect_button = QPushButton("Connect leader")
        self.leader_connect_button.clicked.connect(self._toggle_leader_connection)
        grid.addWidget(self.leader_connect_button, 0, 6)
        note = QLabel(
            "The leader is read-only here: leave its torque off and move it by hand. "
            "Live leader→follower teleoperation is a later stage."
        )
        note.setWordWrap(True)
        grid.addWidget(note, 1, 0, 1, 7)
        layout.addWidget(leader)

        teleop = QGroupBox("Live leader → follower teleoperation")
        teleop_grid = QGridLayout(teleop)
        teleop_grid.addWidget(QLabel("Mapping"), 0, 0)
        self.teleop_mode_combo = QComboBox()
        self.teleop_mode_combo.addItem("Relative / clutch-safe", "relative")
        self.teleop_mode_combo.addItem("Absolute calibrated angles", "absolute")
        teleop_grid.addWidget(self.teleop_mode_combo, 0, 1)
        self.teleop_gripper_check = QCheckBox("Mirror gripper")
        self.teleop_gripper_check.setChecked(True)
        teleop_grid.addWidget(self.teleop_gripper_check, 0, 2)
        self.teleop_button = QPushButton("Start live teleop")
        self.teleop_button.clicked.connect(self._toggle_teleop)
        teleop_grid.addWidget(self.teleop_button, 0, 3)
        self.teleop_status = QLabel(
            "50 Hz guarded streaming. Relative mode maps leader motion from the "
            "follower's current pose and avoids a startup jump."
        )
        self.teleop_status.setWordWrap(True)
        teleop_grid.addWidget(self.teleop_status, 1, 0, 1, 4)
        layout.addWidget(teleop)

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
        layout.addWidget(source)

        points = QGroupBox("Taught points")
        point_grid = QGridLayout(points)
        point_grid.addWidget(QLabel("New point name"), 0, 0)
        self.point_name_edit = QLineEdit()
        self.point_name_edit.setPlaceholderText("pick, above_drop, camera_pose...")
        point_grid.addWidget(self.point_name_edit, 0, 1, 1, 2)
        self.save_point_button = QPushButton("Save current point")
        self.save_point_button.clicked.connect(self._save_taught_point)
        point_grid.addWidget(self.save_point_button, 0, 3)
        point_grid.addWidget(QLabel("Saved point"), 1, 0)
        self.point_combo = QComboBox()
        point_grid.addWidget(self.point_combo, 1, 1)
        self.point_mode_combo = QComboBox()
        self.point_mode_combo.addItem("Joint / angular", "joint")
        self.point_mode_combo.addItem("Cartesian linear", "linear")
        point_grid.addWidget(self.point_mode_combo, 1, 2)
        self.move_point_button = QPushButton("Move follower to point")
        self.move_point_button.clicked.connect(self._move_taught_point)
        point_grid.addWidget(self.move_point_button, 1, 3)
        self.delete_point_button = QPushButton("Delete point")
        self.delete_point_button.clicked.connect(self._delete_taught_point)
        point_grid.addWidget(self.delete_point_button, 2, 3)
        layout.addWidget(points)

        recording = QGroupBox("Exact trajectory recording")
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
            "Raw recordings are immutable. Editing always creates a derived trajectory."
        )
        self.recording_status.setWordWrap(True)
        record_grid.addWidget(self.recording_status, 2, 0, 1, 4)
        layout.addWidget(recording)

        layout.addStretch(1)
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
        layout = QVBoxLayout(page)

        library_box = QGroupBox("Sequence library")
        library_grid = QGridLayout(library_box)
        library_grid.addWidget(QLabel("Saved sequence"), 0, 0)
        self.sequence_combo = QComboBox()
        library_grid.addWidget(self.sequence_combo, 0, 1)
        self.load_sequence_button = QPushButton("Load")
        self.load_sequence_button.clicked.connect(self._load_sequence)
        library_grid.addWidget(self.load_sequence_button, 0, 2)
        self.delete_sequence_button = QPushButton("Delete")
        self.delete_sequence_button.clicked.connect(self._delete_sequence)
        library_grid.addWidget(self.delete_sequence_button, 0, 3)
        library_grid.addWidget(QLabel("Name"), 1, 0)
        self.sequence_name_edit = QLineEdit()
        self.sequence_name_edit.setPlaceholderText("pick_and_place")
        library_grid.addWidget(self.sequence_name_edit, 1, 1, 1, 2)
        self.save_sequence_button = QPushButton("Save / replace sequence")
        self.save_sequence_button.clicked.connect(self._save_sequence)
        library_grid.addWidget(self.save_sequence_button, 1, 3)
        layout.addWidget(library_box)

        builder = QGroupBox("Add steps")
        builder_grid = QGridLayout(builder)
        builder_grid.addWidget(QLabel("Point"), 0, 0)
        self.run_point_combo = QComboBox()
        builder_grid.addWidget(self.run_point_combo, 0, 1)
        self.run_point_mode_combo = QComboBox()
        self.run_point_mode_combo.addItem("Joint", "joint")
        self.run_point_mode_combo.addItem("Linear", "linear")
        builder_grid.addWidget(self.run_point_mode_combo, 0, 2)
        self.add_point_step_button = QPushButton("+ Point")
        self.add_point_step_button.clicked.connect(self._add_point_sequence_step)
        builder_grid.addWidget(self.add_point_step_button, 0, 3)

        self.add_home_step_button = QPushButton("+ Home")
        self.add_home_step_button.clicked.connect(
            lambda _checked=False: self._append_sequence_step(SequenceStep("home"))
        )
        builder_grid.addWidget(self.add_home_step_button, 1, 0)
        self.add_rest_step_button = QPushButton("+ Rest")
        self.add_rest_step_button.clicked.connect(
            lambda _checked=False: self._append_sequence_step(SequenceStep("rest"))
        )
        builder_grid.addWidget(self.add_rest_step_button, 1, 1)

        self.sequence_gripper_spin = self._spin(
            0.0, 1.0, 0.0, decimals=3, step=0.05
        )
        builder_grid.addWidget(self.sequence_gripper_spin, 1, 2)
        self.add_gripper_step_button = QPushButton("+ Gripper")
        self.add_gripper_step_button.clicked.connect(self._add_gripper_sequence_step)
        builder_grid.addWidget(self.add_gripper_step_button, 1, 3)

        self.sequence_wait_spin = self._spin(
            0.0, 60.0, 0.25, decimals=2, step=0.25, suffix=" s"
        )
        builder_grid.addWidget(self.sequence_wait_spin, 2, 0)
        self.add_wait_step_button = QPushButton("+ Wait")
        self.add_wait_step_button.clicked.connect(self._add_wait_sequence_step)
        builder_grid.addWidget(self.add_wait_step_button, 2, 1)

        self.run_trajectory_combo = QComboBox()
        builder_grid.addWidget(self.run_trajectory_combo, 2, 2)
        self.add_trajectory_step_button = QPushButton("+ Trajectory")
        self.add_trajectory_step_button.clicked.connect(self._add_trajectory_sequence_step)
        builder_grid.addWidget(self.add_trajectory_step_button, 2, 3)

        self.run_primitive_combo = QComboBox()
        builder_grid.addWidget(self.run_primitive_combo, 3, 2)
        self.add_primitive_step_button = QPushButton("+ Primitive")
        self.add_primitive_step_button.clicked.connect(self._add_primitive_sequence_step)
        builder_grid.addWidget(self.add_primitive_step_button, 3, 3)
        layout.addWidget(builder)

        self.sequence_step_list = QListWidget()
        layout.addWidget(self.sequence_step_list, 1)

        edit_row = QHBoxLayout()
        self.sequence_up_button = QPushButton("Move up")
        self.sequence_up_button.clicked.connect(lambda _checked=False: self._move_sequence_step(-1))
        edit_row.addWidget(self.sequence_up_button)
        self.sequence_down_button = QPushButton("Move down")
        self.sequence_down_button.clicked.connect(lambda _checked=False: self._move_sequence_step(1))
        edit_row.addWidget(self.sequence_down_button)
        self.sequence_delete_step_button = QPushButton("Delete step")
        self.sequence_delete_step_button.clicked.connect(self._delete_sequence_step)
        edit_row.addWidget(self.sequence_delete_step_button)
        edit_row.addStretch(1)
        layout.addLayout(edit_row)

        run_box = QGroupBox("Execute")
        run_grid = QGridLayout(run_box)
        run_grid.addWidget(QLabel("Repeat"), 0, 0)
        self.sequence_repeat_spin = QSpinBox()
        self.sequence_repeat_spin.setRange(1, 1000)
        self.sequence_repeat_spin.setValue(1)
        run_grid.addWidget(self.sequence_repeat_spin, 0, 1)
        run_grid.addWidget(QLabel("Speed"), 0, 2)
        self.sequence_speed_spin = self._spin(
            0.1, 3.0, 1.0, decimals=2, step=0.1, suffix="×"
        )
        run_grid.addWidget(self.sequence_speed_spin, 0, 3)
        self.run_step_button = QPushButton("Run selected step")
        self.run_step_button.clicked.connect(lambda _checked=False: self._run_sequence(step_only=True))
        run_grid.addWidget(self.run_step_button, 1, 0, 1, 2)
        self.run_sequence_button = QPushButton("Run sequence")
        self.run_sequence_button.clicked.connect(lambda _checked=False: self._run_sequence(step_only=False))
        run_grid.addWidget(self.run_sequence_button, 1, 2)
        self.pause_sequence_button = QPushButton("Pause after current step")
        self.pause_sequence_button.clicked.connect(self._toggle_sequence_pause)
        run_grid.addWidget(self.pause_sequence_button, 1, 3)
        self.stop_sequence_button = QPushButton("STOP / HOLD")
        self.stop_sequence_button.clicked.connect(lambda _checked=False: self.stop_requested.emit())
        run_grid.addWidget(self.stop_sequence_button, 2, 3)
        self.sequence_status = QLabel("No sequence running")
        run_grid.addWidget(self.sequence_status, 2, 0, 1, 3)
        layout.addWidget(run_box)
        return page

    def _build_joint_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        box = QGroupBox("Absolute joint targets")
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

        layout.addWidget(box)

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
        layout = QVBoxLayout(page)

        current_box = QGroupBox("Measured TCP in robot world/base frame")
        current_grid = QGridLayout(current_box)
        self.pose_value_labels: list[QLabel] = []
        for column, label in enumerate(("X mm", "Y mm", "Z mm", "Roll °", "Pitch °", "Yaw °")):
            current_grid.addWidget(QLabel(label), 0, column)
            value = QLabel("—")
            value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            value.setMinimumWidth(90)
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
        self.linear_step = self._spin(0.1, 50.0, 5.0, decimals=1, step=1.0, suffix=" mm")
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
            "Tool-frame XYZ follows the current gripper axes. Every jog is planned as a "
            "Cartesian linear path; it is not a raw servo jump."
        )
        help_label.setWordWrap(True)
        jog_grid.addWidget(help_label, 5, 0, 1, 6)
        layout.addWidget(jog_box)
        layout.addStretch(1)
        return page

    def _build_gripper_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        box = QGroupBox("Stock gripper")
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
        grid.addWidget(QLabel("0 = closed, 1 = open"), 0, 0, 1, 3)
        grid.addWidget(self.gripper_spin, 1, 0)
        grid.addWidget(self.gripper_slider, 1, 1, 1, 2)

        self.close_gripper_button = QPushButton("Close")
        self.close_gripper_button.clicked.connect(lambda: self.gripper_requested.emit(0.0))
        self.move_gripper_button = QPushButton("Move to value")
        self.move_gripper_button.clicked.connect(
            lambda: self.gripper_requested.emit(self.gripper_spin.value())
        )
        self.open_gripper_button = QPushButton("Open")
        self.open_gripper_button.clicked.connect(lambda: self.gripper_requested.emit(1.0))
        grid.addWidget(self.close_gripper_button, 2, 0)
        grid.addWidget(self.move_gripper_button, 2, 1)
        grid.addWidget(self.open_gripper_button, 2, 2)
        self.gripper_measured = QLabel("Measured: —")
        grid.addWidget(self.gripper_measured, 3, 0, 1, 3)
        layout.addWidget(box)
        layout.addStretch(1)
        return page

    def _set_gripper_spin(self, value: float) -> None:
        self.gripper_spin.blockSignals(True)
        self.gripper_spin.setValue(value)
        self.gripper_spin.blockSignals(False)

    def _set_gripper_slider(self, value: float) -> None:
        self.gripper_slider.blockSignals(True)
        self.gripper_slider.setValue(round(value * 1000.0))
        self.gripper_slider.blockSignals(False)

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
        self.connect_requested.emit(
            {
                "simulation": simulation,
                "port": port,
                "robot_id": self.robot_id_edit.text().strip() or "so101",
                "allow_uncalibrated": self.allow_uncalibrated_check.isChecked(),
            }
        )

    def _on_robot_id_changed(self) -> None:
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
        self.leader_connect_requested.emit(
            {
                "simulation": simulation,
                "port": port,
                "robot_id": self.leader_robot_id_edit.text().strip() or "so101-leader",
            }
        )

    def _start_calibration(self) -> None:
        if not self._connected:
            QMessageBox.warning(self, "Follower not connected", "Connect the follower first.")
            return
        if self.simulation_check.isChecked():
            QMessageBox.information(
                self,
                "Hardware calibration only",
                "Simulation has no raw encoders or mechanical stops to calibrate.",
            )
            return
        answer = QMessageBox.question(
            self,
            "Start calibration",
            "Torque will be disabled. Sweep every joint and the gripper repeatedly "
            "between both mechanical stops for the full recording period. Continue?",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self.calibration_status.setText("Calibration recording in progress…")
        self.calibration_requested.emit(self.calibration_duration.value())

    def _on_calibration_completed(self, result: object) -> None:
        values = dict(result)  # type: ignore[arg-type]
        self.calibration_status.setText(
            f"Calibration complete ({values.get('source', 'unknown')}); "
            f"saved to {values.get('path', 'unknown path')}."
        )
        limits = values.get("joint_limits_deg")
        if limits:
            self._apply_joint_limits(dict(limits))
        self._log("Mechanical-stop midpoint calibration completed.")

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
        )

    def _save_named_pose(self, name: str) -> None:
        if not self._latest_state:
            self._on_error("Cannot save pose: follower state is unavailable.")
            return
        try:
            pose = self._saved_pose_from_state(self._latest_state, source="follower")
            path = self._get_pose_library().save(name, pose)
            self._log(f"Saved {name} pose to {path}.")
            self._refresh_named_pose_status()
            self._update_enabled_state()
        except Exception as exc:
            self._on_error(f"Save {name}: {exc}")

    def _go_named_pose(self, name: str) -> None:
        try:
            pose = self._get_pose_library().require(name)
        except Exception as exc:
            self._on_error(f"Load {name}: {exc}")
            return
        for joint, value in pose.joints.items():
            self.joint_spins[joint].setValue(degrees(value))
        self.gripper_spin.setValue(pose.gripper)
        self._move_joints()
        self.gripper_requested.emit(pose.gripper)
        self._log(f"Moving to saved {name} pose.")

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
        self._update_enabled_state()

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
        if state is None:
            self._on_error(f"Cannot save point: {source} state is unavailable.")
            return
        try:
            pose = self._saved_pose_from_state(state, source=source)
            path = self._get_pose_library().save(name, pose)
            self._log(f"Saved taught point {name!r} from {source} to {path}.")
            self.point_name_edit.clear()
            self._refresh_point_list()
            self.point_combo.setCurrentText(name)
        except Exception as exc:
            self._on_error(f"Save taught point: {exc}")

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
            self.tabs.setCurrentIndex(3)
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
        diagnostics = " + effort" if trajectory.effort_current_raw is not None else ""
        self.trajectory_stats.setText(
            f"{trajectory.metadata.get('kind', 'unsaved')} / "
            f"{trajectory.metadata.get('name', 'recording')} — "
            f"{trajectory.sample_count} samples, {duration:.3f} s, "
            f"median {trajectory.sample_rate_hz:.1f} Hz{diagnostics}"
        )
        self._selection_changed()
        self._update_enabled_state()

    def _trajectory_scrub_changed(self, value: int) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return
        cursor = trajectory.duration_s * float(value) / 10000.0
        self.trajectory_timeline.set_cursor(cursor)
        self.trajectory_cursor_label.setText(f"{cursor:.3f} s")

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
            clip = self._selection_clip().retime(self.trajectory_speed_scale.value())
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
        if step.kind == "point":
            return f"POINT {params.get('name')} [{params.get('mode', 'joint')}]"
        if step.kind in {"home", "rest"}:
            return step.kind.upper()
        if step.kind == "gripper":
            return f"GRIPPER {float(params.get('position', 0.0)):.3f}"
        if step.kind == "wait":
            return f"WAIT {float(params.get('seconds', 0.0)):.2f} s"
        if step.kind == "trajectory":
            return (
                f"TRAJECTORY {params.get('kind', 'edited')}:{params.get('name')} "
                f"×{int(params.get('loops', 1))}"
            )
        if step.kind == "primitive":
            return f"PRIMITIVE {params.get('name')} ×{int(params.get('loops', 1))}"
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
        self._update_enabled_state()

    def _append_sequence_step(self, step: SequenceStep) -> None:
        self._sequence_steps.append(step)
        self._refresh_sequence_step_list()
        self.sequence_step_list.setCurrentRow(len(self._sequence_steps) - 1)

    def _add_point_sequence_step(self) -> None:
        name = self.run_point_combo.currentText().strip()
        if name:
            self._append_sequence_step(
                SequenceStep(
                    "point",
                    {"name": name, "mode": self.run_point_mode_combo.currentData()},
                )
            )

    def _add_gripper_sequence_step(self) -> None:
        self._append_sequence_step(
            SequenceStep("gripper", {"position": self.sequence_gripper_spin.value()})
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
                SequenceStep("trajectory", {"kind": kind, "name": name, "loops": 1})
            )

    def _add_primitive_sequence_step(self) -> None:
        name = self.run_primitive_combo.currentText().strip()
        if name:
            self._append_sequence_step(
                SequenceStep("primitive", {"name": name, "loops": 1})
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
            QMessageBox.warning(self, "Sequence name required", "Enter a sequence name.")
            return
        if not self._sequence_steps:
            QMessageBox.warning(self, "No steps", "Add at least one sequence step.")
            return
        try:
            sequence = MotionSequence(name, tuple(self._sequence_steps))
            path = self._get_sequence_library().save(sequence)
            self._log(f"Saved sequence {name!r} to {path}.")
            self._refresh_sequence_list()
            self.sequence_combo.setCurrentText(name)
        except Exception as exc:
            self._on_error(f"Save sequence: {exc}")

    def _load_sequence(self) -> None:
        name = self.sequence_combo.currentText().strip()
        if not name:
            return
        try:
            sequence = self._get_sequence_library().require(name)
            self._sequence_steps = list(sequence.steps)
            self.sequence_name_edit.setText(sequence.name)
            self._refresh_sequence_step_list()
            self._log(f"Loaded sequence {name!r}.")
        except Exception as exc:
            self._on_error(f"Load sequence: {exc}")

    def _delete_sequence(self) -> None:
        name = self.sequence_combo.currentText().strip()
        if not name:
            return
        try:
            self._get_sequence_library().delete(name)
            self._log(f"Deleted sequence {name!r}.")
            self._refresh_sequence_list()
        except Exception as exc:
            self._on_error(f"Delete sequence: {exc}")

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
                    "start_index": start_index,
                    "stop_index": stop_index,
                }
            )
            self.sequence_status.setText(
                f"Running {'step ' + str(start_index + 1) if step_only else sequence.name}…"
            )
        except Exception as exc:
            self._on_error(f"Run sequence: {exc}")

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
                "paused": "Sequence paused",
                "running": "Sequence running",
                "completed": "Sequence complete",
                "failed": "Sequence stopped with an error",
            }
            self.sequence_status.setText(labels.get(status, f"Sequence {status}"))
            return
        if values.get("type") == "teleop":
            if self._teleop_active:
                self.teleop_status.setText(
                    f"Live teleop active — {int(values.get('samples', 0))} samples."
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

    def _toggle_teleop(self) -> None:
        if self._teleop_active:
            self.teleop_stop_requested.emit()
            return
        if not self._connected or not self._torque_enabled:
            QMessageBox.warning(
                self,
                "Follower not ready",
                "Connect and enable the follower before starting live teleoperation.",
            )
            return
        if not self._leader_connected or self._latest_leader_state is None:
            QMessageBox.warning(
                self,
                "Leader not ready",
                "Connect the leader/controller arm before starting live teleoperation.",
            )
            return
        leader = self._latest_leader_state
        self.teleop_start_requested.emit(
            {
                "mode": self.teleop_mode_combo.currentData(),
                "leader_joints_rad": {
                    name: radians(float(leader["joints_deg"][name]))
                    for name in ARM_JOINTS
                },
                "leader_gripper": float(leader["gripper"]),
                "mirror_gripper": self.teleop_gripper_check.isChecked(),
            }
        )
        self.teleop_status.setText("Starting guarded 50 Hz live teleoperation…")

    def _on_teleop_changed(self, active: bool) -> None:
        self._teleop_active = bool(active)
        if active:
            self.teleop_button.setText("Stop live teleop")
            self.teleop_status.setText(
                f"Live teleop active — {self.teleop_mode_combo.currentText()}."
            )
            self.leader_stream_start_requested.emit(50.0)
        else:
            self.teleop_button.setText("Start live teleop")
            self.leader_stream_stop_requested.emit()
            self.teleop_status.setText("Live teleoperation stopped / follower holding.")
        self._update_enabled_state()

    def _on_leader_stream_readout_changed(self, active: bool) -> None:
        if not active and self._teleop_active:
            self.teleop_stop_requested.emit()

    def _on_leader_connected(self, connected: bool) -> None:
        self._leader_connected = connected
        if not connected:
            if self._teleop_active:
                self.teleop_stop_requested.emit()
            self._latest_leader_state = None
        self.leader_connect_button.setText("Disconnect leader" if connected else "Connect leader")
        self._update_teach_readout()
        self._update_enabled_state()

    def _on_leader_state(self, state: object) -> None:
        self._latest_leader_state = dict(state)  # type: ignore[arg-type]
        self._update_teach_readout()

    def _update_teach_readout(self) -> None:
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
        self._joint_targets_initialized = True
        self._load_current_pose()
        self.gripper_spin.setValue(float(state["gripper"]))

    def _load_current_pose(self) -> None:
        state = self._latest_state
        if not state:
            return
        for spin, value in zip(self.absolute_spins, state["pose_mm_deg"], strict=True):
            spin.setValue(float(value))

    def _on_connected(self, connected: bool) -> None:
        self._connected = connected
        if not connected:
            self._torque_enabled = False
            self._busy = False
            self._joint_targets_initialized = False
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self._refresh_named_pose_status()
        self._refresh_point_list()
        self._update_enabled_state()

    def _on_busy(self, busy: bool) -> None:
        self._busy = busy
        if not busy:
            self._sequence_paused = False
            if hasattr(self, "pause_sequence_button"):
                self.pause_sequence_button.setText("Pause after current step")
        self._update_enabled_state()

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
        self.pose_summary.setText(
            f"TCP: X {pose[0]:.1f}  Y {pose[1]:.1f}  Z {pose[2]:.1f} mm"
        )
        gripper = float(values["gripper"])
        self.gripper_measured.setText(f"Measured: {gripper:.3f}")

        if not self._joint_targets_initialized:
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

    def _update_enabled_state(self) -> None:
        simulation = self.simulation_check.isChecked()
        session_editable = not self._connected
        self.simulation_check.setEnabled(session_editable)
        self.port_combo.setEnabled(session_editable and not simulation)
        self.refresh_ports_button.setEnabled(session_editable and not simulation)
        self.robot_id_edit.setEnabled(session_editable)
        self.allow_uncalibrated_check.setEnabled(session_editable and not simulation)

        self.enable_button.setEnabled(self._connected and not self._torque_enabled and not self._busy)
        self.relax_button.setEnabled(self._connected and self._torque_enabled)
        self.stop_button.setEnabled(self._connected)
        self.read_targets_button.setEnabled(self._connected and not self._busy)

        can_move = self._connected and self._torque_enabled and not self._busy
        self.move_joints_button.setEnabled(can_move)
        self.absolute_move_button.setEnabled(can_move)
        self.use_current_pose_button.setEnabled(self._connected and not self._busy)
        for button in self.jog_buttons:
            button.setEnabled(can_move)
        self.close_gripper_button.setEnabled(can_move)
        self.move_gripper_button.setEnabled(can_move)
        self.open_gripper_button.setEnabled(can_move)

        self.run_calibration_button.setEnabled(
            self._connected and not simulation and not self._busy
        )

        can_save_pose = self._connected and self._latest_state is not None and not self._busy
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

        leader_editable = not self._leader_connected
        self.leader_simulation_check.setEnabled(leader_editable)
        self.leader_port_combo.setEnabled(
            leader_editable and not self.leader_simulation_check.isChecked()
        )
        self.leader_refresh_button.setEnabled(
            leader_editable and not self.leader_simulation_check.isChecked()
        )
        self.leader_robot_id_edit.setEnabled(leader_editable)

        source, source_state = self._current_teaching_state()
        self.save_point_button.setEnabled(
            source_state is not None and self._recording_source is None
        )
        has_point = bool(self.point_combo.currentText())
        self.move_point_button.setEnabled(can_move and has_point)
        self.delete_point_button.setEnabled(has_point and self._recording_source is None)

        source_available = source_state is not None
        self.record_button.setEnabled(
            self._recording_source is not None
            or (source_available and not self._busy and not self._teleop_active)
        )

        can_start_teleop = (
            self._connected
            and self._torque_enabled
            and self._leader_connected
            and self._latest_leader_state is not None
            and self._recording_source is None
            and not self._busy
        )
        self.teleop_button.setEnabled(self._teleop_active or can_start_teleop)
        self.teleop_mode_combo.setEnabled(not self._teleop_active)
        self.teleop_gripper_check.setEnabled(not self._teleop_active)
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
        self.add_wait_step_button.setEnabled(not self._busy)
        self.add_trajectory_step_button.setEnabled(
            self.run_trajectory_combo.count() > 0 and not self._busy
        )
        self.add_primitive_step_button.setEnabled(
            self.run_primitive_combo.count() > 0 and not self._busy
        )

    def _on_error(self, message: str) -> None:
        self._log(message)
        self.statusBar().showMessage(message, 8000)

    def _log(self, message: str) -> None:
        self.log.append(message)

    def closeEvent(self, event: QCloseEvent) -> None:
        try:
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
