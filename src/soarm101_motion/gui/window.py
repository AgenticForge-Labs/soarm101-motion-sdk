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
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from soarm101_motion.constants import ARM_JOINTS, JOINT_LIMITS
from soarm101_motion.gui.worker import RobotWorker
from soarm101_motion.hardware import FeetechBackend
from soarm101_motion.poses import HOME_POSE_NAME, REST_POSE_NAME, PoseLibrary, SavedPose


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

        self._worker.state_changed.connect(self._on_state)
        self._worker.connected_changed.connect(self._on_connected)
        self._worker.busy_changed.connect(self._on_busy)
        self._worker.log_message.connect(self._log)
        self._worker.error_message.connect(self._on_error)
        self._worker.calibration_completed.connect(self._on_calibration_completed)

        self._leader_thread = QThread(self)
        self._leader_worker = RobotWorker()
        self._leader_worker.moveToThread(self._leader_thread)
        self._leader_thread.started.connect(self._leader_worker.start)
        self._leader_thread.finished.connect(self._leader_worker.deleteLater)
        self.leader_connect_requested.connect(self._leader_worker.connect_robot)
        self.leader_disconnect_requested.connect(self._leader_worker.disconnect_robot)
        self._leader_worker.state_changed.connect(self._on_leader_state)
        self._leader_worker.connected_changed.connect(self._on_leader_connected)
        self._leader_worker.log_message.connect(lambda message: self._log(f"Leader: {message}"))
        self._leader_worker.error_message.connect(
            lambda message: self._on_error(f"Leader: {message}")
        )

        self._build_ui(port=port, robot_id=robot_id, simulation=simulation)
        self._thread.start()
        self._leader_thread.start()
        self._refresh_ports()
        self._refresh_leader_ports()
        self._refresh_named_pose_status()
        self._update_enabled_state()

    def _build_ui(self, *, port: str | None, robot_id: str, simulation: bool) -> None:
        root = QWidget(self)
        layout = QVBoxLayout(root)
        layout.addWidget(self._build_connection_bar(port, robot_id, simulation))

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_setup_tab(), "Setup")
        self.tabs.addTab(self._build_control_tab(), "Control")
        self.tabs.addTab(self._build_teach_tab(), "Teach")
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
        layout.addStretch(1)
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
        self._refresh_named_pose_status()
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

    def _on_leader_connected(self, connected: bool) -> None:
        self._leader_connected = connected
        if not connected:
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
        self._update_enabled_state()

    def _on_busy(self, busy: bool) -> None:
        self._busy = busy
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
