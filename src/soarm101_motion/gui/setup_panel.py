"""Task-oriented setup presentation over the existing window and SDK operations."""

from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from soarm101_motion.gui.window import MainWindow

from PySide6.QtCore import QSettings, QTimer, QUrl, Qt
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from soarm101_motion.calibration import default_calibration_path, resolve_calibration
from soarm101_motion.setup_backup import export_setup, inspect_backup, restore_setup, setup_root


class SetupPanel(QStackedWidget):
    def __init__(self, window: "MainWindow", calibration, diagnostics):
        super().__init__()
        self.controller = window
        self.settings = QSettings("AgenticForge Labs", "Motion Studio")
        self.role = "follower"
        self.cards = {}
        self._calibration_cache: dict[tuple, tuple[bool, str]] = {}
        overview = QWidget()
        layout = QVBoxLayout(overview)
        heading = QLabel("Your arms")
        heading.setStyleSheet("font-size: 22px; font-weight: 700;")
        layout.addWidget(heading)
        intro = QLabel(
            "Saved calibrations are reused. Connect an arm to begin; calibrate only a new or changed setup."
        )
        intro.setWordWrap(True)
        layout.addWidget(intro)
        self.guidance = QLabel()
        self.guidance.setWordWrap(True)
        layout.addWidget(self.guidance)
        self.guided = QCheckBox("Show setup guidance")
        self.guided.setChecked(bool(self.settings.value("setup/guided", True, type=bool)))
        self.guided.toggled.connect(self._set_guided)
        layout.addWidget(self.guided)
        guide = QPushButton("Guide me through setup…")
        guide.clicked.connect(lambda: self.setCurrentIndex(4))
        layout.addWidget(guide)
        for role in ("follower", "leader"):
            box = QGroupBox(
                "Follower — the arm that moves"
                if role == "follower"
                else "Leader — optional hand controller"
            )
            rows = QVBoxLayout(box)
            status = QLabel()
            status.setWordWrap(True)
            rows.addWidget(status)
            actions = QHBoxLayout()
            connect = QPushButton("Connect")
            source = window.connect_button if role == "follower" else window.leader_connect_button
            connect.clicked.connect(source.click)
            connect.setProperty("buttonRole", "primary")
            manage = QPushButton("Manage…")
            manage.clicked.connect(lambda checked=False, r=role: self.open_manage(r))
            calibrate = QPushButton("Set up this arm…")
            calibrate.clicked.connect(lambda checked=False, r=role: self.open_calibration(r))
            actions.addWidget(connect)
            actions.addWidget(calibrate)
            actions.addWidget(manage)
            actions.addStretch()
            rows.addLayout(actions)
            layout.addWidget(box)
            self.cards[role] = (status, connect, calibrate)
        overview_actions = QGridLayout()
        for index, (title, callback) in enumerate(
            (
                ("Find arms", window.find_arms_button.click),
                ("Back up setup…", self.backup),
                ("Restore setup…", self.restore),
                ("Diagnostics / advanced…", lambda: self.setCurrentIndex(3)),
            )
        ):
            button = QPushButton(title)
            button.clicked.connect(callback)
            overview_actions.addWidget(button, index // 2, index % 2)
        layout.addLayout(overview_actions)
        self.next_task = QPushButton("Open Manual workspace")
        self.next_task.clicked.connect(lambda: window.tabs.setCurrentWidget(window.manual_page))
        layout.addWidget(self.next_task)
        layout.addStretch()
        self.addWidget(self._scroll(overview))
        self.addWidget(
            self._page("Calibration — move the arm by hand; motors remain off", calibration)
        )
        management = QWidget()
        manager = QVBoxLayout(management)
        self.manage_details = QLabel()
        self.manage_details.setWordWrap(True)
        self.manage_details.setTextInteractionFlags(
            self.manage_details.textInteractionFlags()
            | Qt.TextInteractionFlag.TextSelectableByMouse
        )
        manager.addWidget(self.manage_details)
        manager.addWidget(window.follower_connection_panel)
        manager.addWidget(window.leader_connection_panel)
        controls = QHBoxLayout()
        for title, callback in (
            ("Calibrate / Recalibrate…", lambda: self.open_calibration(self.role)),
            ("Open calibration folder", self.open_folder),
            ("View calibration history", self.open_history),
        ):
            button = QPushButton(title)
            button.clicked.connect(callback)
            controls.addWidget(button)
        manager.addLayout(controls)
        manager.addStretch()
        self.addWidget(self._page("Device settings", self._scroll(management)))
        self.addWidget(self._page("Diagnostics / advanced", diagnostics))
        wizard = QWidget()
        wizard_layout = QVBoxLayout(wizard)
        self.guide_status = QLabel()
        self.guide_status.setWordWrap(True)
        wizard_layout.addWidget(self.guide_status)
        self.guide_action = QPushButton()
        self.guide_action.setProperty("buttonRole", "primary")
        self.guide_action.clicked.connect(self._guided_action)
        wizard_layout.addWidget(self.guide_action)
        wizard_layout.addStretch()
        self._confirmed_device: tuple[str, str] | None = None
        self.addWidget(self._page("Guided follower setup", wizard))
        self.role = "follower"
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(750)
        self.refresh()

    @staticmethod
    def _scroll(widget):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(widget)
        return scroll

    def _page(self, title, content):
        page = QWidget()
        layout = QVBoxLayout(page)
        row = QHBoxLayout()
        back = QPushButton("Back to Setup")
        back.clicked.connect(self.back)
        row.addWidget(back)
        heading = QLabel(title)
        heading.setWordWrap(True)
        row.addWidget(heading, 1)
        layout.addLayout(row)
        layout.addWidget(content)
        return page

    def back(self):
        if self.controller._active_calibration_target is not None:
            QMessageBox.information(
                self,
                "Calibration in progress",
                "Complete or cancel the sweep before leaving calibration.",
            )
            return
        self.setCurrentIndex(0)
        self.refresh()

    def _set_guided(self, enabled):
        self.settings.setValue("setup/guided", enabled)
        if hasattr(self.controller, "walkthrough_label"):
            self.controller.walkthrough_label.setVisible(enabled)
        self.refresh()

    def _identity(self, role):
        edit = (
            self.controller.robot_id_edit
            if role == "follower"
            else self.controller.leader_robot_id_edit
        )
        return edit.text().strip() or ("so101" if role == "follower" else "so101-leader")

    def calibration_status(self, role):
        identity = self._identity(role)
        # Use the same resolver as the current GUI connection, including LeRobot import.
        try:
            from soarm101_motion.calibration import discover_lerobot_calibration

            paths = (default_calibration_path(identity), discover_lerobot_calibration(identity))
            signature = tuple(
                (str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in paths if p and p.is_file()
            )
            key = (identity, signature)
            if key in self._calibration_cache:
                return self._calibration_cache[key]
            calibration = resolve_calibration(explicit_path=None, robot_id=identity)
            if calibration is None:
                return False, "No saved calibration — set up this arm or restore a backup."
            calibration.validate()
            if calibration.uncalibrated_motors:
                return False, "Factory motor ranges found — calibration is required."
            result = (
                True,
                f"Saved calibration found · {calibration.calibration_id.split(':')[-1][:12]}",
            )
            self._calibration_cache = {key: result}
            return result
        except Exception as exc:
            return False, f"Calibration needs attention: {exc}"

    def refresh(self):
        w = self.controller
        if hasattr(self, "guide_status"):
            description, action_label, action = self._guided_step()
            self.guide_status.setText(description)
            self.guide_action.setText(action_label)
            self.guide_action.setEnabled(
                action != "wait"
                and not w._follower_connecting
                and not w._busy
                and not w._leader_busy
            )
        self.guidance.setVisible(self.guided.isChecked())
        self.guidance.setText(
            "1. Find arms and confirm their assignments under Manage.  2. Connect with motors off.  3. Open Manual or Teleoperation. Missing calibration? Choose Set up this arm."
        )
        for role, (label, connect, calibrate) in self.cards.items():
            follower = role == "follower"
            connected = w._connected if follower else w._leader_connected
            simulation = (
                w.simulation_check.isChecked()
                if follower
                else w.leader_simulation_check.isChecked()
            )
            source = w.connect_button if follower else w.leader_connect_button
            state = w._latest_state if follower else w._latest_leader_state
            valid, detail = self.calibration_status(role)
            if simulation:
                detail = "Simulation — physical calibration is not required."
            elif (
                connected
                and state
                and state.get("calibration_id")
                and state.get("robot_id") == self._identity(role)
                and not (
                    w._follower_setup_session
                    if follower
                    else w.leader_allow_uncalibrated_check.isChecked()
                )
            ):
                detail = f"Calibration loaded · {str(state['calibration_id']).split(':')[-1][:12]}"
            session = w.follower_session_status if follower else w.leader_session_status
            port = w.port_combo if follower else w.leader_port_combo
            label.setText(
                f"{session.text()}\n{port.currentText() or 'No device selected'} · {self._identity(role)}\n{detail}"
            )
            connect.setText(source.text())
            connect.setEnabled(source.isEnabled())
            calibrate.setVisible(not valid and not simulation)
            calibrate.setEnabled(not w._busy and not w._leader_busy)
        self.next_task.setEnabled(w._connected)
        self.next_task.setToolTip(
            "Connect the follower first."
            if not w._connected
            else "Open direct controls; motion still requires Enable hold."
        )

    def _guided_step(self):
        w = self.controller
        device = (w.port_combo.currentText(), self._identity("follower"))
        valid, detail = self.calibration_status("follower")
        if w.simulation_check.isChecked():
            return (
                "Simulation is selected. No physical calibration is needed.",
                "Open Manual" if w._connected else "Connect simulator",
                "manual" if w._connected else "connect",
            )
        if not device[0]:
            return (
                "Step 1 of 4: Find your arms. Discovery reads device information without enabling motors.",
                "Find arms",
                "find",
            )
        if self._confirmed_device != device:
            return (
                f"Step 2 of 4: Confirm the follower is {device[0]} (profile {device[1]}). Use Manage if this assignment is wrong. Voltage is a clue, not a unique hardware identity.",
                "Confirm follower assignment",
                "confirm",
            )
        if not valid or (w._connected and w._follower_setup_session):
            return (
                f"Step 3 of 4: {detail} Open calibration to record hand movement, or restore an existing setup backup from the overview.",
                "Open calibration",
                "calibrate",
            )
        if not w._connected:
            return (
                "Step 3 of 4: Saved calibration found. Connect to load it; motors remain off.",
                "Connect follower (motors off)",
                "connect",
            )
        state = w._latest_state or {}
        if not state.get("calibration_id") or state.get("robot_id") != device[1]:
            return (
                "Waiting for the connected session to report its calibration. Review connection errors if this persists.",
                "Waiting for calibration",
                "wait",
            )
        return (
            "Step 4 of 4: Calibration loaded. Open Manual to begin. Enabling hold and starting motion remain explicit actions. You can add a leader from the Setup overview later.",
            "Open Manual workspace",
            "manual",
        )

    def _guided_action(self):
        w = self.controller
        action = self._guided_step()[2]
        if action == "find":
            w.find_arms_button.click()
        elif action == "confirm":
            self._confirmed_device = (w.port_combo.currentText(), self._identity("follower"))
        elif action == "connect":
            w.connect_button.click()
        elif action == "calibrate":
            self.open_calibration("follower")
        elif action == "manual":
            w.tabs.setCurrentWidget(w.manual_page)
        self.refresh()

    def open_manage(self, role):
        self.role = role
        path = default_calibration_path(self._identity(role))
        self.manage_details.setText(
            f"{role.title()} calibration: {path}\nSaved outside the source repository. Discovery suggests roles; confirm the physical devices before connecting."
        )
        self.setCurrentIndex(2)

    def open_calibration(self, role):
        w = self.controller
        if w._active_calibration_target is not None:
            self.setCurrentIndex(1)
            return
        w.calibration_target_combo.setCurrentIndex(w.calibration_target_combo.findData(role))
        self.setCurrentIndex(1)

    def open_folder(self):
        path = default_calibration_path(self._identity(self.role)).parent
        path.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def open_history(self):
        path = (
            default_calibration_path(self._identity(self.role)).parent
            / "history"
            / self._identity(self.role)
        )
        if not path.exists():
            QMessageBox.information(
                self, "Calibration history", "No saved history for this profile yet."
            )
            return
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def backup(self):
        name, _ = QFileDialog.getSaveFileName(
            self,
            "Back up setup outside the repository",
            str(Path.home() / "soarm101-setup.json"),
            "Setup backup (*.json)",
        )
        if not name:
            return
        try:
            count = export_setup(Path(name))
            QMessageBox.information(
                self,
                "Setup backed up",
                f"Saved {count} files to {name}. Keep a copy on another disk or your backup service.",
            )
        except Exception as exc:
            QMessageBox.warning(self, "Backup failed", str(exc))

    def restore(self):
        w = self.controller
        if (
            w._connected
            or w._leader_connected
            or w._busy
            or w._leader_busy
            or w._follower_connecting
            or w._leader_connecting
        ):
            QMessageBox.warning(
                self,
                "Disconnect before restore",
                "Disconnect both arms and finish active work before restoring setup.",
            )
            return
        name, _ = QFileDialog.getOpenFileName(
            self, "Restore setup", str(Path.home()), "Setup backup (*.json)"
        )
        if not name:
            return
        try:
            files = inspect_backup(Path(name))
            if (
                QMessageBox.question(
                    self,
                    "Restore saved setup?",
                    f"Restore {len(files)} files to {setup_root()}? Existing files will be preserved in restore-history. Restart Motion Studio afterward and confirm device assignments.",
                )
                != QMessageBox.StandardButton.Yes
            ):
                return
            recovery = restore_setup(Path(name))
            QMessageBox.information(
                self,
                "Setup restored",
                f"Previous files preserved under {recovery}. Restart Motion Studio before connecting.",
            )
            # Close without implicitly connecting against stale in-memory settings.
            w.close()
        except Exception as exc:
            QMessageBox.warning(self, "Restore failed", str(exc))
