"""Persistent robot-status sidebar used across the desktop GUI."""

from __future__ import annotations

from math import degrees
from typing import Any, Mapping

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QVBoxLayout,
    QWidget,
)

from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.gui.cartesian_view import CartesianArmView
from soarm101_motion.kinematics import DEFAULT_GRIPPER_TCP
from soarm101_motion.types import Pose


class RobotStatusPanel(QWidget):
    """Live follower status with optional contextual ghost overlays.

    The primary arm is always the current follower state. Contextual information from
    Teleoperation, Teach, trajectory editing, or Programs is shown only as a secondary
    ghost overlay so the user's physical follower never disappears from the display.
    """

    def __init__(
        self,
        title: str = "Follower",
        *,
        subtitle: str = "Live robot state",
        compact: bool = False,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._title_text = title
        self._subtitle_text = subtitle
        self._compact = bool(compact)

        self.setObjectName("robotStatusPanel")
        self.setMinimumWidth(300 if compact else 330)
        self.setStyleSheet(
            """
            QWidget#robotStatusPanel {
                border: 1px solid palette(midlight);
                border-radius: 16px;
                background: palette(base);
            }
            QLabel#robotStatusTitle {
                font-size: 17px;
                font-weight: 750;
            }
            QLabel#robotStatusSubtitle {
                color: palette(text);
                font-size: 11px;
            }
            QLabel#robotStatusChip {
                padding: 4px 9px;
                border-radius: 10px;
                font-weight: 700;
                background: palette(alternate-base);
            }
            QLabel#robotSectionTitle {
                font-size: 11px;
                font-weight: 700;
                color: palette(text);
                padding-top: 3px;
            }
            QLabel#robotMetricName {
                color: palette(text);
            }
            QLabel#robotMetricValue {
                font-family: monospace;
                font-weight: 650;
            }
            QLabel#robotContext {
                padding: 7px 9px;
                border-radius: 10px;
                background: palette(alternate-base);
            }
            """
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(13, 12, 13, 12)
        layout.setSpacing(8)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        self.title_label = QLabel(title)
        self.title_label.setObjectName("robotStatusTitle")
        title_box.addWidget(self.title_label)
        self.subtitle_label = QLabel(subtitle)
        self.subtitle_label.setWordWrap(True)
        self.subtitle_label.setObjectName("robotStatusSubtitle")
        self.subtitle_label.setVisible(bool(subtitle))
        title_box.addWidget(self.subtitle_label)
        header.addLayout(title_box, 1)

        self.status_chip = QLabel("OFFLINE")
        self.status_chip.setObjectName("robotStatusChip")
        self.status_chip.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header.addWidget(self.status_chip, 0, Qt.AlignmentFlag.AlignTop)
        layout.addLayout(header)

        self.context_label = QLabel("Follower state is always shown live when connected.")
        self.context_label.setObjectName("robotContext")
        self.context_label.setWordWrap(True)
        layout.addWidget(self.context_label)

        self.measurement_label = QLabel("Illustration — not live")
        self.measurement_label.setWordWrap(True)
        layout.addWidget(self.measurement_label)
        self._has_measurement = False
        self.view = CartesianArmView()
        self.view.setMinimumSize(310 if not compact else 270, 310 if not compact else 270)
        self.view.setMaximumHeight(430 if not compact else 300)
        layout.addWidget(self.view, 1)

        pose_title = QLabel("ESTIMATED TOOL POSE · MODEL FRAME")
        pose_title.setObjectName("robotSectionTitle")
        pose_title.setWordWrap(True)
        layout.addWidget(pose_title)

        pose_grid = QGridLayout()
        pose_grid.setHorizontalSpacing(10)
        pose_grid.setVerticalSpacing(3)
        self.pose_value_labels: dict[str, QLabel] = {}
        for row, (name, unit) in enumerate(
            (
                ("X", "mm"),
                ("Y", "mm"),
                ("Z", "mm"),
                ("Roll", "°"),
                ("Pitch", "°"),
                ("Yaw", "°"),
            )
        ):
            name_label = QLabel(name)
            name_label.setObjectName("robotMetricName")
            value_label = QLabel(f"— {unit}")
            value_label.setObjectName("robotMetricValue")
            self.pose_value_labels[name.lower()] = value_label
            column = 0 if row < 3 else 2
            display_row = row if row < 3 else row - 3
            pose_grid.addWidget(name_label, display_row, column)
            pose_grid.addWidget(value_label, display_row, column + 1)
        layout.addLayout(pose_grid)

        joints_title = QLabel("JOINTS")
        joints_title.setObjectName("robotSectionTitle")
        layout.addWidget(joints_title)

        joint_grid = QGridLayout()
        joint_grid.setHorizontalSpacing(8)
        joint_grid.setVerticalSpacing(2)
        self.joint_value_labels: dict[str, QLabel] = {}
        for row, name in enumerate(ARM_JOINTS):
            label = QLabel(name.replace("_", " ").title())
            label.setObjectName("robotMetricName")
            value = QLabel("—°")
            value.setObjectName("robotMetricValue")
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.joint_value_labels[name] = value
            joint_grid.addWidget(label, row, 0)
            joint_grid.addWidget(value, row, 1)
        layout.addLayout(joint_grid)

        tool_title = QLabel("TOOL")
        tool_title.setObjectName("robotSectionTitle")
        layout.addWidget(tool_title)
        gripper_row = QHBoxLayout()
        self.gripper_label = QLabel("Gripper")
        self.gripper_label.setObjectName("robotMetricName")
        gripper_row.addWidget(self.gripper_label)
        self.gripper_bar = QProgressBar()
        self.gripper_bar.setRange(0, 1000)
        self.gripper_bar.setValue(0)
        self.gripper_bar.setFormat("—")
        self.gripper_bar.setTextVisible(True)
        gripper_row.addWidget(self.gripper_bar, 1)
        layout.addLayout(gripper_row)

        self.preview_label = QLabel()
        self.preview_label.setObjectName("robotStatusSubtitle")
        self.preview_label.setWordWrap(True)
        self.preview_label.hide()
        layout.addWidget(self.preview_label)

    def set_title(self, title: str, subtitle: str | None = None) -> None:
        self._title_text = str(title)
        self.title_label.setText(self._title_text)
        if subtitle is not None:
            self._subtitle_text = str(subtitle)
            self.subtitle_label.setText(self._subtitle_text)
            self.subtitle_label.setVisible(bool(self._subtitle_text))

    def set_context(self, text: str) -> None:
        self.context_label.setText(str(text))

    def _set_status_chip(self, text: str, state: str) -> None:
        palette = {
            "fault": ("#7f1d1d", "#ffffff"),
            "moving": ("#854d0e", "#ffffff"),
            "holding": ("#166534", "#ffffff"),
            "free": ("#1d4ed8", "#ffffff"),
            "offline": ("palette(alternate-base)", "palette(text)"),
        }
        background, foreground = palette.get(state, palette["offline"])
        self.status_chip.setText(text)
        self.status_chip.setStyleSheet(
            "padding: 4px 9px; border-radius: 10px; font-weight: 800; "
            f"background: {background}; color: {foreground};"
        )

    def clear_state(self, *, label: str = "OFFLINE") -> None:
        self._set_status_chip(label, "offline")
        self.view.setEnabled(False)
        self.measurement_label.setText(
            "Last measured pose — disconnected"
            if self._has_measurement
            else "Illustration — not live"
        )
        for value in self.pose_value_labels.values():
            value.setText("—")
        for value in self.joint_value_labels.values():
            value.setText("—°")
        self.gripper_bar.setValue(0)
        self.gripper_bar.setFormat("—")
        self.view.clear_target()

    def update_joint_degrees(self, joints_deg: Mapping[str, float]) -> None:
        """Refresh live joint/model state between slower full-state samples."""

        joints = {name: float(joints_deg[name]) for name in ARM_JOINTS}
        self.view.set_joint_degrees(joints)
        for name, angle in joints.items():
            self.joint_value_labels[name].setText(f"{angle:+.1f}°")

    def update_gripper_position(self, position: float) -> None:
        """Refresh the live jaw schematic and aperture readout."""

        gripper = max(0.0, min(1.0, float(position)))
        self.view.set_gripper_position(gripper)
        self.gripper_bar.setValue(round(gripper * 1000))
        self.gripper_bar.setFormat(f"{gripper:.3f}")

    def update_pose_mm_deg(
        self,
        pose_mm_deg: Mapping[str, float] | tuple[float, ...] | list[float],
    ) -> None:
        """Refresh the model-estimated TCP readout from live measured joints."""

        if isinstance(pose_mm_deg, Mapping):
            pose = tuple(
                float(pose_mm_deg[key])
                for key in ("x", "y", "z", "roll", "pitch", "yaw")
            )
        else:
            pose = tuple(float(value) for value in pose_mm_deg)
        if len(pose) != 6:
            raise ValueError("pose_mm_deg must contain six values")
        keys = ("x", "y", "z", "roll", "pitch", "yaw")
        units = ("mm", "mm", "mm", "°", "°", "°")
        for key, value, unit in zip(keys, pose, units, strict=True):
            self.pose_value_labels[key].setText(f"{value:+.1f} {unit}")

    def update_live_measurements(self, values: Mapping[str, Any]) -> None:
        """Apply available live measurements without requiring a full state."""

        if "tcp_xyz_rpy" in values:
            self.view.set_tcp(Pose.from_xyz_rpy(*values["tcp_xyz_rpy"]))
        if "joints_deg" in values:
            self.update_joint_degrees(values["joints_deg"])
        if "pose_mm_deg" in values:
            self.update_pose_mm_deg(values["pose_mm_deg"])
        if "gripper" in values:
            self.update_gripper_position(float(values["gripper"]))

    def update_state(self, state: Mapping[str, Any] | None) -> None:
        if not state:
            self.clear_state()
            return

        self.view.setEnabled(True)
        self._has_measurement = True
        self.measurement_label.setText(
            "Simulated state"
            if state.get("simulation")
            else "Measured joints · model-estimated tool pose"
        )
        joints = {name: float(state["joints_deg"][name]) for name in ARM_JOINTS}
        tcp_values = state.get("tcp_xyz_rpy")
        self.view.set_tcp(
            DEFAULT_GRIPPER_TCP if tcp_values is None else Pose.from_xyz_rpy(*tcp_values)
        )
        self.update_joint_degrees(joints)
        self.update_gripper_position(float(state["gripper"]))
        self.update_pose_mm_deg(state["pose_mm_deg"])

        if bool(state.get("faulted")):
            chip, status_state = "FAULT", "fault"
        elif bool(state.get("moving")):
            chip, status_state = "MOVING", "moving"
        elif bool(state.get("torque_enabled")):
            chip, status_state = "HOLDING", "holding"
        elif bool(state.get("connected")):
            chip, status_state = "RELAXED", "free"
        else:
            chip, status_state = "OFFLINE", "offline"
        if bool(state.get("simulation")):
            chip += " · SIM"
        self._set_status_chip(chip, status_state)

    def show_secondary_state(
        self,
        state: Mapping[str, Any] | None,
        *,
        label: str,
    ) -> None:
        if not state:
            self.clear_secondary()
            return
        self.view.set_secondary_joint_degrees(
            {name: float(state["joints_deg"][name]) for name in ARM_JOINTS},
            gripper=float(state["gripper"]),
            label=label,
        )
        self.preview_label.setText(f"Ghost overlay: {label}")
        self.preview_label.show()

    def show_saved_pose(
        self,
        *,
        joints_rad: Mapping[str, float],
        gripper: float,
        label: str,
    ) -> None:
        self.view.set_secondary_joint_degrees(
            {name: degrees(float(joints_rad[name])) for name in ARM_JOINTS},
            gripper=float(gripper),
            label=label,
        )
        self.preview_label.setText(f"Ghost preview: {label}")
        self.preview_label.show()

    def clear_secondary(self) -> None:
        self.view.clear_secondary()
        self.preview_label.hide()
