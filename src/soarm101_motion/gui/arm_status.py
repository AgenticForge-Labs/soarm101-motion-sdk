"""Reusable modern robot-status card for GUI motion workspaces."""

from __future__ import annotations

from math import degrees
from typing import Any, Mapping

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.gui.cartesian_view import CartesianArmView


class RobotStatusPanel(QWidget):
    """Compact live/preview card built around the shared kinematic renderer."""

    def __init__(
        self,
        title: str,
        *,
        subtitle: str = "",
        compact: bool = False,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._title_text = title
        self._subtitle_text = subtitle
        self._compact = bool(compact)

        self.setObjectName("robotStatusPanel")
        self.setStyleSheet(
            """
            QWidget#robotStatusPanel {
                border: 1px solid palette(midlight);
                border-radius: 14px;
                background: palette(base);
            }
            QLabel#robotStatusTitle {
                font-size: 15px;
                font-weight: 700;
            }
            QLabel#robotStatusSubtitle {
                color: palette(mid);
            }
            QLabel#robotStatusChip {
                padding: 3px 8px;
                border-radius: 9px;
                font-weight: 700;
                background: palette(alternate-base);
            }
            QLabel#robotStatusMetric {
                padding: 2px 0;
                color: palette(text);
            }
            """
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(7)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        self.title_label = QLabel(title)
        self.title_label.setObjectName("robotStatusTitle")
        title_box.addWidget(self.title_label)
        self.subtitle_label = QLabel(subtitle)
        self.subtitle_label.setObjectName("robotStatusSubtitle")
        self.subtitle_label.setVisible(bool(subtitle))
        title_box.addWidget(self.subtitle_label)
        header.addLayout(title_box, 1)

        self.status_chip = QLabel("OFFLINE")
        self.status_chip.setObjectName("robotStatusChip")
        self.status_chip.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header.addWidget(self.status_chip, 0, Qt.AlignmentFlag.AlignTop)
        layout.addLayout(header)

        self.view = CartesianArmView()
        if compact:
            self.view.setMinimumSize(210, 210)
            self.view.setMaximumHeight(300)
        else:
            self.view.setMinimumSize(300, 300)
        layout.addWidget(self.view, 1)

        metrics = QHBoxLayout()
        metrics.setSpacing(14)
        self.tcp_label = QLabel("TCP  —")
        self.tcp_label.setObjectName("robotStatusMetric")
        self.gripper_label = QLabel("Gripper  —")
        self.gripper_label.setObjectName("robotStatusMetric")
        metrics.addWidget(self.tcp_label, 1)
        metrics.addWidget(self.gripper_label)
        layout.addLayout(metrics)

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

    def clear_state(self, *, label: str = "OFFLINE") -> None:
        self.status_chip.setText(label)
        self.tcp_label.setText("TCP  —")
        self.gripper_label.setText("Gripper  —")

    def update_state(self, state: Mapping[str, Any] | None) -> None:
        if not state:
            self.clear_state()
            return

        joints = {name: float(state["joints_deg"][name]) for name in ARM_JOINTS}
        self.view.set_joint_degrees(joints)
        gripper = float(state["gripper"])
        self.view.set_gripper_position(gripper)

        pose = tuple(float(value) for value in state["pose_mm_deg"])
        self.tcp_label.setText(
            f"TCP  {pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f} mm"
        )
        self.gripper_label.setText(f"Gripper  {gripper:.3f}")

        if bool(state.get("moving")):
            chip = "MOVING"
        elif bool(state.get("torque_enabled")):
            chip = "HOLDING"
        elif bool(state.get("connected")):
            chip = "FREE"
        else:
            chip = "OFFLINE"
        if bool(state.get("simulation")):
            chip += " · SIM"
        self.status_chip.setText(chip)

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
        self.preview_label.setText(f"Preview: {label}")
        self.preview_label.show()

    def clear_secondary(self) -> None:
        self.view.clear_secondary()
        self.preview_label.hide()
