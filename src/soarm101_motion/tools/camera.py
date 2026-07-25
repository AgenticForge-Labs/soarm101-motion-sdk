"""Motion-only metadata for a camera mounted as a tool."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from soarm101_motion.tools.base import RobotTool
from soarm101_motion.types import Pose


@dataclass(frozen=True)
class CameraTool(RobotTool):
    tcp_transform: Pose
    name: str = "camera"
    mass_kg: float | None = None

    @property
    def tcp_frames(self) -> Mapping[str, Pose]:
        return {"camera": self.tcp_transform, "tool": self.tcp_transform}
