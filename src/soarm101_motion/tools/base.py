"""End-effector and tool abstractions."""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from typing import Mapping

from soarm101_motion.types import Pose


class RobotTool(ABC):
    name: str

    @property
    def tcp_frames(self) -> Mapping[str, Pose]:
        return {}


@dataclass
class PassiveTool(RobotTool):
    name: str
    frames: Mapping[str, Pose] = field(default_factory=dict)

    @property
    def tcp_frames(self) -> Mapping[str, Pose]:
        return self.frames
