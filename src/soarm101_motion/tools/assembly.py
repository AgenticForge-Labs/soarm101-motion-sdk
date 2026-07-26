"""Multiple TCP frames and actuated tools attached to one wrist assembly."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

from soarm101_motion.hardware.base import SO101HardwareBackend
from soarm101_motion.tools.base import RobotTool
from soarm101_motion.types import Pose


@dataclass
class ToolAssembly(RobotTool):
    primary: RobotTool
    attachments: Mapping[str, RobotTool] = field(default_factory=dict)
    name: str = "tool_assembly"

    def bind(self, backend: SO101HardwareBackend) -> None:
        for tool in (self.primary, *self.attachments.values()):
            bind = getattr(tool, "bind", None)
            if callable(bind):
                bind(backend)

    @property
    def is_moving(self) -> bool:
        return any(
            bool(getattr(tool, "is_moving", False))
            for tool in (self.primary, *self.attachments.values())
        )

    def stop(self, *, wait: bool = True) -> None:
        for tool in (self.primary, *self.attachments.values()):
            stop = getattr(tool, "stop", None)
            if callable(stop):
                stop(wait=wait)

    @property
    def tcp_frames(self) -> Mapping[str, Pose]:
        frames: dict[str, Pose] = dict(self.primary.tcp_frames)
        for key, tool in self.attachments.items():
            for frame_name, transform in tool.tcp_frames.items():
                frames[key if frame_name == "tool" else frame_name] = transform
        return frames
