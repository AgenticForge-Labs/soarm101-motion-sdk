"""Semantic motion primitives backed by saved trajectories."""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


@dataclass(frozen=True)
class MotionPrimitive:
    name: str
    trajectory_name: str
    trajectory_kind: str = "edited"
    tags: tuple[str, ...] = ()
    description: str = ""
    loopable: bool = False
    interruptible: bool = True
    default_speed_scale: float = 1.0
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not _SAFE_NAME.fullmatch(self.name):
            raise ValueError("primitive name contains unsupported characters")
        if not _SAFE_NAME.fullmatch(self.trajectory_name):
            raise ValueError("trajectory name contains unsupported characters")
        if self.trajectory_kind not in {"raw", "edited"}:
            raise ValueError("trajectory_kind must be 'raw' or 'edited'")
        speed = float(self.default_speed_scale)
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("default_speed_scale must be positive and finite")
        object.__setattr__(self, "default_speed_scale", speed)
        object.__setattr__(self, "tags", tuple(str(tag).strip() for tag in self.tags if str(tag).strip()))
        object.__setattr__(self, "metadata", dict(self.metadata))

    def to_mapping(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "trajectory_name": self.trajectory_name,
            "trajectory_kind": self.trajectory_kind,
            "tags": list(self.tags),
            "description": self.description,
            "loopable": self.loopable,
            "interruptible": self.interruptible,
            "default_speed_scale": self.default_speed_scale,
            "metadata": self.metadata,
        }

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "MotionPrimitive":
        return cls(
            name=str(data["name"]),
            trajectory_name=str(data["trajectory_name"]),
            trajectory_kind=str(data.get("trajectory_kind", "edited")),
            tags=tuple(str(value) for value in data.get("tags", [])),
            description=str(data.get("description", "")),
            loopable=bool(data.get("loopable", False)),
            interruptible=bool(data.get("interruptible", True)),
            default_speed_scale=float(data.get("default_speed_scale", 1.0)),
            metadata=dict(data.get("metadata", {})),
        )


def default_primitive_library_path(robot_id: str) -> Path:
    return Path.home() / ".config" / "soarm101" / "primitives" / f"{robot_id}.json"


class MotionPrimitiveLibrary:
    schema_version = 1

    def __init__(self, robot_id: str, *, path: str | Path | None = None) -> None:
        self.robot_id = str(robot_id).strip() or "so101"
        self.path = Path(path).expanduser() if path is not None else default_primitive_library_path(self.robot_id)
        self._items: dict[str, MotionPrimitive] = {}
        self.reload()

    def reload(self) -> None:
        self._items = {}
        if not self.path.is_file():
            return
        data = json.loads(self.path.read_text(encoding="utf-8"))
        self._items = {
            str(name): MotionPrimitive.from_mapping(dict(value))
            for name, value in data.get("primitives", {}).items()
        }

    def _write(self) -> Path:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": self.schema_version,
            "robot_id": self.robot_id,
            "primitives": {
                name: primitive.to_mapping()
                for name, primitive in sorted(self._items.items())
            },
        }
        self.path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return self.path

    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._items))

    def get(self, name: str) -> MotionPrimitive | None:
        return self._items.get(str(name).strip())

    def require(self, name: str) -> MotionPrimitive:
        key = str(name).strip()
        primitive = self.get(key)
        if primitive is None:
            raise KeyError(f"motion primitive {key!r} does not exist")
        return primitive

    def save(self, primitive: MotionPrimitive) -> Path:
        self._items[primitive.name] = primitive
        return self._write()

    def delete(self, name: str) -> Path:
        key = str(name).strip()
        if key not in self._items:
            raise KeyError(key)
        del self._items[key]
        return self._write()
