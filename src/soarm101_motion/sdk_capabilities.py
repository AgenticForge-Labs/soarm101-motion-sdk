"""Pure capability metadata and explicit SDK dispatch shared by human and broker adapters.

This layer is an operation contract, not authorization: callers own hardware session,
human confirmation and (for remote agents) the broker's additional runtime policy.
Importing or inspecting the registry never opens hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from math import pi
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Literal, Mapping

if TYPE_CHECKING:
    from soarm101_motion.arm import SOARM101

ValueKind = Literal["number", "numbers", "choice"]
EffectKind = Literal["read", "motion", "stop"]


@dataclass(frozen=True, slots=True)
class ArgumentSpec:
    name: str
    kind: ValueKind
    description: str
    unit: str | None = None
    required: bool = True
    length: int | None = None
    choices: tuple[str, ...] = ()
    default: object = None

    def validate(self, value: object) -> object:
        if self.kind == "number":
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{self.name} must be numeric")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{self.name} must be finite")
        elif self.kind == "numbers":
            if not isinstance(value, (list, tuple)) or (
                self.length is not None and len(value) != self.length
            ):
                raise ValueError(f"{self.name} must contain {self.length} numbers")
            if any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(float(v)) for v in value):
                raise ValueError(f"{self.name} must contain finite numbers")
            value = tuple(float(v) for v in value)
        elif self.kind == "choice":
            if not isinstance(value, str) or value not in self.choices:
                raise ValueError(f"{self.name} must be one of {self.choices!r}")
        else:
            raise AssertionError(f"unsupported registry argument kind: {self.kind}")
        return value

    def describe(self) -> dict[str, object]:
        result: dict[str, object] = {
            "name": self.name, "kind": self.kind, "description": self.description,
            "required": self.required,
        }
        if self.unit is not None:
            result["unit"] = self.unit
        if self.length is not None:
            result["length"] = self.length
        if self.choices:
            result["choices"] = list(self.choices)
        if not self.required:
            result["default"] = self.default
        return result


@dataclass(frozen=True, slots=True)
class ActionSpec:
    name: str
    description: str
    effect: EffectKind
    arguments: tuple[ArgumentSpec, ...] = ()
    # Eligibility is metadata only; it is NOT a grant of robot authority.
    # Physical motion needs broker-side calibrated/profile/lease checks in PR 2.
    agent_eligible: bool = False

    def validate(self, payload: Mapping[str, object]) -> dict[str, object]:
        if not isinstance(payload, Mapping):
            raise ValueError("capability arguments must be an object")
        allowed = {arg.name for arg in self.arguments}
        unknown = set(payload) - allowed
        if unknown:
            raise ValueError(f"unknown {self.name} arguments: {sorted(unknown)!r}")
        validated: dict[str, object] = {}
        for arg in self.arguments:
            if arg.name in payload:
                validated[arg.name] = arg.validate(payload[arg.name])
            elif arg.required:
                raise ValueError(f"missing required argument: {arg.name}")
            else:
                validated[arg.name] = arg.validate(arg.default)
        return validated

    def describe(self) -> dict[str, object]:
        return {
            "name": self.name, "description": self.description,
            "effect": self.effect, "agent_eligible": self.agent_eligible,
            "arguments": [arg.describe() for arg in self.arguments],
        }


def _number(name: str, unit: str, description: str, default: float) -> ArgumentSpec:
    return ArgumentSpec(name, "number", description, unit, required=False, default=default)


def _triplet(name: str, unit: str, description: str) -> ArgumentSpec:
    return ArgumentSpec(name, "numbers", description, unit, length=3)


_ORIENTATION = ArgumentSpec(
    "orientation_mode", "choice", "How strictly to constrain tool orientation",
    required=False, choices=("compatible", "position_only", "exact"),
    default="compatible",
)
_FRAME = ArgumentSpec(
    "frame", "choice", "Reference frame for a Cartesian jog", required=False,
    choices=("world", "tool"), default="world",
)
_SPECS = (
    ActionSpec("read_pose", "Read the calibrated joint positions and model TCP pose", "read",
               agent_eligible=True),
    ActionSpec("solve_ik", "Solve an absolute model TCP target without commanding motion",
               "read", (
                   _triplet("target_xyz_mm", "mm", "Model TCP position"),
                   _triplet("target_rpy_deg", "deg", "Target roll, pitch and yaw"),
                   _ORIENTATION,
               )),
    ActionSpec("move_joints", "Move all five absolute pose joints in radians", "motion", (
        ArgumentSpec("positions_rad", "numbers", "Five pose joint angles", "rad", length=5),
        _number("speed_rad_s", "rad/s", "Requested joint speed; zero means SDK default", 0.0),
        _number("acceleration_rad_s2", "rad/s^2",
                "Requested joint acceleration; zero means SDK default", 0.0),
    )),
    ActionSpec("move_linear", "Move to a validated absolute model TCP pose", "motion", (
        _triplet("target_xyz_mm", "mm", "Model TCP position"),
        _triplet("target_rpy_deg", "deg", "Target roll, pitch and yaw"),
        _ORIENTATION,
        _number("speed_mm_s", "mm/s", "Requested linear speed", 10.0),
        _number("acceleration_mm_s2", "mm/s^2", "Requested linear acceleration", 40.0),
    )),
    ActionSpec("jog_cartesian", "Guarded relative world/tool TCP jog", "motion", (
        _triplet("translation_mm", "mm", "Requested relative translation"),
        _triplet("rotation_rpy_deg", "deg", "Requested relative rotation"),
        _FRAME, _ORIENTATION,
        _number("speed_mm_s", "mm/s", "Requested linear speed", 10.0),
        _number("acceleration_mm_s2", "mm/s^2", "Requested linear acceleration", 40.0),
    )),
    ActionSpec("move_gripper", "Set the stock gripper normalized position", "motion", (
        ArgumentSpec("position", "number", "Normalized gripper position 0 to 1", "fraction"),
    )),
    ActionSpec("stop", "Request the SDK's guarded STOP/HOLD operation", "stop",
               agent_eligible=True),
)


class CapabilityRegistry:
    """Frozen metadata with an explicit, non-reflective SDK dispatch table."""

    def __init__(self) -> None:
        self._specs = MappingProxyType({spec.name: spec for spec in _SPECS})

    def names(self) -> tuple[str, ...]:
        return tuple(self._specs)

    def describe(self) -> list[dict[str, object]]:
        return [spec.describe() for spec in self._specs.values()]

    def get(self, name: str) -> ActionSpec:
        try:
            return self._specs[name]
        except KeyError as exc:
            raise ValueError(f"unknown SDK capability: {name}") from exc

    def dispatch(self, name: str, arm: SOARM101, payload: Mapping[str, object]) -> Any:
        """Validated SDK operation, with *no* privilege elevation or implicit enable.

        The caller must already own an appropriate connected robot session and
        fulfill operator confirmation / broker authority and profile requirements.
        """
        args = self.get(name).validate(payload)
        if name == "read_pose":
            joints = dict(arm.get_joint_positions().positions)
            pose = arm.get_position().xyz_rpy()
            return {
                "joint_positions_rad": joints,
                "tcp_xyz_mm": [float(v) * 1000.0 for v in pose[:3]],
                "tcp_rpy_deg": [float(v) * 180.0 / pi for v in pose[3:]],
            }
        if name in {"solve_ik", "move_linear"}:
            from soarm101_motion.types import Pose
            xyz = args["target_xyz_mm"]
            rpy = args["target_rpy_deg"]
            target = Pose.from_xyz_rpy(
                *(float(v) / 1000.0 for v in xyz),
                *(float(v) * pi / 180.0 for v in rpy),
            )
            if name == "solve_ik":
                return arm.solve_ik(target, orientation_mode=args["orientation_mode"])
            return arm.move_linear(
                target, orientation_mode=args["orientation_mode"],
                speed=float(args["speed_mm_s"]) / 1000.0,
                acceleration=float(args["acceleration_mm_s2"]) / 1000.0,
            )
        if name == "move_joints":
            speed = float(args["speed_rad_s"])
            accel = float(args["acceleration_rad_s2"])
            return arm.move_joints(
                args["positions_rad"],
                speed=None if speed == 0 else speed,
                acceleration=None if accel == 0 else accel,
            )
        if name == "jog_cartesian":
            from soarm101_motion.control import jog_linear_cli_units
            return jog_linear_cli_units(
                arm, frame=args["frame"], translation_mm=args["translation_mm"],
                rotation_rpy_deg=args["rotation_rpy_deg"],
                orientation_mode=args["orientation_mode"],
                speed_mm_s=float(args["speed_mm_s"]),
                acceleration_mm_s2=float(args["acceleration_mm_s2"]),
            )
        if name == "move_gripper":
            position = float(args["position"])
            if not 0.0 <= position <= 1.0:
                raise ValueError("position must be in [0, 1]")
            return arm.tool.move(position)
        if name == "stop":
            return arm.stop()
        raise AssertionError(f"missing SDK capability dispatcher for {name}")


SDK_CAPABILITIES = CapabilityRegistry()
