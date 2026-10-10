"""Pure capability metadata and explicit SDK dispatch shared by human and broker adapters.

This layer is an operation contract, not authorization: callers own hardware session,
human confirmation and (for remote agents) the broker's additional runtime policy.
Importing or inspecting the registry never opens hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from math import pi
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Literal, Mapping

from soarm101_motion.constants import ARM_JOINTS

if TYPE_CHECKING:
    from soarm101_motion.arm import SOARM101
    from soarm101_motion.camera import CameraSettings

ValueKind = Literal["number", "numbers", "choice", "text", "bool"]
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
        elif self.kind == "text":
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{self.name} must be a nonempty string")
            value = value.strip()
        elif self.kind == "bool":
            if not isinstance(value, bool):
                raise ValueError(f"{self.name} must be a boolean")
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
                validated[arg.name] = (
                    None if arg.default is None else arg.validate(arg.default)
                )
        return validated

    def describe(self) -> dict[str, object]:
        return {
            "name": self.name, "description": self.description,
            "effect": self.effect, "agent_eligible": self.agent_eligible,
            "arguments": [arg.describe() for arg in self.arguments],
        }


def _number(name: str, unit: str, description: str, default: float) -> ArgumentSpec:
    return ArgumentSpec(name, "number", description, unit, required=False, default=default)


def _checked_gripper_raw(value: object, name: str, maximum: int) -> int:
    """Servo register limits: reject float coercion and 0=max-speed sentinel."""
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(float(value)) or int(value) != value):
        raise ValueError(f"gripper {name} must be a finite whole number")
    raw = int(value)
    if not 1 <= raw <= maximum:
        raise ValueError(f"gripper {name} must be in [1, {maximum}]")
    return raw


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
        ArgumentSpec("speed_rad_s", "number", "Optional joint speed", "rad/s",
                     required=False),
        ArgumentSpec("acceleration_rad_s2", "number", "Optional joint acceleration",
                     "rad/s^2", required=False),
    )),
    ActionSpec("jog_joint", "Relative movement of one calibrated pose joint", "motion", (
        ArgumentSpec("joint", "choice", "Name of pose joint",
                     choices=tuple(ARM_JOINTS)),
        ArgumentSpec("delta_rad", "number", "Relative joint angle", "rad"),
        ArgumentSpec("speed_rad_s", "number", "Optional joint speed", "rad/s",
                     required=False),
        ArgumentSpec("acceleration_rad_s2", "number", "Optional joint acceleration",
                     "rad/s^2", required=False),
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
        _number("gripper_speed_raw", "raw", "Feetech gripper speed", 250.0),
        _number("gripper_acceleration_raw", "raw", "Feetech gripper acceleration", 20.0),
    )),
    ActionSpec("list_saved_poses", "List persisted saved pose metadata", "read", (
        ArgumentSpec("robot_id", "text", "Identity of the saved pose library"),
    )),
    ActionSpec("capture_saved_pose", "Capture one measured calibration-bound saved pose", "read", (
        ArgumentSpec("robot_id", "text", "Saved pose library robot identity"),
        ArgumentSpec("name", "text", "Name for the saved pose"),
        ArgumentSpec("source", "choice", "Arm source of the measurement",
                     choices=("follower", "leader"), required=False, default="follower"),
    )),
    ActionSpec("validate_saved_pose", "Check saved pose provenance before torque enable", "read", (
        ArgumentSpec("robot_id", "text", "Saved pose library robot identity"),
        ArgumentSpec("name", "text", "Saved pose to validate"),
    )),
    ActionSpec("replay_saved_pose", "Replay calibration-bound arm and gripper pose", "motion", (
        ArgumentSpec("robot_id", "text", "Saved pose library robot identity"),
        ArgumentSpec("name", "text", "Saved pose to replay"),
        ArgumentSpec("mode", "choice", "Joint or absolute Cartesian playback",
                     choices=("joint", "linear"), required=False, default="joint"),
        _ORIENTATION,
        _number("speed_deg_s", "deg/s", "Joint playback speed", 8.0),
        _number("acceleration_deg_s2", "deg/s^2", "Joint playback acceleration", 25.0),
        _number("speed_mm_s", "mm/s", "Linear playback speed", 10.0),
        _number("acceleration_mm_s2", "mm/s^2", "Linear playback acceleration", 40.0),
        _number("gripper_speed_raw", "raw", "Saved gripper speed", 250.0),
        _number("gripper_acceleration_raw", "raw", "Saved gripper acceleration", 20.0),
    )),
    ActionSpec("camera_profiles", "Read persisted named camera profiles", "read", (
        ArgumentSpec("name", "text", "Specific saved camera name", required=False),
    )),
    ActionSpec("capture_camera", "Capture a named camera still using trusted settings", "read", (
        ArgumentSpec("name", "text", "Saved camera name"),
        ArgumentSpec("output", "text", "Trusted-host output path", required=False),
    )),
    ActionSpec("read_effort_status", "Read the existing SDK effort-safety status", "read", (
        ArgumentSpec("refresh", "bool", "Refresh existing effort diagnostics",
                     required=False, default=False),
    )),
    ActionSpec("read_hardware_state", "Read current connected/torque/motion/fault state", "read"),
    ActionSpec("sleep", "Move to guarded calibrated Sleep posture", "motion", (
        _number("speed_rad_s", "rad/s", "Sleep joint speed", 8.0 * pi / 180.0),
        _number("acceleration_rad_s2", "rad/s^2", "Sleep joint acceleration",
                25.0 * pi / 180.0),
        _number("gripper_speed_raw", "raw", "Sleep gripper speed", 250.0),
        _number("gripper_acceleration_raw", "raw", "Sleep gripper acceleration", 20.0),
    )),
    ActionSpec("sleep_up", "Move to guarded folded Sleep-up posture", "motion", (
        _number("speed_rad_s", "rad/s", "Sleep-up joint speed", 8.0 * pi / 180.0),
        _number("acceleration_rad_s2", "rad/s^2", "Sleep-up joint acceleration",
                25.0 * pi / 180.0),
        _number("gripper_speed_raw", "raw", "Sleep-up gripper speed", 250.0),
        _number("gripper_acceleration_raw", "raw", "Sleep-up gripper acceleration", 20.0),
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

    def dispatch(
        self,
        name: str,
        arm: SOARM101 | None,
        payload: Mapping[str, object],
        *,
        camera_settings: CameraSettings | None = None,
    ) -> Any:
        """Validated SDK operation, with *no* privilege elevation or implicit enable.

        The caller must already own an appropriate connected robot session and
        fulfill operator confirmation / broker authority and profile requirements.
        """
        args = self.get(name).validate(payload)
        if name == "list_saved_poses":
            from soarm101_motion.poses import PoseLibrary
            library = PoseLibrary(str(args["robot_id"]))
            return [
                {"name": pose_name, "source": library.require(pose_name).source,
                 "created_at": library.require(pose_name).created_at}
                for pose_name in library.names()
            ]
        if name == "camera_profiles":
            from soarm101_motion.workstation import WorkstationProfileStore

            profile = WorkstationProfileStore().load()
            selected = args["name"]
            return profile.camera_payload(
                None if selected is None else str(selected)
            )
        if name == "capture_camera":
            from soarm101_motion.camera import CameraCapture
            from soarm101_motion.workstation import WorkstationProfileStore

            camera_name = str(args["name"])
            settings = camera_settings or WorkstationProfileStore().load().camera(
                camera_name
            )
            with CameraCapture(settings) as camera:
                path, metadata = camera.snapshot(args["output"])
            return {"name": camera_name, **metadata}
        if arm is None:
            raise ValueError(f"SDK capability {name} requires a connected SDK session")
        if name in {"validate_saved_pose", "replay_saved_pose"}:
            from soarm101_motion.poses import PoseLibrary

            pose_name = str(args["name"])
            saved = PoseLibrary(str(args["robot_id"])).require(pose_name)
            arm.require_artifact_calibration(
                {
                    "source_robot_id": saved.source_robot_id,
                    "source_calibration_id": saved.source_calibration_id,
                    "target_robot_id": saved.target_robot_id,
                    "target_calibration_id": saved.target_calibration_id,
                },
                artifact_label=f"saved pose {pose_name!r}",
            )
            if name == "validate_saved_pose":
                return saved
            if args["mode"] == "joint":
                result = arm.move_joints_from_saved_pose(
                    saved.joints,
                    speed=float(args["speed_deg_s"]) * pi / 180.0,
                    acceleration=float(args["acceleration_deg_s2"]) * pi / 180.0,
                )
            else:
                from soarm101_motion.types import Pose

                result = arm.move_linear(
                    Pose.from_xyz_rpy(*saved.tcp_xyz_rpy),
                    orientation_mode=args["orientation_mode"],
                    speed=float(args["speed_mm_s"]) / 1000.0,
                    acceleration=float(args["acceleration_mm_s2"]) / 1000.0,
                )
            gripper_result = arm.tool.move(
                saved.gripper,
                speed_raw=_checked_gripper_raw(args["gripper_speed_raw"], "speed", 3400),
                acceleration_raw=_checked_gripper_raw(
                    args["gripper_acceleration_raw"], "acceleration", 254
                ),
            )
            arm.hold()
            return result, gripper_result
        if name == "capture_saved_pose":
            from soarm101_motion.poses import PoseLibrary, SavedPose

            saved = SavedPose.capture(arm, source=str(args["source"]))
            return PoseLibrary(str(args["robot_id"])).save(str(args["name"]), saved)
        if name == "read_hardware_state":
            return arm.get_state()
        if name == "read_effort_status":
            return arm.get_effort_safety_status(refresh=bool(args["refresh"]))
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
            return arm.move_joints(
                args["positions_rad"],
                speed=args["speed_rad_s"],
                acceleration=args["acceleration_rad_s2"],
            )
        if name == "jog_joint":
            return arm.move_joints(
                {str(args["joint"]): float(args["delta_rad"])},
                relative=True,
                speed=args["speed_rad_s"],
                acceleration=args["acceleration_rad_s2"],
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
            return arm.tool.move(
                position,
                speed_raw=_checked_gripper_raw(args["gripper_speed_raw"], "speed", 3400),
                acceleration_raw=_checked_gripper_raw(
                    args["gripper_acceleration_raw"], "acceleration", 254
                ),
            )
        if name in ("sleep", "sleep_up"):
            motion = arm.move_sleep if name == "sleep" else arm.move_sleep_up
            return motion(
                speed=float(args["speed_rad_s"]),
                acceleration=float(args["acceleration_rad_s2"]),
                gripper_speed_raw=_checked_gripper_raw(
                    args["gripper_speed_raw"], "speed", 3400
                ),
                gripper_acceleration_raw=_checked_gripper_raw(
                    args["gripper_acceleration_raw"], "acceleration", 254
                ),
            )
        if name == "stop":
            return arm.stop()
        raise AssertionError(f"missing SDK capability dispatcher for {name}")


SDK_CAPABILITIES = CapabilityRegistry()
