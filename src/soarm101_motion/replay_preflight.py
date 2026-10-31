"""Offline sequence preflight, not a movement executor or authorization grant."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping
from urllib.parse import quote

from soarm101_motion.asset_catalog import AssetDescriptor, discover_assets
from soarm101_motion.constants import ARM_JOINTS
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.provenance import require_calibration_compatibility
from soarm101_motion.replay_policy import ReplayPolicy
from soarm101_motion.replay_snapshot import load_pinned_recording
from soarm101_motion.sequences import SequenceLibrary
from soarm101_motion.trajectories import TrajectoryLibrary


@dataclass(frozen=True)
class EntryTarget:
    """A trajectory entry target that must be authorized before execution."""
    asset_id: str
    joints_rad: tuple[tuple[str, float], ...]
    gripper: float
    occurrences: int


@dataclass(frozen=True)
class PoseTarget:
    """A resolved saved-pose target in sequence order, pinned by asset revision."""
    asset_id: str
    joints_rad: tuple[tuple[str, float], ...]
    gripper: float | None


@dataclass(frozen=True)
class PlannedSequenceStep:
    """One ordered, dependency-pinned sequence action for a future executor.

    Keeping the resolved action in sequence order prevents an executor from
    accidentally grouping all poses before all trajectories. This remains a
    preflight description; it does not authorize or perform movement.
    """
    index: int
    kind: str
    asset_id: str | None
    revision: str | None
    speed_scale: float
    loops: int
    pose_target: PoseTarget | None = None
    entry: EntryTarget | None = None
    gripper_position: float | None = None
    wait_seconds: float | None = None


@dataclass(frozen=True)
class ReplayPreflight:
    sequence_id: str
    dependencies: tuple[tuple[str, str], ...]
    pose_targets: tuple[PoseTarget, ...]
    entries: tuple[EntryTarget, ...]
    planned_steps: tuple[PlannedSequenceStep, ...]
    expanded_steps: int
    known_duration_s: float


@dataclass(frozen=True)
class RecordingPreflight:
    asset_id: str
    revision: str
    entry: EntryTarget
    known_duration_s: float


def preflight_recording(
    policy: ReplayPolicy, asset_id: str, poses: PoseLibrary,
    trajectories: TrajectoryLibrary, sequences: SequenceLibrary, *, speed_scale: float,
    actual_joints: Mapping[str, float], actual_gripper: float,
) -> RecordingPreflight:
    """Check one recorded asset and its current entry state without commanding motion.

    The supplied state is only a snapshot. It must be reacquired and checked at
    execution under the same authority and STOP fence as the eventual movement.
    """
    if (isinstance(speed_scale, bool) or not isinstance(speed_scale, (int, float))
            or not math.isfinite(speed_scale) or not 0 < speed_scale <= policy.max_speed_scale):
        raise ValueError("speed scale exceeds replay policy")
    if {poses.robot_id, trajectories.robot_id, sequences.robot_id} != {policy.robot_id}:
        raise PermissionError("libraries do not match policy robot")
    before = {item.id: item for item in discover_assets(
        poses, trajectories, sequences, current_robot_id=policy.robot_id,
        current_calibration_id=policy.calibration_id,
    )}
    descriptor = before.get(asset_id)
    if descriptor is None or descriptor.kind != "recording":
        raise ValueError("recording is missing from configured library")
    policy.require_asset(asset_id, descriptor.revision)
    if descriptor.compatibility != "calibration_matched":
        raise PermissionError("recording calibration is missing or mismatched")
    recording = load_pinned_recording(policy, asset_id, trajectories).trajectory()
    require_calibration_compatibility(
        recording.metadata, current_robot_id=policy.robot_id,
        current_calibration_id=policy.calibration_id, artifact_label="recording",
    )
    joints = tuple((joint, float(recording.joints_rad[0, i]))
                   for i, joint in enumerate(ARM_JOINTS))
    gripper = float(recording.gripper[0])
    policy.check_start(actual_joints, dict(joints))
    policy.check_gripper_start(actual_gripper, gripper)
    duration = recording.duration_s / speed_scale
    if not math.isfinite(duration) or duration > policy.max_duration_s:
        raise ValueError("recording exceeds replay duration ceiling")
    after = {item.id: item.revision for item in discover_assets(poses, trajectories, sequences)}
    if after.get(asset_id) != descriptor.revision:
        raise RuntimeError("recording changed during preflight")
    return RecordingPreflight(asset_id, descriptor.revision,
                              EntryTarget(asset_id, joints, gripper, 1), duration)


def preflight_sequence(
    policy: ReplayPolicy, sequence_id: str, poses: PoseLibrary,
    trajectories: TrajectoryLibrary, sequences: SequenceLibrary, *, speed_scale: float,
) -> ReplayPreflight:
    """Validate a conservative joint-space subset without connecting to hardware.

    Pose moves and trajectory entry moves have unknown duration, so known_duration_s
    is only a lower bound. Runtime still needs deadlines, full path/rate checks,
    authority and STOP fencing, and revision checks immediately before execution.
    Cartesian, overrides and primitive steps are rejected until independently bounded.
    """
    if (isinstance(speed_scale, bool) or not isinstance(speed_scale, (float, int))
            or not math.isfinite(speed_scale) or not 0 < speed_scale <= policy.max_speed_scale):
        raise ValueError("speed scale exceeds replay policy")
    if {poses.robot_id, trajectories.robot_id, sequences.robot_id} != {policy.robot_id}:
        raise PermissionError("libraries do not match policy robot")
    catalog = {item.id: item for item in discover_assets(
        poses, trajectories, sequences, current_robot_id=policy.robot_id,
        current_calibration_id=policy.calibration_id,
    )}
    dependencies: dict[str, str] = {}
    def require(asset_id: str, kind: str, *, calibration: bool = True) -> AssetDescriptor:
        item = catalog.get(asset_id)
        if item is None or item.kind != kind:
            raise ValueError("asset is missing from configured library")
        policy.require_asset(asset_id, item.revision)
        if calibration and item.compatibility != "calibration_matched":
            raise PermissionError("asset calibration is missing or mismatched")
        dependencies[item.id] = item.revision
        return item

    require(sequence_id, "sequence", calibration=False)
    sequence_names = {f"soarm101/sequence/{quote(name, safe='')}": name for name in sequences.names()}
    name = sequence_names.get(sequence_id)
    if name is None:
        raise ValueError("sequence is missing")
    sequence = sequences.require(name)
    require_calibration_compatibility(
        sequence.metadata, current_robot_id=policy.robot_id,
        current_calibration_id=policy.calibration_id, artifact_label="sequence",
    )
    entries: list[EntryTarget] = []
    pose_targets: list[PoseTarget] = []
    planned_steps: list[PlannedSequenceStep] = []
    expanded_steps = 0
    known_duration = 0.0
    for index, step in enumerate(sequence.steps):
        params = step.params
        allowed = {
            "point": {"name", "mode", "speed_scale"},
            "home": {"mode", "speed_scale"}, "rest": {"mode", "speed_scale"},
            "gripper": {"position", "speed_scale"}, "wait": {"seconds", "speed_scale"},
            "trajectory": {"name", "kind", "loops", "speed_scale"},
        }
        if step.kind not in allowed or set(params) - allowed[step.kind]:
            raise ValueError("sequence contains unsupported step or parameters")
        local = params.get("speed_scale", 1.0)
        if (isinstance(local, bool) or not isinstance(local, (int, float))
                or not math.isfinite(local) or local <= 0 or local * speed_scale > policy.max_speed_scale):
            raise ValueError("sequence step exceeds speed scale ceiling")
        expanded_steps += 1
        step_scale = float(local) * float(speed_scale)
        planned: PlannedSequenceStep
        if step.kind in {"point", "home", "rest"}:
            pose_name = params.get("name") if step.kind == "point" else step.kind
            if not isinstance(pose_name, str) or not pose_name:
                raise ValueError("pose name is required")
            if params.get("mode", "joint") != "joint":
                raise ValueError("only joint-space pose steps are supported")
            pose_id = f"soarm101/pose/{quote(pose_name, safe='')}"
            pose_descriptor = require(pose_id, "pose")
            pose = poses.require(pose_name)
            joints = tuple((joint, float(pose.joints[joint])) for joint in ARM_JOINTS)
            target = PoseTarget(
                pose_id, joints, float(pose.gripper) if step.kind in {"home", "rest"} else None,
            )
            pose_targets.append(target)
            planned = PlannedSequenceStep(index, step.kind, pose_id, pose_descriptor.revision,
                                          step_scale, 1, pose_target=target)
        elif step.kind == "gripper":
            position = params.get("position")
            if (isinstance(position, bool) or not isinstance(position, (float, int))
                    or not math.isfinite(position) or not 0 <= position <= 1):
                raise ValueError("gripper position must be finite and within 0..1")
            planned = PlannedSequenceStep(index, step.kind, None, None, step_scale, 1,
                                          gripper_position=float(position))
        elif step.kind == "wait":
            seconds = params.get("seconds")
            if (isinstance(seconds, bool) or not isinstance(seconds, (float, int))
                    or not math.isfinite(seconds) or seconds < 0):
                raise ValueError("wait must be finite and non-negative")
            known_duration += seconds
            planned = PlannedSequenceStep(index, step.kind, None, None, step_scale, 1,
                                          wait_seconds=float(seconds))
        else:
            trajectory_name, kind, loops = params.get("name"), params.get("kind", "edited"), params.get("loops", 1)
            if not isinstance(trajectory_name, str) or kind not in ("raw", "edited"):
                raise ValueError("invalid trajectory reference")
            if type(loops) is not int or not 1 <= loops <= policy.max_steps:
                raise ValueError("trajectory loops exceed policy")
            asset_id = f"soarm101/recording/{kind}/{quote(trajectory_name, safe='')}"
            recording_descriptor = require(asset_id, "recording")
            trajectory = load_pinned_recording(policy, asset_id, trajectories).trajectory()
            require_calibration_compatibility(
                trajectory.metadata, current_robot_id=policy.robot_id,
                current_calibration_id=policy.calibration_id, artifact_label="trajectory",
            )
            entry = EntryTarget(asset_id,
                tuple((joint, float(trajectory.joints_rad[0, i])) for i, joint in enumerate(ARM_JOINTS)),
                float(trajectory.gripper[0]), loops)
            entries.append(entry)
            planned = PlannedSequenceStep(index, step.kind, asset_id, recording_descriptor.revision,
                                          step_scale, loops, entry=entry)
            expanded_steps += loops - 1
            known_duration += trajectory.duration_s * loops / (speed_scale * local)
        if expanded_steps > policy.max_steps or known_duration > policy.max_duration_s:
            raise ValueError("sequence exceeds replay step or duration ceiling")
        planned_steps.append(planned)
    # Re-read all referenced assets to detect mutation during the preflight window.
    after = {item.id: item.revision for item in discover_assets(poses, trajectories, sequences)}
    if any(after.get(asset_id) != revision for asset_id, revision in dependencies.items()):
        raise RuntimeError("replay asset changed during preflight")
    return ReplayPreflight(sequence_id, tuple(sorted(dependencies.items())),
                           tuple(pose_targets), tuple(entries), tuple(planned_steps),
                           expanded_steps, known_duration)
