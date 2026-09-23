from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from soarm101_motion import SOARM101, SOARM101Config
from soarm101_motion.calibration import (
    MotorCalibration,
    SO101Calibration,
    archive_calibration,
    save_versioned_calibration,
)
from soarm101_motion.constants import ALL_MOTORS, MOTOR_IDS
from soarm101_motion.exceptions import CalibrationError
from soarm101_motion.hardware import SimulationBackend
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.provenance import require_calibration_compatibility
from soarm101_motion.sequences import MotionSequence, SequenceRunner, SequenceStep
from soarm101_motion.trajectories import Trajectory, TrajectoryLibrary


def _calibration(*, offset_delta: int = 0, source: str = "test") -> SO101Calibration:
    return SO101Calibration(
        motors={
            name: MotorCalibration(
                MOTOR_IDS[name],
                0,
                offset_delta if name == "shoulder_pan" else 0,
                100,
                3995,
            )
            for name in ALL_MOTORS
        },
        source=source,
    )


def test_calibration_fingerprint_is_stable_and_source_independent() -> None:
    first = _calibration(source="native")
    second = _calibration(source="imported")
    reordered = SO101Calibration(
        motors=dict(reversed(list(first.motors.items()))),
        source="other",
    )

    assert first.fingerprint == second.fingerprint == reordered.fingerprint
    assert first.calibration_id.startswith("sha256:")
    assert len(first.fingerprint) == 64


def test_calibration_fingerprint_changes_with_physical_fields() -> None:
    assert _calibration().fingerprint != _calibration(offset_delta=1).fingerprint


def test_versioned_calibration_writes_current_and_immutable_history(
    tmp_path: Path,
) -> None:
    calibration = _calibration()
    current = tmp_path / "current.json"
    history = tmp_path / "history"

    current_path, history_path = save_versioned_calibration(
        calibration,
        robot_id="follower",
        current_path=current,
        history_dir=history,
    )

    assert current_path == current
    assert history_path == history / f"{calibration.fingerprint}.json"
    assert SO101Calibration.load(current_path).fingerprint == calibration.fingerprint
    assert SO101Calibration.load(history_path).fingerprint == calibration.fingerprint

    same_path = archive_calibration(
        calibration,
        robot_id="follower",
        history_dir=history,
    )
    assert same_path == history_path


def test_same_robot_artifact_requires_matching_source_calibration() -> None:
    require_calibration_compatibility(
        {
            "source_robot_id": "follower",
            "source_calibration_id": "sha256:abc",
        },
        current_robot_id="follower",
        current_calibration_id="sha256:abc",
        artifact_label="pose",
    )

    with pytest.raises(CalibrationError, match="active calibration"):
        require_calibration_compatibility(
            {
                "source_robot_id": "follower",
                "source_calibration_id": "sha256:old",
            },
            current_robot_id="follower",
            current_calibration_id="sha256:new",
            artifact_label="pose",
        )


def test_cross_robot_artifact_requires_explicit_target_binding() -> None:
    with pytest.raises(CalibrationError, match="no target calibration binding"):
        require_calibration_compatibility(
            {
                "source_robot_id": "leader",
                "source_calibration_id": "sha256:leader",
            },
            current_robot_id="follower",
            current_calibration_id="sha256:follower",
            artifact_label="trajectory",
        )

    require_calibration_compatibility(
        {
            "source_robot_id": "leader",
            "source_calibration_id": "sha256:leader",
            "target_robot_id": "follower",
            "target_calibration_id": "sha256:follower",
        },
        current_robot_id="follower",
        current_calibration_id="sha256:follower",
        artifact_label="trajectory",
    )


def test_target_binding_takes_priority_after_recalibration() -> None:
    with pytest.raises(CalibrationError, match="review and re-bind"):
        require_calibration_compatibility(
            {
                "source_robot_id": "leader",
                "source_calibration_id": "sha256:leader",
                "target_robot_id": "follower",
                "target_calibration_id": "sha256:old-follower",
            },
            current_robot_id="follower",
            current_calibration_id="sha256:new-follower",
            artifact_label="trajectory",
        )


def test_legacy_artifact_fails_closed() -> None:
    with pytest.raises(CalibrationError, match="no calibration provenance"):
        require_calibration_compatibility(
            {},
            current_robot_id="follower",
            current_calibration_id="sha256:current",
            artifact_label="trajectory",
        )


def test_arm_artifact_guard_uses_active_backend_calibration() -> None:
    calibration = _calibration()
    backend = SimulationBackend(realtime=False)
    backend.calibration = calibration  # type: ignore[attr-defined]
    arm = SOARM101(
        SOARM101Config(robot_id="follower"),
        backend=backend,
    )

    arm.require_artifact_calibration(
        {
            "source_robot_id": "follower",
            "source_calibration_id": calibration.calibration_id,
        },
        artifact_label="pose",
    )

    with pytest.raises(CalibrationError, match="active calibration"):
        arm.require_artifact_calibration(
            {
                "source_robot_id": "follower",
                "source_calibration_id": "sha256:stale",
            },
            artifact_label="pose",
        )


def _minimal_trajectory(metadata: dict[str, str]) -> Trajectory:
    return Trajectory(
        timestamps_s=np.array([0.0, 0.1]),
        joints_rad=np.zeros((2, 5)),
        gripper=np.array([0.5, 0.5]),
        metadata=metadata,
    )


def test_public_play_trajectory_rejects_stale_calibration_before_motion() -> None:
    calibration = _calibration()
    backend = SimulationBackend(realtime=False)
    backend.calibration = calibration  # type: ignore[attr-defined]
    arm = SOARM101(
        SOARM101Config(robot_id="follower"),
        backend=backend,
    )
    trajectory = _minimal_trajectory(
        {
            "source_robot_id": "follower",
            "source_calibration_id": "sha256:stale",
        }
    )

    with pytest.raises(CalibrationError, match="active calibration"):
        arm.play_trajectory(trajectory)


def test_sequence_runner_rejects_stale_gripper_only_sequence(
    tmp_path: Path,
) -> None:
    calibration = _calibration()
    backend = SimulationBackend(realtime=False)
    backend.calibration = calibration  # type: ignore[attr-defined]
    arm = SOARM101(
        SOARM101Config(robot_id="follower"),
        backend=backend,
    )
    runner = SequenceRunner(
        arm,
        pose_library=PoseLibrary("follower", path=tmp_path / "poses.json"),
        trajectory_library=TrajectoryLibrary(
            "follower",
            root=tmp_path / "trajectories",
        ),
    )
    sequence = MotionSequence(
        "stale_gripper",
        (SequenceStep("gripper", {"position": 0.5}),),
        metadata={
            "target_robot_id": "follower",
            "target_calibration_id": "sha256:stale",
        },
    )

    with pytest.raises(CalibrationError, match="review and re-bind"):
        runner.run(sequence)
