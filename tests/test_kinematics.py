import numpy as np
import pytest

from soarm101_motion.kinematics import IKOptions, IKSolver, SO101KinematicModel


TARGET_JOINTS = {
    "shoulder_pan": 0.30,
    "shoulder_lift": -0.40,
    "elbow_flex": 0.70,
    "wrist_flex": 0.20,
    "wrist_roll": -0.30,
}


def test_forward_kinematics_is_finite() -> None:
    model = SO101KinematicModel()
    pose = model.forward(TARGET_JOINTS)
    assert pose.position.shape == (3,)
    assert pose.rotation.shape == (3, 3)
    assert np.all(np.isfinite(pose.as_matrix()))
    assert np.linalg.det(pose.rotation) == pytest.approx(1.0, abs=1e-8)


def test_exact_ik_recovers_fk_pose() -> None:
    model = SO101KinematicModel()
    target = model.forward(TARGET_JOINTS)
    seed = {name: 0.0 for name in model.joint_names}
    result = IKSolver(model).solve(
        target,
        seed=seed,
        options=IKOptions(orientation_mode="exact"),
    )
    assert result.success
    assert result.position_error_m < 1e-4
    assert result.orientation_error_rad < 1e-3
    for name, expected in TARGET_JOINTS.items():
        assert result.joints[name] == pytest.approx(expected, abs=0.003)


def test_position_only_ik() -> None:
    model = SO101KinematicModel()
    target = model.forward(TARGET_JOINTS)
    result = IKSolver(model).solve(
        target,
        seed={name: 0.0 for name in model.joint_names},
        options=IKOptions(orientation_mode="position_only"),
    )
    assert result.success
    assert result.position_error_m < 1e-4


def test_jacobian_shape() -> None:
    jacobian = SO101KinematicModel().jacobian(TARGET_JOINTS)
    assert jacobian.shape == (6, 5)
    assert np.all(np.isfinite(jacobian))
