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


def test_ik_task_refinement_can_recover_from_regularization_tradeoff() -> None:
    from soarm101_motion.types import Pose

    class FakeModel:
        joint_names = (
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
        )
        lower_bounds = np.full(5, -1.0)
        upper_bounds = np.full(5, 1.0)

        @staticmethod
        def vector(values):
            if isinstance(values, dict):
                return np.array([float(values[name]) for name in FakeModel.joint_names])
            return np.asarray(values, dtype=float)

        @staticmethod
        def mapping(values):
            vector = np.asarray(values, dtype=float)
            return {
                name: float(vector[index])
                for index, name in enumerate(FakeModel.joint_names)
            }

        @staticmethod
        def forward(values, tcp=None):
            del tcp
            vector = FakeModel.vector(values)
            return Pose(
                np.array([vector[0], 0.0, 0.0]),
                np.eye(3),
            )

    target = Pose(np.array([0.2, 0.0, 0.0]), np.eye(3))
    seed = {name: 0.0 for name in FakeModel.joint_names}
    result = IKSolver(FakeModel()).solve(
        target,
        seed=seed,
        options=IKOptions(
            orientation_mode="position_only",
            position_tolerance_m=1e-5,
            position_weight=1.0,
            continuity_weight=20.0,
            joint_center_weight=0.0,
            multi_start=False,
        ),
    )

    assert result.success is True
    assert result.position_error_m <= 1e-5
    assert result.joints["shoulder_pan"] == pytest.approx(0.2, abs=1e-5)
