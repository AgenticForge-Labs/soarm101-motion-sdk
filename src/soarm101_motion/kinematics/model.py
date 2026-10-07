"""Forward kinematics for the five-axis SO-ARM101."""

from __future__ import annotations

from dataclasses import dataclass
from math import pi
from typing import Mapping

import numpy as np
from numpy.typing import NDArray

from soarm101_motion.constants import ARM_JOINTS, JOINT_LIMITS
from soarm101_motion.kinematics.transforms import rotation_about_axis, transform_from_xyz_rpy
from soarm101_motion.types import Pose

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class PresentationLinkDefinition:
    """Simplified visual-link body derived from the packaged URDF visual."""

    name: str
    frame: str
    start_local: tuple[float, float, float]
    end_local: tuple[float, float, float]
    thickness_m: float


@dataclass(frozen=True)
class PresentationBoxDefinition:
    """Presentation-only oriented box tied to one kinematic link frame."""

    name: str
    frame: str
    center_local: tuple[float, float, float]
    rpy_local: tuple[float, float, float]
    size_local: tuple[float, float, float]

    @property
    def local_transform(self) -> FloatArray:
        return transform_from_xyz_rpy(self.center_local, self.rpy_local)


@dataclass(frozen=True)
class JointDefinition:
    name: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)

    @property
    def origin_transform(self) -> FloatArray:
        return transform_from_xyz_rpy(self.origin_xyz, self.origin_rpy)


SO101_JOINT_DEFINITIONS: tuple[JointDefinition, ...] = (
    JointDefinition(
        "shoulder_pan",
        (0.0388353, -8.97657e-09, 0.0624),
        (pi, 4.18253e-17, -pi),
    ),
    JointDefinition(
        "shoulder_lift",
        (-0.0303992, -0.0182778, -0.0542),
        (-pi / 2, -pi / 2, 0.0),
    ),
    JointDefinition(
        "elbow_flex",
        (-0.11257, -0.028, 1.73763e-16),
        (-3.63608e-16, 8.74301e-16, pi / 2),
    ),
    JointDefinition(
        "wrist_flex",
        (-0.1349, 0.0052, 3.62355e-17),
        (4.02456e-15, 8.67362e-16, -pi / 2),
    ),
    JointDefinition(
        "wrist_roll",
        (5.55112e-17, -0.0611, 0.0181),
        (pi / 2, 0.0486795, pi),
    ),
)

DEFAULT_GRIPPER_TCP = Pose.from_xyz_rpy(
    -0.0079,
    -0.000218121,
    -0.0981274,
    0.0,
    pi,
    0.0,
)

# Stock jaw pivot and nominal angular travel from the same official new-calibration
# URDF as the arm. These describe visualization, not calibrated actuator limits.
STOCK_JAW_JOINT = JointDefinition(
    "gripper", (0.0202, 0.0188, -0.0234), (pi / 2, 0.0, 0.0)
)
STOCK_JAW_LIMITS = (-0.174533, 1.74533)

# Presentation is deliberately separate from kinematic/safety geometry. The detailed
# official SO-101 model contains offset printed members plus one STS3215 mesh at each
# actuator; a joint axis is not generally the end or centerline of the adjacent printed
# arm. Use those mesh origins for the schematic rather than forcing every visible member
# to connect joint-center to joint-center.
#
# Source reference: TheRobotStudio/SO-ARM100,
# Simulation/SO101/so101_new_calib.urdf at
# 385e8d7c68e24945df6c60d9bd68837a4b7411ae.
SO101_PRESENTATION_LINKS: tuple[PresentationLinkDefinition, ...] = (
    PresentationLinkDefinition(
        "base",
        "base_link",
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.06),
        0.090,
    ),
    PresentationLinkDefinition(
        "upper_arm",
        "upper_arm_link",
        (-0.130085, 0.012, 0.0182),
        (-0.000085, 0.012, 0.0182),
        0.026,
    ),
    PresentationLinkDefinition(
        "lower_arm",
        "lower_arm_link",
        (-0.129700, -0.032, 0.0182),
        (0.0, -0.032, 0.0182),
        0.026,
    ),
)

# Feetech specifies the STS3215 case as 45.23 x 24.73 x 35 mm. The official SO-101
# mesh coordinate system uses the case height/width/length ordering below; combined
# with each mesh origin/RPY this places the driven joint axis inside the motor case
# instead of pretending the adjoining printed members terminate at the pivot.
STS3215_PRESENTATION_SIZE = (0.035, 0.02473, 0.04523)

SO101_PRESENTATION_BOXES: tuple[PresentationBoxDefinition, ...] = (
    PresentationBoxDefinition(
        "shoulder_pan_motor",
        "base_link",
        (0.0263353, -8.97657e-09, 0.0437),
        (0.0, 0.0, 0.0),
        STS3215_PRESENTATION_SIZE,
    ),
    PresentationBoxDefinition(
        "shoulder_lift_motor",
        "shoulder_link",
        (-0.0303992, 0.000422241, -0.0417),
        (pi / 2, pi / 2, 0.0),
        STS3215_PRESENTATION_SIZE,
    ),
    PresentationBoxDefinition(
        "elbow_flex_motor",
        "upper_arm_link",
        (-0.11257, -0.0155, 0.0187),
        (-pi, 0.0, -pi / 2),
        STS3215_PRESENTATION_SIZE,
    ),
    PresentationBoxDefinition(
        "wrist_flex_motor",
        "lower_arm_link",
        (-0.1224, 0.0052, 0.0187),
        (-pi, 0.0, -pi),
        STS3215_PRESENTATION_SIZE,
    ),
    PresentationBoxDefinition(
        "wrist_roll_motor",
        "wrist_link",
        (0.0, -0.0424, 0.0306),
        (pi / 2, pi / 2, 0.0),
        STS3215_PRESENTATION_SIZE,
    ),
    PresentationBoxDefinition(
        "gripper_motor",
        "gripper_link",
        (0.0077, 0.0001, -0.0234),
        (-pi / 2, 0.0, 0.0),
        STS3215_PRESENTATION_SIZE,
    ),
)


class SO101KinematicModel:
    """Small native kinematic model independent of ROS or URDF parsers."""

    joint_names = ARM_JOINTS
    joint_limits = JOINT_LIMITS

    def __init__(self, tcp: Pose = DEFAULT_GRIPPER_TCP) -> None:
        self.tcp = tcp

    def vector(self, joints: Mapping[str, float] | FloatArray) -> FloatArray:
        if isinstance(joints, Mapping):
            return np.array([joints[name] for name in self.joint_names], dtype=float)
        return np.asarray(joints, dtype=float).reshape(len(self.joint_names))

    def mapping(self, vector: FloatArray) -> dict[str, float]:
        vector = np.asarray(vector, dtype=float).reshape(len(self.joint_names))
        return {name: float(value) for name, value in zip(self.joint_names, vector, strict=True)}

    @property
    def lower_bounds(self) -> FloatArray:
        return np.array([self.joint_limits[name][0] for name in self.joint_names], dtype=float)

    @property
    def upper_bounds(self) -> FloatArray:
        return np.array([self.joint_limits[name][1] for name in self.joint_names], dtype=float)

    def _chain_transform(
        self,
        joints: Mapping[str, float] | FloatArray,
    ) -> tuple[FloatArray, dict[str, FloatArray]]:
        q = self.vector(joints)
        transform = np.eye(4)
        points: dict[str, FloatArray] = {"base": transform[:3, 3].copy()}
        for definition, angle in zip(SO101_JOINT_DEFINITIONS, q, strict=True):
            transform = transform @ definition.origin_transform
            points[definition.name] = transform[:3, 3].copy()
            transform = transform @ rotation_about_axis(
                np.asarray(definition.axis, dtype=float), float(angle)
            )
        return transform, points

    def link_frames(
        self,
        joints: Mapping[str, float] | FloatArray,
    ) -> dict[str, FloatArray]:
        """Return child-link frames after applying each joint rotation."""

        q = self.vector(joints)
        transform = np.eye(4)
        frames: dict[str, FloatArray] = {"base_link": transform.copy()}
        child_links = (
            "shoulder_link",
            "upper_arm_link",
            "lower_arm_link",
            "wrist_link",
            "gripper_link",
        )
        for definition, child_link, angle in zip(
            SO101_JOINT_DEFINITIONS,
            child_links,
            q,
            strict=True,
        ):
            transform = transform @ definition.origin_transform
            transform = transform @ rotation_about_axis(
                np.asarray(definition.axis, dtype=float), float(angle)
            )
            frames[child_link] = transform.copy()
        return frames

    def presentation_link_segments(
        self,
        joints: Mapping[str, float] | FloatArray,
    ) -> dict[str, tuple[FloatArray, FloatArray]]:
        """Return presentation-only visual-body centerlines in world coordinates.

        Unlike link_points(), these segments follow the packaged URDF visual bodies
        rather than connecting joint origins. They are for drawing only and must not
        be used for collision, workspace, planning, or hardware safety decisions.
        """

        frames = self.link_frames(joints)
        segments: dict[str, tuple[FloatArray, FloatArray]] = {}
        for definition in SO101_PRESENTATION_LINKS:
            frame = frames[definition.frame]

            def world(local: tuple[float, float, float]) -> FloatArray:
                vector = np.asarray(local, dtype=float)
                return frame[:3, 3] + frame[:3, :3] @ vector

            segments[definition.name] = (
                world(definition.start_local),
                world(definition.end_local),
            )
        return segments

    def presentation_link_thicknesses(self) -> dict[str, float]:
        """Return nominal visual-body thicknesses for GUI rendering only."""

        return {
            definition.name: float(definition.thickness_m)
            for definition in SO101_PRESENTATION_LINKS
        }

    def presentation_box_corners(
        self,
        joints: Mapping[str, float] | FloatArray,
    ) -> dict[str, tuple[FloatArray, ...]]:
        """Return mesh-informed motor-case envelope corners for GUI presentation only.

        These envelopes are not collision geometry and never participate in planning or
        safety. They make the physical distinction explicit: printed members can be
        offset from a joint axis, while the actuator body contains that axis.
        """

        frames = self.link_frames(joints)
        boxes: dict[str, tuple[FloatArray, ...]] = {}
        for definition in SO101_PRESENTATION_BOXES:
            transform = frames[definition.frame] @ definition.local_transform
            half = np.asarray(definition.size_local, dtype=float) / 2.0
            corners: list[FloatArray] = []
            for sx in (-1.0, 1.0):
                for sy in (-1.0, 1.0):
                    for sz in (-1.0, 1.0):
                        local = half * np.array([sx, sy, sz], dtype=float)
                        corners.append(
                            transform[:3, 3] + transform[:3, :3] @ local
                        )
            boxes[definition.name] = tuple(corners)
        return boxes

    def link_points(
        self,
        joints: Mapping[str, float] | FloatArray,
        *,
        tcp: Pose | None = None,
    ) -> dict[str, FloatArray]:
        """Return ordered joint/TCP centerline points for coarse safety checks."""
        transform, points = self._chain_transform(joints)
        tool_transform = transform @ (tcp or self.tcp).as_matrix()
        points["tcp"] = tool_transform[:3, 3].copy()
        return points

    def forward_matrix(
        self,
        joints: Mapping[str, float] | FloatArray,
        *,
        tcp: Pose | None = None,
    ) -> FloatArray:
        transform, _ = self._chain_transform(joints)
        return transform @ (tcp or self.tcp).as_matrix()

    def forward(
        self,
        joints: Mapping[str, float] | FloatArray,
        *,
        tcp: Pose | None = None,
    ) -> Pose:
        return Pose.from_matrix(self.forward_matrix(joints, tcp=tcp))

    def stock_gripper_points(
        self, joints: Mapping[str, float] | FloatArray, opening: float
    ) -> dict[str, FloatArray]:
        """Nominal fixed-finger/rotating-jaw outline, independent of the active TCP.

        The jaw endpoints use the official moving-jaw mesh's 82 mm extent and
        18.9 mm visual-origin offset. This is a schematic, not collision geometry;
        normalized opening maps to nominal URDF travel, not measured jaw angle.
        """
        flange = self.forward_matrix(joints, tcp=Pose.identity())
        angle = STOCK_JAW_LIMITS[0] + float(np.clip(opening, 0.0, 1.0)) * (
            STOCK_JAW_LIMITS[1] - STOCK_JAW_LIMITS[0]
        )
        jaw = flange @ STOCK_JAW_JOINT.origin_transform @ rotation_about_axis(
            np.array(STOCK_JAW_JOINT.axis), angle
        )

        def point(frame: FloatArray, local: tuple[float, float, float]) -> FloatArray:
            return frame[:3, 3] + frame[:3, :3] @ np.asarray(local)

        return {
            "origin": flange[:3, 3].copy(),
            "fixed_root": point(flange, (-0.0079, -0.000218121, -0.052)),
            "fixed_tip": point(flange, tuple(DEFAULT_GRIPPER_TCP.position)),
            "moving_pivot": jaw[:3, 3].copy(),
            "moving_root": point(jaw, (0.0, -0.015, 0.0189)),
            "moving_tip": point(jaw, (-0.010, -0.082, 0.0189)),
        }

    def jacobian(
        self,
        joints: Mapping[str, float] | FloatArray,
        *,
        tcp: Pose | None = None,
        epsilon: float = 1e-6,
    ) -> FloatArray:
        """Numerical 6x5 geometric Jacobian suitable for validation and control."""
        q = self.vector(joints)
        base = self.forward(q, tcp=tcp)
        jacobian = np.zeros((6, len(q)))
        from scipy.spatial.transform import Rotation

        for index in range(len(q)):
            shifted = q.copy()
            shifted[index] += epsilon
            pose = self.forward(shifted, tcp=tcp)
            jacobian[:3, index] = (pose.position - base.position) / epsilon
            delta = Rotation.from_matrix(pose.rotation @ base.rotation.T).as_rotvec()
            jacobian[3:, index] = delta / epsilon
        return jacobian
