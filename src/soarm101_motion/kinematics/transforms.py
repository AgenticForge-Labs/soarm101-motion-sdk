"""Rigid-transform helpers."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

FloatArray = NDArray[np.float64]


def transform_from_xyz_rpy(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> FloatArray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = xyz
    return transform


def rotation_about_axis(axis: FloatArray, angle: float) -> FloatArray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
    return transform
