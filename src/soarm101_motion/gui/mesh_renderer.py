"""CPU triangle renderer for topology-simplified SO-101 visual meshes."""

from __future__ import annotations

import json
from itertools import product
import struct
from functools import lru_cache
from importlib.resources import files

import numpy as np
from numpy.typing import NDArray

from soarm101_motion.kinematics.transforms import transform_from_xyz_rpy
from soarm101_motion.kinematics.model import STOCK_JAW_JOINT, STOCK_JAW_LIMITS, rotation_about_axis

FloatArray = NDArray[np.float64]
ASSET_DIR = files("soarm101_motion.gui").joinpath("assets/so101")


@lru_cache(maxsize=1)
def _definitions() -> tuple[dict[str, object], ...]:
    return tuple(json.loads(ASSET_DIR.joinpath("visuals.json").read_text())["visuals"])


@lru_cache(maxsize=1)
def _static_visuals() -> tuple[tuple[dict[str, object], FloatArray], ...]:
    return tuple(
        (definition, transform_from_xyz_rpy(definition["xyz"], definition["rpy"]))
        for definition in _definitions()
    )


@lru_cache(maxsize=32)
def _triangles(filename: str, low_detail: bool = False) -> FloatArray:
    path = ASSET_DIR.joinpath(filename)
    lod_path = path.with_name(f"{path.stem}_lod.stl")
    if low_detail and lod_path.is_file():
        path = lod_path
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    # Binary STL records are 50 bytes: 12 normal bytes, 36 vertex bytes,
    # then a 2-byte attribute. A strided view skips those non-vertex fields.
    triangles = np.ndarray(
        (count, 3, 3), dtype="<f4", buffer=data, offset=96, strides=(50, 12, 4)
    ).astype(float)
    triangles.setflags(write=False)
    return triangles


def world_triangles(
    link_frames: dict[str, FloatArray], gripper_position: float = 0.5,
    *, low_detail: bool = False,
) -> list[tuple[FloatArray, str]]:
    """Map official URDF visual triangles through SDK-authoritative link frames."""
    result: list[tuple[FloatArray, str]] = []
    opening = float(np.clip(gripper_position, 0.0, 1.0))
    jaw_angle = STOCK_JAW_LIMITS[0] + opening * (STOCK_JAW_LIMITS[1] - STOCK_JAW_LIMITS[0])
    jaw_transform = (
        link_frames["gripper_link"]
        @ STOCK_JAW_JOINT.origin_transform
        @ rotation_about_axis(np.asarray(STOCK_JAW_JOINT.axis), jaw_angle)
    )
    for definition, local in _static_visuals():
        link = str(definition["link"])
        if link not in link_frames and link != "moving_jaw_so101_v1_link":
            continue
        parent = jaw_transform if link == "moving_jaw_so101_v1_link" else link_frames[link]
        transform = parent @ local
        triangles = _triangles(str(definition["mesh"]), low_detail)
        world = triangles @ transform[:3, :3].T + transform[:3, 3]
        result.append((world, str(definition["material"])))
    return result


@lru_cache(maxsize=16)
def _local_bounds(filename: str) -> FloatArray:
    vertices = _triangles(filename).reshape((-1, 3))
    lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    return np.asarray(list(product(*zip(lower, upper, strict=True))), dtype=float)


def world_bounds_points(
    link_frames: dict[str, FloatArray], gripper_position: float = 0.5
) -> FloatArray:
    """Conservative transformed bounds for each full-resolution visual mesh."""
    opening = float(np.clip(gripper_position, 0.0, 1.0))
    angle = STOCK_JAW_LIMITS[0] + opening * (STOCK_JAW_LIMITS[1] - STOCK_JAW_LIMITS[0])
    jaw = (
        link_frames["gripper_link"] @ STOCK_JAW_JOINT.origin_transform
        @ rotation_about_axis(np.asarray(STOCK_JAW_JOINT.axis), angle)
    )
    chunks = []
    for definition, local in _static_visuals():
        link = str(definition["link"])
        if link not in link_frames and link != "moving_jaw_so101_v1_link":
            continue
        parent = jaw if link == "moving_jaw_so101_v1_link" else link_frames[link]
        transform = parent @ local
        corners = _local_bounds(str(definition["mesh"]))
        chunks.append(corners @ transform[:3, :3].T + transform[:3, 3])
    return np.concatenate(chunks)

