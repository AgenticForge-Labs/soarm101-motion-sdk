from __future__ import annotations

from importlib.resources import files
from itertools import product
import xml.etree.ElementTree as ET
from pathlib import PurePosixPath

import numpy as np

from soarm101_motion.gui.mesh_renderer import (
    _definitions, _triangles, world_bounds_points, world_triangles,
)
from soarm101_motion.kinematics import SO101KinematicModel
from soarm101_motion.constants import HOME_JOINTS
from soarm101_motion.kinematics.model import STOCK_JAW_JOINT, STOCK_JAW_LIMITS, rotation_about_axis
from soarm101_motion.kinematics.transforms import transform_from_xyz_rpy


def test_packaged_mesh_assets_include_upstream_attribution_and_visuals() -> None:
    assets = files("soarm101_motion.gui").joinpath("assets/so101")
    assert assets.joinpath("LICENSE.upstream").is_file()
    assert assets.joinpath("so101_new_calib.upstream.urdf").is_file()
    definitions = _definitions()
    assert len(definitions) >= 15
    for definition in definitions:
        triangles = _triangles(str(definition["mesh"]))
        assert triangles.shape[1:] == (3, 3)
        assert not triangles.flags.writeable
    full_faces = sum(len(_triangles(str(item["mesh"]))) for item in definitions)
    drag_faces = sum(len(_triangles(str(item["mesh"]), True)) for item in definitions)
    assert drag_faces < full_faces


def test_visual_manifest_matches_every_pinned_urdf_mesh_transform_and_material() -> None:
    assets = files("soarm101_motion.gui").joinpath("assets/so101")
    root = ET.fromstring(assets.joinpath("so101_new_calib.upstream.urdf").read_text())
    expected = []
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find(".//mesh")
            origin = visual.find("origin")
            material = visual.find("material")
            assert mesh is not None and origin is not None and material is not None
            expected.append((
                link.attrib["name"], PurePosixPath(mesh.attrib["filename"]).name,
                tuple(float(v) for v in origin.attrib["xyz"].split()),
                tuple(float(v) for v in origin.attrib["rpy"].split()),
                material.attrib["name"],
            ))
    actual = [(
        item["link"], item["mesh"], tuple(item["xyz"]), tuple(item["rpy"]), item["material"]
    ) for item in _definitions()]
    assert actual == expected


def test_mesh_renderer_uses_sdk_link_frames_and_keeps_pose_inside_view_envelope() -> None:
    model = SO101KinematicModel()
    poses = [model.vector(HOME_JOINTS)]
    poses.extend(
        np.array([model.upper_bounds[i] if bit else model.lower_bounds[i]
                  for i, bit in enumerate(bits)])
        for bits in product((False, True), repeat=len(model.joint_names))
    )
    for pose in poses:
        for opening in (0.0, 1.0):
            meshes = world_triangles(model.link_frames(pose), opening)
            assert meshes
            points = np.concatenate([triangles.reshape(-1, 3) for triangles, _ in meshes])
            assert np.isfinite(points).all()
            assert float(np.max(np.abs(points[:, :2]))) < 0.56
            assert float(np.max(np.abs(points[:, 2]))) < 0.56


def test_moving_jaw_mesh_uses_stock_joint_mapping_at_closed_and_open_positions() -> None:
    model = SO101KinematicModel()
    pose = model.vector(HOME_JOINTS)
    frames = model.link_frames(pose)
    jaw_definition = next(item for item in _definitions()
                          if item["link"] == "moving_jaw_so101_v1_link")
    local = _triangles(str(jaw_definition["mesh"]))
    visual = transform_from_xyz_rpy(jaw_definition["xyz"], jaw_definition["rpy"])
    local = local @ visual[:3, :3].T + visual[:3, 3]
    jaw_meshes = []
    for opening in (0.0, 1.0):
        meshes = world_triangles(frames, opening)
        jaw = meshes[-1][0]
        angle = STOCK_JAW_LIMITS[0] + opening * (STOCK_JAW_LIMITS[1] - STOCK_JAW_LIMITS[0])
        transform = (frames["gripper_link"] @ STOCK_JAW_JOINT.origin_transform
                     @ rotation_about_axis(np.asarray(STOCK_JAW_JOINT.axis), angle))
        expected = local @ transform[:3, :3].T + transform[:3, 3]
        assert np.allclose(jaw, expected)
        jaw_meshes.append(jaw)
        lod_jaw = world_triangles(frames, opening, low_detail=True)[-1][0]
        lod_local = _triangles(str(jaw_definition["mesh"]), True)
        lod_local = lod_local @ visual[:3, :3].T + visual[:3, 3]
        assert np.allclose(lod_jaw, lod_local @ transform[:3, :3].T + transform[:3, 3])
    assert not np.allclose(jaw_meshes[0], jaw_meshes[1])


def test_mesh_bounds_points_conservatively_enclose_full_geometry() -> None:
    model = SO101KinematicModel()
    for joints in (model.vector(HOME_JOINTS), model.lower_bounds, model.upper_bounds):
        frames = model.link_frames(joints)
        for opening in (0.0, 1.0):
            bounds = world_bounds_points(frames, opening)
            vertices = np.concatenate([
                mesh.reshape((-1, 3)) for mesh, _ in world_triangles(frames, opening)
            ])
            assert np.all(vertices.min(axis=0) >= bounds.min(axis=0) - 1e-12)
            assert np.all(vertices.max(axis=0) <= bounds.max(axis=0) + 1e-12)


def test_vectorized_projection_matches_scalar_projection() -> None:
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui.cartesian_view import CartesianArmView

    app = QApplication.instance() or QApplication([])
    view = CartesianArmView()
    view.resize(480, 480)
    view._yaw, view._pitch = 0.7, -0.35
    view._projection_center = (0.12, -0.04)
    view._projection_scale = 600.0
    points = np.asarray([[0.1, 0.2, -0.3], [-0.4, 0.05, 0.2]])
    x, y = view._project_vertices(points)
    for index, point in enumerate(points):
        scalar = view._project(point)
        assert np.allclose([x[index], y[index]], [scalar.x(), scalar.y()])
    view.close()
    app.quit()


def test_mesh_cache_signature_tracks_geometry_view_palette_and_tcp() -> None:
    from PySide6.QtGui import QColor
    from PySide6.QtWidgets import QApplication
    from soarm101_motion.gui.cartesian_view import CartesianArmView
    from soarm101_motion.types import Pose

    app = QApplication.instance() or QApplication([])
    view = CartesianArmView()
    base = QColor(12, 34, 56, 78)  # Include blue and alpha in the cache key.
    original = view._mesh_cache_signature(view._joints, 0.4, base)
    changed_joints = dict(view._joints)
    changed_joints[view._model.joint_names[0]] += 0.01
    assert original != view._mesh_cache_signature(changed_joints, 0.4, base)
    assert original != view._mesh_cache_signature(view._joints, 0.5, base)
    view._yaw += 0.1
    assert original != view._mesh_cache_signature(view._joints, 0.4, base)
    view._yaw -= 0.1
    view.resize(481, 480)
    assert original != view._mesh_cache_signature(view._joints, 0.4, base)
    view.resize(480, 480)
    assert original != view._mesh_cache_signature(view._joints, 0.4, QColor(12, 34, 57, 78))
    assert original != view._mesh_cache_signature(view._joints, 0.4, QColor(12, 34, 56, 79))
    view._interactive_detail = True
    assert original != view._mesh_cache_signature(view._joints, 0.4, base)
    view._interactive_detail = False
    view._zoom = 1.2
    assert original != view._mesh_cache_signature(view._joints, 0.4, base)
    assert original != view._mesh_cache_signature(
        view._joints, 0.4, base, device_pixel_ratio=2.0
    )
    view._model.tcp = Pose(np.asarray([0.01, 0, 0]), np.eye(3))
    assert original != view._mesh_cache_signature(view._joints, 0.4, base)
    view.close()
    app.quit()
