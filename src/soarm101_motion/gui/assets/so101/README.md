# SO-101 visual mesh asset record

- Upstream project: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
- Exact Git revision: `385e8d7c68e24945df6c60d9bd68837a4b7411ae`
- Source URDF: `Simulation/SO101/so101_new_calib.urdf`
- Source meshes: `Simulation/SO101/assets/*.stl`
- License: see `LICENSE.upstream` (Apache License 2.0).

`so101_new_calib.upstream.urdf` retains the pinned source URDF. `visuals.json`
contains every visual's source link, mesh filename, origin XYZ/RPY,
and material from the pinned source URDF. Its mappings are checked against that
URDF in the mesh-renderer tests. The moving jaw is transformed at render time
from the SDK stock-jaw joint origin, axis, and nominal limits, with normalized
gripper state mapped from 0 (closed) to 1 (open).

The `_lod.stl` binary meshes are derivatives of the exact pinned upstream files,
generated with Blender 5.2.2's collapse-mode quadric decimator at ratio 0.05
(see `scripts/generate_gui_mesh_lod.py`). The modifier collapses connected mesh
edges while retaining connected surface topology; no facet-stride or random
facet dropping is used. The GUI uses these derivatives only while the user is
dragging the view and returns to the full upstream meshes on release. It draws
every retained triangle and caches immutable local vertex arrays. These meshes
are presentation references only. They do not replace SDK FK, calibration,
workspace checks, or measured collision data.
