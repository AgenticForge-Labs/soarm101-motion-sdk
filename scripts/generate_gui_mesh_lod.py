"""Run with Blender: blender -b --python scripts/generate_gui_mesh_lod.py -- <asset-dir>"""

import bpy
import sys
from pathlib import Path


asset_dir = Path(sys.argv[-1]).resolve()
for source in sorted(asset_dir.glob("*.stl")):
    if source.stem.endswith("_lod"):
        continue
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    bpy.ops.wm.stl_import(filepath=str(source))
    mesh_object = bpy.context.selected_objects[0]
    if len(mesh_object.data.polygons) > 1000:
        modifier = mesh_object.modifiers.new("Offline quadric visual LOD", "DECIMATE")
        modifier.decimate_type = "COLLAPSE"
        modifier.ratio = 0.05
        modifier.use_collapse_triangulate = True
        bpy.context.view_layer.objects.active = mesh_object
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    destination = source.with_name(f"{source.stem}_lod.stl")
    bpy.ops.wm.stl_export(
        filepath=str(destination),
        ascii_format=False,
        global_scale=1.0,
        use_scene_unit=False,
        forward_axis="Y",
        up_axis="Z",
    )
