# `glb_io.py` — GLB I/O

Centralizes trimesh imports and forces normals to be exported. Without
`include_normals=True`, trimesh silently skips the `NORMAL` attribute
when it's not already attached, giving flat shading in downstream
viewers (rviz, MuJoCo).

## `load_glb(path: str) -> trimesh.Scene | trimesh.Trimesh`

Imports `trimesh` lazily and loads the file. Returns whatever trimesh
returns — a `Scene` (multi-geometry) or a `Trimesh` (single mesh). The
processors branch on the returned type.

## `export_glb(scene_or_mesh, path: str) -> None`

Calls `obj.export(path, file_type="glb", include_normals=True)`. Used by
`robot_builder._fetch_glb` (merged GLB output) and
`mesh_adapter.GLBAdapter.save`.
