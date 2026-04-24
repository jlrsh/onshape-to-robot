# `mesh_adapter.py` — STL/GLB I/O abstraction

Uniform interface for loading, transforming, saving and combining meshes,
so processors (merge, simplify) can be format-agnostic.

## `mesh_adapter_for(mesh_format: str) -> STLAdapter | GLBAdapter`

- `"glb"` → `GLBAdapter()`.
- anything else → `STLAdapter()`.

## `STLAdapter`

```python
extension = ".stl"
```

Backend: `numpy-stl` (`stl.mesh.Mesh`).

| Method | Behavior |
|--------|----------|
| `load(filename) -> stl.mesh.Mesh` | `Mesh.from_file(...)`. |
| `save(mesh_data, filename)` | Monkey-patches `mesh_data.get_header()` to return a **deterministic** 80-byte header and writes binary STL. Ensures repeated runs produce byte-identical output (useful for Git diffs / caching). |
| `transform(mesh_data, matrix)` | In place. Applies rotation `matrix[:3,:3]` to `v0/v1/v2/normals` and translation `matrix[:3,3]` to vertices only. |
| `combine(m1, m2) -> stl.mesh.Mesh` | `Mesh(np.concatenate([m1.data, m2.data]))`. |

## `GLBAdapter`

```python
extension = ".glb"
```

Backend: `trimesh`.

| Method | Behavior |
|--------|----------|
| `load(filename) -> trimesh.Trimesh` | `trimesh.load(filename, force="mesh")` — merges scene geometry into one `Trimesh`. |
| `save(mesh_data, filename)` | Delegates to `glb_io.export_glb` (forces NORMAL attribute). |
| `transform(mesh_data, matrix)` | In place via `mesh_data.apply_transform(matrix)`. |
| `combine(m1, m2) -> trimesh.Trimesh` | `trimesh.util.concatenate([m1, m2])`. |

Both adapters expose the same method signatures so
`ProcessorMergeParts` / `ProcessorSimplifySTLs` can stay polymorphic.
