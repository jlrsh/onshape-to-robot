# `ProcessorSimplifySTLs`

Keeps mesh file sizes under a budget. Three strategies are available:
quadric decimation, voxel remeshing, or alpha-wrap shrink wrapping.

## `o2r.json`

```javascript
{
    "simplify_stls": true,
    "max_stl_size": 3,
    "simplify_strategy": "decimate",

    // decimate
    "decimate_quality_threshold": 0.5,
    "decimate_preserve_normal": true,
    "decimate_preserve_topology": false,
    "decimate_preserve_boundary": false,
    "decimate_planar_quadric": true,

    // voxel
    "voxel_pitch": null,
    "voxel_resolution": 128,
    "voxel_post_decimate": true,

    // alpha
    "alpha": null,
    "alpha_relative": 0.02,
    "alpha_post_decimate": true
}
```

- `simplify_stls` *(default: true)* — `true`, `"visual"`, or
  `"collision"`.
- `max_stl_size` *(default: 3)* — in MB. Meshes larger than this are
  simplified; the quadric decimation aims for `max_stl_size / current_size`.
- `simplify_strategy` *(default: `"decimate"`)* — `"decimate"`,
  `"voxel"`, or `"alpha"`.

### Strategy choice

| Strategy | Best for | Trade-off |
|----------|----------|-----------|
| `decimate` | Clean outer shells | Keeps interior geometry |
| `voxel` | Unions of many pieces / self-intersections | Watertight shell; discards interiors |
| `alpha` | Outer shape with sharper features preserved | Slower than voxel |

### Decimate tunables

- `decimate_quality_threshold` *(default: 0.5)* — MeshLab's `qualitythr`.
- `decimate_preserve_normal` *(default: true)*.
- `decimate_preserve_topology` *(default: false)* — disallow
  hole/handle changes when true.
- `decimate_preserve_boundary` *(default: false)* — pin boundary loops.
- `decimate_planar_quadric` *(default: true)* — extra weight on planar
  regions.

### Voxel tunables

- `voxel_pitch` *(default: null)* — edge length in meters. When null, derived
  from `voxel_resolution` and the mesh's bbox diagonal.
- `voxel_resolution` *(default: 128)*.
- `voxel_post_decimate` *(default: true)* — run quadric decimation after
  the voxel remesh to hit `max_stl_size`.

### Alpha tunables

- `alpha` *(default: null)* — alpha radius in meters. When null, derived
  from `alpha_relative × bbox_diagonal`.
- `alpha_relative` *(default: 0.02)*.
- `alpha_post_decimate` *(default: true)*.

## Requirements

```bash
pip install pymeshlab
```

GLB input additionally needs `trimesh` (usually already installed).

## Behavior

For every mesh in every part, if it matches the selection
(`visual` / `collision` / both):

1. Skip if file is smaller than `max_stl_size`.
2. Compute `target_reduction = max_stl_size / current_size`.
3. Dispatch to the selected strategy:
    - `decimate` → pymeshlab quadric edge-collapse on the file in place.
    - `voxel` → voxelize + marching cubes; optional post-decimation.
    - `alpha` → shrink-wrap (tries `generate_alpha_wrap` first, falls
      back to `generate_alpha_shape`); optional post-decimation.
4. GLB: per-geometry decimation, reattaching original visual data so
   materials and colors are preserved.

## Notes

- `is_safe = True` — runs entirely in-process.
- `is_safe` is fine because the only external dependency is a Python
  package; no subprocess.
- Must run **after** [`merge_parts`](merge_parts.md) so it simplifies the
  merged output.
- Alpha and voxel strategies discard interior geometry on purpose. If you
  care about internal features, stick to `decimate`.
