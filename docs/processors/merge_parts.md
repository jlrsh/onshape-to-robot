# `ProcessorMergeParts`

Combines every part in a link into a single merged mesh/shape bundle,
expressed in the link's center-of-mass frame. Supports STL and GLB, and
can be restricted to visual-only or collision-only.

## `o2r.json`

```javascript
{
    "merge_stls": true
}
```

- `merge_stls` *(default: false)* — `true`, `"visual"`, or `"collision"`.
  `"visual"` / `"collision"` merge only meshes of that role and keep the
  other role untouched.
- `mesh_format` *(default: `"stl"`)* — controls which
  [adapter](../modules/mesh_adapter.md) is used (STL via `numpy-stl`, GLB
  via `trimesh`). Inherited from the global config.
- `collisions_as_visual` *(default: false)* — when true, the merged
  collision mesh is reused as the visual mesh (filename drops the
  `_collision` suffix).

## Behavior

For each link:

1. Compute `T_world_com` using `Link.get_dynamics()`; all merged geometry
   is reframed into this pose.
2. For each role being merged, accumulate part meshes:
    - STL path: `accumulate_meshes()` loads each STL via the adapter,
      transforms into the COM frame, concatenates into one mesh.
    - GLB path: `accumulate_meshes_glb()` groups geometries by
      `material.baseColorFactor` so each color remains a distinct scene
      entry — preserves per-face color without exploding draw calls.
3. Write the merged mesh to `{assets}/{link_name}[_collision].{ext}`.
4. Write a sidecar `{link_name}[_collision].merged.json` manifest listing
   every source part (doc/element/microversion/config/partId) — used for
   diff-ability across runs.
5. Compute a weighted-average color (weighted by part mass) and a merged
   shape list by transforming each shape through its part frame.
6. Replace `link.parts` with a single massless `Part` named
   `{link}_parts` carrying the merged mesh(es) and shape list. When only
   one role is being merged, the other role's original parts are kept
   alongside (so dynamics are still derived from the original parts).
7. Call `cleanup_merged_sources(...)` to delete the original per-part
   mesh files — scoped carefully: only STL/GLB files, only inside
   `config.asset_path("")` (path-traversal check).

## Notes

- `is_safe = True` (no shell out; only writes inside the assets dir).
- Changes `cwd` to `config.output_directory` for the run (restored on
  exit).
- Must run **before** [`simplify_stls`](simplify_stls.md) — simplify runs
  on the merged file.
- For GLB output, materials are preserved via per-color scene entries;
  downstream viewers (MuJoCo, rviz) render them as distinct submeshes.

## Useful subroutines

- `collect_mesh_files(robot)` — set of every mesh filename the robot
  references.
- `reduce_faces(filename, reduction)` — proxy to the mesh adapter's
  decimation; used by the Simplify processor when merge_parts is
  composed with it.
