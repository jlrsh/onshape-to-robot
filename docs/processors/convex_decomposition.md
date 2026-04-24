# `ProcessorConvexDecomposition` (CoACD)

Replaces each collision mesh with several convex hulls produced by
[CoACD](https://github.com/SarahWeiii/CoACD). Most physics engines run
collision queries much faster against convex hulls than arbitrary
triangle soup.

## `o2r.json`

```javascript
{
    "convex_decomposition": true,
    "rainbow_colors": false
}
```

- `convex_decomposition` *(default: false)*.
- `rainbow_colors` *(default: false)* — replaces each hull's color with a
  random RGBA. Handy when visualizing hulls via
  `collisions_as_visual`.

## Behavior

For every part with a single collision mesh:

1. Compute SHA-1 of the mesh bytes.
2. Look up `~/.cache/onshape-to-robot-convex-decomposition/{hash}.pkl`.
3. On cache miss, run `coacd.run_coacd(mesh, max_convex_hull=16)` and
   pickle the result.
4. Export each hull to
   `{assets}/convex_decomposition/{part_name}_{i:05d}.stl` (or `.glb`).
5. Append one new collision `Mesh` per hull to `part.meshes`.
6. Set the original mesh's `collision=False`.
7. Call `part.prune_unused_geometry()`.

Parts with multiple collision meshes are skipped with a warning.

## Notes

- `is_safe = False`. Disabled under `--safe`.
- Depends on `coacd` and `trimesh`. Raises with a `pip install` tip if
  missing.
- `max_convex_hull` is hard-coded to 16.
- Changes `cwd` to `config.output_directory` for the run (restored on
  exit).
- Should run **after** merge/simplify so it operates on the final
  collision mesh of each part.
