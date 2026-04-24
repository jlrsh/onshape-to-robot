# `ProcessorScad` (OpenSCAD pure-shape approximation)

Replaces collision meshes with pure shapes authored in OpenSCAD. Each
mesh `foo.stl` that has a sibling `foo.scad` is compiled with OpenSCAD
into a CSG intermediate, then parsed to extract `Box` / `Cylinder` /
`Sphere` primitives.

## `o2r.json`

```javascript
{
    "use_scads": true,
    "pure_shape_dilatation": 0.0
}
```

- `use_scads` *(default: false)*.
- `pure_shape_dilatation` *(default: 0.0)* — uniform expansion (meters)
  applied to every primitive. Negative values shrink.

## Requirements

- OpenSCAD binary on `$PATH`. On Debian/Ubuntu: `sudo apt-get install openscad`.

## Authoring workflow

For each collision STL you want to approximate:

```bash
onshape-to-robot-edit-shape path/to/part.stl
```

opens the matching `.scad`, pre-populated with the STL import reference
(see [cli.md](../modules/cli.md)). Add `cube(...)`, `cylinder(...)` or
`sphere(...)` primitives; save; re-run `onshape-to-robot`.

## Behavior

For every collision mesh in every part:

1. If a sibling `.scad` exists, invoke `openscad {scad} -o _tmp_data.csg`.
2. Parse the CSG output (see [csg.md](../modules/csg.md)). Units are
   converted from millimeters to meters; matrix stacks for
   `multmatrix {...}` are honored.
3. Append one `Shape` per extracted primitive to the part.
4. Set the original mesh's `collision = False`.
5. `part.prune_unused_geometry()`.

## Notes

- `is_safe = False` — shells out via `os.system`. Disabled under
  `--safe`.
- Changes `cwd` to `config.output_directory` for the run.
- Must run **before** [`merge_parts`](merge_parts.md) so the replacement
  shapes are part of the merge (and so the stl-backed collision meshes
  are gone by the time merging happens).
