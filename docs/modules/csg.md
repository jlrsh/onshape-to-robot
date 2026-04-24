# `csg.py` — OpenSCAD CSG parsing

Glue used by [`ProcessorScad`](../processors/scad.md): runs `openscad` on
a `.scad` file, parses the resulting `.csg` intermediate, extracts pure
shapes with their transforms. The CSG format is declarative — no loops,
no variables, all geometry is expanded.

## Entry point

### `process(filename: str, dilatation: float) -> list[dict]`

1. `os.system(f"openscad {filename} -o _tmp_data.csg")`.
2. Reads `_tmp_data.csg`, calls `parse_csg(data, dilatation)`.
3. `os.system("rm _tmp_data.csg")`.
4. Returns the shape list.

## Parser internals

### `multmatrix_parse(params) -> np.ndarray`

Parses `multmatrix([[...], [...], [...], [...]])`. Returns a 4×4 matrix
with translation columns divided by 1000 (mm → m).

### `cube_parse(params, dilatation) -> (size, center)`

Parses `cube(size = [sx, sy, sz], center = true|false)`. `size` is
returned in meters, expanded by `dilatation` on each axis.

### `cylinder_parse(params, dilatation) -> ((h, r), center)`

Parses `cylinder(h = H, r1 = R, r2 = R, center = …)`. Returns
`(height + dilatation, radius + dilatation/2)` in meters.

### `sphere_parse(params, dilatation) -> float`

Parses `sphere(r = R)`. Returns `R + dilatation` in meters.

### `extract_node_parameters(line) -> (node, params)`

Splits a CSG line into the node name and its parameter string. Accepts
both `node(...)` terminal statements and `node(...) {` block openers.

### `T(x, y, z) -> np.ndarray`

4×4 translation matrix.

### `parse_csg(data: str, dilatation: float) -> list[dict]`

Line-by-line parser. Maintains a stack for nested `multmatrix {...}`
blocks (each `{` pushes, each `}` pops). Every extracted shape is
emitted as:

```python
{
    "type": "cube" | "cylinder" | "sphere",
    "parameters": <size | (h, r) | r>,
    "transform": <4x4 matrix>,
}
```

Non-centered shapes have their half-extent baked into the transform so
the downstream `Shape` is always implicitly centered.
