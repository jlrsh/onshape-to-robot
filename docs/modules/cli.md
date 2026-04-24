# CLI entry points

All CLI scripts live at the top of the package; `pyproject.toml` wires
them up as console-script entry points.

| Command                           | Module                                   | Purpose |
|-----------------------------------|------------------------------------------|---------|
| `onshape-to-robot`                | `onshape_to_robot.export:main`           | Main pipeline. See [`export.md`](export.md). |
| `onshape-to-robot-bullet`         | `onshape_to_robot.bullet:main`           | PyBullet viewer + joint sliders. |
| `onshape-to-robot-mujoco`         | `onshape_to_robot.mujoco:main`           | MuJoCo passive viewer. |
| `onshape-to-robot-clear-cache`    | `onshape_to_robot.clear_cache:main`      | `rm -rf ~/.cache/onshape-to-robot/`. |
| `onshape-to-robot-edit-shape`     | `onshape_to_robot.edit_shape:main`       | Open/create a `.scad` file for a mesh. |
| `onshape-to-robot-pure-shape`     | `onshape_to_robot.pure_sketch:main`      | Convert Onshape sketches into a `.scad` draft. |

## `onshape-to-robot`

See [`export.md`](export.md).

## `onshape-to-robot-bullet`

Loads a URDF into PyBullet and exposes every joint as a slider.

```
onshape-to-robot-bullet [options] <directory>
```

- `directory` — folder containing `robot.urdf`.
- `--fixed` / `-f` — fix the base.
- `--no-self-collisions` / `-n` — disable self-collision detection.
- `-x`, `-y`, `-z` — initial base offset (meters).

Per frame, reads slider values, applies them via PyBullet joint motor
control, prints frame poses and the COM position. Joints whose name ends
with `_speed` switch to velocity control.

## `onshape-to-robot-mujoco`

Loads a MuJoCo XML into a passive viewer.

```
onshape-to-robot-mujoco [options] <directory>
```

- `directory` — folder containing `scene.xml` (falls back to `robot.xml`).
- `--sim` — run the physics engine (default is viewer only).
- `--x`, `--y`, `--z` — initial position (defaults z=0.5).

## `onshape-to-robot-clear-cache`

Clears `~/.cache/onshape-to-robot/` via
`shutil.rmtree(ignore_errors=True)`. Also see [`cache.md`](../cache.md).

## `onshape-to-robot-edit-shape`

```
onshape-to-robot-edit-shape <path_to_stl>
```

Creates the matching `.scad` file if it doesn't exist, seeded with an
import of the STL and commented primitive stubs. Then launches OpenSCAD.

## `onshape-to-robot-pure-shape`

```
onshape-to-robot-pure-shape <path_to_stl> [prefix=PureShapes]
```

Reads the sidecar `.part` metadata, fetches matching sketches from
Onshape, extracts extrusion thickness from the sketch name (e.g.
`"PureShapes 5.3"` → 5.3 mm), and generates a `.scad` with `cylinder`
and `cube` primitives. Used when you want OpenSCAD to start from an
Onshape sketch instead of a blank slate.
