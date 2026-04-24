# onshape-to-robot — Knowledge Base

This directory is the canonical reference for the `onshape-to-robot` codebase.
It is organized for navigation by both humans and LLMs: each file covers a
focused slice of the system, with function-level detail on signatures,
side-effects and data shapes.

## Quick start (users)

- [Getting started](getting_started.md) — install, API keys, first export.
- [Design-time conventions](design.md) — how to annotate an Onshape assembly
  (mate connector naming, frames, kinematic loops, gears).
- [Configuration (`o2r.json`)](config.md) — every top-level option.
- [Kinematic loops](kinematic_loops.md) — modelling closed chains.
- [Cache](cache.md) — how request caching works and how to clear it.

## Pipeline (architecture)

```
Onshape API ──► Assembly ──► RobotBuilder ──► Robot (IR) ──► Processors ──► Exporter ──► URDF / SDF / MuJoCo
                (fetch)      (build links,   (Link/Joint/   (tweak geom,     (write XML)
                             parts, frames)  Part/Closure)  split, merge…)
```

See [architecture.md](architecture.md) for the full pipeline walkthrough with
call sites and data handoffs.

## Modules (reference)

### Core pipeline

- [`assembly.md`](modules/assembly.md) — Onshape assembly fetch + mate
  interpretation. Builds the DOF graph, frames, closures, relations and
  spanning tree.
- [`robot.md`](modules/robot.md) — Intermediate representation: `Robot`,
  `Link`, `Joint`, `Part`, `Closure`, `Relation`.
- [`robot_builder.md`](modules/robot_builder.md) — Walks the assembly tree,
  downloads meshes, resolves colors/dynamics, produces a `Robot`.
- [`geometry.md`](modules/geometry.md) — Geometry primitives: `Mesh`, `Box`,
  `Cylinder`, `Sphere`, base `Shape`/`Geometry`.
- [`config.md`](modules/config.md) — Config loader (contiguous inheritance +
  variants), snake/camel resolution, processor registration.
- [`export.md`](modules/export.md) — `onshape-to-robot` CLI entry point.

### Exporters

- [`exporters.md`](modules/exporters.md) — Base `Exporter` + XML DOM
  pretty-printing.
- [`exporter_urdf.md`](modules/exporter_urdf.md) — URDF exporter.
- [`exporter_sdf.md`](modules/exporter_sdf.md) — SDF exporter.
- [`exporter_mujoco.md`](modules/exporter_mujoco.md) — MuJoCo exporter.
- [`exporter_utils.md`](modules/exporter_utils.md) — Shared XML helpers,
  `frame_x_forward` convention, RPY conversion.

### Processors

- [`processors.md`](processors.md) — Pipeline overview, ordering,
  registration and how to write a custom one.
- [`ball_to_euler.md`](processors/ball_to_euler.md)
- [`collision_as_visual.md`](processors/collision_as_visual.md)
- [`convex_decomposition.md`](processors/convex_decomposition.md)
- [`dummy_base_link.md`](processors/dummy_base_link.md)
- [`fixed_links.md`](processors/fixed_links.md)
- [`merge_parts.md`](processors/merge_parts.md)
- [`no_collision_meshes.md`](processors/no_collision_meshes.md)
- [`scad.md`](processors/scad.md)
- [`simplify_stls.md`](processors/simplify_stls.md)

### Onshape API client

- [`onshape_api.md`](modules/onshape_api.md) — High-level `Client`,
  low-level HMAC signer, on-disk request cache, logging utils.

### Mesh I/O & math

- [`mesh_adapter.md`](modules/mesh_adapter.md) — STL/GLB adapter factory
  used by merge/simplify processors.
- [`glb_io.md`](modules/glb_io.md) — Centralized GLB read/write (normals
  forced on export).
- [`math_utils.md`](modules/math_utils.md) — `normalize_angle_pi`.
- [`expression.md`](modules/expression.md) — Onshape parametric expression
  evaluator (with unit suffixes).
- [`csg.md`](modules/csg.md) — OpenSCAD CSG parsing for the SCAD processor.

### CLI entry points

- [`cli.md`](modules/cli.md) — `onshape-to-robot`, `-bullet`, `-mujoco`,
  `-clear-cache`, `-edit-shape`, `-pure-shape`.

### Other utilities

- [`message.md`](modules/message.md) — colorized console output helpers.
- [`simulation.md`](modules/simulation.md) — PyBullet simulation wrapper.

## Conventions used in this knowledge base

- Signatures are written in Python syntax. Types come from the source where
  provided and from inference otherwise.
- When a function reads config keys, they are listed inline.
- "Side effects" calls out network I/O, disk writes, cache reads/writes,
  subprocess calls, and any in-place mutation of arguments.
- "Ordering" flags where a processor or helper must run before/after another.
- Line numbers are intentionally avoided — they rot. When a specific function
  is needed, grep for its name.

## External references

- [`onshape-to-robot` on PyPI](https://pypi.org/project/onshape-to-robot/)
- [Robots examples repo](https://github.com/rhoban/onshape-to-robot-examples/)
- [Video tutorial](https://www.youtube.com/watch?v=C8oK4uUmbRw) (some parts
  outdated)
