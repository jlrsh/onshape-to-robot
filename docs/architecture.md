# Architecture

End-to-end pipeline of a single `onshape-to-robot` invocation.

```
                     ┌──────────────────┐
                     │  o2r.json        │
                     │  (+ ancestors,   │
                     │   + variants)    │
                     └────────┬─────────┘
                              │  Config(robot_path, safe)
                              ▼
        ┌───────────────────────────────────────────┐
        │ (1) Retrieval                             │
        │                                           │
        │ Assembly (assembly.py)                    │
        │  ├─ resolve workspace/version             │
        │  ├─ locate assembly element               │
        │  ├─ check/normalise configuration         │
        │  ├─ GET assembly, features, matevalues    │
        │  ├─ process mates                         │
        │  │    · dof_   → DOF                      │
        │  │    · fix_   → body merges              │
        │  │    · frame_ → Frame                    │
        │  │    · closing_ → closure frames         │
        │  │    · link_  → name override            │
        │  ├─ build spanning trees                  │
        │  └─ collect relations (gears)             │
        │                                           │
        │ RobotBuilder (robot_builder.py)           │
        │  ├─ walk tree_children                    │
        │  ├─ fetch mesh (STL or GLB)               │
        │  ├─ fetch color via metadata              │
        │  ├─ fetch dynamics (mass/COM/inertia)     │
        │  └─ populate Robot (robot.py)             │
        └───────────────────────┬───────────────────┘
                                │  Robot (intermediate
                                │   representation)
                                ▼
        ┌───────────────────────────────────────────┐
        │ (2) Processors                            │
        │                                           │
        │ config.processors  (default list or       │
        │  custom list from o2r.json)               │
        │                                           │
        │ applied in order; each mutates the Robot  │
        │ (see processors.md)                       │
        └───────────────────────┬───────────────────┘
                                │  Robot (tweaked)
                                ▼
        ┌───────────────────────────────────────────┐
        │ (3) Export                                │
        │                                           │
        │ Exporter.write_xml(robot, filename)       │
        │   build() → XML string → minidom          │
        │   prettyprint → file on disk              │
        │                                           │
        │ URDF ──► .urdf                            │
        │ SDF  ──► .sdf + model.config              │
        │ MuJoCo ─► .xml + scene.xml (first run)    │
        └───────────────────────┬───────────────────┘
                                │
                                ▼
                    post_import_commands
```

## CLI flags

The CLI (`export.py`) splits the pipeline into stages:

| Flag             | Retrieval | Processors | Export |
|------------------|-----------|------------|--------|
| (default)        | ✅        | ✅         | ✅     |
| `--retrieve`     | ✅        | –          | –      |
| `--convert`      | (load pkl)| ✅         | ✅     |
| `--save-pickle`  | ✅ + pkl  | ✅         | ✅     |
| `--safe`         | ✅        | default only, no custom imports | ✅, no post-import |

`robot.pkl` is written/read at `{output_directory}/robot.pkl`.

## Data handoffs

- **`Config` → `Assembly`**: passes `document_id`, `workspace_id`/`version_id`,
  `element_id`, `configuration`, `assembly_name`, `draw_frames`,
  `ignore_limits`.
- **`Assembly` → `RobotBuilder`**: exposes `root_nodes`, `tree_children`,
  `link_names`, `frames`, `closures`, `relations`, `body_instance`,
  `body_occurrences`, `get_dof`, `client`, `is_occurrence_hidden`.
- **`RobotBuilder` → `Processors`**: emits `Robot` with `Link`, `Joint`,
  `Part`, `Closure`, and `Relation` (attached to joints).
- **`Processors` → `Exporter`**: same `Robot` object, mutated in place.
- **`Exporter` → disk**: pretty-printed XML file; for SDF a sibling
  `model.config`; for MuJoCo a sibling `scene.xml` unless one exists.

## Identifiers

- **Body ID** (`int`): local counter assigned by `Assembly.make_body`.
  Internal only — never leaks into the exported model.
- **Link name** (`str`): slugified instance name, optionally overridden via
  a `link_<name>` mate, disambiguated with `_2`, `_3`, … when collisions
  occur.
- **Part name** (`str`): slugified Onshape part name, plus a configuration
  suffix (or MD5 if long) when `include_configuration_suffix=true`.
- **Joint name** (`str`): from the `dof_<name>` mate.
- **Frame name** (`str`): from `frame_<name>` or `closing_<name>` mates.

## Caches

- `~/.cache/onshape-to-robot/` — Onshape API responses (`onshape_api/cache.py`).
  Cleared with `onshape-to-robot-clear-cache`.
- `~/.cache/onshape-to-robot-convex-decomposition/` — CoACD output, keyed by
  SHA-1 of source mesh bytes (processor_convex_decomposition).
- Per-run STL/GLB files live in `{output_dir}/{assets_directory}/` alongside
  sidecar `.part` JSON files (metadata used to fetch them) and, for merges,
  per-link `.merged.json` manifests.
