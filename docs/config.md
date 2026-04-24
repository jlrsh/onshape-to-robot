# Configuration (`o2r.json`)

`o2r.json` is the configuration file read by the CLI. It supports **C/JSON
comments** via `commentjson`. All keys are snake_case; the old camelCase
names (e.g. `documentId`) are still accepted for backward compatibility.

Format-specific keys live in the exporter pages:

- [URDF](modules/exporter_urdf.md#o2rjson-entries)
- [SDF](modules/exporter_sdf.md#o2rjson-entries)
- [MuJoCo](modules/exporter_mujoco.md#o2rjson-entries)

Processors also expose their own keys — see [processors.md](processors.md).

## Full example

```javascript
{
    // Onshape URL of the assembly (required unless document_id is given)
    "url": "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>",
    // Output format: "urdf" | "sdf" | "mujoco" (required)
    "output_format": "urdf",
    // Output filename stem (default: "robot"; "robot_glb" when mesh_format=glb)
    "output_filename": "robot",
    // Asset directory (default: "assets")
    "assets_directory": "assets",

    // Alternative to url:
    "document_id":  "...",
    "version_id":   "...",   // or
    "workspace_id": "...",
    "element_id":   "...",
    "assembly_name": "robot",  // needed when the document has >1 assembly

    // Onshape configuration (default: "default")
    "configuration": "Configuration=BigFoot;RodLength=50mm",
    // or as a dict:
    // "configuration": {"Configuration": "BigFoot", "RodLength": "50mm"},

    "robot_name": "robot",           // default: output directory name
    "ignore_limits": false,          // default: false

    // Part-level visual/collision filtering
    "ignore": {
        "part1":  "visual",
        "screw*": "visual",
        "*":      "collision",
        "!leg":   "collision"   // whitelist: keep leg's collision
    },

    "draw_frames": false,            // keep frame representation parts?
    "color": [0.5, 0.1, 0.1],        // override all colors (RGB 0..1)
    "no_dynamics": false,            // zero out masses/inertias
    "include_configuration_suffix": true,  // append config to STL filenames

    "post_import_commands": [
        "echo 'Import done'"
    ],

    // Custom processor list (replaces the default registry)
    "processors": [
        "my_project.my_custom_processor:MyCustomProcessor"
    ],

    "round_decimals": 12,            // numeric rounding

    // Transport format for meshes fetched from Onshape
    "mesh_format": "stl",            // "stl" | "glb"

    // Optional reorientation for frames (x-forward convention)
    "frame_x_forward": false
}
```

## Top-level keys

### `url` *(required unless `document_id` is set)*
Full assembly URL. The URL parser accepts both workspace URLs (`/w/<wid>/`)
and version URLs (`/v/<vid>/`) — whichever is present is stored, never both.

### `output_format` *(required)*
One of `"urdf"`, `"sdf"`, `"mujoco"`.

### `output_filename` *(default: "robot")*
Stem only; the extension is added by the exporter (`.urdf`, `.sdf`, `.xml`).
The loader strips any character that is not alphanumeric, `_`, or `-`.
When `mesh_format=glb`, the default stem is swapped to `robot_glb` to avoid
colliding with an STL run in the same directory.

### `assets_directory` *(default: "assets")*
Subdirectory (relative to the output directory) that holds STL/GLB files.

### `document_id` / `version_id` / `workspace_id` / `element_id` *(optional)*
URL parts, used when `url` is not specified. `version_id` and `workspace_id`
are mutually exclusive.

### `assembly_name` *(optional)*
When the document contains several assemblies, use this to pick one. If
omitted and multiple are found, the loader raises.

### `configuration` *(default: `"default"`)*
Either a semicolon string (`"Key1=A;Key2=3mm"`) or a dict
(`{"Key1": "A", "Key2": "3mm"}`). Dicts are joined into a string before
submission. Enum keys are translated from user-facing labels to Onshape
parameter IDs by the assembly loader.

### `robot_name` *(default: directory basename)*
Value of `<robot name="...">` and the MuJoCo `<mujoco model="...">`.

### `ignore_limits` *(default: false)*
Drops joint limits fetched from Onshape.

### `ignore` *(default: `{}`)*
Part-name filter. Accepts either a list (treated as `{"name": "all"}`) or a
dict mapping **glob patterns** to one of `"all"`, `"visual"`, `"collision"`.

- Rules evaluate in order; the last match wins.
- Prefix a pattern with `!` to turn it into a whitelist exception.

Example — keep only `leg` in collision:

```json
"ignore": {
    "*": "collision",
    "!leg": "collision"
}
```

Ignored geometry is dropped from export but **mass/inertia is preserved**.

### `draw_frames` *(default: false)*
Keeps the "frame representation" body in the export. Useful for debugging.

### `color` *(default: null)*
`[r, g, b]` in the range 0..1 — overrides every part color.

### `no_dynamics` *(default: false)*
Sets all masses and inertias to zero. In PyBullet, bodies with zero mass are
treated as static (good for environments/terrain).

### `include_configuration_suffix` *(default: true)*
Appends a sanitized configuration string (or MD5 hash when the string is
long) to STL filenames and part names, so parts that only differ by config
don't collide.

### `post_import_commands` *(default: [])*
Shell commands run **after** export completes. Skipped when `--safe` is
passed on the CLI.

### `processors` *(default: null)*
List of entries. Each entry is either:

- `"module.path:ClassName"` — a dotted module path + class, imported at load
  time. Prevented when `--safe`.
- `"ProcessorName"` — a class exported by `onshape_to_robot.processors`.

When omitted (or when `--safe` is used), the full default registry runs in
its canonical order. See [processors.md](processors.md).

### `round_decimals` *(default: 12)*
Number of decimal places used when rounding numeric output. Applied by
`Config.round()` to `float`, `list`, `tuple`, `ndarray`.

### `mesh_format` *(default: "stl")*
Mesh transfer format fetched from Onshape. `"glb"` preserves per-triangle
color/material (needed when you want per-face color in MuJoCo/rviz); `"stl"`
is geometry-only. Each mesh is stored alongside a sidecar `.part` JSON
containing the instance metadata used to fetch it.

### `frame_x_forward` *(default: false)*
When true, frames are post-multiplied by `T_x_forward` (a cyclic
Z→X→Y→Z permutation). Frames whose name starts with `closing_` are
exempted (loop closures require the original orientation). See
[exporter_utils.md](modules/exporter_utils.md).

### `joint_properties` / `geom_properties` / `equalities`
Per-exporter overrides. Keys use fnmatch patterns.  See the exporter pages.

## Nested configs and variants

`Config` supports two resolution modes:

### Contiguous inheritance

When the CLI target has its own `o2r.json`, the loader walks *upward*
through contiguous ancestor directories, collecting every `o2r.json` it
finds, and deep-merges them oldest-first. The walk stops at the first
ancestor directory without an `o2r.json`.

- Child keys win on conflict.
- Dicts merge key-by-key.
- Lists and scalars replace wholesale.

### Centralized variants

When the CLI target has *no* `o2r.json`, the loader looks for the nearest
ancestor that does. If that ancestor has a `variants` block, the entry
matching the target directory's basename is deep-merged on top of the base
config.

```
rtu/tmf3/o2r.json                     # base + variants block
rtu/tmf3/extended_carriage/           # no o2r.json; resolves via variants
rtu/tmf3/short_carriage/              # no o2r.json; resolves via variants
```

```javascript
{
    "output_format": "urdf",
    "mesh_format": "glb",
    "merge_stls": true,
    "url": "https://cad.onshape.com/.../e/<base-eid>",
    "variants": {
        "extended_carriage": {
            "url": "https://cad.onshape.com/.../e/<ext-eid>"
        },
        "short_carriage": {
            "url": "https://cad.onshape.com/.../e/<short-eid>",
            "joint_properties": { "hip": { "damping": 0.2 } },
            "output_filename": "short"
        }
    }
}
```

Running `onshape-to-robot rtu/tmf3/` builds the base URL in `tmf3/`.
Running `onshape-to-robot rtu/tmf3/extended_carriage/` merges the base
config with `variants.extended_carriage` and builds into
`extended_carriage/`.

**A subdirectory that's not listed under `variants` raises** — this prevents
silent inheritance of the wrong config. If a subdirectory later gains its
own `o2r.json`, that file takes precedence and the ancestor's `variants`
block is ignored (contiguous inheritance wins).

The `variants` key is consumed by the loader; processors and exporters
never see it.

## CLI flags

- `--retrieve` — only run the retrieval step, dump to `robot.pkl`.
- `--convert`  — skip retrieval, load `robot.pkl`, apply processors +
  exporter. Useful when iterating on processors/exporters without hitting
  the Onshape API.
- `--save-pickle` — save `robot.pkl` in addition to proceeding with
  conversion.
- `--safe` — disables custom processor imports and `post_import_commands`.
- `--version`
