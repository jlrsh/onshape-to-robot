# `exporter_mujoco.py` — MuJoCo exporter

Emits a MuJoCo XML model and a sidecar `scene.xml`. Frames become `<site>`
elements. Kinematic loops and gear relations are enforced via
`<equality>`.

## `ExporterMuJoCo(Exporter)`

```python
def __init__(self, config: Config | None = None)
```

Config keys:

| Key | Default | Purpose |
|-----|---------|---------|
| `no_dynamics` | `false` | Accepted but MuJoCo still requires positive mass; a tiny floor of 1e-9 is enforced. |
| `equalities` | `{}` | Fnmatch-keyed attributes merged into each `<equality>` emitted. |
| `additional_xml` | `""` | File / list of files included after `<compiler>`/`<default>`. |
| `freejoint` | `true`  | When false, a fixed base is emitted even if the base link is floating in Onshape. |

Sets `self.ext = "xml"`.

### `build(robot) -> str`

1. XML declaration and `<mujoco model="{robot.name}">`.
2. `<compiler meshdir="..." autolimits="true"/>`.
3. `<default>` with named classes `visual` (invisible geom group) and
   `collision` (hidden by default). Stored as `self.default_class`.
4. Appends `self.additional_xml`.
5. Opens `<worldbody>`. For every base link, recursively calls
   `add_link(robot, base, parent_joint=None)`.
6. Closes `</worldbody>`.
7. Emits `<asset>` — one `<mesh>` per filename and one `<material>`
   per unique color (deduplicated via sets populated while walking the
   body tree).
8. Emits `<actuator>` via `add_actuators`.
9. Emits `<equality>` via `add_equalities`.
10. Closes `</mujoco>`.

### `write_xml(robot, filename) -> str`

Overrides the base. After writing the main XML:

- If `scene.xml` already exists in the same directory, prints an info
  note and leaves it alone.
- Otherwise copies `assets/scene.xml` (a bundled template) and formats
  it with the emitted robot filename — giving you a floor, lighting and
  a skybox to test the model without more work.

### `add_link(robot, link, parent_joint=None, T_world_parent=eye(4))`

Recursive emitter. For each link:

- Computes `T_world_link` from `parent_joint.T_world_joint` (or identity
  for base links).
- Computes the relative transform `inv(T_world_parent) @ T_world_link`,
  emitted as `pos` / `quat` on `<body>`.
- Base link: emit a `<freejoint>` unless the link is `fixed` or
  `config.freejoint` is false.
- Non-base: call `add_joint` inside the body.
- `add_inertial`.
- `add_geometries` for each part.
- `add_frame(..., group=3)` for each entry in `link.frames`.
- Recurse for each child joint with `parent_joint=joint` and
  `T_world_parent=T_world_link`.

### `add_joint(joint)`

Emits `<joint>` with:

- `type` mapped from `joint.joint_type`:
    - `revolute`, `continuous` → `hinge`
    - `prismatic` → `slide`
    - `ball` → `ball`
    - `fixed` → skipped entirely (represented by the body hierarchy).
- `axis` from `joint.axis`.
- `range` from `joint.properties["limits"]` or `joint.limits` when
  `joint.properties.get("range", True)` is true.
- `class`, `frictionloss`, `damping`, `armature`, `stiffness` when
  present in `joint.properties`.

### `add_actuators(robot)`

Walks every joint. A joint is actuated when:

- Its type is not `fixed` / `ball`.
- `joint.properties.get("actuated", joint.relation is None)` is truthy.

The actuator element is chosen from `joint.properties["type"]` (default
`position`). Attributes:

- `class` — `joint.properties.get("class", self.default_class)`.
- `kp`, `kv`, `dampratio` when present.
- `forcerange="-F F"` when `forcerange` is set.
- `inheritrange="1"` when limits are present and `range=True`;
  otherwise an explicit `ctrlrange` from the limits.

### `add_equalities(robot)`

Two kinds:

1. **Frame-based loop closures** — from `robot.closures`:
    - `Closure.FIXED` → `<weld site1=... site2=.../>`.
    - `Closure.REVOLUTE` → `<connect site1=... site2=.../>`.
    - `Closure.BALL` → `<connect site1=... site2=.../>`.
    - `Closure.SLIDER` → warning.
   Attributes merged from `get_equality_attributes`.
2. **Joint relations** (`joint.relation`) →
   `<joint joint1="..." joint2="source" polycoef="0 ratio 0 0 0"/>`.

### `get_equality_attributes(closure) -> str`

Collects every attribute from `config["equalities"]` whose key matches
either `closure.frame1` or `closure.frame2` under fnmatch. Concatenated
as `key="val"` pairs into a single string.

### `add_inertial(mass, com, inertia)`

Emits `<inertial pos="..." mass="..." fullinertia="Ixx Iyy Izz Ixy Ixz Iyz"/>`.
Mass and diagonal inertia are clamped to 1e-9 regardless of
`no_dynamics`.

### `add_mesh(part, class_, T_world_link, mesh)`

Emits `<geom type="mesh" class="..." mesh="..." pos=... quat=... material=.../>`.
Tracks:

- `self.meshes` (set) — asset filenames to emit in `<asset>`.
- `self.materials` — per-mesh material names keyed by (color, basename).

### `add_shape(part, class_, T_world_link, shape)`

Emits `<geom>` with `type` one of `box|cylinder|sphere`:

- Box: `size` is **half-extent** (`size/2`).
- Cylinder: `size="radius half_length"`.
- Sphere: `size="radius"`.

### `add_frame(frame, T_world_link, T_world_frame, group=0)`

Emits `<site name="..." pos="..." quat="..." group="..."/>`. Frame pose
is post-multiplied by `apply_frame_x_forward`. Called with `group=3`
from `add_link` so sites are hidden by default.

### `pos_quat(matrix) -> str`

Renders a 4×4 as `pos="x y z" quat="w x y z"` using
`transforms3d.quaternions.mat2quat`.

## `o2r.json` entries

```javascript
{
    "output_format": "mujoco",
    "additional_xml": "my_custom_file.xml",
    "freejoint": false,

    "joint_properties": {
        "*": {
            "actuated": true,
            "forcerange": 10.0,
            "frictionloss": 0.5,
            "limits": [0.5, 1.2]
        },
        "joint_name": {
            "forcerange": 20.0,
            "frictionloss": 0.1
        }
    },

    "geom_properties": {
        "foot": {
            "collision": {
                "name": "left_foot_collision",
                "friction": "1.2 0.005 0.0001"
            }
        },
        "leg_*": {
            "collision": {
                "solimp": "0.9 0.95 0.001",
                "solref": "0.02 1"
            },
            "visual": { "rgba": "1 0 0 1" }
        }
    },

    "equalities": {
        "closing_branch*": {
            "solref": "0.002 1",
            "solimp": "0.99 0.999 0.0005 0.5 2"
        }
    }
}
```

### `joint_properties` keys consumed by MuJoCo

- Joint element: `class`, `frictionloss`, `damping`, `armature`,
  `stiffness`, `limits`, `range`.
- Actuator element: `actuated`, `class`, `type`, `kp`, `kv`, `dampratio`,
  `forcerange`, `range`.

### `geom_properties`

Keys become `<geom>` XML **attributes** directly. Typical keys: `name`,
`friction`, `solimp`, `solref`, `contype`, `conaffinity`, `rgba`.

### `equalities`

Fnmatch keys match closure frame names. Values are attribute dicts
merged into the emitted `<weld>` / `<connect>` tag. Useful for tuning
`solimp` / `solref` per loop.
