# `exporter_urdf.py` — URDF exporter

Emits a URDF document. Frames are represented as dummy links connected via
`fixed` joints — URDF has no native frame element.

## `ExporterURDF(Exporter)`

```python
def __init__(self, config: Config | None = None)
```

Reads from `config`:

| Key | Default | Purpose |
|-----|---------|---------|
| `no_dynamics` | `false` | When true, inertial clamping is skipped. |
| `package_name` | `""`   | Prepended to mesh URIs. |
| `use_package_uri_prefix` | `true` | Wraps mesh URIs as `package://…`. |
| `set_zero_mass_to_fixed` | `false` | Zero-mass links emit mass=0, inertia=0. Recognized by PyBullet as fixed. |
| `sort_joints_ascending` | `false` | Emits joints alphabetically. |
| `additional_xml` | `""` | File or list of files whose content is appended inside `<robot>`. |

### Internal

- `self.ext = "urdf"`.
- `self.additional_xml: str` — contents of every file listed under
  `additional_xml`, read at init time.

### `append(line)`

Appends a line (`+ "\n"` added by `build`) to `self.xml`.

### `add_additional_xml(xml_file)`

Reads a single file from `{config.output_directory}/{xml_file}` and
appends its content to `self.additional_xml`.

### `build(robot) -> str`

Top-level flow:

1. Writes XML declaration and the `<robot name="...">` tag.
2. Warns if multiple base links are present (URDF only supports one).
3. Recursively walks from the first base link via `add_link`, collecting
   joints in a `pending_joints` list.
4. Optionally sorts `pending_joints` by name.
5. Emits each joint.
6. Appends `self.additional_xml`.
7. Closes `</robot>`.

### `add_link(robot, link, T_world_link=eye(4), pending_joints=None)`

Recursive emitter. For the current link:

- Opens `<link name="{link.name}">`.
- Calls `add_inertial(*link.get_dynamics(T_world_link), fixed=link.fixed)`.
- Emits every `Shape` via `add_shape` and every `Mesh` via `add_mesh`
  (visual first, then collision).
- Closes `</link>`.
- Emits every entry in `link.frames` via `add_frame` (dummy link + fixed
  joint).
- For each outgoing joint, recurses into the child link and appends the
  joint to `pending_joints`. The parent is consumed to compute the child
  link's `T_world_link` relative origin.

### `add_inertial(mass, com, inertia, fixed=False)`

Emits `<inertial>` with `<origin>`, `<mass>`, `<inertia>` tags. When
`no_dynamics=false`, mass and diagonal inertia are clamped to ≥1e-9.
When `set_zero_mass_to_fixed=true` **and** `fixed=true`, emits zeros —
PyBullet treats that as a fixed base.

### `add_mesh(part, node, T_world_link, mesh)`

Emits `<visual>` or `<collision>` containing `<geometry><mesh filename=.../></geometry>`.
Mesh URI construction:

1. `mesh.filename` is relative to the output directory.
2. If `use_package_uri_prefix=true`: `package://{package_name}/{filename}`
   (omits the package segment when `package_name` is empty).
3. Otherwise: raw relative path.

Writes material (for visual) using `mesh.color`. Emits every entry of
`mesh.visual_properties` / `mesh.collision_properties` as nested XML
inside the `<visual>` / `<collision>` tag.

### `add_shape(part, node, T_world_link, shape)`

Same as `add_mesh` but for `Box`/`Cylinder`/`Sphere`. Box sizes are
emitted as the **full** extent.

### `add_geometries(part, T_world_link)`

Iterates shapes (visual then collision), then meshes (visual then
collision), calling the corresponding `add_*`.

### `add_joint(joint, T_world_link)`

Emits `<joint name=... type=...>`. The joint type can be overridden by
`joint.properties["type"]`. Defaults applied:

- Revolute and prismatic get `max_effort`, `max_velocity` from
  `joint.properties` (defaults 10). Limits come from `joint.limits` unless
  `joint.properties["limits"]` overrides.
- Continuous joints emit no `<limit>` element.
- `joint.properties["friction"]` becomes `<joint_properties
  friction="..."/>`.
- `joint.relation` becomes `<mimic joint="..." multiplier="..."/>`.

### `add_frame(link, frame, T_world_link, T_world_frame)`

Dummy link + fixed joint. The frame pose is post-multiplied by
`apply_frame_x_forward(T_world_frame, frame, config)`.

### `origin(matrix) -> str`

Renders a 4×4 as `<origin xyz="..." rpy="..."/>` using
`rotation_matrix_to_rpy`.

## `o2r.json` entries

```javascript
{
    "output_format": "urdf",
    "package_name": "my_robot",
    "use_package_uri_prefix": false,
    "additional_xml": "my_custom_file.xml",
    "set_zero_mass_to_fixed": true,
    "sort_joints_ascending": true,

    "joint_properties": {
        "*": {
            "max_effort": 10.0,
            "max_velocity": 6.0,
            "friction": 0.5
        },
        "wheel": { "type": "continuous" },
        "joint_name": {
            "max_effort": 20.0,
            "limits": [0.5, 1.2]
        }
    },

    "geom_properties": {
        "tibia": {
            "collision": { "mu1": "1.2", "mu2": "0.8" }
        },
        "leg_*": {
            "visual": { "material": "leg_material" }
        }
    }
}
```

### `joint_properties` keys consumed by URDF

- `type` — override joint type.
- `max_effort`, `max_velocity` — `<joint effort=...>` / `<joint velocity=...>`.
- `friction` — `<joint_properties friction=...>`.
- `limits` — override `(min, max)`.

### `geom_properties`

Keys are fnmatch part-name patterns (`*`, `?`, `[seq]`). Values can be a
flat dict (applied to both visual and collision) or a dict with `visual`
and/or `collision` sub-dicts. Matches merge in declaration order, later
entries overriding earlier ones.

Typical URDF-relevant keys: `mu1`, `mu2`, `kp`, `kd`, `material`.
