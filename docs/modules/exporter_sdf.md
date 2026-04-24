# `exporter_sdf.py` — SDF exporter

Emits [SDF 1.7](http://sdformat.org/) alongside a Gazebo-style
`model.config` sidecar.

## `ExporterSDF(Exporter)`

```python
def __init__(self, config: Config | None = None)
```

Config keys:

| Key | Default | Purpose |
|-----|---------|---------|
| `no_dynamics` | `false` | Skip inertial clamping. |
| `additional_xml` | `""` | File / list of files included inside `<model>`. |

Sets `self.ext = "sdf"`.

### `build(robot) -> str`

1. XML declaration, header comments, `<sdf version="1.7">`,
   `<model name="...">`.
2. For each base link, recursively emits the link tree via `add_link`
   (starting with `joint=None`). SDF's `relative_to` attributes preserve
   parent/child semantics so URDF-style origin math falls out naturally.
3. Appends `additional_xml`.
4. Closes `</model>` and `</sdf>`.

### `write_xml(robot, filename) -> str`

Overrides the base. Calls `super().write_xml(robot, filename)` and then
writes a `model.config` into the same directory using the
`MODEL_CONFIG_XML` template:

```xml
<?xml version="1.0" ?>
<model>
    <name>{robot.name}</name>
    <version>1.0</version>
    <sdf version="1.7">{output_filename}</sdf>
    <author>…</author>
    <description></description>
</model>
```

### `add_link(robot, link, joint=None)`

Recursive emitter. For the current link:

- Computes the link's world transform: `joint.T_world_joint` if a parent
  joint exists, else identity.
- Emits `<link name=...>` with `<pose>` and `<inertial>`.
- For each part, emits shapes and meshes via `add_geometries` with per-
  part counters (`shape_n`, `mesh_n`) for unique `<visual>` / `<collision>`
  names.
- If `link.fixed`, emits a `<joint type="fixed">` that welds the link to
  `"world"`.
- For each frame, emits a `<frame>` element (see `add_frame`).
- For each outgoing joint, recurses and then emits the joint via
  `add_joint`.

### `add_inertial(mass, com, inertia, frame="")`

Emits an SDF `<inertial>` with `<pose>`, `<mass>`, `<inertia><ixx>...<iyz>`.
The optional `frame` argument is accepted for signature symmetry but is
not used.

### `append_material(color)`

Emits `<material>` with `<ambient>`, `<diffuse>`, `<specular>`,
`<emissive>` entries derived from the RGBA color.

### `add_mesh(link, part, node, T_world_link, mesh, mesh_n)`

Emits `<visual>` or `<collision>`. Mesh URI is
`model://{robot_name}/{mesh.filename}`. Applies
`mesh.visual_properties` / `mesh.collision_properties`.

### `add_shape(link, part, node, T_world_link, shape, shape_n)`

Same for `Box`/`Cylinder`/`Sphere`.

### `add_geometries(link, part, T_world_link)`

Tracks two per-part counters and iterates through shapes then meshes,
emitting each to both the visual and collision track when applicable.

### `add_joint(joint, T_world_link)`

Emits `<joint>` with:

- `type` from `joint.joint_type` (override via `joint.properties["type"]`).
- `<axis>` from `joint.axis`.
- `<limit>` from `joint.limits` (or `joint.properties["limits"]`).
- `<dynamics damping=... friction=...>` if either key is present in
  `joint.properties`.
- `<mimic joint=... multiplier=...>` when `joint.relation` is set.

### `add_frame(link, frame, T_world_link, T_world_frame)`

Emits a native SDF `<frame>`. `T_world_frame` is first passed through
`apply_frame_x_forward`.

### `pose(matrix, relative_to="") -> str`

Renders a 4×4 as `<pose relative_to="...">x y z roll pitch yaw</pose>`.

## `o2r.json` entries

```javascript
{
    "output_format": "sdf",
    "additional_xml": "my_custom_file.xml",

    "joint_properties": {
        "*": { "max_effort": 10.0, "max_velocity": 6.0, "friction": 0.5 },
        "joint_name": { "limits": [0.5, 1.2] },
        "wheel": { "type": "continuous" }
    },

    "geom_properties": {
        "tibia": {
            "collision": { "mu": "1.2", "mu2": "0.8" }
        },
        "leg_*": {
            "collision": { "bounce": "0.5", "max_contacts": "10" }
        }
    }
}
```

### `joint_properties` keys consumed by SDF

Same structural set as URDF (`max_effort`, `max_velocity`, `friction`,
`type`, `limits`). Friction and damping are routed into `<dynamics>`
instead of URDF's `<joint_properties>` element.

### `geom_properties`

Typical SDF-relevant keys: `mu`, `mu2`, `bounce`, `max_contacts`, `kp`,
`kd`.
