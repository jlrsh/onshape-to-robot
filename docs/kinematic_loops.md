# Kinematic loops

A **kinematic loop** is any closed cycle in the kinematic graph — e.g. a
four-bar linkage or a parallel leg. Robot description formats are trees, so
loops must be opened and then re-enforced at runtime.

## The model

1. Pick a place in the loop to open the chain.
2. Add frames on both sides of that cut.
3. Enforce a runtime constraint that keeps those frames coincident.

In Onshape, you express the cut with a **mate connector named
`closing_<name>`** — Onshape sees it as a closed assembly, while
`onshape-to-robot` treats it as the opening point.

## Closure types

The closure type is derived from the Onshape mate type:

| Onshape mate | `Closure.type` |
|--------------|----------------|
| Fastened     | `fixed`        |
| Revolute     | `revolute`     |
| Ball         | `ball`         |
| Slider       | `slider`       |

Each closure produces two frames in the exported model:
`<name>_1` and `<name>_2`. For `revolute` closures, two more frames with a
`_z` suffix enforce axis alignment.

## Output per format

- **URDF / SDF**: `<mimic>` for gear relations; loop closures need external
  runtime enforcement (e.g. PyBullet `createConstraint`, or
  [PlaCo](https://placo.readthedocs.io/) tasks).
- **MuJoCo**: emitted as `<equality>` constraints:
    - `fixed`    → `<weld>`
    - `revolute` → two `<connect>` constraints
    - `ball`     → one `<connect>` constraint
    - `slider`   → warning (not currently emitted)

The per-closure attributes can be tuned via the `equalities` config entry in
`o2r.json` — see [exporter_mujoco.md](modules/exporter_mujoco.md).

## Example

```javascript
{
    "url": "https://cad.onshape.com/.../e/<eid>",
    "output_format": "mujoco",
    "freejoint": false,
    "joint_properties": {
        "passive1": {"actuated": false},
        "passive2": {"actuated": false}
    }
}
```

## Resources

- [MuJoCo equality constraints](https://mujoco.readthedocs.io/en/stable/computation/index.html#coequality)
- PyBullet — `createConstraint`
- [PlaCo loop closures](https://placo.readthedocs.io/en/latest/kinematics/loop_closures.html)
