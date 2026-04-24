# `ProcessorDummyBaseLink`

Adds a single `base_link` at the root and fixes every current base link
to it. Often used with URDF (which cannot express multiple base links) or
when the consuming code expects a canonical root.

## `o2r.json`

```javascript
{
    "add_dummy_base_link": true
}
```

- `add_dummy_base_link` *(default: false)*.

## Behavior

- Create `Link("base_link")` with `fixed=True`.
- Append to `robot.links`.
- For each existing base link:
    - Create a `Joint.FIXED` named
      `base_link_to_{old_base.name}` with `T_world_joint = eye(4)`.
    - Parent: new base_link; child: the old base link.
- Replace `robot.base_links` with `[new_base_link]`.

## Notes

- `is_safe = True`.
- Fixing multiple base links this way collapses their freedom — if your
  model relied on any of those being floating, use the SDF/MuJoCo
  multi-base feature instead of this processor.
