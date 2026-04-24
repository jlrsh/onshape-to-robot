# `ProcessorNoCollisionMeshes`

Strips collision flags from every mesh in the robot.

## `o2r.json`

```javascript
{
    "no_collision_meshes": true
}
```

- `no_collision_meshes` *(default: false)*.

## Behavior

For every `Mesh` in every `Part`, sets `collision = False`. Then calls
`part.prune_unused_geometry()` so meshes with both flags off disappear.

## Notes

- `is_safe = True`.
- Alternative: use `ignore: {"*": "collision"}`, but that stops the mesh
  from ever reaching later processors (e.g. so a SCAD replacement never
  sees it). This processor keeps the meshes available to earlier stages
  and just disables their collision role at the end.
