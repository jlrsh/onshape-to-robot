# `ProcessorCollisionAsVisual`

Uses the collision geometry as the visual geometry.

## `o2r.json`

```javascript
{
    "collisions_as_visual": true
}
```

- `collisions_as_visual` *(default: false)*.

## Behavior

For every `Part` in every `Link`:

- For every `Mesh` and `Shape`, set `visual = collision`.
- Call `part.prune_unused_geometry()` to drop anything that still has no
  role.

## Interactions

- When combined with `merge_stls=true`, only the merged collision mesh is
  kept, and its filename drops the `_collision` suffix.
- Paired with [`no_collision_meshes`](no_collision_meshes.md) you can
  visualize collision hulls without exposing them as collision.

## Notes

- `is_safe = True`.
- Last in the default order; runs after merge/simplify/convex-decomp, so
  "collision as visual" always reflects the final collision state.
