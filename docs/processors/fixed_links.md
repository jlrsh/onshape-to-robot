# `ProcessorFixedLinks`

Expands a link with several parts into one link per part, connected by
fixed joints.

## `o2r.json`

```javascript
{
    "use_fixed_links": true
}
```

- `use_fixed_links` *(default: false)* — `true` for every link, or a list
  of fnmatch patterns to select specific links: `["arm_*", "leg_*"]`.

## Behavior

For each matching link:

1. For each of its parts, create a new `Link(f"{link}_{part}")` that
   owns just that one part.
2. Clear the original link's parts list.
3. For each new sub-link, append a `Joint.FIXED` named
   `{sub_link}_fixed` parented at the original link with
   `T_world_joint = part.T_world_part`.

## Notes

- `is_safe = True`.
- Useful for debugging / selective rendering, but likely to hurt physics
  engine performance (more bodies to track).
