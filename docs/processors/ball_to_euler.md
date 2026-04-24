# `ProcessorBallToEuler`

Replaces every `ball` joint with three revolute joints forming an Euler
chain. Useful when the downstream tool (simulator, solver) doesn't support
ball joints.

## `o2r.json`

```javascript
{
    "ball_to_euler": true,
    "ball_to_euler_order": "xyz"
}
```

- `ball_to_euler` *(default: false)* — `true` to convert every ball joint,
  or a list of fnmatch patterns to convert selected joints:
  `["joint1", "shoulder_*"]`.
- `ball_to_euler_order` *(default: `"xyz"`)* — Euler order; one of
  `xyz`, `xzy`, `zyx`, `zxy`, `yxz`, `yzx`.

## Behavior

For every matching ball joint (name match via `fnmatch`):

1. Create two intermediate `Link`s named `{joint}_link_{axis}` (one per
   non-final axis in the Euler order).
2. Replace the ball joint with three `Joint.REVOLUTE`s named
   `{joint}_{x|y|z}` sharing the original `T_world_joint` and
   `properties`. Axes are permuted according to `ball_to_euler_order`.
3. Remove the original ball joint from `robot.joints`.

## Notes

- `is_safe = True`.
- No external dependencies.
- No per-ball mass is redistributed — the intermediate links have no
  inertia. MuJoCo clamps them to ≥1e-9 at emit time.
