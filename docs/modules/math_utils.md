# `math_utils.py`

One helper.

## `normalize_angle_pi(angle: float) -> float`

Normalizes an angle to the half-open interval `(-π, π]` via
`arctan2(sin(angle), cos(angle))`.

Used by `Assembly.get_limits` when adjusting revolute joint limits by
the current offset. Onshape can hand back accumulated multi-revolution
values (>2π), but joint limits only make sense relative to the
principal angle.
