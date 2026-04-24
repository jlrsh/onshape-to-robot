# `exporter_utils.py` — Shared exporter helpers

## `xml_escape(s: str) -> str`

Escapes `<`, `>`, `&`, `'`, `"` using `xml.sax.saxutils.escape` with a
custom entity map.

## Constants

- **`CLOSING_FRAME_PREFIX = "closing_"`** — frames whose name starts with
  this are **excluded** from `apply_frame_x_forward`; loop closures must
  keep their original orientation for equality constraints to be valid.
- **`T_x_forward`** — 4×4 rotation that cyclically permutes axes
  `Z → X → Y → Z`. Post-multiplied onto a frame to switch from Onshape's
  Z-up mate convention to an x-forward convention some downstream
  consumers expect.

## `apply_frame_x_forward(T_world_frame, frame_name, config) -> np.ndarray`

- When `config.frame_x_forward` is true **and** `frame_name` does not
  start with `CLOSING_FRAME_PREFIX`: returns
  `T_world_frame @ T_x_forward`.
- Otherwise: returns `T_world_frame` unchanged.

Invoked by every exporter when emitting a named frame.

## `rotation_matrix_to_rpy(R) -> np.ndarray`

Converts a 3×3 rotation matrix to ZYX Euler angles `(roll, pitch, yaw)`,
with the singular case at pitch = ±π/2 handled explicitly. Used by URDF
and SDF to fill `<origin rpy="...">` and `<pose>`.
