# `exporter.py` — Base exporter

Thin base class for the format-specific exporters. See:

- [`exporter_urdf.md`](exporter_urdf.md)
- [`exporter_sdf.md`](exporter_sdf.md)
- [`exporter_mujoco.md`](exporter_mujoco.md)
- [`exporter_utils.md`](exporter_utils.md) — shared helpers
  (`apply_frame_x_forward`, `rotation_matrix_to_rpy`, `xml_escape`).

## `Exporter`

```python
class Exporter:
    def __init__(self)
    self.xml: str = ""
    self.ext: str = "xml"
```

Subclasses set `self.ext` to the format extension and assemble `self.xml`
by calling `self.append(...)` / string concatenation inside `build()`.

### `build(robot: Robot) -> str`
Abstract — overridden by each subclass. Populates `self.xml` and returns it.

### `get_xml(robot: Robot) -> str`
Calls `build(robot)`, returns `self.xml`. Override to add pre-processing.

### `remove_empty_text_nodes(node)`
Recursively strips whitespace-only text nodes from a `minidom` tree.
Prevents blank lines in the pretty-printed output.

### `write_xml(robot, filename) -> str`
Template method:

1. `self.build(robot)`.
2. `minidom.parseString(self.xml)` → tree.
3. `remove_empty_text_nodes(tree)`.
4. `tree.toprettyxml(indent='  ')` → written to `filename`.
5. Prints a success line.

SDF and MuJoCo override this to also write a sibling file (`model.config`,
`scene.xml`).
