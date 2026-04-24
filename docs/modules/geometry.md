# `geometry.py` — Geometry primitives

Pure data classes for link geometry, used inside `Part.meshes` and
`Part.shapes`.

## `Geometry` (base)

```python
class Geometry:
    def __init__(
        self,
        color: np.ndarray = np.array([0.5, 0.5, 0.5, 1.0]),  # RGBA
        visual: bool = True,
        collision: bool = True,
    )
    self.visual_properties:    dict = {}
    self.collision_properties: dict = {}
```

- `visual` and `collision` toggle whether the geometry appears in each role.
  Processors like `ProcessorNoCollisionMeshes` flip these flags.
- `*_properties` carry fnmatch-matched `geom_properties` overrides from
  `o2r.json`, populated by `RobotBuilder.add_part`. Exporters emit them as
  XML attributes or nested elements.

### `is_type(what: str) -> bool`
Truth table: `"visual"` → `self.visual`, `"collision"` → `self.collision`,
anything else → false.

## `Mesh(Geometry)`

```python
class Mesh(Geometry):
    def __init__(self, filename: str, color=..., visual=True, collision=True)
    self.filename: str
```

`filename` is **relative to the output directory** (so it renders as a
portable relative path in the XML).

## `Shape(Geometry)`

```python
class Shape(Geometry):
    def __init__(self, T_part_shape: np.ndarray, color=..., visual=True, collision=True)
    self.T_part_shape: np.ndarray   # 4x4 — shape frame relative to part
```

Base class for pure shapes. Exporters use `T_part_shape` to place the
primitive in the parent link's frame.

## `Box(Shape)` / `Cylinder(Shape)` / `Sphere(Shape)`

```python
class Box(Shape):      size: np.ndarray                  # (sx, sy, sz), meters, full extent
class Cylinder(Shape): length: float; radius: float      # meters
class Sphere(Shape):   radius: float                     # meters
```

`Box.size` is the **full** side length, not the half-extent. Exporters that
need half-extents (MuJoCo) halve it at emit time.
