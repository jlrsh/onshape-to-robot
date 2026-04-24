# `robot.py` — Intermediate representation

Minimal, exporter-agnostic data model that sits between the Onshape assembly
and the XML writers. Every processor mutates instances of these classes;
every exporter reads them.

## `Part`

```python
class Part:
    def __init__(
        self,
        name: str,
        T_world_part: np.ndarray,     # 4x4 pose of the part in world
        mass: float,
        com: np.ndarray,              # 3-vector in part frame
        inertia: np.ndarray,          # 3x3 in part frame, at COM
        meshes: list[Mesh] = [],
        shapes: list[Shape] = [],
    )
```

A physical chunk of geometry inside a link. Holds meshes (file-based) and
pure shapes (box/cylinder/sphere). `meshes`/`shapes` are deep-copied on
construction to avoid cross-instance aliasing.

### Methods

- **`prune_unused_geometry()`** — drops meshes/shapes whose both `visual`
  and `collision` flags are false. Called by processors that toggle those
  flags (SCAD, no_collision_meshes, convex_decomposition, …).

## `Link`

```python
class Link:
    def __init__(self, name: str)
    # attributes:
    self.parts: list[Part] = []
    self.frames: dict[str, np.ndarray] = {}   # frame_name -> 4x4 pose
    self.fixed: bool = False                   # came from Onshape "Fixed"?
```

### Methods

- **`get_dynamics(T_world_frame=eye(4)) -> (mass, com, inertia)`** —
  reduces all parts of the link into a single inertial triple, expressed in
  `T_world_frame`. Applies the parallel-axis theorem (Modern Robotics 8.26
  & 8.27).

  The returned COM is in `T_world_frame`; the inertia is about the COM,
  aligned with `T_world_frame`.

## `Joint`

```python
class Joint:
    FIXED      = "fixed"
    REVOLUTE   = "revolute"
    PRISMATIC  = "prismatic"
    CONTINUOUS = "continuous"
    BALL       = "ball"

    def __init__(
        self,
        name: str,
        joint_type: str,
        parent: Link,
        child: Link,
        T_world_joint: np.ndarray,
        properties: dict = {},
        limits: tuple[float, float] | None = None,
        axis: np.ndarray = np.array([0, 0, 1]),
    )
    self.relation: Relation | None = None
```

- `properties` carries exporter-specific overrides (`joint_properties`
  fnmatch-matched entries from `o2r.json`).
- `relation`, when set, becomes `<mimic>` in URDF/SDF or an equality in
  MuJoCo.

## `Relation`

```python
class Relation:
    def __init__(self, source_joint: str, ratio: float)
```

Gear/mimic relationship. The sign of `ratio` already accounts for Onshape's
`reverseDirection` flag (inverted in `Assembly.find_relations`).

## `Closure`

```python
class Closure:
    FIXED     = "fixed"
    REVOLUTE  = "revolute"
    BALL      = "ball"
    SLIDER    = "slider"

    def __init__(self, closure_type: str, frame1: str, frame2: str)
```

Represents one kinematic loop. `frame1`/`frame2` refer to frame names
emitted elsewhere in the robot (see `closing_<name>` in
[design.md](../design.md)).

## `Robot`

```python
class Robot:
    def __init__(self, name: str)
    self.links: list[Link] = []
    self.base_links: list[Link] = []       # subset of self.links
    self.joints: list[Joint] = []
    self.closures: list[Closure] = []
```

### Methods

- **`get_link(name)`** — linear scan. Raises `ValueError` if missing.
- **`get_joint(name)`** — linear scan. Raises `ValueError` if missing.
- **`get_link_joints(link)`** — all joints with `parent == link`. Used by
  exporters to emit joints as they walk from each link.

## Invariants

- `links` is ordered: `base_links` first, then children in DFS order
  (`RobotBuilder.build_robot` appends as it recurses).
- `joints` is ordered the same way as the recursion in `build_robot` — so
  `joint` at index *i* corresponds to the edge taken when emitting the
  link it points to.
- `base_links ⊆ links`.
- A joint whose type is `fixed` may still be present even when the Onshape
  mate was e.g. a revolute — processors like `ProcessorFixedLinks` or
  `ProcessorDummyBaseLink` introduce them.
