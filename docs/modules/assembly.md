# `assembly.py` — Onshape assembly interpreter

Retrieves an Onshape assembly, interprets mate-connector naming conventions,
and produces the kinematic graph consumed by `RobotBuilder`.

This is the fattest file in the codebase and is worth a full tour.

## Module-level

- **`INSTANCE_IGNORE = -1`** — sentinel body id. Instances marked with it
  are excluded from the kinematic tree (e.g. frame-representation parts
  when `draw_frames=false`).

## `Frame`

```python
class Frame:
    def __init__(self, body_id: int, name: str, T_world_frame: np.ndarray)
```

Plain container for a named pose on a body.

## `DOF`

```python
class DOF:
    def __init__(
        self,
        body1_id: int,
        body2_id: int,
        name: str,
        joint_type: str,     # Joint.REVOLUTE | PRISMATIC | FIXED | CONTINUOUS | BALL
        T_world_mate: np.ndarray,
        limits: tuple | None,
        axis: np.ndarray = np.array([0, 0, 1]),
    )
```

### `flip(flip_limits: bool = True)`
Applies a 180° rotation about X to `T_world_mate` and (optionally) swaps
`limits` as `(-max, -min)`.
- Used twice: once when a `dof_` mate name ends with `_inv`/`_inverted`
  (with `flip_limits=True`), and again during tree construction when a
  DOF is traversed parent→child in the "wrong" direction (with
  `flip_limits=False`).

### `other_body(body_id) -> int`
Returns the opposite end of the DOF. Raises if `body_id` is neither end.

## `Assembly`

```python
class Assembly:
    def __init__(self, config: Config)
```

### Initialization order

The constructor drives the full retrieval pipeline:

1. `ensure_workspace_or_version()` — resolve `workspace_id` via
   `client.get_document()` if neither workspace nor version was provided.
2. `find_assembly()` — locate the element id if not in config.
3. `check_configuration()` — validate the `configuration` string against
   the element's parameter schema and translate enum labels to parameter
   IDs.
4. `retrieve_assembly()` — `GET /assemblies/.../e/...`; records
   `assembly_data`, `occurrences`, `microversion_id`.
5. `find_instances()` — walks the instance tree and tucks each instance
   into its occurrence, recursing into sub-assemblies.
6. `load_features()` — `GET /.../features` and `/.../matevalues`.
7. `load_configuration()` — parses `fullConfiguration`, populates the
   expression parser's variables.
8. `process_mates()` — the core — see below.
9. `build_trees()` — spanning tree construction + cycle check.
10. `find_relations()` — gear / mimic relations.

### Key attributes

| Attribute | Type | Meaning |
|-----------|------|---------|
| `client` | `Client` | Cached Onshape REST client. |
| `expression_parser` | `ExpressionParser` | Evaluates Onshape expressions; has a lazy `variables_lazy_loading` callback wired to `load_variables`. |
| `document_id / workspace_id / version_id / element_id` | `str` | Target IDs. |
| `microversion_id` | `str` | Document microversion of the fetched assembly. |
| `assembly_data` | `dict` | Raw JSON from `get_assembly`. |
| `occurrences` | `dict[tuple, dict]` | Keyed by occurrence path; values have `path`, `transform`, `hidden`, and — after `find_instances` — `instance`. |
| `instance_body` | `dict[str, int]` | instance id → body id; may contain `INSTANCE_IGNORE`. |
| `current_body_id` | `int` | Next body id to hand out. |
| `frames` | `list[Frame]` | |
| `dofs` | `list[DOF]` | |
| `closures` | `list[tuple]` | `(closure_type, frame1_name, frame2_name)` (+ extra `_z` entries for revolute closures). |
| `features` | `dict` | Response of `get_features` (features + relations). |
| `matevalues` | `dict` | `get_matevalues` response — current rotation/translation per mate. |
| `configuration_parameters` | `dict` | parameter id → value string. |
| `tree_children` | `dict[int, list[int]]` | Spanning tree: body → children. |
| `root_nodes` | `list[int]` | One per connected component. |
| `link_names` | `dict[int, str]` | Overrides from `link_<name>` mates. |
| `relations` | `dict[str, [str, float]]` | target_joint → (source_joint, ratio). |

### Retrieval helpers

- **`ensure_workspace_or_version()`** — if both `workspace_id` and
  `version_id` are None, asks the API for the document's default
  workspace and stashes it on `self.workspace_id`.
- **`find_assembly()`** — uses `element_id` if set, otherwise lists elements
  and filters type `"Assembly"`. Raises when none or multiple are found
  without `assembly_name`.
- **`check_configuration()`** — looks up
  `client.elements_configuration(...)` when the configuration string is
  not `"default"`. Validates types (enum, bool, quantity) and rewrites user
  labels into parameter IDs.
- **`retrieve_assembly()`** — fills `assembly_data`, `occurrences` (keyed
  by occurrence path tuple), and `microversion_id`.
- **`find_instances(prefix=[], instances=None)`** — recursive. Finds each
  instance's occurrence (including subassembly-prefixed paths) and attaches
  the instance dict to the occurrence. Recurses into non-suppressed
  sub-assemblies.
- **`load_features()`** — `get_features` + `matevalues`.
- **`load_configuration()`** — parses `fullConfiguration` into
  `configuration_parameters` and evaluates each value into
  `expression_parser.variables`.
- **`load_variables()`** — lazy callback: pulls `get_variables` the first
  time the expression parser sees an unknown identifier.

### Occurrence helpers

- **`get_occurrence(path) -> dict`** — raw lookup by path tuple.
- **`is_occurrence_hidden(path) -> bool`** — true if this occurrence **or
  any ancestor** is hidden. Walks the path.
- **`get_occurrence_transform(path) -> np.ndarray`** — reshapes the flat
  `transform` list into a 4×4.

### Geometry helpers

- **`cs_to_transformation(cs) -> np.ndarray`** — builds a 4×4 from a dict
  with `"xAxis"`, `"yAxis"`, `"zAxis"`, `"origin"`.
- **`get_mate_transform(mated_entity) -> np.ndarray`** — wrapper that
  pulls `matedCS` and runs it through `cs_to_transformation`.
- **`translation(x, y, z) -> np.ndarray`** — identity with the translation
  column filled.

### Body bookkeeping

- **`make_body(id)`** — assigns the next `current_body_id` to
  `instance_body[id]` and increments.
- **`merge_bodies(occ_A, occ_B)`** — merges body B into body A (lowest id
  wins after a swap). Rewrites `instance_body` and every DOF that still
  mentions body B.

### `process_mates()`

This is the core routine. Runs in phases:

1. **Root body**: first top-level instance is assigned body id 0.
2. **DOFs** (mates whose name starts with `dof_`):
    - Parse joint type from the Onshape mate type (see
      [design.md](../design.md)).
    - Wheel-like names or names containing `continuous` become
      `Joint.CONTINUOUS`, no limits.
    - For revolute/prismatic/ball: `get_limits(...)` unless
      `ignore_limits=true`.
    - `_inv`/`_inverted` suffix → `flip(flip_limits=True)`.
    - Two `make_body` calls for the mated occurrences, then a `DOF` is
      appended.
3. **Fixed merges**: mates named `fix_*`, and unnamed `FASTENED` mates
   (as long as they aren't `dof_`/`closing_`/`frame_`) → `merge_bodies`.
4. **Mate groups** — every occurrence in a group gets merged into the first.
5. **Frame mates** (`frame_<name>`): one side of the mate must be an orphan
   (unassigned) and the other must already be in the tree. Emits a `Frame`
   on the tree-member body; the orphan is merged in when
   `draw_frames=true` and otherwise assigned `INSTANCE_IGNORE`.
6. **Default body assignment**: every remaining non-suppressed instance
   that's not `INSTANCE_IGNORE` gets a body id.
7. **Loop closures** (`closing_<name>`): creates `<name>_1` and `<name>_2`
   frames on each side, plus `<name>_1_z`/`<name>_2_z` for revolute
   closures, and appends the closure tuple(s) to `self.closures`.
8. **Link renames** (`link_<name>` on a mate connector): records the
   override in `link_names[body_id]`; warns if the occurrence was
   filtered out.
9. **Frame connectors** (`frame_<name>` on a mate connector): emits a
   `Frame` directly from the connector coordinate system.

### Tree construction

- **`build_trees()`** — iterates unassigned bodies (skipping
  `INSTANCE_IGNORE`) and calls `build_tree` on each. Prints root nodes.
- **`build_tree(root_node)`** — BFS. For each DOF that touches the current
  body, the other body becomes a child. When the current body is
  `body1_id`, the DOF is flipped (`flip_limits=False`) so its
  `T_world_mate` consistently represents parent→child. Duplicate visits
  raise — the assembly is not a tree.

### Feature helpers

- **`feature_mating_two_occurrences()`** — generator of
  `(feature_data, occ_A, occ_B)` for every mate that has exactly two mated
  entities and is not suppressed.
- **`feature_mate_groups()`** — returns a list of groups, each a list of
  occurrence ids.
- **`get_feature_by_id(feature_id)`** — linear scan of `features`.

### Relations

- **`find_relations()`** — walks `BTMMateRelation` features, extracts the
  two mate IDs and the ratio expression, strips the `dof_` prefix, and
  stores `relations[target_joint] = [source_joint, ratio]`. Ratio sign is
  flipped unless `reverseDirection` is true. Warns when a target appears
  twice (last one wins).

### Expression / limit helpers

- **`read_parameter_value(parameter, name)`** — evaluates
  `BTMParameterNullableQuantity` or `BTMParameterConfigured`. Configured
  parameters look up the current configuration value and evaluate the
  chosen branch.
- **`read_expression(expression)`** — delegates to
  `expression_parser.eval_expr`; triggers `load_variables` lazily on
  unknown identifiers.
- **`get_offset(name) -> float | None`** — current mate offset
  (`rotationZ` or `translationZ`) from `matevalues`. Only meaningful for
  workspace mode; versions do not expose it.
- **`get_limits(joint_type, name) -> tuple | None`** — finds the matching
  feature, checks `limitsEnabled`, reads the axis-specific min/max:
    - revolute: `limitAxialZMin` / `limitAxialZMax`
    - prismatic: `limitZMin` / `limitZMax`
    - ball: `(0, limitEulerConeAngleMax)`
  - Subtracts the current offset (for revolute joints, the offset is first
    normalized to `(-π, π]` via `math_utils.normalize_angle_pi`). Emits a
    warning when limits are absent (except for continuous joints). Returns
    `None` when limits are disabled or globally ignored.

### Body accessors (used by `RobotBuilder`)

- **`body_instance(body_id) -> dict | None`** — first instance associated
  with the body.
- **`body_occurrences(body_id)`** — generator over every occurrence whose
  root-level instance was assigned to this body.
- **`get_dof(body1_id, body2_id) -> DOF`** — direct DOF lookup between
  two bodies.

## Data shapes, at a glance

```python
self.occurrences[path_tuple] = {
    "path": [instance_id, ...],
    "transform": [16 floats],      # row-major 4x4
    "hidden": bool,
    "instance": {...},             # populated by find_instances
    ...
}

self.closures = [
    ("fixed", "loop_1", "loop_2"),
    ("revolute", "loop_1", "loop_2"),
    ("revolute", "loop_1_z", "loop_2_z"),
    ...
]

self.relations = {"target_joint": ["source_joint", 1.5]}
```

## Edge cases worth knowing

- A `frame_` mate that mates two already-in-tree bodies is disallowed — the
  loader expects exactly one orphan side.
- Body ids can jump (non-contiguous) after merges.
- `frame_` prefix is handled *twice*: once as a mate between two occurrences
  (phase 5) and once as a standalone mate connector (phase 9). Both
  mechanisms are valid; the connector form skips orphan-merging.
- `CYLINDRICAL` mates are treated as `REVOLUTE` for joint-type purposes
  (respects the `wheel`/`continuous` keywords).
