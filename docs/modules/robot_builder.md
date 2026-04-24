# `robot_builder.py` — Assembly → Robot

Drives the second half of retrieval: given an `Assembly`, walks the
spanning tree, downloads mesh/color/dynamics for every part, and builds a
`Robot` ready for the processor pipeline.

## `RobotBuilder`

```python
class RobotBuilder:
    def __init__(self, config: Config)
```

On init:

1. Instantiates `Assembly(config)`.
2. Creates a `Robot(config.robot_name)`.
3. Converts `assembly.closures` tuples into `Closure` objects on the robot.
4. Seeds unique-name bookkeeping (`unique_names`, `stl_filenames`).
5. Opens a `contextlib.ExitStack` that owns per-studio GLB extract temp
   directories (`_gltf_extract_dirs`) — released by `close()` / context
   manager / `finally` block in `export.py`.
6. Recursively calls `build_robot` on each root node of the spanning tree
   and appends the results to `robot.base_links`.

### Context management

```python
with RobotBuilder(config) as builder:
    ...
```

or explicit `builder.close()`. Both invoke `ExitStack.close()` which
removes any temp directories used while extracting GLB archives.

### `part_is_ignored(name, what) -> bool`

Applies the `ignore` rules from `o2r.json`. Walks every rule in order;
prefix `!` flips the match into "keep this one". Returns the final verdict
for `what` ∈ `{"visual", "collision"}`. The Onshape `<N>` occurrence
suffix is stripped first.

### `slugify(value) -> str`

Non-alphanumeric characters are replaced with `_`; leading/trailing `_`
are trimmed.

### `printable_configuration(instance) -> str`

Resolves the Onshape-internal `List_…` enum IDs back to human labels by
calling `client.elements_configuration`. Used only for display and for
part-name disambiguation.

### `part_name(part, include_configuration=False) -> str`

Slugifies the instance name (minus the `<N>` tail). When
`include_configuration=True` and
`config.include_configuration_suffix=True`, a configuration-derived suffix
is appended:

- Short configs (< 40 chars): readable — `"_Length_30mm"`.
- Long configs: MD5 hash, keeps filenames bounded.

### `unique_name(part, type) -> str`

Disambiguates duplicate names by appending `_2`, `_3`, … . Uses a
`type`-scoped registry (`"link"`, `"part"`) so a link and a part can
share a name without colliding. Skips names that collide with frames.

### `instance_request_params(instance) -> dict`

Packs an instance's `documentId`, `documentVersion` or
`documentMicroversion`, `elementId`, `configuration`, and the active
assembly's `document_id` as `linked_document_id` into the keyword shape
that `Client` methods expect (`did`, `wmvid`, `wmv`, `eid`,
`linked_document_id`, `configuration`).

### `glb_request_params(instance) -> dict`

Same as above but **downgrades microversion to workspace or version** —
Onshape's GLB export endpoint refuses microversion (`m`). Prefers
`version_id` when available, otherwise `workspace_id`.

### `get_stl_filename(instance) -> str`

Stable filename (no extension) keyed by
`(documentId, documentMicroversion, elementId, configuration, partId)`.
Collisions append `__2`, `__3`, … and emit a warning.

### `get_stl(instance) -> str`

Downloads mesh content and writes a sidecar `.part` JSON manifest next to
it. Branch on `config.mesh_format`:

- `"glb"` → extracts the part-studio GLTF archive once, dispatches to
  `_fetch_glb` which finds the right entry (see below) and writes a
  single GLB.
- `"stl"` → `client.part_studio_stl_m` → writes raw STL bytes.

Returns the absolute path of the written mesh.

### `_get_gltf_extract_dir(instance) -> str`

Fetches the entire part-studio GLTF (Onshape ignores `partid` filters for
GLTF in practice, so doing it per-studio is strictly cheaper), zips it
into a temp dir owned by the `ExitStack`, and caches by
`(documentId, elementId, configuration)`.

### `_fetch_glb(instance, stl_filename, filename)`

Selects the right entry from the extracted archive and writes a single
`.glb`. Matching strategy (falls back in order):

1. **Exact entity-name match** after stripping Onshape's "(N)" pattern-
   feature suffix.
2. **Studio-prefix / surface-split match** when the archive only contains
   surface entries whose names don't mention the part.

If one candidate remains it's loaded directly. If several remain:

- Distinct logical names → treat as surface-split pieces of one part;
  concatenate into a single mesh.
- Same logical name (pattern duplicates) → fetch the per-part STL and
  pick the candidate whose bounding-box best matches. Uses the L2 norm
  of bbox-size difference as a score.

Export via `glb_io.export_glb` (forces NORMAL attribute).

Requires `trimesh`; raises with a `pip install trimesh` tip if missing.

### `get_color(instance) -> np.ndarray`

Override from `config.color` if set; otherwise pulls part metadata and
reads the first `value.color` + `opacity` pair, normalized to `[0,1]^4`.
Defaults to grey.

### `get_dynamics(instance) -> (mass, com[3], inertia[3,3])`

- `config.no_dynamics` → zeros.
- Standard-content parts → `standard_cont_mass_properties` (requires
  `documentVersion`).
- Otherwise → `part_mass_properties`.

Returns `None` for surfaces (no mass properties) and emits a warning.
Also warns when mass is < 1e-9 ("maybe you should assign a material").

### `add_part(occurrence)`

Single-part ingest:

1. Skip suppressed, hidden (via `Assembly.is_occurrence_hidden`), or
   `partId == ""` occurrences.
2. Logs a "+" or "-" symbol (depending on whether any `ignore` rule
   matched).
3. Download mesh unless both `visual` and `collision` are ignored.
4. Get color and dynamics.
5. Compute world pose from `occurrence["transform"]` reshaped to 4×4.
6. Build a `Mesh`; flip `visual`/`collision` flags according to the
   `ignore` rules; only add if at least one flag is still true.
7. Resolve a unique part name, match every fnmatch key in
   `config.geom_properties`, merge nested `visual`/`collision` entries
   into `mesh.visual_properties` / `mesh.collision_properties`.
8. Construct the `Part` and append to `robot.links[-1].parts`.

### `build_robot(body_id) -> Link`

Recursive DFS that populates `robot.links` and `robot.joints`:

1. Look up the body's instance.
2. Pick link name: override from `assembly.link_names` if present,
   otherwise `unique_name(instance, "link")`.
3. Create the `Link`, append to `robot.links`.
4. Walk every occurrence belonging to this body (via
   `assembly.body_occurrences`). Parts are added via `add_part`.
   `occurrence["fixed"]` → `link.fixed = True`.
5. Attach frames from `assembly.frames` whose body matches this body.
6. For each child in `assembly.tree_children[body_id]`:
    - Look up the connecting `DOF`.
    - Resolve the child body via `dof.other_body(body_id)`.
    - Merge `config.joint_properties["default"]` with every fnmatch-matched
      entry to produce the joint's `properties` dict.
    - Build a `Joint` (child is filled in after the recursion).
    - If the joint's name is in `assembly.relations`, set
      `joint.relation = Relation(source, ratio)`.
    - Append to `robot.joints` **before** recursing — this keeps
      `robot.joints` in DFS order.
    - Recurse → `joint.child = <returned Link>`.

Returns the link created at this node.

## Notable non-obvious behavior

- The STL/GLB filename bookkeeping is keyed by instance identity, so the
  same part reused across the assembly downloads and writes once.
- The `.part` sidecar JSON files are handy for caching / diff-ability —
  the merge processor writes per-link manifests in the same spirit.
- The `ExitStack` pattern matters: GLB extracts can be multi-megabyte and
  must be cleaned up even if a later stage raises.
- `joint.child` is populated after the recursion returns, not at
  construction time; exporters and processors can rely on it afterwards.
