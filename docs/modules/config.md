# `config.py` — Configuration loader

Loads `o2r.json`, resolves inheritance (contiguous ancestors + variants),
and constructs the list of processors. Also provides the numeric
rounding helper used by exporters.

User-facing documentation for every `o2r.json` key lives in
[`../config.md`](../config.md).

## Module-level

- **`CONFIG_FILENAME = "o2r.json"`** — file name the loader looks for.

### `deep_merge(base: dict, overlay: dict) -> dict`

Recursive merge. Overlay wins on conflict; nested dicts merge key-by-key;
lists and scalars replace wholesale. Inputs are **not** mutated
(`copy.deepcopy` internally).

### `_resolve_chain(config_path: str) -> list[str]`

Walks upward from `config_path` collecting `o2r.json` files in **contiguous
ancestor directories**. Returns oldest-first (root ancestor first, `self`
last). Stops at the first ancestor directory without an `o2r.json` or at
the filesystem root.

### `_find_nearest_ancestor_config(start_dir: str) -> str | None`

Walks upward from `start_dir` looking for the first ancestor that contains
an `o2r.json`. Returns the absolute path or `None`. Does not inspect
`start_dir` itself.

## `Config`

```python
class Config:
    def __init__(self, robot_path: str, safe: bool = False)
```

On init:

1. Resolves the target directory and decides whether the CLI target *has*
   its own `o2r.json`.
2. Either:
    - (target has one) walks contiguous ancestors and deep-merges oldest
      first; `self.variant_name = None`.
    - (target has no config) finds the nearest ancestor config, walks its
      contiguous chain, and records `self.variant_name` = target basename.
3. Strips the loader-only `variants` key from the merged dict; if a variant
   name was set, deep-merges `variants[name]` on top. Raises when the
   variant name has no entry — known variants are listed in the error.
4. Calls `self.read_configuration()` to populate public attributes.
5. Creates the output directory if missing.

### Attributes

After construction:

- `self.safe: bool`
- `self.config_file: str` — path to the `o2r.json` actually loaded.
- `self.output_directory: str` — **always** the dir the CLI was pointed at,
  never the ancestor whose config is loaded.
- `self.variant_name: str | None`
- `self.config: dict` — fully merged config payload (minus `variants`).
- `self.processors: list[Processor]`
- plus the fields populated by `read_configuration()` (listed in
  [`../config.md`](../config.md)).

### `to_camel_case(snake_str) -> str`

`"output_format"` → `"outputFormat"`. Used by `get()` for legacy key
fallback.

### `get(name, default=None, required=True, values_list=None)`

Configuration accessor. Looks for `name` first, then its camelCase
equivalent. Raises when missing and `required=True` with no `default`
provided. Enforces `values_list` when given.

### `printable_version() -> str`

Human-readable identifier for log lines (`url` if present; otherwise
`document_id` + `version_id`/`workspace_id`).

### `parse_url()`

Splits `self.url` into `document_id`, `workspace_id` or `version_id` (not
both), and `element_id`. Pattern: `https://(.*)/(.*)/([wv])/(.*)/e/(.*)`.

### `asset_path(asset_name: str) -> str`

`{output_directory}/{assets_directory}/{asset_name}`.

### `read_configuration()`

Populates the public attributes from `self.config`. Notable behavior:

- `output_filename` is stripped to `[A-Za-z0-9_-]`.
- When both `version_id` and `workspace_id` are set → raises.
- When neither `url` nor `document_id` is set → raises.
- `configuration` accepts a dict; dicts are flattened to `"k=v;k=v"`.
- `ignore` accepts a list (converted to `{"name": "all"}`).
- `mesh_format` is validated against `["stl", "glb"]`; when `"glb"` and
  `output_filename` is left at the default `"robot"`, it is renamed to
  `"robot_glb"` so parallel STL/GLB runs don't clobber each other.
- Processors are instantiated here:
    - No `processors` key (or `--safe`) → use `processors.default_processors`,
      filtered to `is_safe` processors when `self.safe`.
    - Entries shaped `"module.path:Cls"` are imported via
      `__import__(module, fromlist=[cls])` (unless `safe`, since arbitrary
      import is the attack surface `--safe` blocks).
    - Entries shaped `"ClsName"` are looked up in
      `onshape_to_robot.processors`.

### `round(object)`

Rounds a `float`, `list`, `tuple`, or `ndarray` to `self.round_decimals`.
Preserves the input container type.

## Safe mode

`Config(..., safe=True)` (backed by the `--safe` CLI flag) does two things:

- Forces the default processor list, filtered to `is_safe`.
- Blocks `post_import_commands` execution in `export.py`.

`is_safe` is defined on each `Processor` subclass; today the opt-outs are
`ProcessorScad` and `ProcessorConvexDecomposition` (both shell out /
subprocess).
