# `onshape_api/` — Onshape REST client

Three files:

- `onshape.py` — low-level HTTP + HMAC signer.
- `client.py` — high-level wrapper with caching decorators.
- `cache.py` — pickle-based disk cache.
- `utils.py` — logging helper.

## Authentication (`onshape.py`)

### `class Onshape`

```python
def __init__(self, stack: str, creds: str = "./config.json", logging: bool = True)
```

Reads credentials from environment variables in preference, falling back
to `creds` (a legacy JSON file). Envs:

| Var | Purpose |
|-----|---------|
| `ONSHAPE_API`         | Base URL (e.g. `https://cad.onshape.com`). |
| `ONSHAPE_ACCESS_KEY`  | API key. |
| `ONSHAPE_SECRET_KEY`  | API secret. |
| `ONSHAPE_SECRET_BEARER` | Alternative: bearer token (used instead of key/secret). |

`creds` file format (legacy):

```json
{
    "onshape_api": "https://cad.onshape.com",
    "onshape_access_key": "...",
    "onshape_secret_key": "..."
}
```

### HMAC signing

`_append_auth` mutates the outgoing header dict to add `Authorization`
and `On-Nonce`. Two modes:

1. **Bearer token** — `Authorization: Bearer <ONSHAPE_SECRET_BEARER>`.
2. **HMAC-SHA256** — canonical string is
   `(method + '\n' + nonce + '\n' + date + '\n' + ctype + '\n' + path + '\n' + query + '\n').lower()`,
   signed with the secret key; base64-encoded signature joins
   `On {access_key}:HmacSHA256:<sig>`. A fresh 25-char nonce is generated
   per request.

### `request(method, path, query={}, headers={}, body={}, base_url=None) -> requests.Response`

Issues the HTTP request with signed headers. Follows HTTP 307 redirects
recursively (Onshape uses them to bounce requests into a specific
regional stack). Non-OK (not 200-206) responses print the error and
exit via `os._exit(1)`. Returns the raw `requests.Response` — callers
use `.json()` or `.content` as appropriate.

## `Client` (client.py)

```python
def __init__(self, stack="https://cad.onshape.com", logging=True, creds="./config.json")
```

A thin wrapper around `Onshape`. All endpoint methods below are
decorated with `@cache_response` unless explicitly noted.

### Generic helpers

- `request(url, **kwargs) -> dict` — JSON GET.
- `request_binary(url, **kwargs) -> bytes` — raw GET (STL/GLB exports).
- `escape(s)` — URL-escape `/` → `%2f`, `+` → `%2b` for ids.

### Constants

- `API_VERSION = "v11"`.
- `TRANSLATION_POLL_TIMEOUT_S = 300.0`.
- `TRANSLATION_POLL_INITIAL_DELAY_S = 1.0`.
- `TRANSLATION_POLL_MAX_DELAY_S = 10.0`.
- `TRANSLATION_POLL_BACKOFF = 1.5`.

### Document / assembly

| Method | HTTP | Returns |
|--------|------|---------|
| `get_document(did)` | `GET /api/documents/{did}` | document metadata |
| `list_elements(did, wid, wmv="w")` | `GET /api/documents/d/{did}/{wmv}/{wid}/elements` | list of elements |
| `get_assembly(did, wmvid, eid, wmv="w", configuration="default")` | `GET /api/assemblies/d/{did}/{wmv}/{wmvid}/e/{eid}` | full assembly tree (includes mate features, mate connectors, non-solids, and the chosen configuration) |
| `get_features(did, wvid, eid, wmv="w", configuration="default")` | `GET /api/assemblies/d/{did}/{wmv}/{wvid}/e/{eid}/features` | feature list (mates, mate relations, …) |

### Parts / sketches

| Method | HTTP | Returns |
|--------|------|---------|
| `get_sketches(did, mid, eid, configuration)` | `GET /api/partstudios/d/{did}/m/{mid}/e/{eid}/sketches` | sketch geometry + transforms (`includeGeometry=true`). |
| `get_parts(did, mid, eid, configuration)` | `GET /api/parts/d/{did}/m/{mid}/e/{eid}` | list of parts. |
| `find_new_partid(did, mid, eid, partid, configuration_before, configuration)` | (derived — calls `get_parts` twice) | new partId across a configuration change, matched by part name. |

### Meshes

| Method | HTTP | Returns |
|--------|------|---------|
| `part_studio_stl_m(did, wmvid, eid, partid="", wmv="m", configuration="default", linked_document_id=None)` | `GET /api/parts/.../stl` | binary STL (`mode=binary`, `units=meter`). |
| `part_studio_gltf(did, wmvid, eid, partid="", wmv="m", configuration="default", linked_document_id=None)` | `POST /api/{API_VERSION}/partstudios/.../export/gltf` | GLB bytes. Requires `wmv` ∈ {`w`, `v`}. Uses async translation + `_poll_translation`. |

### Translation polling

`_poll_translation(translation_id, timeout_s=300.0)` polls
`GET /api/{API_VERSION}/translations/{id}` with exponential backoff
(`initial=1.0s`, `max=10.0s`, `factor=1.5`). Returns when `requestState
== "DONE"`; raises on `FAILED` or on timeout.

### Metadata / dynamics

| Method | HTTP | Returns |
|--------|------|---------|
| `part_get_metadata(did, wmvid, eid, partid, wmv="m", configuration="default", linked_document_id=None)` | `GET /api/metadata/d/...` | part metadata (name, color, custom properties). |
| `part_mass_properties(did, wmvid, eid, partid, wmv="m", configuration="default", linked_document_id=None)` | `GET /api/parts/.../massproperties` | mass, COM, inertia. Uses `useMassPropertyOverrides=true`. |
| `standard_cont_mass_properties(did, vid, eid, partid, linked_document_id, configuration)` | same endpoint with `v` prefix | used for standard-content parts (screws, bearings, …) which always reference a specific version. |

### Assembly mates / configuration

| Method | HTTP | Returns |
|--------|------|---------|
| `matevalues(did, wmvid, eid, wmv="w", configuration="default")` | `GET /api/assemblies/.../matevalues` | current rotation/translation per mate. Only meaningful in workspace mode. |
| `elements_configuration(did, wmvid, eid, wmv, linked_document_id=None, configuration=None)` | `GET /api/elements/.../configuration` | configuration schema + values. |
| `get_variables(did, wvid, eid, wmv, configuration)` | `GET /api/variables/.../variables` | variables defined in the Part Studio (for expression evaluation). |

## `cache.py`

- Cache directory: `~/.cache/onshape-to-robot/`, created lazily.
- `cache_response(method)` is the decorator applied to the `Client`
  endpoint methods.
- `can_cache(method, *args, **kwargs)` returns **false** when the request
  uses `wmv="w"`. Workspace content isn't cacheable; microversion and
  version content are.
- Cache filename: `{method.__qualname__}_{sha1(pickle(args,kwargs))}.pkl`.

Full user-facing notes at [`../cache.md`](../cache.md).

## `utils.py`

`log(msg, level=0)` — timestamped coloured log. `level=0` → green to
stdout; `level=1` → red to stderr. Uses `colorama`.
