# Cache

`onshape-to-robot` caches most Onshape API responses under
`~/.cache/onshape-to-robot/` to avoid re-hitting the API on repeat runs.

## What gets cached

`onshape_api/cache.py` exposes a `@cache_response` decorator that wraps the
high-level `Client` methods (assembly fetch, features, parts, STL, GLB,
metadata, mass properties, variables, configuration). The cache key is a
SHA-1 hash of `pickle`-serialized `(args, kwargs)`; the value is the full
response object pickled to a file named `{method_qualname}_{hash}.pkl`.

## What doesn't get cached

`can_cache()` returns **false** when the request uses `wmv="w"` (workspace
mode). Workspace URLs can change under you, so caching them would serve
stale data. Anything requested against a microversion (`m`) or fixed
version (`v`) is cacheable and cached.

## Clearing

```bash
onshape-to-robot-clear-cache
```

This runs `shutil.rmtree("~/.cache/onshape-to-robot/", ignore_errors=True)`.

## Related

- [`onshape_api.md`](modules/onshape_api.md) — the `Client` wrapper whose
  methods are cached.
- [`cli.md`](modules/cli.md) — all CLI entry points.
