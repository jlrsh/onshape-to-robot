# Getting started

## Install

```bash
pip install onshape-to-robot
```

## API key setup

Generate an API key and secret at
<https://cad.onshape.com/user/developer/apiKeys>.

The tooling reads credentials from environment variables. Either export them:

```bash
# .bashrc
export ONSHAPE_API=https://cad.onshape.com
export ONSHAPE_ACCESS_KEY=Your_Access_Key
export ONSHAPE_SECRET_KEY=Your_Secret_Key
```

or drop a `.env` file in the project root:

```bash
# .env
ONSHAPE_API=https://cad.onshape.com
ONSHAPE_ACCESS_KEY=Your_Access_Key
ONSHAPE_SECRET_KEY=Your_Secret_Key
```

`.env` is loaded automatically via `python-dotenv` at the start of the CLI
(see `onshape_to_robot/export.py`).

An alternative `ONSHAPE_SECRET_BEARER` env var is also supported for
token-based auth (see [onshape_api.md](modules/onshape_api.md)).

## Minimum export

Create a directory and put an `o2r.json` inside:

```json
{
    // Onshape URL of the assembly
    "url": "https://cad.onshape.com/documents/<did>/w/<wid>/e/<eid>",
    // Output format: "urdf", "sdf", or "mujoco"
    "output_format": "urdf"
}
```

Make sure the URL is copied while the assembly tab is active.

Run:

```bash
onshape-to-robot my-robot
```

## Test the export

PyBullet viewer:

```bash
onshape-to-robot-bullet my-robot
```

MuJoCo viewer:

```bash
onshape-to-robot-mujoco my-robot
```

## Next

- Read [design-time conventions](design.md) — how `onshape-to-robot` learns
  about DOFs, frames, merging and kinematic loops from mate connector names.
- Review [`o2r.json` options](config.md).
- Browse the [examples repo](https://github.com/rhoban/onshape-to-robot-examples).
