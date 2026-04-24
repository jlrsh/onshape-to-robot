# Processors

Processors are the middle stage of the pipeline: they mutate the `Robot`
produced by `RobotBuilder` before the exporter writes XML.

## Base class

`onshape_to_robot/processor.py`:

```python
class Processor:
    is_safe: bool = True

    def __init__(self, config: Config): ...
    def process(self, robot: Robot): ...
```

- `is_safe` — set to `False` on processors that shell out, mutate the
  filesystem in destructive ways, or import external binaries. Running
  with `--safe` skips those.
- `process(robot)` — invoked once per export, in the order defined by
  `config.processors`. Always mutates `robot` in place.

## Default order

`onshape_to_robot/processors.py` registers the default list
(`default_processors`):

1. [`ProcessorBallToEuler`](processors/ball_to_euler.md)
2. [`ProcessorScad`](processors/scad.md) — `is_safe=False`
3. [`ProcessorMergeParts`](processors/merge_parts.md)
4. [`ProcessorSimplifySTLs`](processors/simplify_stls.md)
5. [`ProcessorFixedLinks`](processors/fixed_links.md)
6. [`ProcessorDummyBaseLink`](processors/dummy_base_link.md)
7. [`ProcessorConvexDecomposition`](processors/convex_decomposition.md) — `is_safe=False`
8. [`ProcessorNoCollisionMeshes`](processors/no_collision_meshes.md)
9. [`ProcessorCollisionAsVisual`](processors/collision_as_visual.md)

Ordering rationale:

- Ball-to-Euler expansion runs early so every other processor sees the
  fully split joint chain.
- SCAD replaces collision meshes with pure shapes — must run before merge
  so the shapes are kept and the replaced meshes are dropped.
- Merge → simplify — simplify operates on the merged result.
- Fixed links → dummy base — structural additions in stable order.
- Convex decomposition runs after merge/simplify so it operates on the
  final collision meshes.
- `no_collision_meshes` and `collision_as_visual` are last — they flip
  visibility flags on the final geometry.

## Registering a custom processor

### Minimal example

```python
# my_project/my_custom_processor.py
from onshape_to_robot.processor import Processor
from onshape_to_robot.config import Config
from onshape_to_robot.robot import Robot

class MyCustomProcessor(Processor):
    def __init__(self, config: Config):
        super().__init__(config)
        self.use_my_custom: bool = config.get("use_my_custom", False)

    def process(self, robot: Robot):
        if self.use_my_custom:
            print(f"Custom processing for {robot.name}")
```

### Registration

In `o2r.json`:

```javascript
{
    "processors": [
        "my_project.my_custom_processor:MyCustomProcessor",
        "ProcessorScad",
        "ProcessorMergeParts",
        "ProcessorNoCollisionMeshes"
    ]
}
```

- `"module.path:ClassName"` imports by dotted path (blocked under
  `--safe`).
- `"ClassName"` resolves against the symbols exported by
  `onshape_to_robot.processors` — see that file for the current list.

The `processors` list **replaces** the default registry; only the
processors you list are instantiated. If you want to keep every default
plus yours, include them explicitly.

## Retrieve & convert modes

`--retrieve` skips processors entirely and writes `robot.pkl`. `--convert`
loads `robot.pkl` and runs just the processors + exporter. Useful for
iterating on a processor without re-hitting the Onshape API. See
[`config.md`](config.md) for the flags.

## Processor reference

Per-processor details:

- [Ball to Euler](processors/ball_to_euler.md)
- [Collision as visual](processors/collision_as_visual.md)
- [Convex decomposition (CoACD)](processors/convex_decomposition.md)
- [Dummy base link](processors/dummy_base_link.md)
- [Fixed links](processors/fixed_links.md)
- [Merge STLs / GLBs](processors/merge_parts.md)
- [No collision meshes](processors/no_collision_meshes.md)
- [SCAD pure shapes](processors/scad.md)
- [Simplify STLs / GLBs](processors/simplify_stls.md)
