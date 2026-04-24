# `simulation.py` — PyBullet wrapper

Developer utility used by `bullet.py` and example scripts. Wraps
PyBullet with a small convenience surface: named frames/joints, real-
time sync, contact helpers, debug lines, reset presets.

Not part of the export pipeline — feel free to use it or ignore it.

## `Simulation`

```python
def __init__(
    self,
    robotPath: str,
    floor: bool = True,
    fixed: bool = False,
    transparent: bool = False,
    gui: bool = True,
    ignore_self_collisions: bool = False,
    realTime: bool = True,
    panels: bool = False,
    useUrdfInertia: bool = True,
    dt: float = 0.002,
    physicsClient: int | None = None,
)
```

Loads `robotPath` into either a GUI or DIRECT PyBullet client, resolves
joint and frame indices, and optionally drops a floor plane.

### Pose / transforms

- `getRobotPose() -> (pos, rpy)`
- `setRobotPose(pos, orn_quat)`
- `frameToWorldMatrix(frame) -> np.matrix`
- `transformation(frameA, frameB) -> np.matrix`
- `poseToMatrix(pose)`, `matrixToPose(matrix)`

### Joint control

- `setJoints(joints_dict)` — joint name → setpoint (position, or velocity
  when the name ends with `_speed`). Passive joints forced to zero
  velocity.
- `resetJoints(joints_dict)` — instantaneous, non-physics.
- `getJoints() -> list[str]`, `getJointsInfos(name)` — metadata.

### Frame queries

- `getFrame(frame)`, `getFrames()`, `getVelocity(frame)`.

### Physics queries

- `getRobotMass()` (cached).
- `getCenterOfMassPosition()`.
- `contactPoints()` — list of ground-contact tuples.
- `autoCollisions()` — total non-floor collision force (N).

### Constraints / debug

- `addConstraint(frameA, frameB, constraint=JOINT_POINT2POINT) -> id`.
- `addDebugPosition(position, color=None, duration=30)`,
  `drawDebugLines()`.
- `reset(height=0.5, orientation='straight'|'front'|'back')`.
- `resetPose(pos, orn)`.
- `setFloorFrictions(lateral=1, spinning=-1, rolling=-1)`.
- `lookAt(target)`.

### Stepping

- `tick()` — one simulation step, optionally sleeps to maintain wall-
  clock sync when `realTime=True`.
- `execute()` — blocking infinite loop of `tick()`.
