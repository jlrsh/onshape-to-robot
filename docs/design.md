# Design-time conventions

`onshape-to-robot` reads an **assembly** (top-level) and interprets mate
connector *names* as annotations. Follow these conventions when building the
Onshape model.

## Assembly rules

- Point the exporter at a **top-level assembly**; each instance in it
  becomes a link (or gets merged into another link).
- The **first instance** in the Assembly list is treated as the **base link**.
- Every instance in the assembly becomes a link by default.
- Mate connectors drive the kinematics via their **names** (see below).
- Orphaned links that never reach the kinematic chain are **fixed to the
  base link**, with a warning.

## Naming conventions for mate connectors

Only these prefixes are interpreted; anything else is ignored.

| Prefix      | Meaning                                               |
|-------------|-------------------------------------------------------|
| `dof_...`   | Create a degree of freedom (joint).                   |
| `frame_...` | Create a named frame (site in MuJoCo).                |
| `fix_...`   | Weld two links together (merge them).                 |
| `closing_...` | Close a kinematic loop (see [kinematic_loops.md](kinematic_loops.md)). |
| `link_...`  | Override a link's name.                               |

The **base mate type** determines the joint type of a `dof_` mate:

| Onshape mate type         | Exported joint type |
|---------------------------|---------------------|
| Cylindrical / Revolute    | `revolute`          |
| Slider                    | `prismatic`         |
| Fastened                  | `fixed`             |
| Ball                      | `ball`              |

Joint **limits** set in Onshape are exported. To disable, set
`ignore_limits: true`. Continuous-rotation joints (`wheel` in the name, or
the word `continuous`) are emitted as `continuous` URDF joints.

### Inverting axis direction

Suffix a DOF with `_inv` (or `_inverted`) to flip the joint axis:

```
dof_head_pitch_inv   → joint "head_pitch" with reversed axis
```

This triggers `DOF.flip()` which applies a 180° rotation around X and flips
the limits.

### Naming a link

`link_<name>` on a mate connector attached to an instance renames that
instance's link. Example: `link_torso` produces `<link name="torso">`.

### Custom frames

Two ways to attach a named frame to a body:

1. Add a mate connector named `frame_<name>`, OR
2. Add any relation named `frame_<name>` between a "frame representation"
   body and the body it should attach to.

Frames become:

| Format  | Representation                           |
|---------|------------------------------------------|
| URDF    | Dummy link + `fixed` joint               |
| SDF     | Native `<frame>` element                 |
| MuJoCo  | `<site>` inside the parent body          |

Onshape "frame" representation parts are **excluded** from the output by
default. Set `draw_frames: true` in `o2r.json` to keep them (useful when
debugging). There is a ready-made frame part at:
<https://cad.onshape.com/documents/7adc786257f47ce24706bb32/w/774dd3de6bd5bfd65fb4462b/e/c60f72b9088ac4e5058b8904?renderMode=0&uiState=67b64076077d3a02bf5e1c0f>
(insert with composite parts turned on).

### Joint frames

When you click a mate connector in the Onshape tree, the axes shown are the
joint frame. **Z is always the rotation/translation axis**.

## Fixed robot

Use Onshape's "Fixed" feature to lock the base link to ground. In MuJoCo,
the `<freejoint>` is omitted; in URDF/SDF, no free base is created.

## Multiple base links

If multiple disconnected kinematic chains exist, each becomes its own base
link.

- URDF does not support multiple base links. Either write multiple URDF
  files or enable [`add_dummy_base_link`](processors/dummy_base_link.md)
  (this fixes all bases to a single `base_link`, removing their freedom).
- SDF and MuJoCo support multiple base links natively.

## Gear relations

Onshape gear relations are exported as:

- URDF / SDF: `<mimic joint="..." multiplier="..."/>`
- MuJoCo: equality constraints (`<joint joint1=... joint2=... polycoef="..."/>`)

**Click the source joint first, then the target joint** when creating the
relation in Onshape. The sign of `reverseDirection` controls the ratio sign
during export.
