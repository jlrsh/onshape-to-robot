"""Unified URDF viewer for robots, attachments, and RTUs.

Usage:
    python viz.py --robot <name> [--spherized] [--glb]
    python viz.py --attachment <name>
    python viz.py --rtu <family>/<carriage>

In --robot mode, VAMP collision checking and path planning are wired up when
vamp is installed and the robot is registered in vamp.robots.
"""

from __future__ import annotations

import json
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import tyro
import viser
from viser.extras import ViserUrdf
from scipy.spatial.transform import Rotation

try:
    import vamp

    VAMP_AVAILABLE = True
except ImportError:
    VAMP_AVAILABLE = False

ASSETS_DIR = Path(__file__).resolve().parent 

# Debug visualization colors/opacity.
DEBUG_SPHERE_VALID_COLOR = (0, 255, 0)
DEBUG_SPHERE_INVALID_COLOR = (255, 0, 0)
DEBUG_SPHERE_OPACITY = 0.5
DEBUG_CUBOID_COLOR = (0, 128, 255)
DEBUG_CUBOID_OPACITY = 0.4


# ---------------------------------------------------------------------------
# URDF parsing utilities
# ---------------------------------------------------------------------------


def _strip_tag_namespace(tag: str) -> str:
    return tag.split("}", 1)[-1]


def get_actuated_joint_info(
    urdf_path: Path,
) -> dict[str, dict[str, float | None | str]]:
    """Return ordered joint metadata for non-fixed joints: type and limits."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joint_info: dict[str, dict[str, float | None | str]] = {}
    for elem in root.iter():
        if _strip_tag_namespace(elem.tag) != "joint":
            continue
        joint_type = elem.attrib.get("type", "").lower()
        if joint_type == "fixed":
            continue
        joint_name = elem.attrib.get("name")
        if not joint_name:
            continue
        lower = None
        upper = None
        for child in elem:
            if _strip_tag_namespace(child.tag) != "limit":
                continue
            lower = child.attrib.get("lower")
            upper = child.attrib.get("upper")
            lower = float(lower) if lower is not None else None
            upper = float(upper) if upper is not None else None
            break
        joint_info[joint_name] = {
            "type": joint_type,
            "lower": lower,
            "upper": upper,
        }
    return joint_info


def find_special_frames(
    urdf_path: Path,
) -> list[tuple[str, str, np.ndarray, np.ndarray]]:
    """Find joints whose names contain "frame" and return
    (joint_name, viser_path, position, wxyz)."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    parent_map: dict[str, str] = {}
    for elem in root.iter():
        if _strip_tag_namespace(elem.tag) != "joint":
            continue
        parent_elem = elem.find("parent")
        child_elem = elem.find("child")
        if parent_elem is not None and child_elem is not None:
            parent = parent_elem.attrib.get("link")
            child = child_elem.attrib.get("link")
            if parent and child:
                parent_map[child] = parent

    all_links = {l.get("name") for l in root.findall("link") if l.get("name")}
    root_links = all_links - set(parent_map.keys())

    found_frames: list[tuple[str, str, np.ndarray, np.ndarray]] = []

    for elem in root.iter():
        if _strip_tag_namespace(elem.tag) != "joint":
            continue
        joint_name = elem.attrib.get("name", "")
        if "frame" not in joint_name.lower():
            continue

        parent_elem = elem.find("parent")
        if parent_elem is None:
            continue
        parent_link = parent_elem.attrib.get("link")
        if not parent_link:
            continue

        origin_elem = elem.find("origin")
        if origin_elem is not None:
            xyz = np.array(
                [float(v) for v in origin_elem.attrib.get("xyz", "0 0 0").split()]
            )
            rpy = np.array(
                [float(v) for v in origin_elem.attrib.get("rpy", "0 0 0").split()]
            )
        else:
            xyz = np.zeros(3)
            rpy = np.zeros(3)

        r, p, y = rpy / 2
        wxyz = np.array(
            [
                np.cos(r) * np.cos(p) * np.cos(y)
                + np.sin(r) * np.sin(p) * np.sin(y),
                np.sin(r) * np.cos(p) * np.cos(y)
                - np.cos(r) * np.sin(p) * np.sin(y),
                np.cos(r) * np.sin(p) * np.cos(y)
                + np.sin(r) * np.cos(p) * np.sin(y),
                np.cos(r) * np.cos(p) * np.sin(y)
                - np.sin(r) * np.sin(p) * np.cos(y),
            ]
        )

        # Build viser scene path by walking from parent link up to the root.
        chain: list[str] = []
        current = parent_link
        while current and current not in root_links:
            chain.append(current)
            current = parent_map.get(current)
        chain.reverse()
        if chain:
            viser_path = "/visual/" + "/".join(chain) + "/" + joint_name
        else:
            viser_path = "/visual/" + joint_name

        found_frames.append((joint_name, viser_path, xyz, wxyz))

    return found_frames


def _spherized_or_fallback_urdf(robot_name: str) -> Path:
    """Prefer spherized.urdf; fall back to robot.urdf. Used by VAMP cuboid init."""
    for name in ("spherized.urdf", "robot.urdf"):
        maybe = ASSETS_DIR / "robots" / robot_name / name
        if maybe.exists():
            return maybe.resolve()
    raise FileNotFoundError(f"No URDF for robot '{robot_name}'")


def _load_hp_params(hp_json_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load H and P matrices from an hp.json file.

    Returns (H, P) where H is 3xN and P is 3x(N+1).
    """
    with open(hp_json_path) as f:
        data = json.load(f)
    hp = data["hp_params"]
    H = np.array([hp["h_0"], hp["h_1"], hp["h_2"]])
    P = np.array([hp["p_0"], hp["p_1"], hp["p_2"]])
    return H, P


def _make_viser_cfg_converter(vamp_module: Any, viser_urdf: ViserUrdf):
    """Return a function that reorders a vamp-ordered config to viser joint order."""
    vamp_joint_names = list(vamp_module.joint_names())
    vamp_name_to_index = {name: idx for idx, name in enumerate(vamp_joint_names)}

    viser_joint_order = list(viser_urdf.get_actuated_joint_names())
    missing = [name for name in viser_joint_order if name not in vamp_name_to_index]
    if missing:
        raise RuntimeError(
            "URDF joints missing from VAMP model: " + ", ".join(missing)
        )

    def _converter(config: np.ndarray) -> np.ndarray:
        return np.asarray(
            [config[vamp_name_to_index[name]] for name in viser_joint_order],
            dtype=float,
        )

    return _converter


def _create_debug_spheres(
    server: viser.ViserServer, root_node_name: str, count: int
) -> list:
    debug_root = f"{root_node_name.rstrip('/')}/debug"
    server.scene.add_frame(debug_root, show_axes=False)
    handles = []
    for idx in range(count):
        handle = server.scene.add_icosphere(
            name=f"{debug_root}/sphere_{idx}",
            radius=0.05,
            color=DEBUG_SPHERE_VALID_COLOR,
            opacity=DEBUG_SPHERE_OPACITY,
        )
        handles.append(handle)
    return handles


def _parse_urdf_boxes(urdf_path: Path) -> list[dict]:
    """Parse URDF for box collision geometries."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    boxes = []
    for link in root.findall("link"):
        link_name = link.get("name")
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            if geometry is None:
                continue
            box = geometry.find("box")
            if box is None:
                continue
            size = [float(x) for x in box.get("size", "0 0 0").split()]
            origin = collision.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
                rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
            boxes.append(
                {"link_name": link_name, "size": size, "xyz": xyz, "rpy": rpy}
            )
    return boxes


def _build_link_scene_paths(urdf_path: Path, root_node_name: str) -> dict[str, str]:
    """Build a mapping from link name to its viser scene path matching ViserUrdf's
    internal hierarchy.

    ViserUrdf places visual geometry under ``{root_node_name}/visual`` and nests
    child links following the kinematic chain (omitting the root link name from
    the path).  For example, with root ``"/"`` and kinematic chain
    ``rail -> carriage -> base_link``, the paths are::

        rail       -> /visual
        carriage   -> /visual/carriage
        base_link  -> /visual/carriage/base_link
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    child_to_parent: dict[str, str] = {}
    for joint_elem in root.findall("joint"):
        parent_elem = joint_elem.find("parent")
        child_elem = joint_elem.find("child")
        if parent_elem is not None and child_elem is not None:
            parent_link = parent_elem.get("link")
            child_link = child_elem.get("link")
            if parent_link and child_link:
                child_to_parent[child_link] = parent_link

    all_links = {link.get("name") for link in root.findall("link") if link.get("name")}
    root_links = all_links - set(child_to_parent.keys())

    visual_prefix = f"{root_node_name.rstrip('/')}/visual"

    paths: dict[str, str] = {}
    for link_name in all_links:
        chain: list[str] = []
        current = link_name
        while current not in root_links:
            chain.append(current)
            current = child_to_parent[current]
        if chain:
            paths[link_name] = visual_prefix + "/" + "/".join(reversed(chain))
        else:
            paths[link_name] = visual_prefix
    return paths


def _create_debug_cuboids(
    server: viser.ViserServer, root_node_name: str, urdf_path: Path
) -> list:
    """Create viser boxes for each URDF box collision, parented to the correct link frame."""
    boxes = _parse_urdf_boxes(urdf_path)
    link_scene_paths = _build_link_scene_paths(urdf_path, root_node_name)
    handles = []
    for idx, box_info in enumerate(boxes):
        link_name = box_info["link_name"]
        link_path = link_scene_paths.get(link_name, f"{root_node_name}/{link_name}")
        xyzw = Rotation.from_euler("XYZ", box_info["rpy"]).as_quat()
        wxyz = np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]])
        handle = server.scene.add_box(
            name=f"{link_path}/collision_box_{idx}",
            dimensions=tuple(box_info["size"]),
            position=tuple(box_info["xyz"]),
            wxyz=tuple(wxyz),
            color=DEBUG_CUBOID_COLOR,
            opacity=DEBUG_CUBOID_OPACITY,
        )
        handles.append(handle)
    return handles


def _compute_invalid_sphere_indices(debug_info) -> set[int]:
    reason_lists, colliding_pairs = debug_info
    invalid = {idx for idx, reasons in enumerate(reason_lists) if reasons}
    for a, b in colliding_pairs:
        invalid.add(a)
        invalid.add(b)
    return invalid


def _update_debug_spheres(handles, spheres, invalid_indices: set[int]) -> None:
    if len(spheres) != len(handles):
        raise RuntimeError("Mismatch between FK spheres and debug handles")
    for idx, (handle, sphere) in enumerate(zip(handles, spheres)):
        handle.radius = sphere.r
        handle.position = (sphere.x, sphere.y, sphere.z)
        handle.color = (
            DEBUG_SPHERE_INVALID_COLOR
            if idx in invalid_indices
            else DEBUG_SPHERE_VALID_COLOR
        )
        handle.opacity = DEBUG_SPHERE_OPACITY


# ---------------------------------------------------------------------------
# Application state
# ---------------------------------------------------------------------------


@dataclass
class AppState:
    """Mutable application state shared across all GUI callbacks."""

    server: viser.ViserServer = field(repr=False)
    viser_urdf: ViserUrdf = field(repr=False)
    urdf_path: Path = field(repr=False)
    root_node_name: str = "/"

    # Joint control
    slider_handles: list[Any] = field(default_factory=list)
    slider_joint_names: list[str] = field(default_factory=list)
    viser_joint_order: list[str] = field(default_factory=list)
    joint_info: dict[str, dict] = field(default_factory=dict)
    initial_slider_config: list[float] = field(default_factory=list)
    initial_viser_config: list[float] = field(default_factory=list)

    # VAMP state (None/empty when vamp is not active)
    vamp_robot_name: str | None = None
    vamp_module: Any = None
    planner_func: Any = None
    plan_settings: Any = None
    simp_settings: Any = None
    viser_cfg_converter: Any = None
    vamp_joint_names: list[str] = field(default_factory=list)
    dof: int = 0

    # Debug visualization handles
    debug_sphere_handles: list[Any] = field(default_factory=list)
    debug_cuboid_handles: list[Any] = field(default_factory=list)

    # eaik (optional FK point cloud)
    hpbot: Any = None
    enable_eaik: bool = False
    hp_base_correction: np.ndarray = field(default_factory=lambda: np.eye(4))

    # Flag to suppress cascading slider callbacks during bulk updates
    suppress_slider_callbacks: bool = False


# ---------------------------------------------------------------------------
# VAMP helper functions
# ---------------------------------------------------------------------------


def _update_fk_point_cloud(state: AppState, slider_config: np.ndarray) -> None:
    """Recompute and display the eaik FK point cloud."""
    if not state.enable_eaik or state.hpbot is None:
        return

    # Strip prismatic (RTU) joint — eaik expects only revolute joints
    rev_mask = [
        state.joint_info[n]["type"] in ("revolute", "continuous")
        for n in state.slider_joint_names
    ]
    robot_config = slider_config[rev_mask]

    hp_fk = state.hpbot.fwdKin(robot_config)
    if hp_fk is None:
        return

    # Get world-to-base_link from yourdfpy (accounts for carriage/RTU position)
    T_world_base = state.viser_urdf._urdf.get_transform("base_link")

    fk_world = T_world_base @ state.hp_base_correction @ hp_fk

    state.server.scene.add_point_cloud(
        "fk",
        points=fk_world[:3, 3].reshape(1, 3),
        colors=(0, 255, 0),
        point_size=0.05,
    )


def _vamp_config_to_slider(
    vamp_config: np.ndarray, state: AppState,
) -> np.ndarray:
    """Reorder a vamp-ordered config to slider joint order."""
    name_to_val = dict(zip(state.vamp_joint_names, vamp_config))
    return np.array(
        [name_to_val.get(n, 0.0) for n in state.slider_joint_names], dtype=np.float32
    )


def _update_vamp_visualization(state: AppState, config_np: np.ndarray) -> bool:
    """Update debug spheres and cuboids for *config_np* (vamp joint order). Return validity."""
    if state.vamp_module is None or not state.debug_sphere_handles:
        return True
    is_valid = bool(state.vamp_module.validate(config_np))
    debug_info = state.vamp_module.debug(config_np)
    invalid_indices = _compute_invalid_sphere_indices(debug_info)

    # validate() can detect collisions that debug() misses (different
    # internal checks).  When that happens, mark ALL spheres invalid so
    # the visual feedback matches the validate() verdict.
    if not is_valid and not invalid_indices:
        invalid_indices = set(range(len(state.debug_sphere_handles)))

    spheres = state.vamp_module.fk(config_np)
    _update_debug_spheres(state.debug_sphere_handles, spheres, invalid_indices)
    cuboid_color = DEBUG_SPHERE_VALID_COLOR if is_valid else DEBUG_SPHERE_INVALID_COLOR
    for handle in state.debug_cuboid_handles:
        handle.color = cuboid_color
    return is_valid


def _display_config_on_robot(state: AppState, config_np: np.ndarray) -> None:
    """Set viser URDF and sliders to *config_np* (vamp joint order).

    IMPORTANT: caller must set ``state.suppress_slider_callbacks = True``
    before calling and reset it to ``False`` *after* any subsequent
    ``_update_vamp_visualization`` call.  This prevents async slider
    callbacks from overwriting the debug-sphere colours with a stale
    (slider-rounded) configuration.
    """
    if state.viser_cfg_converter is None:
        return
    viser_cfg = state.viser_cfg_converter(config_np)
    state.viser_urdf.update_cfg(viser_cfg)

    vamp_name_to_val = dict(zip(state.vamp_joint_names, config_np))
    for name, handle in zip(state.slider_joint_names, state.slider_handles):
        if name in vamp_name_to_val:
            handle.value = float(
                np.clip(vamp_name_to_val[name], handle.min, handle.max)
            )


def _slider_config_to_vamp(
    slider_config: np.ndarray,
    slider_joint_names: list[str],
    state: AppState,
) -> np.ndarray:
    """Reorder a slider-ordered config to vamp joint order."""
    if not state.vamp_joint_names:
        return slider_config
    name_to_val = dict(zip(slider_joint_names, slider_config))
    return np.array(
        [name_to_val.get(n, 0.0) for n in state.vamp_joint_names], dtype=np.float32
    )


def _initialize_vamp_robot(state: AppState, robot_name: str) -> None:
    """Initialise VAMP module and create debug scene nodes for *robot_name*."""
    _teardown_vamp_robot(state)

    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs(robot_name, "rrtc")
    )

    plan_settings.max_iterations = 1_000_000
    plan_settings.max_samples = 1000
    plan_settings.range = 1.25

    state.vamp_module = vamp_module
    state.planner_func = planner_func
    state.plan_settings = plan_settings
    state.simp_settings = simp_settings
    state.vamp_robot_name = robot_name
    state.vamp_joint_names = list(vamp_module.joint_names())
    state.dof = len(state.vamp_joint_names)

    state.viser_cfg_converter = _make_viser_cfg_converter(vamp_module, state.viser_urdf)

    state.debug_sphere_handles = _create_debug_spheres(
        state.server, state.root_node_name, vamp_module.n_spheres()
    )
    # Use the spherized URDF for cuboids — robot.urdf has no <box> elements,
    # only spherized.urdf contains them.
    spherized_urdf = _spherized_or_fallback_urdf(robot_name)
    state.debug_cuboid_handles = _create_debug_cuboids(
        state.server, state.root_node_name, spherized_urdf
    )

    slider_config = np.array(
        [h.value for h in state.slider_handles], dtype=np.float32
    )
    vamp_config = _slider_config_to_vamp(slider_config, state.slider_joint_names, state)
    _update_vamp_visualization(state, vamp_config)


def _teardown_vamp_robot(state: AppState) -> None:
    """Remove debug scene nodes and reset VAMP fields."""
    for handle in state.debug_sphere_handles:
        handle.remove()
    for handle in state.debug_cuboid_handles:
        handle.remove()
    debug_root = f"{state.root_node_name.rstrip('/')}/debug"
    try:
        state.server.scene.remove(debug_root)
    except Exception:
        pass

    state.debug_sphere_handles.clear()
    state.debug_cuboid_handles.clear()
    state.vamp_module = None
    state.planner_func = None
    state.plan_settings = None
    state.simp_settings = None
    state.viser_cfg_converter = None
    state.vamp_joint_names.clear()
    state.vamp_robot_name = None
    state.dof = 0


def _parse_config_text(text: str, expected_dof: int) -> np.ndarray | None:
    """Parse comma-separated floats. Return None on bad input or wrong length."""
    try:
        values = [float(v.strip()) for v in text.split(",") if v.strip()]
    except ValueError:
        return None
    if len(values) != expected_dof:
        return None
    return np.array(values, dtype=np.float32)


# ---------------------------------------------------------------------------
# Slider creation
# ---------------------------------------------------------------------------


def create_robot_control_sliders(
    server: viser.ViserServer,
    viser_urdf: ViserUrdf,
    state: AppState,
    ordered_joint_names: list[str] | None = None,
    joint_info: dict[str, dict[str, float | None | str]] | None = None,
) -> tuple[list[viser.GuiInputHandle[float]], list[float], list[float]]:
    joint_limits = viser_urdf.get_actuated_joint_limits()
    viser_joint_order = list(joint_limits.keys())

    slider_handles: list[viser.GuiInputHandle[float]] = []
    slider_joint_names: list[str] = []
    initial_slider_config: list[float] = []

    if ordered_joint_names is None:
        slider_joint_names = viser_joint_order.copy()
    else:
        slider_joint_names = [
            name for name in ordered_joint_names if name in joint_limits
        ]
        missing = [name for name in ordered_joint_names if name not in joint_limits]
        if missing:
            print(
                "Warning: joints found in URDF but missing from visualization:",
                ", ".join(missing),
            )

    state.slider_joint_names = slider_joint_names
    state.viser_joint_order = viser_joint_order

    def current_values_by_name() -> dict[str, float]:
        return {
            name: handle.value
            for name, handle in zip(slider_joint_names, slider_handles)
        }

    for joint_name in slider_joint_names:
        joint_meta = joint_info.get(joint_name) if joint_info else None
        joint_type = joint_meta.get("type") if joint_meta else "revolute"
        lower, upper = joint_limits[joint_name]
        if joint_meta:
            lower = (
                joint_meta.get("lower")
                if joint_meta.get("lower") is not None
                else lower
            )
            upper = (
                joint_meta.get("upper")
                if joint_meta.get("upper") is not None
                else upper
            )
        if joint_type == "prismatic":
            default_range = 0.1
        else:
            default_range = np.pi
        lower = lower if lower is not None else -default_range
        upper = upper if upper is not None else default_range

        initial_pos = (
            0.0 if lower < -0.1 and upper > 0.1 else (lower + upper) / 2.0
        )
        step = 1e-4 if joint_type == "prismatic" else 1e-3

        slider = server.gui.add_slider(
            label=joint_name,
            min=lower,
            max=upper,
            step=step,
            initial_value=initial_pos,
        )

        def _on_slider_update(_):
            if state.suppress_slider_callbacks:
                return

            values_by_name = current_values_by_name()
            slider_config = np.array(
                [values_by_name[name] for name in slider_joint_names]
            )

            viser_cfg = np.array(
                [
                    values_by_name[name]
                    for name in viser_joint_order
                    if name in values_by_name
                ]
            )
            viser_urdf.update_cfg(viser_cfg)

            _update_fk_point_cloud(state, slider_config)

            if state.vamp_module is not None and state.debug_sphere_handles:
                vamp_config = _slider_config_to_vamp(
                    slider_config, slider_joint_names, state
                )
                _update_vamp_visualization(state, vamp_config)

        slider.on_update(_on_slider_update)
        slider_handles.append(slider)
        initial_slider_config.append(initial_pos)

    initial_values_by_name = dict(zip(slider_joint_names, initial_slider_config))
    initial_viser_config = [
        initial_values_by_name[name]
        for name in viser_joint_order
        if name in initial_values_by_name
    ]

    return slider_handles, initial_slider_config, initial_viser_config


# ---------------------------------------------------------------------------
# Mode resolution
# ---------------------------------------------------------------------------


def _resolve_mode_and_urdf(
    robot: str | None,
    attachment: str | None,
    rtu: str | None,
    spherized: bool,
) -> tuple[str, str, Path]:
    """Validate mutual exclusivity and return (mode, name, urdf_path)."""
    provided = [(k, v) for k, v in (("robot", robot), ("attachment", attachment), ("rtu", rtu)) if v is not None]
    if len(provided) != 1:
        raise RuntimeError(
            "Exactly one of --robot, --attachment, --rtu must be provided "
            f"(got {len(provided)}: {[k for k, _ in provided]})"
        )
    mode, name = provided[0]

    if mode == "robot":
        urdf_name = "spherized.urdf" if spherized else "robot.urdf"
        urdf_path = (ASSETS_DIR / "robots" / name / urdf_name).resolve()
    elif mode == "attachment":
        urdf_path = (ASSETS_DIR / "attachments" / name / "attachment.urdf").resolve()
    else:  # rtu
        urdf_path = (ASSETS_DIR / "rtu" / name / "robot_glb.urdf").resolve()

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")
    return mode, name, urdf_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(
    robot: str | None = None,
    attachment: str | None = None,
    rtu: str | None = None,
    load_meshes: bool = True,
    load_collision_meshes: bool = True,
    spherized: bool = False,
    glb: bool = False,
) -> None:
    """Unified URDF viewer.

    Args:
        robot: Robot name — loads robots/<name>/robot.urdf (or spherized.urdf).
            Enables VAMP collision checking and path planning when vamp is
            installed and the robot is registered in vamp.robots.
        attachment: Attachment name — loads attachments/<name>/attachment.urdf.
        rtu: RTU path — loads rtu/<path>/robot_glb.urdf (e.g.
            "tmf3/extended_carriage").
        load_meshes: Whether to load visual meshes.
        load_collision_meshes: Whether to load collision meshes.
        spherized: Use spherized.urdf instead of robot.urdf (--robot only).
        glb: Use .glb meshes instead of .stl when loading the URDF (--robot only).
    """

    mode, name, urdf_path = _resolve_mode_and_urdf(robot, attachment, rtu, spherized)

    server = viser.ViserServer()
    print("Viser server started (check terminal for URL)")
    print(f"Loading URDF from: {urdf_path}")

    if mode == "robot" and glb:
        import yourdfpy

        def _glb_filename_handler(fname: str) -> str:
            p = Path(fname)
            if p.suffix.lower() == ".stl":
                p = p.with_suffix(".glb")
            return str(urdf_path.parent / p)

        urdf_model = yourdfpy.URDF.load(
            str(urdf_path),
            filename_handler=_glb_filename_handler,
            load_meshes=load_meshes,
            load_collision_meshes=load_collision_meshes,
            build_collision_scene_graph=load_collision_meshes,
        )
        viser_urdf = ViserUrdf(
            server,
            urdf_or_path=urdf_model,
            load_meshes=load_meshes,
            load_collision_meshes=load_collision_meshes,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
        )
    else:
        viser_urdf = ViserUrdf(
            server,
            urdf_or_path=urdf_path,
            load_meshes=load_meshes,
            load_collision_meshes=load_collision_meshes,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
        )
    print("URDF loaded successfully")

    state = AppState(server=server, viser_urdf=viser_urdf, urdf_path=urdf_path)

    joint_info = get_actuated_joint_info(urdf_path)
    joint_names = list(joint_info.keys())
    state.joint_info = joint_info

    # --- eaik (robot mode only) -----------------------------------------------
    if mode == "robot":
        hp_json_path = ASSETS_DIR / "robots" / name / "hp.json"
        if hp_json_path.exists():
            try:
                import eaik

                H, P = _load_hp_params(hp_json_path)
                try:
                    state.hpbot = eaik.HPRobot(H.T, P.T, None)
                    state.enable_eaik = True

                    n_rev = sum(
                        1 for n in joint_names
                        if joint_info[n]["type"] in ("revolute", "continuous")
                    )
                    zero_cfg = {n: 0.0 for n in viser_urdf.get_actuated_joint_names()}
                    viser_urdf._urdf.update_cfg(zero_cfg)
                    T_world_base_zero = viser_urdf._urdf.get_transform("base_link")
                    T_world_flange_zero = viser_urdf._urdf.get_transform("flange")
                    FK_zero = state.hpbot.fwdKin(np.zeros(n_rev))
                    if FK_zero is not None:
                        T_base_to_flange = np.linalg.inv(T_world_base_zero) @ T_world_flange_zero
                        state.hp_base_correction = T_base_to_flange @ np.linalg.inv(FK_zero)

                    print(f"eaik loaded successfully (hp.json: {hp_json_path.name})")
                except Exception as exc:
                    print(f"Warning: eaik init failed ({exc}). Disabling eaik visuals.")
            except ImportError:
                print("eaik not installed -- FK point clouds disabled")
        else:
            print(f"No hp.json found at {hp_json_path} -- FK point clouds disabled")

    # --- Joint Position Control folder ----------------------------------------
    with server.gui.add_folder("Joint Position Control"):
        (
            slider_handles,
            initial_slider_config,
            initial_viser_config,
        ) = create_robot_control_sliders(
            server,
            viser_urdf,
            state,
            ordered_joint_names=joint_names,
            joint_info=joint_info,
        )
    state.slider_handles = slider_handles
    state.initial_slider_config = initial_slider_config
    state.initial_viser_config = initial_viser_config

    # --- Auto-initialise VAMP (robot mode only) -------------------------------
    vamp_active = (
        mode == "robot"
        and VAMP_AVAILABLE
        and name in vamp.robots
    )
    if vamp_active:
        _initialize_vamp_robot(state, name)
        print(
            f"VAMP '{name}' loaded: {state.dof} DOF, "
            f"{len(state.debug_sphere_handles)} debug spheres, "
            f"{len(state.debug_cuboid_handles)} debug cuboids"
        )
    elif mode == "robot" and not VAMP_AVAILABLE:
        print("vamp not installed -- VAMP collision checking & path planning disabled")
    elif mode == "robot":
        print(f"Robot '{name}' not in vamp.robots -- VAMP features disabled")

    # --- Special frames -------------------------------------------------------
    special_frames = find_special_frames(urdf_path)

    # --- Visibility folder ----------------------------------------------------
    with server.gui.add_folder("Visibility"):
        show_meshes_cb = server.gui.add_checkbox(
            "Show meshes", viser_urdf.show_visual
        )
        show_collision_meshes_cb = server.gui.add_checkbox(
            "Show collision meshes", viser_urdf.show_collision
        )
        frame_checkboxes = []
        for frame_name, _, __, ___ in special_frames:
            cb = server.gui.add_checkbox(
                f"Show {frame_name} frame", initial_value=True
            )
            frame_checkboxes.append(cb)

    @show_meshes_cb.on_update
    def _(_):
        viser_urdf.show_visual = show_meshes_cb.value

    @show_collision_meshes_cb.on_update
    def _(_):
        viser_urdf.show_collision = show_collision_meshes_cb.value

    show_meshes_cb.visible = load_meshes
    show_collision_meshes_cb.visible = load_collision_meshes

    viser_urdf.update_cfg(np.array(initial_viser_config))

    frame_handles = []
    for i, (frame_name, frame_path, position, wxyz) in enumerate(special_frames):
        try:
            axes_length = 0.25 + (i * 0.05)
            axes_radius = 0.01 + (i * 0.002)
            frame_handle = server.scene.add_frame(
                frame_path,
                wxyz=wxyz,
                position=position,
                axes_length=axes_length,
                axes_radius=axes_radius,
            )
            frame_handles.append(frame_handle)
            print(f"Added {frame_name} frame at: {frame_path}")
        except Exception as exc:
            print(f"Warning: Could not add {frame_name} frame at {frame_path}: {exc}")
            frame_handles.append(None)

    for checkbox, frame_handle in zip(frame_checkboxes, frame_handles):
        if frame_handle is not None:

            def make_callback(handle, cb):
                def callback(_):
                    handle.visible = cb.value

                return callback

            checkbox.on_update(make_callback(frame_handle, checkbox))

    # --- Grid -----------------------------------------------------------------
    server.scene.add_grid(
        "/grid",
        width=2,
        height=2,
        position=(0.0, 0.0, 0.0),
    )

    # --- Reset button ---------------------------------------------------------
    reset_button = server.gui.add_button("Reset")

    @reset_button.on_click
    def _(_):
        for s, init_q in zip(slider_handles, initial_slider_config):
            s.value = init_q

    # --- VAMP Collision Checking & Path Planning (robot + vamp only) ----------
    if vamp_active:
        with server.gui.add_folder("VAMP Collision Checking"):
            vamp_status = server.gui.add_markdown(
                f"**{name}** loaded: {state.dof} DOF, "
                f"{len(state.debug_sphere_handles)} debug spheres, "
                f"{len(state.debug_cuboid_handles)} debug cuboids"
            )
            random_sample_btn = server.gui.add_button("Random Sample")
            show_spheres_cb = server.gui.add_checkbox(
                "Show debug spheres", initial_value=True
            )
            show_cuboids_cb = server.gui.add_checkbox(
                "Show debug cuboids", initial_value=True
            )

        @random_sample_btn.on_click
        def _(_):
            lower = []
            upper = []
            for jname in state.vamp_joint_names:
                info = state.joint_info.get(jname)
                if info is None:
                    vamp_status.content = (
                        f"Joint '{jname}' not found in URDF joint info"
                    )
                    return
                lo = info.get("lower")
                hi = info.get("upper")
                if lo is None or hi is None:
                    vamp_status.content = (
                        f"Joint '{jname}' missing limits in URDF"
                    )
                    return
                lower.append(lo)
                upper.append(hi)
            lower_np = np.array(lower, dtype=np.float32)
            upper_np = np.array(upper, dtype=np.float32)
            config_np = np.random.uniform(lower_np, upper_np).astype(np.float32)

            state.suppress_slider_callbacks = True
            try:
                _display_config_on_robot(state, config_np)
                is_valid = _update_vamp_visualization(state, config_np)
                _update_fk_point_cloud(state, _vamp_config_to_slider(config_np, state))
            finally:
                state.suppress_slider_callbacks = False
            vamp_status.content = (
                f"Random config: **{'VALID' if is_valid else 'INVALID'}**\n\n"
                f"`[{', '.join(f'{v:.4f}' for v in config_np)}]`"
            )

        @show_spheres_cb.on_update
        def _(_):
            for h in state.debug_sphere_handles:
                h.visible = show_spheres_cb.value

        @show_cuboids_cb.on_update
        def _(_):
            for h in state.debug_cuboid_handles:
                h.visible = show_cuboids_cb.value

        zero_cfg = ", ".join(["0"] * state.dof)
        with server.gui.add_folder("Path Planning"):
            start_text = server.gui.add_text(
                "Start config",
                initial_value=zero_cfg,
            )
            goal_text = server.gui.add_text(
                "Goal config",
                initial_value=zero_cfg,
            )
            plan_btn = server.gui.add_button("Plan & Visualize")
            plan_status = server.gui.add_markdown(
                "Enter start and goal configs above"
            )

        @plan_btn.on_click
        def _(_):
            start = _parse_config_text(start_text.value, state.dof)
            goal = _parse_config_text(goal_text.value, state.dof)
            if start is None:
                plan_status.content = (
                    f"Invalid start config (expected {state.dof} comma-separated floats)"
                )
                return
            if goal is None:
                plan_status.content = (
                    f"Invalid goal config (expected {state.dof} comma-separated floats)"
                )
                return

            env = vamp.Environment()
            sampler = state.vamp_module.halton()
            plan_status.content = "Planning..."

            try:
                result = state.planner_func(
                    start, goal, env, state.plan_settings, sampler
                )
            except Exception as exc:
                plan_status.content = f"Planning error: {exc}"
                return

            if not result.solved:
                plan_status.content = "Planning **FAILED** -- no path found"
                return

            try:
                simp_result = state.vamp_module.simplify(
                    result.path, env, state.simp_settings, sampler
                )
                path = simp_result.path
                path.interpolate_to_resolution(state.vamp_module.resolution())
                path_np = path.numpy()
            except Exception as exc:
                plan_status.content = f"Simplification error: {exc}"
                return

            n_states = path_np.shape[0]
            plan_status.content = f"Path found: {n_states} states. Visualizing..."

            for i in range(n_states):
                waypoint = path_np[i].astype(np.float32)
                state.suppress_slider_callbacks = True
                try:
                    _display_config_on_robot(state, waypoint)
                    is_valid = _update_vamp_visualization(state, waypoint)
                    _update_fk_point_cloud(state, _vamp_config_to_slider(waypoint, state))
                finally:
                    state.suppress_slider_callbacks = False

                status = "VALID" if is_valid else "INVALID"
                plan_status.content = f"State {i + 1}/{n_states}: **{status}**"

                if not is_valid:
                    plan_status.content += " -- STOPPED (invalid state)"
                    return

                time.sleep(0.1)

            plan_status.content = f"Path complete: all {n_states} states valid"

    while True:
        time.sleep(10)


if __name__ == "__main__":
    tyro.cli(main)
