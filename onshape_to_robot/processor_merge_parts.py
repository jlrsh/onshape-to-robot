import json
import numpy as np
import os
import shutil
from .config import Config
from .robot import Robot, Link, Part
from .processor import Processor
from .geometry import Mesh
from .message import bright, info, error, warning
from .mesh_adapter import mesh_adapter_for


def _glb_material_key(geom) -> tuple:
    """Extract a hashable key from a trimesh geometry's material for grouping."""
    v = geom.visual
    if hasattr(v, "material") and v.material and hasattr(v.material, "baseColorFactor"):
        c = v.material.baseColorFactor
        if c is not None:
            return tuple(int(x) for x in c)
    return ("default",)


class ProcessorMergeParts(Processor):
    """
    This processor merge all parts into a single one, combining the STL
    """

    def __init__(self, config: Config):
        super().__init__(config)
        self.merge_stls = config.get("merge_stls", False)
        self.collisions_as_visual = config.get("collisions_as_visual", False)
        self.mesh_format = config.get("mesh_format", "stl")
        self.adapter = mesh_adapter_for(self.mesh_format)
        self.mesh_ext = self.adapter.extension
        # Cache once so path-traversal checks in cleanup_merged_sources don't
        # recompute per file.
        self._assets_root = os.path.abspath(self.config.asset_path(""))

    def process(self, robot: Robot):
        if self.merge_stls:
            merged_source_files = set()
            getcwd = os.getcwd()
            os.chdir(self.config.output_directory)
            for link in robot.links:
                self.write_link_manifest(link, merged_source_files)
                self.merge_parts(link, merged_source_files)
            if self.merge_everything():
                self.cleanup_merged_sources(robot, merged_source_files)
                # Remove stale "merged" subdirectory from previous runs
                merged_dir = self.config.asset_path("merged")
                if os.path.isdir(merged_dir):
                    shutil.rmtree(merged_dir)
            os.chdir(getcwd)

    def write_link_manifest(self, link: Link, merged_source_files: set[str]):
        """
        Write a per-link manifest listing constituent part instances (with
        documentMicroversion, partId, configuration) so that merged links
        like rail/carriage remain diffable under version control even after
        merge_stls_clean_up deletes the per-part .part files.
        """
        entries = []
        for part in link.parts:
            for part_mesh in part.meshes:
                mesh_path = os.path.abspath(part_mesh.filename)
                part_file = os.path.splitext(mesh_path)[0] + ".part"
                if not os.path.exists(part_file):
                    continue
                try:
                    with open(part_file, "r", encoding="utf-8") as f:
                        entries.append(json.load(f))
                except (OSError, json.JSONDecodeError):
                    continue
        if not entries:
            return
        manifest_path = self.config.asset_path(f"{link.name}.part")
        os.makedirs(os.path.dirname(os.path.abspath(manifest_path)), exist_ok=True)
        with open(manifest_path, "w", encoding="utf-8") as f:
            json.dump(entries, f, indent=4, sort_keys=True)

    def merge_everything(self) -> bool:
        return self.merge_stls != "collision" and self.merge_stls != "visual"

    def _inside_assets_root(self, abs_path: str) -> bool:
        try:
            return os.path.commonpath([abs_path, self._assets_root]) == self._assets_root
        except ValueError:
            return False

    def cleanup_merged_sources(self, robot: Robot, merged_source_files: set[str]):
        remaining = self.collect_mesh_files(robot)
        for filename in sorted(merged_source_files - remaining):
            if not filename.lower().endswith((".stl", ".glb", ".gltf")):
                continue
            abs_path = os.path.abspath(filename)
            if not self._inside_assets_root(abs_path):
                continue
            try:
                os.remove(abs_path)
            except OSError as exc:
                print(warning(f"WARNING: Failed to remove merged mesh {abs_path}: {exc}"))

            part_path = os.path.splitext(abs_path)[0] + ".part"
            if not self._inside_assets_root(part_path):
                continue
            if os.path.exists(part_path):
                try:
                    os.remove(part_path)
                except OSError as exc:
                    print(
                        warning(
                            f"WARNING: Failed to remove merged part file {part_path}: {exc}"
                        )
                    )

    def collect_mesh_files(self, robot: Robot) -> set[str]:
        mesh_files = set()
        for link in robot.links:
            for part in link.parts:
                for part_mesh in part.meshes:
                    mesh_files.add(part_mesh.filename)
        return mesh_files

    def load_mesh(self, mesh_file: str):
        return self.adapter.load(mesh_file)

    def save_mesh(self, mesh_data, mesh_file: str):
        self.adapter.save(mesh_data, mesh_file)

    def transform_mesh(self, mesh_data, matrix: np.ndarray):
        self.adapter.transform(mesh_data, matrix)

    def combine_meshes(self, m1, m2):
        return self.adapter.combine(m1, m2)

    def merge_parts(self, link: Link, merged_source_files: set[str]):
        print(info(f"+ Merging parts for {link.name}"))

        merge_everything = self.merge_everything()

        # Computing the frame where the new part will be located at
        _, com, __ = link.get_dynamics()
        T_world_com = np.eye(4)
        T_world_com[:3, 3] = com

        # Computing a new color, weighting by masses. Only parts that contribute
        # meshes are counted toward total_mass so empty parts don't dilute the
        # resulting alpha/color.
        color = np.zeros(4)
        weighted_mass = 0.0
        mesh_colors = []
        for part in link.parts:
            if not len(part.meshes):
                continue
            meshes_color = np.mean([mesh.color for mesh in part.meshes], axis=0)
            color += meshes_color * part.mass
            mesh_colors.append(meshes_color)
            weighted_mass += part.mass

        if weighted_mass > 0:
            color /= weighted_mass
        elif mesh_colors:
            color = np.mean(mesh_colors, axis=0)
        else:
            color = np.array([0.5, 0.5, 0.5, 1.0])

        # Changing shapes frame
        merged_shapes = []
        for part in link.parts:
            if part.shapes is not None:
                for shape in part.shapes:
                    if merge_everything or shape.is_type(self.merge_stls):
                        # Changing the shape frame
                        T_world_shape = part.T_world_part @ shape.T_part_shape
                        shape.T_part_shape = np.linalg.inv(T_world_com) @ T_world_shape
                        merged_shapes.append(shape)

        # Merging mesh files
        def accumulate_meshes(which: str):
            mesh = None
            for part in link.parts:
                for part_mesh in part.meshes:
                    if part_mesh.is_type(which):
                        merged_source_files.add(part_mesh.filename)
                        if which == "visual":
                            part_mesh.visual = False
                        else:
                            part_mesh.collision = False

                        # Retrieving meshes
                        part_mesh = self.load_mesh(part_mesh.filename)

                        # Expressing meshes in the merged frame
                        T_com_part = np.linalg.inv(T_world_com) @ part.T_world_part
                        self.transform_mesh(part_mesh, T_com_part)

                        if mesh is None:
                            mesh = part_mesh
                        else:
                            mesh = self.combine_meshes(mesh, part_mesh)
            return mesh

        def accumulate_meshes_glb(which: str):
            """
            For GLB: accumulate meshes grouped by material color.
            Geometries sharing the same baseColorFactor are concatenated into
            a single mesh, drastically reducing draw calls while preserving
            per-material coloring.

            Source part stems are tracked per material bucket and encoded in
            the geom name as 'material_{idx}::{src1}+{src2}+...' so that
            downstream logging can show which top-level parts contributed
            to each material.
            """
            import trimesh
            from collections import defaultdict

            # Group meshes by material color; track contributing part stems
            # (ordered, deduped) so the merged geom_name can surface them.
            color_groups = defaultdict(list)
            color_sources: dict = defaultdict(list)

            for part in link.parts:
                for part_mesh in part.meshes:
                    if part_mesh.is_type(which):
                        merged_source_files.add(part_mesh.filename)
                        if which == "visual":
                            part_mesh.visual = False
                        else:
                            part_mesh.collision = False

                        src_stem = os.path.splitext(
                            os.path.basename(part_mesh.filename)
                        )[0]
                        loaded = trimesh.load(part_mesh.filename)
                        T_com_part = np.linalg.inv(T_world_com) @ part.T_world_part

                        if isinstance(loaded, trimesh.Scene):
                            for geom in loaded.geometry.values():
                                geom.apply_transform(T_com_part)
                                key = _glb_material_key(geom)
                                color_groups[key].append(geom)
                                if src_stem not in color_sources[key]:
                                    color_sources[key].append(src_stem)
                        else:
                            loaded.apply_transform(T_com_part)
                            key = _glb_material_key(loaded)
                            color_groups[key].append(loaded)
                            if src_stem not in color_sources[key]:
                                color_sources[key].append(src_stem)

            if not color_groups:
                return None

            # Build scene with one merged mesh per unique material
            scene = trimesh.Scene()
            for idx, (key, geoms) in enumerate(color_groups.items()):
                combined = trimesh.util.concatenate(geoms)
                # Reattach the material from the first geometry in the group
                combined.visual = geoms[0].visual
                sources = color_sources.get(key, [])
                if sources:
                    geom_name = f"material_{idx}::" + "+".join(sources)
                else:
                    geom_name = f"material_{idx}"
                scene.add_geometry(combined, geom_name=geom_name)

            return scene

        merged_meshes = []
        use_glb = self.mesh_ext == ".glb"

        # For GLB visual meshes, use scene-based accumulation to preserve per-part materials.
        # For collision and STL, use flat mesh accumulation (materials don't matter for physics).
        accumulate_visual = accumulate_meshes_glb if use_glb else accumulate_meshes

        if self.merge_stls != "collision" and not self.collisions_as_visual:
            visual_mesh = accumulate_visual("visual")
            if visual_mesh is not None:
                if merge_everything:
                    filename = self.config.asset_path(f"{link.name}_visual{self.mesh_ext}")
                else:
                    os.makedirs(self.config.asset_path("merged"), exist_ok=True)
                    filename = self.config.asset_path(
                        "merged/" + "/" + link.name + f"_visual{self.mesh_ext}"
                    )
                self.save_mesh(visual_mesh, filename)
                merged_meshes.append(
                    Mesh(os.path.relpath(filename, self.config.output_directory), color, visual=True, collision=False)
                )

        if self.merge_stls != "visual":
            # For collision+visual (collisions_as_visual), use scene accumulation for GLB
            accumulate_collision = accumulate_meshes_glb if (use_glb and self.collisions_as_visual) else accumulate_meshes
            collision_mesh = accumulate_collision("collision")
            if collision_mesh is not None:
                if merge_everything:
                    if self.collisions_as_visual:
                        filename = self.config.asset_path(f"{link.name}{self.mesh_ext}")
                    else:
                        filename = self.config.asset_path(f"{link.name}_collision{self.mesh_ext}")
                else:
                    os.makedirs(self.config.asset_path("merged"), exist_ok=True)
                    if self.collisions_as_visual:
                        filename = self.config.asset_path(
                            "merged/" + "/" + link.name + self.mesh_ext
                        )
                    else:
                        filename = self.config.asset_path(
                            "merged/" + "/" + link.name + f"_collision{self.mesh_ext}"
                        )
                self.save_mesh(collision_mesh, filename)
                merged_meshes.append(
                    Mesh(
                        os.path.relpath(filename, self.config.output_directory),
                        color,
                        visual=self.collisions_as_visual,
                        collision=True,
                    )
                )

        mass, com, inertia = link.get_dynamics(T_world_com)
        if merge_everything:
            # Remove all parts
            link.parts = []
        else:
            # We keep the existing parts and add a massless part with merged meshes
            mass = 0
            inertia *= 0

        # Replacing parts with a single one
        link.parts.append(
            Part(
                f"{link.name}_parts",
                T_world_com,
                mass,
                com,
                inertia,
                merged_meshes,
                merged_shapes,
            )
        )
