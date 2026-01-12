import numpy as np
import os
from .config import Config
from .robot import Robot, Link, Part
from .processor import Processor
from .geometry import Mesh
from .message import bright, info, error, warning
from stl import mesh, Mode


class ProcessorMergeParts(Processor):
    """
    This processor merge all parts into a single one, combining the STL
    """

    def __init__(self, config: Config):
        super().__init__(config)
        self.merge_stls = config.get("merge_stls", False)
        self.collisions_as_visual = config.get("collisions_as_visual", False)

    def process(self, robot: Robot):
        if self.merge_stls:
            merged_source_files = set()
            for link in robot.links:
                self.merge_parts(link, merged_source_files)
            if self.merge_everything():
                self.cleanup_merged_sources(robot, merged_source_files)

    def merge_everything(self) -> bool:
        return self.merge_stls != "collision" and self.merge_stls != "visual"

    def cleanup_merged_sources(self, robot: Robot, merged_source_files: set[str]):
        assets_root = os.path.abspath(self.config.asset_path(""))
        remaining = self.collect_mesh_files(robot)
        for filename in sorted(merged_source_files - remaining):
            if not filename.lower().endswith(".stl"):
                continue
            abs_path = os.path.abspath(filename)
            if os.path.commonpath([abs_path, assets_root]) != assets_root:
                continue
            try:
                os.remove(abs_path)
            except OSError as exc:
                print(warning(f"WARNING: Failed to remove merged STL {abs_path}: {exc}"))

            part_path = os.path.splitext(abs_path)[0] + ".part"
            if os.path.commonpath([part_path, assets_root]) != assets_root:
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

    def load_mesh(self, stl_file: str) -> mesh.Mesh:
        return mesh.Mesh.from_file(stl_file)

    def save_mesh(self, mesh: mesh.Mesh, stl_file: str):
        # Tweaking STL header to avoid timestamp
        # This ensures that same process will result in same STL file
        def get_header(name):
            header = "onshape-to-robot"
            return header[:80].ljust(80, " ")

        mesh.get_header = get_header
        mesh.save(stl_file, mode=Mode.BINARY)

    def transform_mesh(self, mesh: mesh.Mesh, matrix: np.ndarray):
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]

        def transform(points):
            return (rotation @ points.T).T + translation

        mesh.v0 = transform(mesh.v0)
        mesh.v1 = transform(mesh.v1)
        mesh.v2 = transform(mesh.v2)
        mesh.normals = transform(mesh.normals)

    def combine_meshes(self, m1: mesh.Mesh, m2: mesh.Mesh):
        return mesh.Mesh(np.concatenate([m1.data, m2.data]))

    def merge_parts(self, link: Link, merged_source_files: set[str]):
        print(info(f"+ Merging parts for {link.name}"))

        merge_everything = self.merge_everything()

        # Computing the frame where the new part will be located at
        _, com, __ = link.get_dynamics()
        T_world_com = np.eye(4)
        T_world_com[:3, 3] = com

        # Computing a new color, weighting by masses
        color = np.zeros(4)
        total_mass = 0
        mesh_colors = []
        for part in link.parts:
            if len(part.meshes):
                meshes_color = np.mean([mesh.color for mesh in part.meshes], axis=0)
                color += meshes_color * part.mass
                mesh_colors.append(meshes_color)
            total_mass += part.mass

        if total_mass > 0:
            color /= total_mass
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

        # Merging STL files
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

        merged_meshes = []

        if self.merge_stls != "collision" and not self.collisions_as_visual:
            visual_mesh = accumulate_meshes("visual")
            if visual_mesh is not None:
                if merge_everything:
                    filename = self.config.asset_path(f"{link.name}_visual.stl")
                else:
                    os.makedirs(self.config.asset_path("merged"), exist_ok=True)
                    filename = self.config.asset_path(
                        "merged/" + "/" + link.name + "_visual.stl"
                    )
                self.save_mesh(visual_mesh, filename)
                merged_meshes.append(
                    Mesh(os.path.relpath(filename, self.config.output_directory), color, visual=True, collision=False)
                )

        if self.merge_stls != "visual":
            collision_mesh = accumulate_meshes("collision")
            if collision_mesh is not None:
                if merge_everything:
                    if self.collisions_as_visual:
                        filename = self.config.asset_path(f"{link.name}.stl")
                    else:
                        filename = self.config.asset_path(f"{link.name}_collision.stl")
                else:
                    os.makedirs(self.config.asset_path("merged"), exist_ok=True)
                    if self.collisions_as_visual:
                        filename = self.config.asset_path(
                            "merged/" + "/" + link.name + ".stl"
                        )
                    else:
                        filename = self.config.asset_path(
                            "merged/" + "/" + link.name + "_collision.stl"
                        )
                self.save_mesh(collision_mesh, filename)
                merged_meshes.append(
                    Mesh(
                        filename,
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
