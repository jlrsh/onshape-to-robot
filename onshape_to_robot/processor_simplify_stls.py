import os
from .config import Config
from .robot import Robot
from .processor import Processor
from .message import bright, info, error


class ProcessorSimplifySTLs(Processor):
    """
    Allow for mesh simplifications using MeshLab
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # STL merge / simplification
        self.simplify_stls = config.get("simplify_stls", True)
        self.max_stl_size = config.get("max_stl_size", 3)

        if self.simplify_stls:
            self.pymeshlab = self.check_meshlab()

    def check_meshlab(self):
        print(bright("* Checking pymeshlab presence..."))
        try:
            import pymeshlab

            return pymeshlab
        except ImportError:
            self.simplify_stls = False
            print(error("No pymeshlab, disabling STL simplification support"))
            print(info("TIP: consider installing pymeshlab:"))
            print(info("pip install pymeshlab"))

    def process(self, robot: Robot):
        if self.simplify_stls:
            simplify_all = (
                self.simplify_stls != "visual" and self.simplify_stls != "collision"
            )
            simplified = set()
            getcwd = os.getcwd()
            # Changing directory to output to have relative paths working
            os.chdir(self.config.output_directory)
            for link in robot.links:
                for part in link.parts:
                    for mesh in part.meshes:
                        if (
                            simplify_all or mesh.is_type(self.simplify_stls)
                        ) and mesh.filename not in simplified:
                            simplified.add(mesh.filename)
                            self.simplify_mesh(mesh.filename)
            os.chdir(getcwd)

    def reduce_faces(self, filename: str, reduction: float = 0.9):
        if filename.lower().endswith((".glb", ".gltf")):
            self._reduce_faces_glb(filename, reduction)
        else:
            self._reduce_faces_stl(filename, reduction)

    def _reduce_faces_stl(self, filename: str, reduction: float):
        mesh_set = self.pymeshlab.MeshSet()
        mesh_set.load_new_mesh(filename)
        mesh_set.apply_filter(
            "meshing_decimation_quadric_edge_collapse",
            targetperc=reduction,
            qualitythr=0.5,
            preserveboundary=False,
            boundaryweight=1,
            preservenormal=True,
            preservetopology=False,
            optimalplacement=True,
            planarquadric=True,
            qualityweight=False,
            planarweight=0.001,
            autoclean=True,
            selected=False,
        )
        mesh_set.save_current_mesh(filename)

    def _reduce_faces_glb(self, filename: str, reduction: float):
        """
        Decimate a GLB file while preserving materials/colors.
        Loads as a trimesh Scene, decimates each geometry with pymeshlab,
        then reattaches the original visual data before saving.
        """
        import trimesh
        import numpy as np

        scene = trimesh.load(filename)

        if isinstance(scene, trimesh.Trimesh):
            # Single mesh, not a scene
            original_visual = scene.visual
            scene = self._decimate_mesh(scene, reduction)
            scene.visual = original_visual
            scene.export(filename, file_type="glb")
            return

        # Scene with potentially multiple geometries
        for name in list(scene.geometry.keys()):
            geom = scene.geometry[name]
            original_visual = geom.visual
            decimated = self._decimate_mesh(geom, reduction)
            decimated.visual = original_visual
            scene.geometry[name] = decimated

        scene.export(filename, file_type="glb")

    def _decimate_mesh(self, mesh, reduction: float):
        """Decimate a trimesh.Trimesh using pymeshlab, return new Trimesh."""
        import trimesh
        import tempfile

        # Write to a temp STL for pymeshlab (it can't save GLB)
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
            tmp_path = tmp.name
            mesh.export(tmp_path)

        try:
            mesh_set = self.pymeshlab.MeshSet()
            mesh_set.load_new_mesh(tmp_path)
            mesh_set.apply_filter(
                "meshing_decimation_quadric_edge_collapse",
                targetperc=reduction,
                qualitythr=0.5,
                preserveboundary=False,
                boundaryweight=1,
                preservenormal=True,
                preservetopology=False,
                optimalplacement=True,
                planarquadric=True,
                qualityweight=False,
                planarweight=0.001,
                autoclean=True,
                selected=False,
            )
            m = mesh_set.current_mesh()
            return trimesh.Trimesh(
                vertices=m.vertex_matrix(), faces=m.face_matrix()
            )
        finally:
            os.unlink(tmp_path)

    def simplify_mesh(self, filename: str):
        size_M = os.path.getsize(filename) / (1024 * 1024)

        if size_M > self.max_stl_size:
            print(
                info(
                    f"+ {os.path.basename(filename)} is {size_M:.2f} M, running mesh simplification"
                )
            )
            self.reduce_faces(filename, self.max_stl_size / size_M)
