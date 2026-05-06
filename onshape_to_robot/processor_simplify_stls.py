import os
from .config import Config
from .robot import Robot
from .processor import Processor
from .message import bright, info, error
from .glb_io import export_glb, geom_display_name


class ProcessorSimplifySTLs(Processor):
    """
    Allow for mesh simplifications using MeshLab
    """

    def __init__(self, config: Config):
        super().__init__(config)

        # STL merge / simplification
        self.simplify_stls = config.get("simplify_stls", True)
        self.max_stl_size = config.get("max_stl_size", 3)

        # Strategy: "decimate" (default, quadric edge-collapse),
        # "voxel" (voxel remesh — strips internal geometry),
        # "alpha" (alpha-wrap — shrink-wrap outer shell).
        self.simplify_strategy = config.get(
            "simplify_strategy",
            "decimate",
            values_list=["decimate", "voxel", "alpha"],
        )

        # Tunables for "decimate"
        self.decimate_quality_threshold = config.get("decimate_quality_threshold", 0.5)
        self.decimate_preserve_normal = config.get("decimate_preserve_normal", True)
        self.decimate_preserve_topology = config.get(
            "decimate_preserve_topology", False
        )
        self.decimate_preserve_boundary = config.get(
            "decimate_preserve_boundary", False
        )
        self.decimate_planar_quadric = config.get("decimate_planar_quadric", True)

        # Tunables for "voxel". voxel_pitch is in meters; if None we derive it
        # from the mesh's bounding-box diagonal and voxel_resolution (how many
        # voxels across the diagonal).
        self.voxel_pitch = config.get("voxel_pitch", None, required=False)
        self.voxel_resolution = config.get("voxel_resolution", 128)
        # .fill() does expensive flood-fill over a dense voxel grid — cheap
        # for watertight meshes, pathological for open/self-intersecting ones.
        # Disable when OOM or runaway fill times are seen.
        self.voxel_fill = config.get("voxel_fill", True)
        # Re-decimate after remesh to hit the size target
        self.voxel_post_decimate = config.get("voxel_post_decimate", True)

        # Tunables for "alpha". alpha is in meters; if None we derive it from
        # the bounding-box diagonal and alpha_relative.
        self.alpha = config.get("alpha", None, required=False)
        self.alpha_relative = config.get("alpha_relative", 0.02)
        self.alpha_post_decimate = config.get("alpha_post_decimate", True)
        # Pre-decimate to at most this many faces before alpha-wrap. qhull's
        # Delaunay tetrahedralization is the alpha-shape bottleneck and
        # scales poorly past ~10–20K vertices; pre-decimation makes the
        # input tractable. Set to None/0 to disable.
        self.alpha_pre_decimate_faces = config.get(
            "alpha_pre_decimate_faces", 20000
        )

        # Per-geometry targeted simplification. When true, instead of
        # decimating the whole merged GLB proportionally to hit max_stl_size,
        # we walk each geometry and only simplify ones whose face count
        # exceeds simplify_face_threshold. Clean own-CAD parts stay pristine;
        # dense vendor parts get wrapped/decimated individually. Only applies
        # to GLB output.
        self.simplify_per_geometry = config.get("simplify_per_geometry", False)
        self.simplify_face_threshold = config.get(
            "simplify_face_threshold", 50000
        )

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

    def _apply_pymeshlab_decimation(self, mesh_set, reduction: float) -> None:
        """Apply the quadric edge-collapse decimation in-place on a MeshSet."""
        mesh_set.apply_filter(
            "meshing_decimation_quadric_edge_collapse",
            targetperc=reduction,
            qualitythr=self.decimate_quality_threshold,
            preserveboundary=self.decimate_preserve_boundary,
            boundaryweight=1,
            preservenormal=self.decimate_preserve_normal,
            preservetopology=self.decimate_preserve_topology,
            optimalplacement=True,
            planarquadric=self.decimate_planar_quadric,
            qualityweight=False,
            planarweight=0.001,
            autoclean=True,
            selected=False,
        )

    def _reduce_faces_stl(self, filename: str, reduction: float):
        mesh_set = self.pymeshlab.MeshSet()
        mesh_set.load_new_mesh(filename)
        self._apply_pymeshlab_decimation(mesh_set, reduction)
        mesh_set.save_current_mesh(filename)

    def _reduce_faces_glb(self, filename: str, reduction: float):
        """
        Decimate a GLB file while preserving materials/colors. Loads as a
        trimesh Scene, decimates each geometry with pymeshlab (via the numpy
        MeshSet API to avoid disk round-trips), and reattaches the original
        visual data before saving.
        """
        import trimesh

        scene = trimesh.load(filename)

        if isinstance(scene, trimesh.Trimesh):
            original_visual = scene.visual
            scene = self._decimate_mesh(scene, reduction)
            scene.visual = original_visual
            export_glb(scene, filename)
            return

        for name in list(scene.geometry.keys()):
            geom = scene.geometry[name]
            original_visual = geom.visual
            decimated = self._decimate_mesh(geom, reduction)
            decimated.visual = original_visual
            scene.geometry[name] = decimated

        export_glb(scene, filename)

    def _decimate_mesh(self, mesh, reduction: float):
        """Decimate a trimesh.Trimesh using pymeshlab, return a new Trimesh."""
        import trimesh

        ml_mesh = self.pymeshlab.Mesh(
            vertex_matrix=mesh.vertices, face_matrix=mesh.faces
        )
        mesh_set = self.pymeshlab.MeshSet()
        mesh_set.add_mesh(ml_mesh)
        self._apply_pymeshlab_decimation(mesh_set, reduction)
        result = mesh_set.current_mesh()
        return trimesh.Trimesh(
            vertices=result.vertex_matrix(), faces=result.face_matrix()
        )

    # ------------------------------------------------------------------
    # Voxel remesh
    # ------------------------------------------------------------------

    def _voxel_remesh_mesh(self, mesh):
        """
        Voxelize a trimesh, extract the filled surface with marching cubes,
        and return a new watertight Trimesh. Interior geometry is discarded
        because marching cubes only walks the occupancy boundary.
        """
        import trimesh
        import numpy as np
        import time

        pitch = self.voxel_pitch
        if pitch is None:
            diagonal = float(np.linalg.norm(mesh.extents))
            pitch = diagonal / max(self.voxel_resolution, 1)

        print(
            info(
                f"    voxel-remesh input: {len(mesh.vertices)} verts, "
                f"{len(mesh.faces)} faces, pitch={pitch:.4g}"
            )
        )

        t0 = time.monotonic()
        voxelized = mesh.voxelized(pitch=pitch)
        if self.voxel_fill:
            voxelized = voxelized.fill()
        print(
            info(
                f"    voxelize{'+fill' if self.voxel_fill else ''} in "
                f"{time.monotonic() - t0:.1f}s"
            )
        )
        t0 = time.monotonic()
        remeshed = voxelized.marching_cubes
        print(
            info(
                f"    marching_cubes -> "
                f"{0 if remeshed is None else len(remeshed.faces)} faces in "
                f"{time.monotonic() - t0:.1f}s"
            )
        )
        if remeshed is None or len(remeshed.faces) == 0:
            # Fallback: voxelization produced nothing usable
            return mesh
        return trimesh.Trimesh(vertices=remeshed.vertices, faces=remeshed.faces)

    def _voxel_remesh_file(self, filename: str, target_reduction: float):
        """
        Apply voxel remesh to an STL or GLB file in place. If
        voxel_post_decimate is set, follow with quadric decimation to hit the
        size target (voxel remesh usually produces more triangles than needed).
        """
        import trimesh

        if filename.lower().endswith((".glb", ".gltf")):
            scene = trimesh.load(filename)
            if isinstance(scene, trimesh.Trimesh):
                original_visual = scene.visual
                remeshed = self._voxel_remesh_mesh(scene)
                if self.voxel_post_decimate and target_reduction < 1.0:
                    remeshed = self._decimate_mesh(remeshed, target_reduction)
                remeshed.visual = original_visual
                export_glb(remeshed, filename)
                return

            import gc

            for name in list(scene.geometry.keys()):
                geom = scene.geometry[name]
                original_visual = geom.visual
                remeshed = self._voxel_remesh_mesh(geom)
                if self.voxel_post_decimate and target_reduction < 1.0:
                    remeshed = self._decimate_mesh(remeshed, target_reduction)
                remeshed.visual = original_visual
                scene.geometry[name] = remeshed
                del geom, remeshed
                gc.collect()
            export_glb(scene, filename)
            return

        mesh = trimesh.load(filename, force="mesh")
        remeshed = self._voxel_remesh_mesh(mesh)
        if self.voxel_post_decimate and target_reduction < 1.0:
            remeshed = self._decimate_mesh(remeshed, target_reduction)
        remeshed.export(filename)

    # ------------------------------------------------------------------
    # Alpha wrap
    # ------------------------------------------------------------------

    def _alpha_wrap_mesh(self, mesh):
        """
        Shrink-wrap the outer surface of a mesh using MeshLab's alpha-complex
        reconstruction. Interior walls are dropped because only vertices on
        the outer alpha-boundary survive.
        """
        import trimesh
        import numpy as np
        import time

        alpha = self.alpha
        if alpha is None:
            diagonal = float(np.linalg.norm(mesh.extents))
            alpha = diagonal * self.alpha_relative

        # qhull's Delaunay tetrahedralization is the bottleneck — pre-decimate
        # so it operates on a tractable vertex count.
        cap = self.alpha_pre_decimate_faces
        if cap and len(mesh.faces) > cap:
            t_pre = time.monotonic()
            reduction = cap / len(mesh.faces)
            print(
                info(
                    f"    alpha-wrap pre-decimate: "
                    f"{len(mesh.faces)} -> ~{cap} faces "
                    f"(reduction={reduction:.3f})"
                )
            )
            mesh = self._decimate_mesh(mesh, reduction)
            print(
                info(
                    f"    alpha-wrap pre-decimate done: "
                    f"{len(mesh.faces)} faces in {time.monotonic() - t_pre:.1f}s"
                )
            )

        print(
            info(
                f"    alpha-wrap input: {len(mesh.vertices)} verts, "
                f"{len(mesh.faces)} faces, alpha={alpha:.4g}"
            )
        )

        ml_mesh = self.pymeshlab.Mesh(
            vertex_matrix=mesh.vertices, face_matrix=mesh.faces
        )
        mesh_set = self.pymeshlab.MeshSet()
        mesh_set.add_mesh(ml_mesh)

        t0 = time.monotonic()
        used = None
        try:
            mesh_set.apply_filter(
                "generate_alpha_wrap",
                alpha_fraction=self.alpha_relative,
            )
            used = "generate_alpha_wrap"
        except Exception:
            try:
                # filtering=1 returns the alpha-shape boundary surface.
                # filtering=0 returns the whole alpha complex (interior
                # simplices included), which densifies instead of wrapping.
                mesh_set.apply_filter(
                    "generate_alpha_shape",
                    alpha=self.pymeshlab.PercentageValue(
                        self.alpha_relative * 100.0
                    ),
                    filtering=1,
                )
                used = "generate_alpha_shape"
            except Exception as exc:
                print(
                    error(
                        f"alpha-wrap failed ({exc}); leaving mesh untouched"
                    )
                )
                return mesh

        result = mesh_set.current_mesh()
        out_faces = result.face_number()
        in_faces = len(mesh.faces)
        print(
            info(
                f"    {used} -> {out_faces} faces in "
                f"{time.monotonic() - t0:.1f}s"
            )
        )
        if out_faces == 0:
            return mesh
        # Sanity check: if the "wrap" produced more faces than the input,
        # something is misconfigured (usually alpha_relative too large or
        # the wrong filter mode). Bail back to the original mesh so the
        # post-decimate pass works on something tractable.
        if out_faces > 1.5 * in_faces:
            print(
                error(
                    f"    {used} densified ({in_faces} -> {out_faces} faces); "
                    f"alpha_relative={self.alpha_relative} is likely too "
                    f"large. Falling back to input mesh."
                )
            )
            return mesh
        return trimesh.Trimesh(
            vertices=result.vertex_matrix(), faces=result.face_matrix()
        )

    def _alpha_wrap_file(self, filename: str, target_reduction: float):
        import trimesh

        if filename.lower().endswith((".glb", ".gltf")):
            scene = trimesh.load(filename)
            if isinstance(scene, trimesh.Trimesh):
                original_visual = scene.visual
                wrapped = self._alpha_wrap_mesh(scene)
                if self.alpha_post_decimate and target_reduction < 1.0:
                    wrapped = self._decimate_mesh(wrapped, target_reduction)
                wrapped.visual = original_visual
                export_glb(wrapped, filename)
                return

            for name in list(scene.geometry.keys()):
                geom = scene.geometry[name]
                original_visual = geom.visual
                wrapped = self._alpha_wrap_mesh(geom)
                if self.alpha_post_decimate and target_reduction < 1.0:
                    wrapped = self._decimate_mesh(wrapped, target_reduction)
                wrapped.visual = original_visual
                scene.geometry[name] = wrapped
            export_glb(scene, filename)
            return

        mesh = trimesh.load(filename, force="mesh")
        wrapped = self._alpha_wrap_mesh(mesh)
        if self.alpha_post_decimate and target_reduction < 1.0:
            wrapped = self._decimate_mesh(wrapped, target_reduction)
        wrapped.export(filename)

    # ------------------------------------------------------------------
    # Per-geometry selective simplification
    # ------------------------------------------------------------------

    def _simplify_geometry(self, geom):
        """
        Simplify a single trimesh.Trimesh down to ~simplify_face_threshold
        faces using the configured strategy. Returns a new Trimesh.
        """
        import trimesh

        target_faces = self.simplify_face_threshold
        if len(geom.faces) <= target_faces:
            return geom

        reduction = target_faces / len(geom.faces)

        if self.simplify_strategy == "voxel":
            out = self._voxel_remesh_mesh(geom)
            if self.voxel_post_decimate and len(out.faces) > target_faces:
                out = self._decimate_mesh(out, target_faces / len(out.faces))
            return out

        if self.simplify_strategy == "alpha":
            out = self._alpha_wrap_mesh(geom)
            if self.alpha_post_decimate and len(out.faces) > target_faces:
                out = self._decimate_mesh(out, target_faces / len(out.faces))
            return out

        return self._decimate_mesh(geom, reduction)

    def _simplify_per_geometry_glb(self, filename: str):
        """
        Load a GLB, simplify only the geometries whose face count exceeds
        simplify_face_threshold, leave everything else untouched, save back.
        """
        import gc
        import trimesh

        scene = trimesh.load(filename)

        if isinstance(scene, trimesh.Trimesh):
            if len(scene.faces) <= self.simplify_face_threshold:
                print(
                    info(
                        f"  single mesh {len(scene.faces)} faces — "
                        f"under threshold, skipping"
                    )
                )
                return
            original_visual = scene.visual
            simplified = self._simplify_geometry(scene)
            simplified.visual = original_visual
            export_glb(simplified, filename)
            return

        heavy = [
            name
            for name, g in scene.geometry.items()
            if len(g.faces) > self.simplify_face_threshold
        ]
        heavy_display = (
            ", ".join(geom_display_name(n) for n in heavy) if heavy else "(none)"
        )
        print(
            info(
                f"  {len(heavy)}/{len(scene.geometry)} geometries exceed "
                f"threshold: {heavy_display}"
            )
        )

        for name in heavy:
            geom = scene.geometry[name]
            original_visual = geom.visual
            before = len(geom.faces)
            simplified = self._simplify_geometry(geom)
            simplified.visual = original_visual
            scene.geometry[name] = simplified
            print(
                info(
                    f"    {geom_display_name(name)}: "
                    f"{before} -> {len(simplified.faces)} faces"
                )
            )
            del geom, simplified
            gc.collect()

        export_glb(scene, filename)

    # ------------------------------------------------------------------

    def simplify_mesh(self, filename: str):
        import time

        size_M = os.path.getsize(filename) / (1024 * 1024)

        is_glb = filename.lower().endswith((".glb", ".gltf"))

        # Per-geometry mode: decision is based on face counts, not file size,
        # so it runs regardless of max_stl_size. Only makes sense for GLB.
        if self.simplify_per_geometry and is_glb:
            start = time.monotonic()
            print(
                info(
                    f"+ {os.path.basename(filename)} running per-geometry "
                    f"simplification ({self.simplify_strategy}, threshold="
                    f"{self.simplify_face_threshold} faces)"
                )
            )
            self._simplify_per_geometry_glb(filename)
            elapsed = time.monotonic() - start
            new_size_M = os.path.getsize(filename) / (1024 * 1024)
            print(
                info(
                    f"  -> {new_size_M:.2f} M in {elapsed:.1f}s"
                )
            )
            return

        if size_M <= self.max_stl_size:
            return

        target_reduction = self.max_stl_size / size_M
        print(
            info(
                f"+ {os.path.basename(filename)} is {size_M:.2f} M, running "
                f"mesh simplification ({self.simplify_strategy}, "
                f"target_reduction={target_reduction:.3f})"
            )
        )

        start = time.monotonic()
        if self.simplify_strategy == "voxel":
            self._voxel_remesh_file(filename, target_reduction)
        elif self.simplify_strategy == "alpha":
            self._alpha_wrap_file(filename, target_reduction)
        else:
            self.reduce_faces(filename, target_reduction)
        elapsed = time.monotonic() - start
        new_size_M = os.path.getsize(filename) / (1024 * 1024)
        print(
            info(
                f"  -> {new_size_M:.2f} M in {elapsed:.1f}s"
            )
        )
