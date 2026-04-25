"""
Format-specific mesh I/O. Wraps the differences between numpy-stl and trimesh
so processors can stay format-agnostic instead of branching on hasattr().

Two adapters are provided: STLAdapter operates on stl.mesh.Mesh objects;
GLBAdapter operates on trimesh.Trimesh / trimesh.Scene objects.
"""
from __future__ import annotations

import os
from typing import Any

import numpy as np

from .glb_io import export_glb


class STLAdapter:
    extension = ".stl"

    def load(self, filename: str) -> Any:
        from stl import mesh

        return mesh.Mesh.from_file(filename)

    def save(self, mesh_data: Any, filename: str) -> None:
        from stl import Mode

        # Deterministic header so identical runs produce byte-identical STLs.
        def _header(_name: str) -> str:
            return "onshape-to-robot".ljust(80, " ")

        mesh_data.get_header = _header
        mesh_data.save(filename, mode=Mode.BINARY)

        try:
            size_mb = os.path.getsize(filename) / (1024 * 1024)
            face_count = len(mesh_data.vectors)
        except (OSError, AttributeError):
            return
        print(
            f"  [save_stl] {os.path.basename(filename)}: "
            f"{size_mb:.2f} MB, {face_count:,} faces"
        )
        if face_count > 500_000:
            print(
                f"  [save_stl]   WARNING: {face_count:,} faces — consider "
                f"reducing tessellation on this part in Onshape"
            )

    def transform(self, mesh_data: Any, matrix: np.ndarray) -> None:
        rotation = matrix[:3, :3]
        translation = matrix[:3, 3]

        def _apply(points: np.ndarray) -> np.ndarray:
            return (rotation @ points.T).T + translation

        mesh_data.v0 = _apply(mesh_data.v0)
        mesh_data.v1 = _apply(mesh_data.v1)
        mesh_data.v2 = _apply(mesh_data.v2)
        mesh_data.normals = _apply(mesh_data.normals)

    def combine(self, m1: Any, m2: Any) -> Any:
        from stl import mesh

        return mesh.Mesh(np.concatenate([m1.data, m2.data]))


class GLBAdapter:
    extension = ".glb"

    def load(self, filename: str) -> Any:
        import trimesh

        return trimesh.load(filename, force="mesh")

    def save(self, mesh_data: Any, filename: str) -> None:
        export_glb(mesh_data, filename)

    def transform(self, mesh_data: Any, matrix: np.ndarray) -> None:
        mesh_data.apply_transform(matrix)

    def combine(self, m1: Any, m2: Any) -> Any:
        import trimesh

        return trimesh.util.concatenate([m1, m2])


def mesh_adapter_for(mesh_format: str):
    """Return the adapter matching `mesh_format` ("stl" or "glb")."""
    if mesh_format == "glb":
        return GLBAdapter()
    return STLAdapter()
