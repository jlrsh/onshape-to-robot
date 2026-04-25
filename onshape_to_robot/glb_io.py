"""
Single source of truth for GLB read/write. Importing trimesh is kept inside
functions because it is only needed when mesh_format=glb is selected.
"""
from __future__ import annotations

import os
from typing import Any


def load_glb(path: str) -> Any:
    """Load a GLB file and return the resulting trimesh Scene or Trimesh."""
    import trimesh

    return trimesh.load(path)


def export_glb(scene_or_mesh: Any, path: str) -> None:
    """
    Export a trimesh Scene or Trimesh to GLB at `path`.

    include_normals=True forces the NORMAL attribute into the output. trimesh
    would otherwise skip NORMAL when normals aren't explicitly attached, which
    leads to flat shading in downstream viewers (e.g. rviz, MuJoCo).
    """
    scene_or_mesh.export(path, file_type="glb", include_normals=True)
    _print_export_summary(scene_or_mesh, path)


def _print_export_summary(scene_or_mesh: Any, path: str) -> None:
    """
    Log a one-liner per exported GLB with size, face count, and — for scenes
    — a breakdown of the heaviest geometries. Lets users spot which part of
    an assembly is bloating the output before it becomes an OOM problem
    downstream in simplification.
    """
    import trimesh

    try:
        size_mb = os.path.getsize(path) / (1024 * 1024)
    except OSError:
        size_mb = float("nan")

    name = os.path.basename(path)

    if isinstance(scene_or_mesh, trimesh.Trimesh):
        print(
            f"  [export_glb] {name}: {size_mb:.2f} MB, "
            f"{len(scene_or_mesh.faces):,} faces"
        )
        return

    geometries = getattr(scene_or_mesh, "geometry", {}) or {}
    if not geometries:
        print(f"  [export_glb] {name}: {size_mb:.2f} MB, empty scene")
        return

    ranked = sorted(
        geometries.items(), key=lambda kv: len(kv[1].faces), reverse=True
    )
    total_faces = sum(len(g.faces) for _, g in ranked)
    top_line = ", ".join(
        f"{gname}={_fmt_count(len(g.faces))}" for gname, g in ranked[:3]
    )
    print(
        f"  [export_glb] {name}: {size_mb:.2f} MB, {len(ranked)} geoms, "
        f"{total_faces:,} faces; heaviest: {top_line}"
    )

    # Flag any single geometry that dominates (>50% of faces) or is egregiously
    # large in absolute terms — those are prime upstream-cleanup candidates.
    heaviest_name, heaviest_geom = ranked[0]
    heaviest_faces = len(heaviest_geom.faces)
    if total_faces > 0 and (
        heaviest_faces / total_faces > 0.5 or heaviest_faces > 500_000
    ):
        pct = 100.0 * heaviest_faces / total_faces
        print(
            f"  [export_glb]   WARNING: '{heaviest_name}' is {heaviest_faces:,} "
            f"faces ({pct:.0f}% of total) — consider reducing tessellation "
            f"on that part in Onshape"
        )


def _fmt_count(n: int) -> str:
    if n >= 1_000_000:
        return f"{n / 1_000_000:.1f}M"
    if n >= 1_000:
        return f"{n / 1_000:.1f}K"
    return str(n)
