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


def log_mesh_summary(scene_or_mesh: Any, path: str) -> None:
    """
    Log a one-liner for an output mesh with size, face count, and — for scenes
    — a breakdown of the heaviest geometries. Called explicitly for final
    output files (e.g. merged links) so per-part intermediate exports stay
    quiet; lets users spot which part of an assembly is bloating the output.
    """
    import trimesh

    try:
        size_mb = os.path.getsize(path) / (1024 * 1024)
    except OSError:
        size_mb = float("nan")

    name = os.path.basename(path)

    if isinstance(scene_or_mesh, trimesh.Trimesh):
        print(
            f"+ {name}: {size_mb:.2f} MB, "
            f"{len(scene_or_mesh.faces):,} faces"
        )
        return

    geometries = getattr(scene_or_mesh, "geometry", {}) or {}
    if not geometries:
        print(f"+ {name}: {size_mb:.2f} MB, empty scene")
        return

    ranked = sorted(geometries.items(), key=lambda kv: len(kv[1].faces), reverse=True)
    total_faces = sum(len(g.faces) for _, g in ranked)
    top_line = ", ".join(
        f"{geom_display_name(gname)}={_fmt_count(len(g.faces))}"
        for gname, g in ranked[:3]
    )
    print(
        f"+ {name}: {size_mb:.2f} MB, {len(ranked)} geoms, "
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
            f"    WARNING: {geom_display_name(heaviest_name)} "
            f"is {heaviest_faces:,} faces ({pct:.0f}% of total) — consider "
            f"reducing tessellation on that part in Onshape"
        )


def geom_display_name(name: str, max_sources: int = 5) -> str:
    """
    Format a merged-scene geometry name for human-readable output.

    Merge produces names like ``material_16::schunk_gehaeuse+dp31148_b_mount``;
    this splits on ``::`` and renders as
    ``material_16 [schunk_gehaeuse, dp31148_b_mount]``. If more than
    ``max_sources`` source parts contributed, the remainder is summarized as
    ``+N more``. Names without the ``::`` marker pass through unchanged.
    """
    if "::" not in name:
        return name
    prefix, rest = name.split("::", 1)
    sources = [s for s in rest.split("+") if s]
    if not sources:
        return prefix
    if len(sources) > max_sources:
        shown = ", ".join(sources[:max_sources])
        return f"{prefix} [{shown}, +{len(sources) - max_sources} more]"
    return f"{prefix} [{', '.join(sources)}]"


def _fmt_count(n: int) -> str:
    if n >= 1_000_000:
        return f"{n / 1_000_000:.1f}M"
    if n >= 1_000:
        return f"{n / 1_000:.1f}K"
    return str(n)
