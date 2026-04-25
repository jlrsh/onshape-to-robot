"""
Load a GLB scene and highlight the heaviest geometries in viser so you can
see which physical part of the assembly is dominating the file size.

Usage:
    python scripts/viz_biggest_mesh.py path/to/file.glb [--top N]

Geometries are drawn in their original world transform. The top-N by face
count are rendered in bright red; everything else is drawn translucent grey
for context. A sidebar GUI lists the ranking with face/vertex counts and
lets you toggle individual geometries.
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import trimesh
import viser


def list_geometries(scene: trimesh.Scene):
    """Return [(node_name, geometry_name, geom, world_transform), ...]."""
    entries = []
    if isinstance(scene, trimesh.Trimesh):
        entries.append(("root", "root", scene, np.eye(4)))
        return entries

    graph = scene.graph
    for node_name in graph.nodes_geometry:
        transform, geom_name = graph.get(node_name)
        if geom_name is None or geom_name not in scene.geometry:
            continue
        geom = scene.geometry[geom_name]
        entries.append((node_name, geom_name, geom, np.asarray(transform)))
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="Path to GLB/GLTF file")
    parser.add_argument(
        "--top", type=int, default=1,
        help="How many of the biggest geometries to highlight (default 1)",
    )
    parser.add_argument(
        "--port", type=int, default=8080, help="Viser port (default 8080)"
    )
    args = parser.parse_args()

    print(f"Loading {args.path}...")
    scene = trimesh.load(args.path)
    entries = list_geometries(scene)
    if not entries:
        print("No geometries found.", file=sys.stderr)
        sys.exit(1)

    ranked = sorted(entries, key=lambda e: len(e[2].faces), reverse=True)
    top_names = {(e[0], e[1]) for e in ranked[: args.top]}

    total_faces = sum(len(e[2].faces) for e in ranked)
    print(f"\n{len(ranked)} geometries, {total_faces:,} total faces\n")
    print(f"{'rank':>4}  {'faces':>10}  {'verts':>10}  name")
    for i, (node_name, geom_name, geom, _) in enumerate(ranked[:20]):
        marker = "*" if (node_name, geom_name) in top_names else " "
        print(
            f"{i + 1:>4}{marker} {len(geom.faces):>10,}  "
            f"{len(geom.vertices):>10,}  {node_name} / {geom_name}"
        )
    if len(ranked) > 20:
        print(f"     ... ({len(ranked) - 20} more)")

    print(f"\nStarting viser on http://localhost:{args.port} ...")
    server = viser.ViserServer(port=args.port)

    with server.gui.add_folder("Ranked geometries"):
        for i, (node_name, geom_name, geom, _) in enumerate(ranked):
            is_top = (node_name, geom_name) in top_names
            label = (
                f"{'★ ' if is_top else '  '}"
                f"#{i + 1} {geom_name} "
                f"({len(geom.faces):,}f)"
            )
            server.gui.add_text(label, initial_value="", disabled=True)

    for i, (node_name, geom_name, geom, transform) in enumerate(ranked):
        is_top = (node_name, geom_name) in top_names
        color = (240, 40, 40) if is_top else (160, 160, 160)
        opacity = 1.0 if is_top else 0.25

        translation = np.array(transform[:3, 3], copy=True)
        wxyz = trimesh.transformations.quaternion_from_matrix(
            np.array(transform, copy=True)
        )

        server.scene.add_mesh_simple(
            name=f"/geom/{i:03d}_{geom_name}",
            vertices=np.asarray(geom.vertices),
            faces=np.asarray(geom.faces),
            color=color,
            opacity=opacity,
            wxyz=wxyz,
            position=translation,
        )

    print("Viser running. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
