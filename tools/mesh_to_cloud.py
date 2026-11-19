#!/usr/bin/env python3
"""Sample a point cloud from a triangle mesh with Open3D."""

from __future__ import annotations

import argparse
from pathlib import Path

import open3d as o3d


def mesh_to_cloud(
    mesh: o3d.geometry.TriangleMesh,
    number_of_points: int = 100_000,
    method: str = "uniform",
) -> o3d.geometry.PointCloud:
    """Sample ``number_of_points`` from ``mesh``.

    ``method`` is ``uniform`` (area-weighted) or ``poisson`` (even spacing).
    When the mesh has vertex colors, they are interpolated onto the samples.
    """
    if number_of_points < 1:
        raise ValueError("number_of_points must be positive")
    if not mesh.has_triangles():
        raise ValueError("mesh has no triangles")

    method = method.lower()
    if method not in ("uniform", "poisson"):
        raise ValueError("method must be 'uniform' or 'poisson'")

    if mesh.has_vertex_colors():
        return _sample_colored(mesh, number_of_points, method)
    if method == "uniform":
        return mesh.sample_points_uniformly(number_of_points=number_of_points)
    return mesh.sample_points_poisson_disk(number_of_points=number_of_points)


def _sample_colored(mesh, number_of_points, method):
    # The tensor sampler barycentric-interpolates vertex colors. The legacy
    # sampler returns positions only.
    tensor_mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    if method == "uniform":
        cloud = tensor_mesh.sample_points_uniformly(number_of_points)
    else:
        cloud = tensor_mesh.sample_points_poisson_disk(number_of_points)
    return cloud.to_legacy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mesh", type=Path, help="triangle mesh (.ply, .obj, .stl)")
    parser.add_argument("output", type=Path, help="point cloud (.pcd, .ply)")
    parser.add_argument("--points", type=int, default=100_000)
    parser.add_argument("--method", choices=("uniform", "poisson"), default="uniform")
    args = parser.parse_args()

    mesh = o3d.io.read_triangle_mesh(str(args.mesh))
    cloud = mesh_to_cloud(mesh, args.points, args.method)
    if not o3d.io.write_point_cloud(str(args.output), cloud):
        raise SystemExit(f"failed to write {args.output}")
    print(f"{args.output}: {len(cloud.points)} points")


if __name__ == "__main__":
    main()
