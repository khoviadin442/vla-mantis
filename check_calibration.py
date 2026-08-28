#!/usr/bin/env python3
"""Quantify how well the two cameras agree once their clouds are in the robot base frame.

Runs on the PLYs written by make_pointclouds.py and reports, per frame:
  * the table plane each camera sees on its own (RANSAC): height above the base z = 0 plane and
    tilt away from horizontal - both should be near zero if the extrinsics are right,
  * the distance from each left point to the nearest right point over the region the two
    cameras both see, which is the direct left-vs-right calibration error.
"""
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

PCD = Path("/home/ros/share/pointclouds")
TABLE_BAND = (-0.10, 0.25)
OVERLAP_BAND = (-0.05, 0.60)


def table_plane(pc):
    """(height at origin, tilt in degrees, inlier count) of the dominant plane near z = 0."""
    pts = np.asarray(pc.points)
    band = pts[(pts[:, 2] > TABLE_BAND[0]) & (pts[:, 2] < TABLE_BAND[1])]
    if len(band) < 500:
        return float("nan"), float("nan"), 0
    sub = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(band))
    model, inliers = sub.segment_plane(distance_threshold=0.006, ransac_n=3, num_iterations=400)
    a, b, c, d = model
    n = np.array([a, b, c], float)
    if n[2] < 0:
        n, d = -n, -d
    tilt = float(np.degrees(np.arccos(np.clip(n[2] / np.linalg.norm(n), -1, 1))))
    height = float(-d / n[2])
    return height, tilt, len(inliers)


def overlap_distance(left, right):
    """(n_compared, p50, p95) mm from left points to the nearest right point, where they overlap."""
    l = np.asarray(left.points)
    r = np.asarray(right.points)
    lo, hi = OVERLAP_BAND
    l = l[(l[:, 2] > lo) & (l[:, 2] < hi)]
    r = r[(r[:, 2] > lo) & (r[:, 2] < hi)]
    if len(l) < 100 or len(r) < 100:
        return 0, float("nan"), float("nan")
    rmin, rmax = r.min(0), r.max(0)
    inside = np.all((l > rmin) & (l < rmax), axis=1)
    l = l[inside]
    if len(l) < 100:
        return 0, float("nan"), float("nan")
    if len(l) > 60000:
        l = l[np.random.default_rng(0).choice(len(l), 60000, replace=False)]
    a = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(l))
    b = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(r))
    d = np.asarray(a.compute_point_cloud_distance(b)) * 1000.0
    return len(d), float(np.percentile(d, 50)), float(np.percentile(d, 95))


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "stack_three_cups"
    frames = sorted({int(p.stem.split("_f")[1].split("_")[0])
                     for p in PCD.glob(f"{name}_f*_merged.ply")})
    if not frames:
        print(f"no clouds for {name}; run make_pointclouds.py first")
        return 1
    print(f"{name}: checking {len(frames)} frames\n")
    print("        table plane per camera                 left-vs-right overlap")
    print(" frame  left h/tilt        right h/tilt        n        p50      p95")
    heights, p50s = [], []
    for f in frames:
        left = o3d.io.read_point_cloud(str(PCD / f"{name}_f{f}_left.ply"))
        right = o3d.io.read_point_cloud(str(PCD / f"{name}_f{f}_right.ply"))
        lh, lt, _ = table_plane(left)
        rh, rt, _ = table_plane(right)
        n, p50, p95 = overlap_distance(left, right)
        heights += [lh, rh]
        p50s.append(p50)
        print(f" {f:5d}  {lh*1000:+6.1f}mm {lt:4.1f}deg   {rh*1000:+6.1f}mm {rt:4.1f}deg   "
              f"{n:6d}  {p50:6.1f}mm {p95:6.1f}mm")
    h = np.array([x for x in heights if np.isfinite(x)])
    p = np.array([x for x in p50s if np.isfinite(x)])
    print(f"\n table height above base z=0 : mean {h.mean()*1000:+.1f} mm, spread {h.std()*1000:.1f} mm")
    print(f" left-vs-right median distance: {p.mean():.1f} mm")
    verdict = ("extrinsics look consistent" if abs(h.mean()) < 0.05 and p.mean() < 30
               else "CHECK THE CALIBRATION - cameras disagree or the table is not at z=0")
    print(f" verdict: {verdict}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
