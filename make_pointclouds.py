#!/usr/bin/env python3
"""Build coloured point clouds from a recorded dataset to check the camera calibration.

Unprojects each camera's depth with the intrinsics from meta/intrinsics.json, moves the points
into the robot base frame with the pose from meta/extrinsics.json, and writes one PLY per frame
plus per-camera PLYs. If the calibration is right the two cameras' clouds land on the same
table plane, which the printed agreement check quantifies.

The pose in extrinsics.json places `<cam>_camera_calibration_pose`; the fixed hardware chain
from there to `<cam>_camera_color_optical_frame` is read out of the URDF, so the optical-frame
convention comes from the robot model rather than being reimplemented here.
"""
import json
import os
import re
import sys
from pathlib import Path

os.environ.setdefault("HF_HUB_OFFLINE", "1")
import numpy as np

SHARE = Path("/home/ros/share")
URDF = SHARE / "mantis_ws" / "mantis.urdf"
OUT = SHARE / "pointclouds"
BASE_LINK = "prl_ur5_base"
STRIDE = 2
Z_MIN, Z_MAX = 0.05, 3.0


def rpy_to_R(r, p, y):
    """URDF/ROS fixed-axis roll-pitch-yaw -> rotation matrix (Rz @ Ry @ Rx)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def T_of(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def urdf_fixed_joints(path):
    """{child link -> (parent link, 4x4 transform)} for every fixed joint in the URDF."""
    text = path.read_text()
    out = {}
    pattern = re.compile(
        r'<joint name="[^"]*" type="fixed">\s*<origin([^/]*)/>\s*'
        r'<parent link="([^"]*)"\s*/>\s*<child link="([^"]*)"', re.S)
    for origin, parent, child in pattern.findall(text):
        rpy = re.search(r'rpy="([^"]*)"', origin)
        xyz = re.search(r'xyz="([^"]*)"', origin)
        rpy = [float(v) for v in rpy.group(1).split()] if rpy else [0.0, 0.0, 0.0]
        xyz = [float(v) for v in xyz.group(1).split()] if xyz else [0.0, 0.0, 0.0]
        out[child] = (parent, T_of(xyz, rpy))
    return out


def chain_transform(joints, frm, to):
    """Transform frm->to by walking `to` up the fixed-joint tree until `frm` is reached."""
    T = np.eye(4)
    link = to
    while link != frm:
        if link not in joints:
            raise KeyError(f"{to} is not connected to {frm} through fixed joints (stuck at {link})")
        parent, Tp = joints[link]
        T = Tp @ T
        link = parent
    return T


def to_numpy(x):
    return x.numpy() if hasattr(x, "numpy") else np.asarray(x)


def unproject(depth_m, K, stride=STRIDE):
    """(N,3) points in the camera optical frame from a metric depth image."""
    h, w = depth_m.shape
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    z = depth_m[::stride, ::stride]
    ok = np.isfinite(z) & (z > Z_MIN) & (z < Z_MAX)
    us, vs, z = us[ok], vs[ok], z[ok]
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    return np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z], axis=1), ok, (h, w)


def write_ply(path, xyz, rgb):
    """Binary PLY with per-point colour."""
    n = len(xyz)
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n").encode()
    data = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                              ("r", "u1"), ("g", "u1"), ("b", "u1")])
    data["x"], data["y"], data["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    data["r"], data["g"], data["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, "wb") as f:
        f.write(header)
        f.write(data.tobytes())


def plane_z(xyz):
    """Height of the dominant horizontal surface: the mode of z over a 1 cm histogram."""
    if len(xyz) < 100:
        return float("nan")
    hist, edges = np.histogram(xyz[:, 2], bins=np.arange(-0.5, 1.5, 0.01))
    i = int(np.argmax(hist))
    return float(0.5 * (edges[i] + edges[i + 1]))


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "stack_three_cups"
    n_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    root = SHARE / "lerobot_data" / name
    intr = json.loads((root / "meta" / "intrinsics.json").read_text())["cameras"]
    extr = json.loads((root / "meta" / "extrinsics.json").read_text())["cameras"]
    joints = urdf_fixed_joints(URDF)

    T_base_opt = {}
    for cam, e in extr.items():
        prefix = e["camera"]
        pose = e["pose"]
        T_calib = T_of([pose["x"], pose["y"], pose["z"]],
                       [pose["roll"], pose["pitch"], pose["yaw"]])
        T_fixed = chain_transform(joints, f"{prefix}_camera_calibration_pose",
                                  f"{prefix}_camera_color_optical_frame")
        T_base_opt[cam] = T_calib @ T_fixed
        t = T_base_opt[cam][:3, 3]
        print(f"  {cam:5s} ({prefix}): optical frame at base xyz = "
              f"[{t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f}]")

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset(f"khoviadin/{name}", root=root)
    idx = np.linspace(0, ds.num_frames - 1, n_frames, dtype=int)
    OUT.mkdir(parents=True, exist_ok=True)
    print(f"\ndataset {name}: {ds.num_frames} frames, exporting {list(idx)}")

    for f_i, i in enumerate(idx):
        s = ds[int(i)]
        merged_xyz, merged_rgb, per_cam = [], [], {}
        for cam in intr:
            K = intr[cam]["K"]
            depth = to_numpy(s[f"observation.images.{cam}_depth"])[0] / 1000.0
            color = to_numpy(s[f"observation.images.{cam}"])
            pts, ok, _ = unproject(depth, K)
            rgb = (np.transpose(color, (1, 2, 0))[::STRIDE, ::STRIDE][ok] * 255).astype(np.uint8)
            T = T_base_opt[cam]
            pts_base = pts @ T[:3, :3].T + T[:3, 3]
            per_cam[cam] = pts_base
            merged_xyz.append(pts_base)
            merged_rgb.append(rgb)
            write_ply(OUT / f"{name}_f{f_i}_{cam}.ply", pts_base, rgb)
        xyz = np.concatenate(merged_xyz)
        rgb = np.concatenate(merged_rgb)
        write_ply(OUT / f"{name}_f{f_i}_merged.ply", xyz, rgb)
        zs = {c: plane_z(p) for c, p in per_cam.items()}
        agree = abs(zs["left"] - zs["right"]) * 1000.0
        print(f"  frame {i:5d}: {len(xyz):7d} pts | dominant plane z: "
              + " ".join(f"{c}={v:+.3f}m" for c, v in zs.items())
              + f" | left-right agreement {agree:.0f} mm")
    print(f"\nwrote {len(list(OUT.glob('*.ply')))} PLY files to {OUT}")


if __name__ == "__main__":
    main()
