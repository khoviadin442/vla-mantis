# Camera calibration — checking it, and fixing datasets that predate it

> Part of **vla-mantis**. Recording itself is in [teleop.md](teleop.md#recording-a-dataset);
> the pipeline as a whole is in [the top-level README](../README.md).

A policy trained on these datasets learns from pixels *and* from where those pixels are in
space. If the camera extrinsics are wrong, every episode is consistently wrong in the same
way, and nothing downstream will tell you — the loss curve looks fine. So the calibration is
worth checking directly, before you spend GPU hours on it.

The check does not need a calibration target. It uses the fact that **two cameras looking at
one table must agree**: unproject both depth images into the robot base frame and, if the
extrinsics are right, both clouds land on the same plane at `z = 0`, on top of each other.

## What the recorder stores

Every dataset written by `lerobot_recorder.py` carries the calibration it was recorded with:

| File | Source |
|---|---|
| `meta/intrinsics.json` | latched from the live `camera_info` topics — fx, fy, cx, cy per stream |
| `meta/extrinsics.json` | the fixed-cameras calibration file, one pose per camera |

The calibration file is
`mantis_ws/src/prl_ur5_robot_configuration/config/fixed_cameras/dataset_collection.yaml`,
shipped in `patches/` and applied by `setup_workstation.sh`. **It holds this bench's measured
camera poses.** If you move a camera, or set the stack up on a different bench, recalibrate
and update that file — otherwise every dataset you record is confidently, consistently wrong,
and the check below is what tells you.

Depth is registered to its colour frame, so it shares that camera's intrinsics and pose.
This is what makes the check below possible after the fact, off a recorded dataset, with the
robot switched off.

## Prerequisite: open3d

`check_calibration.py` and `view_pointclouds.py` need **open3d**, which the container image
does **not** ship. `make_pointclouds.py` does not — it writes PLY by hand — so you can build
the clouds without it and only hit this at the analysis step.

Install it into a vendored prefix in the share dir, once:

```bash
docker exec -it mantis bash          # enter the container
  python3 -m pip install --target ~/share/o3d_libs open3d==0.19.0
```

`--target` rather than a normal install because it is large (~1.7 GB with dependencies) and
belongs in the bind-mounted share dir, where it survives the `--rm` container instead of
being rebuilt every time. `view_pointclouds.py` falls back to that prefix automatically:

```python
try:
    import open3d as o3d
except ModuleNotFoundError:
    sys.path.insert(0, str(SHARE / "o3d_libs"))
    import open3d as o3d
```

`check_calibration.py` imports it plainly, with no such fallback, so it needs open3d on the
normal import path — run it from a shell where `~/share/o3d_libs` is on `PYTHONPATH`:

```bash
PYTHONPATH=~/share/o3d_libs python3 ~/share/check_calibration.py <dataset-name>
```

## The check

```bash
docker exec -it mantis bash          # enter the container

# then, inside it — on a recorded dataset
  python3 ~/share/make_pointclouds.py  <dataset-name> [n_frames]   # default 4 frames
  PYTHONPATH=~/share/o3d_libs python3 ~/share/check_calibration.py <dataset-name>
```

Everything on this page runs inside the container, from that shell.

`make_pointclouds.py` unprojects each camera's depth with its intrinsics, moves the points
into the robot base frame with its pose, and writes one PLY per frame plus per-camera PLYs
under `~/share/pointclouds/`. The hardware chain from the calibration pose to the optical
frame is read out of the URDF, so the optical-frame convention comes from the robot model
rather than being reimplemented — that is the part most likely to be silently wrong if you
hand-roll it.

`check_calibration.py` then reports, per frame:

```
        table plane per camera                 left-vs-right overlap
 frame  left h/tilt        right h/tilt        n        p50      p95
```

- **table plane per camera** — RANSAC-fits the table each camera sees on its own, and reports
  its height above the base `z = 0` plane and its tilt from horizontal. Both should be near
  zero. A camera that is right about the table but wrong about where it is standing shows up
  here as a height offset.
- **left-vs-right overlap** — the distance from each left point to the nearest right point,
  over the region both cameras see. This is the direct left-against-right error, and it is
  the number that matters: two cameras can each be tilted and still agree with each other,
  but if they disagree, at least one pose is wrong.

Read the verdict line at the bottom, but read the spread too — a good mean with a large
spread across frames means something is moving that should not be.

## Looking at it

Numbers say *how much*; the clouds say *which way*.

```bash
python3 ~/share/view_pointclouds.py <dataset-name>                 # offscreen PNGs
python3 ~/share/view_pointclouds.py <dataset-name> --interactive   # orbit, measure, pick points
```

Offscreen mode writes an iso/side/top triple per frame, twice: once in the clouds' own
colours, once tinted per camera (left red / right blue) so a misalignment reads as a red
surface floating above a blue one rather than as a slightly thick table. Interactive mode
adds frame stepping, per-camera toggles, a side-by-side split, a z-slice, an on-demand
calibration readout and point picking. Press **H** in the window for the key map.

## Timestamp skew between the cameras

Spatial calibration is only half of it. If the two cameras' frames are taken at different
moments, they disagree about anything that moves — including the arm.

```bash
python3 ~/share/cam_skew.py     # with the camera drivers running
```

It subscribes to both colour topics and reports the offset between their header stamps. A
steady offset is a fixed lag you can account for; a wandering one means the two streams are
not being driven off a common clock, which is a driver/transport problem rather than a
calibration one. See [teleop.md](teleop.md#health-check--diagnostics) if it coincides with
dropped frames — the cameras and the robot share a link on this bench.

## Datasets recorded before the recorder stored calibration

Older datasets have no `meta/intrinsics.json` or `meta/extrinsics.json`, so the tools above
have nothing to work from. Backfill them:

```bash
python3 ~/share/backfill_camera_meta.py
```

It writes both files into existing datasets under `~/share/lerobot_data`, in the same layout
the recorder now writes for new ones — intrinsics from a capture of the live `camera_info`
topics, extrinsics from the fixed-cameras calibration file.

**This backfills what is true *now*.** If the cameras were moved or re-calibrated since those
episodes were recorded, it writes the current calibration onto old data and makes it
confidently wrong instead of merely incomplete. Only backfill datasets you know were recorded
under the calibration currently in the file.
