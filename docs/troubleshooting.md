# Troubleshooting

Symptoms seen while bringing the teleop up, and what they mean. Setup itself is in
[teleop.md](teleop.md). Paths use the README defaults (`~/teleop_share` for the share dir,
`~/prl_ur5_ros2` for the robot repo); adjust if you set `SHARE_DIR` differently.

## Quick diagnostics

```bash
# host
ls ~/teleop_share/mantis_ws/src                          # 7 packages
ls ~/mantis_host_deps/fakeprefix/share                   # 13 packages + ament_index
ls -l ~/teleop_share/mantis_ws/mantis.urdf
source "$ROS_SETUP"                                      # host publisher environment
which python && python -c "import rclpy, openvr, pinocchio, pink; print('host env OK')"

# container
python3 -c "import pinocchio, pink, qpsolvers, lerobot; print('deps OK')"
ros2 topic hz /vive/pose                                 # healthy ~70 Hz (adb) / ~250 Hz (ALVR)
ros2 topic echo /vive/buttons                            # 6 fields
```

## Motion is choppy / the gripper lags / it gets worse after a while

By far the most common runtime complaint, and it is almost always the **Quest tracking
stream**, not the robot or the code. When the headset can no longer see the controller well
it drops from 6-DOF to orientation-only tracking, and the pose stream falls from ~70 Hz to
~50 Hz and jittery. The arm turns choppy, recordings show flicks, and the gripper's
press → start delay grows. Classic tell: fine at the start of the day, degrades after a
couple of hours.

The teleop logs **`QUEST POSE RATE LOW`** when `/vive/pose` drops below `teleop.pose_hz_warn`
(65 Hz). Confirm and fix:

```bash
ros2 topic hz /vive/pose        # ~50 Hz instead of ~70 Hz confirms it
adb shell dumpsys OVRRemoteService | grep -iE "TrackingStatus|tracking lost"
#   TrackingStatus: ORIENTATION (+ a climbing "tracking lost count") = the headset lost the controller
```

Fixes, in order:

1. **Reboot the headset** — resets heat and tracking to the "start of day" state:
   `adb reboot`, then restart `run_quest_pub.sh`.
2. Keep the headset's cameras **looking at the controller**, in **even light** — no glare,
   dark corners, or blank/reflective walls.
3. Keep the headset **cool** (a fan; don't run it continuously for hours) and put a **fresh
   AA** in the controller.

Restarting `quest_pub` or `adb` does **not** fix it — the fault is on the headset side.
The publisher passes the headset's samples through untouched, so nothing masks a degraded
stream. For a full session picture, run `teleop_monitor.py start` before the session and
`teleop_monitor.py report` after.

## The container has no `pip`, `pinocchio` or `lerobot`

The image was built before the teleop patch was applied. Confirm the patched Dockerfile is in
place and rebuild:

```bash
grep -c "pin-pink\|lerobot" ~/prl_ur5_ros2/docker-ros2/Dockerfile   # non-zero
docker rm -f mantis
cd ~/prl_ur5_ros2/docker-ros2 && ./start_docker.bash mantis ~/teleop_share --rebuild
```

If the grep is zero, you skipped the container-patch step — copy this repo's
`patches/prl_ur5_ros2/docker-ros2/Dockerfile` over `~/prl_ur5_ros2/docker-ros2/` first
(see README, "The container image"). `--rebuild` passes `--no-cache`; dropping it reuses the
layer cache and is usually what you want.

## `container name /mantis is already in use`

A stopped container kept the name. `docker rm -f mantis`.

## `bash: /home/ros/share/mantis_ws/install/setup.bash: No such file or directory`

Printed on entry by `.bashrc` until the workspace has been built once. Harmless; run
`colcon build --symlink-install` in `~/share/mantis_ws`.

## `error: option --uninstall not recognized` during `colcon build`

`--symlink-install` runs `setup.py develop --uninstall`, and setuptools 80 removed the
`develop` command. The Dockerfile pins `setuptools<80`; an image built before that pin needs
it applied by hand:

```bash
python3 -m pip install --user --break-system-packages "setuptools<80"
```

## `xacro.XacroException: package '<name>' not found`

The host package prefix is incomplete — `$(find ...)` resolves through
`share/ament_index/resource_index/packages/<name>`, not the directory alone. Start the
container and re-run `./setup_workstation.sh`, which builds the whole prefix: eight packages
symlinked out of `mantis_ws/src`, and `ur_description`, `ur_robot_driver`, `ur_client_library`,
`realsense2_description`, `orbbec_description` copied out of the image.

## `ValueError: Mesh package://... could not be found`

The teleop resolves `package://` against `AMENT_PREFIX_PATH`, so every prefix holding a
referenced package must be sourced. In the container that means all three:

```bash
source /opt/ros/jazzy/setup.bash                 # ur_description, realsense2_description
source ~/ros2_utils_ws/install/setup.bash        # orbbec_description
source ~/share/mantis_ws/install/setup.bash      # prl_ur5_description, wsg_50_simulation
```

An interactive shell gets all three from `.bashrc`; `docker exec ... bash -c` does not. If
instead the URDF holds absolute `file:///...` mesh paths, it was generated by an older setup
script — delete `mantis.urdf` and regenerate it.

## The gripper wire box is missing from the URDF

```bash
grep -c 'left_gripper_connector_link' ~/teleop_share/mantis_ws/mantis.urdf   # != 0
```

Zero means the URDF was generated before the workspace patches were applied. Delete it and
re-run `./setup_workstation.sh`. (The wire box lives in
`patches/wsg50-ros-pkg/wsg_50_simulation/urdf/wsg_50.urdf.xacro`; upstream has no such link,
which is why the patch must be applied.)

## `adb devices` shows `no permissions`

The only step that needs root: a udev rule for the headset's USB vendor id. Without it, use
the ALVR backend (`QUEST_BACKEND=openvr`) over Wi-Fi instead of `adb` over USB, or run the
adb server in the container (README, "ADB on the workstation").

## The host publisher dies on `import openvr`

`run_quest_pub.sh` picked up the system python instead of the host ROS environment. After
sourcing `$ROS_SETUP`, `which python` must point inside that environment. With the pixi
workspace, `pinocchio`, `pink`, `qpsolvers`, `scipy`, `coal` and the pypi `openvr` must all be
present — a workspace whose `pixi.toml` predates the teleop is missing them:

```bash
pixi add pinocchio pink qpsolvers scipy coal
pixi add --pypi openvr
```

## No sudo on the machine

Everything except the udev rule installs into the home directory: pixi via the installer from
pixi.sh, `adb` from an unpacked `platform-tools`. Docker must already be installed with your
user in the `docker` group.

## Before the first run on hardware

These are per-bench values that no script can guess:

- gripper IP, force and speed — `patches/wsg50-ros-pkg/wsg_50_driver/config/wsg50_setup.yaml`
- arm mounting poses — `patches/prl_ur5_robot_configuration/config/standard_setup.yaml`
- camera topics and `fps` — the `record:` section of `config_teleop_mantis.yaml`

The values in `config_teleop_mantis.yaml` were tuned on the robot and depend on each other
(speeds, `out_accel`, `max_joint_lead`); change them as a set, not one line at a time. The one
safe first-run exception is lowering `teleop.scale` to `0.5`. Bring the simulation up before
the hardware, and keep the E-stop within reach.
