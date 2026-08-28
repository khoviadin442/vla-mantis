# Teleoperation — VR control of the arm (Meta Quest 2 / HTC Vive)

> Part of **vla-mantis**. This is the teleop half: driving the arm by hand and recording
> what you drove. For running a trained policy on the same arm see [../policy/README.md](../policy/README.md);
> for the pipeline as a whole see [the top-level README](../README.md).

Map a VR controller onto a robot arm through differential IK with a collision barrier, and
record episodes straight into a stock **LeRobot v3** dataset. A controller publisher turns
Quest 2 (or HTC Vive) poses into `/vive/pose` + `/vive/buttons`; a bridge maps them onto the
arm and drives its gripper. Episodes replay with the stock `lerobot-replay` CLI.

```
operator hand ─► quest_pub.py ─► /vive/pose, /vive/buttons ─► teleop_mantis.py ─► <arm command topic>
   (host: USB adb, or ALVR/SteamVR)                             (container: Pink IK + collision barrier)
```

The teleop itself is **robot-agnostic** — the robot is defined entirely by a URDF/SRDF and a
config file (see [Porting to another arm](#porting-to-another-arm)). It ships tuned for the
PRL **mantis** rig (dual UR5, left arm + Weiss WSG50 gripper), which the examples below use.

> **Scope.** This is the teleop *layer*. It needs a ROS 2 workspace and a container image,
> which come from **[prl_ur5_ros2](https://github.com/inria-paris-robotics-lab/prl_ur5_ros2)**
> (this repo carries the teleop-specific patches for it). And it targets real hardware — an
> arm, a gripper, and a Quest 2. The *software* install below is meant to run start to finish;
> the per-bench values you must set are under [Before the first run](#before-the-first-run-on-hardware).

**Contents:** [What you need](#what-you-need) · [Install](#install) · [First run](#first-run-smoke-test)
· [Running](#running) · [Controls](#controls-right-touch-controller) · [Recording](#recording-a-dataset)
· [Replay](#replay) · [Health check & diagnostics](#health-check--diagnostics) · [How it works](#how-it-works)
· [Porting to another arm](#porting-to-another-arm) · [Troubleshooting](troubleshooting.md)

---

## What you need

**Hardware**
- Meta Quest 2 + a Touch controller, in **developer mode**, with a USB-C cable to the workstation.
- The robot: an arm with a joint-group position controller, a gripper exposing a `control_msgs/GripperCommand` action, and (optional, for recording) cameras. The reference bench is the mantis (dual UR5 + WSG50 + Femto Mega cameras).

**Software on the workstation**
- Linux with a working **`docker`** (user in the `docker` group), plus an NVIDIA runtime if you use the GPU flags.
- **`git`** and **`git-lfs`**.
- **`adb`** for the Quest-over-USB backend — an unpacked Google `platform-tools` in
  `~/platform-tools`, first on `PATH`. It must be the *same* binary the adb-server
  container runs (see [ADB on the workstation](#adb-on-the-workstation-adb-server-in-a-container));
  a distro `android-tools-adb` on `PATH` will fight it over version mismatch.
- ROS 2, the teleop Python stack, and lerobot all live inside the container image — you do **not** install them on the host.

Only one step needs root: a udev rule so `adb` sees the headset (avoidable — see [troubleshooting.md](troubleshooting.md)). Everything else installs into your home directory.

Paths below use these defaults; all are overridable (env vars in the scripts):

| What | Default location |
|---|---|
| this repo | `~/vla-mantis` |
| robot repo (image + workspace source) | `~/prl_ur5_ros2` |
| host controller environment | `~/SO-100-HTC-vive-teleop` |
| container **share dir** (workspace, generated URDF, datasets) | `~/teleop_share` (`SHARE_DIR`) |
| host package prefix | `~/mantis_host_deps/fakeprefix` (`HOST_DEPS`) |
| USB backend deps | `~/oculus_reader`, `~/quest_deps` |

---

## Install

### 1. This repo

```bash
git clone https://github.com/khoviadin442/vla-mantis.git ~/vla-mantis
```

### 2. The container image

The teleop runs in the `prl_ur5_ros2` ROS 2 Jazzy image extended with the teleop Python
stack (pinocchio, pink, qpsolvers, CPU torch, `lerobot[async,dataset]==0.6.1`). That
extension ships here as a patch, so build the image from a `prl_ur5_ros2` clone with the
patch applied:

```bash
git clone https://github.com/inria-paris-robotics-lab/prl_ur5_ros2.git ~/prl_ur5_ros2

# apply this repo's container patches (teleop Dockerfile, its vision_visp fix-up script,
# and the --ipc=host start script) - copy all three, the Dockerfile COPYs the fix-up script
cp ~/vla-mantis/patches/prl_ur5_ros2/docker-ros2/Dockerfile          ~/prl_ur5_ros2/docker-ros2/
cp ~/vla-mantis/patches/prl_ur5_ros2/docker-ros2/fix_vision_visp.py  ~/prl_ur5_ros2/docker-ros2/
cp ~/vla-mantis/patches/prl_ur5_ros2/docker-ros2/start_docker.bash   ~/prl_ur5_ros2/docker-ros2/
```

(Or clone a `prl_ur5_ros2` fork that already has these applied and skip the two `cp`s.)

### 3. Lay out the workspace (first pass)

`setup_workstation.sh` copies the teleop files into the share dir, clones the ROS workspace,
applies the workspace patches, builds the host package prefix, and generates the URDF. Run it
once now — it reports the container-only packages as MISSING, which is expected:

```bash
cd ~/vla-mantis
./setup_workstation.sh
```

### 4. Build the image and the workspace

```bash
# first build is ~30 min: torch + lerobot are several GB
cd ~/prl_ur5_ros2/docker-ros2
./start_docker.bash mantis ~/teleop_share

# inside the container:
cd ~/share/mantis_ws && colcon build --symlink-install
pip install -e ~/share/lerobot_robot_mantis        # only needed for replay
```

### 5. Finish the setup (second pass)

Back on the host, with the container still up — this completes the host prefix and writes the URDF:

```bash
cd ~/vla-mantis
./setup_workstation.sh                              # "generated (N lines)"
```

If any step behaves differently, **[troubleshooting.md](troubleshooting.md)** lists the known
failures and what they mean.

### The host controller environment

`quest_pub.py` runs on the host and needs a Python where `import rclpy, numpy` passes (plus
`openvr` for the ALVR backend, or `oculus_reader, ppadb` for USB). Any such environment works.
The reference one is a **pixi** workspace that needs no root:

```bash
git clone https://github.com/khoviadin442/SO-100-HTC-vive-teleop.git ~/SO-100-HTC-vive-teleop
cd ~/SO-100-HTC-vive-teleop && pixi install
source .pixi/envs/default/setup.bash
which python && python -c "import rclpy, numpy; print('host env OK')"
```

`which python` must point inside `.pixi/envs/default/bin`. If your environment lives
elsewhere, point the launch scripts at it instead of editing them:

```bash
export ROS_SETUP=/path/to/setup.bash          # host ROS env
export HOST_DEPS=/path/to/fakeprefix          # AMENT_PREFIX_PATH entry with the mesh packages
export OCULUS_READER=/path/to/oculus_reader   # USB backend only
export QUEST_DEPS=/path/to/quest_deps         # USB backend only (ppadb)
```

### The Quest-over-USB backend (recommended)

```bash
git clone https://github.com/rail-berkeley/oculus_reader ~/oculus_reader
pip3 install --target ~/quest_deps pure-python-adb
```

The APK in that repo is stored in **Git LFS** — without `git-lfs` the clone gets a 132-byte
stub and installation fails with `Failed to parse APK file`. Fetch it directly if that happens:

```bash
cd ~/oculus_reader/oculus_reader/APK
curl -sL -o teleop-debug.apk \
  https://media.githubusercontent.com/media/rail-berkeley/oculus_reader/main/oculus_reader/APK/teleop-debug.apk
```

Plug in the headset, accept "Allow USB debugging" inside the Quest, and confirm:

```bash
adb devices          # the headset must be listed as 'device', not 'unauthorized' / 'no permissions'
```

### ADB on the workstation (adb server in a container)

Rather than depend on the host's adb version and USB permissions, run the **adb server inside
a small privileged container** that owns the USB bus and shares the host network — so every
adb client (the host `adb`, the ROS container, and `quest_pub` via `ppadb`) talks to one
server on `127.0.0.1:5037`.

Why a container: a pinned `platform-tools` adb binary (read-only bind), USB via
`--privileged --device=/dev/bus/usb`, the adb auth key persisted through `~/.android`, and one
shared server via `--net=host`. Create it once:

```bash
docker run -d --name adb-server \
  --privileged --net=host \
  --device=/dev/bus/usb \
  -v ~/platform-tools:/pt:ro \
  -v ~/.android:/root/.android \
  --entrypoint /pt/adb \
  prl_ros2:$(id -un) \
  nodaemon server
```

Daily use — **kill first, then start**, the order matters:

```bash
adb kill-server                   # FIRST: kills whatever server owns 127.0.0.1:5037
docker start adb-server           # then bring up the container's server
                                  # >>> put the headset ON and accept "Allow USB debugging" <<<
adb devices                       # the headset should read 'device'
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close   # keep it awake while off your head
```

**The headset prompts every time the server restarts.** Starting `adb-server` starts a fresh
adb server, which re-handshakes with the Quest, and the Quest pops "Allow USB debugging?" —
inside the headset, where you will not see it unless you put it on. Until you tap **Allow**,
`adb devices` reads `unauthorized` and `quest_pub` cannot attach. So: `docker start`, headset
on, accept, *then* `adb devices`. Ticking "Always allow from this computer" makes the prompt
stick for the current key, but a `kill-server`/restart cycle or a headset reboot usually brings
it back — treat the prompt as part of the daily routine, not a one-off.

`adb kill-server` is not "kill the host's server" — it connects to `127.0.0.1:5037` and kills
whoever is listening there. Because the container shares the host network, once `adb-server` is
up **that is the container's server**, and killing it takes the container down with it (the
server is the entrypoint, and there is no restart policy — `docker logs adb-server` shows
`adb server killed by remote request`). Run it the other way round and you have to
`docker start adb-server` again.

The host `adb` client does keep working against the container's server: `adb devices`,
`adb shell`, `adb reboot` all behave normally from the host, because `--net=host` puts the
server on the host's own `127.0.0.1:5037` and `adb` on `PATH` is the same pinned
`~/platform-tools/adb` binary the container runs. Version parity is what matters — a
*different* host adb (e.g. a distro `android-tools-adb`) would refuse to talk to it, print
`adb server version doesn't match this client; killing...`, and silently take the container
down. Keep `~/platform-tools` ahead of `/usr/bin` on `PATH`.

If tracking degrades mid-session, `adb reboot` the headset (see [Health check](#health-check--diagnostics)).

### What `setup_workstation.sh` does

It copies the teleop files into `$SHARE_DIR`, clones + patches the ROS workspace
(`mantis_ws`), builds the host package prefix, and generates `mantis.urdf`. It **never
overwrites** existing files — pass `--force-files` / `--force-patches`, and delete
`mantis_ws/mantis.urdf` to regenerate it. Paths are overridable via `SHARE_DIR`, `HOST_DEPS`,
`CONTAINER_NAME`, `ROS_SETUP`.

The workspace build lives in the share dir, so it survives the container. The editable
`lerobot_robot_mantis` install does **not** (the container runs `--rm`) — repeat step 4's
`pip install` in each new container that needs `run_replay.sh`.

---

## First run (smoke test)

Before touching the robot, confirm the pieces talk to each other. Container up, headset plugged in:

```bash
# host: what does the controller report? (publishes nothing)
~/teleop_share/run_quest_pub.sh --scan

# host: start the publisher, then in the container check the stream arrives
~/teleop_share/run_quest_pub.sh
#   in the container:
ros2 topic hz /vive/pose        # healthy ≈ 70 Hz (adb) / 250 Hz (ALVR); see Health check
ros2 topic echo /vive/buttons   # 6 fields, changing as you press buttons
```

If `/vive/pose` is silent, the headset is asleep or out of tracking — see [Health check](#health-check--diagnostics).

---

## Running

Canonical setup: robot stack and teleop **in the container**, controller publisher **on the host**.

```bash
# terminal 1 (container) — hardware; wait for "forward_position_controller ... activated"
ros2 launch prl_ur5_run real.launch.py launch_moveit:=false   # add activate_cameras:=true to record
# terminal 2 (container) — teleop; wait for "Teleop ready"
python3 ~/share/teleop_mantis.py
# terminal 3 (host) — Quest publisher (defaults to USB adb)
~/teleop_share/run_quest_pub.sh
```

- Everything on the host in one command, no container: `./run_teleop_quest.sh`.
- Vive equivalents: `run_vive_pub.sh` and `run_teleop_mantis.sh`.
- **Run only one publisher at a time** — they share the topics.

`run_quest_pub.sh` is preconfigured (`QUEST_BACKEND=adb`, `QUEST_ROT_OFFSET=-90,0,0`); both are
overridable on the command line. Over adb the publisher emits one message per headset sample,
exactly as measured - it never predicts or resamples.

---

## Controls (right Touch controller)

| Button | Action |
|---|---|
| **thumbstick click** | engage / freeze |
| **trigger**, squeezed fully | gripper toggle: one click closes, the next opens |
| **B** | episode start / stop (start from frozen also engages; stop freezes and drives HOME) |
| **A** | HOME ramp to the home pose; press again to cancel |
| **grip**, held | axis lock ("drawer mode"): orientation frozen, motion constrained to the gripper axis |

**Tracking is inside-out:** the headset's cameras must see the controller — keep it facing
your hands (wear it, or aim it at the workspace) in even light. Parked on a desk, the
controller drifts out of view, tracking drops, and motion turns choppy (see [Health check](#health-check--diagnostics)).

---

## Recording a dataset

`record:` in the config controls the recorder (dataset root, `fps`, cameras). Recording needs
the cameras up (`activate_cameras:=true`). **Confirm the pose stream is healthy first** — a
degraded stream produces choppy episodes. Then:

1. **Engage** with the thumbstick and move to a start pose.
2. Press **B** to start an episode (`EPISODE N RECORDING`).
3. Perform the task; toggle the gripper with the trigger.
4. Press **B** to stop — the arm freezes and drives HOME; the episode is queued to save.
5. Repeat for each episode.
6. **Exit the teleop with Ctrl-C** — this finalizes the dataset and encodes the videos. Wait
   for `dataset finalized`; don't Ctrl-C twice or you lose the queued episodes.

Notes:
- `record.fps` **must equal the camera stream rate** (a mismatch skews replay speed; the recorder warns).
- Set the dataset **name** and **task** before launching the teleop:
  `RECORD_NAME=my_task RECORD_TASK="pick up the mug" python3 ~/share/teleop_mantis.py`.
  `RECORD_NAME` becomes the folder under `record.root` (and the `<hf_namespace>/my_task` push repo);
  `RECORD_TASK` is the natural-language instruction stored with every frame (falls back to `record.task` in the config).
- Episodes shorter than `record.min_frames` are discarded (why MENU is debounced).

## Replay

```bash
pip install -e lerobot_robot_mantis      # once, in the container
./run_replay.sh 0                        # episode 0
./run_replay.sh 3 my_task                # episode 3 of the RECORD_NAME=my_task dataset
```

Teleop must be stopped — both it and replay publish the arm command topic.

---

## Health check & diagnostics

Almost every "teleop feels bad" symptom — choppy motion, flicks, a laggy gripper — traces to
a **degraded Quest tracking stream**, not the robot or the code. Watch the pose rate; the
teleop prints a `POSESTREAM` line every 10 s, and `ros2 topic hz /vive/pose` shows it live.

| | healthy | degraded |
|---|---|---|
| `/vive/pose` rate | ~70 Hz | ~50 Hz |
| jitter (p95 interval) | ~24 ms | ~52 ms |
| feel | smooth, responsive gripper | choppy, gripper lags |

Below `teleop.pose_hz_warn` (65 Hz) the teleop logs **`QUEST POSE RATE LOW`** — the headset
lost solid 6-DOF tracking (out of view, or warm after hours). Fix it:

1. **Reboot the headset** — the "reset to morning" button: `adb reboot`, then restart the publisher.
2. Keep the headset's cameras on the controller, in good even light.
3. Keep the headset cool; fresh AA in the controller for long sessions.

Confirm the cause directly (should read `POSITION`/6-DOF, not `ORIENTATION`):

```bash
adb shell dumpsys OVRRemoteService | grep -iE "TrackingStatus|tracking lost"
```

There is no smoothing or prediction to hide a bad stream, so fix the headset when the warning fires.

**Session monitor.** `teleop_monitor.py` (host) samples the whole system once a second and,
afterward, correlates it with the teleop log, the driver log, and the recorded dataset — UDP
drops, CPU/thermal, robot-network ping, Quest thermals, per-episode camera loss, flick
signatures — and prints ranked problems:

```bash
~/teleop_share/teleop_monitor.py start     # before a session
~/teleop_share/teleop_monitor.py report    # any time, also mid-session
~/teleop_share/teleop_monitor.py stop
```

`udp_watch.py` is a lighter UDP-drop-only watcher (`start` / `report` / `stop`).

---

## How it works

Three processes, split across the host and the container.

1. **Controller publisher (`quest_pub.py`, host).** Reads the Quest pose over USB (adb → the
   `oculus_reader` APK, parsed via `adb logcat`) and republishes `/vive/pose` +
   `/vive/buttons`, one message per headset sample (~70 Hz when tracking is healthy).
2. **Teleop bridge (`teleop_mantis.py`, container).** On engage it anchors the controller
   pose to the arm and maps hand motion → a target EE pose (scaled, heading-normalized,
   one-euro filtered), solves joint velocities with **Pink differential IK** under a
   **collision barrier** (a CBF-QP that makes the arm dodge instead of freeze), and
   re-publishes through an output **shaper thread at 250 Hz** under velocity/acceleration
   limits — so the arm stays smooth even when the input or the executor hiccups. The gripper
   runs on its own timer.
3. **Recorder (`lerobot_recorder.py`).** MENU starts/stops an episode; each primary-camera
   frame stores one LeRobot frame (state = measured joints + gripper, action = commanded),
   pairs the secondary/depth cameras by nearest timestamp, and writes a stock LeRobot v3
   dataset. Video encoding is deferred to teleop exit.

Host ↔ container traffic is ROS 2 DDS forced **UDP-only** (`fastdds_udp_only.xml`), because
FastDDS shared memory does not cross the container boundary. The arm command runs on its own
250 Hz thread, so camera/recording load never stutters the arm — but the gripper and buttons
share the ROS executor, which is why they feel a degraded input stream first.

---

## Porting to another arm

The teleop is robot-agnostic: the URDF/SRDF and the joint lists in the config drive it, and
the IK (Pink), the SVD-based singularity damping, and the SRDF collision barrier are all
generic. To bring up a different arm:

1. Provide its **URDF + SRDF**, and point `urdf:` / `srdf:` / `ee_frame:` at them.
2. List its joints in `arm` / `arm_cmd_joints`, set `shoulder_joint`, `home_q_deg`, the
   collision floors, and the gripper action.
3. If it is **not a 6-DOF UR**, set the shape keys (they default to the 6-DOF UR behaviour):
   ```yaml
   teleop:
     reach_shell_joints: [...]     # base/positioning joints (default: all but the last 2)
     roll_joint: <joint>           # tool-roll joint (default: the last arm joint)
     sing_report_joint: <joint>    # diagnostic only (default: the 5th joint)
   ik:
     wrist_vel_joints: [...]       # joints capped at ik.wrist_vel_scale (default: any *wrist*)
   ```
   Redundancy (e.g. a 7-DOF Franka) is handled automatically by Pink's null-space (`posture_cost`).

Collision tuning (`hull_split`, `d_min_*`, `wire_box_*`) and `home_q_deg` are per-robot
*values*, set in the config — not code.

---

## Before the first run on hardware

Per-bench values no script can guess:
- gripper IP, force and speed — `patches/wsg50-ros-pkg/wsg_50_driver/config/wsg50_setup.yaml`
- arm mounting poses — `patches/prl_ur5_robot_configuration/config/standard_setup.yaml`
- camera topics and `fps` — the `record:` section of `config_teleop_mantis.yaml`

The tuned values in `config_teleop_mantis.yaml` depend on each other (speeds, `out_accel`,
`max_joint_lead`); change them **as a set**. The one safe first-run change is lowering
`teleop.scale` to `0.5`. **Bring up the simulation before the hardware, and keep the E-stop
within reach.**

---

## Files & environment variables

See the config file for the full parameter set, and `quest_pub.py`'s header for the
`QUEST_*` environment variables (backend, buttons, rates). Key files:
`teleop_mantis.py` (bridge), `config_teleop_mantis.yaml` (all tuning), `quest_pub.py` /
`vive_pub.py` (publishers), `lerobot_recorder.py` (recorder), `lerobot_robot_mantis/` (replay
plugin), `patches/` (workspace patches, applied by `setup_workstation.sh`),
`teleop_monitor.py` / `udp_watch.py` (diagnostics).
