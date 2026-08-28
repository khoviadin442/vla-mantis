# vla-mantis

**Teleoperate a real robot arm in VR, record what you did as a LeRobot dataset, then hand the
same arm over to a policy trained on it.** Both halves drive the same hardware through the
same robot plugin, so what the policy sees at inference is what the teleop wrote at
recording time — the single property that decides whether any of this transfers.

```
    ┌── you, in a headset ──────────────────────────────────────────────┐
    │                                                                   │
    ▼                                                                   │
 quest_pub.py ──► /vive/pose ──► teleop_mantis.py ──► safety ──► UR5 ───┤
    (host, USB adb)              (Pink IK + collision)                  │
                                        │                               │
                                        ▼                               │
                              lerobot_recorder.py                       │
                                        │                               │
                                        ▼                               │
                          LeRobot v3 dataset (+ camera calibration)     │
                                        │                               │
                                        ▼                               │
                            train, on the GPU box  ─ not in this repo   │
                                        │                               │
                                        ▼                               │
 robot_client_cpu.py ◄── gRPC ── policy server ─────────────────────────┘
    │                             (pi0, GPU box)
    ▼
 mantis_follower ──► safety_filter_controller ──► the same UR5
```

The arm never moves without a filter in front of it: the teleop clamps and collision-checks
inside `teleop_mantis.py`, and the policy path routes through `safety_filter_controller`,
which delta-clamps every reference and holds the last safe command rather than following a
bad one.

> **Scope.** This is the robot-side stack. It needs a ROS 2 workspace and a container image,
> which come from [prl_ur5_ros2](https://github.com/inria-paris-robotics-lab/prl_ur5_ros2)
> (the teleop-specific patches for it live in `patches/`), and it targets real hardware — an
> arm, a gripper, cameras and a Quest 2. **Training is not here.** This repo gets you data in
> and actions out; the checkpoint is trained elsewhere and served over gRPC.
>
> The teleop layer itself is robot-agnostic — the robot is a URDF/SRDF plus a config file.
> It ships tuned for the PRL **mantis** rig (dual UR5, left arm + Weiss WSG50 gripper).

## Guides

| | |
|---|---|
| **[docs/teleop.md](docs/teleop.md)** | Install, first run, controls, recording a dataset, replay. **Start here.** |
| **[policy/README.md](policy/README.md)** | Running a trained policy: the client, the bring-up ladder, the controller contract. |
| **[docs/calibration.md](docs/calibration.md)** | Verifying the camera extrinsics off a recorded dataset, and backfilling old ones. |
| **[docs/tools.md](docs/tools.md)** | Which diagnostic answers which question. |
| **[docs/troubleshooting.md](docs/troubleshooting.md)** | Symptoms seen during bring-up, and what they mean. |

## Install

Full version in [docs/teleop.md](docs/teleop.md#install) — it is the part with the most ways
to go wrong. The shape of it:

```bash
git clone https://github.com/khoviadin442/vla-mantis.git ~/vla-mantis

# 1. the container image (prl_ur5_ros2 + this repo's Dockerfile patch, ~30 min first build)
git clone https://github.com/inria-paris-robotics-lab/prl_ur5_ros2.git ~/prl_ur5_ros2
cp ~/vla-mantis/patches/prl_ur5_ros2/docker-ros2/* ~/prl_ur5_ros2/docker-ros2/

# 2. lay out the share dir, clone + patch the ROS workspace, build the host prefix
cd ~/vla-mantis && ./setup_workstation.sh

# 3. build image + workspace  (add --rebuild if a prl_ros2 image predates the patch)
cd ~/prl_ur5_ros2/docker-ros2 && ./start_docker.bash mantis ~/teleop_share
#   inside: cd ~/share/mantis_ws && colcon build --symlink-install
#           python3 -m pip install --user --break-system-packages -e ~/share/lerobot_robot_mantis

# 4. second pass, now that the container packages exist — writes mantis.urdf
cd ~/vla-mantis && ./setup_workstation.sh
```

`setup_workstation.sh` never overwrites existing files (`--force-files`, `--force-patches` to
insist), and every path is overridable: `SHARE_DIR`, `HOST_DEPS`, `CONTAINER_NAME`,
`ROS_SETUP`.

The calibration tools additionally need open3d, which the image does not ship — see
[docs/calibration.md](docs/calibration.md#prerequisite-open3d). Nothing else does, so skip it
until you need it.

## Teleoperate and record

Get a shell in the container first, then launch from inside it. An interactive shell sources
`.bashrc`, which is where the three ROS prefixes get sourced; a one-shot
`docker exec -it mantis <script>` skips `.bashrc` and can leave `AMENT_PREFIX_PATH` short, so
`package://` mesh lookups fail.

```bash
# terminal 1 — robot stack (container)

# terminal 2 — teleop bridge
docker exec -it mantis bash          # enter the container
  ~/share/run_teleop_mantis.sh       # then, inside it

# terminal 3 — controller publisher (host, not the container)
~/teleop_share/run_quest_pub.sh
```

Controls, the recording keys and the dataset layout are in
[docs/teleop.md](docs/teleop.md#controls-right-touch-controller).

## Run a policy

Bring it up one component at a time — the ladder in
[policy/README.md](policy/README.md#bring-up-order) exists so that a failure names its own
cause. The last two rungs:

```bash
docker exec -it mantis bash          # enter the container

# then, inside it — read-only: one observation, prints the returned chunk,
# creates no publisher, so the arm cannot move
  PROBE=1 ~/share/policy/run_policy.sh "Grab green cube and place it in the box"

# the real thing (safety_filter_controller must be loaded and active)
  ~/share/policy/run_policy.sh "Grab green cube and place it in the box"
```

Switching the arm from the teleop controller to the safety filter is a manual, two-command
step — `run_policy.sh` checks it and refuses to run blind rather than doing it for you. The
commands are in [policy/README.md](policy/README.md#switching-to-it).

Defaults: `pi0`, 15 fps, chunks of 50 aggregated by weighted average, both colour cameras,
safety filter on, server at `127.0.0.1:8080`. All overridable from the environment.

Two things are not negotiable, and both fail loudly rather than silently:

- **The task string must match the wording the checkpoint was trained on**, not merely
  describe the same intent.
- **`CAMERA_TOPICS` must be exactly the cameras it was trained with.** A key the checkpoint
  has no encoder for is a server-side `KeyError`.

## Repo map

| Path | What |
|---|---|
| `teleop_mantis.py` | The bridge: VR pose → differential IK (Pink) → joint commands, with a collision barrier. |
| `quest_pub.py`, `vive_pub.py` | Controller publishers — Quest 2 over USB adb, or Vive/ALVR over OpenVR. |
| `home_planner.py` | Collision-free joint-space path search to the home pose, shared by teleop and policy. |
| `lerobot_recorder.py` | Episode recording into a stock LeRobot v3 dataset, with camera intrinsics + extrinsics. |
| `policy/` | The robot-side inference client, plus the dummy servers and smoketests used to bring it up. |
| `lerobot_robot_mantis/` | The `mantis_follower` LeRobot robot plugin — the ROS 2 driver both replay and policy talk to. |
| `ros2_packages/safety_filter_controller/` | Our terminal controller: delta-clamps and collision-checks every reference. |
| `patches/` | Changes to `prl_ur5_ros2` and friends: the teleop Dockerfile, gripper fixes, joint limits, camera config. |
| `check_robot.sh`, `teleop_monitor.py`, `udp_watch.py` | Diagnostics — see [docs/tools.md](docs/tools.md). |
| `make_pointclouds.py`, `check_calibration.py`, `view_pointclouds.py`, `cam_skew.py`, `backfill_camera_meta.py` | Camera calibration — see [docs/calibration.md](docs/calibration.md). |

### The share dir is flat on purpose

`setup_workstation.sh` copies the Python files into `$SHARE_DIR` **side by side**, and the
repo root mirrors that. This is load-bearing, not an accident of history:

- `teleop_mantis.py` does `from home_planner import HomePlanner` and
  `from lerobot_recorder import EpisodeRecorder`,
- `policy/probe_policy_server.py` does `sys.path.insert(0, "/home/ros/share")` and imports
  `lerobot_recorder` and `teleop_mantis` from there.

Tidying these into subpackages breaks the container at runtime, where the flat share dir is
what actually exists. If you add a module the teleop imports, add it to the copy list in
`setup_workstation.sh` too.

### The editable install does not survive the container

The container runs `--rm`, so `python3 -m pip install --user --break-system-packages -e
~/share/lerobot_robot_mantis` has to be repeated in each new container that runs replay or
policy. It shows up as `ModuleNotFoundError: lerobot_robot_mantis`; `run_policy.sh` checks
for it and prints the line to run.

## Porting to another arm

The teleop is defined by a URDF/SRDF and `config_teleop_mantis.yaml` — see
[docs/teleop.md](docs/teleop.md#porting-to-another-arm). The policy path additionally needs a
LeRobot robot plugin exposing your arm's observation/action contract; `lerobot_robot_mantis/`
is the worked example, and `policy/_sim_plugin/` is the same contract with the ROS layer
removed, which is the cheaper thing to copy first.
