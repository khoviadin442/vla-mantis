# policy/

> Part of **vla-mantis**. The teleop half — driving the arm by hand and recording episodes —
> is in [../docs/teleop.md](../docs/teleop.md); the pipeline as a whole is in
> [the top-level README](../README.md).

Everything for running a trained policy on the Mantis left arm. The policy itself runs on
the GPU box; what lives here is the **robot side** — the client that streams observations
there and executes the action chunks that come back, plus the scaffolding used to bring
that path up without a checkpoint in the loop.

```
run_policy.sh  ->  robot_client_cpu.py  ->  gRPC 127.0.0.1:8080  ->  policy server (GPU box)
                          |
                          v
                   mantis_follower  ->  /safety_filter_controller/commands  ->  UR5 left arm
```

## Run it

```bash
docker exec -it mantis ~/share/policy/run_policy.sh "Grab green cube and place it in the box"
```

Read-only dry run first — one observation, prints the returned chunk, creates no publisher
so the arm cannot move:

```bash
PROBE=1 ~/share/policy/run_policy.sh "Grab green cube and place it in the box"
```

Overridable from the environment: `POLICY_SERVER`, `CHECKPOINT`, `CAMERA_TOPICS`,
`POLICY_TYPE`, `POLICY_DEVICE`, `FPS`, `ACTIONS_PER_CHUNK`, `CHUNK_SIZE_THRESHOLD`,
`USE_SAFETY_FILTER`, `ROBOT_ID`, `SKIP_CONTROLLER_CHECK`. Extra arguments pass through to
the client. See the header of `run_policy.sh`.

The task string must match the wording the checkpoint was trained on, and `CAMERA_TOPICS`
must be exactly the cameras it was trained with — a key the checkpoint has no encoder for
is a server-side `KeyError`, not a silent ignore.

## Files

| File | Role |
|---|---|
| `run_policy.sh` | The entry point. Sources ROS, refuses to run against a wrong/absent controller, execs the client. |
| `robot_client_cpu.py` | The stock lerobot `async_client`, unmodified, plus a 2-line shim remapping the server's CUDA tensor storages to CPU at unpickle time (this machine's torch is CPU-only). |
| `probe_policy_server.py` | One observation, all four RPCs, prints the chunk. No publisher is created, so the arm cannot move. |
| `dummy_policy_server.py` | Serves a hardcoded trajectory. No GPU, no checkpoint, ignores observations — proves the client/server plumbing on its own. |
| `hardcoded_episode.py` | That trajectory: episode 52 of `grab_and_place_cube` baked into literal Python. |
| `export_episode_actions.py` | Regenerates the above from any recorded episode. |
| `dummy_policy_server_smoketest.py` | Speaks the wire protocol directly, no robot. Catches column-order / chunk-indexing / aggregation bugs. |
| `_sim_plugin/` | `MantisFollower`'s observation/action contract without ROS; logs executed actions to CSV. Run artifacts (`logs/`, `client.log`, `executed_actions.csv`) are gitignored. |

## Bring-up order

Each step adds one real component, so a failure names its own cause:

1. `dummy_policy_server_smoketest.py` — protocol only, no robot, no server.
2. `dummy_policy_server.py` + `_sim_plugin` — the client's threads, queue and aggregation.
3. `dummy_policy_server.py` + the real arm — motion, on a trajectory that is known-good.
4. `PROBE=1 run_policy.sh` — the real server accepts our feature spec and its tensors
   deserialize here. Still cannot move the arm.
5. `run_policy.sh` — the real thing.

## What is NOT in this folder: the robot plugin

`../lerobot_robot_mantis/` is the `mantis_follower` plugin — the piece that actually talks to
the arm. lerobot does not know what a UR5 is; it loads a *robot plugin* that exposes two
things, "give me an observation" and "execute this action", and drives the ROS 2 topics
behind them. Every client here goes through it.

It sits one level up rather than inside `policy/` because `../run_replay.sh` uses it too. It
is a peer of this folder, not part of it.

### Why a pip install at all

lerobot finds the plugin by **importing it by name** (`import lerobot_robot_mantis`), not by
file path. A folder sitting in the share dir is invisible to Python until something puts it
on the import path. That is all the install does — it registers the package with the
container's Python.

Run it **inside the container**, once per container:

```bash
python3 -m pip install --user --break-system-packages -e ~/share/lerobot_robot_mantis
```

| Flag | Why |
|---|---|
| `-e` | *Editable.* Installs a pointer to `~/share/lerobot_robot_mantis`, not a copy. Edit `mantis_follower.py` on the host and the next run picks it up — no reinstall. |
| `--user` | Installs into your home instead of the system Python. No root needed. |
| `--break-system-packages` | The container's Python is externally managed (PEP 668), so pip refuses to touch it without this. It sounds alarming; it only means "yes, I know this is a distro Python". |

### It does not survive the container

`start_docker.bash` runs the container with `--rm`, so the container filesystem — including
this install — is destroyed on exit. The share dir is a bind mount and survives; the
installed *pointer* to it does not. **Every new container needs the line above again.**

The symptom is:

```
ModuleNotFoundError: No module named 'lerobot_robot_mantis'
```

`run_policy.sh` checks for this before doing anything else and prints the exact command, so
you do not have to remember it.

Two things here do **not** need the plugin, which is why they are the first rungs of the
bring-up ladder: `probe_policy_server.py` only subscribes to topics, and
`dummy_policy_server_smoketest.py` speaks the wire protocol with no robot at all.

## Controller

Policy runs command `safety_filter_controller` (`../ros2_packages/safety_filter_controller/`,
installed into `mantis_ws/src/` by `setup_workstation.sh`),
which delta-clamps every reference and collision-checks its lookahead horizon, holding the
last safe command instead of following a bad one. It is a **terminal** controller — it
replaces `forward_position_controller` rather than sitting in front of it, so exactly one of
the two may be active. Loading and activating it is a manual step; `run_policy.sh` only
checks and refuses to run blind.

### Switching to it

Run both, in this order, inside the container with the robot stack up:

```bash
# 1. load and configure the controller, but leave it stopped
ros2 run controller_manager spawner safety_filter_controller \
    -p ~/share/mantis_ws/src/safety_filter_controller/config/safety_filter_controller.yaml \
    --inactive

# 2. swap it in for the teleop controller, atomically
ros2 control switch_controllers \
    --deactivate forward_position_controller \
    --activate safety_filter_controller
```

The order matters. The spawner only *loads* the controller — `--inactive` means it is
configured and ready but not driving anything, which is what you want while
`forward_position_controller` still holds the arm. `switch_controllers` then does the
handover in a single call, so the arm is never left with no controller or with two of them
fighting over the same joints.

Confirm before you run a policy — exactly one of the two should read `active`:

```bash
ros2 control list_controllers
```

To go back to teleop afterwards, reverse the switch:

```bash
ros2 control switch_controllers \
    --deactivate safety_filter_controller \
    --activate forward_position_controller
```

The spawner step is not needed again for the rest of the container's life: once loaded, the
controller stays loaded and you only switch between the two.

`config_teleop_mantis.yaml`'s `safety_filter_joints` must equal that controller's
`joints.active_joint`, in the same order — the `Float64MultiArray` is indexed positionally,
not by name. `MantisFollower._command_sink()` fails at `connect()` if it does not cover the
driven arm.
