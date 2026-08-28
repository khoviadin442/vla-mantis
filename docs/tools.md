# Diagnostics — which script answers which question

> Part of **vla-mantis**. Symptoms and their causes are in [troubleshooting.md](troubleshooting.md);
> this page is the index of the tools themselves.

The scripts each answer one question, and they are ordered here from "is the robot even
listening" outward to "why did that episode feel wrong". When something is broken, start at
the top — a failure at step 1 makes every measurement below it meaningless.

| Question | Tool | Runs |
|---|---|---|
| Can the arm execute commands at all, or is it only reporting position? | `check_robot.sh` | container |
| Is the VR pose stream healthy? | `teleop_monitor.py`, `ros2 topic hz /vive/pose` | host |
| Are we losing UDP packets? | `udp_watch.py` | host |
| Do the two cameras agree about where things are? | `check_calibration.py` | container |
| Are the two cameras taking frames at the same moment? | `cam_skew.py` | container |
| What actually went wrong in that session? | `teleop_monitor.py report` | host |

## `check_robot.sh` — is the arm listening?

```bash
docker exec -it mantis bash          # enter the container
  ~/share/check_robot.sh             # then, inside it — WHILE the robot stack is up
```

The distinction it draws is the one that wastes the most time: an arm that publishes
`/joint_states` looks alive in every ROS tool, and will still ignore every command you send.
Five checks, in causal order:

1. **Controllers** — `forward_position_controller` must be `active`. `inactive` or
   `unconfigured` means commands go nowhere. If `controller_manager` does not answer at all,
   the robot driver is not running. (On the policy path the terminal controller is
   `safety_filter_controller` instead — exactly one of the two may be active. See
   [../policy/README.md](../policy/README.md#controller).)
2. **State stream** — the `/joint_states` rate. Silence here means the driver is not talking
   to the arm.
3. **Command topic** — `/forward_position_controller/commands` needs at least one subscriber,
   which is the controller. Zero subscribers means nobody reads what the teleop publishes,
   and the arm will sit still while every log looks healthy.
4. **UR mode and safety** — `robot_mode: 7` is RUNNING; anything less and the arm is not
   ready. `safety_mode: 1` is NORMAL, `3` is PROTECTIVE_STOP, `7` is EMERGENCY_STOP.
   `robot_program_running: false` means the program on the teach pendant is not going, so
   commands are accepted and then not executed.
5. **Gripper** — the gripper action server must be listed. Its absence is the
   `gripper action server not ready` line in the teleop log.

Output is in Russian; the section headers are numbered so they line up with the list above.

## Session monitoring

Covered in [teleop.md](teleop.md#health-check--diagnostics): `teleop_monitor.py`
(`start` / `report` / `stop`) samples the whole system once a second and afterwards
correlates it against the teleop log, the driver log and the recorded dataset, printing
ranked problems. `udp_watch.py` is the lighter UDP-drops-only version of the same loop.

Reach for the monitor when a session *felt* wrong but nothing errored — that is the case the
correlation is built for.

## Camera calibration

Covered in [calibration.md](calibration.md): `make_pointclouds.py` → `check_calibration.py`
→ `view_pointclouds.py` to verify the extrinsics off a recorded dataset, `cam_skew.py` for
inter-camera timestamp offset, and `backfill_camera_meta.py` to write calibration metadata
into datasets recorded before the recorder stored it.
