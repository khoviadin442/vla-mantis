"""Dummy policy server: replays a HARDCODED trajectory to `lerobot.async_inference.robot_client`.

Step 1 of bringing up async inference: prove the client<->server plumbing (handshake,
observation streaming, action chunking, aggregation, execution on the robot) *before* a real
checkpoint is in the loop. The server ignores the observation content entirely and answers
every request with a slice of a fixed table.

Two sources for that table (`--source`):

  episode    (default) `hardcoded_episode.ACTIONS` - a literal Python list of joint vectors,
             generated from one recorded LeRobotDataset episode by `export_episode_actions.py`.
             No dataset, no network, no HF cache at serve time: the trajectory is source code.
  waypoints  the hand-authored WAYPOINTS below, interpolated at `--fps`. Useful when there is
             no recording yet, or to author a deliberately tiny motion.

Actions are indexed by the client's action timestep, NOT by a server-side cursor: the chunk for
observation #t is rows t .. t+actions_per_chunk-1. Overlapping chunks therefore carry identical
values for identical timesteps, so `--aggregate_fn_name` (weighted_average etc.) is a no-op and
the executed trajectory is exactly the table, whatever the client's queue does.

Run (inside the mantis container, which is --network host):
```shell
python3 dummy_policy_server.py --host=127.0.0.1 --port=8080 --fps=15 --actions_per_chunk=50
```
Inspect the table without serving anything:
```shell
python3 dummy_policy_server.py --dry_run=true
```
"""

# NB: no `from __future__ import annotations` - draccus reads __annotations__ raw and
# would see the config class as a string.
import logging
import pickle  # nosec - same trusted local channel the stock client/server pair uses
import threading
import time
from concurrent import futures
from dataclasses import asdict, dataclass, field
from pprint import pformat
from queue import Queue

import draccus
import grpc
import numpy as np
import torch

from lerobot.async_inference.configs import PolicyServerConfig
from lerobot.async_inference.helpers import (
    FPSTracker,
    RemotePolicyConfig,
    TimedObservation,
    get_logger,
)
from lerobot.async_inference.policy_server import PolicyServer
from lerobot.transport import services_pb2, services_pb2_grpc  # type: ignore

try:
    import hardcoded_episode
except ModuleNotFoundError as exc:  # pragma: no cover - a deployment mistake, not a code path
    raise SystemExit(
        "hardcoded_episode.py is missing next to this file. Generate it with "
        "export_episode_actions.py, or run through run_dummy_policy_server.sh, which copies both."
    ) from exc

# ── Robot contract ───────────────────────────────────────────────────────────
# Column order of both the action vector and observation.state. Must match
# `MantisFollower.action_features` (config_teleop_mantis.yaml -> `arm`, then the gripper),
# because the client maps tensor column i to the i-th key of `robot.action_features`.
# The client also advertises its own order in SendPolicyInstructions; we re-order to that
# when it disagrees, and refuse to run when the two cannot be reconciled.
ARM_JOINTS = [
    "left_shoulder_pan_joint",
    "left_shoulder_lift_joint",
    "left_elbow_joint",
    "left_wrist_1_joint",
    "left_wrist_2_joint",
    "left_wrist_3_joint",
]
ACTION_NAMES = [f"{j}.pos" for j in ARM_JOINTS] + ["gripper.pos"]

# Arm joints are RADIANS; the WSG50 gripper is an opening WIDTH in METRES
# (config_teleop_mantis.yaml -> gripper.grip_open / grip_close).
GRIPPER_OPEN_M = 0.11
GRIPPER_CLOSED_M = 0.0

# config_teleop_mantis.yaml -> shaper.out_vel: the per-joint speed cap the CommandShaper
# enforces on the stream it publishes. A table asking for more is not dangerous - it is
# simply clamped, and the arm lags - so it warns rather than refuses.
TELEOP_OUT_VEL_RAD_S = 1.5

# `teleop.home_q_deg` from config_teleop_mantis.yaml: the pose the teleop's own HOME button
# ramps to, so it is a known-reachable, collision-free anchor for a hand-authored table.
HOME_DEG = np.array([0.0, -110.0, 66.0, -90.0, -90.0, 0.0], dtype=np.float64)


def _home(**deltas_deg: float) -> np.ndarray:
    """HOME with a few named joints offset, in degrees. Keeps the table readable."""
    q = HOME_DEG.copy()
    for name, delta in deltas_deg.items():
        q[ARM_JOINTS.index(f"left_{name}_joint")] += delta
    return q


# ── Hand-authored fallback trajectory (`--source=waypoints`) ─────────────────
# (seconds to travel from the previous waypoint, 6 arm joints in DEGREES, gripper width in M).
# Joints interpolate linearly across a segment; the gripper is commanded at the START of the
# segment that reaches it, so a "close the gripper" waypoint that repeats the previous pose
# holds the arm still while the fingers move.
WAYPOINTS: list = [
    (0.0, _home(), GRIPPER_OPEN_M),                                            # row 0: start at HOME
    (3.0, _home(), GRIPPER_OPEN_M),                                            # dwell: let the approach ramp settle
    (4.0, _home(shoulder_pan=12.0), GRIPPER_OPEN_M),                           # swing the base
    (2.0, _home(shoulder_pan=12.0), GRIPPER_CLOSED_M),                         # close, arm still
    (4.0, _home(shoulder_pan=12.0, shoulder_lift=-10.0), GRIPPER_CLOSED_M),    # lift
    (4.0, _home(shoulder_lift=-10.0), GRIPPER_CLOSED_M),                       # swing back, still lifted
    (4.0, _home(), GRIPPER_CLOSED_M),                                          # lower to HOME
    (2.0, _home(), GRIPPER_OPEN_M),                                            # release, arm still
    (2.0, _home(), GRIPPER_OPEN_M),                                            # dwell before the table ends
]


def build_waypoint_table(fps: int) -> np.ndarray:
    """Interpolate WAYPOINTS at `fps` into an (n_steps, 7) float32 table of absolute targets."""
    rows: list[np.ndarray] = []
    q_prev = np.radians(np.asarray(WAYPOINTS[0][1], float))
    rows.append(np.concatenate([q_prev, [WAYPOINTS[0][2]]]))

    for duration_s, q_deg, grip_m in WAYPOINTS[1:]:
        q_next = np.radians(np.asarray(q_deg, float))
        n_steps = max(1, int(round(duration_s * fps)))
        for i in range(1, n_steps + 1):
            q = q_prev + (i / n_steps) * (q_next - q_prev)
            rows.append(np.concatenate([q, [grip_m]]))  # gripper steps at the segment start
        q_prev = q_next

    return np.asarray(rows, dtype=np.float32)


def build_episode_table(fps: int) -> np.ndarray:
    """The generated literal table, checked against this server's robot contract."""
    names = list(hardcoded_episode.ACTION_NAMES)
    if names != ACTION_NAMES:
        raise ValueError(
            f"hardcoded_episode.py was generated for action keys {names}, this server serves "
            f"{ACTION_NAMES}. Re-export from a dataset recorded on this robot."
        )
    if hardcoded_episode.FPS != fps:
        raise ValueError(
            f"hardcoded_episode.py holds {hardcoded_episode.FPS} fps rows but --fps={fps}. One row "
            f"is executed per 1/fps, so replay would run at {fps / hardcoded_episode.FPS:.2f}x speed. "
            f"Pass --fps={hardcoded_episode.FPS} here and to the client."
        )
    return np.asarray(hardcoded_episode.ACTIONS, dtype=np.float32)


def build_table(source: str, fps: int, scale: float = 1.0) -> np.ndarray:
    """Rows of absolute joint targets, one per control step.

    `scale` shrinks every row towards row 0, so scale=0 is a static table ("hold the start
    pose") and scale=0.25 is a quarter-amplitude rehearsal of the same shape.
    """
    if source == "episode":
        table = build_episode_table(fps)
    elif source == "waypoints":
        table = build_waypoint_table(fps)
    else:
        raise ValueError(f"--source must be 'episode' or 'waypoints', got {source!r}")

    if scale != 1.0:
        table = (table[0] + scale * (table - table[0])).astype(np.float32)
    return table


def check_speed(table: np.ndarray, fps: int, max_joint_vel: float, warn_joint_vel: float, logger) -> None:
    """Refuse a table with a gross discontinuity; warn about steps the shaper will clamp."""
    if len(table) < 2:
        raise ValueError("action table needs at least 2 rows")
    step = np.abs(np.diff(table[:, : len(ARM_JOINTS)], axis=0))
    speed = step.max(axis=1) * fps
    peak = float(speed.max())

    if peak > max_joint_vel:
        row = int(speed.argmax())
        col = int(step[row].argmax())
        raise ValueError(
            f"row {row}->{row + 1} moves {ARM_JOINTS[col]} by {step[row, col]:.4f} rad in one control "
            f"step = {peak:.2f} rad/s, past the --max_joint_vel={max_joint_vel} backstop. That is a "
            f"discontinuity, not a hand motion: check the table, or lower --scale."
        )

    logger.info(
        f"Trajectory: {len(table)} rows, {len(table) / fps:.1f}s at {fps} fps | "
        f"joint speed p50 {np.percentile(speed, 50):.2f}, p99 {np.percentile(speed, 99):.2f}, "
        f"peak {peak:.2f} rad/s"
    )
    over = int((speed > warn_joint_vel).sum())
    if over:
        logger.warning(
            f"{over} of {len(speed)} steps ask for more than {warn_joint_vel} rad/s (the teleop's "
            f"shaper.out_vel), peaking at {peak:.2f}. CommandShaper clamps them and the arm lags "
            f"briefly - the same clamping that applied while the episode was recorded."
        )


@dataclass
class DummyPolicyServerConfig(PolicyServerConfig):
    """`--fps` MUST match the client's `--fps`: it is the row rate of the table."""

    source: str = field(default="episode", metadata={"help": "'episode' (hardcoded_episode.py) or 'waypoints'"})
    actions_per_chunk: int = field(
        default=50, metadata={"help": "Rows returned per observation. Keep >= the client's value."}
    )
    scale: float = field(default=1.0, metadata={"help": "Shrink every row towards row 0. 0 = a static table."})
    max_joint_vel: float = field(
        default=5.0, metadata={"help": "Hard backstop, rad/s. Catches discontinuities, not speed"}
    )
    warn_joint_vel: float = field(
        default=TELEOP_OUT_VEL_RAD_S, metadata={"help": "Warn above this, rad/s (teleop shaper.out_vel)"}
    )
    loop: bool = field(default=False, metadata={"help": "Wrap around instead of holding the last row"})
    dry_run: bool = field(default=False, metadata={"help": "Print the table and exit without serving"})


class DummyPolicyServer(PolicyServer):
    """`PolicyServer` with the policy replaced by a table lookup.

    Everything on the wire (Ready / SendObservations / GetActions, the observation queue,
    chunk timestamping) is inherited unchanged, so the stock client cannot tell the
    difference between this and a real policy server.
    """

    prefix = "dummy_policy_server"
    logger = get_logger(prefix)

    def __init__(self, config: DummyPolicyServerConfig):  # noqa: D107 - deliberately skips PolicyServer.__init__
        self.config = config
        self.shutdown_event = threading.Event()
        self.fps_tracker = FPSTracker(target_fps=config.fps)
        self.observation_queue = Queue(maxsize=1)
        self._predicted_timesteps_lock = threading.Lock()
        self._predicted_timesteps = set()
        self.last_processed_obs = None

        # Filled by SendPolicyInstructions; no policy is ever loaded.
        self.lerobot_features = None
        self.policy = None
        self.preprocessor = None
        self.postprocessor = None
        self.device = "cpu"
        self.policy_type = "dummy"
        self.actions_per_chunk = config.actions_per_chunk
        self._column_order = None
        self._exhausted_logged = False

        self.actions = build_table(config.source, config.fps, config.scale)
        if config.source == "episode":
            self.logger.info(
                f"Source: hardcoded_episode.py | {hardcoded_episode.SOURCE} episode "
                f"{hardcoded_episode.EPISODE} | task {hardcoded_episode.TASK!r}"
            )
        else:
            self.logger.info(f"Source: WAYPOINTS ({len(WAYPOINTS)} waypoints, hand-authored)")
        if config.scale != 1.0:
            self.logger.warning(f"--scale={config.scale}: every row shrunk towards row 0, this is NOT the recording")
        check_speed(self.actions, config.fps, config.max_joint_vel, config.warn_joint_vel, self.logger)

    @property
    def policy_image_features(self):  # never used: observations are ignored
        return {}

    def SendPolicyInstructions(self, request, context):  # noqa: N802
        """Record the client's feature spec. No policy is loaded (there is none)."""
        if not self.running:
            self.logger.warning("Server is not running. Ignoring policy instructions.")
            return services_pb2.Empty()

        policy_specs = pickle.loads(request.data)  # nosec
        if not isinstance(policy_specs, RemotePolicyConfig):
            raise TypeError(f"Policy specs must be a RemotePolicyConfig. Got {type(policy_specs)}")

        self.lerobot_features = policy_specs.lerobot_features
        self._column_order = self._resolve_column_order()

        if policy_specs.actions_per_chunk > self.actions_per_chunk:
            self.logger.warning(
                f"Client asked for {policy_specs.actions_per_chunk} actions per chunk but this server "
                f"serves {self.actions_per_chunk}. Raise --actions_per_chunk to match."
            )

        self.logger.info(
            f"Client {context.peer()} ready | requested policy '{policy_specs.policy_type}' "
            f"({policy_specs.pretrained_name_or_path}) is IGNORED - serving the hardcoded table "
            f"({len(self.actions)} rows, {len(self.actions) / self.config.fps:.1f}s at {self.config.fps} fps)"
        )
        return services_pb2.Empty()

    def _resolve_column_order(self):
        """Map our table columns onto the order the client will unpack them in."""
        try:
            client_names = list(self.lerobot_features["observation.state"]["names"])
        except (KeyError, TypeError):
            self.logger.warning("Client advertised no observation.state names; assuming %s", ACTION_NAMES)
            return None

        if client_names == ACTION_NAMES:
            return None
        if sorted(client_names) == sorted(ACTION_NAMES):
            self.logger.warning(f"Re-ordering table columns to the client's order: {client_names}")
            return [ACTION_NAMES.index(n) for n in client_names]
        raise ValueError(
            f"Client expects action keys {client_names}, this table provides {ACTION_NAMES}. "
            f"Wrong robot type, or ARM_JOINTS is out of sync with config_teleop_mantis.yaml."
        )

    def _obs_sanity_checks(self, obs: TimedObservation, previous_obs: TimedObservation) -> bool:
        """Only reject already-answered timesteps.

        The parent also drops observations whose joint-space distance to the last processed
        one is under 1 (`observations_similar`). That threshold is tuned for SO-10x degrees;
        on a UR5 in radians a whole stroke is well under 1, so it would drop nearly everything.
        The table is a pure function of the timestep anyway, so this check is all we need.
        """
        with self._predicted_timesteps_lock:
            already_answered = obs.get_timestep() in self._predicted_timesteps
        if already_answered:
            self.logger.debug(f"Skipping observation #{obs.get_timestep()} - timestep already answered")
        return not already_answered

    def _row(self, timestep: int) -> np.ndarray:
        """Absolute target for an action timestep, holding (or looping) past the end."""
        if timestep < len(self.actions):
            return self.actions[timestep]
        if self.config.loop:
            return self.actions[timestep % len(self.actions)]
        if not self._exhausted_logged:
            self._exhausted_logged = True
            self.logger.info(
                f"Trajectory finished at timestep {len(self.actions) - 1}; holding the last row. "
                f"Ctrl-C the client to stop."
            )
        return self.actions[-1]

    def _predict_action_chunk(self, observation_t: TimedObservation) -> list:
        """Rows [t, t + actions_per_chunk) of the table, as the client's TimedActions."""
        t_0 = observation_t.get_timestep()
        rows = np.stack([self._row(t_0 + i) for i in range(self.actions_per_chunk)])
        if self._column_order is not None:
            rows = rows[:, self._column_order]

        self.last_processed_obs = observation_t
        # CPU tensors: the client may run on a machine without CUDA.
        actions = [torch.from_numpy(np.ascontiguousarray(row)) for row in rows]

        self.logger.info(
            f"Observation #{t_0} -> rows {t_0}:{t_0 + self.actions_per_chunk} | "
            f"first={np.round(rows[0], 4).tolist()}"
        )
        return self._time_action_chunk(observation_t.get_timestamp(), actions, t_0)


@draccus.wrap()
def serve(cfg: DummyPolicyServerConfig):
    logging.info(pformat(asdict(cfg)))
    policy_server = DummyPolicyServer(cfg)

    if cfg.dry_run:
        header = "  ".join(f"{n:>28}" for n in ACTION_NAMES)
        print(f"{'step':>6}  {'t(s)':>7}  {header}")
        for i, row in enumerate(policy_server.actions):
            values = "  ".join(f"{v:>28.5f}" for v in row)
            print(f"{i:>6}  {i / cfg.fps:>7.2f}  {values}")
        return

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    services_pb2_grpc.add_AsyncInferenceServicer_to_server(policy_server, server)
    if server.add_insecure_port(f"{cfg.host}:{cfg.port}") == 0:
        raise OSError(f"could not bind {cfg.host}:{cfg.port}")
    policy_server.logger.info(f"DummyPolicyServer listening on {cfg.host}:{cfg.port}")
    server.start()
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        policy_server.logger.info("Interrupted")
    policy_server.logger.info("Server terminated")


if __name__ == "__main__":
    serve()
