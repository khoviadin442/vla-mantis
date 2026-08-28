"""Smoke-test `dummy_policy_server` without a robot.

Speaks the exact wire protocol `lerobot.async_inference.robot_client` speaks (Ready ->
SendPolicyInstructions -> SendObservations/GetActions) and models its control loop
single-threaded, then checks that the sequence of executed actions is byte-for-byte the
server's table. Catches column-order, chunk-indexing and aggregation bugs before the arm
is ever powered.

Run (inside the mantis container, server already listening):
```shell
python3 dummy_policy_server_smoketest.py --steps=120 --chunk_size_threshold=0.5
```
"""

import argparse
import pickle  # nosec
import time

import numpy as np
import torch

from lerobot.async_inference.configs import get_aggregate_function
from lerobot.async_inference.helpers import RemotePolicyConfig, TimedObservation
from lerobot.transport import services_pb2, services_pb2_grpc
from lerobot.transport.utils import send_bytes_in_chunks

from dummy_policy_server import ACTION_NAMES, build_table

import grpc


def lerobot_features() -> dict:
    """What `map_robot_keys_to_lerobot_features(MantisFollower)` produces on the real client."""
    return {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(ACTION_NAMES),),
            "names": list(ACTION_NAMES),
        }
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--server_address", default="127.0.0.1:8080")
    p.add_argument("--steps", type=int, default=120, help="actions to execute")
    p.add_argument("--fps", type=int, default=15, help="must match the server's --fps")
    p.add_argument("--actions_per_chunk", type=int, default=50)
    p.add_argument("--chunk_size_threshold", type=float, default=0.5)
    p.add_argument("--aggregate_fn_name", default="weighted_average")
    p.add_argument("--source", default="episode", help="must match the server's --source")
    p.add_argument("--scale", type=float, default=1.0, help="must match the server's --scale")
    p.add_argument("--realtime", action="store_true", help="sleep 1/fps between steps")
    args = p.parse_args()

    aggregate_fn = get_aggregate_function(args.aggregate_fn_name)
    channel = grpc.insecure_channel(args.server_address)
    stub = services_pb2_grpc.AsyncInferenceStub(channel)

    stub.Ready(services_pb2.Empty())
    stub.SendPolicyInstructions(
        services_pb2.PolicySetup(
            data=pickle.dumps(
                RemotePolicyConfig("dummy", "dummy", lerobot_features(), args.actions_per_chunk, "cpu")
            )
        )
    )
    print(f"Handshake OK with {args.server_address}")

    queue: dict[int, torch.Tensor] = {}  # timestep -> action, mirrors the client's action_queue
    latest_action = -1
    executed: list[np.ndarray] = []
    chunks_received = 0

    while len(executed) < args.steps:
        # (1) perform the next queued action
        ready = [ts for ts in queue if ts > latest_action]
        if ready:
            timestep = min(ready)
            executed.append(queue.pop(timestep).numpy().copy())
            latest_action = timestep

        # (2) stream an observation when the queue has drained past the threshold
        if len(queue) / max(args.actions_per_chunk, 1) <= args.chunk_size_threshold:
            observation = TimedObservation(
                timestamp=time.time(),
                timestep=max(latest_action, 0),
                # Content is irrelevant to this server, but shaped like the real robot's.
                observation={**{k: 0.0 for k in ACTION_NAMES}, "task": "dummy"},
                must_go=not queue,
            )
            stub.SendObservations(send_bytes_in_chunks(pickle.dumps(observation), services_pb2.Observation))
            response = stub.GetActions(services_pb2.Empty())
            if response.data:
                chunks_received += 1
                for timed_action in pickle.loads(response.data):  # nosec
                    timestep = timed_action.get_timestep()
                    if timestep <= latest_action:
                        continue  # stale, exactly as the client drops it
                    action = timed_action.get_action()
                    queue[timestep] = aggregate_fn(queue[timestep], action) if timestep in queue else action
        if args.realtime:
            time.sleep(1.0 / args.fps)

    channel.close()

    # ── Verify against the table the server built from the same WAYPOINTS ─────
    expected_table = build_table(args.source, args.fps, args.scale)
    expected = np.stack([expected_table[min(i, len(expected_table) - 1)] for i in range(len(executed))])
    got = np.stack(executed)
    max_error = float(np.abs(got - expected).max())

    print(f"Executed {len(executed)} actions from {chunks_received} chunks | max |error| = {max_error:.3e}")
    print(f"  first executed: {np.round(got[0], 5).tolist()}")
    print(f"  last  executed: {np.round(got[-1], 5).tolist()}")
    largest_step = float(np.abs(np.diff(got[:, :6], axis=0)).max())
    print(f"  largest joint step between consecutive actions: {largest_step:.5f} rad ({largest_step * args.fps:.3f} rad/s)")

    if max_error > 1e-6:
        raise SystemExit(f"FAIL: executed trajectory differs from the server table by {max_error}")
    print("PASS: the executed trajectory is exactly the hardcoded table.")


if __name__ == "__main__":
    main()
