"""Send ONE real observation to the policy server and print the chunk it returns.

Read-only dry run of the whole inference path: it subscribes to /joint_states and the camera,
builds the exact `lerobot_features` the robot client advertises, and does the same four RPCs
(Ready, SendPolicyInstructions, SendObservations, GetActions). It creates no publisher and
never calls send_action, so the arm cannot move - which makes it the safe way to prove that
the server accepts our feature spec, and that the returned tensors deserialize here, before
committing the real arm to a run.

Run (inside the mantis container, ROS sourced, tunnel up):
```shell
python3 probe_policy_server.py --task="Grab green cube and place it in the box"
```
"""

import argparse
import io
import pickle  # nosec
import time

import numpy as np
import torch

# Same shim as robot_client_cpu.py: the server pickles CUDA tensors. Must precede any unpickle.
torch.storage._load_from_bytes = lambda b: torch.load(io.BytesIO(b), map_location="cpu", weights_only=False)

import grpc  # noqa: E402
from lerobot.async_inference.helpers import RemotePolicyConfig, TimedObservation  # noqa: E402
from lerobot.utils.feature_utils import hw_to_dataset_features  # noqa: E402
from lerobot.transport import services_pb2, services_pb2_grpc  # noqa: E402
from lerobot.transport.utils import send_bytes_in_chunks  # noqa: E402


def parse_topics(spec: str) -> dict:
    """'{left: /a, right: /b}' -> {'left': '/a', 'right': '/b'}. Same shape draccus accepts."""
    spec = spec.strip().strip("{}").strip()
    if not spec:
        return {}
    return dict(part.split(":", 1) for part in spec.split(",")) and {
        k.strip(): v.strip() for k, v in (part.split(":", 1) for part in spec.split(","))
    }


def grab_observation(camera_topics: dict, timeout: float):
    """One frame per camera + one complete joint state, off the ROS graph. Subscriptions only."""
    import sys

    sys.path.insert(0, "/home/ros/share")
    import lerobot_recorder
    import rclpy
    import teleop_mantis as T
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, JointState

    to_rgb = lerobot_recorder.EpisodeRecorder._to_rgb
    rclpy.init()
    node = rclpy.create_node("policy_server_probe")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    state, frames = {}, {}
    node.create_subscription(JointState, T.JOINT_STATES_TOPIC,
                             lambda m: state.update(zip(m.name, m.position)), qos)
    for key, topic in camera_topics.items():
        node.create_subscription(Image, topic, (lambda k: lambda m: frames.__setitem__(k, m))(key), qos)

    deadline = time.time() + timeout
    warm_until = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if len(frames) == len(camera_topics) and all(j in state for j in T.ARM):
            # All streams are alive; let them run 2s so the skew reflects steady state
            # rather than the order the subscriptions happened to come up in.
            warm_until = warm_until or time.time() + 2.0
            if time.time() >= warm_until:
                break
    else:
        missing = [camera_topics[k] for k in camera_topics if k not in frames]
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f"within {timeout}s: joint_states incomplete and/or no frame on {missing}")

    joints = {f"{j}.pos": float(state[j]) for j in T.ARM}
    joints["gripper.pos"] = float(state.get(T.GRIP_JOINT, T.GRIP_OPEN))
    # Latest frame of each, taken independently - the same thing MantisFollower.get_observation
    # does. The femto-megas are hardware-synced, so the skew between them is sub-frame.
    stamps = {k: m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for k, m in frames.items()}
    images = {k: to_rgb(m) for k, m in frames.items()}
    node.destroy_node()
    rclpy.shutdown()
    if len(stamps) > 1:
        skew = (max(stamps.values()) - min(stamps.values())) * 1000
        print(f"Camera skew between {list(stamps)}: {skew:.1f} ms")
    return joints, images


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--server_address", default="127.0.0.1:8080")
    p.add_argument("--task", default="Grab green cube and place it in the box")
    p.add_argument("--policy_type", default="pi0")
    p.add_argument("--camera_topics",
                   default="{left: /camera/echo_camera/color/image_raw, "
                           "right: /camera/foxtrot_camera/color/image_raw}")
    p.add_argument("--actions_per_chunk", type=int, default=50)
    p.add_argument("--timeout", type=float, default=15.0)
    args = p.parse_args()

    topics = parse_topics(args.camera_topics)
    joints, images = grab_observation(topics, args.timeout)
    shown = ", ".join(f"{k} {v.shape}" for k, v in images.items())
    print(f"Observation: {len(joints)} joints + {shown}")

    # Exactly what map_robot_keys_to_lerobot_features(MantisFollower) produces.
    hw = {k: float for k in joints}
    for key, image in images.items():
        hw[key] = image.shape
    features = hw_to_dataset_features(hw, "observation", use_video=False)
    shapes = {k: v["shape"] for k, v in features.items()}
    print(f"Advertising: {shapes}")

    channel = grpc.insecure_channel(args.server_address)
    stub = services_pb2_grpc.AsyncInferenceStub(channel)
    stub.Ready(services_pb2.Empty())
    stub.SendPolicyInstructions(
        services_pb2.PolicySetup(
            data=pickle.dumps(
                RemotePolicyConfig(args.policy_type, "persistent", features, args.actions_per_chunk, "cuda")
            )
        )
    )
    print("Handshake OK")

    observation = TimedObservation(
        timestamp=time.time(), timestep=0, observation={**joints, **images, "task": args.task}, must_go=True
    )
    payload = pickle.dumps(observation)
    print(f"Sending observation: {len(payload) / 1e6:.2f} MB, task={args.task!r}")
    t0 = time.perf_counter()
    stub.SendObservations(send_bytes_in_chunks(payload, services_pb2.Observation))
    response = stub.GetActions(services_pb2.Empty())
    elapsed = time.perf_counter() - t0
    channel.close()

    if not response.data:
        raise SystemExit("Server returned no actions (observation filtered, or inference failed)")

    timed = pickle.loads(response.data)  # nosec
    chunk = np.stack([t.get_action().numpy() for t in timed])
    print(f"\nGot {len(timed)} actions in {elapsed * 1000:.0f} ms (round trip incl. inference)")
    print(f"  tensor device: {timed[0].get_action().device}  dtype: {timed[0].get_action().dtype}")
    print(f"  chunk shape:   {chunk.shape}")
    now = np.array([joints[k] for k in joints])
    print(f"  current pose:  {np.round(now, 4).tolist()}")
    print(f"  action[0]:     {np.round(chunk[0], 4).tolist()}")
    print(f"  action[-1]:    {np.round(chunk[-1], 4).tolist()}")
    print(f"  |action[0] - current| max: {np.abs(chunk[0] - now).max():.4f} rad "
          f"(> 0.05 triggers the slow collision-checked approach)")
    step = np.abs(np.diff(chunk[:, :6], axis=0))
    print(f"  peak joint speed within the chunk @15fps: {step.max() * 15:.2f} rad/s "
          f"(teleop shaper.out_vel = 1.5)")


if __name__ == "__main__":
    main()
