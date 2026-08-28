"""Write meta/intrinsics.json and meta/extrinsics.json into existing LeRobot datasets.

Intrinsics come from a capture of the live camera_info topics; extrinsics from the
fixed-cameras calibration file. Same layout the recorder now writes for new datasets.
"""
import json
import sys
from pathlib import Path

import yaml

ROOT = Path("/home/ros/share/lerobot_data")
EXTRINSICS_FILE = Path("/home/ros/share/mantis_ws/src/prl_ur5_robot_configuration/config/fixed_cameras/dataset_collection.yaml")
LIVE_INTRINSICS = Path("/home/ros/share/live_intrinsics.json")
TOPICS = {
    "left": "/camera/echo_camera/color/image_raw",
    "right": "/camera/foxtrot_camera/color/image_raw",
}


def camera_prefix(topic):
    for part in topic.strip("/").split("/"):
        if part.endswith("_camera"):
            return part[: -len("_camera")]
    return None


def info_topic(topic):
    return topic.rsplit("/", 1)[0] + "/camera_info"


def load_extrinsics():
    out = {}
    for e in yaml.safe_load(EXTRINSICS_FILE.read_text()) or []:
        if isinstance(e, dict) and "name_prefix" in e:
            out[str(e["name_prefix"])] = {
                "type": e.get("type"),
                "pose": e.get("pose"),
                "offset": e.get("offset"),
                "fixture_orientation": e.get("fixture_orientation"),
            }
    return out


def build(features):
    """(intrinsics, extrinsics) payloads, one entry per camera present in the dataset.

    Depth is registered to its colour frame, so it shares that camera's entry.
    """
    live = json.loads(LIVE_INTRINSICS.read_text())
    extr = load_extrinsics()
    intr_out, extr_out = {}, {}
    for key, topic in TOPICS.items():
        if f"observation.images.{key}" not in features:
            continue
        prefix = camera_prefix(topic)
        intr_out[key] = {"camera": prefix, "topic": topic,
                         "camera_info_topic": info_topic(topic), **live[key]}
        extr_out[key] = {"camera": prefix, **(extr.get(prefix) or {})}
    return ({"cameras": intr_out},
            {"source": str(EXTRINSICS_FILE),
             "frame": "robot base (as configured in the fixed-cameras file)",
             "cameras": extr_out})


def main():
    datasets = sorted(p for p in ROOT.iterdir() if (p / "meta" / "info.json").is_file())
    for ds in datasets:
        info = json.loads((ds / "meta" / "info.json").read_text())
        features = set(info.get("features", {}))
        intr, extr = build(features)
        missing = []
        for fname, payload in (("intrinsics.json", intr), ("extrinsics.json", extr)):
            (ds / "meta" / fname).write_text(json.dumps(payload, indent=2) + "\n")
        print(f"  {ds.name}: {len(intr['cameras'])} cameras written"
              + (f"  UNMAPPED FEATURES: {missing}" if missing else ""))
    print(f"done: {len(datasets)} datasets")


if __name__ == "__main__":
    sys.exit(main())
