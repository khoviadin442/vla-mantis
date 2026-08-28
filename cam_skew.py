"""Measure the header-stamp offset between the two colour cameras."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

TOPICS = {"left": "/camera/echo_camera/color/image_raw",
          "right": "/camera/foxtrot_camera/color/image_raw"}

rclpy.init()
n = Node("cam_skew")
last = {}
pairs = []
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


def stamp(m):
    return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec


def cb(k):
    def f(m):
        last[k] = stamp(m)
        if len(last) == 2:
            pairs.append((last["right"] - last["left"]) * 1000.0)
    return f


for k, t in TOPICS.items():
    n.create_subscription(Image, t, cb(k), qos)
t0 = time.time()
while time.time() - t0 < 10 and len(pairs) < 60:
    rclpy.spin_once(n, timeout_sec=0.2)
if pairs:
    p = sorted(abs(x) for x in pairs)
    print("  samples=%d  |skew| p50=%.1f ms  p95=%.1f ms  max=%.1f ms"
          % (len(p), p[len(p)//2], p[int(len(p)*0.95)], p[-1]))
    print("  one frame period at 15 fps = 66.7 ms")
else:
    print("  no paired frames seen")
rclpy.shutdown()
