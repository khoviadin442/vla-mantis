"""Same observation/action contract as MantisFollower, no ROS. Logs every executed action."""
import csv
import os
import time
from dataclasses import dataclass

from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot

ARM = ["left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_joint",
       "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"]
KEYS = [f"{j}.pos" for j in ARM] + ["gripper.pos"]
HOME = [0.0, -1.91986, 1.15192, -1.5708, -1.5708, 0.0, 0.11]


@RobotConfig.register_subclass("mantis_sim")
@dataclass
class MantisSimConfig(RobotConfig):
    out_csv: str = "executed_actions.csv"


class MantisSim(Robot):
    config_class = MantisSimConfig
    name = "mantis_sim"

    def __init__(self, config):
        super().__init__(config)
        self.config = config
        self._state = dict(zip(KEYS, HOME))
        self._connected = False
        self._f = None
        self._w = None

    @property
    def observation_features(self):
        return {k: float for k in KEYS}

    @property
    def action_features(self):
        return dict(self.observation_features)

    @property
    def is_connected(self):
        return self._connected

    @property
    def is_calibrated(self):
        return True

    def calibrate(self):
        pass

    def configure(self):
        pass

    def connect(self, calibrate: bool = True):
        self._f = open(self.config.out_csv, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["wall_time"] + KEYS)
        self._connected = True

    def disconnect(self):
        self._connected = False
        if self._f:
            self._f.close()
            self._f = None

    def get_observation(self):
        return dict(self._state)

    def send_action(self, action):
        self._state = {k: float(action[k]) for k in KEYS}
        self._w.writerow([f"{time.time():.6f}"] + [f"{self._state[k]:.9f}" for k in KEYS])
        self._f.flush()
        return dict(action)
