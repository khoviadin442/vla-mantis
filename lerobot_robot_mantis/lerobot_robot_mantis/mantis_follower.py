"""MantisFollower: a lerobot `Robot` that drives the mantis LEFT arm over ROS2.

Topics, joint names, gripper widths and shaper limits are read from the teleop's own
YAML by importing teleop_mantis, and commands go through the same CommandShaper the
teleop uses, so a replayed episode runs through the stage that recorded it. Beyond
pass-through: a 100 Hz feeder thread interpolates between the fps-rate waypoints of
lerobot-replay, a first action far from the measured pose is reached by a slow ramp,
and that approach path is checked against the teleop's collision floors first.

Homing (home_on_connect) does not refuse when the straight joint path to HOME_Q is
blocked: it searches for another one — staged joint groups, one joint at a time, a
deliberate step backwards, finally RRT-Connect — relaxes the clearance cushion only
if nothing routes at the full one, executes the result as a collision-checked
polyline, and replans from the measured pose until the arm is home.
"""
import importlib
import os
import sys
import threading
import time

import numpy as np

from lerobot.robots.robot import Robot

from .config_mantis_follower import MantisFollowerConfig

FEED_PERIOD_S = 0.01
DEFAULT_GAP_S = 1.0 / 15.0


class MantisFollower(Robot):
    config_class = MantisFollowerConfig
    name = "mantis_follower"

    def __init__(self, config: MantisFollowerConfig):
        super().__init__(config)
        self.config = config
        self._T = None
        self._node = None
        self._exe = None
        self._spin_thread = None
        self._feed_thread = None
        self._shaper = None
        self._shaper_started = False
        self._grip = None
        self._grip_sent = None
        self._pos = {}
        self._pos_lock = threading.Lock()
        self._parked = {}
        self._locked = {}
        self._first_action = True
        self._connected = False
        self._we_inited_rclpy = False
        self._feed_lock = threading.Lock()
        self._seg = None
        self._gap_ema = DEFAULT_GAP_S
        self._img_msg = {}
        self._img_lock = threading.Lock()
        self._img_shape = {}
        self._to_rgb = None
        self._last_send_t = None
        self._ik = None
        self._cmd_topic = None
        self._cmd_joints = None

    def _teleop(self):
        """Import teleop_mantis (loads the YAML next to it, or $teleop_config)."""
        if self._T is None:
            path = os.path.abspath(self.config.teleop_config)
            os.environ.setdefault("teleop_config", path)
            sys.path.insert(0, os.path.dirname(path))
            self._T = importlib.import_module("teleop_mantis")
        return self._T

    def _command_sink(self):
        """(topic, joint order) of the controller this run commands.

        Both come from the teleop YAML rather than from here, because for the safety filter
        they are not free: that controller claims the hardware position interfaces of its own
        `joints.active_joint` and reads the Float64MultiArray positionally, so our vector must
        be exactly its list in exactly its order. `safety_filter_joints` is that list, and the
        check below fails loudly at connect() rather than letting us stream a well-formed
        command that silently drives the wrong arm.
        """
        T = self._teleop()
        if not self.config.use_safety_filter:
            return T.ARM_CMD_TOPIC, list(T.ARM_CMD_JOINTS)
        joints = list(T.SAFETY_FILTER_JOINTS)
        missing = [j for j in T.ARM if j not in joints]
        if missing:
            raise RuntimeError(
                f"safety_filter_joints does not cover the driven joints {missing}. It must be "
                f"the safety filter controller's `joints.active_joint`, verbatim, and that "
                f"controller has to claim the arm being commanded ({list(T.ARM)}). Fix both "
                f"the controller's YAML and safety_filter_joints, or pass "
                f"--robot.use_safety_filter=false to command "
                f"{T.ARM_CMD_TOPIC} unfiltered instead.")
        return T.ARM_CMD_SAFETY_TOPIC, joints

    def _joint_features(self) -> dict:
        T = self._teleop()
        ft = {f"{j}.pos": float for j in T.ARM}
        ft["gripper.pos"] = float
        return ft

    @property
    def observation_features(self) -> dict:
        """Joints plus one (h, w, 3) entry per camera.

        `hw_to_dataset_features` turns each tuple-valued entry into
        `observation.images.<key>`, so these keys are the contract with the policy: they must
        be the camera names the checkpoint was trained on. Shapes are only known once a frame
        has arrived, i.e. after connect().
        """
        ft = self._joint_features()
        for key in self.config.camera_topics:
            if key not in self._img_shape:
                raise RuntimeError(f"camera {key!r} has no frame yet - call connect() first")
            ft[key] = self._img_shape[key]
        return ft

    @property
    def action_features(self) -> dict:
        return self._joint_features()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Float64MultiArray
        from rclpy.action import ActionClient
        from control_msgs.action import GripperCommand

        T = self._teleop()
        if not rclpy.ok():
            rclpy.init()
            self._we_inited_rclpy = True
        self._node = rclpy.create_node("mantis_follower")
        qos_js = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self._node.create_subscription(JointState, T.JOINT_STATES_TOPIC, self._on_js, qos_js)
        # Same decode the recorder used, imported rather than copied so inference cannot drift
        # from training on encoding or stride handling. _teleop() put its directory on sys.path.
        import lerobot_recorder
        self._to_rgb = lerobot_recorder.EpisodeRecorder._to_rgb
        qos_img = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        for key, topic in self.config.camera_topics.items():
            self._node.create_subscription(
                Image, topic, (lambda k: lambda m: self._on_image(k, m))(key), qos_img)
        self._cmd_topic, self._cmd_joints = self._command_sink()
        pub = self._node.create_publisher(Float64MultiArray, self._cmd_topic, 10)
        self._shaper = T.CommandShaper(pub, T.OUT_RATE or 250.0, T.OUT_VEL, T.OUT_ACCEL, T.OUT_KP)
        self._grip = ActionClient(self._node, GripperCommand, T.GRIP_ACTION)

        self._exe = rclpy.executors.SingleThreadedExecutor()
        self._exe.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._exe.spin, daemon=True)
        self._spin_thread.start()

        t0 = time.monotonic()
        while time.monotonic() - t0 < self.config.connect_timeout:
            with self._pos_lock:
                if all(j in self._pos for j in T.ARM_CMD_JOINTS):
                    break
            time.sleep(0.05)
        else:
            self.disconnect()
            raise TimeoutError(
                f"no complete {T.JOINT_STATES_TOPIC} within {self.config.connect_timeout}s "
                f"— is the robot stack up?")
        missing = self._wait_for_frames()
        if missing:
            self.disconnect()
            raise TimeoutError(
                f"no frame on {[self.config.camera_topics[k] for k in missing]} within "
                f"{self.config.camera_timeout}s - are the cameras publishing?")
        with self._pos_lock:
            # _parked fills the command vector, so it covers only what we actually publish.
            # _locked is the collision model's view of the rest of the robot and must cover every
            # non-driven joint: PinkIK pins ALL of them, and any it is not given a value for sits
            # at the URDF neutral pose - i.e. the other arm folded somewhere it is not.
            self._parked = {j: float(self._pos[j]) for j in self._cmd_joints if j not in T.ARM}
            self._locked = {j: float(self._pos[j]) for j in T.ARM_CMD_JOINTS if j not in T.ARM}
        self._shaper.start()
        self._shaper_started = True
        self._feed_thread = threading.Thread(target=self._feeder, daemon=True)
        self._connected = True
        self._feed_thread.start()
        self._node.get_logger().info(
            f"mantis_follower connected: cmd {self._cmd_topic} @ {T.OUT_RATE:.0f} Hz "
            f"({'SAFETY FILTER' if self.config.use_safety_filter else 'UNFILTERED'}, "
            f"{len(self._cmd_joints)} joints, {len(self._parked)} latched parked). "
            f"Nothing moves unless that controller is the active one.")
        if self.config.home_on_connect:
            self._go_home()

    def disconnect(self) -> None:
        if self._connected and self._shaper is not None:
            with self._feed_lock:
                seg = self._seg
            if seg is not None and seg[3] <= 0.3:
                t0 = time.monotonic()
                while time.monotonic() - t0 < 0.5:
                    with self._feed_lock:
                        if self._segment_value(time.monotonic())[1] >= 1.0:
                            break
                    time.sleep(0.02)
                time.sleep(0.15)
        self._connected = False
        if self._shaper is not None:
            if self._shaper_started:
                self._shaper.stop()
                self._shaper_started = False
            self._shaper = None
        if self._exe is not None:
            self._exe.shutdown()
            self._exe = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._we_inited_rclpy:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
            self._we_inited_rclpy = False

    def _on_js(self, msg):
        with self._pos_lock:
            self._pos.update(zip(msg.name, msg.position))

    def _on_image(self, key, msg):
        with self._img_lock:
            self._img_msg[key] = msg

    def _wait_for_frames(self) -> list:
        """Block until every camera has produced one frame; returns the keys still missing."""
        t0 = time.monotonic()
        while True:
            with self._img_lock:
                missing = [k for k in self.config.camera_topics if k not in self._img_msg]
            if not missing or time.monotonic() - t0 > self.config.camera_timeout:
                break
            time.sleep(0.05)
        with self._img_lock:
            self._img_shape = {k: self._to_rgb(m).shape for k, m in self._img_msg.items()}
        return missing

    def _segment_value(self, now):
        """(current point, progress s) of the active segment. Caller holds _feed_lock."""
        q_from, q_to, t0, T = self._seg
        s = 1.0 if T <= 0.0 else min(1.0, (now - t0) / T)
        return q_from + s * (q_to - q_from), s

    def _feeder(self):
        """100 Hz continuous-target feed (see module doc: judder fix + keepalive)."""
        while self._connected:
            with self._feed_lock:
                cur = self._segment_value(time.monotonic())[0] if self._seg is not None else None
            if cur is not None:
                self._set_target(cur)
            time.sleep(FEED_PERIOD_S)

    def _set_segment(self, q_from, q_to, T):
        with self._feed_lock:
            self._seg = (np.asarray(q_from, float), np.asarray(q_to, float),
                         time.monotonic(), float(T))

    def _wait_segment(self, deadline=None, gate_min=None, gate_scale=1.0):
        """Block until the active segment completes (the long approach, and every home leg).

        Returns 'done', 'timeout', 'disconnected', or 'gate' — the MEASURED pose crossed
        the safety gate mid-segment, in which case the target is frozen where it is rather
        than left running at a waypoint the arm is no longer on its way to. The gate is the
        same live check the teleop keeps during its own home ramp: the plan certified the
        COMMANDED path, this certifies the one the controller is actually producing."""
        t_gate = 0.0
        while self._connected:
            with self._feed_lock:
                s = self._segment_value(time.monotonic())[1] if self._seg is not None else 1.0
            if s >= 1.0:
                return "done"
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                self._freeze_feed()
                return "timeout"
            if gate_min is not None and now - t_gate >= 0.05:
                t_gate = now
                m, pair = self._margin(self._meas(), gate_scale)
                if m < gate_min:
                    self._freeze_feed()
                    self._node.get_logger().warn(
                        f"HOME: stopped mid-segment — the measured pose is "
                        f"{1000.0 * (gate_min - m):.0f} mm past the {gate_scale:.2f}x "
                        f"gate of {pair}")
                    return "gate"
            time.sleep(0.02)
        return "disconnected"

    def _set_target(self, q_arm):
        T = self._T
        full = []
        m = dict(zip(T.ARM, q_arm))
        for j in self._cmd_joints:
            if j in m:
                full.append(float(m[j]))
            elif j in self._parked:
                full.append(self._parked[j])
            else:
                raise RuntimeError(f"joint {j} neither driven nor latched")
        self._shaper.set_target(np.asarray(full, float))

    def get_observation(self) -> dict:
        T = self._teleop()
        with self._pos_lock:
            pos = dict(self._pos)
        obs = {f"{j}.pos": float(pos[j]) for j in T.ARM}
        obs["gripper.pos"] = float(pos.get(T.GRIP_JOINT, self._grip_sent
                                           if self._grip_sent is not None else T.GRIP_OPEN))
        if self.config.camera_topics:
            with self._img_lock:
                msgs = dict(self._img_msg)
            now = self._node.get_clock().now().nanoseconds * 1e-9
            for key in self.config.camera_topics:
                msg = msgs.get(key)
                if msg is None:
                    raise RuntimeError(f"camera {key!r} stopped publishing")
                age = now - (msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
                if age > self.config.camera_stale_s:
                    self._node.get_logger().warn(
                        f"camera {key!r} frame is {age:.2f}s old - the policy is seeing the past",
                        throttle_duration_sec=2.0)
                obs[key] = self._to_rgb(msg)
        return obs

    def send_action(self, action: dict) -> dict:
        T = self._teleop()
        q = np.array([float(action[f"{j}.pos"]) for j in T.ARM], float)
        now = time.monotonic()
        if self._first_action:
            self._first_action = False
            meas = self._meas()
            gap = float(np.max(np.abs(q - meas)))
            if gap > self.config.approach_tol:
                self._path_check(meas, q)
                v = min(self.config.approach_vel, 0.9 * T.OUT_VEL,
                        0.5 * T.MAX_JOINT_LEAD * T.OUT_KP)
                self._node.get_logger().info(
                    f"pre-positioning to the first waypoint ({gap:.2f} rad away, "
                    f"{gap / v:.1f}s at {v:.2f} rad/s, path collision-checked)")
                self._set_segment(meas, q, gap / max(v, 1e-6))
                self._wait_segment()
            else:
                self._set_segment(meas, q, max(gap / 0.5, 0.02))
        else:
            if self._last_send_t is not None:
                g = min(max(now - self._last_send_t, 0.02), 0.3)
                self._gap_ema += 0.3 * (g - self._gap_ema)
            with self._feed_lock:
                cur = self._segment_value(now)[0] if self._seg is not None else q
            self._set_segment(cur, q, self._gap_ema)
        self._last_send_t = now
        if "gripper.pos" in action:
            g = float(action["gripper.pos"])
            if self._grip_sent is None or abs(g - self._grip_sent) > self.config.gripper_tol:
                self._send_grip(g)
        return dict(action)

    # ------------------------------------------------------------------ homing
    def _cfg(self, name, default):
        """A homing knob, read through getattr so this file stays drop-in against a
        config_mantis_follower.py that has not grown the field yet. Adding the field to
        that dataclass is what makes it settable from the CLI (--robot.home_max_attempts=6)."""
        v = getattr(self.config, name, None)
        return default if v is None else v

    def _meas(self):
        """Measured arm pose, in ARM order."""
        T = self._T
        with self._pos_lock:
            return np.array([self._pos[j] for j in T.ARM], float)

    def _go_home(self):
        """Bring the arm to the teleop's HOME_Q, planning around whatever is in the way.

        The straight joint ramp is still what runs whenever it is clear: same pose, same
        speed cap, same pre-check as the teleop's HOME button, so the ordinary case is
        bit-for-bit the motion this file always did. What changed is the other case.
        Refusing to move ("move the arm clear first") left the one situation that most
        needs homing — an aborted episode with the gripper parked under the camera arch
        or against the table — to be untangled by hand, and a straight line through the
        6-dimensional joint space is a very poor test of whether the arm CAN go home: it
        is one path out of infinitely many, and it is the one most likely to sweep the
        wrist across the table on the way.

        So a blocked straight path now starts a search (_plan_home) for another one, the
        plan is executed as a checked polyline, and the whole thing is a RETRY LOOP: after
        every attempt the arm is measured again and, if it is not home, replanned from
        where it actually ended up. That matters because the reasons a plan under-delivers
        (the measured pose drifting off the commanded one, the safety gate stopping a
        segment, a partial plan that only got closer) all change the problem, and the next
        search sees the new one.

        Returns True when the measured pose is inside HOME_DONE_TOL. Raises only after the
        search is genuinely exhausted — with the arm left at the best pose it reached, not
        wherever it started.
        """
        T = self._T
        log = self._node.get_logger()
        home = np.asarray(T.HOME_Q, float)
        gap = float(np.max(np.abs(home - self._meas())))
        if gap <= self.config.approach_tol:
            log.info(f"HOME: already there ({gap:.3f} rad), no ramp needed")
            return True
        self._ensure_ik()
        for j, v in zip(T.ARM, home):
            qi = self._ik.qindex(j)
            if not (self._ik.model.lowerPositionLimit[qi] - 1e-6 <= v
                    <= self._ik.model.upperPositionLimit[qi] + 1e-6):
                raise RuntimeError(
                    f"HOME impossible by construction: teleop.home_q_deg puts {j} at "
                    f"{v:+.3f} rad, outside its limits — no path exists, fix the config")
        # The ramp must stay within what the shaper can FOLLOW: past resync_snap
        # (max_joint_lead) it publishes one unlimited sample straight into servoj.
        v = min(T.HOME_VEL, 0.9 * T.OUT_VEL, 0.5 * T.MAX_JOINT_LEAD * T.OUT_KP)
        self._send_grip(T.GRIP_OPEN)
        n_scales = len(list(self._cfg("home_floor_scales", (1.0, 0.5, 0.0))))
        t_end = time.monotonic() + float(self._cfg("home_total_time_s", 120.0))
        attempts = max(1, int(self._cfg("home_max_attempts", 4)))
        plan_budget = float(self._cfg("home_plan_time_s", 8.0))
        settle_tol = float(self._cfg("home_settle_tol", 3.0 * T.HOME_DONE_TOL))
        budget = plan_budget
        min_scale, prev_gap, frozen, tried = 0, None, 0, []
        for attempt in range(1, attempts + 1):
            if not self._connected:
                return False
            meas = self._meas()
            gap = float(np.max(np.abs(home - meas)))
            if gap <= T.HOME_DONE_TOL:
                log.info(f"HOME reached ({1000.0 * gap:.0f} mrad max error)")
                return True
            if prev_gap is not None and gap > prev_gap - 0.02:
                # The last attempt bought nothing. Doing the same search again would
                # find the same nothing, so drop a cushion tier before re-searching.
                min_scale = min(min_scale + 1, n_scales - 1)
            prev_gap = gap
            if time.monotonic() >= t_end:
                tried.append(f"attempt {attempt}: out of the {self._cfg('home_total_time_s', 120.0):.0f}s budget")
                break
            plan = self._plan_home(meas, home, min(t_end, time.monotonic() + budget),
                                   min_scale=min_scale, seed=attempt)
            if plan is None:
                tried.append(f"attempt {attempt}: no path found from {np.degrees(meas).round(0)} deg")
                log.warn(f"HOME: no collision-free path found in {budget:.1f}s on attempt "
                         f"{attempt} (gap {gap:.2f} rad) — retrying with a relaxed cushion "
                         f"and twice the search")
                # A search that found nothing does not deserve the same budget twice. RRT
                # is the tier that pays for time, and it is the tier that was cut off.
                budget = min(2.0 * budget, max(1.0, t_end - time.monotonic()))
                min_scale = min(min_scale + 1, n_scales - 1)
                continue
            path, scale, how, complete = plan
            span = float(np.max(np.abs(np.asarray(path[-1], float) - meas)))
            if scale < 1.0 or not complete or how != "direct":
                log.warn(
                    f"HOME: the straight path is blocked — going via {how}, "
                    f"{len(path)} waypoint(s), {span:.2f} rad, {span / v:.0f}s at {v:.2f} rad/s"
                    + ("" if complete else ", PARTIAL (gets closer, then replans)")
                    + ("" if scale >= 1.0 else f", clearance floors relaxed to {scale:.2f}x"))
            else:
                log.info(f"HOME: {span:.2f} rad away, {span / v:.1f}s at {v:.2f} rad/s, "
                         f"path collision-checked")
            status = self._run_path(path, v, t_end, scale)
            arrived, err = self._settle(
                home, T.HOME_SETTLE_GRACE if complete else 1.0, T.HOME_DONE_TOL)
            if arrived:
                log.info(f"HOME reached ({1000.0 * err:.0f} mrad max error)")
                return True
            if complete and status == "done" and err <= settle_tol:
                # The plan ran to the end and the arm is a servo offset away, not an
                # obstruction away. Replanning a 50 mrad gap finds the same straight line
                # and commands it again; this is the pre-planner warning, kept verbatim,
                # because a stiff joint must not turn a working replay into a hard failure.
                log.warn(
                    f"HOME commanded but the measured pose is still {err:.3f} rad off after "
                    f"{T.HOME_SETTLE_GRACE:.1f}s - the arm is blocked, or the controller you "
                    f"are commanding is not the active one")
                return False
            moved = float(np.max(np.abs(self._meas() - meas)))
            tried.append(f"attempt {attempt}: {how} at {scale:.2f}x floors -> {status}, "
                         f"moved {moved:.2f} rad, still {err:.2f} rad off")
            if moved < 0.02:
                # Nothing on the arm answered a fully published, collision-checked ramp.
                # No amount of replanning fixes a controller that is not listening.
                frozen += 1
                log.warn(f"HOME: the command went out but the arm did not move "
                         f"({moved:.3f} rad) — is the controller you are commanding "
                         f"({self._cmd_topic}) the active one?")
                if frozen >= 2:
                    raise RuntimeError(
                        f"HOME failed: two full ramps published on {self._cmd_topic} and the "
                        f"measured pose never moved. The arm is not being driven — activate "
                        f"that controller (or set --robot.use_safety_filter accordingly) "
                        f"and rerun.")
            elif status == "gate":
                log.warn("HOME: the measured pose entered the safety gate mid-segment — "
                         "stopped and replanning from where the arm actually is")
        raise RuntimeError(
            "HOME failed: the arm is not at home and every route the planner could find "
            "has been tried. It has been left where the last attempt ended (still "
            f"{float(np.max(np.abs(home - self._meas()))):.2f} rad from home). Attempts:\n  "
            + "\n  ".join(tried)
            + "\nRaise --robot.home_max_attempts / --robot.home_plan_time_s to search "
              "harder, or clear the obstruction and rerun.")

    # ------------------------------------------------------------------ home planner
    def _ensure_ik(self):
        """The teleop's own collision model (its floors, its hulls, the other arm latched
        at the pose it is really in), built once and shared by the path check and planner."""
        T = self._T
        if self._ik is None:
            self._node.get_logger().info(
                "building the collision model for the path check (~seconds)...")
            self._ik = T.PinkIK(T.URDF, T.EE_FRAME, T.ARM, srdf_path=T.srdf_path(),
                                package_dirs=T.mesh_pkg_dirs(), locked_q=dict(self._locked))
        return self._ik

    def _check_step(self):
        return max(float(getattr(self._T, "HOME_CHECK_STEP", np.radians(2.0))), 1e-3)

    def _q_full(self, q_arm):
        """Arm vector -> full model configuration (everything else at its latched pose)."""
        ik, T = self._ik, self._T
        q = ik.neutral()
        for j, x in zip(T.ARM, np.asarray(q_arm, float)):
            q[ik.qindex(j)] = float(x)
        return q

    def _margin(self, q_arm, floor_scale=1.0):
        """(min margin, pair) at one arm pose. inf when there is no collision geometry."""
        ik = self._ik
        if ik is None or ik.geom is None:
            return float("inf"), ""
        return ik.margin_at(self._q_full(q_arm), floor_scale)

    def _edge_margin(self, a, b, floor_scale=1.0, step=None, stop_below=None):
        """(worst margin, pair) over the straight joint segment a->b, endpoints included.

        The feeder interpolates linearly between waypoints, so this samples exactly the
        motion that will be executed. stop_below returns early on the first sample below
        it — the planner rejects far more edges than it accepts, and a rejected edge
        usually fails within a couple of samples."""
        ik = self._ik
        if ik is None or ik.geom is None:
            return float("inf"), ""
        a = np.asarray(a, float)
        d = np.asarray(b, float) - a
        n = int(np.ceil(float(np.max(np.abs(d))) / (self._check_step() if step is None else step))) + 1
        ss = np.linspace(0.0, 1.0, min(max(n, 2), 400))
        # Endpoint first: it is the sample most likely to be the bad one, and the whole
        # interior is wasted work when it is.
        order = np.concatenate(([1.0, 0.0], ss[1:-1])) if len(ss) > 2 else ss[::-1]
        worst, wpair = float("inf"), ""
        for s in order:
            m, pair = ik.margin_at(self._q_full(a + s * d), floor_scale)
            if m < worst:
                worst, wpair = m, pair
                if stop_below is not None and worst < stop_below:
                    break
        return worst, wpair

    def _plan_home(self, q0, home, deadline, min_scale=0, seed=0):
        """Search for a collision-free joint path q0 -> home.

        Returns (waypoints after q0, floor_scale, method, complete) or None. Tiers run
        cheapest and most predictable first, so an arm that only needs to fold its wrist
        out of the way does not get an RRT's random detour:
          direct  — the straight segment, i.e. what the teleop's HOME button does;
          staged  — one joint GROUP at a time (wrists home first, then the arm, ...);
          greedy  — one JOINT at a time, always taking the branch with the best clearance;
          detour  — back one joint out by 45/90/135 deg first, then re-try the above;
          rrt     — RRT-Connect over the arm joints, shortcut-smoothed.
        Every segment of every returned path is sampled through the same collision floors
        the barrier enforces, at the teleop's own home_check_step, so an accepted plan is
        certified motion rather than an approximation of it.

        FLOOR RELAXATION. Those floors are a safety cushion (d_min 15 mm, table 25 mm) on
        top of real contact, and they are what a blocked home is usually blocked by. When
        nothing routes at the full cushion the whole search repeats at a fraction of it,
        and finally at 0 — real geometric contact only. An arm parked in a corner is not
        safer than one that walks out with 8 mm of clearance instead of 25.

        NON-WORSENING. Whatever the scale, no pose on the path may be worse than the pose
        the arm is ALREADY in (floor_min below). That is what lets a start that is itself
        inside the cushion — exactly the case the old straight-path check refused, because
        s=0 already fails — be left at all, and lets it be left only in a direction that
        does not dig deeper.
        """
        if not bool(self._cfg("home_planner", True)):
            # Rollback knob: the straight path or nothing, i.e. the behaviour this file had
            # before the planner. _go_home still retries and still reports, it just has one
            # route to offer.
            got = self._plan_direct(q0, home, 1.0, 0.0, deadline, None)
            return None if got is None else (got[0], 1.0, got[1], True)
        scales = list(self._cfg("home_floor_scales", (1.0, 0.5, 0.0)))[min_scale:]
        rng = np.random.default_rng(seed)
        best_partial = None
        warned = False
        for scale in scales:
            m0, p0 = self._margin(q0, scale)
            mh, ph = self._margin(home, scale)
            floor_min = min(0.0, m0)
            if m0 < 0.0 and not warned:
                warned = True
                self._node.get_logger().warn(
                    f"HOME: the arm is ALREADY {1000.0 * -m0:.0f} mm inside the "
                    f"{scale:.2f}x floor of {p0} — planning a path that never goes deeper")
            if mh < floor_min:
                # Not the path's fault: home itself does not clear this cushion. Only a
                # relaxation can help, so do not burn the budget searching at this one.
                continue
            for planner in (self._plan_direct, self._plan_staged, self._plan_greedy,
                            self._plan_detour, self._plan_rrt):
                if time.monotonic() >= deadline:
                    break
                got = planner(q0, home, scale, floor_min, deadline, rng)
                if got is None:
                    continue
                path, how = got
                if float(np.max(np.abs(np.asarray(path[-1], float) - home))) <= 1e-6:
                    return [np.asarray(w, float) for w in path], scale, how, True
                # A partial: the tier got closer but not there. Keep the best one and go on
                # — a complete path at a more relaxed cushion beats a partial at a strict one.
                gain = (float(np.max(np.abs(home - q0)))
                        - float(np.max(np.abs(home - np.asarray(path[-1], float)))))
                if gain > 0.1 and (best_partial is None or gain > best_partial[0]):
                    best_partial = (gain, [np.asarray(w, float) for w in path], scale, how)
        if best_partial is not None and bool(self._cfg("home_allow_partial", True)):
            _, path, scale, how = best_partial
            return path, scale, how, False
        return None

    def _plan_direct(self, q0, home, scale, floor_min, deadline, rng):
        w, _ = self._edge_margin(q0, home, scale, stop_below=floor_min)
        return ([np.asarray(home, float)], "direct") if w >= floor_min else None

    def _groups(self):
        """Joint groups by name, so this survives a different `arm:` list in the YAML."""
        T = self._T
        wrist = [i for i, j in enumerate(T.ARM) if "wrist" in j]
        pan = [i for i, j in enumerate(T.ARM) if "pan" in j]
        upper = [i for i in range(len(T.ARM)) if i not in wrist and i not in pan]
        return wrist, pan, upper

    def _plan_staged(self, q0, home, scale, floor_min, deadline, rng):
        """Two-segment paths that take one joint group home first.

        These are the moves an operator makes by hand: fold the wrist to its home angles
        before swinging the arm, or lift the shoulder and elbow off the table before
        rotating the base. They keep the synchronized-arrival feel of the plain ramp,
        which is why they are tried before anything that moves joints one at a time."""
        wrist, pan, upper = self._groups()
        q0 = np.asarray(q0, float)
        for how, g in (("wrists first", wrist), ("shoulder+elbow first", upper),
                       ("arm before wrists", pan + upper), ("pan last", upper + wrist),
                       ("pan first", pan)):
            if time.monotonic() >= deadline:
                return None
            if not g or len(g) >= len(q0):
                continue
            w = q0.copy()
            for i in g:
                w[i] = home[i]
            if float(np.max(np.abs(w - q0))) < 1e-4:
                continue
            m1, _ = self._edge_margin(q0, w, scale, stop_below=floor_min)
            if m1 < floor_min:
                continue
            m2, _ = self._edge_margin(w, home, scale, stop_below=floor_min)
            if m2 >= floor_min:
                return [w, np.asarray(home, float)], how
        return None

    def _plan_greedy(self, q0, home, scale, floor_min, deadline, rng):
        """Coordinate descent: repeatedly take the single joint move to its home value
        that leaves the most clearance, falling back to a fraction of that move when the
        whole of it collides. Six joints, so at worst six slow single-joint segments —
        ugly to watch, but it is the tier that unpicks a wrist wound into a corner."""
        q0 = np.asarray(q0, float)
        cur = q0.copy()
        path = []
        while True:
            if time.monotonic() >= deadline or len(path) > 3 * len(cur):
                return None
            left = [i for i in range(len(cur)) if abs(home[i] - cur[i]) > 1e-4]
            if not left:
                return (path, "one joint at a time") if path else None
            best = None
            for i in left:
                for frac in (1.0, 0.6, 0.3):
                    cand = cur.copy()
                    cand[i] = cur[i] + frac * (home[i] - cur[i])
                    w, _ = self._edge_margin(cur, cand, scale, stop_below=floor_min)
                    if w >= floor_min:
                        if best is None or w > best[0]:
                            best = (w, i, frac, cand)
                        break   # a bigger move on the same joint is always preferable
            if best is None:
                return None
            path.append(best[3])
            cur = best[3]

    def _plan_detour(self, q0, home, scale, floor_min, deadline, rng):
        """Back ONE joint out first, then re-run the cheap tiers from there.

        A wrist pressed onto the table and a gripper hooked under the camera arch are both
        the same shape of problem: every direct move is blocked because the arm is inside a
        pocket, and the way out is one deliberate step BACKWARDS before any progress is
        possible. Straight-line planners cannot express that; two segments can."""
        ik, T = self._ik, self._T
        q0 = np.asarray(q0, float)
        lo = np.array([ik.model.lowerPositionLimit[ik.qindex(j)] for j in T.ARM], float)
        hi = np.array([ik.model.upperPositionLimit[ik.qindex(j)] for j in T.ARM], float)
        for delta in (np.radians(45.0), np.radians(90.0), np.radians(135.0)):
            for i in range(len(q0)):
                for sgn in (1.0, -1.0):
                    if time.monotonic() >= deadline:
                        return None
                    w = q0.copy()
                    w[i] = float(np.clip(q0[i] + sgn * delta, lo[i], hi[i]))
                    if abs(w[i] - q0[i]) < 1e-3:
                        continue
                    m, _ = self._edge_margin(q0, w, scale, stop_below=floor_min)
                    if m < floor_min:
                        continue
                    name = f"{T.ARM[i]} backed out {np.degrees(w[i] - q0[i]):+.0f} deg"
                    m2, _ = self._edge_margin(w, home, scale, stop_below=floor_min)
                    if m2 >= floor_min:
                        return [w, np.asarray(home, float)], name
                    sub = self._plan_staged(w, home, scale, floor_min, deadline, rng)
                    if sub is not None:
                        return [w] + list(sub[0]), f"{name}, then {sub[1]}"
        return None

    def _plan_rrt(self, q0, home, scale, floor_min, deadline, rng):
        """RRT-Connect over the arm joints, then shortcut-smoothed.

        The general answer, and the reason "impossible" now means "no path exists in the
        time given" rather than "the straight line was blocked". It grows two trees, one
        from the arm and one from home, and every edge it accepts is checked at the same
        resolution as the final path — so what it returns needs no re-verification.

        If the trees never meet before the deadline it still returns the branch of the
        start tree that got CLOSEST to home. That is not a failure to be discarded: the
        outer loop executes it and replans from there, and a pose closer to home in joint
        space is usually a pose out of whatever pocket blocked the first search."""
        ik, T = self._ik, self._T
        q0 = np.asarray(q0, float)
        home = np.asarray(home, float)
        lo = np.array([ik.model.lowerPositionLimit[ik.qindex(j)] for j in T.ARM], float)
        hi = np.array([ik.model.upperPositionLimit[ik.qindex(j)] for j in T.ARM], float)
        lo = np.minimum(lo, np.minimum(q0, home))
        hi = np.maximum(hi, np.maximum(q0, home))
        step = float(self._cfg("home_rrt_step", 0.35))

        def steer(a, b):
            d = b - a
            n = float(np.max(np.abs(d)))
            return b.copy() if n <= step else a + d * (step / n)

        def ok(a, b):
            return self._edge_margin(a, b, scale, stop_below=floor_min)[0] >= floor_min

        A, PA = [q0.copy()], [-1]          # tree rooted at the arm
        B, PB = [home.copy()], [-1]        # tree rooted at home
        swapped = False

        def chain(tree, par, k):
            out = []
            while k >= 0:
                out.append(tree[k])
                k = par[k]
            return out[::-1]

        while time.monotonic() < deadline:
            qr = rng.uniform(lo, hi)
            arr = np.asarray(A)
            ia = int(np.argmin(np.max(np.abs(arr - qr), axis=1)))
            qn = steer(A[ia], qr)
            if ok(A[ia], qn):
                A.append(qn)
                PA.append(ia)
                # Greedy CONNECT from the other tree: the half of RRT-Connect that makes
                # it find paths in seconds instead of minutes.
                brr = np.asarray(B)
                k = int(np.argmin(np.max(np.abs(brr - qn), axis=1)))
                while time.monotonic() < deadline:
                    qs = steer(B[k], qn)
                    if not ok(B[k], qs):
                        break
                    B.append(qs)
                    PB.append(k)
                    k = len(B) - 1
                    if float(np.max(np.abs(qs - qn))) < 1e-9:
                        ca = chain(A, PA, len(A) - 1)
                        cb = chain(B, PB, k)[::-1][1:]
                        pts = (cb[::-1] + ca[::-1]) if swapped else (ca + cb)
                        # Smoothing gets its own clock, NOT what is left of the search
                        # deadline: an RRT that only just made it would otherwise return
                        # its raw random walk, and the raw walk is what the arm executes.
                        pts = self._shortcut(
                            pts, scale, floor_min,
                            time.monotonic() + float(self._cfg("home_smooth_time_s", 2.0)), rng)
                        return pts[1:], "rrt"
            A, PA, B, PB = B, PB, A, PA
            swapped = not swapped
        start, pstart = (B, PB) if swapped else (A, PA)
        d = np.max(np.abs(np.asarray(start) - home), axis=1)
        k = int(np.argmin(d))
        if k == 0:
            return None
        pts = self._shortcut(chain(start, pstart, k), scale, floor_min,
                             time.monotonic() + 1.0, rng)
        return pts[1:], "rrt (partial)"

    def _shortcut(self, pts, scale, floor_min, deadline, rng, iters=200):
        """Drop waypoints whose neighbours can see each other. An RRT path is a random
        walk; unsmoothed it would drive the arm around the room to get across it."""
        pts = [np.asarray(p, float) for p in pts]
        for _ in range(iters):
            if len(pts) <= 2 or time.monotonic() >= deadline:
                break
            i = int(rng.integers(0, len(pts) - 2))
            j = int(rng.integers(i + 2, len(pts)))
            if self._edge_margin(pts[i], pts[j], scale, stop_below=floor_min)[0] >= floor_min:
                pts = pts[:i + 1] + pts[j:]
        return pts

    # ------------------------------------------------------------------ home execution
    def _seg_now(self):
        with self._feed_lock:
            return None if self._seg is None else self._segment_value(time.monotonic())[0]

    def _run_path(self, path, v, deadline, scale=1.0):
        """Execute a checked polyline at v rad/s. Returns 'done' | 'gate' | 'timeout'.

        Between waypoints the target dwells briefly. The shaper rounds corners (its accel
        limit plus the v/out_kp standing lag), and a rounded corner is the one part of the
        motion the polyline check did NOT certify; pausing at each waypoint keeps the arm
        on the path that was checked instead of near it.

        The measured-pose gate is armed at min(meas_gate_frac, the plan's own floor scale)
        and never above the clearance the plan itself was allowed. Armed at the teleop's
        fixed 0.6x it would fire on the first tick of every relaxed-cushion plan — which is
        every plan that needed relaxing, i.e. exactly the ones that must be allowed to run.
        What it still catches is the arm leaving the checked path: a tracking failure, a
        collision the model does not know about, someone leaning into the cell."""
        T = self._T
        gate_min = gate_scale = None
        if self._cfg("home_gate", True) and float(getattr(T, "MEAS_GATE_FRAC", 0.0)) > 0.0:
            gate_scale = min(float(T.MEAS_GATE_FRAC), float(scale))
            gate_min = min(0.0, self._margin(self._meas(), scale)[0])
        dwell = float(self._cfg("home_waypoint_dwell", 0.2))
        cur = self._seg_now()
        if cur is None:
            cur = self._meas()
        elif float(np.max(np.abs(cur - self._meas()))) > float(getattr(T, "MAX_JOINT_LEAD", 0.15)):
            # The published command and the arm have diverged — a stalled joint, a
            # protective stop, a previous leg the arm never finished. The plan was checked
            # from where the arm IS, and an arm chasing a target it never reached is
            # travelling an unchecked path, so walk the target back onto the measured pose
            # first. As a SEGMENT, not a jump: a target more than max_joint_lead from what
            # the shaper is publishing makes it snap, i.e. publish one unlimited sample
            # straight into servoj.
            meas = self._meas()
            self._node.get_logger().warn(
                f"HOME: the command is {float(np.max(np.abs(cur - meas))):.2f} rad ahead of "
                f"the arm — re-anchoring the target onto the measured pose before the plan")
            path = [meas] + list(path)
        for k, wp in enumerate(path):
            wp = np.asarray(wp, float)
            d = float(np.max(np.abs(wp - cur)))
            self._set_segment(cur, wp, max(d / max(v, 1e-6), 0.02))
            st = self._wait_segment(deadline, gate_min, gate_scale)
            if st != "done":
                return st
            cur = wp
            if dwell > 0.0 and k < len(path) - 1:
                time.sleep(dwell)
        return "done"

    def _freeze_feed(self):
        """Park the target where the interpolation currently is: the feeder keeps
        republishing it (the shaper's OUT_STALE keepalive) and nothing moves."""
        cur = self._seg_now()
        if cur is not None:
            self._set_segment(cur, cur, 0.0)

    def _settle(self, target, grace, tol):
        """The segment finishing means the COMMAND arrived; the arm still has to. Returns
        (arrived, max error) — used both to confirm home and to let a partial plan come to
        rest before the next search measures the pose it starts from."""
        t0 = time.monotonic()
        err = float("inf")
        while self._connected and time.monotonic() - t0 < grace:
            err = float(np.max(np.abs(self._meas() - np.asarray(target, float))))
            if err <= tol:
                return True, err
            time.sleep(0.05)
        return False, err

    def _path_check(self, q_from, q_to):
        """Sample the straight joint segment through the teleop's collision floors and refuse a colliding approach."""
        self._ensure_ik()
        m, pair = self._edge_margin(q_from, q_to, stop_below=0.0)
        if m < 0.0:
            raise RuntimeError(
                f"approach path to the episode start COLLIDES ({pair}, "
                f"{1000.0 * m:.1f} mm below its floor) — move the arm clear "
                f"first (teleop HOME button), then rerun the replay")

    def _send_grip(self, width):
        T = self._T
        from control_msgs.action import GripperCommand
        if not self._grip.server_is_ready():
            self._node.get_logger().warn("gripper action server not ready", throttle_duration_sec=2.0)
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = T.GRIP_EFFORT
        self._grip.send_goal_async(goal)
        self._grip_sent = float(width)
