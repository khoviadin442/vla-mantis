from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("mantis_follower")
@dataclass
class MantisFollowerConfig(RobotConfig):
    """Robot specifics come from the teleop's own YAML, so the two stacks cannot disagree."""
    teleop_config: str = "/home/ros/share/config_teleop_mantis.yaml"
    # Which controller this run commands. True (the default, and what a policy run needs) =
    # safety_filter_controller on `topics.arm_cmd_safety`, which delta-clamps and
    # horizon-collision-checks every reference and holds the last safe command instead of
    # following a bad one. False = the raw forward_position_controller on `topics.arm_cmd`,
    # unfiltered - that is what lerobot-replay of an already-vetted episode uses.
    # The two are terminal controllers for the same hardware interfaces, so exactly one can
    # be active; commanding the inactive one moves nothing, which is the safe way to fail.
    use_safety_filter: bool = True
    # Ramp to the teleop's HOME_Q (teleop.home_q_deg) at connect(), before the policy is allowed
    # to send anything, so every run starts from the pose the episodes were recorded from instead
    # of wherever the arm was left. Uses the teleop's own home speed cap and barrier pre-check,
    # and opens the gripper, exactly as the teleop's HOME button does. If the straight joint path
    # is not clear it searches for another one (see the homing knobs below) instead of refusing
    # to move, and raises only once that search is genuinely exhausted.
    home_on_connect: bool = True

    # ---- homing search ---------------------------------------------------------------------
    # Knobs for that search. Each is read through getattr against the same default hardcoded in
    # mantis_follower.py, so the follower stays drop-in if a field is missing; what declaring it
    # here buys is the CLI override (--robot.home_plan_time_s=20).
    #
    # False = no search: the straight joint path or nothing, i.e. what this file did before the
    # planner. The retry loop and the reporting stay, there is just one route to offer. This is
    # the rollback if the planner ever misbehaves on hardware.
    home_planner: bool = True
    # Collision-cushion fractions to search at, in order. The floors (d_min 15 mm, table 25 mm)
    # are a safety margin on top of real contact, and they are what a blocked home is usually
    # blocked by; when nothing routes at the full cushion the whole search repeats at each
    # fraction, 0.0 being real geometric contact only. No pose on a path may ever be worse than
    # the one the arm is already in, whatever the scale. Shrink to [1.0] to forbid relaxing.
    home_floor_scales: list[float] = field(default_factory=lambda: [1.0, 0.5, 0.0])
    # plan -> execute -> re-measure -> replan cycles before giving up. Each attempt replans from
    # where the arm actually ended up; an attempt that bought nothing drops a cushion tier first.
    home_max_attempts: int = 4
    home_plan_time_s: float = 8.0     # search budget per attempt
    home_total_time_s: float = 120.0  # wall-clock ceiling across ALL attempts
    # Accept a path that only gets CLOSER to home when no complete one is found. The retry loop
    # then replans from the new pose, and progress out of a corner is usually what unblocks the
    # next search. False = complete paths only, or refuse.
    home_allow_partial: bool = True
    # Arm the measured-pose gate while a plan executes. The plan certifies the COMMANDED path;
    # this watches the one the controller actually produces and freezes the target if the arm
    # leaves it. Only disable to debug a gate firing spuriously.
    home_gate: bool = True
    home_rrt_step: float = 0.35       # RRT-Connect extension step (rad)
    home_smooth_time_s: float = 2.0   # shortcut-smoothing budget for an RRT result
    home_waypoint_dwell: float = 0.2  # pause at each waypoint so rounded corners stay on the checked path
    # "The arm has come to rest" tolerance between legs. None = the teleop's 3 x home_done_tol.
    home_settle_tol: float | None = None
    approach_vel: float = 0.3
    approach_tol: float = 0.05
    connect_timeout: float = 10.0
    gripper_tol: float = 0.005
    # Feature name -> ROS color image topic. The names become `observation.images.<name>`, so
    # they are the contract with the policy: list exactly the cameras the checkpoint was
    # trained on. Both is what config_teleop_mantis.yaml records and what the datasets carry.
    # A key the checkpoint has no encoder for is not ignored - the server indexes
    # `policy_image_features[key]` and raises KeyError - so drop `right` here when running
    # against a checkpoint trained on echo alone.
    # Empty dict = joints only, which is what the hardcoded dummy server wants. NB: the field
    # cannot be called `cameras` - RobotConfig.__post_init__ validates that name as
    # CameraConfig objects (width/height/fps), which ROS topics are not.
    camera_topics: dict = field(default_factory=lambda: {
        "left": "/camera/echo_camera/color/image_raw",     # echo   (recorder's primary)
        "right": "/camera/foxtrot_camera/color/image_raw",  # foxtrot
    })
    camera_timeout: float = 10.0   # s to wait at connect() for one frame per camera
    camera_stale_s: float = 1.0    # warn when the newest frame is older than this
