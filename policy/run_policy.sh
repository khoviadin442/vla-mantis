#!/usr/bin/env bash
# Run a trained policy on the Mantis left arm through the async-inference client.
#
#   ./run_policy.sh "Grab green cube and place it in the box"
#
# The policy itself runs on the GPU box; this only starts the robot-side client, which
# streams observations there and executes the returned action chunks. Commands go to
# safety_filter_controller, not forward_position_controller — activate/deactivate those
# yourself, this script only checks and refuses to run blind.
#
# Everything below is overridable from the environment, e.g.
#   POLICY_SERVER=10.0.0.5:8080 CAMERA_TOPICS="{left: /camera/echo_camera/color/image_raw}" \
#       ./run_policy.sh "Fold the pink cloth in half"
set -eo pipefail

TASK="${1:?usage: run_policy.sh \"<task string>\" [extra lerobot args]
  the task string is what the policy was trained to condition on; it must match
  the wording used at training time, not just describe the same intent}"
shift || true

SHARE="${SHARE_DIR:-/home/ros/share}"
SERVER="${POLICY_SERVER:-127.0.0.1:8080}"
POLICY_TYPE="${POLICY_TYPE:-pi0}"
# The persistent server holds the checkpoint itself and ignores this; set it only when
# talking to a stock policy_server that loads on request.
CHECKPOINT="${CHECKPOINT:-persistent}"
FPS="${FPS:-15}"
ACTIONS_PER_CHUNK="${ACTIONS_PER_CHUNK:-50}"
CHUNK_SIZE_THRESHOLD="${CHUNK_SIZE_THRESHOLD:-0.0}"
AGGREGATE_FN="${AGGREGATE_FN:-weighted_average}"
POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
# Must be exactly the cameras the checkpoint was trained on: a key the checkpoint has no
# encoder for is a server-side KeyError, not a silent ignore.
CAMERA_TOPICS="${CAMERA_TOPICS:-{left: /camera/echo_camera/color/image_raw, right: /camera/foxtrot_camera/color/image_raw\}}"
USE_SAFETY_FILTER="${USE_SAFETY_FILTER:-true}"
HOME_ON_CONNECT="${HOME_ON_CONNECT:-true}"   # ramp to HOME_Q before the policy starts
ROBOT_ID="${ROBOT_ID:-mantis_left}"

die() { echo "run_policy.sh: $*" >&2; exit 1; }

source /opt/ros/jazzy/setup.bash
# Chains in ros2_utils_ws, so orbbec_description's meshes resolve for the approach check.
source "$SHARE/mantis_ws/install/setup.bash"
export LD_LIBRARY_PATH="$HOME/.local/lib/python3.12/site-packages/cmeel.prefix/lib:${LD_LIBRARY_PATH:-}"
export HF_HUB_OFFLINE=1

# PROBE=1 -> one observation, print the returned chunk, exit. Creates no publisher, so the
# arm cannot move and none of the checks below apply: this is the safe first run against a
# server you have not talked to yet.
if [[ -n "${PROBE:-}" ]]; then
    echo "run_policy.sh: PROBE — read-only, no publisher is created, the arm cannot move"
    exec python3 "$SHARE/policy/probe_policy_server.py" \
        --server_address="$SERVER" \
        --task="$TASK" \
        --policy_type="$POLICY_TYPE" \
        --camera_topics="$CAMERA_TOPICS" \
        --actions_per_chunk="$ACTIONS_PER_CHUNK" \
        "$@"
fi

python3 -c "import lerobot_robot_mantis" 2>/dev/null || die \
    "the mantis robot plugin is not installed in this container. Run:
    python3 -m pip install --user --break-system-packages -e $SHARE/lerobot_robot_mantis
  (editable, so later edits under $SHARE need no reinstall; a recreated container loses it)"

# Both publish the arm command topic; two writers means neither trajectory is what runs.
if pgrep -f "python3? .*teleop_mantis\.py" >/dev/null 2>&1; then
    die "teleop_mantis.py is running — stop it first, it publishes the arm command topic too"
fi

# Advisory only: the run is still refused, but with SKIP_CONTROLLER_CHECK=1 you can bypass
# (e.g. the controller_manager is namespaced, or you know the state better than this check).
if [[ -z "${SKIP_CONTROLLER_CHECK:-}" ]]; then
    if ctrl=$(timeout 8 ros2 control list_controllers 2>/dev/null); then
        want=$([[ "$USE_SAFETY_FILTER" == "true" ]] \
               && echo safety_filter_controller || echo forward_position_controller)
        other=$([[ "$USE_SAFETY_FILTER" == "true" ]] \
                && echo forward_position_controller || echo safety_filter_controller)
        line=$(grep -E "^[[:space:]]*${want}\b" <<<"$ctrl" || true)
        [[ -n "$line" ]] || die "$want is not loaded in the controller_manager.
  Load and activate it (that step is yours), or re-run with SKIP_CONTROLLER_CHECK=1.
  Loaded now:
$(sed 's/^/    /' <<<"$ctrl")"
        [[ "$(awk '{print $NF}' <<<"$line")" == active ]] || die "$want is loaded but not active ($line).
  Commanding an inactive controller moves nothing. Activate it, or SKIP_CONTROLLER_CHECK=1."
        if [[ "$(grep -E "^[[:space:]]*${other}\b" <<<"$ctrl" | awk '{print $NF}')" == active ]]; then
            die "$other is ALSO active. Both are terminal controllers for the same
  hardware position interfaces — deactivate $other before running."
        fi
    else
        echo "run_policy.sh: WARNING — could not reach the controller_manager;" >&2
        echo "  skipping the controller check. If the robot stack is down the client" >&2
        echo "  will fail at connect() with a /joint_states timeout." >&2
    fi
fi

echo "run_policy.sh: task=\"$TASK\" server=$SERVER policy=$POLICY_TYPE" \
     "safety_filter=$USE_SAFETY_FILTER cams=$CAMERA_TOPICS"

exec python3 "$SHARE/policy/robot_client_cpu.py" \
    --robot.type=mantis_follower \
    --robot.id="$ROBOT_ID" \
    --robot.teleop_config="$SHARE/config_teleop_mantis.yaml" \
    --robot.use_safety_filter="$USE_SAFETY_FILTER" \
    --robot.home_on_connect="$HOME_ON_CONNECT" \
    --robot.camera_topics="$CAMERA_TOPICS" \
    --task="$TASK" \
    --policy_type="$POLICY_TYPE" \
    --pretrained_name_or_path="$CHECKPOINT" \
    --server_address="$SERVER" \
    --policy_device="$POLICY_DEVICE" \
    --client_device=cpu \
    --fps="$FPS" \
    --actions_per_chunk="$ACTIONS_PER_CHUNK" \
    --chunk_size_threshold="$CHUNK_SIZE_THRESHOLD" \
    --aggregate_fn_name="$AGGREGATE_FN" \
    "$@"
