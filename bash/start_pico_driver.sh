#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
pico_scripts="$repo_root/src/teleop_inputs/pico_controller/scripts"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export EXO_REQUESTED_ROS_DOMAIN_ID="${EXO_REQUESTED_ROS_DOMAIN_ID:-$ROS_DOMAIN_ID}"
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"

# Set project defaults before the ROS SDK can supply its own localhost policy.
source "$repo_root/bash/environment.sh"


python3 "$pico_scripts/ensure_pico_tracking_adb.py"

echo "PICO driver starting (domain=$ROS_DOMAIN_ID)"
exec ros2 launch pico_bridge start_pico_bridge.launch.py "$@"
