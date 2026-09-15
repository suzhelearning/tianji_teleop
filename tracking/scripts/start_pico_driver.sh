#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

source "$repo_root/scripts/environment.sh"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export EXO_REQUESTED_ROS_DOMAIN_ID="${EXO_REQUESTED_ROS_DOMAIN_ID:-$ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"


command -v adb >/dev/null || { echo "adb not found" >&2; exit 2; }
adb forward tcp:9999 tcp:9999

echo "PICO driver starting (domain=$ROS_DOMAIN_ID)"
exec ros2 launch pico_bridge start_pico_bridge.launch.py "$@"
