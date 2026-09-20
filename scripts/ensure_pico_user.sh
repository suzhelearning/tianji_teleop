#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source scripts/environment.sh
export ROS_DOMAIN_ID=120 EXO_REQUESTED_ROS_DOMAIN_ID=120 ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1
source tracking/scripts/environment.sh
exec "$TIANJI_PYTHON" scripts/ensure_pico_user.py "$@"
