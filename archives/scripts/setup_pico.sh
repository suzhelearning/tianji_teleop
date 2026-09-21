#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source scripts/environment.sh
if [[ $# == 1 && ( "$1" == --help || "$1" == -h ) ]]; then
  exec "$TIANJI_PYTHON" scripts/setup_pico.py --help
fi
export ROS_DOMAIN_ID=120 EXO_REQUESTED_ROS_DOMAIN_ID=120 ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1
source tracking/scripts/environment.sh
exec "$TIANJI_PYTHON" scripts/setup_pico.py "$@"
