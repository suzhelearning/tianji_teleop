#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export TIANJI_PYTHON="${REALSENSE_PYTHON:-${TIANJI_PYTHON:-$PWD/.venv/bin/python}}"
source scripts/environment.sh
source tracking/scripts/environment.sh --build

exec "$TIANJI_PYTHON" real_robot/realsense_camera.py "$@"