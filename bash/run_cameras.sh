#!/usr/bin/env bash
# The official driver has its own native ABI; communicate with default over DDS.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${PIXI_ENVIRONMENT_NAME:-}" != cameras ]]; then
  exec pixi run --locked -e cameras --manifest-path "$root/pixi.toml" \
    bash "$root/bash/run_cameras.sh" "$@"
fi
export TIANJI_WORKSPACE="$root" TIANJI_ENVIRONMENT=cameras
export TIANJI_PYTHON="$CONDA_PREFIX/bin/python"
# Do not source default's overlay or inherit its compiled Python/ROS libraries.
export AMENT_PREFIX_PATH="$CONDA_PREFIX" CMAKE_PREFIX_PATH="$CONDA_PREFIX"
unset COLCON_PREFIX_PATH PYTHONPATH PYTHONHOME
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"
source "$root/bash/environment.sh" --build
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
# These three roots contain only the camera app and shared pure-Python contracts.
# data_collector.config is stdlib-only; no dataset/model module is imported here.
export PYTHONPATH="$root/src/cameras/tianji_cameras:$root/src/interfaces/tianji_interfaces:$root/src/data_collector"
exec "$TIANJI_PYTHON" -m tianji_cameras.monitor --launch "$@"
