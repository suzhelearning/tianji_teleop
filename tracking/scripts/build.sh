#!/usr/bin/env bash
set -euo pipefail
tracking_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
packages=(imu_ros2 pico_bridge fisheye_camera data_collector)
if [[ "${1:-}" == --all ]]; then
  packages+=(odin_ros_driver odin_ros_driver_rev1 pico_odin)
  shift
elif [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo "Usage: bash tracking/scripts/build.sh [--all] [colcon build options]"
  echo "Default: PICO/IMU/cameras/collector. --all also builds both Odin drivers and fusion."
  exit 0
fi
source "$tracking_root/scripts/environment.sh" --build
export ROS_VERSION=2
cd "$tracking_root"
if [[ " ${packages[*]} " == *" odin_ros_driver_rev1 "* ]]; then
  bash "$tracking_root/src/odin/scripts/ensure_odin_sdk.sh"
fi
"$VIRTUAL_ENV/bin/python" "$VIRTUAL_ENV/bin/colcon" build --base-paths src --symlink-install \
  --packages-select "${packages[@]}" "$@" \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
  -DPython_EXECUTABLE="$VIRTUAL_ENV/bin/python" -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"
