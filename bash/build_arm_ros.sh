#!/usr/bin/env bash
# Build the unchanged native arm core with its own ABI-compatible ROS boundary.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${PIXI_ENVIRONMENT_NAME:-}" != arm-ros ]]; then
  exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD \
    -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u TIANJI_PYTHON -u ROS_DISTRO -u ROS_VERSION \
    pixi run --locked --manifest-path "$root/pixi.toml" -e arm-ros \
    bash "${BASH_SOURCE[0]}" "$@"
fi
# Do not source default/policy: headers, IDL and typesupport must use this ABI.
unset PYTHONPATH PYTHONHOME COLCON_PREFIX_PATH ROS_LOCALHOST_ONLY
export ROS_DISTRO=jazzy ROS_VERSION=2 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export AMENT_PREFIX_PATH="$CONDA_PREFIX" CMAKE_PREFIX_PATH="$CONDA_PREFIX"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib"
colcon --log-base "$root/log/arm-ros" build \
  --base-paths "$root/src/interfaces/tianji_interfaces" \
  --build-base "$root/build/arm-ros/ros" --install-base "$root/install/arm-ros" \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"
cmake -S "$root/src/teleop_outputs/tianji/tianji_controller/native" \
  -B "$root/build/arm-ros/core" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/install/control" \
  -DCMAKE_PREFIX_PATH="$root/install/arm-ros/tianji_interfaces;$CONDA_PREFIX" \
  -DTIANJI_BUILD_ROS=ON -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" "$@"
cmake --build "$root/build/arm-ros/core" --target tianji_arm_ros \
  --parallel "${TIANJI_BUILD_JOBS:-4}"
cmake --install "$root/build/arm-ros/core" --component arm_ros
