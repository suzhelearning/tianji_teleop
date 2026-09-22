#!/usr/bin/env bash
# Build the installed ROS acquisition, adapter and SDK Hand2 target publisher.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh" --build
environment="${PIXI_ENVIRONMENT_NAME:-default}"
exec colcon --log-base "$root/log/$environment" build \
  --base-paths "$root/src" \
  --build-base "$root/build/$environment" \
  --install-base "$root/install/$environment" \
  --symlink-install --packages-up-to manus_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  "$@"
