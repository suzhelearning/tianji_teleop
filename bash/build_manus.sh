#!/usr/bin/env bash
# Build rawviz with the system toolchain, then build/install the isolated
# retargeter using the manus environment's own Python and compiler.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
package="$root/src/teleop_inputs/manus_bridge"

if [[ ! -r "$package/build.sh" ]]; then
  printf 'Missing Manus build script: %s/build.sh\n' "$package" >&2
  exit 1
fi
bash "$package/build.sh" "$@"
exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD \
  -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u TIANJI_PYTHON -u ROS_DISTRO -u ROS_VERSION \
  TIANJI_ENVIRONMENT=manus \
  pixi run --locked --manifest-path "$root/pixi.toml" -e manus \
  python -I "$package/build_runtime.py" "$root"
