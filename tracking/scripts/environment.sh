#!/usr/bin/env bash
# Source this file from a tracking entry point; all Python uses the root environment.
tracking_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
project_root="$(cd "$tracking_root/.." && pwd -P)"
export TIANJI_PYTHON="${TIANJI_PYTHON:-$project_root/.venv/bin/python}"
if [[ ! -x "$TIANJI_PYTHON" ]]; then
  echo "Missing Python environment: $TIANJI_PYTHON; follow the root installation guide." >&2
  return 2
fi
unset PYTHONHOME
export VIRTUAL_ENV="$(cd "$(dirname "$TIANJI_PYTHON")/.." && pwd -P)"
export PATH="$VIRTUAL_ENV/bin:$PATH"
if [[ "${1:-}" == --python-only ]]; then
  return 0
fi
ros_setup="${ROS_SETUP:-$project_root/.pixi/envs/tracking/setup.bash}"
if [[ ! -r "$ros_setup" ]]; then
  echo "ROS 2 SDK setup missing: $ros_setup; install ROS 2 Humble or set ROS_SETUP." >&2
  return 2
fi
# Discard inherited checkout overlays; ROS_SETUP supplies the selected SDK.
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
# Preserve the active SDK's Conda toolchain context for its setup hooks.
tracking_nounset=false
[[ $- != *u* ]] || tracking_nounset=true
set +u
if ! source "$ros_setup"; then
  [[ "$tracking_nounset" != true ]] || set -u
  return 2
fi
if [[ "${1:-}" != --build ]]; then
  if [[ ! -r "$tracking_root/install/local_setup.bash" ]]; then
    echo "Tracking overlay missing; run: bash $tracking_root/scripts/build.sh" >&2
    [[ "$tracking_nounset" != true ]] || set -u
    return 2
  fi
  if ! source "$tracking_root/install/local_setup.bash"; then
    [[ "$tracking_nounset" != true ]] || set -u
    return 2
  fi
fi
[[ "$tracking_nounset" != true ]] || set -u
# SDK Python modules also load native libraries (for example GLFW) at runtime.
# A plain Bash launch has no Pixi activation to expose the selected SDK's lib/.
ros_library_dir="$(cd -- "$(dirname -- "$ros_setup")" && pwd -P)/lib"
if [[ -d "$ros_library_dir" ]]; then
  export LD_LIBRARY_PATH="$ros_library_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export PATH="$VIRTUAL_ENV/bin:$PATH"
"$TIANJI_PYTHON" -c 'import rclpy' || {
  echo "Root .venv must use the ROS SDK Python ABI and expose its system packages." >&2
  return 2
}
