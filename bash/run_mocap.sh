#!/usr/bin/env bash
# Replay and policy entry points share the active Jazzy workspace overlay.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
environment=default
case "${1:-}" in
  infer|live|regrind-real|regrind-hand-sim) environment=policy ;;
esac
if [[ "${PIXI_ENVIRONMENT_NAME:-}" != "$environment" ]]; then
  exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD \
    -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u TIANJI_PYTHON -u ROS_DISTRO -u ROS_VERSION \
    TIANJI_PIXI_ACTIVE=1 TIANJI_ENVIRONMENT="$environment" \
    pixi run --locked --manifest-path "$root/pixi.toml" -e "$environment" \
    bash "${BASH_SOURCE[0]}" "$@"
fi
export TIANJI_PIXI_ACTIVE=1 TIANJI_ENVIRONMENT="$environment"
export TIANJI_PYTHON="$CONDA_PREFIX/bin/python"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
exec "$TIANJI_PYTHON" -m mocap_policy_runtime "$@"
