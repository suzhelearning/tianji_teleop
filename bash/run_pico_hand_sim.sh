#!/usr/bin/env bash
# PICO bare-hand DLS/Ruckig targets: ROS JointCommand to SPD plus native viewer.
# Simulation only: no hardware driver, TJRC export, or --real mode.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
# An inherited activation flag is not evidence of the selected environment.
if [[ "${CONDA_PREFIX:-}" != "$root/.pixi/envs/spd" ]]; then
  exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD \
    -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u TIANJI_PYTHON -u ROS_DISTRO -u ROS_VERSION \
    pixi run --locked --manifest-path "$root/pixi.toml" -e spd \
    bash "$root/bash/run_pico_hand_sim.sh" "$@"
fi
export TIANJI_PIXI_ACTIVE=1 TIANJI_ENVIRONMENT=spd
export TIANJI_PYTHON="$CONDA_PREFIX/bin/python"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
exec "$TIANJI_PYTHON" -m pico2_hands.run_sim "$@"
