#!/usr/bin/env bash
# PICO bare-hand DLS/Ruckig targets: ROS JointCommand to SPD plus native viewer.
# Simulation only: no hardware driver, TJRC export, or --real mode.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
exec "$TIANJI_PYTHON" -m pico2_hands.run_sim "$@"
