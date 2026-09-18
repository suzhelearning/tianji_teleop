#!/usr/bin/env bash
set -euo pipefail
PICO2_SIM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PICO2_SIM_ROOT"
if [[ -x "$PICO2_SIM_ROOT/pico2_hands/.venv/bin/python" ]]; then
  PICO2_SIM_DEFAULT="$PICO2_SIM_ROOT/pico2_hands/.venv/bin/python"
else
  PICO2_SIM_DEFAULT="$PICO2_SIM_ROOT/.pixi/envs/default/bin/python"
fi
PICO2_SIM_PYTHON="${PICO2_PYTHON:-$PICO2_SIM_DEFAULT}"
if [[ ! -x "$PICO2_SIM_PYTHON" ]]; then
  echo 'Install the project Pixi environment, or set PICO2_PYTHON to its Python executable.' >&2
  exit 1
fi
# Deliberately no --real, ADB management, ROS driver, or UDP command publisher.
exec "$PICO2_SIM_PYTHON" -m pico2_hands.run_sim "$@"
