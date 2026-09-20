#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ $# == 1 && ( "$1" == --help || "$1" == -h ) ]]; then
  echo 'Usage: pixi run -e tracking pico-simple --user NAME'
  echo 'Loads only the published pico-simple profile; starts PICO input, not an executor.'
  exit 0
fi
if [[ $# != 2 || "$1" != --user || -z "$2" ]]; then
  echo 'Usage: bash scripts/start_simple_pico.sh --user NAME' >&2
  exit 2
fi
source scripts/environment.sh
if tmux has-session -t pico_tianji_teleop 2>/dev/null; then
  echo 'Existing PICO session: stop it explicitly before selecting a simple profile.' >&2
  exit 2
fi
calibration_dir="$("$TIANJI_PYTHON" scripts/calibrate_pico_simple.py --user "$2" --resolve)"
exec bash tracking/scripts/start_tianji_pico_teleop.sh --detach \
  --calibration-dir "$calibration_dir" --pico-world-x-offset 0.20
