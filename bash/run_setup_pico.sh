#!/usr/bin/env bash
# Interactive named TCP calibration and explicit publish/cancel wizard.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
exec "$TIANJI_PYTHON" "$root/src/teleop_inputs/pico_controller/scripts/setup_pico.py" "$@"
