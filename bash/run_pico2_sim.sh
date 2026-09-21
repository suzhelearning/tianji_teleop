#!/usr/bin/env bash
# PICO2 bare-hand V131 or shared-root DLS simulation. Never drives hardware or
# publishes on the teleop ports; --real does not exist by design.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
export TIANJI_ALLOW_UNBUILT=1
exec "$TIANJI_PYTHON" -m pico2_hands.run_sim "$@"
