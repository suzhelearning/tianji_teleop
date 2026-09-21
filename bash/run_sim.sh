#!/usr/bin/env bash
# Headless-capable simulation. --headless / --dynamics / --ik-backend ... pass through.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
export TIANJI_ALLOW_UNBUILT=1
exec "$TIANJI_PYTHON" -m simulation.run_sim "$@"
