#!/usr/bin/env bash
# Three RGB cameras, the existing PICO video bridge, and the saved RViz layout.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
exec "$TIANJI_PYTHON" -m tianji_cameras.camera_views "$@"
