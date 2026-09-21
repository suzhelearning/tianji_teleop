#!/usr/bin/env bash
# Standalone schema-v1 collector. Subscribes to the camera driver and the
# executor feedback topics; it never connects to a robot or opens a camera.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
exec "$TIANJI_PYTHON" -m data_collector.node "$@"
