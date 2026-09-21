#!/usr/bin/env bash
# Read-only RealSense preflight: enumerates devices and validates the schema-v1
# RGB mode. Opens no pipeline, so it cannot disturb a driver already streaming.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
exec "$TIANJI_PYTHON" -m tianji_cameras.preflight "$@"
