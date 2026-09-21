#!/usr/bin/env bash
# Top RGB DDS subscriber -> H.264 -> PICO. Never opens a camera pipeline.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
exec "$TIANJI_PYTHON" -m tianji_cameras.pico_stream "$@"
