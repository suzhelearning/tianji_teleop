#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
source scripts/environment.sh
exec "$TIANJI_PYTHON" scripts/calibrate_pico_simple.py "$@"
