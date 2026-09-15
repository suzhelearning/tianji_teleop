#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export TIANJI_PYTHON="${COLLECTION_PYTHON:-${TIANJI_PYTHON:-$PWD/.venv/bin/python}}"
source scripts/environment.sh
exec "$TIANJI_PYTHON" -m tianji collect "$@"
