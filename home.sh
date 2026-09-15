#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export TIANJI_PYTHON="${TIANJI_REAL_PYTHON:-${TIANJI_PYTHON:-$PWD/.venv/bin/python}}"
source scripts/environment.sh
for argument in "$@"; do
  case "$argument" in
    --help|-h|--dry-run)
      exec "$TIANJI_PYTHON" -m tianji home "$@"
      ;;
  esac
done

# Explicit one-command authorization: arms only, no PICO/Manus or Wuji SDK.
exec "$TIANJI_PYTHON" -m tianji home --confirm-real "$@"
