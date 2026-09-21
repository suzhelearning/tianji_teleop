#!/usr/bin/env bash
# Staged return-Home. Requires explicit --confirm-real; arms only.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
for argument in "$@"; do
  case "$argument" in
    --help|-h|--dry-run) exec "$TIANJI_PYTHON" -m tianji_controller.return_home "$@" ;;
  esac
done
exec "$TIANJI_PYTHON" -m tianji_controller.return_home --confirm-real "$@"
