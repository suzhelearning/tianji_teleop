#!/usr/bin/env bash
# PICO acquisition + TJVR publisher. Requires a calibrated personnel profile.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
usage() { printf "Usage: %s --user NAME | --list-users\n" "$0"; }
case "${1:-}" in
  --help|-h) usage; exit 0 ;;
  --list-users) exec "$TIANJI_PYTHON" "$root/src/teleop_inputs/pico_controller/scripts/calibrate_pico_simple.py" --list-users ;;
  --user) [[ $# -eq 2 && -n "$2" ]] || { usage >&2; exit 2; } ;;
  *) usage >&2; exit 2 ;;
esac
exec "$TIANJI_PYTHON" "$root/src/teleop_inputs/pico_controller/scripts/ensure_pico_user.py" "$@"
