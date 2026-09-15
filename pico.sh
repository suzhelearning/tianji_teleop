#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export TIANJI_PYTHON="${TIANJI_PYTHON:-$PWD/.venv/bin/python}"

usage() {
  printf 'Usage: %s --user NAME\n       %s --list-users\n' "$0" "$0"
}

case "${1:-}" in
  --help|-h) usage; exit 0 ;;
  --list-users)
    [[ $# == 1 ]] || { usage >&2; exit 2; }
    source scripts/environment.sh
    exec "$TIANJI_PYTHON" -m tianji profile --list-users
    ;;
  --user)
    [[ $# == 2 && -n "$2" ]] || { usage >&2; exit 2; }
    ;;
  *) usage >&2; exit 2 ;;
esac

source scripts/environment.sh
calibration_dir="$("$TIANJI_PYTHON" -m tianji profile \
  --user "$2" --component pico)"
exec ./tracking/scripts/start_tianji_pico_teleop.sh \
  --detach --calibration-dir "$calibration_dir"