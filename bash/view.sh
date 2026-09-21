#!/usr/bin/env bash
# Browse today's (or a chosen date's) compressed dataset on port 8777.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
usage() {
  printf "Usage: bash view.sh [--date YYYYMMDD]\n"
  printf "Open http://127.0.0.1:8777/ after startup.\n"
}
case "${1:-}" in
  "") day="$(date +%Y%m%d)" ;;
  --date) [[ $# == 2 && "$2" =~ ^[0-9]{8}$ ]] || { usage >&2; exit 2; }; day="$2" ;;
  --help|-h) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac
exec "$TIANJI_PYTHON" -m tianji visualize --date "$day" --port 8777
