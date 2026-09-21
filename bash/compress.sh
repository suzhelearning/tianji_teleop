#!/usr/bin/env bash
# Offline JPEG Q50 re-encode of one date directory. Source files are untouched.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
usage() {
  printf "Usage: bash compress.sh [--date YYYYMMDD]\n"
  printf "Convert the raw date directory to the compressed dataset (JPEG Q50).\n"
}
case "${1:-}" in
  "") day="$(date +%Y%m%d)" ;;
  --date) [[ $# == 2 && "$2" =~ ^[0-9]{8}$ ]] || { usage >&2; exit 2; }; day="$2" ;;
  --help|-h) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac
if ! normalized="$(date -d "${day:0:4}-${day:4:2}-${day:6:2}" +%Y%m%d 2>/dev/null)" || [[ "$normalized" != "$day" ]]; then
  printf "Invalid date: %s (expected YYYYMMDD).\n" "$day" >&2; exit 2
fi
exec "$TIANJI_PYTHON" -m tianji compress --date "$day"
