#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

usage() {
  printf 'Usage: bash compress.sh [--date YYYYMMDD]\n'
  printf 'Compress $HOME/Documents/TianjiData/YYYYMMDD to YYYYMMDD_compressed (JPEG Q50).\n'
  printf 'Default: today. Existing outputs are skipped only after validation.\n'
}

case "${1:-}" in
  '') day="$(date +%Y%m%d)" ;;
  --date)
    if [[ $# != 2 || ! "$2" =~ ^[0-9]{8}$ ]]; then
      usage >&2; exit 2
    fi
    day="$2"
    ;;
  --help|-h) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac
if ! normalized="$(date -d "${day:0:4}-${day:4:2}-${day:6:2}" +%Y%m%d 2>/dev/null)" || [[ "$normalized" != "$day" ]]; then
  printf 'Invalid date: %s (expected YYYYMMDD).\n' "$day" >&2
  exit 2
fi

source scripts/environment.sh
source_dir="$HOME/Documents/TianjiData/$day"
destination_dir="${source_dir}_compressed"
printf 'Source: %s\nDestination: %s\n' "$source_dir" "$destination_dir"
exec "$TIANJI_PYTHON" -m tianji compress "$source_dir" "$destination_dir"
