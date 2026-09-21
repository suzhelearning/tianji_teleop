#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

usage() {
  printf 'Usage: bash compress.sh [--date YYYYMMDD]\n'
  printf 'Convert DATASET/YYYYMMDD to COMPRESSED_DATASET/YYYYMMDD_compressed (JPEG Q50).\n'
  printf 'Default DATASET: /data/TianjiData/raw; override with TIANJI_DATASET.\n'
  printf 'Default: today. Existing outputs are skipped only after validation.\n'
  printf 'JPEG input is re-encoded lossily; source files remain unchanged.\n'
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
dataset_paths="$("$TIANJI_PYTHON" -c 'from tianji.paths import DATASET, COMPRESSED_DATASET; print(DATASET); print(COMPRESSED_DATASET)')"
mapfile -t roots <<< "$dataset_paths"
source_dir="${roots[0]}/$day"
destination_dir="${roots[1]}/${day}_compressed"
printf 'Source: %s\nDestination: %s\n' "$source_dir" "$destination_dir"
exec "$TIANJI_PYTHON" -m tianji compress "$source_dir" "$destination_dir"
