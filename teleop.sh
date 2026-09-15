#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
usage() {
  printf 'Usage: %s (--sim | --real | --data) [arguments...]\n' "$0"
  printf '  --sim      Actuator-driven simulation without hardware\n'
  printf '  --data     Real data collection; --task required, --dataset optional\n'
  printf '  --real     Real robot execution with safety confirmation\n'
}
mode="${1:-}"
case "$mode" in
  --data) command=collect; shift ;;
  --sim|--real) command="${mode#--}"; shift ;;
  --help|-h) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac
for argument in "$@"; do
  case "$argument" in
    --data|--sim|--real)
      printf 'Select exactly one operating mode.\n' >&2; exit 2 ;;
  esac
done
export TIANJI_PYTHON="${TIANJI_REAL_PYTHON:-${TIANJI_PYTHON:-$PWD/.venv/bin/python}}"
source scripts/environment.sh
if [[ "$mode" == --real ]]; then
  exec "$TIANJI_PYTHON" -m tianji real --devices all --confirm-real "$@"
fi
exec "$TIANJI_PYTHON" -m tianji "$command" "$@"
