#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

usage() {
  printf 'Usage: %s --user NAME [Manus arguments...]\n       %s --list-users\n' "$0" "$0"
}

case "${1:-}" in
  --help|-h) usage; exit 0 ;;
  --list-users)
    [[ $# == 1 ]] || { usage >&2; exit 2; }
    source scripts/environment.sh
    exec "$TIANJI_PYTHON" -m tianji profile --list-users
    ;;
  --user)
    [[ $# -ge 2 && -n "$2" ]] || { usage >&2; exit 2; }
    user="$2"
    shift 2
    ;;
  *) usage >&2; exit 2 ;;
esac

for argument in "$@"; do
  case "$argument" in
    --u|--us|--use|--user|--u=*|--us=*|--use=*|--user=*)
      printf 'Select exactly one personnel profile with --user NAME.\n' >&2
      exit 2
      ;;
  esac
done

source scripts/environment.sh
manus_user="$("$TIANJI_PYTHON" -m tianji profile \
  --user "$user" --component manus)"
source tracking/scripts/environment.sh --build
exec "$TIANJI_PYTHON" manus/start_hand_teleop.py --user "$manus_user" "$@"