#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

usage() {
  printf 'Usage: %s (--user NAME | --calibration-user NAME) [Manus arguments...]\n       %s (--list-users | --list-calibration-users)\n' "$0" "$0"
}

direct_calibration=false
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
  --calibration-user)
    [[ $# -ge 2 && "$2" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || { usage >&2; exit 2; }
    user="$2"
    direct_calibration=true
    shift 2
    ;;
  --list-calibration-users)
    [[ $# == 1 ]] || { usage >&2; exit 2; }
    # Listing files is read-only and does not require the SDK/Python runtime.
    for left_file in manus/calibration/*LeftMetaglovePro.mcal; do
      [[ -f "$left_file" ]] || continue
      calibration_name="${left_file##*/}"
      calibration_name="${calibration_name%LeftMetaglovePro.mcal}"
      if [[ -f "manus/calibration/${calibration_name}RightMetaglovePro.mcal" ]]; then
        printf '%s\n' "$calibration_name"
      fi
    done
    exit 0
    ;;
  *) usage >&2; exit 2 ;;
esac

for argument in "$@"; do
  case "$argument" in
    --u|--us|--use|--user|--u=*|--us=*|--use=*|--user=*|--calibration-user|--calibration-user=*)
      printf 'Select exactly one personnel profile with --user NAME.\n' >&2
      exit 2
      ;;
  esac
done

source scripts/environment.sh
if [[ "$direct_calibration" == true ]]; then
  manus_user="$user"
  for side in Left Right; do
    [[ -f "manus/calibration/${manus_user}${side}MetaglovePro.mcal" ]] || {
      printf 'Missing Manus calibration for %s (%s); no input started.\n' "$manus_user" "$side" >&2
      exit 2
    }
  done
else
  manus_user="$("$TIANJI_PYTHON" -m tianji profile \
    --user "$user" --component manus)"
fi
source tracking/scripts/environment.sh --build
exec "$TIANJI_PYTHON" manus/start_hand_teleop.py --user "$manus_user" "$@"
