#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
for arg in "$@"; do
  case "$arg" in
    --real|--confirm-real|--data|--ik-backend|--ik-backend=*)
      printf '%s\n' 'This shortcut is mapped-palm simulation only.' >&2; exit 2 ;;
  esac
done
exec bash teleop.sh --sim --ik-backend mapped-palm "$@"
