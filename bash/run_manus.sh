#!/usr/bin/env bash
# Manus glove input -> TJH2 UDP. Exactly one personnel profile.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
# The supervisor and its ROS nodes always use default. Native dependencies
# belong exclusively to the worker started with `pixi run -e manus`.
if [[ "${PIXI_ENVIRONMENT_NAME:-}" != default ]]; then
  exec env -u TIANJI_PYTHON -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH \
    -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    TIANJI_PIXI_ACTIVE=1 TIANJI_ENVIRONMENT=default \
    pixi run --locked --manifest-path "$root/pixi.toml" -e default \
    bash "${BASH_SOURCE[0]}" "$@"
fi
export TIANJI_PYTHON="$CONDA_PREFIX/bin/python" TIANJI_ENVIRONMENT=default
source "$root/bash/environment.sh"
usage() {
  printf "Usage: %s (--user NAME | --calibration-user NAME) [Manus arguments...]\n" "$0"
  printf "       %s (--list-users | --list-calibration-users)\n" "$0"
}
case "${1:-}" in
  --help|-h) usage; exit 0 ;;
  --list-users|--list-calibration-users)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    exec "$TIANJI_PYTHON" -m manus_bridge.list_calibrations ;;
  --user|--calibration-user)
    [[ $# -ge 2 && "$2" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]] || { usage >&2; exit 2; }
    user="$2"; shift 2 ;;
  *) usage >&2; exit 2 ;;
esac
for argument in "$@"; do
  case "$argument" in
    --u|--us|--use|--user|--u=*|--us=*|--use=*|--user=*|--calibration-user|--calibration-user=*)
      printf "Select exactly one personnel profile with --user NAME.\n" >&2; exit 2 ;;
  esac
done
exec "$TIANJI_PYTHON" -m manus_bridge.start_hand_teleop --user "$user" "$@"
