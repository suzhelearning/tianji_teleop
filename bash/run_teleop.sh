#!/usr/bin/env bash
# Single entry point for teleoperation. Chooses the operating mode, then runs
# the matching Pixi task.
#
#   --sim    actuator-driven simulation, no hardware
#   --real   real robot execution with the interactive authorization gate
#   --data   real execution with schema-v1 collection (requires camera_views)
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
# shellcheck source=bash/pixi.bash
source "$root/bash/pixi.bash" "$@"

usage() {
  printf 'Usage: %s (--sim | --real | --data) [arguments...]\n' "$0"
  printf '  --sim    Actuator-driven simulation without hardware\n'
  printf '  --real   Real robot execution with safety confirmation\n'
  printf '  --data   Real execution with RGB collection; requires --task LABEL [--dataset PATH]\n'
}

mode="${1:-}"
case "$mode" in
  --sim|--real|--data) shift ;;
  --help|-h) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac
for argument in "$@"; do
  case "$argument" in
    --sim|--real|--data)
      printf 'Select exactly one operating mode.\n' >&2; exit 2 ;;
  esac
done

cd -- "$root"
source "$root/bash/environment.sh"

case "$mode" in
  --sim)
    export TIANJI_ALLOW_UNBUILT=1
    exec "$TIANJI_PYTHON" -m simulation.run_sim "$@"
    ;;
  --real)
    exec "$TIANJI_PYTHON" -m tianji_controller.run_teleop \
      --devices all --confirm-real "$@"
    ;;
  --data)
    exec "$TIANJI_PYTHON" -m tianji_controller.run_teleop \
      --devices all --confirm-real --collection "$@"
    ;;
esac
