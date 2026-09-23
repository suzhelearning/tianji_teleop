#!/usr/bin/env bash
# Exercise the SPD client only, isolated from the deployment ROS domain.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${1:-}" != pico2 ]]; then
  printf 'Usage: %s pico2 [pytest args]\n' "$0" >&2
  exit 2
fi
shift
source "$root/bash/environment.sh"
cd -- "$root"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_DOMAIN_ID="${TIANJI_TEST_ROS_DOMAIN_ID:-121}"
exec "$TIANJI_PYTHON" -m pytest src/teleop_inputs/pico_hand/python/pico2_hands/tests "$@"
