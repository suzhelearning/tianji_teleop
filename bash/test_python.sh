#!/usr/bin/env bash
# Run one Python test group inside the workspace environment with its overlay
# sourced, so installed packages (including the generated interfaces) resolve.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

group="${1:-}"
shift || true

# shellcheck source=bash/environment.sh
source "$root/bash/environment.sh"
cd -- "$root"

export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
# Tests must never observe or publish into the deployment domain.
export ROS_DOMAIN_ID="${TIANJI_TEST_ROS_DOMAIN_ID:-121}"

# Each group names the suites that actually exist for it; an unknown group fails
# loudly rather than silently running nothing.
extra=()
case "$group" in
  interfaces)  paths=(src/interfaces/tianji_interfaces/tianji_runtime/tests) ;;
  controller)  paths=(src/tianji/tianji_controller/tianji_controller/tests) ;;
  collection)  paths=(src/data_collector/tests) extra=(--ignore=src/data_collector/tests/test_ros_collection.py) ;;
  ros)         paths=(src/data_collector/tests/test_ros_collection.py) ;;
  simulation)  paths=(src/simulation/simulation/tests) ;;
  pico)        paths=(src/teleop_inputs/pico_bridge/test) ;;
  pico2)       paths=(src/teleop_inputs/pico2_hands/python/pico2_hands/tests) ;;
  retargeting) paths=(src/wuji/wuji_retargeting/tests) ;;
  manus)       paths=(src/teleop_inputs/manus_bridge/tests) ;;
  mocap)       paths=(src/inference/mocap_policy_runtime/tests) ;;
  all)
    paths=(
      src/interfaces/tianji_interfaces/tianji_runtime/tests
      src/tianji/tianji_controller/tianji_controller/tests
      src/data_collector/tests
      src/simulation/simulation/tests
    )
    ;;
  *)
    printf 'Usage: %s <interfaces|controller|collection|simulation|pico|pico2|mocap|retargeting|manus|ros|all> [pytest args]\n' "$0" >&2
    exit 2
    ;;
esac


existing=()
for path in "${paths[@]}"; do
  [[ -e "$path" ]] && existing+=("$path")
done
if (( ${#existing[@]} == 0 )); then
  printf 'No test directory exists for group %s\n' "$group" >&2
  exit 2
fi

exec python -m pytest "${existing[@]}" ${extra+"${extra[@]}"} ${1+"$@"}
