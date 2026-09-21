#!/usr/bin/env bash
set -euo pipefail

repo_root="${TIANJI_WORKSPACE:?Run within the workspace Pixi environment}"
side=""
tcp_artifact=""
wrist_pivot_artifact=""
start_mode="space"
domain="42"
output_dir=""
countdown_s="3"
duration_scale="1.0"
verbose="0"


usage() {
  echo "usage: $0 --side left|right --tcp-artifact FILE --wrist-pivot-artifact FILE [--start-mode space|auto] [--domain ID] [--output-dir DIR]" >&2
}

while (($#)); do
  case "$1" in
    --side) side="${2:-}"; shift 2 ;;
    --tcp-artifact) tcp_artifact="${2:-}"; shift 2 ;;
    --wrist-pivot-artifact) wrist_pivot_artifact="${2:-}"; shift 2 ;;
    --start-mode) start_mode="${2:-}"; shift 2 ;;
    --domain) domain="${2:-}"; shift 2 ;;
    --output-dir) output_dir="${2:-}"; shift 2 ;;
    --countdown-s) countdown_s="${2:-}"; shift 2 ;;
    --duration-scale) duration_scale="${2:-}"; shift 2 ;;
    --verbose) verbose="1"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ "$side" != "left" && "$side" != "right" ]]; then
  echo "--side must be left or right" >&2
  exit 2
fi
if [[ "$start_mode" != "space" && "$start_mode" != "auto" ]]; then
  echo "--start-mode must be space or auto" >&2
  exit 2
fi
for artifact in "$tcp_artifact" "$wrist_pivot_artifact"; do
  if [[ -z "$artifact" || ! -r "$artifact" ]]; then
    echo "artifact missing or unreadable: $artifact" >&2
    exit 2
  fi
done


cd "$repo_root"
source "$repo_root/src/teleop_inputs/pico_bridge/scripts/pico_process_cleanup.sh"
source "$repo_root/bash/environment.sh"
export ROS_DOMAIN_ID="$domain"
export EXO_REQUESTED_ROS_DOMAIN_ID="$domain"


publisher_pid=""
cleanup() {
  local owned_pid="$publisher_pid"
  trap - EXIT INT TERM
  publisher_pid=""
  pico_stop_process_group "$owned_pid"
}
trap cleanup EXIT INT TERM

setsid ros2 run pico_bridge pico_palm_tcp_publisher \
  --side "$side" --artifact "$tcp_artifact" &
publisher_pid=$!

calibrator_args=(
  --side "$side"
  --tcp-artifact "$tcp_artifact"
  --wrist-pivot-artifact "$wrist_pivot_artifact"
  --start-mode "$start_mode"
  --countdown-s "$countdown_s"
  --duration-scale "$duration_scale"
)
if [[ -n "$output_dir" ]]; then
  calibrator_args+=(--output-dir "$output_dir")
fi
if [[ "$verbose" == "1" ]]; then
  calibrator_args+=(--verbose)
fi

ros2 run pico_bridge pico_arm_geometry_calibrator "${calibrator_args[@]}"
