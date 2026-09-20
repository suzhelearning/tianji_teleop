#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$script_dir"
while [[ "$repo_root" != / && ! -f "$repo_root/scripts/environment.sh" ]]; do
  repo_root="$(dirname "$repo_root")"
done
if [[ "$repo_root" == / ]]; then
  echo "Cannot locate tracking/scripts/environment.sh" >&2
  exit 2
fi
source "$script_dir/pico_process_cleanup.sh"
domain="120"
left_tcp="$HOME/.config/pico_tracker/pico_left_palm_tcp.yaml"
right_tcp="$HOME/.config/pico_tracker/pico_right_palm_tcp.yaml"
left_wrist="$HOME/.config/pico_tracker/pico_left_wrist_pivot.yaml"
right_wrist="$HOME/.config/pico_tracker/pico_right_wrist_pivot.yaml"
left_geometry=""
right_geometry=""
require_left_geometry=false
require_right_geometry=false
viewer=false
record=false
duration="0"
record_output=""
readiness_timeout="20"


usage() {
  cat >&2 <<'EOF'
usage: start_pico_m0_runtime.sh [options]
  --domain ID
  --left-tcp FILE --right-tcp FILE
  --left-wrist FILE --right-wrist FILE
  --left-geometry FILE --right-geometry FILE
  --require-left-geometry --require-right-geometry
  --viewer --record [--record-output DIR] [--duration SECONDS]
  --readiness-timeout SECONDS
EOF
}

while (($#)); do
  case "$1" in
    --domain) domain="${2:-}"; shift 2 ;;
    --left-tcp) left_tcp="${2:-}"; shift 2 ;;
    --right-tcp) right_tcp="${2:-}"; shift 2 ;;
    --left-wrist) left_wrist="${2:-}"; shift 2 ;;
    --right-wrist) right_wrist="${2:-}"; shift 2 ;;
    --left-geometry) left_geometry="${2:-}"; shift 2 ;;
    --right-geometry) right_geometry="${2:-}"; shift 2 ;;
    --require-left-geometry) require_left_geometry=true; shift ;;
    --require-right-geometry) require_right_geometry=true; shift ;;
    --viewer) viewer=true; shift ;;
    --record) record=true; shift ;;
    --record-output) record_output="${2:-}"; shift 2 ;;
    --duration) duration="${2:-}"; shift 2 ;;
    --readiness-timeout) readiness_timeout="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if ! domain="$(pico_normalize_ros_domain "$domain")"; then
  echo "--domain must be an integer between 0 and 232" >&2
  exit 2
fi

for artifact in "$left_tcp" "$right_tcp" "$left_wrist" "$right_wrist"; do
  if [[ -z "$artifact" || ! -r "$artifact" ]]; then
    echo "artifact missing or unreadable: $artifact" >&2
    exit 2
  fi
done
if [[ "$require_left_geometry" == true && ( -z "$left_geometry" || ! -r "$left_geometry" ) ]]; then
  echo "required left geometry artifact missing or unreadable: $left_geometry" >&2
  exit 2
fi
if [[ "$require_right_geometry" == true && ( -z "$right_geometry" || ! -r "$right_geometry" ) ]]; then
  echo "required right geometry artifact missing or unreadable: $right_geometry" >&2
  exit 2
fi
for artifact in "$left_geometry" "$right_geometry"; do
  if [[ -n "$artifact" && ! -r "$artifact" ]]; then
    echo "geometry artifact missing or unreadable: $artifact" >&2
    exit 2
  fi
done
if ! [[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] || ! [[ "$readiness_timeout" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "--duration and --readiness-timeout must be non-negative numbers" >&2
  exit 2
fi


cd "$repo_root"
source "$repo_root/scripts/environment.sh"
export ROS_DOMAIN_ID="$domain"
export EXO_REQUESTED_ROS_DOMAIN_ID="$domain"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"

lock_file="${XDG_RUNTIME_DIR:-${TMPDIR:-/tmp}}/pico_m0_domain_${domain}.lock"
exec {pico_m0_lock_fd}>"$lock_file"
if ! flock -n "$pico_m0_lock_fd"; then
  echo "PICO M0 is already running in ROS domain $domain; stop the existing instance first" >&2
  exit 2
fi


python - <<'PY'
import os
import sys
import time
from pathlib import Path

import rclpy


def reject_duplicate(message):
    print(message, file=sys.stderr)
    raise SystemExit(2)


target_domain = os.environ.get("ROS_DOMAIN_ID", "0")
existing_pids = []
for process_dir in Path("/proc").glob("[0-9]*"):
    try:
        arguments = (process_dir / "cmdline").read_bytes().split(b"\0")
        if not any(
            Path(argument.decode(errors="ignore")).name
            == "pico_palm_skeleton_filter"
            for argument in arguments
            if argument
        ):
            continue
        environment = {}
        for entry in (process_dir / "environ").read_bytes().split(b"\0"):
            if b"=" not in entry:
                continue
            name, value = entry.split(b"=", 1)
            environment[name.decode(errors="ignore")] = value.decode(errors="ignore")
        if environment.get("ROS_DOMAIN_ID", "0") == target_domain:
            existing_pids.append(process_dir.name)
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue

if existing_pids:
    reject_duplicate(
        f"PICO M0 is already running in ROS domain {target_domain}: "
        f"pico_palm_skeleton_filter pid(s)={','.join(existing_pids)}; "
        "stop the existing instance first"
    )

node = None
try:
    rclpy.init()
    node = rclpy.create_node(f"pico_m0_singleton_probe_{os.getpid()}")
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        names = node.get_node_names_and_namespaces()
        if any(name == "pico_palm_skeleton_filter" for name, _namespace in names):
            reject_duplicate(
                "PICO M0 is already running: ROS graph contains "
                "pico_palm_skeleton_filter; stop the existing instance first"
            )
finally:
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY

child_pids=()
filter_pid=""
viewer_pid=""
recorder_pid=""
cleanup_done=false
cleanup() {
  local graceful_attempts
  local pid
  if [[ "$cleanup_done" == true ]]; then
    return
  fi
  cleanup_done=true
  trap - EXIT INT TERM
  for ((idx=${#child_pids[@]}-1; idx>=0; idx--)); do
    pid="${child_pids[$idx]}"
    graceful_attempts=20
    if [[ -n "$recorder_pid" && "$pid" == "$recorder_pid" ]]; then
      # The comparison recorder compresses and atomically renames its NPZ on
      # SIGINT. Give it the former 30-second save window before escalation.
      graceful_attempts=600
    fi
    pico_stop_process_group "$pid" "$graceful_attempts" 10
  done
}
trap cleanup EXIT INT TERM

launch_args=(
  "left_tcp_artifact:=$left_tcp"
  "right_tcp_artifact:=$right_tcp"
  "left_wrist_pivot_artifact:=$left_wrist"
  "right_wrist_pivot_artifact:=$right_wrist"
  # A side with strict quick geometry is already required by the node to have
  # its matching wrist artifact.  Keep unselected sides optional so one valid
  # arm can run while the other remains on the raw-SMPL baseline.
  "require_wrist_pivot_artifact:=false"
)
if [[ -n "$left_geometry" ]]; then
  launch_args+=(
    "left_arm_geometry_artifact:=$left_geometry"
    "require_left_arm_geometry_artifact:=$require_left_geometry"
  )
fi
if [[ -n "$right_geometry" ]]; then
  launch_args+=(
    "right_arm_geometry_artifact:=$right_geometry"
    "require_right_arm_geometry_artifact:=$require_right_geometry"
  )
fi
setsid ros2 launch pico_bridge start_pico_palm_skeleton_filter.launch.py "${launch_args[@]}" &
filter_pid=$!
child_pids+=("$filter_pid")

python "$script_dir/pico_m0_readiness.py" \
  --timeout-s "$readiness_timeout"

session_log_dir=""
if [[ "$viewer" == true || "$record" == true ]]; then
  session_log_dir="$(mktemp -d "${TMPDIR:-/tmp}/pico_m0.XXXXXX")"
  echo "runtime logs: $session_log_dir"
fi

if [[ "$record" == true ]]; then
  if [[ -z "$record_output" ]]; then
    record_output="$repo_root/recordings/pico_m0_$(date +%Y%m%d_%H%M%S).npz"
  fi
  if [[ -e "$record_output" ]]; then
    echo "record output already exists: $record_output" >&2
    exit 2
  fi
  setsid ros2 run pico_bridge pico_m0_comparison_report record --output "$record_output" \
    >"$session_log_dir/recorder.log" 2>&1 &
  recorder_pid=$!
  child_pids+=("$recorder_pid")
  echo "recording: $record_output"
fi

if [[ "$viewer" == true ]]; then
  setsid ros2 run pico_bridge smpl_mujoco_visualizer \
    --topic /pico/smpl_palm_corrected_ik \
    --show-raw --raw-topic /pico/smpl_raw \
    --palm-topic /pico/palm_left \
    --right-palm-topic /pico/palm_right \
    --show-arm-axes --show-controllers \
    --raw-offset 0 0 0 --rate 60 --timeout 1.0 \
    >"$session_log_dir/viewer.log" 2>&1 &
  viewer_pid=$!
  child_pids+=("$viewer_pid")
  sleep 1
  if ! kill -0 "$viewer_pid" 2>/dev/null; then
    echo "viewer exited; M0 startup failed" >&2
    tail -n 20 "$session_log_dir/viewer.log" >&2 || true
    exit 2
  fi
fi

echo "PICO bilateral M0 ready (domain=$domain, active control disabled)"
if [[ "$duration" != "0" && "$duration" != "0.0" ]]; then
  sleep "$duration"
  exit 0
fi

if [[ "$viewer" == true ]]; then
  # Either child exiting invalidates the required input+visualization session.
  wait -n "$filter_pid" "$viewer_pid" || true
  echo "M0 filter or required skeleton viewer exited; session no longer ready" >&2
  tail -n 20 "$session_log_dir/viewer.log" >&2 || true
  exit 2
fi
wait "$filter_pid"
