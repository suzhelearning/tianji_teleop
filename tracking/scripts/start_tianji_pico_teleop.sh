#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_name="pico_tianji_teleop"
mode="attach"
mode_option=""
calibration_dir=""

usage() {
  cat <<'EOF'
Usage: ./scripts/start_tianji_pico_teleop.sh [--detach] [--calibration-dir ABS_DIR]
       ./scripts/start_tianji_pico_teleop.sh --status|--stop|--help

Start and manage the PICO-side Tianji teleoperation pipeline in tmux.
The Tianji DLS/Spark MuJoCo viewer is not started by this script.

Options:
  --detach  Start the session without attaching to it.
  --calibration-dir ABS_DIR
            Require both arm calibrations from this directory; never use globals.
  --status  Show the managed tmux session and its windows.
  --stop    Stop only the managed pico_tianji_teleop session.
  --help    Show this help text.
EOF
}

while (($#)); do
  case "$1" in
    --detach|--status|--stop|--help|-h)
      if [[ -n "$mode_option" ]]; then
        echo "Duplicate or conflicting options: $mode_option and $1" >&2
        exit 2
      fi
      mode_option="$1"
      mode="${1#--}"
      [[ "$1" != "-h" ]] || mode="help"
      shift
      ;;
    --calibration-dir)
      if [[ -n "$calibration_dir" ]]; then
        echo "Duplicate option: --calibration-dir" >&2
        exit 2
      fi
      if (($# < 2)) || [[ "$2" != /* ]]; then
        echo "--calibration-dir requires an absolute directory path." >&2
        exit 2
      fi
      calibration_dir="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -n "$calibration_dir" && "$mode" != "attach" && "$mode" != "detach" ]]; then
  echo "--calibration-dir cannot be combined with --$mode." >&2
  exit 2
fi
if [[ "$mode" == "help" ]]; then
  usage
  exit 0
fi
if [[ -n "$calibration_dir" ]]; then
  if [[ ! -d "$calibration_dir" ]]; then
    echo "Calibration directory does not exist: $calibration_dir" >&2
    exit 2
  fi
  calibration_dir="$(cd -- "$calibration_dir" && pwd -P)"
fi


command -v tmux >/dev/null || {
  echo "tmux not found; install the system tmux package." >&2
  exit 2
}

session_exists() {
  tmux has-session -t "$session_name" 2>/dev/null
}

if [[ "$mode" == "status" ]]; then
  if ! session_exists; then
    echo "Tianji PICO session is not running: $session_name" >&2
    exit 1
  fi
  tmux list-windows -t "$session_name"
  exit 0
fi

if [[ "$mode" == "stop" ]]; then
  if session_exists; then
    tmux kill-session -t "$session_name"
    echo "Stopped Tianji PICO session: $session_name"
  else
    echo "Tianji PICO session is already stopped: $session_name"
  fi
  exit 0
fi

source "$repo_root/scripts/environment.sh"
if [[ -n "$calibration_dir" ]]; then
  for side in left right; do
    if ! python3 "$repo_root/src/pico_bridge/scripts/pico_calibration_artifact.py" \
        validate --path "$calibration_dir/pico_${side}_arm_geometry.yaml" \
        --kind geometry --side "$side" \
        --tcp-path "$calibration_dir/pico_${side}_palm_tcp.yaml" \
        --wrist-path "$calibration_dir/pico_${side}_wrist_pivot.yaml" >/dev/null; then
      echo "Invalid $side calibration chain in: $calibration_dir; existing session unchanged." >&2
      exit 2
    fi
  done
  echo "PICO calibration directory: $calibration_dir"
fi

command -v adb >/dev/null || {
  echo "adb not found; install Android platform tools first." >&2
  exit 2
}
if [[ ! -f "$repo_root/install/local_setup.bash" ]]; then
  echo "PICO workspace is not built: missing $repo_root/install/local_setup.bash" >&2
  echo "Run: bash $repo_root/scripts/build.sh" >&2
  exit 2
fi
if [[ "$(adb get-state 2>/dev/null || true)" != "device" ]]; then
  echo "No authorized PICO device detected by adb." >&2
  echo "Connect the headset over USB, accept USB debugging, then run adb devices." >&2
  exit 2
fi

if session_exists; then
  echo "Restarting Tianji PICO session: $session_name"
  tmux kill-session -t "$session_name"
fi
python3 "$repo_root/scripts/cleanup_tianji_pico_processes.py"

printf -v repo_quoted '%q' "$repo_root"
ros_environment="export ROS_DOMAIN_ID=120 EXO_REQUESTED_ROS_DOMAIN_ID=120 ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1"
driver_inner="cd $repo_quoted && $ros_environment && exec ./scripts/start_pico_driver.sh"
m0_inner="cd $repo_quoted && $ros_environment && exec ./scripts/start_pico_m0.sh --viewer"
if [[ -n "$calibration_dir" ]]; then
  printf -v calibration_dir_quoted '%q' "$calibration_dir"
  m0_inner+=" --calibration-dir $calibration_dir_quoted"
fi
bridge_inner="cd $repo_quoted && $ros_environment && source scripts/environment.sh && exec ros2 launch pico_bridge start_tianji_mujoco_teleop.launch.py destination_address:=127.0.0.1 destination_port:=15000 position_retargeting_mode:=robot_arm_segments robot_arm_reach_scale:=0.95"
printf -v driver_inner_quoted '%q' "$driver_inner"
printf -v m0_inner_quoted '%q' "$m0_inner"
printf -v bridge_inner_quoted '%q' "$bridge_inner"
driver_command="exec bash --noprofile --norc -c $driver_inner_quoted"
m0_command="exec bash --noprofile --norc -c $m0_inner_quoted"
bridge_command="exec bash --noprofile --norc -c $bridge_inner_quoted"

window_name="driver"
tmux new-session -d -s "$session_name" -n "$window_name"
tmux set-option -t "$session_name" remain-on-exit on >/dev/null
tmux send-keys -t "$session_name:$window_name" "$driver_command" C-m

window_name="m0"
tmux new-window -t "$session_name" -n "$window_name"
tmux send-keys -t "$session_name:$window_name" "$m0_command" C-m

window_name="bridge"
tmux new-window -t "$session_name" -n "$window_name"
tmux send-keys -t "$session_name:$window_name" "$bridge_command" C-m
tmux select-window -t "$session_name:driver"

echo "Started Tianji PICO tmux session: $session_name"
echo "Windows: driver, m0, bridge"
echo "Detach with Ctrl-b d; stop with ./scripts/stop_tianji_pico_teleop.sh"

if [[ "$mode" == "detach" ]]; then
  exit 0
fi
exec tmux attach-session -t "$session_name"
