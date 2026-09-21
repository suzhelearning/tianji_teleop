#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
session_name="pico_tianji_teleop"
mode="attach"
mode_option=""
calibration_dir=""
pico_world_x_offset=""

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
  --pico-world-x-offset METERS
            Explicit bridge X offset; omitted preserves the original 0.10 m.
  --status  Show the managed tmux session and its windows.
  --stop    Stop only the managed pico_tianji_teleop session.
  --help    Show this help text.
EOF
}

while (($#)); do
  case "$1" in
    --pico-world-x-offset)
      if [[ -n "$pico_world_x_offset" ]] || (($# < 2)) || [[ ! "$2" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
        echo "--pico-world-x-offset requires one decimal value (no duplicates)." >&2; exit 2
      fi
      pico_world_x_offset="$2"
      shift 2
      ;;
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

if [[ ( -n "$calibration_dir" || -n "$pico_world_x_offset" ) && "$mode" != "attach" && "$mode" != "detach" ]]; then
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
    owner="$(tmux show-options -t "$session_name" -v @tianji_checkout 2>/dev/null || true)"
    if [[ "$owner" != "$repo_root" ]]; then
      echo 'Session ownership is unknown or belongs to another checkout; refusing to stop it.' >&2
      exit 2
    fi
    tmux kill-session -t "$session_name"
    echo "Stopped Tianji PICO session: $session_name"
  else
    echo "Tianji PICO session is already stopped: $session_name"
  fi
  exit 0
fi

source "$repo_root/scripts/environment.sh"
calibration_sha256=""
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
  calibration_sha256="$("$TIANJI_PYTHON" "$repo_root/../scripts/ensure_pico_user.py" --fingerprint "$calibration_dir")"
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
  echo "Existing session $session_name; stop it explicitly before starting another input." >&2
  exit 2
fi
# Read-only conflict check. Never terminate processes by their executable name.
"$TIANJI_PYTHON" "$repo_root/scripts/cleanup_tianji_pico_processes.py"

printf -v repo_quoted '%q' "$repo_root"
ros_environment="export ROS_DOMAIN_ID=120 EXO_REQUESTED_ROS_DOMAIN_ID=120 ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1"
# An existing tmux server can retain a different Python/ROS environment. Pin
# this launch's SDK in each child command instead of modifying the server.
printf -v python_quoted '%q' "$TIANJI_PYTHON"
printf -v setup_quoted '%q' "$ros_setup"
ros_environment+=" TIANJI_PYTHON=$python_quoted ROS_SETUP=$setup_quoted"
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  printf -v conda_quoted '%q' "$CONDA_PREFIX"
  ros_environment+=" CONDA_PREFIX=$conda_quoted"
fi
driver_inner="cd $repo_quoted && $ros_environment && exec ./scripts/start_pico_driver.sh"
m0_inner="cd $repo_quoted && $ros_environment && exec ./scripts/start_pico_m0.sh --viewer"
if [[ -n "$calibration_dir" ]]; then
  printf -v calibration_dir_quoted '%q' "$calibration_dir"
  m0_inner+=" --calibration-dir $calibration_dir_quoted"
fi
bridge_inner="cd $repo_quoted && $ros_environment && source scripts/environment.sh && exec ros2 launch pico_bridge start_tianji_mujoco_teleop.launch.py destination_address:=127.0.0.1 destination_port:=15000 position_retargeting_mode:=robot_arm_segments robot_arm_reach_scale:=0.95"
if [[ -n "$pico_world_x_offset" ]]; then
  bridge_inner+=" pico_world_x_offset_m:=$pico_world_x_offset"
fi
printf -v driver_inner_quoted '%q' "$driver_inner"
printf -v m0_inner_quoted '%q' "$m0_inner"
printf -v bridge_inner_quoted '%q' "$bridge_inner"
driver_command="exec bash --noprofile --norc -c $driver_inner_quoted"
m0_command="exec bash --noprofile --norc -c $m0_inner_quoted"
bridge_command="exec bash --noprofile --norc -c $bridge_inner_quoted"

window_name="driver"
if [[ -n "${TIANJI_PICO_SIM_OWNER:-}" ]]; then
  tmux new-session -d -s "$session_name" -n "$window_name" \; \
    set-option -t "$session_name" @tianji_sim_owner "$TIANJI_PICO_SIM_OWNER" \; \
    set-option -t "$session_name" @tianji_checkout "$repo_root"
else
  tmux new-session -d -s "$session_name" -n "$window_name"
fi
tmux set-option -t "$session_name" @tianji_checkout "$repo_root"
tmux set-option -t "$session_name" @tianji_calibration_dir "$calibration_dir"
tmux set-option -t "$session_name" @tianji_calibration_sha256 "$calibration_sha256"
tmux set-option -w -t "$session_name:$window_name" remain-on-exit on >/dev/null
tmux send-keys -t "$session_name:$window_name" "$driver_command" C-m

window_name="m0"
tmux new-window -t "$session_name" -n "$window_name"
tmux set-option -w -t "$session_name:$window_name" remain-on-exit on >/dev/null
tmux send-keys -t "$session_name:$window_name" "$m0_command" C-m

window_name="bridge"
tmux new-window -t "$session_name" -n "$window_name"
tmux set-option -w -t "$session_name:$window_name" remain-on-exit on >/dev/null
tmux send-keys -t "$session_name:$window_name" "$bridge_command" C-m
tmux select-window -t "$session_name:driver"

if ! ROS_DOMAIN_ID=120 "$TIANJI_PYTHON" "$repo_root/scripts/check_pico_session_ready.py" \
    --session "$session_name" --checkout "$repo_root" --timeout-s 30; then
  echo "PICO startup failed; panes retained for diagnosis. No automatic cleanup was performed." >&2
  # Preserve the failing pane before a later explicit stop removes the session.
  # Diagnostic failures must not hide the original startup failure.
  if diagnostic_dir="$(mktemp -d "${TMPDIR:-/tmp}/pico-startup.XXXXXX")"; then
    for window_name in driver m0 bridge; do
      tmux capture-pane -p -t "$session_name:$window_name" -S -200 \
        >"$diagnostic_dir/$window_name.log" 2>&1 || true
      echo "PICO $window_name output (last 30 lines):" >&2
      tail -n 30 "$diagnostic_dir/$window_name.log" >&2 || true
    done
    echo "PICO startup diagnostic logs: $diagnostic_dir" >&2
  fi
  exit 2
fi

echo "Started Tianji PICO tmux session: $session_name"
echo "Windows: driver, m0, bridge"
echo "Detach with Ctrl-b d; stop from project root with: pixi run -e tracking stop-pico"

if [[ "$mode" == "detach" ]]; then
  exit 0
fi
exec tmux attach-session -t "$session_name"
