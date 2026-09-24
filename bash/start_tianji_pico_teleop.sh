#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
pico_scripts="$repo_root/src/teleop_inputs/pico_controller/scripts"
session_name="pico_tianji_teleop"
mode="attach"
mode_option=""
calibration_dir=""
pico_world_x_offset=""
launcher_arguments=("$@")

usage() {
  cat <<'EOF'
Usage: ./bash/start_tianji_pico_teleop.sh [--foreground | --detach] [--calibration-dir ABS_DIR]
       ./bash/start_tianji_pico_teleop.sh --status|--stop|--help

Start the PICO-side Tianji teleoperation pipeline; --foreground streams all node logs.
The Tianji Franka DLS/Ruckig viewer is not started by this script.

Options:
  --foreground
            Own the input processes in this terminal, without tmux; Ctrl+C stops them.
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
    --foreground|--detach|--status|--stop|--help|-h)
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

if [[ ( -n "$calibration_dir" || -n "$pico_world_x_offset" ) && "$mode" != "attach" && "$mode" != "detach" && "$mode" != "foreground" ]]; then
  echo "--calibration-dir cannot be combined with --$mode." >&2
  exit 2
fi
if [[ "$mode" == "help" ]]; then
  usage
  exit 0
fi

# Direct callers receive the same logger as run_pico.sh; inherited runs stay shared.
if [[ ( "$mode" == "attach" || "$mode" == "detach" || "$mode" == "foreground" ) && -z "${TIANJI_RUN_LOG_DIR:-}" ]]; then
  exec python3 "$repo_root/src/teleop_outputs/tianji/tianji_controller/tianji_controller/run_logging.py" \
    --log-root "$repo_root/tmp/logs/pico" -- \
    bash "$repo_root/bash/start_tianji_pico_teleop.sh" "${launcher_arguments[@]}"
fi

if [[ -n "$calibration_dir" ]]; then
  if [[ ! -d "$calibration_dir" ]]; then
    echo "Calibration directory does not exist: $calibration_dir" >&2
    exit 2
  fi
  calibration_dir="$(cd -- "$calibration_dir" && pwd -P)"
fi


if [[ "$mode" != "foreground" ]]; then
  command -v tmux >/dev/null || {
    echo "tmux not found; the background input mode requires tmux." >&2
    exit 2
  }
fi

session_exists() {
  command -v tmux >/dev/null && tmux has-session -t "$session_name" 2>/dev/null
}

report_session_logs() {
  local existing_log_dir
  existing_log_dir="$(tmux show-options -t "$session_name" -v @tianji_run_log_dir 2>/dev/null || true)"
  if [[ -n "$existing_log_dir" ]]; then
    echo "Existing PICO session logs: $existing_log_dir"
  else
    echo "Existing PICO session has no recorded log directory; logging was not changed."
  fi
}

if [[ "$mode" == "status" ]]; then
  if ! session_exists; then
    echo "Tianji PICO session is not running: $session_name" >&2
    exit 1
  fi
  tmux list-windows -t "$session_name"
  report_session_logs
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

if session_exists; then
  echo "Existing session $session_name; stop it explicitly before starting another input." >&2
  if [[ "$mode" == "foreground" ]]; then
    echo "先在终端 4 停止真机执行器，再显式运行 bash bash/run_stop_pico.sh；不会接管或自动停止旧会话。" >&2
  fi
  report_session_logs >&2
  exit 2
fi

export TIANJI_RUN_LOG_DIR
TIANJI_RUN_LOG_DIR="$(cd -- "$TIANJI_RUN_LOG_DIR" && pwd -P)"
export PYTHONUNBUFFERED=1

source "$repo_root/bash/environment.sh"
calibration_sha256=""
if [[ -n "$calibration_dir" ]]; then
  for side in left right; do
    if ! "$TIANJI_PYTHON" "$pico_scripts/pico_calibration_artifact.py" \
        validate --path "$calibration_dir/pico_${side}_arm_geometry.yaml" \
        --kind geometry --side "$side" \
        --tcp-path "$calibration_dir/pico_${side}_palm_tcp.yaml" \
        --wrist-path "$calibration_dir/pico_${side}_wrist_pivot.yaml" >/dev/null; then
      echo "Invalid $side calibration chain in: $calibration_dir; existing session unchanged." >&2
      exit 2
    fi
  done
  echo "PICO calibration directory: $calibration_dir"
  calibration_sha256="$("$TIANJI_PYTHON" "$pico_scripts/ensure_pico_user.py" --fingerprint "$calibration_dir")"
fi

command -v adb >/dev/null || {
  echo "adb not found; install Android platform tools first." >&2
  exit 2
}
if [[ ! -f "$repo_root/install/${TIANJI_ENVIRONMENT:-default}/local_setup.bash" ]]; then
  echo "PICO workspace is not built: missing $repo_root/install/${TIANJI_ENVIRONMENT:-default}/local_setup.bash" >&2
  echo "Run: bash $repo_root/bash/build.sh" >&2
  exit 2
fi
if [[ "$(adb get-state 2>/dev/null || true)" != "device" ]]; then
  echo "No authorized PICO device detected by adb." >&2
  echo "Connect the headset over USB, accept USB debugging, then run adb devices." >&2
  exit 2
fi

# Read-only conflict check. Never terminate processes by their executable name.
"$TIANJI_PYTHON" "$pico_scripts/cleanup_tianji_pico_processes.py"

printf -v repo_quoted '%q' "$repo_root"
# Pin this launch's activated environment: tmux may have been started elsewhere.
ros_environment="unset ROS_LOCALHOST_ONLY; export"
for variable in CONDA_PREFIX PATH LD_LIBRARY_PATH PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE RMW_IMPLEMENTATION TIANJI_WORKSPACE TIANJI_ENVIRONMENT TIANJI_PYTHON TIANJI_RUN_LOG_DIR PYTHONUNBUFFERED DISPLAY XAUTHORITY; do
  printf -v value_quoted '%q' "${!variable-}"
  ros_environment+=" $variable=$value_quoted"
done
ros_environment+=" ROS2CLI_DISABLE_DAEMON=1"
driver_inner="cd $repo_quoted && $ros_environment && exec ./bash/start_pico_driver.sh"
m0_inner="cd $repo_quoted && $ros_environment && exec ./bash/start_pico_m0.sh --viewer"
if [[ -n "$calibration_dir" ]]; then
  printf -v calibration_dir_quoted '%q' "$calibration_dir"
  m0_inner+=" --calibration-dir $calibration_dir_quoted"
fi
bridge_inner="cd $repo_quoted && $ros_environment && source bash/environment.sh && exec ros2 launch tianji_cmd_pub pico_arm_input.launch.py position_retargeting_mode:=robot_arm_segments robot_arm_reach_scale:=0.95"
if [[ -n "$pico_world_x_offset" ]]; then
  bridge_inner+=" pico_world_x_offset_m:=$pico_world_x_offset"
fi

# Refuse to replace historical output, even if a caller supplies an old run directory.
driver_log="$TIANJI_RUN_LOG_DIR/driver.log"
m0_log="$TIANJI_RUN_LOG_DIR/skeleton-calibration-viewer.log"
bridge_log="$TIANJI_RUN_LOG_DIR/arm-input-publisher.log"
for window_name in driver m0 bridge; do
  command_variable="${window_name}_inner"
  log_variable="${window_name}_log"
  (set -o noclobber; printf 'PICO process: %s\nCommand: %s\n\n' \
    "$window_name" "${!command_variable}" >"${!log_variable}")
done

if [[ "$mode" == "foreground" ]]; then
  echo "PICO 前台跟踪：本终端持续显示 driver / m0 / bridge 日志，不创建 tmux 会话。"
  echo "右手柄 A：Set Ground；请等待地面锁定提示。真机使能后不要重设跟踪坐标。"
  echo "停止顺序：先在终端 4 停止真机执行器，再在本终端按 Ctrl+C 停止 PICO。"
  foreground_arguments=(
    --checkout "$repo_root" --log-dir "$TIANJI_RUN_LOG_DIR"
    --driver "$driver_inner" --m0 "$m0_inner" --bridge "$bridge_inner"
  )
  if [[ -n "$calibration_dir" ]]; then
    foreground_arguments+=(--calibration-dir "$calibration_dir" --calibration-sha256 "$calibration_sha256")
  fi
  exec "$TIANJI_PYTHON" "$pico_scripts/pico_foreground.py" "${foreground_arguments[@]}"
fi

start_logged_pane() {
  local window="$1" command="$2" log_path="$3"
  local log_quoted pipe_command pipe_quoted command_quoted pane_command
  printf -v log_quoted '%q' "$log_path"
  pipe_command="exec cat >> $log_quoted"
  printf -v pipe_quoted '%q' "$pipe_command"
  printf -v command_quoted '%q' "$command"
  # Establish the pipe in the final pane, before any node can print or fail.
  # tmux owns the pipe after this launcher exits; output still reaches the PTY.
  pane_command="tmux pipe-pane -O -t \"\$TMUX_PANE\" $pipe_quoted && exec bash --noprofile --norc -c $command_quoted"
  tmux set-option -w -t "$session_name:$window" @tianji_log_path "$log_path"
  tmux set-option -w -t "$session_name:$window" @tianji_command "$command"
  tmux set-option -w -t "$session_name:$window" remain-on-exit on >/dev/null
  # Execute argv directly: typing long environments through the PTY can truncate them.
  tmux respawn-pane -k -t "$session_name:$window" bash --noprofile --norc -c "$pane_command"
}

window_name="driver"
if [[ -n "${TIANJI_PICO_SIM_OWNER:-}" ]]; then
  [[ "$TIANJI_PICO_SIM_OWNER" =~ ^[0-9a-f]{32}$ ]] || { echo "Invalid simulation owner token" >&2; exit 2; }
  tmux new-session -d -s "$session_name" -n "$window_name" \; \
    set-option -t "$session_name" @tianji_sim_owner "$TIANJI_PICO_SIM_OWNER" \; \
    set-option -t "$session_name" @tianji_checkout "$repo_root"
else
  tmux new-session -d -s "$session_name" -n "$window_name"
fi
tmux set-option -t "$session_name" @tianji_checkout "$repo_root"
tmux set-option -t "$session_name" @tianji_calibration_dir "$calibration_dir"
tmux set-option -t "$session_name" @tianji_calibration_sha256 "$calibration_sha256"
tmux set-option -t "$session_name" @tianji_run_log_dir "$TIANJI_RUN_LOG_DIR"
start_logged_pane "$window_name" "$driver_inner" "$driver_log"

window_name="m0"
tmux new-window -t "$session_name" -n "$window_name"
start_logged_pane "$window_name" "$m0_inner" "$m0_log"

window_name="bridge"
tmux new-window -t "$session_name" -n "$window_name"
start_logged_pane "$window_name" "$bridge_inner" "$bridge_log"
tmux select-window -t "$session_name:driver"

if ! "$TIANJI_PYTHON" "$pico_scripts/check_pico_session_ready.py" \
    --session "$session_name" --checkout "$repo_root" --timeout-s 30; then
  echo "PICO startup failed; panes retained for diagnosis. No automatic cleanup was performed." >&2
  # Preserve the failing pane before a later explicit stop removes the session.
  # Diagnostic failures must not hide the original startup failure.
  if diagnostic_dir="$(mktemp -d "$TIANJI_RUN_LOG_DIR/startup-failure.XXXXXX")"; then
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
echo "PICO pane logs: $TIANJI_RUN_LOG_DIR"
echo "Detach with Ctrl-b d; stop from project root with: pixi run stop-pico"

if [[ "$mode" == "detach" ]]; then
  exit 0
fi
exec tmux attach-session -t "$session_name"
