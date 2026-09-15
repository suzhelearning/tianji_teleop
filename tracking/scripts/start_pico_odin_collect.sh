#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${SESSION:-pico_odin_collect}"
ODIN_IMPL="${ODIN_IMPL:-original}"
ODIN_MAP="${ODIN_MAP:-}"
ODIN_MAP_INIT="${ODIN_MAP_INIT:-}"
ENABLE_ODIN_LOCAL_POSE="${ENABLE_ODIN_LOCAL_POSE:-true}"
COLLECT_CONFIG="${COLLECT_CONFIG:-src/data_collector/config/collect_config.yaml}"
START_PICO_BRIDGE=true
START_FISHEYE=true

while [[ $# -gt 0 ]]; do
  case "$1" in
    --odin-impl) ODIN_IMPL="$2"; shift 2 ;;
    --odin1|--original) ODIN_IMPL="original"; shift ;;
    --odin-lite|--lite) ODIN_IMPL="lite"; shift ;;
    --odin-map) ODIN_MAP="$2"; shift 2 ;;
    --odin-map-init) ODIN_MAP_INIT="$2"; shift 2 ;;
    --no-odin-local-pose) ENABLE_ODIN_LOCAL_POSE=false; shift ;;
    --config) COLLECT_CONFIG="$2"; shift 2 ;;
    --no-pico-bridge) START_PICO_BRIDGE=false; shift ;;
    --no-fisheye) START_FISHEYE=false; shift ;;
    --session) SESSION="$2"; shift 2 ;;
    --help|-h)
      sed -n '2,38p' "$0"
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

case "$ODIN_IMPL" in
  original|lite) ;;
  *) echo "--odin-impl must be original or lite" >&2; exit 1 ;;
esac

if [ -n "$ODIN_MAP" ] && [ ! -f "$ODIN_MAP" ]; then
  echo "Odin map does not exist: $ODIN_MAP" >&2
  exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is required" >&2
  exit 1
fi

source "$ROOT_DIR/scripts/environment.sh"
printf -v SETUP_CMD 'source %q' "$ROOT_DIR/scripts/environment.sh"

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n odin
tmux send-keys -t "${SESSION}:odin" "cd '${ROOT_DIR}' && ${SETUP_CMD}" C-m

ODIN_CMD="ros2 launch pico_odin odin_select.launch.py odin_impl:=${ODIN_IMPL} enable_local_pose:=${ENABLE_ODIN_LOCAL_POSE}"
if [ -n "$ODIN_MAP" ]; then
  ODIN_CMD="${ODIN_CMD} odin_map:='${ODIN_MAP}'"
fi
if [ -n "$ODIN_MAP_INIT" ]; then
  ODIN_CMD="${ODIN_CMD} odin_map_init:='${ODIN_MAP_INIT}'"
fi
tmux send-keys -t "${SESSION}:odin" "sleep 1 && ${ODIN_CMD}" C-m

if [ "$START_PICO_BRIDGE" = true ]; then
  tmux new-window -t "$SESSION" -n pico
  tmux send-keys -t "${SESSION}:pico" "cd '${ROOT_DIR}' && ${SETUP_CMD}" C-m
  tmux send-keys -t "${SESSION}:pico" "adb forward tcp:9999 tcp:9999 || true; ros2 launch pico_bridge start_pico_bridge.launch.py" C-m
fi

if [ "$START_FISHEYE" = true ]; then
  tmux new-window -t "$SESSION" -n fisheye
  tmux send-keys -t "${SESSION}:fisheye" "cd '${ROOT_DIR}' && ${SETUP_CMD}" C-m
  tmux send-keys -t "${SESSION}:fisheye" "sleep 2 && ros2 launch fisheye_camera fisheye_camera.launch.py" C-m
fi

tmux new-window -t "$SESSION" -n collector
tmux send-keys -t "${SESSION}:collector" "cd '${ROOT_DIR}' && ${SETUP_CMD}" C-m
tmux send-keys -t "${SESSION}:collector" "sleep 4 && ros2 run data_collector data_collector_node --config '${ROOT_DIR}/${COLLECT_CONFIG}'" C-m

tmux select-window -t "${SESSION}:collector"
echo "Started tmux session: ${SESSION}"
echo "Attach: tmux attach -t ${SESSION}"
