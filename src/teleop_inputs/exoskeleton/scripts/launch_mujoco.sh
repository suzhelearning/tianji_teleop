#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DATAGLOVE_PIXI_BIN="${DATAGLOVE_PIXI_BIN:-pixi}"
GENERATION="v1"
HAND=""
SIMULATOR_ARGS=()

usage() {
  cat <<'EOF'
用法：
  pixi run mujoco-v1-left
  pixi run mujoco-v1-right
  pixi run mujoco-v1-both
  pixi run mujoco-v2-left
  pixi run mujoco-v2-right
  pixi run mujoco-v2-both
  bash scripts/launch_mujoco.sh \
    --generation v1|v2 --hand left|right|both -- [查看器参数]

单手环境变量：
  DATAGLOVE_HOST、DATAGLOVE_PORT、DATAGLOVE_CAL_FILE、DATAGLOVE_ZMQ_PORT

双手环境变量：
  DATAGLOVE_LEFT_HOST、DATAGLOVE_RIGHT_HOST（必须显式设置）
  DATAGLOVE_LEFT_PORT、DATAGLOVE_RIGHT_PORT
  DATAGLOVE_LEFT_CAL_FILE、DATAGLOVE_RIGHT_CAL_FILE
  DATAGLOVE_LEFT_ZMQ_PORT、DATAGLOVE_RIGHT_ZMQ_PORT

该脚本只启动数据手套桥和 MuJoCo，不启动 Wuji 真机。
EOF
}

while (($#)); do
  case "$1" in
    --generation)
      [[ $# -ge 2 ]] || { echo "错误：--generation 缺少参数" >&2; exit 2; }
      GENERATION="$2"
      shift 2
      ;;
    --hand)
      [[ $# -ge 2 ]] || { echo "错误：--hand 缺少参数" >&2; exit 2; }
      HAND="$2"
      shift 2
      ;;
    --)
      shift
      SIMULATOR_ARGS=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "错误：未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$GENERATION" != "v1" && "$GENERATION" != "v2" ]]; then
  echo "错误：--generation 必须是 v1 或 v2" >&2
  exit 2
fi
if [[ "$HAND" != "left" && "$HAND" != "right" && "$HAND" != "both" ]]; then
  echo "错误：--hand 必须是 left、right 或 both" >&2
  exit 2
fi
if ! command -v "$DATAGLOVE_PIXI_BIN" >/dev/null 2>&1; then
  echo "错误：未找到 pixi：$DATAGLOVE_PIXI_BIN" >&2
  exit 1
fi
if ! command -v setsid >/dev/null 2>&1; then
  echo "错误：未找到 setsid，无法可靠管理仿真子进程" >&2
  exit 1
fi

calibration_path() {
  local hand="$1"
  printf '%s/config/dataglove/calibration/%s.json\n' "$PROJECT_DIR" "$hand"
}

mapping_path() {
  local generation="$1"
  local hand="$2"
  printf '%s/config/hands/wuji_%s/%s_mapping.json\n' \
    "$PROJECT_DIR" "$generation" "$hand"
}

check_hand_files() {
  local generation="$1"
  local hand="$2"
  local calibration="$3"
  local mapping
  mapping="$(mapping_path "$generation" "$hand")"
  if [[ ! -f "$calibration" ]]; then
    echo "错误：${hand} 标定文件不存在：$calibration" >&2
    echo "请先运行 pixi run calibrate-${hand}" >&2
    exit 1
  fi
  if [[ ! -f "$mapping" ]]; then
    echo "错误：${hand} 映射文件不存在：$mapping" >&2
    exit 1
  fi
}

declare -a child_pids=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  local process_id
  for process_id in "${child_pids[@]}"; do
    kill -TERM -- "-$process_id" 2>/dev/null || true
  done
  for process_id in "${child_pids[@]}"; do
    wait "$process_id" 2>/dev/null || true
  done
  exit "$status"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

start_simulator() {
  local hand="$1"
  local zmq_port="$2"
  setsid --wait "$DATAGLOVE_PIXI_BIN" run python \
    -m "data_glove_wuji_teleop.adapters.simulation.mujoco_wuji_${GENERATION}" \
    --hand "$hand" \
    --zmq-host 127.0.0.1 \
    --port "$zmq_port" \
    --config "$(mapping_path "$GENERATION" "$hand")" \
    "${SIMULATOR_ARGS[@]}" &
  child_pids+=("$!")
}

cd "$PROJECT_DIR"

case "$HAND" in
  left)
    default_glove_host="192.168.7.2"
    default_zmq_port="15559"
    ;;
  right)
    default_glove_host="192.168.8.2"
    default_zmq_port="15558"
    ;;
  both)
    default_glove_host=""
    default_zmq_port=""
    ;;
esac

if [[ "$HAND" == "left" || "$HAND" == "right" ]]; then
  hand="$HAND"
  glove_host="${DATAGLOVE_HOST:-$default_glove_host}"
  glove_port="${DATAGLOVE_PORT:-5580}"
  calibration="${DATAGLOVE_CAL_FILE:-$(calibration_path "$hand")}"
  zmq_port="${DATAGLOVE_ZMQ_PORT:-$default_zmq_port}"
  check_hand_files "$GENERATION" "$hand" "$calibration"

  echo "启动 Wuji ${GENERATION} ${hand} 数据手套 MuJoCo"
  setsid --wait "$DATAGLOVE_PIXI_BIN" run python \
    -m data_glove_wuji_teleop.cli run \
    --hand "$hand" \
    --cal-file "$calibration" \
    --host "$glove_host" \
    --port "$glove_port" \
    --zmq-bind 127.0.0.1 \
    --zmq-port "$zmq_port" &
  child_pids+=("$!")
  start_simulator "$hand" "$zmq_port"
else
  left_host="${DATAGLOVE_LEFT_HOST:-}"
  right_host="${DATAGLOVE_RIGHT_HOST:-}"
  if [[ -z "$left_host" || -z "$right_host" ]]; then
    echo "错误：both 模式必须设置 DATAGLOVE_LEFT_HOST 和 DATAGLOVE_RIGHT_HOST" >&2
    exit 1
  fi
  left_port="${DATAGLOVE_LEFT_PORT:-5580}"
  right_port="${DATAGLOVE_RIGHT_PORT:-5580}"
  left_cal="${DATAGLOVE_LEFT_CAL_FILE:-$(calibration_path left)}"
  right_cal="${DATAGLOVE_RIGHT_CAL_FILE:-$(calibration_path right)}"
  left_zmq="${DATAGLOVE_LEFT_ZMQ_PORT:-15559}"
  right_zmq="${DATAGLOVE_RIGHT_ZMQ_PORT:-15558}"
  check_hand_files "$GENERATION" left "$left_cal"
  check_hand_files "$GENERATION" right "$right_cal"

  echo "启动 Wuji ${GENERATION} 双手数据手套 MuJoCo"
  setsid --wait "$DATAGLOVE_PIXI_BIN" run python \
    -m data_glove_wuji_teleop.cli run \
    --hand both \
    --left-cal-file "$left_cal" \
    --right-cal-file "$right_cal" \
    --left-host "$left_host" \
    --right-host "$right_host" \
    --left-port "$left_port" \
    --right-port "$right_port" \
    --zmq-bind 127.0.0.1 \
    --left-zmq-port "$left_zmq" \
    --right-zmq-port "$right_zmq" &
  child_pids+=("$!")
  start_simulator left "$left_zmq"
  start_simulator right "$right_zmq"
fi

echo "子进程：${child_pids[*]}"
echo "关闭任一 MuJoCo 窗口或按 Ctrl+C 可停止整组进程。"

set +e
wait -n "${child_pids[@]}"
status=$?
set -e
exit "$status"
