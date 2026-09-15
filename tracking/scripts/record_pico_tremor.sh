#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$repo_root/src/pico_bridge/scripts/pico_process_cleanup.sh"

domain="120"
duration_s="30"
countdown_s="5"
readiness_timeout_s="30"
output_root=""
dry_run=false
caller_dir="$PWD"

stages=(fixed elbow_supported arm_unsupported)
stage_labels=("固定手柄" "肘部支撑" "自然悬臂")
stage_instructions=(
  "将左右手柄刚性固定在支架或桌面上，保持在头显可追踪范围内。"
  "自然握住左右手柄，将双肘或前臂稳定支撑，不要刻意对抗轻微抖动。"
  "采用实际遥操作姿势，双臂离开支撑并自然保持，不要主动修正细小晃动。"
)

record_topics=(
  /pico/pose/head
  /pico/pose/left_hand
  /pico/pose/right_hand
  /pico/palm_left
  /pico/palm_right
  /pico/smpl_raw
  /pico/smpl_palm_corrected
  /pico/smpl_palm_corrected_ik
  /pico/smpl_palm_corrected/status
  /pico/tracking_epoch
)

required_nonempty_topics=(
  /pico/pose/head
  /pico/pose/left_hand
  /pico/pose/right_hand
  /pico/palm_left
  /pico/palm_right
  /pico/smpl_raw
  /pico/smpl_palm_corrected
  /pico/smpl_palm_corrected_ik
)

usage() {
  cat <<'EOF'
用法: ./scripts/record_pico_tremor.sh [选项]

自动启动 PICO bridge 和掌心修正链路，然后依次录制：
  1. 固定手柄
  2. 肘部支撑
  3. 自然悬臂

默认每个工况录制 30 秒，开始前倒计时 5 秒，各工况之间不额外休息。

选项:
  --output DIR              输出目录（默认 recordings/pico_tremor_<时间>）
  --duration SECONDS        每个工况录制秒数，正整数（默认 30）
  --countdown SECONDS       每个工况准备倒计时，正整数（默认 5）
  --readiness-timeout SEC   PICO 数据就绪超时，正整数（默认 30）
  --domain ID               ROS_DOMAIN_ID（默认 120）
  --dry-run                 只打印采集流程，不访问硬件
  -h, --help                显示帮助
EOF
}

while (($#)); do
  case "$1" in
    --output) output_root="${2:-}"; shift 2 ;;
    --duration) duration_s="${2:-}"; shift 2 ;;
    --countdown) countdown_s="${2:-}"; shift 2 ;;
    --readiness-timeout) readiness_timeout_s="${2:-}"; shift 2 ;;
    --domain) domain="${2:-}"; shift 2 ;;
    --dry-run) dry_run=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

require_positive_integer() {
  local value="$1"
  local option="$2"
  if ! [[ "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "$option must be a positive integer" >&2
    exit 2
  fi
}

require_positive_integer "$duration_s" "--duration"
require_positive_integer "$countdown_s" "--countdown"
require_positive_integer "$readiness_timeout_s" "--readiness-timeout"
if ! domain="$(pico_normalize_ros_domain "$domain")"; then
  echo "--domain must be an integer between 0 and 232" >&2
  exit 2
fi

print_plan() {
  local index
  for index in "${!stages[@]}"; do
    printf 'stage=%s label=%s countdown_s=%s duration_s=%s repeat=1/1\n' \
      "${stages[$index]}" "${stage_labels[$index]}" "$countdown_s" \
      "$duration_s"
  done
}

if [[ "$dry_run" == true ]]; then
  print_plan
  exit 0
fi


cd "$repo_root"
source "$repo_root/scripts/environment.sh"
export ROS_DOMAIN_ID="$domain"
export EXO_REQUESTED_ROS_DOMAIN_ID="$domain"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE \
  FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI

for command_name in adb flock python ros2 setsid; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "缺少命令: $command_name" >&2
    exit 2
  fi
done
ros2 pkg prefix pico_bridge >/dev/null

if [[ -z "$output_root" ]]; then
  output_root="$repo_root/recordings/pico_tremor_$(date +%Y%m%d_%H%M%S)"
elif [[ "$output_root" != /* ]]; then
  output_root="$caller_dir/$output_root"
fi
if [[ -e "$output_root" ]]; then
  echo "输出目录已存在: $output_root" >&2
  exit 2
fi
mkdir -p "$output_root/logs"
manifest="$output_root/manifest.tsv"
printf 'stage\tlabel\tduration_s\tstarted_utc\tfinished_utc\tbag_path\n' > "$manifest"

lock_file="${XDG_RUNTIME_DIR:-${TMPDIR:-/tmp}}/pico_tremor_domain_${domain}.lock"
exec {capture_lock_fd}>"$lock_file"
if ! flock -n "$capture_lock_fd"; then
  echo "ROS domain $domain 已有一个 PICO 颤振采集脚本运行" >&2
  exit 2
fi

child_pids=()
bridge_pid=""
m0_pid=""
recorder_pid=""
cleanup_done=false

cleanup() {
  local index
  local pid
  if [[ "$cleanup_done" == true ]]; then
    return
  fi
  cleanup_done=true
  trap - EXIT INT TERM
  if [[ -n "$recorder_pid" ]]; then
    pico_stop_process_group "$recorder_pid" 600 20
    recorder_pid=""
  fi
  for ((index=${#child_pids[@]}-1; index>=0; index--)); do
    pid="${child_pids[$index]}"
    pico_stop_process_group "$pid" 100 20
  done
}

on_signal() {
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

show_failure_logs() {
  local log_file
  echo "最近运行日志：" >&2
  for log_file in "$output_root/logs/bridge.log" "$output_root/logs/m0.log"; do
    if [[ -s "$log_file" ]]; then
      echo "--- $log_file" >&2
      tail -n 30 "$log_file" >&2 || true
    fi
  done
}

node_is_running() {
  local node_name="$1"
  ros2 node list --no-daemon --spin-time 2 2>/dev/null |
    grep -Fxq "$node_name"
}

if node_is_running "/pico_bridge"; then
  echo "[1/4] 复用当前 PICO bridge"
else
  if ! adb get-state 2>/dev/null | grep -qx device; then
    echo "未检测到可用 PICO ADB 设备，请先处理 unauthorized/offline 状态" >&2
    exit 2
  fi

  echo "[1/4] 启动 PICO APK 与 ADB 转发"
  adb forward --remove tcp:9999 >/dev/null 2>&1 || true
  adb forward tcp:9999 tcp:9999 >/dev/null
  adb shell monkey -p com.PICO.wholebody_stream.unity 1 >/dev/null
  sleep 2
  if [[ -z "$(adb shell pidof com.PICO.wholebody_stream.unity 2>/dev/null | tr -d '\r')" ]]; then
    echo "PICO Streaming APK 未成功启动" >&2
    exit 2
  fi

  echo "[2/4] 启动原始 PICO bridge"
  setsid ros2 launch pico_bridge start_pico_bridge.launch.py \
    host:=127.0.0.1 port:=9999 \
    >"$output_root/logs/bridge.log" 2>&1 &
  bridge_pid=$!
  child_pids+=("$bridge_pid")
fi

python - "$readiness_timeout_s" <<'PY'
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

timeout_s = float(sys.argv[1])
topics = (
    "/pico/pose/head",
    "/pico/pose/left_hand",
    "/pico/pose/right_hand",
)
received = set()
node = None
subscriptions = []
try:
    rclpy.init()
    node = Node(f"pico_tremor_raw_readiness_{os.getpid()}")
    for topic in topics:
        subscriptions.append(
            node.create_subscription(
                PoseStamped,
                topic,
                lambda _message, name=topic: received.add(name),
                qos_profile_sensor_data,
            )
        )
    deadline = time.monotonic() + timeout_s
    while len(received) != len(topics) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
finally:
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
missing = [topic for topic in topics if topic not in received]
if missing:
    print("原始 PICO 数据就绪失败，缺少: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(2)
print("原始头显和左右手柄数据已就绪")
PY

if [[ -n "$bridge_pid" ]] && ! kill -0 "$bridge_pid" 2>/dev/null; then
  echo "PICO bridge 提前退出" >&2
  show_failure_logs
  exit 2
fi

if node_is_running "/pico_palm_skeleton_filter"; then
  echo "[3/4] 复用当前掌心与 corrected IK 链路"
else
  echo "[3/4] 启动掌心与 corrected IK 链路"
  setsid env PICO_M0_CLEAN_ENV=1 \
    "$repo_root/scripts/start_pico_m0.sh" \
    >"$output_root/logs/m0.log" 2>&1 &
  m0_pid=$!
  child_pids+=("$m0_pid")
fi

if ! python "$repo_root/src/pico_bridge/scripts/pico_m0_readiness.py" \
    --timeout-s "$readiness_timeout_s"; then
  echo "掌心或 corrected IK 数据未就绪" >&2
  show_failure_logs
  exit 2
fi
if [[ -n "$m0_pid" ]] && ! kill -0 "$m0_pid" 2>/dev/null; then
  echo "PICO M0 链路提前退出" >&2
  show_failure_logs
  exit 2
fi

validate_bag() {
  local bag_path="$1"
  python - "$bag_path" "${required_nonempty_topics[@]}" <<'PY'
from pathlib import Path
import sys
import yaml

bag = Path(sys.argv[1])
required = tuple(sys.argv[2:])
metadata_path = bag / "metadata.yaml"
if not metadata_path.is_file():
    raise SystemExit(f"录制未生成 metadata.yaml: {bag}")
metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
information = metadata.get("rosbag2_bagfile_information", {})
counts = {
    item["topic_metadata"]["name"]: int(item["message_count"])
    for item in information.get("topics_with_message_count", [])
}
missing = [topic for topic in required if counts.get(topic, 0) <= 0]
if missing:
    raise SystemExit("以下必需话题没有消息: " + ", ".join(missing))
total = int(information.get("message_count", 0))
if total <= 0:
    raise SystemExit(f"rosbag 没有任何消息: {bag}")
print(f"rosbag 校验通过: {bag.name}, messages={total}")
PY
}

run_countdown() {
  local label="$1"
  local remaining
  for ((remaining=countdown_s; remaining>0; remaining--)); do
    printf '\r[%s] %d 秒后开始录制... ' "$label" "$remaining"
    sleep 1
  done
  printf '\r[%s] 开始录制。                 \n' "$label"
}

record_stage() {
  local stage="$1"
  local label="$2"
  local instruction="$3"
  local bag_path="$output_root/$stage"
  local log_path="$output_root/logs/${stage}.log"
  local started_utc
  local finished_utc
  local remaining
  local database_ready=false

  echo
  echo "============================================================"
  echo "工况：$label"
  echo "$instruction"
  echo "============================================================"
  run_countdown "$label"

  started_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  setsid ros2 bag record \
    --storage sqlite3 \
    --output "$bag_path" \
    "${record_topics[@]}" \
    >"$log_path" 2>&1 &
  recorder_pid=$!

  for _ in {1..50}; do
    if compgen -G "$bag_path/*.db3" >/dev/null; then
      database_ready=true
      break
    fi
    if ! kill -0 "$recorder_pid" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if [[ "$database_ready" != true ]]; then
    echo "rosbag recorder 未成功启动：$stage" >&2
    tail -n 40 "$log_path" >&2 || true
    exit 2
  fi

  for ((remaining=duration_s; remaining>0; remaining--)); do
    if ! kill -0 "$recorder_pid" 2>/dev/null; then
      echo "rosbag recorder 在录制期间提前退出：$stage" >&2
      tail -n 40 "$log_path" >&2 || true
      exit 2
    fi
    printf '\r[%s] 正在录制，剩余 %d 秒... ' "$label" "$remaining"
    sleep 1
  done
  printf '\r[%s] 正在安全保存 rosbag...          \n' "$label"
  pico_stop_process_group "$recorder_pid" 600 20
  recorder_pid=""

  validate_bag "$bag_path"
  finished_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$stage" "$label" "$duration_s" "$started_utc" "$finished_utc" \
    "$bag_path" >> "$manifest"
}

echo "[4/4] 所有数据流已就绪，自动开始三工况采集"
echo "输出目录: $output_root"
print_plan

for index in "${!stages[@]}"; do
  record_stage "${stages[$index]}" "${stage_labels[$index]}" \
    "${stage_instructions[$index]}"
done

echo
echo "全部三组采集完成。"
echo "数据目录: $output_root"
echo "采集清单: $manifest"
