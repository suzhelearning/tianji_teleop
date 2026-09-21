#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

side="${1:-}"
if [[ "$side" != "left" && "$side" != "right" ]]; then
  echo "用法: $0 <left|right>" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export EXO_REQUESTED_ROS_DOMAIN_ID="${EXO_REQUESTED_ROS_DOMAIN_ID:-$ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"

source "$repo_root/scripts/environment.sh"
source "$repo_root/src/pico_bridge/scripts/pico_process_cleanup.sh"

artifact="$HOME/.config/pico_tracker/pico_${side}_palm_tcp.yaml"
service="/pico/palm_orientation/${side}/calibrate"
publisher_pid=""

cleanup() {
  local owned_pid="$publisher_pid"
  trap - EXIT INT TERM
  publisher_pid=""
  pico_stop_process_group "$owned_pid"
}
trap cleanup EXIT INT TERM

if ! python "$repo_root/src/pico_bridge/scripts/pico_calibration_artifact.py" \
    validate --path "$artifact" --kind tcp --side "$side" >/dev/null; then
  echo "✗ $side TCP 标定文件缺失或无效: $artifact" >&2
  echo "  请先运行: ./scripts/calibrate_pico_arm.sh $side tcp" >&2
  exit 2
fi

service_ready() {
  ros2 service list 2>/dev/null | /usr/bin/grep -Fxq "$service"
}

if ! service_ready; then
  setsid ros2 run pico_bridge pico_palm_tcp_publisher \
    --side "$side" --artifact "$artifact" &
  publisher_pid=$!
  for _ in {1..50}; do
    if ! pico_process_group_is_running "$publisher_pid"; then
      echo "✗ $side TCP publisher 启动失败" >&2
      exit 2
    fi
    service_ready && break
    sleep 0.1
  done
fi
if ! service_ready; then
  echo "✗ 姿态标定服务未就绪: $service" >&2
  exit 2
fi

label="左侧"
[[ "$side" == "right" ]] && label="右侧"
echo
echo "开始${label}掌心姿态重标定（只更新 TCP 姿态）："
echo "  - 身体自然直立，头部正视前方"
echo "  - 双臂向正前方水平伸直"
echo "  - 左右掌心相对，手腕保持中立"
echo "  - 按空格后保持约 2 秒静止"
echo "  - TCP 平移、腕部距离和上臂/前臂骨长不会改变"
echo "摆好后按空格开始，q 取消。"

while true; do
  IFS= read -rsn1 key
  [[ "$key" == "q" || "$key" == "Q" ]] && exit 130
  [[ "$key" == " " ]] && break
done

result="$(ros2 run pico_bridge pico_palm_orientation_client \
  --side "$side" --timeout-s 8.0)" || {
    echo
    echo "✗ ${label}掌心姿态重标定失败；原 TCP 文件和运行时姿态保持不变。" >&2
    exit 2
  }

python - "$result" <<'PY'
import json
import math
import sys

result = json.loads(sys.argv[1])
label = "左侧" if result["side"] == "left" else "右侧"
print()
print(f"✓ {label}掌心姿态重标定成功")
print(f"  姿态误差 RMS: {math.degrees(result['orientation_rms_rad']):.2f}°")
print(f"  本次修正角: {math.degrees(result['correction_angle_rad']):.2f}°")
print(f"  TCP 总修订号: {result['calibration_revision']}")
print(f"  TCP 姿态修订号: {result['orientation_revision']}")
print(f"  TCP 平移修订号（保持不变）: {result['translation_revision']}")
print(f"  标定结果文件: {result['artifact_path']}")
print("  腕部距离和上臂/前臂骨长继续有效")
PY
