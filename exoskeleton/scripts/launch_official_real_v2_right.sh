#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PIXI_BIN="${DATAGLOVE_PIXI_BIN:-pixi}"
source "$PROJECT_DIR/scripts/lib/dataglove_network.sh"
HAND2_IP="${WUJI_HAND2_IP:-192.168.1.111}"
HAND2_SN="${WUJI_HAND2_SN:-}"
HAND2_INTERFACE="${WUJI_HAND2_INTERFACE:-enp4s0}"
HAND2_HOST_ADDRESS="${WUJI_HAND2_HOST_ADDRESS:-192.168.1.100/24}"
SIM_READY_TIMEOUT_SECONDS="${SIM_READY_TIMEOUT_SECONDS:-30}"
SIM_PID=""
CONFIRM_REAL=false

usage() {
  cat <<'EOF'
用法：
  ./scripts/launch_official_real_v2_right.sh --confirm-real

必须显式传入 --confirm-real，并确认物理急停可用、Hand2 周围没有障碍物。

必填环境变量：
  WUJI_HAND2_SN
可选环境变量：
  DATAGLOVE_HOST、DATAGLOVE_PORT、WUJI_HAND2_IP
  DATAGLOVE_INTERFACE、DATAGLOVE_MAC、DATAGLOVE_HOST_ADDRESS
  DATAGLOVE_SYS_CLASS_NET、DATAGLOVE_USB_WAIT_TIMEOUT_SECONDS
  WUJI_HAND2_INTERFACE、WUJI_HAND2_HOST_ADDRESS
  DATAGLOVE_PIXI_BIN、SIM_READY_TIMEOUT_SECONDS
EOF
}

while (($#)); do
  case "$1" in
    --confirm-real)
      CONFIRM_REAL=true
      shift
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

if [[ "$CONFIRM_REAL" != true ]]; then
  echo "错误：必须显式传入 --confirm-real" >&2
  usage >&2
  exit 2
fi
if [[ -z "$HAND2_SN" ]]; then
  echo "错误：必须显式设置 WUJI_HAND2_SN，禁止沿用其他设备序列号" >&2
  exit 2
fi
validate_glove_network_settings
for command_name in "$PIXI_BIN" nc ip setsid sudo; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "错误：缺少命令：$command_name" >&2
    exit 1
  fi
done

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
    echo "停止官方仿真与目标桥..."
    kill -INT -- "-$SIM_PID" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$SIM_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$SIM_PID" 2>/dev/null; then
      kill -TERM -- "-$SIM_PID" 2>/dev/null || true
    fi
    wait "$SIM_PID" 2>/dev/null || true
  fi
  exit "$status"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

cd "$PROJECT_DIR"


echo "[预检 1/4] 配置数据手套专用网卡并检查 TCP..."
configure_dataglove_network
if ! nc -z -w 2 "$GLOVE_HOST" "$GLOVE_PORT"; then
  echo "错误：数据手套不可达：${GLOVE_HOST}:${GLOVE_PORT}" >&2
  exit 1
fi
sleep 0.2

echo "[预检 2/4] 配置 Hand2 专用网卡并检查路由..."
sudo ip link set dev "$HAND2_INTERFACE" up
sudo ip addr replace "$HAND2_HOST_ADDRESS" dev "$HAND2_INTERFACE"
sudo ip route replace "${HAND2_IP}/32" \
  dev "$HAND2_INTERFACE" src "${HAND2_HOST_ADDRESS%/*}"
if ! route_matches "$HAND2_IP" "$HAND2_INTERFACE" "$HAND2_HOST_ADDRESS"; then
  echo "错误：Hand2 路由未使用 $HAND2_INTERFACE / $HAND2_HOST_ADDRESS" >&2
  ip route get "$HAND2_IP" >&2 || true
  exit 1
fi

echo "[预检 3/4] SDK 扫描、右手属性和 20/20 在线关节..."
"$PIXI_BIN" run python - "$HAND2_SN" <<'PY'
import sys
import time
import wuji_sdk

serial = sys.argv[1]
manager = wuji_sdk.SdkManager.instance()
matches = [device for device in wuji_sdk.SdkManager.instance().scan() if device.sn == serial]
if len(matches) != 1:
    raise RuntimeError(f"未唯一发现 Hand2 SN={serial}")
if matches[0].device_type != wuji_sdk.DeviceType.WujiHand2:
    raise RuntimeError(f"SN={serial} 不是 WujiHand2")

device_name = "official_real_v2_right_preflight"
device = manager.connect(
    sn=serial,
    device_name=device_name,
    options=wuji_sdk.ConnectOptions(enable_bridge=False),
)
try:
    side = str(device.handedness().get()).lower().rsplit(".", 1)[-1]
    if side != "right":
        raise RuntimeError(f"设备不是右手：{side}")
    online = int(device.online_joints_count().get())
    if online != 20:
        raise RuntimeError(f"Hand2 在线关节不是 20/20：{online}/20")
    print(f"Hand2 预检通过：SN={serial} side={side} online={online}/20")
finally:
    manager.disconnect(device_name)
    time.sleep(0.2)
PY

echo "[预检 4/4] 启动官方仿真与 right hand_dof 目标桥..."
if "$PIXI_BIN" run python - <<'PY' >/dev/null 2>&1
from data_glove_wuji_teleop.adapters.runtime.simulation_session import require_active_simulation
require_active_simulation(generation="v2", hand="right")
PY
then
  echo "错误：已有 v2/right 仿真 lease；请先关闭旧仿真" >&2
  exit 1
fi
setsid "$PIXI_BIN" run official-retarget-v2-right -- \
  --host "$GLOVE_HOST" \
  --port "$GLOVE_PORT" \
  --publish-targets \
  --skip-network-setup \
  --column-spacing 0.22 \
  --apply-filter \
  --pinch-alpha-max 1.0 &
SIM_PID=$!

ready=false
deadline=$((SECONDS + SIM_READY_TIMEOUT_SECONDS))
while ((SECONDS < deadline)); do
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    wait "$SIM_PID"
    exit $?
  fi
  if "$PIXI_BIN" run python - "$SIM_PID" <<'PY' >/dev/null 2>&1
import os
import sys

from data_glove_wuji_teleop.adapters.runtime.simulation_session import require_active_simulation

expected_process_group = int(sys.argv[1])
session = require_active_simulation(generation="v2", hand="right")
if os.getpgid(session.pid) != expected_process_group:
    raise RuntimeError(
        f"lease pid={session.pid} 不属于本次进程组 {expected_process_group}"
    )
PY
  then
    ready=true
    break
  fi
  sleep 0.1
done
if [[ "$ready" != true ]]; then
  echo "错误：官方仿真在 ${SIM_READY_TIMEOUT_SECONDS}s 内未就绪" >&2
  exit 1
fi

echo "等待 right hand_dof 第一帧..."
"$PIXI_BIN" run python - <<'PY'
import time

from data_glove_wuji_teleop.adapters.transport.zmq_dof import ZmqTargetSubscriber

subscriber = ZmqTargetSubscriber(
    "127.0.0.1",
    15558,
    "right",
    timeout_seconds=0.1,
)
deadline = time.monotonic() + 5.0
target = None
try:
    while target is None and time.monotonic() < deadline:
        target = subscriber.recv()
finally:
    subscriber.close()
if target is None:
    raise RuntimeError("5s 内未收到 right hand_dof")
if target.source != "official_wuji_retarget":
    raise RuntimeError(f"目标来源错误：{target.source}")
print(f"目标预检通过：source={target.source} sequence={target.sequence}")
PY

echo "官方仿真 lease 与目标首帧已就绪，启动 Hand2 真机控制..."
"$PIXI_BIN" run real-v2-right-realtime -- \
  --hand-sn "$HAND2_SN" \
  --kp 8.0 \
  --kd 0.2 \
  --realtime-max-velocity 3.0 \
  --home-tolerance 0.10 \
  --home-timeout 15 \
  --confirm-real
