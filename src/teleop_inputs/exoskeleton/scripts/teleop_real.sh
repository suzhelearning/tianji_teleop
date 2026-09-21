#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PIXI_BIN="${DATAGLOVE_PIXI_BIN:-pixi}"

# 调用本脚本即请求真机模式；仿真首帧、硬件身份和在线检查仍由统一入口强制执行。
cd -- "$PROJECT_DIR"
exec "$PIXI_BIN" run official-teleop-v2 --confirm-real "$@"
