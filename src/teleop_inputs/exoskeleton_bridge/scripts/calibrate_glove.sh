#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PIXI_BIN="${DATAGLOVE_PIXI_BIN:-pixi}"

# --hand left/right 自动识别实物 ID；只有六步采集完整后才更新 default 绑定。
cd -- "$PROJECT_DIR"
exec "$PIXI_BIN" run calibrate-dataglove-urdf-zero "$@"
