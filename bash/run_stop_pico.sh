#!/usr/bin/env bash
# Stop only the background PICO session owned by this checkout.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
echo "本命令仅停止本工程的后台 tmux 输入；前台 run_pico.sh 请在其终端按 Ctrl+C。"
exec bash "$root/bash/start_tianji_pico_teleop.sh" --stop "$@"
