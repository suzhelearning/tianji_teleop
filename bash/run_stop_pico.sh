#!/usr/bin/env bash
# Stop only the PICO session owned by this checkout.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$root/bash/pixi.bash" "$@"
source "$root/bash/environment.sh"
exec bash "$root/bash/start_tianji_pico_teleop.sh" --stop "$@"
