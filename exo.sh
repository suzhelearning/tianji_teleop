#!/usr/bin/env bash
# Start both gloves by default; explicit sender options override hand selection.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exo_root="$project_root/exoskeleton"
if ! command -v pixi >/dev/null; then
  printf '%s\n' \
    'Pixi missing; no hardware was contacted.' \
    'Install Pixi: https://pixi.sh/latest/installation/ then run bash install.sh --exoskeleton.' >&2
  exit 1
fi
if [[ ! -r "$exo_root/pixi.toml" || ! -r "$exo_root/pixi.lock" ]]; then
  printf 'Missing internal exoskeleton Pixi manifest or lockfile: %s; no hardware was contacted.\n' "$exo_root" >&2
  exit 1
fi
if [[ ! -x "$exo_root/.pixi/envs/default/bin/python" ]]; then
  printf '%s\n' \
    "Exoskeleton Python environment missing: $exo_root/.pixi/envs/default; no hardware was contacted." \
    'Run bash install.sh --exoskeleton from the project root first.' >&2
  exit 1
fi
# Preserve the original sender's relative config paths, regardless of caller cwd.
cd -- "$exo_root"
exec pixi run --manifest-path "$exo_root/pixi.toml" --locked -e default tianji-send \
  --hand both --confirm-send --commission-directions "$@"
