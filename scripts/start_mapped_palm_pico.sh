#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ ( $# != 2 && $# != 3 ) || "$1" != --user || -z "$2" || ( $# == 3 && "$3" != --symmetric-geometry ) ]]; then
  printf '%s\n' 'Usage: bash scripts/start_mapped_palm_pico.sh --user NAME [--symmetric-geometry]' >&2; exit 2
fi
source scripts/environment.sh
if tmux has-session -t pico_tianji_teleop 2>/dev/null; then
  printf '%s\n' 'Existing PICO session: refusing to assume its profile/X offset. Stop it explicitly before switching routes.' >&2
  exit 2
fi
calibration_dir="$("$TIANJI_PYTHON" -m tianji profile --user "$2" --component pico)"
if [[ "${3:-}" == --symmetric-geometry ]]; then
  calibration_dir="$("$TIANJI_PYTHON" scripts/pico_symmetric_profile.py --source "$calibration_dir" --resolve)"
fi
exec bash tracking/scripts/start_tianji_pico_teleop.sh --detach \
  --calibration-dir "$calibration_dir" --pico-world-x-offset 0.20
