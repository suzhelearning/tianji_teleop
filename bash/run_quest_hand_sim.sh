#!/usr/bin/env bash
# Quest v1 uses the same FLU/OpenXR wire format as the PICO bare-hand input.
# Reuse this checkout's calibration, DLS/Ruckig, Hand2 and ROS publisher.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec bash "$root/bash/run_pico_hand_sim.sh" "$@"
