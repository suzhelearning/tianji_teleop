#!/usr/bin/env bash
set -euo pipefail
PICO2_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PICO2_PROJECT_ROOT"
PICO2_CMAKE="$PICO2_PROJECT_ROOT/.pixi/envs/default/bin/cmake"
PICO2_HAND_PREFIX="$PICO2_PROJECT_ROOT/pico2_hands/tools/wuji_hand_native/.pixi/envs/default"
if [[ ! -x "$PICO2_CMAKE" || ! -f "$PICO2_HAND_PREFIX/include/nlopt.h" ]]; then
  echo 'Install the project environment and run:' >&2
  echo 'pixi install --locked --manifest-path pico2_hands/tools/wuji_hand_native/pixi.toml' >&2
  exit 1
fi
"$PICO2_CMAKE" -S pico2_hands/native -B build/pico2-v131 \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PICO2_PROJECT_ROOT/.pixi/envs/default"
"$PICO2_CMAKE" --build build/pico2-v131 --parallel 1
"$PICO2_CMAKE" -S pico2_hands/native/hand/optimizer -B build/pico2-hand \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PICO2_HAND_PREFIX" \
  -DCMAKE_BUILD_RPATH="$PICO2_HAND_PREFIX/lib" \
  -DTIANJI_HAND_OPTIMIZER_TESTS=ON -DTIANJI_HAND_SCHEDULER_TESTS=ON
"$PICO2_CMAKE" --build build/pico2-hand --parallel 1
"$PICO2_PROJECT_ROOT/.pixi/envs/default/bin/ctest" --test-dir build/pico2-v131 --output-on-failure
"$PICO2_PROJECT_ROOT/.pixi/envs/default/bin/ctest" --test-dir build/pico2-hand --output-on-failure
build/pico2-hand/test_hand_optimizer \
  pico2_hands/third_party/wuji_hand_retargeting/wuji_retargeting/wuji-description/hand2/hand2_beta1/body/urdf/right.urdf
"$PICO2_HAND_PREFIX/bin/python" pico2_hands/scripts/compare_hand_worker.py
