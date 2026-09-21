#!/usr/bin/env bash
# Build and verify the independent Hand2 native ABI. The arm DLS worker is
# built separately by the main workspace's `pixi run build`.
#
# The build trees live inside the package (`native/build/...`) because the
# Hand2 artifacts are installed as package data: `hand_worker.py` locates them
# relative to the package in both the checkout and an installed copy.
set -euo pipefail
# The importable package keeps its config/native tree beside its modules, so the
# build trees and the artifacts they install live under python/pico2_hands/.
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PACKAGE_DIR="$ROOT_DIR/python/pico2_hands"
WORKSPACE="${TIANJI_WORKSPACE:-$(cd -- "$ROOT_DIR/../../.." && pwd -P)}"
PICO2_CMAKE="$WORKSPACE/.pixi/envs/default/bin/cmake"
PICO2_HAND_PREFIX="$ROOT_DIR/tools/wuji_hand_native/.pixi/envs/default"
HAND_BUILD="$PACKAGE_DIR/native/build/pico2-hand"
if [[ ! -x "$PICO2_CMAKE" || ! -f "$PICO2_HAND_PREFIX/include/nlopt.h" ]]; then
  echo 'Install the project environment and run:' >&2
  echo "pixi install --locked --manifest-path $ROOT_DIR/tools/wuji_hand_native/pixi.toml" >&2
  exit 1
fi
"$PICO2_CMAKE" -S "$PACKAGE_DIR/native/hand/optimizer" -B "$HAND_BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PICO2_HAND_PREFIX" \
  -DCMAKE_BUILD_RPATH="$PICO2_HAND_PREFIX/lib" \
  -DTIANJI_HAND_OPTIMIZER_TESTS=ON -DTIANJI_HAND_SCHEDULER_TESTS=ON
"$PICO2_CMAKE" --build "$HAND_BUILD" --parallel 1
"$WORKSPACE/.pixi/envs/default/bin/ctest" --test-dir "$HAND_BUILD" --output-on-failure
"$HAND_BUILD/test_hand_optimizer" \
  "$ROOT_DIR/third_party/wuji_hand_retargeting/wuji_retargeting/wuji-description/hand2/hand2_beta1/body/urdf/right.urdf"
"$PICO2_HAND_PREFIX/bin/python" "$PACKAGE_DIR/scripts/compare_hand_worker.py" \
  --official-root "$ROOT_DIR/third_party/wuji_hand_retargeting"
