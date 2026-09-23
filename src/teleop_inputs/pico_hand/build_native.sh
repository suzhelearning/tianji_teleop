#!/usr/bin/env bash
# Compile the independent Hand2 ABI into the dedicated SPD install prefix.
set -euo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PACKAGE_DIR="$ROOT_DIR/python/pico2_hands"
WORKSPACE="$(cd -- "$ROOT_DIR/../../.." && pwd -P)"
PICO2_CMAKE="$WORKSPACE/.pixi/envs/control/bin/cmake"
PICO2_HAND_PREFIX="$ROOT_DIR/tools/wuji_hand_native/.pixi/envs/default"
HAND_BUILD="$WORKSPACE/build/spd/hand"
if [[ ! -x "$PICO2_CMAKE" || ! -f "$PICO2_HAND_PREFIX/include/nlopt.h" ]]; then
  echo 'Run pixi run --locked -e spd build to install the native toolchains.' >&2
  exit 1
fi
# Do not inherit ROS/default compiler search paths or Python activation. The
# hand lock supplies Eigen/Pinocchio 4 and its own C++ runtime, not control's ABI.
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH CMAKE_PREFIX_PATH CMAKE_ARGS CPPFLAGS CFLAGS CXXFLAGS LDFLAGS
export CONDA_PREFIX="$PICO2_HAND_PREFIX"
"$PICO2_CMAKE" -S "$PACKAGE_DIR/native/hand/optimizer" -B "$HAND_BUILD" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DCMAKE_PREFIX_PATH="$PICO2_HAND_PREFIX" \
  -DCMAKE_INSTALL_PREFIX="$WORKSPACE/install/spd" \
  -DCMAKE_BUILD_RPATH="$PICO2_HAND_PREFIX/lib" \
  -DCMAKE_INSTALL_RPATH="$PICO2_HAND_PREFIX/lib" \
  -DTIANJI_HAND_OPTIMIZER_TESTS=OFF -DTIANJI_HAND_SCHEDULER_TESTS=OFF
"$PICO2_CMAKE" --build "$HAND_BUILD" --parallel 1
"$PICO2_CMAKE" --install "$HAND_BUILD"
