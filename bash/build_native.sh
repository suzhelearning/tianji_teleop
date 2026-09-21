#!/usr/bin/env bash
# Build the two independent native CMake projects.
#
# `tianji_controller/native` and its `mapped_palm` sub-project have precise,
# separately pinned dependency sets, so they keep separate build trees instead
# of being merged into one CMake project.
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
native="$root/src/tianji/tianji_controller/native"
core_build="$root/build/control/core"
palm_build="$root/build/control/mapped-palm"
install_prefix="$root/install/control"

# Prefer Ninja; the CMake option list must stay flat (a literal `--` separator
# is not a CMake argument and is rejected).
generator=(-G Ninja)
if ! command -v ninja >/dev/null 2>&1; then
  generator=()
fi

# Ceres is fetched once and installed into the control prefix, so subsequent
# configures must be able to find it there.
cmake_prefix_path="$install_prefix"
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  cmake_prefix_path="$cmake_prefix_path;$CONDA_PREFIX"
fi

printf '== control core ==\n'
cmake -S "$native" -B "$core_build" "${generator[@]}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$install_prefix" \
  -DCMAKE_PREFIX_PATH="$cmake_prefix_path" \
  -DTIANJI_ENABLE_CERES="${TIANJI_ENABLE_CERES:-ON}" \
  -DPython3_EXECUTABLE="$root/.pixi/envs/default/bin/python"
cmake --build "$core_build" --parallel "${TIANJI_BUILD_JOBS:-4}"
# Install so `tianji_runtime.native_executable` finds the binaries and
# `controller_profile` finds the installed profiles.
cmake --install "$core_build"

printf '== mapped_palm ==\n'
cmake -S "$native/mapped_palm" -B "$palm_build" "${generator[@]}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$install_prefix" \
  -DCMAKE_PREFIX_PATH="$cmake_prefix_path" \
  -DPython3_EXECUTABLE="$root/.pixi/envs/default/bin/python"
cmake --build "$palm_build" --parallel "${TIANJI_BUILD_JOBS:-4}"
cmake --install "$palm_build"
