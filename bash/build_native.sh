#!/usr/bin/env bash
# Build the single retained Franka DLS/Ruckig native core and its workers.
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
native="$root/src/teleop_outputs/tianji/tianji_controller/native"
core_build="$root/build/control/core"
install_prefix="$root/install/control"

# Prefer Ninja; the CMake option list must stay flat (a literal `--` separator
# is not a CMake argument and is rejected).
generator=(-G Ninja)
if ! command -v ninja >/dev/null 2>&1; then
  generator=()
fi

# Resolve all numerical dependencies in the active isolated control environment.
cmake_prefix_path="$install_prefix"
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  cmake_prefix_path="$cmake_prefix_path;$CONDA_PREFIX"
fi

printf '== control core ==\n'
cmake -S "$native" -B "$core_build" "${generator[@]}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$install_prefix" \
  -DCMAKE_PREFIX_PATH="$cmake_prefix_path" \
  -DPython3_EXECUTABLE="$root/.pixi/envs/default/bin/python"
cmake --build "$core_build" --parallel "${TIANJI_BUILD_JOBS:-4}"
# Install so `tianji_runtime.native_executable` finds the binaries and
# `controller_profile` finds the installed profiles.
cmake --install "$core_build"
