#!/usr/bin/env bash
# Build shared control native targets and the non-SPD ament workspace.
# The PICO bare-hand route has its own `pixi run -e spd build` entry point.
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
environment="${PIXI_ENVIRONMENT_NAME:-${TIANJI_ENVIRONMENT:-default}}"
export TIANJI_ENVIRONMENT="$environment"
cd -- "$root"
# shellcheck source=bash/environment.sh
source "$root/bash/environment.sh" --build

if [[ ! -x "$CONDA_PREFIX/bin/colcon" ]]; then
  printf 'colcon is missing from %s; install the environment first.\n' "$CONDA_PREFIX" >&2
  exit 1
fi

# The workspace owns install/ and log/; build/ holds both the native CMake
# trees and the per-environment colcon output.
mkdir -p "$root/build/$environment" "$root/install/$environment" "$root/log/$environment"

printf '== building control native targets ==\n'
# The native controller has its own pinned toolchain, so it is built through the
# `control` environment rather than the default one.
pixi run --locked --manifest-path "$root/pixi.toml" -e control \
  bash "$root/bash/build_native.sh"

printf '== isolated ROS arm controller ==\n'
bash "$root/bash/build_arm_ros.sh"


# Resolve the interpreter once and pass it explicitly: ament's FindPython3 runs
# inside CMake, where `$CONDA_PREFIX` is not expanded by the caller's shell.
python_bin="$(command -v python)"
printf '== building ROS packages (environment: %s, python: %s) ==\n' "$environment" "$python_bin"
exec colcon --log-base "$root/log/$environment" build \
  --base-paths "$root/src" \
  --build-base "$root/build/$environment" \
  --install-base "$root/install/$environment" \
  --symlink-install \
  --packages-ignore pico2_hands tianji_spd_interfaces \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$root/install/$environment" \
  -DPython3_EXECUTABLE="$python_bin" \
  -DPython_EXECUTABLE="$python_bin" \
  "$@"
