#!/usr/bin/env bash
# Build the whole workspace: the control native targets and then every ament
# package in src/. Run from a Pixi environment ('pixi run build').
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

# The vendored Odin driver links a sibling SDK static library that colcon does
# not build. That SDK cannot be compiled without `certs/certs.h`, an mTLS
# credential the repository deliberately never commits ("provision separately;
# never commit private keys"). Absent it the Odin packages are skipped with an
# explicit reason rather than failing the workspace build.
odin_certs="$root/src/teleop_inputs/odin/odin-sdk2/sdk/utils/http/certs/certs.h"
skipped_packages=()
if [[ -r "$odin_certs" ]]; then
  printf '== building vendored Odin SDK ==\n'
  bash "$root/src/teleop_inputs/odin/scripts/ensure_odin_sdk.sh"
else
  printf '%s\n' \
    '== skipping Odin packages ==' \
    "   missing mTLS credential: ${odin_certs#"$root"/}" \
    '   Provision it to enable the Odin sensor path; the rest of the workspace is unaffected.'
  skipped_packages+=(--packages-skip odin_ros_driver odin_ros_driver_rev1 pico_odin)
fi

# Resolve the interpreter once and pass it explicitly: ament's FindPython3 runs
# inside CMake, where `$CONDA_PREFIX` is not expanded by the caller's shell.
python_bin="$(command -v python)"
printf '== building ROS packages (environment: %s, python: %s) ==\n' "$environment" "$python_bin"
exec colcon --log-base "$root/log/$environment" build \
  --base-paths "$root/src" \
  --build-base "$root/build/$environment" \
  --install-base "$root/install/$environment" \
  --symlink-install \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$root/install/$environment" \
  -DPython3_EXECUTABLE="$python_bin" \
  -DPython_EXECUTABLE="$python_bin" \
  "${skipped_packages[@]}" \
  "$@"
