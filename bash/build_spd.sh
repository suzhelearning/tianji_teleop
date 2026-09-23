#!/usr/bin/env bash
# Build only the PICO hand -> SPD runtime. Never starts a simulator or hardware.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${CONDA_PREFIX:-}" != "$root/.pixi/envs/spd" ]]; then
  exec env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD \
    -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u TIANJI_PYTHON -u ROS_DISTRO -u ROS_VERSION \
    pixi run --locked --manifest-path "$root/pixi.toml" -e spd \
    bash "$root/bash/build_spd.sh" "$@"
fi
export TIANJI_WORKSPACE="$root" TIANJI_ENVIRONMENT=spd
export TIANJI_PYTHON="$CONDA_PREFIX/bin/python"
source "$root/bash/environment.sh" --build
cd -- "$root"

# Reuse the reviewed native ABI, not the default/policy runtime or overlay.
pixi install --locked --manifest-path "$root/pixi.toml" -e control
native="$root/src/teleop_outputs/tianji/tianji_controller/native"
core_build="$root/build/spd/core"
install_prefix="$root/install/spd"
control=(env -u PYTHONPATH -u PYTHONHOME -u LD_LIBRARY_PATH -u LD_PRELOAD
  -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH
  pixi run --locked --manifest-path "$root/pixi.toml" -e control)
printf '== SPD DLS worker and auxiliary viewer ==\n'
"${control[@]}" cmake -S "$native" -B "$core_build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$install_prefix" \
  -DCMAKE_PREFIX_PATH="$root/.pixi/envs/control" \
  -DBUILD_TESTING=OFF -DTIANJI_BUILD_ROS=OFF
"${control[@]}" cmake --build "$core_build" --parallel "${TIANJI_BUILD_JOBS:-4}" \
  --target pico2_dls_worker tianji_qp_ik_viewer
"${control[@]}" cmake --install "$core_build" --component spd

pico_root="$root/src/teleop_inputs/pico_hand"
pixi install --locked --manifest-path "$pico_root/tools/wuji_hand_native/pixi.toml"
bash "$pico_root/build_native.sh"

printf '== SPD ROS interfaces, Python runtime and model resources ==\n'
exec "$CONDA_PREFIX/bin/colcon" --log-base "$root/log/spd" build \
  --base-paths "$root/src/interfaces/tianji_interfaces" \
    "$root/src/interfaces/tianji_spd_interfaces" "$pico_root" \
    "$root/src/teleop_outputs/tianji/tianji_description" \
    "$root/src/teleop_outputs/tianji/tianji_controller" "$root/src/simulation" \
  --packages-select tianji_interfaces tianji_spd_interfaces pico2_hands \
    tianji_description tianji_controller simulation \
  --build-base "$root/build/spd/ros" --install-base "$install_prefix" \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE="$TIANJI_PYTHON" -DPython_EXECUTABLE="$TIANJI_PYTHON" "$@"
