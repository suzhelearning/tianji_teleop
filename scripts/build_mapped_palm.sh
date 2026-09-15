#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
pixi run cmake -S control/mapped_palm -B control/build-mapped-palm -G Ninja -DCMAKE_BUILD_TYPE=Release
pixi run cmake --build control/build-mapped-palm --parallel 2
pixi run ctest --test-dir control/build-mapped-palm --output-on-failure
