#!/usr/bin/env bash
# Native DLS/Ruckig test suite. No hardware is contacted.
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

# The native suites link against the `control` environment's libraries, so they
# run there; ctest is found in that environment too.
if [[ -z "${TIANJI_NATIVE_ENV_ACTIVE:-}" ]] && command -v pixi >/dev/null 2>&1; then
  exec env TIANJI_NATIVE_ENV_ACTIVE=1 pixi run --locked --manifest-path "$root/pixi.toml" \
    -e control bash "${BASH_SOURCE[0]}" "$@"
fi

export TIANJI_WORKSPACE="$root"

status=0

printf '== control core CTest ==\n'
if [[ -d "$root/build/control/core" ]]; then
  ctest --test-dir "$root/build/control/core" --output-on-failure || status=1
else
  printf 'control core build tree missing; run bash/bash/build_native.sh first.\n' >&2
  status=1
fi


exit "$status"
