#!/usr/bin/env bash
# Install this checkout into locked Pixi environments and build the native
# targets. Never launches or enables hardware.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd -- "$project_root"

usage() {
  printf '%s\n' \
    'Usage: bash bash/install.sh [--exoskeleton | --check | --help]' \
    'Default: install every locked Pixi environment and build the native targets.' \
    '--exoskeleton: install only the exoskeleton glove environment.' \
    '--check: verify prerequisites only; download and build nothing.' \
    'Requires Linux x86_64, Pixi, /usr/bin/g++ and libudev/libsub/zlib headers.' \
    'Install Pixi first: https://pixi.sh/latest/installation/'
}
if (( $# > 1 )); then usage >&2; exit 2; fi
case "${1:-}" in
  --help|-h) usage; exit 0 ;;
  ''|--check|--exoskeleton) ;;
  *) usage >&2; exit 2 ;;
esac

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || fail 'Only Linux x86_64 is supported by pixi.lock.'
command -v pixi >/dev/null || fail 'Install Pixi first: https://pixi.sh/latest/installation/'
[[ -x /usr/bin/g++ && -x /usr/bin/gcc ]] || fail 'Install build-essential.'

exoskeleton_root="$project_root/src/teleop_inputs/exoskeleton"

install_exoskeleton() {
  [[ -r "$exoskeleton_root/pixi.toml" && -r "$exoskeleton_root/pixi.lock" ]] \
    || fail "Missing exoskeleton Pixi manifest or lockfile under $exoskeleton_root."
  # Reinstall the editable package so native-only changes cannot leave a stale
  # extension behind: the manifest pins Python 3.12 independently.
  CC=/usr/bin/gcc CXX=/usr/bin/g++ pixi reinstall \
    --manifest-path "$exoskeleton_root/pixi.toml" --locked data-glove-wuji-teleop
}

if [[ "${1:-}" == --exoskeleton ]]; then
  install_exoskeleton
  printf '%s\n' 'Exoskeleton environment installed. No hardware was contacted.'
  exit 0
fi

[[ -r pixi.toml && -r pixi.lock ]] || fail 'Missing Pixi manifest or lockfile.'
printf '%s\n' 'Basic prerequisites OK. USB permissions, network access and disk space are not validated.'
[[ "${1:-}" != --check ]] || exit 0
trap 'printf "Installation failed at line %s; fix the reported error and rerun bash bash/install.sh.\n" "$LINENO" >&2' ERR

pixi install --locked --all
install_exoskeleton
pixi run --locked check-env
pixi run --locked build
pixi run --locked -e policy build
bash bash/build_manus.sh
pico2_root="$project_root/src/teleop_inputs/pico_hand"
pixi install --locked --manifest-path "$pico2_root/tools/wuji_hand_native/pixi.toml"
bash "$pico2_root/build_native.sh"

printf '%s\n' \
  'Installation and compilation completed. No hardware was contacted.' \
  'Configure device access and calibrate for the actual operator before teleoperation.'
