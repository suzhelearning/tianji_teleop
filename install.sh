#!/usr/bin/env bash
# Install into this checkout; never launch or enable hardware.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
cd -- "$project_root"

usage() {
  printf '%s\n' \
    'Usage: bash install.sh [--check | --help]' \
    'Default: install locked Pixi environments, Python dependencies, control, Manus and tracking.' \
    '--check: check basic prerequisites only; do not download or build.' \
    'Requires Linux x86_64, Pixi, /usr/bin/g++, libudev, libusb and zlib.' \
    'Ubuntu/Debian system setup: sudo apt-get install build-essential libudev1 libusb-1.0-0 zlib1g adb tmux' \
    'Install Pixi first: https://pixi.sh/latest/installation/' \
    'Allow ample disk space for both environments and package caches; network access is required.'
}
if (( $# > 1 )); then usage >&2; exit 2; fi
case "${1:-}" in
  --help|-h) usage; exit 0 ;;
  ''|--check) ;;
  *) usage >&2; exit 2 ;;
esac
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || fail 'Only Linux x86_64 is supported by pixi.lock.'
command -v pixi >/dev/null || fail 'Install Pixi first: https://pixi.sh/latest/installation/'
[[ -x /usr/bin/g++ ]] || fail 'Install build-essential; manus/build.sh requires /usr/bin/g++.'
sdk=manus/ManusSDK/lib/libManusSDK_Integrated.so
[[ -r "$sdk" ]] || fail "Missing vendor SDK: $sdk"
[[ "$(LC_ALL=C od -An -tx1 -N4 "$sdk" | tr -d ' \n')" == 7f454c46 ]] || fail 'Manus SDK is not an ELF library (possibly a Git LFS pointer); obtain the real SDK library.'
[[ -r pixi.lock && -r pixi.toml ]] || fail 'Missing Pixi manifest or lockfile.'
printf '%s\n' 'Basic prerequisites OK. USB permissions, network access and available disk space are not validated.'
[[ "${1:-}" != --check ]] || exit 0
trap 'printf "Installation failed at line %s; fix the reported error and rerun bash install.sh.\n" "$LINENO" >&2' ERR

pixi install --locked
pixi install --locked -e tracking
if [[ ! -e .venv ]]; then
  pixi run --locked python -m venv .venv
fi
[[ -x .venv/bin/python ]] || fail 'Existing .venv is incomplete; move it aside before retrying.'
.venv/bin/python -c 'import sys; assert sys.version_info[:2] == (3, 11), "Root .venv must use Python 3.11"'
# Use the declared native compiler environment for the Python extension too.
pixi run --locked .venv/bin/python -m pip install -e '.[collection,retargeting,tracking,test]'
pixi run --locked configure
pixi run --locked build
pixi run --locked build-manus
export TIANJI_PYTHON="$project_root/.venv/bin/python"
export ROS_SETUP="$project_root/.pixi/envs/tracking/setup.bash"
pixi run --locked -e tracking bash tracking/scripts/build.sh

.venv/bin/tianji --help
.venv/bin/python -c 'from retargeting.wuji_retargeting import _native'
[[ -x manus/build/manus_raw && -r tracking/install/local_setup.bash ]] || fail 'Expected build outputs are missing.'
printf '%s\n' \
  'Installation and compilation completed. No hardware was contacted.' \
  'Keep this checkout in place: the Python installation is editable.' \
  'Configure device access and calibrate for the actual operator before teleoperation.'
