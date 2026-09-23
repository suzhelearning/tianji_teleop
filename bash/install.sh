#!/usr/bin/env bash
# Install and build only the PICO bare-hand client for SPD. No devices are opened.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
usage() {
  printf '%s\n' \
    'Usage: bash bash/install.sh [--check | --help]' \
    'Install the locked SPD, native control and package-local Hand2 environments, then build.' \
    '--check: check prerequisites only; download and build nothing.'
}
if (( $# > 1 )); then usage >&2; exit 2; fi
case "${1:-}" in
  --help|-h) usage; exit 0 ;;
  ''|--check) ;;
  *) usage >&2; exit 2 ;;
esac
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || fail 'Only Linux x86_64 is supported.'
command -v pixi >/dev/null || fail 'Install Pixi: https://pixi.sh/latest/installation/'
[[ -r "$root/pixi.toml" && -r "$root/pixi.lock" ]] || fail 'Missing Pixi manifest or lockfile.'
printf '%s\n' 'Basic prerequisites OK. USB permissions, network access and disk space are not validated.'
[[ "${1:-}" != --check ]] || exit 0
pixi install --locked --manifest-path "$root/pixi.toml" -e spd
bash "$root/bash/build_spd.sh"
printf '%s\n' 'SPD client installation and build completed. No headset or simulator was contacted.'
