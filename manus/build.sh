#!/usr/bin/env bash
set -euo pipefail

manus_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
sdk="$manus_root/ManusSDK/lib/libManusSDK_Integrated.so"
if [[ ! -r "$sdk" ]] || [[ "$(LC_ALL=C od -An -tx1 -N4 "$sdk" | tr -d ' \n')" != 7f454c46 ]]; then
  printf 'Manus SDK is missing or a Git LFS pointer; run git lfs pull for ManusSDK before building. No devices contacted.\n' >&2
  exit 2
fi
mkdir -p "$manus_root/build"
/usr/bin/g++ -std=c++17 -O2 -pthread \
  -I"$manus_root/ManusSDK/include" "$manus_root/rawviz.cpp" \
  -L"$manus_root/ManusSDK/lib" -l:libManusSDK_Integrated.so \
  -l:libudev.so.1 -l:libusb-1.0.so.0 -l:libz.so.1 \
  '-Wl,-rpath,$ORIGIN/../ManusSDK/lib' \
  -o "$manus_root/build/manus_raw"
printf 'Built %s\n' "$manus_root/build/manus_raw"
