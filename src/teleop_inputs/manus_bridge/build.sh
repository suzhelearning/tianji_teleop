#!/usr/bin/env bash
# Build the Manus Integrated SDK collector (rawviz) into this package.
set -euo pipefail

manus_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
sdk_root="$(cd "$manus_root/../../.." && pwd -P)/vendor/manus_sdk"
sdk="$sdk_root/lib/libManusSDK_Integrated.so"
if [[ ! -r "$sdk" ]] || [[ "$(LC_ALL=C od -An -tx1 -N4 "$sdk" | tr -d ' \n')" != 7f454c46 ]]; then
  printf 'Manus SDK is missing or a Git LFS pointer (%s); run git lfs pull for vendor/manus_sdk before building. No devices contacted.\n' "$sdk" >&2
  exit 2
fi
/usr/bin/g++ -std=c++17 -O2 -pthread \
  -I"$sdk_root/include" "$manus_root/rawviz.cpp" \
  -L"$sdk_root/lib" -l:libManusSDK_Integrated.so \
  -l:libudev.so.1 -l:libusb-1.0.so.0 -l:libz.so.1 \
  '-Wl,-rpath,$ORIGIN/../../../vendor/manus_sdk/lib' \
  '-Wl,-rpath,$ORIGIN/../../../../vendor/manus_sdk/lib' \
  -o "$manus_root/rawviz"
printf 'Built %s\n' "$manus_root/rawviz"
