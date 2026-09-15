#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT_DIR/build"

# Default install prefix is build/install, can be overridden via argument or environment
# Usage: ./build.sh [INSTALL_PREFIX]
# Example: ./build.sh /usr/local
#          INSTALL_PREFIX=/opt/odin ./build.sh
if [[ -n "${1:-}" ]]; then
    INSTALL_PREFIX="$1"
elif [[ -n "${INSTALL_PREFIX:-}" ]]; then
    INSTALL_PREFIX="${INSTALL_PREFIX}"
else
    INSTALL_PREFIX="$BUILD_DIR/install"
fi

# Create install directory if it doesn't exist
mkdir -p "$INSTALL_PREFIX"

# Clean build directory
rm -rf "$BUILD_DIR"

# Configure with custom install prefix
cmake -S "$ROOT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"

# Build
cmake --build "$BUILD_DIR"

# Install (no sudo needed for local install)
if [[ "$INSTALL_PREFIX" == /usr* ]] || [[ "$INSTALL_PREFIX" == /opt* ]]; then
    sudo cmake --install "$BUILD_DIR"
else
    cmake --install "$BUILD_DIR"
fi

cp "$BUILD_DIR/compile_commands.json" "$ROOT_DIR/"

echo ""
echo "========================================"
echo "Build and install complete!"
echo "Install directory: $INSTALL_PREFIX"
echo "  - Headers: $INSTALL_PREFIX/include/"
echo "  - Library: $INSTALL_PREFIX/lib/"
echo "========================================"
echo ""
echo "To use a custom install directory:"
echo "  ./build.sh /your/custom/path"
echo "  or"
echo "  INSTALL_PREFIX=/your/custom/path ./build.sh"
echo ""