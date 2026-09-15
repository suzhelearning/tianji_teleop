#!/bin/bash
# Generate public package for ros_driver
# Excludes .git, build directories and compiled files
# Usage: ./genpubpkg.sh [beta|release]

set -e

# Parse arguments
BUILD_TYPE="${1:-release}"
if [[ "$BUILD_TYPE" != "beta" && "$BUILD_TYPE" != "release" ]]; then
    echo "Usage: $0 [beta|release]"
    echo "  beta    - Beta version package"
    echo "  release - Release version package (default)"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_NAME="ros_driver"
VERSION=$(grep -oP '(?<=<version>)[^<]+' "$SCRIPT_DIR/package.xml" 2>/dev/null || echo "unknown")
DATE=$(date +%Y%m%d)
OUTPUT_NAME="${PACKAGE_NAME}_v${VERSION}_${BUILD_TYPE}_${DATE}"
OUTPUT_FILE="${SCRIPT_DIR}/${OUTPUT_NAME}.tar.xz"

echo "=== Generating public package ==="
echo "  Source:  $SCRIPT_DIR"
echo "  Version: $VERSION"
echo "  Type:    $BUILD_TYPE"
echo "  Output:  $OUTPUT_FILE"

# Create temp directory
TEMP_DIR=$(mktemp -d)
TEMP_PKG="$TEMP_DIR/$PACKAGE_NAME"

echo "  Copying files..."

# Copy ros_driver to temp, excluding unwanted files
rsync -a --progress \
    --exclude='.git' \
    --exclude='.git*' \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    --exclude='devel/' \
    --exclude='*.tar.xz' \
    --exclude='*.tar.gz' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.cache' \
    --exclude='CMakeCache.txt' \
    --exclude='CMakeFiles/' \
    --exclude='Makefile' \
    --exclude='cmake_install.cmake' \
    --exclude='*.o' \
    --exclude='*.a' \
    --exclude='*.so' \
    "$SCRIPT_DIR/" "$TEMP_PKG/"

echo "  Compressing..."

# Create tar.xz archive
tar -C "$TEMP_DIR" -cJf "$OUTPUT_FILE" "$PACKAGE_NAME"

# Cleanup temp directory
rm -rf "$TEMP_DIR"

# Show result
SIZE=$(du -h "$OUTPUT_FILE" | cut -f1)
echo ""
echo "=== Package created successfully ==="
echo "  File: $OUTPUT_FILE"
echo "  Size: $SIZE"
