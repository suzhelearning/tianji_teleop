#!/usr/bin/env bash
set -euo pipefail

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Tracking workspace contains the project-owned SDK and both ROS drivers.
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"


ensure_sdk() {
    bash "${SCRIPT_DIR}/../../scripts/ensure_odin_sdk.sh"
}

if [[ ! -d "${WORKSPACE_DIR}/src" ]]; then
    echo "ROS workspace not found under ${WORKSPACE_DIR}/src." >&2
    exit 1
fi

if [[ -n "${ROS1_SETUP:-}" ]]; then
    # shellcheck disable=SC1090
    source "${ROS1_SETUP}"
fi

ensure_sdk

if ! command -v catkin_make >/dev/null 2>&1; then
    echo "catkin_make command not found. Source your ROS 1 environment or set ROS1_SETUP=/path/to/setup.bash." >&2
    exit 1
fi

export ROS_VERSION=1

# Build with all outputs inside WORKSPACE_DIR
cd "${WORKSPACE_DIR}"
catkin_make \
    --directory "${WORKSPACE_DIR}" \
    --build "${WORKSPACE_DIR}/build" \
    "$@"

echo "Build complete. Artifacts in ${WORKSPACE_DIR}/{build,devel}"
