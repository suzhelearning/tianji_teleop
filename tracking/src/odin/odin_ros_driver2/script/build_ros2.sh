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

source "${WORKSPACE_DIR}/scripts/environment.sh" --build

ensure_sdk

if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon command not found. Source your ROS 2 environment or set ROS2_SETUP=/path/to/setup.bash." >&2
    exit 1
fi

export ROS_VERSION=2

# Build with all outputs inside WORKSPACE_DIR
cd "${WORKSPACE_DIR}"
colcon build \
    --symlink-install \
    --base-paths "${WORKSPACE_DIR}/src" \
    --build-base "${WORKSPACE_DIR}/build" \
    --install-base "${WORKSPACE_DIR}/install" \
    --packages-select odin_ros_driver_rev1 \
    "$@"

echo "Build complete. Artifacts in ${WORKSPACE_DIR}/{build,install}"

# Ensure AMENT_PREFIX_PATH can discover the package when sourcing install/setup.bash.
HOOK_DIR="${WORKSPACE_DIR}/install/odin_ros_driver_rev1/share/odin_ros_driver_rev1"
HOOK_FILE="${HOOK_DIR}/hook/cmake_prefix_path.sh"
AMENT_HOOK="${HOOK_DIR}/hook/ament_prefix_path.sh"
PACKAGE_DSV="${HOOK_DIR}/package.dsv"

if [[ -f "${HOOK_FILE}" ]]; then
    if [[ ! -f "${AMENT_HOOK}" ]] || grep -q '_colcon_prefix_sh_prepend_unique_value' "${AMENT_HOOK}"; then
        cat > "${AMENT_HOOK}" <<'EOF'
if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
    case ":${AMENT_PREFIX_PATH}:" in
        *":${COLCON_CURRENT_PREFIX}:"*) ;;
        *) export AMENT_PREFIX_PATH="${COLCON_CURRENT_PREFIX}:${AMENT_PREFIX_PATH}";;
    esac
else
    export AMENT_PREFIX_PATH="${COLCON_CURRENT_PREFIX}"
fi
EOF
        chmod +x "${AMENT_HOOK}"
    fi
    if ! grep -q 'AMENT_PREFIX_PATH' "${HOOK_FILE}"; then
        cat >> "${HOOK_FILE}" <<'EOF'
if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
    case ":${AMENT_PREFIX_PATH}:" in
        *":${COLCON_CURRENT_PREFIX}:"*) ;;
        *) export AMENT_PREFIX_PATH="${COLCON_CURRENT_PREFIX}:${AMENT_PREFIX_PATH}";;
    esac
else
    export AMENT_PREFIX_PATH="${COLCON_CURRENT_PREFIX}"
fi
EOF
    fi
    if [[ -f "${PACKAGE_DSV}" ]] && ! grep -q 'ament_prefix_path.sh' "${PACKAGE_DSV}"; then
        echo "source;share/odin_ros_driver_rev1/hook/ament_prefix_path.sh" >> "${PACKAGE_DSV}"
    fi
fi
