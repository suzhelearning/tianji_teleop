#!/usr/bin/env bash
# Post-build fixup: the driver's unified ROS1/ROS2 CMakeLists does not emit the
# ament_prefix_path environment hook, so `source install/setup.bash` fails to put
# the package on AMENT_PREFIX_PATH (colcon's merged-install setup regenerates env
# from .dsv files and ignores the .sh hooks). Create a standalone hook and
# reference it from package.dsv via a literal `source;` entry. Mirrors the vendor
# build_ros2.sh fixup. Must run after every colcon build (package.dsv is
# regenerated each build).
set -euo pipefail

HOOK_DIR="install/odin_ros_driver_rev1/share/odin_ros_driver_rev1/hook"
AMENT_HOOK="${HOOK_DIR}/ament_prefix_path.sh"
PACKAGE_DSV="install/odin_ros_driver_rev1/share/odin_ros_driver_rev1/package.dsv"

[[ -d "${HOOK_DIR}" ]] || { echo "[fix_ament_hook] ${HOOK_DIR} not present (odin-lite not built) — skipping."; exit 0; }

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

if [[ -f "${PACKAGE_DSV}" ]] && ! grep -q 'ament_prefix_path.sh' "${PACKAGE_DSV}"; then
    echo "source;share/odin_ros_driver_rev1/hook/ament_prefix_path.sh" >> "${PACKAGE_DSV}"
    echo "added ament_prefix_path.sh source entry to package.dsv"
else
    echo "package.dsv already references ament_prefix_path.sh (or missing)"
fi
