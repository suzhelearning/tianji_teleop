#!/usr/bin/env bash
# Single activation helper for every workspace entry point.
#
# It does NOT build a Python environment: the Pixi environment running this
# script already *is* the environment. What it does is:
#   * assert the running interpreter is Python 3.12 on ROS 2 Jazzy,
#   * source the matching colcon overlay when one has been built,
#   * clear the legacy overlay/Humble variables that a stale shell may carry.
#
# Usage:  source bash/environment.sh [--build]
#   --build   skip the workspace overlay (used while building it)

_tianji_env_error() {
  printf '%s\n' "$*" >&2
  return 2
}

# --- 1. running environment -------------------------------------------------
if [[ -z "${CONDA_PREFIX:-}" ]]; then
  _tianji_env_error "Not running inside the Pixi environment. Use 'pixi run <task>' or 'pixi shell'."
fi

export TIANJI_WORKSPACE="${TIANJI_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)}"
# Tested same-host RGB transport; preserve an operator's explicit profile.
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${FASTRTPS_DEFAULT_PROFILES_FILE:-$TIANJI_WORKSPACE/config/fastdds.xml}}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$FASTDDS_DEFAULT_PROFILES_FILE}"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  _tianji_env_error "ROS_DISTRO is unset: source this file inside 'pixi run'/'pixi shell', not in a bare shell."
fi
if [[ "$ROS_DISTRO" != "jazzy" ]]; then
  _tianji_env_error "Expected the ROS 2 Jazzy environment, found ROS_DISTRO='$ROS_DISTRO'."
fi

# --- 2. interpreter ----------------------------------------------------------
# Fall back to the environment's own python when the caller has not chosen one.
if [[ -z "${TIANJI_PYTHON:-}" ]]; then
  TIANJI_PYTHON="$CONDA_PREFIX/bin/python"
fi
if [[ ! -x "$TIANJI_PYTHON" ]]; then
  _tianji_env_error "Python interpreter missing: $TIANJI_PYTHON"
fi
export TIANJI_PYTHON
export TIANJI_ENVIRONMENT="${TIANJI_ENVIRONMENT:-$(basename "$CONDA_PREFIX")}"

# --- 3. stale overlays -------------------------------------------------------
# A shell that previously sourced a *different* ROS SDK or workspace overlay
# must not leak into this one. The Pixi environment's own prefixes are kept:
# `AMENT_PREFIX_PATH` is what the ament index is read through (colcon and
# rosidl's typesupport lookup both depend on it), and `CMAKE_PREFIX_PATH`
# carries the environment's CMake package locations.
unset ROS_LOCALHOST_ONLY PYTHONHOME

_strip_stale_paths() {
  local variable="$1"
  local kept=() entry
  local current="${!variable-}"
  [[ -n "$current" ]] || return 0
  while IFS= read -r entry; do
    [[ -n "$entry" ]] || continue
    case "$entry" in
      "${CONDA_PREFIX}"|"${CONDA_PREFIX}"/*) kept+=("$entry") ;;
      "${TIANJI_WORKSPACE}/install/${TIANJI_ENVIRONMENT}"|"${TIANJI_WORKSPACE}/install/${TIANJI_ENVIRONMENT}"/*) kept+=("$entry") ;;
      *) : ;;
    esac
  done <<< "${current//:/$'\n'}"
  if (( ${#kept[@]} )); then
    local joined
    joined="$(IFS=:; printf '%s' "${kept[*]}")"
    printf -v "$variable" '%s' "$joined"
    export "${variable?}"
  else
    unset "$variable"
  fi
}

# Keep only this environment and its selected overlay. Old tracking/install and
# other workspace environments must not survive simply because they share root.
_strip_stale_paths AMENT_PREFIX_PATH
_strip_stale_paths CMAKE_PREFIX_PATH
_strip_stale_paths COLCON_PREFIX_PATH
_strip_stale_paths PYTHONPATH
# LD_LIBRARY_PATH is inherited from the activation and must stay intact, so it
# is filtered the same way rather than cleared.
_strip_stale_paths LD_LIBRARY_PATH
unset -f _strip_stale_paths

# --- 4. workspace overlay ----------------------------------------------------
overlay="${TIANJI_WORKSPACE}/install/${TIANJI_ENVIRONMENT}/local_setup.bash"
if [[ "${1:-}" == "--build" ]]; then
  return 0
fi
if [[ -r "$overlay" ]]; then
  # colcon's generated setup scripts reference unset variables before defining
  # them, which trips `set -u` in the entry-point wrappers.
  _tianji_nounset=false
  [[ $- != *u* ]] || _tianji_nounset=true
  set +u
  # shellcheck disable=SC1090
  source "$overlay"
  [[ "$_tianji_nounset" != true ]] || set -u
  unset _tianji_nounset
elif [[ -n "${TIANJI_ALLOW_UNBUILT:-}" ]]; then
  return 0
else
  # Not fatal: pure-Python entry points (compression, visualization, config
  # checks) run without a built workspace. Anything that needs a message
  # package fails later with an explicit "not built" error from the tool.
  printf '%s\n' \
    "Note: workspace overlay not built yet (${overlay})." \
    "      Run 'pixi run build' before using the ROS nodes." >&2
fi
