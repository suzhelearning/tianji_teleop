#!/usr/bin/env bash
# Re-exec the calling script inside the workspace Pixi environment.
#
# Every user-facing bash/ entry point starts with:
#     source "$root/bash/pixi.bash" "$@"
# which either hands control to `pixi run --locked <this script + args>` or
# returns, after which the caller runs in the activated environment.
#
# Environment variables that must survive re-entry are exported here; anything
# passed to this file is forwarded verbatim so the script sees its own argv.

_tianji_pixi_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"

# Only the four operator terminals get a console log. Wrap before Pixi setup so
# activation failures are captured too; stdin remains the operator's real TTY.
_tianji_pixi_entry="$(cd -- "$(dirname -- "${BASH_SOURCE[1]}")" && pwd -P)/${BASH_SOURCE[1]##*/}"
_tianji_log_role=""
case "$_tianji_pixi_entry" in
  "$_tianji_pixi_root/bash/run_pico.sh") _tianji_log_role=pico ;;
  "$_tianji_pixi_root/bash/run_manus.sh") _tianji_log_role=manus ;;
  "$_tianji_pixi_root/bash/run_camera_views.sh") _tianji_log_role=camera ;;
  "$_tianji_pixi_root/bash/run_teleop.sh") _tianji_log_role=teleop ;;
esac
if [[ -n "$_tianji_log_role" && "${_TIANJI_ENTRY_LOG_REENTRY:-}" != "$_tianji_pixi_entry" ]]; then
  exec env _TIANJI_ENTRY_LOG_REENTRY="$_tianji_pixi_entry" \
    python3 "$_tianji_pixi_root/src/teleop_outputs/tianji/tianji_controller/tianji_controller/run_logging.py" \
    --log-root "$_tianji_pixi_root/tmp/logs/$_tianji_log_role" \
    -- bash "$_tianji_pixi_entry" "$@"
fi
# This is a one-handoff marker, not an inherited "logging is active" flag.
# A separately launched entry must get a fresh log even when it inherits a logdir.
unset _TIANJI_ENTRY_LOG_REENTRY

if [[ -z "${TIANJI_PIXI_ACTIVE:-}" ]]; then
  if ! command -v pixi >/dev/null 2>&1; then
    printf '%s\n' \
      "Pixi is not installed; no hardware was contacted." \
      "Install it from https://pixi.sh/latest/installation/ then run 'pixi install --locked'." >&2
    exit 1
  fi
  if [[ ! -r "$_tianji_pixi_root/pixi.toml" || ! -r "$_tianji_pixi_root/pixi.lock" ]]; then
    printf 'Missing Pixi manifest or lockfile under %s; no hardware was contacted.\n' \
      "$_tianji_pixi_root" >&2
    exit 1
  fi
  # Forward the already-parsed argv; the wrapper sees the same arguments it
  # would have seen had the user invoked it inside `pixi shell`.
  exec env TIANJI_PIXI_ACTIVE=1 \
    _TIANJI_ENTRY_LOG_REENTRY="${_tianji_log_role:+$_tianji_pixi_entry}" \
    pixi run --locked --manifest-path "$_tianji_pixi_root/pixi.toml" \
    bash "${BASH_SOURCE[1]}" "$@"
fi

# --- inside the environment -------------------------------------------------
export TIANJI_WORKSPACE="$_tianji_pixi_root"
# Drop cwd dependence: every entry point resolves resources from TIANJI_WORKSPACE.
cd -- "$_tianji_pixi_root"
