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
    pixi run --locked --manifest-path "$_tianji_pixi_root/pixi.toml" \
    bash "${BASH_SOURCE[1]}" "$@"
fi

# --- inside the environment -------------------------------------------------
export TIANJI_WORKSPACE="$_tianji_pixi_root"
# Drop cwd dependence: every entry point resolves resources from TIANJI_WORKSPACE.
cd -- "$_tianji_pixi_root"
