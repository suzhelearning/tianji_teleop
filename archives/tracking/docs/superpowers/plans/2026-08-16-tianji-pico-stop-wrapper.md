# Tianji PICO Stop Wrapper Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `./scripts/stop_tianji_pico_teleop.sh` as a safe standalone command for stopping the managed Tianji PICO tmux session.

**Architecture:** Implement a thin Bash wrapper that resolves its own directory and `exec`s the existing launcher with `--stop`. Reuse the launcher's fixed-session ownership and idempotency instead of adding process-name matching.

**Tech Stack:** Bash, pytest, tmux test doubles, Markdown

## Global Constraints

- Stop only the `pico_tianji_teleop` session managed by `start_tianji_pico_teleop.sh`.
- Do not use `pkill`, stop the PICO APK, or remove ADB forwarding.
- The command must work from any current directory and be executable.
- Work directly on the current `main` branch as requested.

---

### Task 1: Standalone Stop Command

**Files:**
- Create: `scripts/stop_tianji_pico_teleop.sh`
- Modify: `src/pico_bridge/test/test_pico_m0_scripts.py`
- Modify: `README.md`
- Modify: `README.zh-CN.md`

**Interfaces:**
- Consumes: `scripts/start_tianji_pico_teleop.sh --stop`
- Produces: executable command `./scripts/stop_tianji_pico_teleop.sh`

- [ ] **Step 1: Add failing contract and behavior tests**

Define `TIANJI_TMUX_STOPPER` beside the launcher test constant. Assert the script exists, is executable, contains no `pkill`, and invokes the launcher with `--stop`. Run it under the existing fake launcher environment and assert the session is stopped idempotently.

- [ ] **Step 2: Verify RED**

Run:

```bash
pixi run pytest -q src/pico_bridge/test/test_pico_m0_scripts.py -k tianji_tmux_stop
```

Expected: FAIL because the stop script does not exist.

- [ ] **Step 3: Add the minimal wrapper**

Create an executable script with:

```bash
#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
exec "$script_dir/start_tianji_pico_teleop.sh" --stop
```

- [ ] **Step 4: Update concise startup documentation**

Add `./scripts/stop_tianji_pico_teleop.sh` to the English and Chinese launcher management command blocks.

- [ ] **Step 5: Verify implementation**

Run:

```bash
pixi run pytest -q src/pico_bridge/test/test_pico_m0_scripts.py
bash -n scripts/stop_tianji_pico_teleop.sh
git diff --check
```

Expected: all tests pass and both checks are silent.

- [ ] **Step 6: Commit**

```bash
git add scripts/stop_tianji_pico_teleop.sh src/pico_bridge/test/test_pico_m0_scripts.py README.md README.zh-CN.md
git commit -m "feat: add Tianji PICO stop command"
```
