# Tianji PICO Exclusive Startup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every normal Tianji PICO start remove historical driver, M0, and bridge instances before creating one fresh managed session.

**Architecture:** Add a focused Python `/proc` cleanup utility with exact target matching and staged signals. Invoke it from the Bash launcher only after build and ADB preflight, after terminating the managed tmux session, and before creating new windows.

**Tech Stack:** Python 3.11, Bash, pytest, tmux test doubles

## Global Constraints

- Clean matching processes across all work directories and ROS domains.
- Never match generic Python, ROS 2, paths, APK processes, or ADB forwarding.
- Fail closed if a matching process survives cleanup.
- Preserve `--status`, `--stop`, and standalone stop-script behavior.
- Work directly on the current `main` branch as explicitly requested.

---

### Task 1: Exact Historical Process Cleanup

**Files:**
- Create: `scripts/cleanup_tianji_pico_processes.py`
- Create: `src/pico_bridge/test/test_cleanup_tianji_pico_processes.py`

**Interfaces:**
- Produces: `find_target_processes(proc_root, excluded_pids)` and CLI exit status 0 only when no targets remain

- [ ] **Step 1: Write failing matching tests**

Create fake `/proc/<pid>/cmdline` entries for every specified launch file and executable, plus unrelated ROS/Python processes. Assert only exact targets are returned and excluded ancestor PIDs are omitted.

- [ ] **Step 2: Verify RED**

Run `pixi run pytest -q src/pico_bridge/test/test_cleanup_tianji_pico_processes.py`; expect import failure because the cleanup utility is absent.

- [ ] **Step 3: Implement exact matching and staged cleanup**

Read null-delimited cmdlines, match exact argument basenames, exclude self/ancestors, send SIGINT then SIGTERM then SIGKILL with bounded waits, and return nonzero if targets remain. Support `--proc-root` and `--dry-run` for deterministic tests.

- [ ] **Step 4: Verify Task 1**

Run `pixi run pytest -q src/pico_bridge/test/test_cleanup_tianji_pico_processes.py`; expect all tests to pass.

### Task 2: Preflight, Cleanup, and Fresh Session Orchestration

**Files:**
- Modify: `scripts/start_tianji_pico_teleop.sh`
- Modify: `src/pico_bridge/test/test_pico_m0_scripts.py`
- Modify: `README.md`
- Modify: `README.zh-CN.md`

**Interfaces:**
- Consumes: `cleanup_tianji_pico_processes.py`
- Produces: normal start always recreates the managed three-window session

- [ ] **Step 1: Change fake-launcher tests to require restart behavior**

Assert a second normal `--detach` start records two `new-session` operations and one intervening `kill-session`, while `--status` and `--stop` remain unchanged. Assert the cleanup utility is invoked before `new-session`.

- [ ] **Step 2: Verify RED**

Run the focused launcher test; expect failure because the current launcher returns early for an existing session.

- [ ] **Step 3: Implement launcher orchestration**

Move the existing-session handling after build/ADB preflight, kill the managed session if present, execute the cleanup utility, then create fresh windows. Preserve attach versus detach behavior after creation.

- [ ] **Step 4: Update documentation**

State in both README launcher sections that every normal start cleans historical instances across old work directories and ROS domains before starting a fresh chain.

- [ ] **Step 5: Full verification**

Run:

```bash
pixi run pytest -q src/pico_bridge/test/test_cleanup_tianji_pico_processes.py src/pico_bridge/test/test_pico_m0_scripts.py
python -m py_compile scripts/cleanup_tianji_pico_processes.py
bash -n scripts/start_tianji_pico_teleop.sh
bash -n scripts/stop_tianji_pico_teleop.sh
git diff --check
```

- [ ] **Step 6: Commit implementation**

Commit tests, cleanup utility, launcher, and dual-language documentation with message `fix: restart Tianji PICO pipeline exclusively`.
