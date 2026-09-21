# Tianji PICO tmux Launcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a one-command tmux launcher for the PICO-side Tianji Spark teleoperation pipeline and document the current TJVR v4 workflow in English and Chinese.

**Architecture:** A repository-root Bash script validates prerequisites, automatically re-enters the Pixi environment, and owns one fixed tmux session with `driver`, `m0`, and `bridge` windows. Existing startup scripts remain the process-level source of truth; the new launcher only orchestrates them and never starts the Tianji viewer.

**Tech Stack:** Bash 5, tmux, Pixi, ADB, ROS 2 Humble, pytest, Markdown.

## Global Constraints

- The managed tmux session name is exactly `pico_tianji_teleop`.
- The launcher starts only PICO driver, M0 correction, and TJVR v4 bridge processes.
- It must not start, stop, or modify the Tianji DLS/Spark MuJoCo viewer.
- Defaults remain ROS domain 120, localhost-only ROS, UDP destination `127.0.0.1:15000`, `robot_arm_segments`, and reach scale `0.95`.
- Only explicit `--stop` may terminate the launcher-owned tmux session.
- Both `README.md` and `README.zh-CN.md` must describe the same commands and protocol behavior.

---

### Task 1: Test the tmux launcher contract

**Files:**
- Modify: `src/pico_bridge/test/test_pico_m0_scripts.py`
- Test: `src/pico_bridge/test/test_pico_m0_scripts.py`

**Interfaces:**
- Consumes: repository scripts through their filesystem paths.
- Produces: executable contract for `scripts/start_tianji_pico_teleop.sh` and its `--detach`, `--status`, and `--stop` operations.

- [ ] **Step 1: Add source-level contract tests**

Add `TIANJI_TMUX_LAUNCHER = REPO_ROOT / "scripts" / "start_tianji_pico_teleop.sh"` and tests asserting that the script contains the fixed session/window names, invokes `start_pico_driver.sh` and `start_pico_m0.sh`, launches `start_tianji_mujoco_teleop.launch.py` with the documented v4 bridge defaults, and contains neither `tianji_qp_ik_viewer` nor `TJ_arm_control_DLS_IK`.

- [ ] **Step 2: Add behavioral tests with fake executables**

Create temporary fake `tmux`, `adb`, and `pixi` executables. Have fake tmux persist a session marker and append arguments to a log. Run the launcher with `--detach` and assert creation of exactly three windows named `driver`, `m0`, and `bridge`; run it again and assert no duplicate `new-session`; run `--status` and assert success; run `--stop` and assert only `kill-session -t pico_tianji_teleop` was requested.

- [ ] **Step 3: Run the focused test and verify RED**

Run:

```bash
pixi run python -m pytest src/pico_bridge/test/test_pico_m0_scripts.py -q
```

Expected: new tests fail because `scripts/start_tianji_pico_teleop.sh` does not exist.

---

### Task 2: Implement the tmux launcher

**Files:**
- Create: `scripts/start_tianji_pico_teleop.sh`
- Test: `src/pico_bridge/test/test_pico_m0_scripts.py`

**Interfaces:**
- Consumes: `scripts/start_pico_driver.sh`, `scripts/start_pico_m0.sh`, `install/local_setup.bash`, and the ROS launch file `start_tianji_mujoco_teleop.launch.py`.
- Produces: CLI `start_tianji_pico_teleop.sh [--detach|--status|--stop|--help]`.

- [ ] **Step 1: Implement argument parsing and management operations**

Use `set -euo pipefail`, resolve `repo_root` from `BASH_SOURCE`, and recognize only `--detach`, `--status`, `--stop`, and `--help`. `--status` calls `tmux list-windows -t pico_tianji_teleop`; `--stop` calls `tmux kill-session -t pico_tianji_teleop` only when that session exists; unknown arguments exit 2 with usage.

- [ ] **Step 2: Implement prerequisite validation and Pixi re-entry**

Check `tmux`, `pixi`, and `adb`, verify `install/local_setup.bash`, and verify `adb get-state` returns `device`. If `PIXI_PROJECT_ROOT` is not the repository, re-execute with:

```bash
exec pixi run bash "$repo_root/scripts/start_tianji_pico_teleop.sh" "${original_args[@]}"
```

Do not perform build or calibration writes.

- [ ] **Step 3: Implement the three-window session**

Create a detached session with `driver` as its first window, add `m0` and `bridge`, enable `remain-on-exit`, and send commands that export the four ROS environment variables before running the existing driver/M0 scripts or the bridge launch command. Configure the bridge with `127.0.0.1`, port `15000`, `robot_arm_segments`, and reach scale `0.95`.

- [ ] **Step 4: Implement duplicate handling and attach behavior**

If the session already exists, print a concise message. Return immediately for `--detach`; otherwise replace the launcher with `tmux attach-session -t pico_tianji_teleop`.

- [ ] **Step 5: Run focused tests and syntax verification**

Run:

```bash
bash -n scripts/start_tianji_pico_teleop.sh
pixi run python -m pytest src/pico_bridge/test/test_pico_m0_scripts.py -q
```

Expected: syntax succeeds and all focused tests pass.

- [ ] **Step 6: Commit launcher and tests**

```bash
git add scripts/start_tianji_pico_teleop.sh src/pico_bridge/test/test_pico_m0_scripts.py
git commit -m "feat: add Tianji PICO tmux launcher"
```

---

### Task 3: Update English and Chinese quick-start documentation

**Files:**
- Modify: `README.md`
- Modify: `README.zh-CN.md`

**Interfaces:**
- Consumes: launcher CLI from Task 2 and TJVR v4 constants in `tianji_teleop_protocol.hpp`.
- Produces: matching bilingual quick-start and troubleshooting documentation.

- [ ] **Step 1: Replace the Tianji PICO-side multi-terminal quick start**

Use this primary command in both files:

```bash
./scripts/start_tianji_pico_teleop.sh
```

Document `--detach`, `--status`, `--stop`, tmux window navigation, and the fact that the DLS/Spark viewer is not launched.

- [ ] **Step 2: Correct protocol and retargeting descriptions**

State TJVR v4, 656-byte atomic packets, complete corrected upper-limb positions and rotations, bilateral arm redundancy directions, `robot_arm_segments`, and default reach scale 0.95. Remove the obsolete scale 1.0, 160-byte, old viewer repository, and velocity/acceleration QP toggle descriptions from the Spark quick start.

- [ ] **Step 3: Keep an independent Spark viewer command**

Document the existing Tianji command under a clearly separate manual step using `/home/zj/current_robotics/TJ_arm/TJ_arm_control_DLS_IK`, `build_dls/tianji_qp_ik_viewer`, and `--algorithm spark_upper_qpoases`.

- [ ] **Step 4: Verify bilingual consistency and stale-text removal**

Run:

```bash
rg -n "160-byte|160 字节|position scale 1.0|位置比例固定为 1.0|TJ_arm_control_pico_mujoco_teleop_v1" README.md README.zh-CN.md
rg -n "TJVR v4|656-byte|656 字节|start_tianji_pico_teleop.sh|spark_upper_qpoases" README.md README.zh-CN.md
```

Expected: the first search returns no matches; the second finds matching current descriptions in both files.

- [ ] **Step 5: Commit documentation**

```bash
git add README.md README.zh-CN.md
git commit -m "docs: simplify Tianji Spark teleop startup"
```

---

### Task 4: Final verification and publication readiness

**Files:**
- Verify only: all files changed by Tasks 1-3.

**Interfaces:**
- Consumes: completed launcher, tests, and documentation.
- Produces: verified local `main` commits ready to push.

- [ ] **Step 1: Run build and PICO bridge tests**

```bash
pixi run build-core
pixi run bash -lc 'export ROS_VERSION=2; colcon test --base-paths src --packages-select pico_bridge --event-handlers console_direct+'
pixi run colcon test-result --test-result-base build/pico_bridge
```

Expected: build succeeds and pico_bridge reports zero failures.

- [ ] **Step 2: Run final script and documentation checks**

```bash
bash -n scripts/start_tianji_pico_teleop.sh scripts/start_pico_driver.sh scripts/start_pico_m0.sh
git diff --check
git status -sb
```

Expected: shell syntax and whitespace checks pass; only intentional commits remain ahead of `origin/main`.

- [ ] **Step 3: Review commit range**

```bash
git log --oneline origin/main..HEAD
git diff --stat origin/main..HEAD
```

Expected: one design commit, one plan commit, one launcher/test commit, and one documentation commit with no unrelated files.
