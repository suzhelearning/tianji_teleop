# PICO Calibration Interaction Polish Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make arm-geometry capture start with Space, improve stage readability, and give interactive users an explicit retry/menu/exit choice after geometry completes or is rejected.

**Architecture:** Keep calibration math, gates, capture artifacts, and non-interactive exit codes unchanged. Add a TTY-safe Space key helper in the geometry calibrator and keep post-run navigation solely in the top-level interactive Bash menu.

**Tech Stack:** Bash, Python 3.11, ROS 2 Humble/rclpy, pytest, colcon.

## Global Constraints

- Rejected geometry candidates remain in `recordings/` and are never activated.
- `left|right geometry` remains non-interactive after the geometry process returns and preserves its exit code.
- Do not commit or delete recordings.

---

### Task 1: Space-triggered geometry capture and readable stage output

**Files:**
- Modify: `src/pico_bridge/scripts/pico_arm_geometry_calibrator.py`
- Modify: `src/pico_bridge/scripts/pico_arm_geometry_pixi.sh`
- Test: `src/pico_bridge/test/test_pico_arm_geometry_calibrator.py`
- Test: `src/pico_bridge/test/test_pico_m0_scripts.py`

**Interfaces:**
- Consumes: `--start-mode space|auto`.
- Produces: TTY Space start with `q` cancellation and blank lines around every stage.

- [ ] Write tests that reject Enter as the documented default and require Space prompts/blank stage separation.
- [ ] Run focused tests and observe failure.
- [ ] Add a terminal-state-restoring Space reader and stage spacing.
- [ ] Run focused tests and observe success.

### Task 2: Explicit interactive completion menu

**Files:**
- Modify: `scripts/calibrate_pico_arm.sh`
- Test: `src/pico_bridge/test/test_pico_calibration_menu.py`

**Interfaces:**
- Consumes: geometry exit status and selected side.
- Produces: `1) 返回主菜单`, `2) 重新进行当前骨长标定`, `3) 退出`.

- [ ] Write failing menu-contract tests.
- [ ] Run focused tests and observe failure.
- [ ] Add the post-geometry choice loop without changing non-interactive behavior.
- [ ] Run focused tests and observe success.

### Task 3: Installation and regression verification

**Files:**
- Verify: `install/pico_bridge/lib/pico_bridge/pico_arm_geometry_calibrator`

**Interfaces:**
- Consumes: Tasks 1-2 source changes.
- Produces: refreshed installed executable used by `ros2 run`.

- [ ] Build `pico_bridge` with `--symlink-install`.
- [ ] Run all direct Python tests.
- [ ] Run `colcon test` and require zero failures.
- [ ] Run `bash -n`, `git diff --check`, and confirm no calibration process remains.
