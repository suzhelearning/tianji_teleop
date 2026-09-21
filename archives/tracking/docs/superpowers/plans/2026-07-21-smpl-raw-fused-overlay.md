# SMPL Raw and Fused Overlay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an optional same-coordinate raw PICO skeleton overlay to the existing fused SMPL MuJoCo visualizer.

**Architecture:** Generate a second, uniquely named and thinner set of MuJoCo geometry only when raw overlay is enabled. Keep primary and raw PoseArray caches independent, then update both geometry sets in the same render loop without making raw data a prerequisite for primary rendering.

**Tech Stack:** Python 3.11, ROS 2 rclpy, MuJoCo, NumPy, unittest/pytest.

## Global Constraints

- `--show-raw` defaults to false and preserves current behavior.
- `--raw-topic` defaults to `/pico/smpl`.
- Both skeletons share coordinates, scale, and viewer yaw.
- Raw input and stale state never block fused rendering.
- Raw geometry is thinner and semi-transparent green.

---

### Task 1: Parser and geometry contract

**Files:**
- Modify: `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`
- Modify: `src/pico_bridge/test/test_smpl_mujoco_geometry.py`

**Interfaces:**
- Produces: `build_parser()` options `show_raw: bool`, `raw_topic: str`.
- Produces: `_make_xml(show_raw: bool = False)` with optional `raw_joint_*`, `raw_bone_*`, and `raw_foot_*` names.

- [ ] **Step 1: Add failing tests** for parser defaults/flags and raw XML names/styles.
- [ ] **Step 2: Run** `pixi run python -m unittest src/pico_bridge/test/test_smpl_mujoco_geometry.py -v` and verify failure.
- [ ] **Step 3: Add parser options and optional raw XML bodies** with smaller green geometry.
- [ ] **Step 4: Rerun the focused tests** and verify success.
- [ ] **Step 5: Commit** with `feat: define raw SMPL overlay geometry`.

### Task 2: Independent raw subscription and rendering

**Files:**
- Modify: `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`
- Modify: `src/pico_bridge/test/test_smpl_mujoco_geometry.py`

**Interfaces:**
- Consumes: constructor arguments `raw_topic` and `show_raw`.
- Produces: independent raw PoseArray cache, receipt time, MuJoCo address arrays, and scene update.

- [ ] **Step 1: Add failing static/runtime-light tests** proving separate raw cache fields, optional subscription, and distinct geometry identifiers.
- [ ] **Step 2: Implement a shared PoseArray conversion helper** used by primary and raw callbacks, while retaining independent warning messages and timestamps.
- [ ] **Step 3: Update raw joints, bones, and feet independently** when raw data exists; apply raw stale colors without changing primary geometry.
- [ ] **Step 4: Pass parsed options** from `main()` into the visualizer constructor and preserve camera targeting from primary data.
- [ ] **Step 5: Run focused tests and commit** with `feat: overlay raw PICO skeleton in MuJoCo`.

### Task 3: Documentation and verification

**Files:**
- Modify: `docs/PICO_FOOT_IMU_FUSION.md`
- Modify: `pico_full_test_commands.txt`

- [ ] **Step 1: Document** the `--show-raw --raw-topic /pico/smpl` comparison command and colors.
- [ ] **Step 2: Run** `git diff --check`, `pixi run build`, and `pixi run test`.
- [ ] **Step 3: Confirm** `colcon test-result` reports zero failures.
- [ ] **Step 4: Commit** with `docs: add raw and fused skeleton overlay command`.
- [ ] **Step 5: Push** `feature/pico-foot-imu-fusion` to `personal`.
