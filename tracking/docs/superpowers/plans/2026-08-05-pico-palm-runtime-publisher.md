# PICO Palm Runtime Publisher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Start both artifact-backed palm publishers and the palm-constrained skeleton filter from one launch command.

**Architecture:** Put artifact validation and SE(3) composition in a ROS-independent module. Add a side-specific, read-only ROS publisher executable and conditionally launch two instances alongside the existing filter.

**Tech Stack:** Python 3.11, NumPy, ROS 2 Humble `rclpy`, ROS launch, pytest, ament/colcon.

## Global Constraints

- Apply `T_controller_palm` exactly once.
- Runtime publisher must never modify an artifact.
- Preserve controller source timestamp and require frame `pico`.
- Left and right publishers remain independent.
- Invalid or missing artifact produces no palm output.
- Existing interactive calibrator behavior remains available.

---

### Task 1: Runtime transform core

**Files:**
- Create: `src/pico_bridge/scripts/pico_palm_tcp_runtime.py`
- Create: `src/pico_bridge/test/test_pico_palm_tcp_runtime.py`

- [ ] Write tests for valid artifact loading, side mismatch, transform direction, quaternion validation, and rotated TCP translation.
- [ ] Run focused test and verify import failure.
- [ ] Implement `TcpTransform`, `load_tcp_transform()` and `apply_tcp_transform()`.
- [ ] Run focused test and verify all cases pass.

### Task 2: Read-only ROS publisher

**Files:**
- Create: `src/pico_bridge/scripts/pico_palm_tcp_publisher.py`
- Modify: `src/pico_bridge/CMakeLists.txt`
- Test: `src/pico_bridge/test/test_pico_palm_tcp_runtime.py`

- [ ] Add a failing test for PoseStamped conversion preserving source stamp and producing frame `pico`.
- [ ] Implement side-specific publisher with `--side` and `--artifact`; do not create keyboard or artifact-writing paths.
- [ ] Install publisher and runtime module through CMake.
- [ ] Run focused tests.

### Task 3: One-command launch

**Files:**
- Modify: `src/pico_bridge/launch/start_pico_palm_skeleton_filter.launch.py`
- Modify: `src/pico_bridge/test/test_launch_integration.py`
- Modify: `docs/PICO_PALM_SKELETON_FILTER.md`

- [ ] Add failing launch-contract tests for `start_palm_publishers`, both TCP artifact arguments, two conditional publishers, and the filter.
- [ ] Add both publisher nodes guarded by `IfCondition` and keep filter unconditional.
- [ ] Document the one-command workflow and duplicate-publisher avoidance switch.
- [ ] Run launch and focused tests.

### Task 4: Full verification

- [ ] Run Python tests for all `pico_bridge` modules.
- [ ] Run `py_compile` for new scripts.
- [ ] Build and test `pico_bridge` with colcon.
- [ ] Run `colcon test-result --all` and `git diff --check`.
