# PICO Gravity-leveled Palm Orientation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make full TCP and orientation-only palm calibration preserve HMD yaw while rejecting HMD pitch/roll, using time-coherent robust multi-frame capture.

**Architecture:** A shared pure SO(3) function constructs a gravity-leveled heading frame from the HMD pose. Existing `OrientationCaptureBuffer` supplies timestamp/epoch-coherent controller/head pairs and `solve_orientation()` computes the robust controller-to-palm rotation. The full TCP calibrator reuses these primitives for its fifth-stage window instead of saving one frame.

**Tech Stack:** Python 3.11, NumPy, ROS 2 Humble/rclpy, pytest, colcon.

## Global Constraints

- PICO world `+Z` is the gravity-up axis.
- Palm pitch/roll are world-horizontal; palm yaw follows HMD heading.
- Full TCP and orientation-only recalibration use identical math.
- Controller/head pair skew is at most 30 ms.
- Tracking epoch must be explicit and constant during capture.
- Default minimum sample count is 120 and orientation RMS gate is 0.05236 rad.
- Failed or incomplete calibration never overwrites the active artifact.
- TCP translation, wrist and arm-geometry schemas are unchanged.
- Do not commit changes.

---

### Task 1: Shared gravity-leveled SO(3) reference

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_orientation_core.py`
- Modify: `src/pico_bridge/test/test_pico_palm_orientation_core.py`

**Interfaces:**
- Produces: `gravity_leveled_heading_rotation(head_rotation: np.ndarray) -> np.ndarray`.
- Consumed by: `solve_orientation()` and full TCP calibration.

- [ ] Add a failing test with HMD yaw/pitch/roll and assert output keeps yaw while world Z is exact.
- [ ] Add a failing degeneracy test for a vertical HMD +X axis.
- [ ] Implement projected-heading frame construction and use it inside `solve_orientation()`.
- [ ] Run focused orientation-core tests.

### Task 2: Orientation-only service compatibility

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_orientation_core.py`
- Modify: `src/pico_bridge/test/test_pico_palm_orientation_service.py`

**Interfaces:**
- Consumes: timestamp/epoch-coherent `OrientationSample` values.
- Produces: orientation-only TCP update marked `gravity_leveled_hmd_heading`.

- [ ] Change integration input to include HMD tilt and assert saved palm target is yaw-only.
- [ ] Update artifact reference/method metadata.
- [ ] Run core and service tests.

### Task 3: Full TCP fifth-stage capture window

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_tcp_calibrator.py`
- Modify: `src/pico_bridge/test/test_pico_palm_tcp_calibrator.py`

**Interfaces:**
- Consumes: `/pico/pose/<side>_hand`, `/pico/pose/head`, `/pico/tracking_epoch`, `/pico/tracking_epoch/status`.
- Produces: complete `pico_palm_tcp_v2` candidate only after robust orientation success.

- [ ] Replace the old fifth-sample immediate-save test with a failing start-window test.
- [ ] Add tests for insufficient samples, pair skew/epoch rejection through the shared buffer, and translation preservation.
- [ ] Add explicit epoch subscriptions and `OrientationCaptureBuffer` state.
- [ ] Make fifth SPACE start capture; finalize from the keyboard owner thread after at least 120 samples.
- [ ] Save orientation covariance, RMS, sample count, epoch and leveled-reference metadata.
- [ ] Keep the position-stage retry and cancellation behavior unchanged.
- [ ] Run focused TCP tests.

### Task 4: Operator text, build and regression

**Files:**
- Modify: `scripts/calibrate_pico_arm.sh`
- Modify: `README.md`
- Modify: `README.zh-CN.md`
- Test: `src/pico_bridge/test/`

**Interfaces:**
- Produces: Chinese instruction that fifth SPACE starts a 1–2 second capture window.

- [ ] Update prompts and documentation.
- [ ] Build `pico_bridge` through the clean pixi environment.
- [ ] Run all `pico_bridge` tests and `git diff --check`.
- [ ] Report manual per-side calibration command without committing.
