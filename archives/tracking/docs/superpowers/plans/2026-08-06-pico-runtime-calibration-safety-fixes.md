# PICO Runtime and Calibration Safety Fixes Implementation Plan

> **For agentic workers:** Execute inline with test-driven development. Do not
> commit; the user requires the working tree to remain uncommitted.

**Goal:** Make PICO reset/reconnect and calibration artifact handling
fail-closed without changing nominal palm-constrained IK geometry.

**Architecture:** Add epoch ownership to runtime samples and reset only
world-dependent state. Reuse one strict artifact contract at activation and
runtime boundaries, make raw SMPL evidence non-blocking, and harden process and
epoch persistence lifecycle.

**Tech Stack:** ROS 2 Humble through Pixi Python 3.11, rclpy, rclcpp, NumPy,
pytest, GoogleTest, Bash.

## Global Constraints

- Preserve TCP, wrist positive-X, calibrated bone-length, and `correct_side()`
  nominal math.
- Do not modify or delete `recordings/`.
- Do not create Git commits.
- Every production change requires a failing regression test first.

---

### Task 1: Epoch-isolated palm skeleton filtering

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_skeleton_filter_node.py`
- Test: `src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py`

**Required behavior:** Palm/raw correction requires consistent explicit epoch;
epoch changes clear caches, baseline buffers, and temporal elbow history;
invalid epoch publishes raw fallback.

- [ ] Add tests for a transition during baseline and after a ready calibration.
- [ ] Verify the tests fail because old state survives and correction is not gated.
- [ ] Add epoch to cached samples and a single transition-reset helper.
- [ ] Verify focused filter tests pass.

### Task 2: Truly diagnostic-only raw SMPL geometry

**Files:**
- Modify: `src/pico_bridge/scripts/pico_arm_geometry_core.py`
- Modify: `src/pico_bridge/scripts/pico_arm_geometry_calibrator.py`
- Test: `src/pico_bridge/test/test_pico_arm_geometry_core.py`
- Test: `src/pico_bridge/test/test_pico_arm_geometry_calibrator.py`

**Required behavior:** Zero/invalid raw diagnostic vectors do not throw or
invalidate palm-derived structural geometry; diagnostics explicitly report
unavailable evidence.

- [ ] Add a regression test with valid palm wrist clouds and degenerate raw joints.
- [ ] Verify it fails with `right-angle raw arm vectors must be non-zero`.
- [ ] Make diagnostic calculations optional and serialize their availability.
- [ ] Verify geometry tests pass.

### Task 3: Strict artifact activation and explicit M0 inputs

**Files:**
- Modify: `scripts/calibrate_pico_arm.sh`
- Modify: `scripts/start_pico_m0.sh`
- Test: `src/pico_bridge/test/test_pico_calibration_menu.py`
- Test: `src/pico_bridge/test/test_pico_m0_scripts.py`

**Required behavior:** Activation validates schema 3, exact TCP/wrist hashes,
all Gates, quality, covariance, bounds, and epoch metadata. M0 reads only active
artifacts and never scans `recordings/`.

- [ ] Add tests for stale hashes, failed Gates, bad schema, and recordings scan.
- [ ] Verify the tests fail against current shallow validators/scanner.
- [ ] Implement strict validation and remove implicit candidate discovery.
- [ ] Verify shell-focused tests pass.

### Task 4: Wrist artifact, input, and epoch safety

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_wrist_calibrator.py`
- Modify: `src/pico_bridge/scripts/pico_palm_skeleton_filter_node.py`
- Test: `src/pico_bridge/test/test_pico_palm_wrist_calibrator.py`
- Test: `src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py`

**Required behavior:** Wrist artifacts satisfy schema/quality Gates; the
calibrator rejects non-finite/wrong-frame data and invalidates capture across an
epoch change.

- [ ] Add failing finite/frame/schema/quality/epoch tests.
- [ ] Implement finite solution checks and epoch-aware capture reset.
- [ ] Verify wrist and filter tests pass.

### Task 5: Interactive lifecycle and status

**Files:**
- Modify: `scripts/calibrate_pico_arm.sh`
- Test: `src/pico_bridge/test/test_pico_calibration_menu.py`

**Required behavior:** A failed single item returns to the interactive menu,
status reports artifact metadata/lineage, and Ctrl-C always stops the temporary
TCP publisher.

- [ ] Add failing menu recovery, status, and cleanup tests.
- [ ] Add per-step cleanup and non-terminal operation failure handling.
- [ ] Verify menu tests pass.

### Task 6: Concurrent durable tracking epoch allocation

**Files:**
- Modify: `src/pico_bridge/src/tracking_epoch_store.cpp`
- Test: `src/pico_bridge/test/test_tracking_epoch_store.cpp`

**Required behavior:** Concurrent processes reserve unique increasing epochs;
the committed state is durable across process failure.

- [ ] Add a fork-based concurrent reservation test.
- [ ] Verify duplicate reservations are reproducible without a lock.
- [ ] Add advisory locking and file/directory synchronization.
- [ ] Verify tracking epoch tests pass.

### Task 7: Full verification

- [ ] Run focused Python and C++ tests.
- [ ] Run `colcon test --packages-select pico_bridge` and require zero failures.
- [ ] Build `pico_bridge` in the Pixi environment.
- [ ] Run Bash syntax checks and `git diff --check`.
- [ ] Confirm `recordings/` contents were not modified by the implementation.
