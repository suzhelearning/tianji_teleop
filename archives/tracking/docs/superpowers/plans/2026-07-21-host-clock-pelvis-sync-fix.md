# Host-Clock Pelvis Synchronization Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Synchronize PICO pelvis frames with Odin Lite high-frequency odometry inside the C++ runtime without comparing unrelated device clocks, while closing the remaining validation, restart, configuration, and integration-test gaps.

**Architecture:** ROS callbacks record a shared host `steady_clock` receipt timestamp and retain device timestamps only for per-stream continuity. Calibration estimates residual receive-path lag from motion; runtime keeps that lag in memory, queues PICO frames until high-rate Odin samples bracket their host-time target, and interpolates Odin poses before rigidly re-anchoring the skeleton. The persisted file contains only spatial installation extrinsics and quality metrics that remain valid across device restarts.

**Tech Stack:** ROS 2 Humble, rclcpp, Eigen3, yaml-cpp, GoogleTest, pytest/rclpy, pixi/colcon.

## Global Constraints

- Run build and tests through pixi; source `install/setup.bash` before ROS commands.
- Never compare PICO `header.stamp` with Odin or ROS wall time.
- Raw `/pico/smpl` and `/raw/odom/odin_highfreq` remain unchanged.
- Installation YAML never persists a device-clock or transport-delay offset.
- Corrected output is suppressed when input, synchronization, or calibration validity checks fail.
- Do not stage or commit `src/pico_bridge/config/imu900_feet.yaml`, APK/DEB artifacts, recordings, or daily-summary files.

---

### Task 1: Make host receipt time explicit in the solver API

**Files:**
- Modify: `src/pico_odin/include/pico_odin/extrinsic_solver.hpp`
- Modify: `src/pico_odin/src/extrinsic_solver.cpp`
- Modify: `src/pico_odin/test/test_extrinsic_solver.cpp`

**Interfaces:**
- Produces: `HostTimedPose { double receipt_sec; Pose3 pose; }`.
- Produces: `ReceiveLagEstimate estimate_receive_lag(...)` for calibrator and runtime use.
- Produces: `SolverResult::receive_lag_sec`, which is session-only and is not serialized.

- [ ] **Step 1: Write failing tests for unrelated device epochs and public receive-lag estimation**

Add a synthetic test that assigns arbitrary, unrelated source epochs outside `HostTimedPose`, feeds host receipt times separated by a known 24 ms lag, and expects `estimate_receive_lag()` to recover `0.024 ± 0.006` seconds. Rename existing `TimedPose::stamp_sec` assertions to `HostTimedPose::receipt_sec` so accidental device-time use no longer compiles.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
pixi run bash -lc 'cmake --build build/pico_odin --target test_extrinsic_solver -j2 && ./build/pico_odin/test_extrinsic_solver'
```

Expected: compilation fails because `HostTimedPose` and `estimate_receive_lag` do not exist.

- [ ] **Step 3: Implement the explicit host-time API**

Replace the ambiguous sample type with:

```cpp
struct HostTimedPose {
  double receipt_sec{0.0};
  Pose3 pose;
};

struct ReceiveLagEstimate {
  bool valid{false};
  double lag_sec{0.0};
  double correlation{0.0};
  bool at_boundary{false};
};

ReceiveLagEstimate estimate_receive_lag(
  const std::vector<HostTimedPose> & pico,
  const std::vector<HostTimedPose> & odin,
  const SolverOptions & options);
```

Move the existing correlation search behind this public function and make every interpolation/sortedness check use `receipt_sec`. Keep lag sign defined as `odin_target_receipt = pico_receipt + lag_sec`.

- [ ] **Step 4: Run solver tests and verify GREEN**

Run the focused command from Step 2. Expected: all `ExtrinsicSolver` tests pass.

---

### Task 2: Stop persisting session timing in the extrinsics file

**Files:**
- Modify: `src/pico_odin/include/pico_odin/extrinsics_file.hpp`
- Modify: `src/pico_odin/src/extrinsics_file.cpp`
- Modify: `src/pico_odin/test/test_extrinsics_file.cpp`

**Interfaces:**
- Produces: schema version 3 documents with no `time_offset_sec` field.
- Consumes: legacy schema 1/2 files; ignores their stored time offset.

- [ ] **Step 1: Write failing schema-3 and legacy-ignore tests**

Assert that a newly saved document contains `schema_version: 3`, does not contain `time_offset_sec`, and round-trips spatial extrinsics and remaining quality metrics. Load schema 1/2 fixtures containing a large time offset and assert no runtime-facing timing value is produced.

- [ ] **Step 2: Run and verify RED**

```bash
pixi run bash -lc 'cmake --build build/pico_odin --target test_extrinsics_file -j2 && ./build/pico_odin/test_extrinsics_file'
```

Expected: current schema is 2 and still serializes `time_offset_sec`.

- [ ] **Step 3: Implement schema 3**

Set the current schema to 3, remove persisted time offset from `CalibrationMetrics`, and retain `time_correlation` as a calibration quality metric. Accept schemas 1 and 2 by parsing and discarding `quality.time_offset_sec`; do not expose it to `RuntimeAlignment`.

- [ ] **Step 4: Run and verify GREEN**

Run Step 2 again. Expected: all `ExtrinsicsFile` tests pass.

---

### Task 3: Reject invalid ROS poses before they enter core state

**Files:**
- Modify: `src/pico_odin/include/pico_odin/se3.hpp`
- Modify: `src/pico_odin/test/test_se3.cpp`
- Modify: `src/pico_odin/src/odin_pelvis_calibrator_node.cpp`
- Modify: `src/pico_odin/src/odin_pelvis_runtime_node.cpp`

**Interfaces:**
- Produces: `from_pose_msg_checked(const geometry_msgs::msg::Pose &, double quaternion_norm_tolerance)`.

- [ ] **Step 1: Write failing finite-position and quaternion-normalization tests**

Add tests expecting `std::invalid_argument` for NaN/Inf position, zero quaternion, and quaternion norm outside a default `1e-3` tolerance. A valid normalized pose must round-trip unchanged.

- [ ] **Step 2: Run and verify RED**

```bash
pixi run bash -lc 'cmake --build build/pico_odin --target test_se3 -j2 && ./build/pico_odin/test_se3'
```

Expected: NaN translation and scaled quaternions are currently accepted.

- [ ] **Step 3: Implement checked conversion and use it at ROS boundaries**

```cpp
inline Pose3 from_pose_msg_checked(
  const geometry_msgs::msg::Pose & message,
  double quaternion_norm_tolerance = 1e-3)
{
  const Eigen::Vector3d translation(
    message.position.x, message.position.y, message.position.z);
  const Eigen::Quaterniond rotation(
    message.orientation.w, message.orientation.x,
    message.orientation.y, message.orientation.z);
  if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
      std::abs(rotation.norm() - 1.0) > quaternion_norm_tolerance) {
    throw std::invalid_argument("pose contains non-finite or non-normalized values");
  }
  return Pose3{normalized(rotation), translation};
}
```

Use this function in both node callbacks so invalid samples never reach continuity monitors, buffers, session state, or publishers.

- [ ] **Step 4: Run and verify GREEN**

Run Step 2 and all current `pico_odin` unit tests.

---

### Task 4: Convert installation calibration to host receipt time

**Files:**
- Modify: `src/pico_odin/src/odin_pelvis_calibrator_node.cpp`
- Modify: `src/pico_odin/include/pico_odin/calibration_session.hpp`
- Modify: `src/pico_odin/test/test_calibration_session.cpp`

**Interfaces:**
- Consumes: `HostTimedPose` and `steady_seconds()` receipt timestamps.
- Device timestamps are passed only to `PoseStationarity`/`PoseStreamContinuity` for same-stream checks.

- [ ] **Step 1: Write a failing calibration-session test with unrelated source epochs**

Drive PICO source timestamps near 500 seconds and Odin source timestamps near 9000 seconds while host receipt times share a common 0–5 second timeline. Assert readiness, excitation, and solving depend on receipt time rather than either source epoch.

- [ ] **Step 2: Run and verify RED**

```bash
pixi run bash -lc 'cmake --build build/pico_odin --target test_calibration_session -j2 && ./build/pico_odin/test_calibration_session'
```

Expected: current session samples have only one ambiguous timestamp.

- [ ] **Step 3: Implement callback clock separation**

In each callback capture `const double receipt = steady_seconds()` exactly once. Store `HostTimedPose{receipt, pose}` in the session; use `stamp_seconds(message.header.stamp)` only in the corresponding stream's stationarity/continuity object. Rename `max_clock_offset_sec`/`min_clock_correlation` parameters and diagnostics to `max_receive_lag_sec`/`min_receive_lag_correlation`.

Reject calibrator startup unless `pico_topic == "/pico/smpl"` and `odin_topic == "/raw/odom/odin_highfreq"`, preventing corrected-output feedback.

- [ ] **Step 4: Run and verify GREEN**

Run Step 2 and `test_extrinsic_solver`.

---

### Task 5: Add runtime host-time buffering, pending PICO queues, and per-run lag estimation

**Files:**
- Create: `src/pico_odin/include/pico_odin/host_time_synchronizer.hpp`
- Create: `src/pico_odin/src/host_time_synchronizer.cpp`
- Create: `src/pico_odin/test/test_host_time_synchronizer.cpp`
- Modify: `src/pico_odin/CMakeLists.txt`
- Modify: `src/pico_odin/src/odin_pelvis_runtime_node.cpp`
- Modify: `src/pico_odin/include/pico_odin/runtime_alignment.hpp`
- Modify: `src/pico_odin/src/runtime_alignment.cpp`
- Modify: `src/pico_odin/test/test_runtime_alignment.cpp`

**Interfaces:**
- Produces: `HostTimeSynchronizer::add_odin(receipt_sec, pose)`.
- Produces: `HostTimeSynchronizer::add_pico(receipt_sec, root_pose)` for online lag observation.
- Produces: `HostTimeSynchronizer::interpolate_for_pico(receipt_sec)`.
- Produces: `HostTimeSynchronizer::reset()` and diagnostic `receive_lag_sec()`.

- [ ] **Step 1: Write failing synchronizer tests**

Cover: unrelated source epochs are irrelevant; interpolation waits until Odin brackets a PICO target; zero-lag fallback works before motion; known lag is estimated from moving synthetic poses; lag updates are bounded/smoothed; reset clears lag and buffers; stale targets are rejected.

- [ ] **Step 2: Run and verify RED**

```bash
pixi run bash -lc 'cmake --build build/pico_odin --target test_host_time_synchronizer -j2'
```

Expected: target does not exist.

- [ ] **Step 3: Implement the core synchronizer**

Use bounded deques of `HostTimedPose`. Interpolate only when two Odin samples bracket `pico_receipt + receive_lag`; otherwise return `std::nullopt`. Accumulate rolling PICO/Odin root histories, call `estimate_receive_lag` only after sufficient motion and sample count, reject boundary/low-correlation estimates, and update accepted lag with configurable exponential smoothing.

- [ ] **Step 4: Integrate pending skeleton queues in the runtime node**

On PICO callback, validate and enqueue the complete message plus its host receipt timestamp. On every high-frequency Odin callback, append the Odin sample, update lag estimation, and drain raw/fused queues whose targets are now bracketed. Drop pending frames that exceed `max_pending_age_sec`. Keep raw and fused queues independent and preserve each original message header on output except for the configured frame ID.

Remove persisted timing from `RuntimeAlignment`; it now owns only spatial installation and per-run world alignment transforms.

- [ ] **Step 5: Run and verify GREEN**

Run the new synchronizer test, `test_runtime_alignment`, and existing solver tests.

---

### Task 6: Make restart handling complete and physical validation configurable

**Files:**
- Modify: `src/pico_odin/include/pico_odin/extrinsic_solver.hpp`
- Modify: `src/pico_odin/src/extrinsic_solver.cpp`
- Modify: `src/pico_odin/src/odin_pelvis_calibrator_node.cpp`
- Modify: `src/pico_odin/src/odin_pelvis_runtime_node.cpp`
- Modify: `src/pico_odin/launch/odin_select.launch.py`
- Modify: `src/pico_odin/test/test_extrinsic_solver.cpp`
- Modify: `src/pico_odin/test/test_pose_stream_continuity.cpp`
- Modify: `src/pico_odin/test/test_launch_integration.py`

**Interfaces:**
- Produces solver options for min/max rear offset X, maximum absolute Y/Z, and maximum rear-prior angle.
- Produces independent low/high-rate Odin continuity monitors that clear one shared runtime session.

- [ ] **Step 1: Write failing bounds and dual-rate restart tests**

Verify custom bounds accept/reject a synthetic solution predictably. Verify a low-rate-only timestamp rollback or pose-origin jump clears runtime synchronization state before corrected low-rate odometry can publish.

- [ ] **Step 2: Run and verify RED**

Run `test_extrinsic_solver`, `test_pose_stream_continuity`, and `test_launch_integration`; expect missing options/launch arguments.

- [ ] **Step 3: Implement configurable bounds and independent continuity monitors**

Add solver parameters `mount_x_min_m`, `mount_x_max_m`, `mount_abs_y_max_m`, `mount_abs_z_max_m`, and `mount_rear_angle_max_rad`, validate their ordering/ranges, and replace hard-coded constants. Give low- and high-frequency Odin streams separate `PoseStreamContinuity` instances. Any discontinuity calls one function that clears host-time buffers, pending PICO queues, receive-lag estimate, and per-run alignment before publishing.

- [ ] **Step 4: Run and verify GREEN**

Run the tests from Step 2.

---

### Task 7: Expand ROS integration coverage and update operator documentation

**Files:**
- Modify: `src/pico_odin/test/test_runtime_ros_integration.py`
- Create: `src/pico_odin/test/test_calibrator_ros_integration.py`
- Modify: `src/pico_odin/CMakeLists.txt`
- Modify: `README.md`
- Modify: `README.zh-CN.md`
- Modify: `pico_full_test_commands.txt`

**Interfaces:**
- Tests public ROS topics and executable behavior only.

- [ ] **Step 1: Add failing ROS tests**

Add cases for: deliberately unrelated PICO/Odin source epochs; raw-only and fused-only independent outputs; both outputs together; non-identity rigid correction of all 24 joints; missing calibration file; stale synchronization; NaN input rejection; and low-rate restart suppression. Add a calibrator process test that exercises raw-topic enforcement and a synthetic one-cycle calibration without hardware.

- [ ] **Step 2: Run and verify RED**

```bash
pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --event-handlers console_direct+'
```

Expected: new cases fail against the pre-fix runtime behavior.

- [ ] **Step 3: Finish ROS wiring and documentation**

Expose host-time synchronization queue, lag search/smoothing, pending-age, restart, and physical-bound parameters with portable defaults. Update both READMEs and the command sheet to state that source `header.stamp` epochs may differ, runtime synchronization is C++ host-receipt based, and timing state is rebuilt every run rather than loaded from YAML.

- [ ] **Step 4: Run package tests and verify GREEN**

Run Step 2. Expected: every `pico_odin` test passes.

---

### Task 8: Full verification and final review

**Files:**
- Verify all files changed by Tasks 1–7.

- [ ] **Step 1: Build the complete selected workspace**

```bash
pixi run build
```

Expected: 7 packages build successfully.

- [ ] **Step 2: Run the complete test suite**

```bash
pixi run test
pixi run colcon test-result --test-result-base build --verbose
```

Expected: zero errors and zero failures.

- [ ] **Step 3: Inspect scope and whitespace**

```bash
git diff --check
git status --short
```

Expected: no whitespace errors; unrelated dirty artifacts remain unstaged and unchanged.

- [ ] **Step 4: Review against the design**

Confirm that no cross-stream `header.stamp` comparison remains, no timing offset is serialized, every ROS input is validated before buffering, both Odin rates revoke stale session alignment, and all corrected topics suppress invalid/stale output.
