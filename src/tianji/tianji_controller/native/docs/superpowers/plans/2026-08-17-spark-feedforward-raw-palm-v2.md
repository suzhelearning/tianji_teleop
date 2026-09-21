# SPARK Feedforward Raw-Palm v2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `spark_upper_qpoases_feedforward_velocity_qp` track the latest SPARK palm pose with timestamp-aware palm-twist feedforward while using smoothed two-stage IK only as a weak joint-posture task.

**Architecture:** Add a small stateful SE(3) source-frame twist estimator with strict sequence, timestamp, median-dt, epoch, and discontinuity handling. Wire its output and the latest SPARK palm pose into the existing Cartesian servo, leaving the Velocity-Level QP and all hard constraints unchanged; retain `SparkFeedforwardReference7` solely for bounded posture guidance.

**Tech Stack:** C++17, Eigen, MuJoCo, Pinocchio, qpOASES, GoogleTest, CMake/CTest.

## Global Constraints

- Modify the existing `spark_upper_qpoases_feedforward_velocity_qp`; do not add or rename an algorithm mode.
- Keep the Velocity-Level QP decision variable and hard constraints unchanged.
- Keep Relative Mapping and the Cartesian Servo feedback form unchanged.
- Use the model state for Cartesian feedback.
- Do not derive Cartesian pose or twist from `FK(q_ff)` or `J(q_ff) qdot_ff`.
- Preserve all unrelated dirty worktree changes.
- Execute inline in the current worktree without subagents.

---

### Task 1: Timestamp-aware SPARK palm twist estimator

**Files:**
- Create: `include/tianji_qp_ik/spark_palm_twist_estimator.hpp`
- Create: `src/spark_palm_twist_estimator.cpp`
- Create: `tests/test_spark_palm_twist_estimator.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `Pose`, source `sequence`, source timestamp in nanoseconds, tracking epoch, and stream-discontinuity flag.
- Produces: `SparkPalmTwistEstimator::update(const Pose&, std::uint64_t, std::int64_t, std::uint64_t, bool) -> SparkPalmTwistDecision`, plus `reset()` and `twist()`.

- [ ] **Step 1: Write failing estimator tests**

  Cover first-frame zero output, constant translation and rotation, duplicate sequence hold, non-monotonic timestamp hold, median-dt dropout rejection, reversal attenuation, discontinuity reset, and linear/angular norm bounds.

- [ ] **Step 2: Run the focused test and confirm it fails**

  Run: `cmake --build build -j$(nproc) --target test_spark_palm_twist_estimator && ./build/test_spark_palm_twist_estimator`

  Expected: build failure because the estimator interface does not exist.

- [ ] **Step 3: Implement the estimator**

  Use source-frame updates only:

  ```cpp
  v_raw = (pose.position - previous_pose.position) / source_dt;
  w_raw = logSO3(pose.orientation * previous_pose.orientation.transpose()) /
          source_dt;
  ```

  Reject duplicate/non-monotonic/median-dt-invalid updates without replacing the previous accepted pose or twist. On epoch/discontinuity, synchronize pose and emit zero twist. Filter translation and rotation independently, decay stale direction on stationarity/reversal, and saturate vector norms.

- [ ] **Step 4: Run the estimator test and confirm it passes**

  Run: `cmake --build build -j$(nproc) --target test_spark_palm_twist_estimator && ./build/test_spark_palm_twist_estimator`

  Expected: all estimator cases pass.

### Task 2: Rewire feedforward guidance to raw SPARK palm

**Files:**
- Modify: `include/tianji_qp_ik/spark_guidance.hpp`
- Modify: `src/spark_guidance.cpp`
- Modify: `tests/test_spark_guidance.cpp`

**Interfaces:**
- Consumes: the estimator from Task 1 and existing `latest_targets_`, source metadata, `SparkFeedforwardReference7`, and posture-task structures.
- Produces: feedforward-mode `cartesian_targets` and `cartesian_references` whose pose equals the latest SPARK palm and whose twist equals the same palm estimator output multiplied by existing position/orientation feedforward gains.

- [ ] **Step 1: Replace old FK-based expectations with failing raw-palm tests**

  Assert that the target pose equals `result.left.target.palm`/`result.right.target.palm`, that constant palm motion creates finite same-direction twist, that a duplicate control tick holds the last twist, and that posture targets remain active with source `kSparkFeedforwardJointReference` without changing Cartesian pose.

- [ ] **Step 2: Run the guidance tests and confirm failure against the old implementation**

  Run: `cmake --build build -j$(nproc) --target test_spark_guidance && ./build/test_spark_guidance`

  Expected: raw-palm pose/twist assertions fail because the old code uses `FK(q_ff)` and `J(q_ff)qdot_ff`.

- [ ] **Step 3: Integrate one estimator per arm**

  Reset both estimators with guidance reset. Update them only through source sequence/timestamp metadata. Set:

  ```cpp
  cartesian_pose = latest_targets_.arm.palm;
  cartesian_twist.head<3>() = position_feedforward_gain * palm_twist.head<3>();
  cartesian_twist.tail<3>() = orientation_feedforward_gain * palm_twist.tail<3>();
  ```

  Keep the existing jerk-limited `q_ik` feedforward result and posture target. Do not add `J*qdot_posture` to the Cartesian command.

- [ ] **Step 4: Preserve Cartesian operation when posture reference rejects a frame**

  A rejected `q_ik`/posture target must retain the last accepted posture seed/reference, while a valid palm target continues to update the Cartesian task. Preserve existing bilateral commit semantics for accepted posture seeds.

- [ ] **Step 5: Run focused tests**

  Run: `cmake --build build -j$(nproc) --target test_spark_guidance test_velocity_ik && ctest --test-dir build --output-on-failure -R 'spark_(guidance|palm_twist)|velocity_ik'`

  Expected: all focused tests pass and no QP interface changes are required.

### Task 3: Regression and offline performance verification

**Files:**
- Modify: `docs/verification/spark_feedforward_velocity_qp_results.md`
- Preserve outputs under: `benchmark_results/`

**Interfaces:**
- Consumes: existing fast-PICO replay, Cartesian FRF benchmark, telemetry and constraint-report scripts.
- Produces: reproducible before/after tracking, delay, gain, overshoot, solver-failure, and joint-bound evidence.

- [ ] **Step 1: Run the complete compiled test suite**

  Run: `cmake --build build -j$(nproc) && ctest --test-dir build --output-on-failure -j$(nproc)`

  Expected: all tests pass.

- [ ] **Step 2: Run the existing deterministic fast-PICO replay comparison**

  Use the same 2134-frame trace and existing comparison runner used by the current verification document. Store output in a new timestamped directory without overwriting historical baselines.

- [ ] **Step 3: Run the Cartesian FRF matrix in parallel**

  Use the existing FRF runner with the same arm/axis/frequency/amplitude matrix and multi-core worker setting. Confirm low-frequency gain, 1--5 Hz group delay, and resonant peak from the generated summary.

- [ ] **Step 4: Check hard constraints and solver health**

  Verify zero control failures and zero position/velocity/acceleration/jerk bound violations in telemetry. Report any missed acceptance target rather than hiding it.

- [ ] **Step 5: Update verification documentation**

  Record commands, output directories, metrics, and the comparison against the old feedforward and plain SPARK Velocity-QP baselines in `docs/verification/spark_feedforward_velocity_qp_results.md`.

## Self-review

- Spec coverage: source timestamp validation, same-source pose/twist, reset behavior, posture isolation, unchanged QP, tests, replay, FRF, and hard-bound verification are each assigned to a task.
- Placeholder scan: no deferred implementation items or unspecified test categories remain.
- Type consistency: `SparkPalmTwistEstimator` and `SparkPalmTwistDecision` are introduced in Task 1 and consumed unchanged in Task 2.
