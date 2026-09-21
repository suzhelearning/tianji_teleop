# SPARK Feedforward Velocity-QP Output Continuity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the SPARK feedforward velocity QP from amplifying smooth joint references into hard-jerk boundary switching while preserving Cartesian response and arm posture.

**Architecture:** Extend the existing posture-task interface with a source-specific jerk-continuity weight. The hierarchical QP adds a soft objective around `qdot_prev + qddot_prev * dt`, while all existing hard constraints and algorithms remain unchanged by default.

**Tech Stack:** C++17, Eigen, yaml-cpp, GoogleTest, CMake/CTest, qpOASES, MuJoCo, Python fixed-trace replay.

## Global Constraints

- Keep `qdot` as the velocity-QP decision variable.
- Do not change hard joint position, velocity, acceleration, jerk, braking, or collision constraints.
- Apply the new objective only to `kSparkFeedforwardJointReference`.
- Keep all unrelated algorithms behaviorally unchanged through a zero default.
- Preserve the current branch and worktree and do not commit.

---

### Task 1: Posture-task jerk-continuity objective

**Files:**
- Modify: `include/tianji_qp_ik/velocity_ik.hpp`
- Modify: `src/hierarchical_qp.cpp`
- Test: `tests/test_hierarchical_qp.cpp`

**Interfaces:**
- Consumes: `ArmIkInput::qdot_prev`, `ArmIkInput::qddot_prev`, and `ArmIkInput::dt`.
- Produces: `JointVelocityPostureTask::jerk_smoothness_weight` and its exact quadratic objective contribution.

- [x] Add a failing builder test expecting Hessian increment `w_jerk I` and gradient increment `-w_jerk(qdot_prev + qddot_prev dt)` for `kSparkFeedforwardJointReference`.
- [x] Run `cmake --build build -j12 --target test_hierarchical_qp && ./build/test_hierarchical_qp --gtest_filter='HierarchicalQpBuilder.SparkFeedforwardJointReferenceAddsJerkContinuity'` and verify RED because the field/objective is absent.
- [x] Add the zero-default task field and the guarded objective contribution.
- [x] Re-run the focused test and verify GREEN.

### Task 2: Configuration and guidance wiring

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `src/spark_guidance.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Test: `tests/test_config.cpp`
- Test: `tests/test_spark_guidance.cpp`

**Interfaces:**
- Consumes: YAML key `spark_upper_qpoases.joint_reference_jerk_smoothness_weight`.
- Produces: validated `SparkHeadroomFeedforwardVelocityQpConfig::joint_reference_smoothness_weight`, `joint_reference_jerk_smoothness_weight`, and propagated posture-task weights.

- [x] Add failing configuration and guidance assertions for the new headroom-only `3.0e4` values while preserving the shared historical `3.0e3` value.
- [x] Build and run `test_config` and `test_spark_guidance`; verify RED on the missing field.
- [x] Add parsing, validation, YAML values, and headroom-feedforward-only guidance propagation.
- [x] Re-run focused tests and verify GREEN.

### Task 3: Regression and fixed-trace evidence

**Files:**
- Generate: `benchmark_results/pico_live/output_continuity_fix/`

**Interfaces:**
- Consumes: `/home/zj/current_robotics/TJ_arm/vr_data/converted_inputs/tjvr/pico_fast_motion_20260812_205428_v4.tjvr`.
- Produces: telemetry, joint telemetry, and before/after tracking/jerk statistics.

- [x] Build all targets and run all 76 CTest cases in bounded batches with `--output-on-failure`.
- [x] Replay the fixed trace through `spark_upper_qpoases_headroom_feedforward_velocity_qp` with `--model-state-only`, saving telemetry and joint telemetry.
- [x] Compare mean/P95 Cartesian error, P50/P95/P99 jerk, jerk saturation, hard-bound violations, and control failures against `/tmp/pico_shake_qgain4*.csv`.
- [x] Retain the implementation: control failures and hard-bound violations are zero, jerk saturation decreases from `49.1%/39.9%` to `23.2%/19.7%`, and mean Cartesian position error changes by only `+0.68/-0.05 mm`.
