# SPARK Headroom-Aware Feedforward Velocity QP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an isolated SPARK velocity-QP mode that preserves raw-palm feedforward when joint/task headroom is available and smoothly withdraws it before sustained hard-bound saturation.

**Architecture:** A small per-arm `SparkConstraintHeadroomGovernor` consumes the previous accepted controller output (`qdot`, derived `qddot`, task scales) and produces a causal scale for the next control cycle. `DualArmSparkGuidance` applies that scale only in the new algorithm mode; the existing feedforward path, QP formulation, hard limits and controller integration remain unchanged.

**Tech Stack:** C++20, Eigen, GoogleTest, YAML-CPP, MuJoCo Viewer telemetry, existing qpOASES velocity QP and Python replay/analyzer scripts.

## Global Constraints

- Do not modify existing algorithm behavior or rename existing CLI modes.
- Do not modify joint position, velocity, acceleration, jerk, braking, outward-only or collision hard limits.
- Do not add a second QP solve or a new runtime dependency.
- Use model reference state, not real-arm feedback, for the control loop.
- Keep left and right governor state independent.
- Run implementation inline in the current worktree; do not use subagents.

---

### Task 1: Constraint headroom governor

**Files:**
- Create: `include/tianji_qp_ik/spark_constraint_headroom.hpp`
- Create: `src/spark_constraint_headroom.cpp`
- Create: `tests/test_spark_constraint_headroom.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `SparkHeadroomFeedforwardVelocityQpConfig`, `ArmLimits`, `JointAccelerationLimitConfig`, accepted `qdot/qddot/task_scale` feedback and `dt`.
- Produces: `SparkConstraintHeadroomResult SparkConstraintHeadroomGovernor::update(const SparkConstraintHeadroomFeedback&, double)` and `reset()`.

- [x] **Step 1: Write failing tests** for neutral headroom, minimum-component selection, 0/1 smoothstep endpoints, fast reduction, slow recovery, invalid feedback and reset.
- [x] **Step 2: Run** `cmake --build build -j12 --target test_spark_constraint_headroom && ./build/test_spark_constraint_headroom` and verify the target or symbols are missing.
- [x] **Step 3: Implement** the focused governor with `h_velocity`, `h_acceleration`, `h_jerk`, `h_task`, `h_raw`, filtered headroom, scale and dominant-source diagnostics. Compute jerk from consecutive accepted `qddot`; return zero scale until valid history exists.
- [x] **Step 4: Re-run** the focused test and verify all cases pass.

### Task 2: Configuration and independent algorithm identity

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Modify: `tests/test_config.cpp`
- Modify: `tests/test_velocity_ik.cpp`

**Interfaces:**
- Produces enum `kSparkUpperQpoasesHeadroomFeedforwardVelocityQp`, predicate `usesSparkHeadroomFeedforwardVelocityQp()`, string `spark_upper_qpoases_headroom_feedforward_velocity_qp`, and validated `spark_headroom_feedforward_velocity_qp` configuration.

- [x] **Step 1: Add failing config/algorithm tests** proving the new name parses, participates in Spark upper qpOASES velocity guidance, remains distinct from fixed feedforward, and rejects invalid thresholds/time constants.
- [x] **Step 2: Run** the focused config and velocity-IK tests and verify they fail for the missing mode/config.
- [x] **Step 3: Add the enum, predicates, YAML parser, validation and defaults** exactly as specified in the design.
- [x] **Step 4: Re-run** focused tests and verify old and new algorithm identities both pass.

### Task 3: Guidance integration and causal feedback

**Files:**
- Modify: `include/tianji_qp_ik/spark_guidance.hpp`
- Modify: `src/spark_guidance.cpp`
- Modify: `tests/test_spark_guidance.cpp`

**Interfaces:**
- Adds `DualArmSparkGuidance::updateHeadroomFeedback(left, right, dt)`.
- Exposes each arm's `SparkConstraintHeadroomResult` in `SparkGuidanceArmDiagnostics`.

- [x] **Step 1: Add failing guidance tests** proving fixed feedforward is unchanged, the new mode starts at zero scale, accepted high-headroom feedback recovers, low headroom rapidly reduces twist, and left feedback cannot alter right scale.
- [x] **Step 2: Run** `cmake --build build -j12 --target test_spark_guidance && ./build/test_spark_guidance` and verify expected failures.
- [x] **Step 3: Add one governor per `ArmState`**, reset them with guidance state, apply their scale to both low/high-frequency Cartesian feedforward only in the new mode, and keep the position-feedback/weak posture targets active at all scales.
- [x] **Step 4: Re-run** guidance tests and confirm fixed-feedforward snapshots remain bit-for-bit equivalent where asserted.

### Task 4: Viewer, FRF and telemetry wiring

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `apps/benchmark_cartesian_frf.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `tests/test_cartesian_frf_runner.py`

**Interfaces:**
- Viewer passes accepted controller feedback to guidance after each velocity-QP solve.
- CSV adds per-arm velocity/acceleration/jerk/task/raw/filtered headroom, scale, state and dominant-source columns.

- [x] **Step 1: Add failing integration assertions** to `tests/test_pico_viewer_integration.py` for CLI selection, CSV columns `left_headroom_raw` through `right_headroom_dominant_source`, finite values, and a scale in `[0,1]`; register `pico_viewer_integration_spark_upper_qpoases_headroom_feedforward_velocity_qp` in `CMakeLists.txt`.
- [x] **Step 2: Run** `ctest --test-dir build -R 'pico_viewer_integration_spark_upper_qpoases_headroom_feedforward_velocity_qp|cartesian_frf_runner_integration' --output-on-failure` and verify failures are due to missing wiring.
- [x] **Step 3: Wire the new posture mode**, construct feedback from `ControllerDiagnostics::ik.qdot`, controller `previousAcceleration`, task scales and acceptance, and publish diagnostics without changing existing columns.
- [x] **Step 4: Re-run focused tests** and one 2-second headless smoke run; require accepted output and finite headroom telemetry.

### Task 5: Full regression and fixed-trace A/B

**Files:**
- Modify: `docs/verification/spark_feedforward_velocity_qp_results.md`
- Generate: `benchmark_results/spark_headroom_feedforward_20260817/`

**Interfaces:**
- Consumes the fixed v4 trace and existing comparison analyzer.
- Produces telemetry, joint telemetry, plots and an evidence-backed acceptance summary.

- [x] **Step 1: Build and run** `cmake --build build -j12 && ctest --test-dir build -j12 --output-on-failure`.
- [x] **Step 2: Replay** `/home/zj/current_robotics/TJ_arm/vr_data/converted_inputs/tjvr/pico_fast_motion_20260812_205428_v4.tjvr` through both fixed and headroom feedforward modes with model-state-only velocity control.
- [x] **Step 3: Generate aligned reports** using the last live control sample per `pico_sequence`, excluding `1:30` and `807:840`.
- [x] **Step 4: Verify** zero hard-limit violations/failures, delay at most 99 ms, mean at most 102.31 mm and P95 below 302.62 mm. If P95 misses, tune only the new mode's thresholds/time constants and repeat.
- [x] **Step 5: Update verification documentation** with exact parameters, metrics, caveats and output paths; do not commit implementation unless the user requests it.
