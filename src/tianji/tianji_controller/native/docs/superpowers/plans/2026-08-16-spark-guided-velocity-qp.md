# Spark-Guided Velocity QP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an isolated `spark_guided_velocity_qp` PICO teleoperation mode that uses robot-length Spark upper-limb targets and position IK to select a continuous seven-joint posture while the existing velocity-level QP remains the final constrained Cartesian solver.

**Architecture:** Port the verified Pinocchio kinematics, Spark scaler, and two-stage qpOASES position IK from `TJ_arm_control_DLS_IK`. Feed its Ruckig-limited joint reference into the existing hierarchical velocity QP as a soft posture task, while feeding the geometrically consistent Spark palm pose into the existing Cartesian OTG and servo. Preserve `outward_only` as a hard barrier and keep every existing algorithm selectable.

**Tech Stack:** C++20, Eigen 3.4, Pinocchio 3.x, MuJoCo, qpOASES, Ruckig, yaml-cpp, GoogleTest, Python replay analysis, CMake/CTest.

## Global Constraints

- Work in the current branch and dirty worktree; do not create another worktree.
- Do not use subagents.
- Do not commit or push.
- Do not change current joint position, velocity, acceleration, jerk, braking, or outward-limit values.
- Do not replace the velocity-level QP decision variable.
- Keep existing algorithms and benchmark outputs intact.
- First validate only MuJoCo and offline replay; do not add real-robot servo behavior.

---

### Task 1: Pinocchio Arm Kinematics

**Files:**
- Modify: `pixi.toml`
- Modify: `CMakeLists.txt`
- Create: `include/tianji_qp_ik/pinocchio_arm_kinematics.hpp`
- Create: `src/pinocchio_arm_kinematics.cpp`
- Create: `tests/test_pinocchio_arm_kinematics.cpp`

**Interfaces:**
- Consumes: local Tianji URDF and `ArmSide`, `Vec7`, `ArmKinematicSample`.
- Produces: `PinocchioArmKinematics::sample(ArmSide, const Vec7&)` with TCP, shoulder, elbow and wrist poses/Jacobians in the same world convention as `MujocoRobot`.

- [ ] Write the Pinocchio-vs-MuJoCo FK/Jacobian test by porting the verified DLS/Spark test unchanged except for current joint-limit fixtures.
- [ ] Run `cmake --build build -j4 --target test_pinocchio_arm_kinematics` and confirm the target is missing before implementation.
- [ ] Add `libpinocchio = ">=3.8,<4"`, `find_package(pinocchio CONFIG REQUIRED)`, the source, target link, and test target.
- [ ] Port the verified kinematics implementation and keep frame names and world transform handling identical to the DLS/Spark baseline.
- [ ] Run the focused test and require all FK/Jacobian tolerances to pass.

### Task 2: Spark Skeleton Scaling

**Files:**
- Create: `include/tianji_qp_ik/spark_upper_retarget.hpp`
- Create: `src/spark_upper_retarget.cpp`
- Create: `tests/test_spark_upper_retarget.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: current TJVR v4 `PicoUpperLimbSkeleton`, corrected left/right palm poses, and robot segment geometry.
- Produces: `SparkUpperTargets` containing consistent shoulder, elbow, wrist, hand and palm targets for both arms.

- [ ] Port tests for robot segment length, rotation-priority mapping, position fallback, invalid segment rejection, and maximum rotation jump rejection.
- [ ] Run the new test target and confirm RED because Spark types are absent.
- [ ] Port `UpperSparkSkeletonScaler` and adapt only protocol field names when the current TJVR v4 structure differs.
- [ ] Preserve the baseline rule that the Spark hand position replaces the old relative-mapping Cartesian translation in this mode.
- [ ] Run the focused test and current PICO protocol/overlay tests.

### Task 3: Two-Stage Spark Position IK and Joint Reference

**Files:**
- Create: `include/tianji_qp_ik/spark_upper_qpoases_ik.hpp`
- Create: `src/spark_upper_qpoases_ik.cpp`
- Create: `include/tianji_qp_ik/spark_posture_reference.hpp`
- Create: `src/spark_posture_reference.cpp`
- Create: `tests/test_spark_upper_qpoases_ik.cpp`
- Create: `tests/test_spark_posture_reference.cpp`
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `SparkUpperArmTarget`, previous valid `q_spark`, Pinocchio kinematics, current arm limits, and control `dt`.
- Produces: accepted `q_spark`, Stage 1/2 diagnostics, and Ruckig-limited `q_spark_ref/qdot_spark_ref/qddot_spark_ref`.

- [ ] Port the direction Jacobian, two-stage task composition, trust-region, line-search, hotstart and deadline tests.
- [ ] Add a failing posture-reference test asserting velocity, acceleration, jerk, reset and stale-hold continuity.
- [ ] Port the verified two-stage IK implementation without changing task weights or candidate acceptance semantics.
- [ ] Implement `SparkPostureReference7` as a focused wrapper around existing `JointTrajectoryLimiter7`, including reset, hold, update and `qdot_posture = qdot_ref + K*(q_ref-q_model)`.
- [ ] Add validated `spark_guided_velocity_qp` configuration fields and copy the proven Spark stage parameters plus a distinct soft posture weight/gain.
- [ ] Run Spark IK, posture reference, config, qpoases and joint trajectory tests.

### Task 4: Velocity-QP Spark Posture Objective

**Files:**
- Modify: `include/tianji_qp_ik/types.hpp`
- Modify: `include/tianji_qp_ik/hierarchical_qp.hpp`
- Modify: `src/hierarchical_qp.cpp`
- Modify: `include/tianji_qp_ik/controller.hpp`
- Modify: `src/controller.cpp`
- Modify: `tests/test_hierarchical_qp.cpp`
- Modify: `tests/test_controller.cpp`

**Interfaces:**
- Consumes: active Spark posture velocity target, per-joint activation/weight, and current velocity-QP input.
- Produces: an additive soft cost `0.5*w*||W*(qdot-qdot_posture)||^2`; all hard QP bounds and Cartesian equality rows remain unchanged.

- [ ] Add a failing QP builder test that checks the exact Hessian and gradient contribution of the Spark posture target.
- [ ] Add a controller test asserting Spark posture can select redundancy without changing Cartesian equality or outward inequality rows.
- [ ] Extend the existing posture-task data structure with a named Spark source/diagnostic rather than adding a second competing QP path.
- [ ] Implement the additive objective and ensure zero activation produces bitwise-equivalent existing QP matrices.
- [ ] Run hierarchical QP, solver, controller, safety and acceleration-controller regression tests.

### Task 5: PICO Viewer Integration and Mode Isolation

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `tests/test_snapshot_exchange.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: accepted atomic PICO v4 frames and `--algorithm spark_guided_velocity_qp`.
- Produces: Spark targets for Cartesian OTG, Spark joint posture references for the velocity QP, Viewer overlay and CSV diagnostics.

- [ ] Add a failing integration test selecting `spark_guided_velocity_qp` and asserting Spark target validity, `outward_only`, velocity-level control, finite telemetry, and non-PICO mode isolation.
- [ ] Add algorithm parsing/string conversion and CLI selection without changing existing defaults.
- [ ] Construct Spark scaler/IK/reference state once in the control loop, update it only on accepted new PICO frames, and reset it on epoch/resynchronization transitions.
- [ ] Route scaled Spark palm poses into `TargetManager/CartesianOtg` only in the new mode; preserve old Relative Mapping for every other mode.
- [ ] Pass Spark posture targets into both arm velocity-QP calls and keep PICO arm-angle directions disconnected.
- [ ] Add overlay and CSV fields for targets, stages, joint references, saturation and limit occupancy.
- [ ] Run all PICO Viewer tests serially.

### Task 6: Deterministic Replay and Acceptance

**Files:**
- Modify: `scripts/check_pico_wrist_stability.py`
- Create: `scripts/compare_spark_guided_velocity_qp.py`
- Create: `tests/test_compare_spark_guided_velocity_qp.py`
- Create: `docs/verification/spark_guided_velocity_qp_results.md`

**Interfaces:**
- Consumes: Cartesian and joint CSV outputs from baseline outward-only QP, DLS/Spark baseline, and Spark-guided velocity QP.
- Produces: deterministic pass/fail metrics and a saved comparison report.

- [ ] Add a failing analyzer fixture for simultaneous multi-joint velocity saturation and individual J3/J5-J7 hard-limit surfing; retain the existing `+/-pi` branch detector.
- [ ] Implement P50/P95/P99/max Cartesian error, bound occupancy, multi-joint saturation, q/dq/ddq/jerk, solver failure and cycle-time comparisons.
- [ ] Re-run the exact trace `vr_data/converted_inputs/tjvr/pico_fast_motion_20260812_205428.tjvr` serially for each available algorithm.
- [ ] Generate fresh CSV and report under a new benchmark result directory without overwriting historical outputs.
- [ ] Require no hard-bound violation, no inward upper-arm violation, no sustained wrist/elbow branch instability, and lower saturation/limit occupancy than the current outward-only run.
- [ ] Run `cmake --build build -j4`, full `ctest --output-on-failure -j4`, analyzer tests, replay check and `git diff --check`.
