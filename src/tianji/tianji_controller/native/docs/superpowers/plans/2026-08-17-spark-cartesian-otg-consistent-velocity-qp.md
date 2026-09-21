# SPARK Cartesian OTG Consistent Velocity QP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an isolated `spark_upper_qpoases_cartesian_otg_velocity_qp` mode whose Cartesian pose/twist and SPARK joint soft reference are generated from the same Cartesian OTG phase.

**Architecture:** Extend the SPARK guidance module with an opt-in OTG-consistent mode. A dedicated equality-constrained qpOASES wrapper lets the second position-IK stage preserve the OTG TCP task while optimizing the SPARK upper-limb shape; the Viewer consumes the resulting `CartesianReference` directly and leaves the historical mode untouched.

**Tech Stack:** C++17, Eigen, qpOASES, Ruckig Cartesian OTG, Pinocchio kinematics, MuJoCo, GoogleTest, Python replay analysis.

## Global Constraints

- Keep the current branch and worktree; do not modify or delete local `benchmark_results/` data.
- Do not change the existing `spark_upper_qpoases_velocity_qp` behavior.
- Do not change Velocity QP decision variables, weights, task scaling, or hard joint limits.
- Use model-reference state for feedback.
- Initialize every new IK/OTG state from the current model state on startup, reset, algorithm switch, and PICO resynchronization.
- Stage 2 may yield to Stage 1, but must never degrade an accepted OTG TCP solution beyond configured tolerances.
- Implement with TDD and run the recorded PICO trace for the final A/B.

---

### Task 1: Register the isolated algorithm

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_config.cpp`

**Interfaces:**
- Produces: `IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp`.
- Produces: `usesSparkOtgConsistentVelocityQp(IkAlgorithm) noexcept`.
- Produces CLI/config name `spark_upper_qpoases_cartesian_otg_velocity_qp`.

- [ ] **Step 1: Write failing parsing and classification tests**

Add a temporary YAML fixture based on the PICO profile with:

```yaml
ik:
  algorithm: spark_upper_qpoases_cartesian_otg_velocity_qp
```

Assert:

```cpp
EXPECT_EQ(config.ik_algorithm,
          IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp);
EXPECT_TRUE(usesSparkGuidance(config.ik_algorithm));
EXPECT_TRUE(usesHierarchicalVelocityQp(config.ik_algorithm));
EXPECT_TRUE(usesSparkOtgConsistentVelocityQp(config.ik_algorithm));
EXPECT_FALSE(usesSparkUpperQpoasesDirect(config.ik_algorithm));
EXPECT_EQ(toString(config.ik_algorithm),
          "spark_upper_qpoases_cartesian_otg_velocity_qp");
```

- [ ] **Step 2: Run RED**

Run:

```bash
cmake --build build -j"$(nproc)" --target test_config
```

Expected: compilation fails because the enum/helper do not exist.

- [ ] **Step 3: Add enum, helpers, parser, formatter and CLI option**

Classify the new mode as SPARK guidance plus hierarchical Velocity QP, but not direct qpos. Keep the old joint-reference mode distinguishable:

```cpp
inline bool usesSparkOtgConsistentVelocityQp(IkAlgorithm algorithm) noexcept {
  return algorithm ==
         IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp;
}
```

- [ ] **Step 4: Run GREEN**

Run:

```bash
cmake --build build -j"$(nproc)" --target test_config
./build/test_config
```

Expected: all config tests pass.

### Task 2: Add a fixed-size equality-constrained qpOASES seam

**Files:**
- Create: `include/tianji_qp_ik/equality_constrained_qpoases_solver.hpp`
- Create: `src/equality_constrained_qpoases_solver.cpp`
- Create: `tests/test_equality_constrained_qpoases_solver.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces `EqualityConstrainedQpProblem7` with `H`, `g`, `A`, `equality`, `lower`, and `upper`.
- Produces `EqualityConstrainedQpoasesSolver7::solve(const EqualityConstrainedQpProblem7&) -> SolverResult7`.
- Consumes existing `QpoasesConfig`, `SolverStatus`, `Vec7`, `Vec6`, `Mat77`, and `Mat67`.

- [ ] **Step 1: Write a failing constrained-QP test**

Use a diagonal objective with six exact equalities:

```cpp
EqualityConstrainedQpProblem7 problem;
problem.H.setIdentity();
problem.g.setZero();
problem.A.setZero();
problem.A.leftCols<6>().setIdentity();
problem.equality << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
problem.lower.setConstant(-1.0);
problem.upper.setConstant(1.0);

const SolverResult7 result = solver.solve(problem);
ASSERT_EQ(result.status, SolverStatus::kSolved);
EXPECT_TRUE((problem.A * result.qdot)
                .isApprox(problem.equality, 1.0e-9));
```

Also test invalid bounds and an infeasible equality.

- [ ] **Step 2: Run RED**

Run:

```bash
cmake --build build -j"$(nproc)" --target test_equality_constrained_qpoases_solver
```

Expected: target/header missing.

- [ ] **Step 3: Implement the fixed 7-variable/6-equality wrapper**

Use `qpOASES::SQProblem(7, 6)`, `lbA == ubA == equality`, hotstart when matrix dimensions are unchanged, and one reset/reinitialize fallback. Validate all matrices, bounds, finite values and equality residual before accepting the solution.

- [ ] **Step 4: Run GREEN**

Run:

```bash
cmake --build build -j"$(nproc)" --target test_equality_constrained_qpoases_solver
./build/test_equality_constrained_qpoases_solver
```

Expected: constrained, invalid and infeasible cases pass.

### Task 3: Generate `q_ik_otg` from the same OTG palm

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `include/tianji_qp_ik/spark_upper_qpoases_ik.hpp`
- Modify: `src/spark_upper_qpoases_ik.cpp`
- Modify: `tests/test_spark_upper_qpoases_ik.cpp`

**Interfaces:**
- Produces `SparkUpperQpoasesIk7::solveOtgConsistent(const SparkUpperArmTarget&, const Pose&, const Vec7&, deadline) -> SparkUpperIkResult`.
- Consumes `EqualityConstrainedQpoasesSolver7` from Task 2.
- Adds config keys `otg_position_tolerance_m`, `otg_orientation_tolerance_rad`, and `otg_continuity_weight` under `spark_upper_qpoases`.

- [ ] **Step 1: Write failing IK consistency tests**

Create a reachable `otg_pose` from a known joint state and a deliberately different raw SPARK elbow shape. Assert:

```cpp
const auto result = solver.solveOtgConsistent(
    raw_shape_target, otg_pose, previous_q, deadline);
ASSERT_TRUE(result.accepted) << result.detail;
const auto sample = robot.armKinematicsAt(side, result.q);
EXPECT_LT((sample.tcp_pose.position - otg_pose.position).norm(), 2.0e-3);
EXPECT_LT(poseErrorWorld(otg_pose, sample.tcp_pose).tail<3>().norm(),
          2.0e-2);
```

Add a test where the Stage 2 shape objective is incompatible; require the result to stay within TCP tolerance and remain closer to `previous_q` than a branch-flipped seed.

- [ ] **Step 2: Run RED**

Run:

```bash
cmake --build build -j"$(nproc)" --target test_spark_upper_qpoases_ik
```

Expected: `solveOtgConsistent` is missing.

- [ ] **Step 3: Implement Stage 1 pose-only iterations**

Build a box-constrained QP from:

```text
min ||J_tcp*delta_q - pose_error(T_otg, FK(q))||^2
  + damping*||delta_q||^2
  + otg_continuity_weight*||q+delta_q-q_previous||^2
```

Use the existing trust region, joint margin, line search, deadline and hotstart conventions. Do not call the historical direction-first Stage 1.

- [ ] **Step 4: Implement Stage 2 constrained shape iterations**

Build the shape objective from upper/forearm direction, elbow position and continuity. Constrain:

```text
J_tcp*delta_q = pose_error(T_otg, FK(q))
```

Use the Task 2 solver. Accept a candidate only if its exact nonlinear FK remains within configured TCP tolerances; otherwise return the accepted Stage 1 result.

- [ ] **Step 5: Run GREEN**

Run:

```bash
cmake --build build -j"$(nproc)" --target test_spark_upper_qpoases_ik test_config
./build/test_spark_upper_qpoases_ik
./build/test_config
```

Expected: historical and OTG-consistent IK tests pass.

### Task 4: Integrate Cartesian OTG inside SPARK guidance

**Files:**
- Modify: `include/tianji_qp_ik/spark_guidance.hpp`
- Modify: `src/spark_guidance.cpp`
- Modify: `tests/test_spark_guidance.cpp`

**Interfaces:**
- Adds `SparkPostureGuideMode::kOtgConsistentJointReferenceVelocity`.
- Adds `DualArmReferences cartesian_references` and `bool cartesian_references_valid` to `SparkGuidanceDiagnostics`.
- Owns one `CartesianReferenceGenerator` per arm in the new mode.

- [ ] **Step 1: Write failing guidance tests**

Feed a moving synthetic skeleton for multiple 5 ms cycles. Assert:

```cpp
EXPECT_TRUE(result.cartesian_references_valid);
EXPECT_GT(result.cartesian_references.left.twist.head<3>().norm(), 0.0);
EXPECT_TRUE(result.posture_tasks.left.active);
EXPECT_EQ(result.posture_tasks.left.source,
          JointVelocityPostureSource::kSparkJointReference);
```

Check `FK(result.left.q_ik)` against `result.cartesian_references.left.pose`, reset continuity, and stale bounded stopping. Also assert the historical joint-reference mode still emits direct zero-twist references.

- [ ] **Step 2: Run RED**

Run:

```bash
cmake --build build -j"$(nproc)" --target test_spark_guidance
```

Expected: new mode and diagnostics fields are missing.

- [ ] **Step 3: Implement same-phase guidance**

For each arm and cycle:

```cpp
reference = otg.update(raw_target.palm, Vec6::Zero(), stale, dt);
ik = solver.solveOtgConsistent(raw_target, reference.pose,
                               last_valid_q_ik, deadline);
posture = makePostureVelocity(ik.q, model.q, attack*Kq,
                              limits.velocity);
```

On reset, initialize OTG from the model TCP pose and `last_valid_q_ik` from model `q`. On Stage 2 failure use Stage 1; on complete IK failure hold the last accepted joint guide and reference.

- [ ] **Step 4: Run GREEN**

Run:

```bash
cmake --build build -j"$(nproc)" --target test_spark_guidance
./build/test_spark_guidance
```

Expected: new and historical guidance tests pass.

### Task 5: Route the new references through the Viewer and telemetry

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `README.md`

**Interfaces:**
- Consumes `SparkGuidanceDiagnostics::cartesian_references` from Task 4.
- Preserves `--algorithm spark_upper_qpoases_velocity_qp` behavior.
- Adds `--algorithm spark_upper_qpoases_cartesian_otg_velocity_qp`.

- [ ] **Step 1: Write a failing integration test**

Run the Viewer in the new mode with synthetic PICO v4 packets and assert:

```text
algorithm=spark_upper_qpoases_cartesian_otg_velocity_qp
otg_enabled=1
left_v_ref > 0 during motion
right_v_ref > 0 during motion
control_failures=0
```

Verify initial joint telemetry uses the configured initial posture and all joint reference derivatives are finite.

- [ ] **Step 2: Run RED**

Run:

```bash
cmake --build build -j"$(nproc)" --target tianji_qp_ik_viewer
ctest --test-dir build --output-on-failure -R pico_viewer_integration
```

Expected: the new CLI algorithm is rejected or reports zero reference twist.

- [ ] **Step 3: Route guidance-owned references**

When `usesSparkOtgConsistentVelocityQp()` is true, pass `spark_diagnostics.cartesian_references` to `DualArmController::step`. Do not run the Viewer-owned OTG a second time. Keep the old mode's direct zero-twist reference route unchanged.

- [ ] **Step 4: Add telemetry and usage documentation**

Expose OTG-consistent IK position/orientation residuals and reference validity using existing SPARK telemetry fields where possible. Document the new CLI name and state that it is an A/B mode, not a replacement for the historical baseline.

- [ ] **Step 5: Run GREEN**

Run:

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -R pico_viewer_integration -j1
```

Expected: all PICO Viewer integration profiles pass.

### Task 6: Recorded-trace A/B and final verification

**Files:**
- Create: `docs/verification/spark_cartesian_otg_consistent_velocity_qp_results.md`
- Reuse: `/home/zj/current_robotics/TJ_arm/vr_data/tools/replay_pico_udp_trace.py`
- Reuse local outputs under: `benchmark_results/spark_cartesian_otg_consistent_velocity_qp/`

**Interfaces:**
- Consumes both algorithm modes and the fixed v4 TJVR trace.
- Produces a source-backed comparison report; raw CSV remains untracked.

- [ ] **Step 1: Run the complete deterministic test suite**

Run:

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -j1
```

Expected: 100% pass.

- [ ] **Step 2: Replay the historical baseline**

Run the headless Viewer for 37 seconds with:

```text
--algorithm spark_upper_qpoases_velocity_qp
--control-level velocity
--model-state-only
```

Replay `pico_fast_motion_20260812_205428_v4.tjvr` and save telemetry/joint telemetry under the Task 6 result directory.

- [ ] **Step 3: Replay the OTG-consistent mode**

Repeat with only:

```text
--algorithm spark_upper_qpoases_cartesian_otg_velocity_qp
```

- [ ] **Step 4: Generate the comparison report**

Report unaligned and time-aligned TCP errors, OTG-to-IK residuals, final-three-second peak-to-peak, q/qdot/qddot/jerk percentiles, slack, task scaling, active bounds, control failures, deadline misses and solver timing.

- [ ] **Step 5: Verify acceptance and document gaps honestly**

The new mode is accepted only if it has no branch flip, meets the 2 mm/0.02 rad OTG-to-IK P95 residuals, respects all hard joint limits, has zero control failure/deadline miss, and either improves dynamic error or improves smoothness with quantified latency tradeoff.

