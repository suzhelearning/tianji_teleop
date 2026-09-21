# Spark Direct / Pose Velocity-QP A/B Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add independently selectable Spark-direct and Spark-pose-only velocity-QP modes and compare them with the preserved Spark-Ruckig mode on the same PICO v4 trace.

**Architecture:** Extend the existing Spark guidance module with an explicit posture-reference policy: Ruckig, direct proportional joint guide, or disabled. All three policies share the same corrected-skeleton scaler, palm targets, Cartesian OTG, Cartesian velocity servo, velocity-level QP, hard dynamic bounds and outward barrier; only the construction of the soft posture objective changes.

**Tech Stack:** C++17, Eigen, Pinocchio, qpOASES, MuJoCo, GoogleTest, Python 3 replay/CSV analysis, CMake/CTest.

## Global Constraints

- Preserve `spark_guided_velocity_qp` behavior and every existing algorithm.
- Add `spark_direct_velocity_qp` and `spark_pose_velocity_qp` as explicit CLI/config names.
- Change no joint position, velocity, acceleration, jerk or braking limit value.
- Apply the existing Spark-only hard-jerk enablement identically to all three Spark modes.
- Force `outward_only` for all three Spark modes.
- Initial evaluation is MuJoCo/model-state-only; do not command real hardware.
- Keep the current branch/worktree and do not commit or push without a separate user request.
- Implement sequentially in the current agent; do not dispatch subagents.

---

### Task 1: Algorithm identities and common Spark-mode classification

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `tests/test_config.cpp`
- Modify: `tests/test_velocity_ik.cpp`

**Interfaces:**
- Consumes: existing `IkAlgorithm`, `parseIkAlgorithm`, `toString`, `usesHierarchicalVelocityQp`.
- Produces: `IkAlgorithm::kSparkDirectVelocityQp`, `IkAlgorithm::kSparkPoseVelocityQp`, and `usesSparkVelocityQp(IkAlgorithm) noexcept`.

- [ ] **Step 1: Add failing parse, string and backend-classification tests**

Add assertions equivalent to:

```cpp
EXPECT_EQ(parseIkAlgorithm("spark_direct_velocity_qp"),
          IkAlgorithm::kSparkDirectVelocityQp);
EXPECT_EQ(parseIkAlgorithm("spark_pose_velocity_qp"),
          IkAlgorithm::kSparkPoseVelocityQp);
EXPECT_EQ(toString(IkAlgorithm::kSparkDirectVelocityQp),
          "spark_direct_velocity_qp");
EXPECT_TRUE(usesSparkVelocityQp(IkAlgorithm::kSparkPoseVelocityQp));
EXPECT_TRUE(usesHierarchicalVelocityQp(
    IkAlgorithm::kSparkDirectVelocityQp));
```

- [ ] **Step 2: Run the focused tests and confirm RED**

Run:

```bash
pixi run cmake --build build -j4 --target test_config test_velocity_ik
```

Expected: compilation fails because the two enum values and
`usesSparkVelocityQp` do not exist.

- [ ] **Step 3: Implement the enum, parser, string conversion and shared predicate**

Use one common predicate:

```cpp
inline bool usesSparkVelocityQp(IkAlgorithm algorithm) noexcept {
  return algorithm == IkAlgorithm::kSparkGuidedVelocityQp ||
         algorithm == IkAlgorithm::kSparkDirectVelocityQp ||
         algorithm == IkAlgorithm::kSparkPoseVelocityQp;
}
```

Make `usesHierarchicalVelocityQp` return true for all three Spark modes.

- [ ] **Step 4: Run focused tests and confirm GREEN**

Run:

```bash
pixi run cmake --build build -j4 --target test_config test_velocity_ik
pixi run ./build/test_config
pixi run ./build/test_velocity_ik
```

Expected: both test binaries pass.

---

### Task 2: Spark guidance policies

**Files:**
- Modify: `include/tianji_qp_ik/spark_guidance.hpp`
- Modify: `src/spark_guidance.cpp`
- Modify: `tests/test_spark_guidance.cpp`

**Interfaces:**
- Consumes: `SparkUpperTargets`, `SparkUpperQpoasesIk7`, `SparkPostureReference7`, `DualArmJointVelocityPostureTasks`.
- Produces: `enum class SparkPostureGuideMode { kRuckig, kDirect, kDisabled }` and a constructor argument selecting it.

- [ ] **Step 1: Add a failing test for direct posture guidance**

Construct guidance with `SparkPostureGuideMode::kDirect`, feed the existing
valid bilateral v4 fixture, call `step`, and assert:

```cpp
EXPECT_TRUE(result.accepted);
EXPECT_TRUE(result.left.ik.accepted);
EXPECT_TRUE(result.posture_tasks.left.active);
EXPECT_EQ(result.posture_tasks.left.source,
          JointVelocityPostureSource::kSparkSoftQp);
EXPECT_FALSE(result.left.reference.accepted);
EXPECT_TRUE(result.posture_tasks.left.target.allFinite());
```

Also check each direct target component is bounded by the arm's URDF velocity
limit.

- [ ] **Step 2: Run the direct test and confirm RED**

Run:

```bash
pixi run cmake --build build -j4 --target test_spark_guidance
```

Expected: compilation fails because `SparkPostureGuideMode` is missing.

- [ ] **Step 3: Implement direct guidance without advancing Ruckig**

After accepted bilateral Spark IK, compute each target as:

```cpp
Vec7 direct = config_.spark_upper_qpoases.posture_position_gain *
              (q_ik - q_model);
direct = direct.cwiseMax(-limits.velocity).cwiseMin(limits.velocity);
```

Populate the soft Spark posture task directly. Do not call
`SparkPostureReference7::update` in this policy.

- [ ] **Step 4: Run the direct test and confirm GREEN**

Run `pixi run ./build/test_spark_guidance`.

Expected: all Spark guidance tests pass.

- [ ] **Step 5: Add a failing pose-only test**

Construct guidance with `SparkPostureGuideMode::kDisabled`, feed the same
fixture, and assert:

```cpp
EXPECT_TRUE(result.accepted);
EXPECT_TRUE(result.target_valid);
EXPECT_FALSE(result.left.ik.accepted);
EXPECT_FALSE(result.right.ik.accepted);
EXPECT_FALSE(result.posture_tasks.left.active);
EXPECT_FALSE(result.posture_tasks.right.active);
EXPECT_TRUE(result.cartesian_targets.left.position.allFinite());
```

- [ ] **Step 6: Run the pose-only test and confirm RED**

Expected failure: current `step` always runs Spark IK and emits a posture task.

- [ ] **Step 7: Implement pose-only early acceptance**

After target blending and Cartesian target assignment, return accepted
diagnostics before constructing the IK deadline when the policy is disabled:

```cpp
if (posture_mode_ == SparkPostureGuideMode::kDisabled) {
  result.left.accepted = true;
  result.right.accepted = true;
  result.accepted = true;
  result.detail = "spark_pose_guidance_accepted";
  return result;
}
```

Keep IK/reference diagnostics at their zero/inactive defaults.

- [ ] **Step 8: Run all guidance tests and confirm GREEN**

Run:

```bash
pixi run cmake --build build -j4 --target test_spark_guidance
pixi run ./build/test_spark_guidance
```

Expected: direct, pose-only and preserved Ruckig tests all pass.

---

### Task 3: Viewer selection, telemetry and integration isolation

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `CMakeLists.txt`
- Modify: `README.md`

**Interfaces:**
- Consumes: `usesSparkVelocityQp`, `SparkPostureGuideMode`, existing PICO v4 Viewer path.
- Produces: CLI-selectable `--algorithm spark_direct_velocity_qp` and `--algorithm spark_pose_velocity_qp`.

- [ ] **Step 1: Add failing CLI integration cases for both algorithms**

Extend the Python argument choices and add CTest entries using velocity control,
model-state feedback and the existing v4 packet sender. For direct mode assert
Spark IK is active and a posture task is emitted. For pose-only assert targets
are valid while IK/posture activity stays zero.

- [ ] **Step 2: Run both integration cases and confirm RED**

Run:

```bash
pixi run cmake -S . -B build -G Ninja -DBUILD_TESTING=ON
pixi run cmake --build build -j4 --target tianji_qp_ik_viewer
pixi run ctest --test-dir build -R 'pico_viewer_integration_spark_(direct|pose)' --output-on-failure
```

Expected: CLI rejects the new names or the new CTests fail their mode-specific
diagnostics.

- [ ] **Step 3: Map algorithms to guidance policies in the Viewer**

Use this exact mapping:

```cpp
kSparkGuidedVelocityQp -> SparkPostureGuideMode::kRuckig
kSparkDirectVelocityQp -> SparkPostureGuideMode::kDirect
kSparkPoseVelocityQp   -> SparkPostureGuideMode::kDisabled
```

Replace single-value Spark checks with `usesSparkVelocityQp`. Apply PICO and
velocity-control validation, `outward_only`, scaled Cartesian targets,
Spark-only hard jerk and telemetry for all three modes.

- [ ] **Step 4: Update CLI help and README commands**

Document all three algorithm names and state that they differ only in posture
reference construction.

- [ ] **Step 5: Run Viewer integration tests and confirm GREEN**

Run:

```bash
pixi run cmake --build build -j4 --target tianji_qp_ik_viewer
pixi run ctest --test-dir build -R 'pico_viewer_integration_(spark|velocity|acceleration)' --output-on-failure
```

Expected: all selected integration tests pass.

---

### Task 4: Deterministic three-way replay and hard-bound audit

**Files:**
- Modify: `scripts/compare_spark_guided_velocity_qp.py`
- Modify: `tests/test_compare_spark_guided_velocity_qp.py`
- Create: `docs/verification/spark_velocity_qp_three_way_results.md`
- Generate: `benchmark_results/spark_velocity_qp_three_way/*.csv`

**Interfaces:**
- Consumes: the three Viewer algorithm names and
  `vr_data/converted_inputs/tjvr/pico_fast_motion_20260812_205428_v4.tjvr`.
- Produces: one telemetry CSV and one joint CSV per algorithm plus a Markdown
  comparison table.

- [ ] **Step 1: Add failing analyzer assertions for algorithm identity and Spark activity**

Extend the analyzer summary to report the final algorithm name and averages for
`left_spark_posture_active`, `right_spark_posture_active`,
`left_spark_ik_accepted` and `right_spark_ik_accepted`. Add a synthetic test
that distinguishes Ruckig/direct from pose-only.

- [ ] **Step 2: Run the analyzer test and confirm RED**

Run `pixi run python tests/test_compare_spark_guided_velocity_qp.py`.

Expected: missing summary keys fail the test.

- [ ] **Step 3: Implement analyzer fields and confirm GREEN**

Run the same command; expect all Python tests to pass.

- [ ] **Step 4: Replay all three modes headlessly on unique UDP ports**

For each mode run the Viewer for 27 seconds with `--model-state-only`,
`--telemetry`, and `--joint-telemetry`, then replay the 2134-frame v4 trace.
Require Viewer exit code zero and `control_failures=0`.

- [ ] **Step 5: Audit all joint references**

For every non-reset sample require q, qdot, qddot and jerk to lie within the
corresponding logged lower/upper columns with tolerance `1e-6`.

- [ ] **Step 6: Generate the three-way report**

Report live non-stale Cartesian P50/P95/P99/max, cycle P99/max, IK/posture
activity, resets, failures, deadline misses and all hard-bound violation counts.
State that metrics use the same input trace and model-state route.

- [ ] **Step 7: Run the complete regression suite**

Run:

```bash
pixi run cmake --build build -j4
pixi run ctest --test-dir build --output-on-failure
git diff --check
```

Expected: all tests pass and diff check produces no output.

- [ ] **Step 8: Start graphical A/B replays sequentially**

Run each new mode with `--pico-skeleton-overlay` against the same v4 trace,
first `spark_direct_velocity_qp`, then `spark_pose_velocity_qp`. Leave no
background Viewer or replay process after the user finishes comparison.

---

### Task 5: Processed Spark skeleton overlay

**Files:**
- Modify: `include/tianji_qp_ik/pico_skeleton_overlay.hpp`
- Modify: `src/pico_skeleton_overlay.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_pico_skeleton_overlay.cpp`

**Interfaces:**
- Consumes: blended `SparkUpperTargets` from `DualArmSparkGuidance`.
- Produces: a second eight-point Viewer skeleton and distinct Spark colors.

- [ ] **Step 1: Add failing conversion and dual-overlay tests**

Require Spark shoulder/elbow/wrist/hand points to map into the eight protocol
indices, and require appending PICO plus Spark skeletons to create 30 decorative
geometries with different colors.

- [ ] **Step 2: Confirm RED**

Build `test_pico_skeleton_overlay`; expect missing Spark conversion/style API.

- [ ] **Step 3: Implement Spark conversion and style-aware rendering**

Keep the existing PICO colors as the default. Render scaled Spark left bones in
orange, right bones in green, and the shoulder bridge in yellow.

- [ ] **Step 4: Publish the blended Spark skeleton in Viewer snapshots**

Set it valid only while a Spark mode is active, the PICO stream is live, and
the blended Spark target is valid. Render it next to the corrected PICO
skeleton whenever `--pico-skeleton-overlay` is enabled.

- [ ] **Step 5: Confirm GREEN and include in full regression**

Run `test_pico_skeleton_overlay` and all three Spark Viewer integration tests.
