# SPARK Feedforward Velocity QP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an isolated `spark_upper_qpoases_feedforward_velocity_qp` mode that derives a low-latency jerk-limited joint reference, Cartesian pose, and Cartesian twist feedforward from the same SPARK two-stage qpOASES IK sequence.

**Architecture:** A focused `SparkFeedforwardReference7` module owns source-timestamp validation, joint unwrapping, alpha-beta velocity estimation, and 200 Hz jerk-limited reference shaping. `DualArmSparkGuidance` uses it only for the new posture mode, evaluates Pinocchio FK/Jacobian at the shaped state, and supplies the existing Velocity-Level QP with a phase-consistent Cartesian target and joint velocity soft reference.

**Tech Stack:** C++20, Eigen, Pinocchio, yaml-cpp, MuJoCo, qpOASES, GoogleTest, CMake/CTest, Python 3 offline analysis.

## Global Constraints

- Do not change the behavior or configuration semantics of any existing algorithm.
- Keep Velocity-Level QP decision variables, objectives, and hard constraints unchanged.
- Use model reference state for Cartesian feedback.
- Do not run Cartesian OTG or Joint Ruckig during normal Active motion.
- New mode name is exactly `spark_upper_qpoases_feedforward_velocity_qp`.
- Reference acceleration defaults to `[60, 60, 60, 90, 90, 90, 90] rad/s^2`.
- Reference jerk defaults to `[1000, 1000, 1000, 1500, 1500, 1500, 1500] rad/s^3`.
- Preserve untracked `benchmark_results/` content.
- Implement test-first and commit each independently testable task.

## Execution Order

Execute Task 2 before Task 1 because the focused reference module consumes
`SparkFeedforwardVelocityQpConfig`. The dependency-safe order is
`2 -> 1 -> 3 -> 4 -> 5 -> 6`; task numbering remains aligned with the design
sections below.

---

### Task 1: Timestamp-aware jerk-limited reference module

**Files:**
- Create: `include/tianji_qp_ik/spark_feedforward_reference.hpp`
- Create: `src/spark_feedforward_reference.cpp`
- Create: `tests/test_spark_feedforward_reference.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `SparkFeedforwardVelocityQpConfig`, `ArmLimits`, accepted `q_ik`, source sequence/timestamp, model state, and control `dt`.
- Produces: `SparkFeedforwardReferenceResult SparkFeedforwardReference7::update(...)`, `stop(...)`, `reset(...)` with `q`, `qdot`, `qddot`, `jerk`, source timing diagnostics, state, and acceptance detail.

- [ ] **Step 1: Write failing reference tests**

Add tests covering initialization, duplicate timestamps, 30 ms dropped-frame rejection against an 11 ms median, `+pi/-pi` unwrapping, `0.35 rad` jump rejection, acceleration/jerk bounds, Active-to-Stopping decay, and epoch reset. Use the public API:

```cpp
SparkFeedforwardReference7 reference(config, limits, 0.005);
reference.reset(model_state, 85U);
auto first = reference.acceptTarget(q0, 1U, 1'000'000'000LL, 85U);
auto result = reference.step(model_state, 0.005, true);
EXPECT_TRUE(first.accepted);
EXPECT_TRUE(result.valid);
EXPECT_LE(result.qddot.cwiseAbs().maxCoeff(), 90.0 + 1.0e-9);
EXPECT_LE(result.jerk.cwiseAbs().maxCoeff(), 1500.0 + 1.0e-9);
```

- [ ] **Step 2: Run RED**

Run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)" --target test_spark_feedforward_reference
```

Expected: build fails because `spark_feedforward_reference.hpp` and target do not exist.

- [ ] **Step 3: Implement the focused module**

Define:

```cpp
enum class SparkFeedforwardState { kHold, kActive, kStopping };

struct SparkFeedforwardTargetDecision {
  bool accepted{false};
  bool dt_valid{false};
  bool jump_rejected{false};
  double source_dt_seconds{0.0};
  double median_dt_seconds{0.0};
  std::string_view detail{"not_updated"};
};

struct SparkFeedforwardReferenceResult {
  bool valid{false};
  Vec7 q{Vec7::Zero()};
  Vec7 qdot{Vec7::Zero()};
  Vec7 qddot{Vec7::Zero()};
  Vec7 jerk{Vec7::Zero()};
  double activation{0.0};
  SparkFeedforwardState state{SparkFeedforwardState::kHold};
  std::string_view detail{"not_updated"};
};
```

Store the last accepted sequence/timestamp, 31-value valid-dt ring, unwrapped target, alpha-beta state, and shaped state. Apply shortest-angle differences, reject source `dt` outside configured median ratios once at least three valid periods exist, and clamp `qddot` first by jerk then acceleration and `qdot` by arm velocity.

- [ ] **Step 4: Run GREEN**

Run:

```bash
cmake --build build -j"$(nproc)" --target test_spark_feedforward_reference
ctest --test-dir build --output-on-failure -R '^test_spark_feedforward_reference$'
```

Expected: one test target passes with all reference cases green.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt include/tianji_qp_ik/spark_feedforward_reference.hpp \
  src/spark_feedforward_reference.cpp tests/test_spark_feedforward_reference.cpp
git commit -m "feat: add SPARK feedforward reference shaper"
```

### Task 2: Independent configuration and algorithm identity

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Modify: `tests/test_config.cpp`

**Interfaces:**
- Produces: `SparkFeedforwardVelocityQpConfig QpIkConfig::spark_feedforward_velocity_qp` and `IkAlgorithm::kSparkUpperQpoasesFeedforwardVelocityQp`.
- Preserves: existing enum values and existing `toString` outputs.

- [ ] **Step 1: Write failing config tests**

Parse a temporary YAML with:

```yaml
ik:
  algorithm: spark_upper_qpoases_feedforward_velocity_qp
spark_feedforward_velocity_qp:
  alpha: 0.65
  beta: 0.12
  dt_median_window: 31
  dt_min_ratio: 0.5
  dt_max_ratio: 1.5
  maximum_joint_jump_rad: 0.35
  stale_velocity_decay_seconds: 0.02
  reference_velocity_scale: 1.0
  reference_acceleration_scale: 1.0
  reference_jerk_scale: 1.0
  position_feedforward_gain: 1.0
  orientation_feedforward_gain: 1.0
  joint_position_gain: 4.0
  attack_seconds: 0.08
  release_seconds: 0.05
```

Assert the new enum/string round trip, helper predicates include it in SPARK upper qpOASES and hierarchical Velocity QP routes, and invalid alpha, beta, ratios, even median window, or non-positive times throw.

- [ ] **Step 2: Run RED**

```bash
cmake --build build -j"$(nproc)" --target test_config
ctest --test-dir build --output-on-failure -R '^test_config$'
```

Expected: compilation fails because the enum and config struct are absent.

- [ ] **Step 3: Implement config parsing and validation**

Append the new enum without reordering existing values. Add the config struct with the exact defaults from the design, parse all keys under `spark_feedforward_velocity_qp`, require `0 < alpha <= 1`, `0 < beta <= 1`, odd window at least 3, `0 < dt_min_ratio < 1 < dt_max_ratio`, and positive limits/times/gains.

- [ ] **Step 4: Run GREEN and regression config tests**

```bash
cmake --build build -j"$(nproc)" --target test_config
ctest --test-dir build --output-on-failure -R '^test_config$'
```

Expected: config tests pass, including historical algorithm cases.

- [ ] **Step 5: Commit**

```bash
git add include/tianji_qp_ik/config.hpp src/config.cpp \
  config/qp_ik_pico_teleop.yaml tests/test_config.cpp
git commit -m "feat: configure SPARK feedforward velocity QP"
```

### Task 3: Phase-consistent SPARK guidance

**Files:**
- Modify: `include/tianji_qp_ik/spark_guidance.hpp`
- Modify: `src/spark_guidance.cpp`
- Modify: `tests/test_spark_guidance.cpp`

**Interfaces:**
- Adds: `SparkPostureGuideMode::kFeedforwardJointReferenceVelocity`.
- Consumes: source sequence/timestamp/epoch saved by `updatePicoFrame`, accepted two-stage qpOASES `q_ik`, and model states.
- Produces: `cartesian_targets` from `FK(q_ref)`, target twists from `J(q_ref)qdot_ref`, and posture targets `qdot_ref + Kq(q_ref-q_model)`.

- [ ] **Step 1: Write failing guidance tests**

Extend the deterministic synthetic skeleton fixture. Select the new mode, feed two source frames 11 ms apart, and assert:

```cpp
EXPECT_TRUE(result.accepted);
EXPECT_TRUE(result.feedforward_references_valid);
EXPECT_GT(result.cartesian_targets.left_twist.norm(), 0.0);
EXPECT_NEAR((result.cartesian_targets.left.position -
             kinematics.sample(ArmSide::kLeft,
                               result.left.feedforward.q).tcp_pose.position)
                .norm(), 0.0, 1.0e-9);
EXPECT_NEAR((result.posture_tasks.left.target -
             (result.left.feedforward.qdot + 4.0 *
              (result.left.feedforward.q - left_model.q)))
                .norm(), 0.0, 1.0e-9);
```

Also instantiate the historical joint-reference mode and prove its Cartesian target remains the raw SPARK palm and its twist remains zero.

- [ ] **Step 2: Run RED**

```bash
cmake --build build -j"$(nproc)" --target test_spark_guidance
ctest --test-dir build --output-on-failure -R '^test_spark_guidance$'
```

Expected: compilation fails on the new mode and diagnostics.

- [ ] **Step 3: Integrate the reference module**

Give each `ArmState` one `SparkFeedforwardReference7`. Cache source metadata in `updatePicoFrame`. After both IK solutions are accepted, call `acceptTarget` only once per new sequence and `step` every control cycle. Use `PinocchioArmKinematics::sample` at the shaped `q`; assign sample pose and `jacobian * qdot` to the Cartesian target, and construct the strong joint task with the new independent attack/release activation.

On stale input call `stop`, retain the last shaped pose, and release only after shaped velocity reaches zero. On epoch change call `reset(model_state, epoch)` before accepting the first target.

- [ ] **Step 4: Run GREEN plus historical guidance tests**

```bash
cmake --build build -j"$(nproc)" --target test_spark_guidance
ctest --test-dir build --output-on-failure -R 'test_spark_(guidance|upper_qpoases_ik)'
```

Expected: new and historical guidance tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/tianji_qp_ik/spark_guidance.hpp src/spark_guidance.cpp \
  tests/test_spark_guidance.cpp
git commit -m "feat: generate phase-consistent SPARK feedforward guidance"
```

### Task 4: Viewer registration and integration telemetry

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `src/telemetry.cpp`
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- CLI accepts `--algorithm spark_upper_qpoases_feedforward_velocity_qp`.
- Viewer maps only that enum to `kFeedforwardJointReferenceVelocity`.
- CSV appends feedforward state, source timing, `q/qdot/qddot/jerk`, rejection counters, activation, and linear/angular feedforward magnitudes.

- [ ] **Step 1: Write failing CLI/integration test**

Add a CTest invocation using velocity control, model-state-only, the PICO config and synthetic UDP input. Require summary algorithm equality, at least one non-zero feedforward twist sample, finite reference derivatives, stale transition to stopping/hold, zero control failures, and zero deadline misses.

- [ ] **Step 2: Run RED**

```bash
cmake --build build -j"$(nproc)" --target tianji_qp_ik_viewer
ctest --test-dir build --output-on-failure \
  -R 'pico_viewer_integration_spark_upper_qpoases_feedforward_velocity_qp'
```

Expected: Viewer rejects the unsupported algorithm.

- [ ] **Step 3: Register the mode and append diagnostics**

Append the CLI parser/toString route, choose the new guidance mode, pass its Cartesian pose/twist and posture task into the unchanged controller interface, and append—not rename—CSV columns. Ensure no diagnostics branch is executed for historical modes.

- [ ] **Step 4: Run GREEN and all PICO integrations**

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -R 'pico_viewer_integration' -j1
```

Expected: all existing and new PICO integration tests pass.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt apps/run_qp_ik_viewer.cpp \
  include/tianji_qp_ik/telemetry.hpp src/telemetry.cpp \
  tests/test_pico_viewer_integration.py
git commit -m "feat: expose SPARK feedforward velocity QP mode"
```

### Task 5: Deterministic four-way benchmark and plots

**Files:**
- Modify: `scripts/compare_spark_three_way_following.py`
- Modify: `tests/test_compare_spark_three_way_following.py`
- Create: `docs/verification/spark_feedforward_velocity_qp_results.md`
- Generate untracked: `benchmark_results/spark_four_way_20260817/`

**Interfaces:**
- Analyzer accepts optional `--feedforward-velocity-qp` and emits four rows when supplied.
- Reuses the raw SPARK palm intent and exclusion windows from the validated three-way comparison.

- [ ] **Step 1: Write failing analyzer test**

Create synthetic telemetry with known 0/1/2/3-frame lags and monotonic smoothing differences. Require a fourth `feedforward_velocity_qp` row, correct lag, acceleration/jerk P50/P95/P99/max, settling peak-to-peak, and unchanged three-way behavior when the optional input is omitted.

- [ ] **Step 2: Run RED**

```bash
python3 -m unittest tests/test_compare_spark_three_way_following.py -v
```

Expected: parser rejects `--feedforward-velocity-qp` or fourth row is absent.

- [ ] **Step 3: Extend analyzer without changing historical metrics**

Add the optional input, label and color. Compute derivatives from the fixed 200 Hz control timestamp for each algorithm, exclude invalid reset samples, and write the fourth curve to trajectory, time-series, CDF, lag and summary plots.

- [ ] **Step 4: Run GREEN**

```bash
python3 -m unittest tests/test_compare_spark_three_way_following.py -v
ctest --test-dir build --output-on-failure \
  -R '^compare_spark_three_way_following_analysis$'
```

Expected: analyzer tests pass in both three- and four-way modes.

- [ ] **Step 5: Replay and analyze the fixed trace**

Run the new Viewer headless for 28 seconds on port 15114 with velocity control, model-state-only and the fixed v4 trace. Save telemetry under `benchmark_results/spark_four_way_20260817/`, then run the analyzer against the three validated historical CSV files plus the new CSV.

Expected: 2134 source frames, zero control failures/deadline misses, readable four-way plots and summary.

- [ ] **Step 6: Tune only the new config if necessary**

If acceptance fails, adjust only `spark_feedforward_velocity_qp` alpha, beta, attack/release, or reference scales. Repeat Task 5 Step 5 and record every tested parameter set in the verification document; do not modify historical modes.

- [ ] **Step 7: Commit code and sourced verification report**

```bash
git add scripts/compare_spark_three_way_following.py \
  tests/test_compare_spark_three_way_following.py \
  docs/verification/spark_feedforward_velocity_qp_results.md
git commit -m "test: benchmark SPARK feedforward velocity QP"
```

### Task 6: Complete regression and MuJoCo visual A/B

**Files:**
- Modify only if verification exposes a defect in the new mode.
- Generate untracked: `benchmark_results/spark_feedforward_visual_20260817/`

**Interfaces:**
- Uses the fixed v4 trace and identical model/config/control settings for historical and new Velocity QP modes.

- [ ] **Step 1: Run complete verification**

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -j1
git diff --check
```

Expected: 100% tests pass and no whitespace errors.

- [ ] **Step 2: Run sequential MuJoCo visualization**

Run `spark_upper_qpoases_velocity_qp` and `spark_upper_qpoases_feedforward_velocity_qp` for 28 seconds each, on separate UDP ports, with `--pico-skeleton-overlay`, velocity control and model-state-only. Replay the same 2134-frame trace with a 1.5-second lead.

Expected: both windows complete, new mode has no branch flip, and its start/stop motion is visually no rougher than the historical mode.

- [ ] **Step 3: Verify repository state**

```bash
git status --short
git log --oneline -8
```

Expected: only `benchmark_results/` remains untracked; all source, tests, config and verification documentation are committed.
