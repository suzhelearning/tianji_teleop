# PICO MuJoCo Joint Reference/Feedback Live Plot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a non-blocking MuJoCo Viewer panel that plots the active controller's planned and simulated `q`, `qdot`, `qddot`, and jerk for all seven joints of either arm together with effective bounds.

**Architecture:** The 200 Hz control thread constructs complete bilateral samples and sends them through a dedicated bounded SPSC queue. Pure sampling/history code owns fixed-step derivative validity; the Viewer thread owns circular history and four-line MuJoCo figures. Controller behavior remains unchanged, with only read-only effective-bound diagnostics added where required.

**Tech Stack:** C++17, Eigen, MuJoCo `mjvFigure`, GLFW, GoogleTest, CMake/CTest.

## Global Constraints

- Work in `/home/zj/current_robotics/TJ_arm/TJ_arm_control` on `feature/pico-mujoco-teleop-v1`.
- Preserve the untracked `benchmark_results/` directory and all existing branches/worktrees.
- Do not change PICO mapping, Cartesian OTG, arm-angle control, QP formulation, solver behavior, gains, limits, reference integration, or MuJoCo commands.
- Use the fixed 200 Hz control period (`dt = 0.005 s`) for planned and simulated derivatives.
- Acceleration-level `qddot_ref` must be the accepted controller/QP acceleration state; velocity-level `qddot_ref` is the fixed-step derivative of accepted `qdot_ref`.
- Plot reference, MuJoCo actual, lower bound, and upper bound as four distinct curves.
- Queue overflow may only increment a visualization-drop counter.

---

### Task 1: Joint kinematics sampling and circular history

**Files:**
- Create: `include/tianji_qp_ik/joint_kinematics_plot.hpp`
- Create: `src/joint_kinematics_plot.cpp`
- Create: `tests/test_joint_kinematics_plot.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `ReferenceAccelerationSource`, `JointKinematicsDifferentiator::update(...)`, `JointKinematicsHistory::push(...)`, `JointKinematicsHistory::series(...)`, `plotMetricName(...)`, `plotMetricUnit(...)`, and `nextPlotMetric(...)`.
- Consumes: `Vec7`, `ArmSide`, direct acceleration state, fixed `dt`, and reset flags.

- [ ] **Step 1: Write failing derivative and history tests**

Specify velocity-derived and direct-QP acceleration, independent reference/actual validity, reset warm-up, source transitions, and circular ordering:

```cpp
JointKinematicsDifferentiator differentiator;
const Vec7 zero = Vec7::Zero();

const auto first = differentiator.update(
    zero, zero, ReferenceAccelerationSource::kDifferentiateVelocity,
    zero, 0.005, false);
EXPECT_FALSE(first.reference_acceleration_valid);
EXPECT_FALSE(first.actual_acceleration_valid);

const auto second = differentiator.update(
    Vec7::Constant(0.1), zero,
    ReferenceAccelerationSource::kDifferentiateVelocity,
    Vec7::Constant(0.2), 0.005, false);
EXPECT_TRUE(second.reference_acceleration.isApprox(Vec7::Constant(20.0)));
EXPECT_TRUE(second.actual_acceleration.isApprox(Vec7::Constant(40.0)));

const auto direct = differentiator.update(
    Vec7::Constant(0.1), Vec7::Constant(7.0),
    ReferenceAccelerationSource::kDirectQpOutput,
    Vec7::Constant(0.2), 0.005, false);
EXPECT_TRUE(direct.reference_acceleration.isApprox(Vec7::Constant(7.0)));
EXPECT_FALSE(direct.reference_jerk_valid);  // source change resets warm-up
```

Add a history test in which reference acceleration is valid while actual acceleration is invalid and assert the two validity vectors remain independent.

- [ ] **Step 2: Run the new target and witness RED**

Run:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --target test_joint_kinematics_plot -j"$(nproc)"
```

Expected: compilation/configuration fails because the new source, header, and test target do not exist.

- [ ] **Step 3: Implement the minimal sampling API**

Define fixed-size sample state and separate validity:

```cpp
enum class PlotMetric { kPosition, kVelocity, kAcceleration, kJerk };
enum class ReferenceAccelerationSource {
  kDifferentiateVelocity,
  kDirectQpOutput,
};

struct ArmJointKinematicsSample {
  JointKinematicsState reference;
  JointKinematicsState actual;
  JointKinematicsBounds bounds;
  bool reference_acceleration_valid{false};
  bool reference_jerk_valid{false};
  bool actual_acceleration_valid{false};
  bool actual_jerk_valid{false};
};

JointKinematicsDerivatives update(
    const Vec7& reference_velocity,
    const Vec7& direct_reference_acceleration,
    ReferenceAccelerationSource source,
    const Vec7& actual_velocity,
    double fixed_dt,
    bool reset) noexcept;
```

Reset on non-finite input, invalid `dt`, explicit reset, or acceleration-source change. For direct-QP mode, mark reference acceleration valid immediately but warm up jerk from the next direct sample. Always differentiate actual velocity independently. Store `reference_valid` and `actual_valid` arrays separately in `JointPlotSeries`.

- [ ] **Step 4: Register, build, and run GREEN tests**

Add `src/joint_kinematics_plot.cpp` to `tianji_qp_ik_core`, register `test_joint_kinematics_plot`, then run:

```bash
cmake --build build --target test_joint_kinematics_plot -j"$(nproc)"
ctest --test-dir build -R '^test_joint_kinematics_plot$' --output-on-failure
```

Expected: one test target passes with no failures.

- [ ] **Step 5: Commit Task 1**

```bash
git add CMakeLists.txt include/tianji_qp_ik/joint_kinematics_plot.hpp \
  src/joint_kinematics_plot.cpp tests/test_joint_kinematics_plot.cpp
git commit -m "feat: add joint reference feedback history"
```

### Task 2: Four-curve MuJoCo joint plot panel

**Files:**
- Create: `include/tianji_qp_ik/mujoco_joint_plot.hpp`
- Create: `src/mujoco_joint_plot.cpp`
- Create: `tests/test_mujoco_joint_plot.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `JointKinematicsHistory`, `ArmSide`, `PlotMetric`, and a five-second display window.
- Produces: `JointPlotLayout`, `computeJointPlotLayout(...)`, `pointInJointPlotPanel(...)`, and `MujocoJointPlot`.

- [ ] **Step 1: Write failing layout and rendering tests**

Require a 2-by-4 right panel and four lines per joint:

```cpp
MujocoJointPlot plot;
plot.update(history, ArmSide::kLeft, PlotMetric::kVelocity, 5.0);
for (int joint = 0; joint < kArmDof; ++joint) {
  const mjvFigure& figure = plot.figure(joint);
  EXPECT_EQ(figure.linepnt[0], 4);  // QP reference
  EXPECT_EQ(figure.linepnt[1], 4);  // MuJoCo actual
  EXPECT_EQ(figure.linepnt[2], 4);  // lower
  EXPECT_EQ(figure.linepnt[3], 4);  // upper
  EXPECT_STREQ(figure.linename[0], "QP reference");
  EXPECT_STREQ(figure.linename[1], "MuJoCo actual");
}
```

Also assert cyan reference, blue actual, red lower/upper, small-window hiding, and plot-panel hit testing.

- [ ] **Step 2: Run the target and witness RED**

Run:

```bash
cmake --build build --target test_mujoco_joint_plot -j"$(nproc)"
```

Expected: failure because `mujoco_joint_plot.hpp` and its target are absent.

- [ ] **Step 3: Implement the layout and renderer**

Use four `mjvFigure` lines:

```cpp
setColor(figure.linergb[0], 0.20F, 0.90F, 0.82F);  // reference
setColor(figure.linergb[1], 0.35F, 0.65F, 1.00F);  // actual
setColor(figure.linergb[2], 1.00F, 0.38F, 0.48F);  // lower
setColor(figure.linergb[3], 1.00F, 0.38F, 0.48F);  // upper
```

Append reference and actual only when their respective validity flags are true; append finite bounds independently. Keep all rendering in the Viewer thread.

- [ ] **Step 4: Register and run both plot modules**

```bash
cmake --build build --target test_joint_kinematics_plot test_mujoco_joint_plot -j"$(nproc)"
ctest --test-dir build \
  -R 'test_(joint_kinematics_plot|mujoco_joint_plot)' \
  --output-on-failure
```

Expected: both targets pass.

- [ ] **Step 5: Commit Task 2**

```bash
git add CMakeLists.txt include/tianji_qp_ik/mujoco_joint_plot.hpp \
  src/mujoco_joint_plot.cpp tests/test_mujoco_joint_plot.cpp
git commit -m "feat: render live MuJoCo joint plots"
```

### Task 3: Expose effective velocity bounds as diagnostics

**Files:**
- Modify: `include/tianji_qp_ik/controller.hpp`
- Modify: `src/controller.cpp`
- Modify: `tests/test_controller.cpp`

**Interfaces:**
- Produces: `ArmControllerDiagnostics::bounds` as a read-only copy of the `JointVelocityBounds` already used by the active velocity IK step.
- Consumes: each arm's `ArmIkInput::bounds`.

- [ ] **Step 1: Write a failing controller diagnostic test**

After one accepted controller step, assert:

```cpp
EXPECT_TRUE(diagnostics.left.bounds.lower.allFinite());
EXPECT_TRUE(diagnostics.left.bounds.upper.allFinite());
EXPECT_TRUE((diagnostics.left.bounds.lower.array() <=
             diagnostics.left.bounds.upper.array()).all());
```

- [ ] **Step 2: Run and witness RED**

```bash
cmake --build build --target test_controller -j"$(nproc)"
```

Expected: compilation fails because `ArmControllerDiagnostics::bounds` is absent.

- [ ] **Step 3: Copy already-computed bounds into diagnostics**

Add:

```cpp
JointVelocityBounds bounds;
```

to `ArmControllerDiagnostics`, then assign `diagnostics.left.bounds = left_input.bounds` and `diagnostics.right.bounds = right_input.bounds` after each input is constructed. Do not use the copied diagnostics in any controller decision.

- [ ] **Step 4: Run controller regression tests**

```bash
cmake --build build --target test_controller -j"$(nproc)"
ctest --test-dir build -R '^test_controller$' --output-on-failure
```

Expected: controller tests pass and reference commands remain unchanged.

- [ ] **Step 5: Commit Task 3**

```bash
git add include/tianji_qp_ik/controller.hpp src/controller.cpp \
  tests/test_controller.cpp
git commit -m "feat: expose effective velocity bounds"
```

### Task 4: Viewer sampling, queue, controls, telemetry, and integration

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `tests/test_snapshot_exchange.cpp`
- Modify: `tests/test_pico_viewer_integration.py`
- Create: `tests/assert_viewer_joint_plot.cmake`
- Modify: `CMakeLists.txt`
- Modify: `README.md`

**Interfaces:**
- Consumes: controller `referenceState(...)`, velocity/acceleration effective bounds, MuJoCo arm state, fixed control `dt`, and `BoundedSpscQueue<JointKinematicsSample>`.
- Produces: live Viewer figures, `F2`/`F3`/`F4`/`F5` controls, plot counters in `ViewerSnapshot`, and headless plot diagnostics.

- [ ] **Step 1: Extend tests first**

Add snapshot assertions:

```cpp
snapshot.joint_plot_samples = 123U;
snapshot.joint_plot_drops = 4U;
snapshot.joint_plot_derivatives_valid = true;
```

Add headless integration requirements:

```python
assert int(summary["joint_plot_drained"]) > 100
assert summary["joint_plot_both_arms_finite"] == "1"
assert summary["joint_plot_reference_jerk_valid_seen"] == "1"
assert summary["joint_plot_actual_jerk_valid_seen"] == "1"
```

Add a source-level Viewer test that requires `F2`, `F3`, `F4`, and `F5` bindings and four plot labels.

- [ ] **Step 2: Run focused tests and witness RED**

```bash
cmake --build build --target test_snapshot_exchange -j"$(nproc)"
ctest --test-dir build --output-on-failure \
  -R 'test_snapshot_exchange|pico_viewer_integration_(velocity|acceleration)'
```

Expected: compile/assertion failures because the queue and summary fields are not integrated.

- [ ] **Step 3: Add Viewer application state and the bounded queue**

Add:

```cpp
JointKinematicsHistory joint_plot_history{2001U};
MujocoJointPlot joint_plot;
PlotMetric joint_plot_metric{PlotMetric::kPosition};
ArmSide joint_plot_arm{ArmSide::kLeft};
bool show_joint_plots{true};
bool joint_plot_arm_locked{false};
```

Create `BoundedSpscQueue<JointKinematicsSample> joint_plot_queue(4096U)` beside the existing command/snapshot/telemetry exchanges and pass it by reference to control, headless, and Viewer paths.

- [ ] **Step 4: Construct samples from accepted controller state**

After each control step, read `referenceState(side)` from the active controller. Select:

```cpp
const auto source = config.control_level == ControlLevel::kAcceleration
    ? ReferenceAccelerationSource::kDirectQpOutput
    : ReferenceAccelerationSource::kDifferentiateVelocity;
```

Populate reference state from the active controller, actual state from `MujocoRobot`, and bounds from robot limits plus the active controller diagnostics. Mark reset on pause/resume, nominal/reference synchronization, control-level switch, rejected/frozen result, non-finite data, or invalid `dt`. A failed `tryPush` only increments `joint_plot_drops`.

- [ ] **Step 5: Drain, render, and add keyboard controls**

Drain all available samples before each render. Compute the split scene/panel layout, render the seven figures, and prevent marker/camera mouse handling when the pointer is inside the plot panel.

Bind:

```text
F2 show/hide
F3 q/dq/ddq/jerk
F4 left/right + lock
F5 follow selected arm
```

Render a status cell with selected arm, metric/unit, 200 Hz, history occupancy, produced/dropped counts, and derivative validity.

- [ ] **Step 6: Extend snapshots, headless output, and README**

Add to `ViewerSnapshot`:

```cpp
std::uint64_t joint_plot_samples{0U};
std::uint64_t joint_plot_drops{0U};
bool joint_plot_reference_derivatives_valid{false};
bool joint_plot_actual_derivatives_valid{false};
```

Headless PICO output must report drained samples, bilateral finite state, and whether reference/actual jerk validity was observed. Document the colors and F-key controls in `README.md`. Keep the existing telemetry CSV schema unchanged in this feature; live plot samples use their dedicated queue rather than the CSV writer.

- [ ] **Step 7: Build and run focused GREEN tests**

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure \
  -R 'test_(joint_kinematics_plot|mujoco_joint_plot|snapshot_exchange|controller)|pico_viewer_integration_(velocity|acceleration)'
```

Expected: all focused tests pass.

- [ ] **Step 8: Commit Task 4**

```bash
git add CMakeLists.txt README.md apps/run_qp_ik_viewer.cpp \
  include/tianji_qp_ik/telemetry.hpp tests/test_snapshot_exchange.cpp \
  tests/test_pico_viewer_integration.py tests/assert_viewer_joint_plot.cmake
git commit -m "feat: integrate live joint plots into PICO Viewer"
```

### Task 5: Full verification and handoff

**Files:**
- Verify only; modify tests or implementation only if verification exposes an in-scope defect.

**Interfaces:**
- Consumes: completed live-plot feature.
- Produces: reproducible build/test evidence and the runtime test command.

- [ ] **Step 1: Run formatting/diff checks and full build**

```bash
git diff --check
cmake --build build -j"$(nproc)"
```

Expected: both commands exit zero.

- [ ] **Step 2: Run the complete test suite**

```bash
ctest --test-dir build --output-on-failure
```

Expected: all deterministic tests pass. If the pre-existing `test_viewer_explicit_profile` state-transfer timing failure recurs, record it, rerun that test alone, and do not misreport the one-shot full-suite result.

- [ ] **Step 3: Run velocity and acceleration headless smoke tests**

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_pico_fast.xml \
  --pico-teleop --pico-bind 127.0.0.1 --pico-port 15000 \
  --control-level velocity --headless --duration 2

./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_pico_fast.xml \
  --pico-teleop --pico-bind 127.0.0.1 --pico-port 15000 \
  --control-level acceleration --headless --duration 2
```

Expected: healthy exit, positive drained sample count, finite bilateral state, and reference/actual jerk observed after warm-up.

- [ ] **Step 4: Verify repository scope**

```bash
git status --short --branch
git log -6 --oneline --decorate
```

Expected: only `benchmark_results/` remains untracked; no benchmark data is staged or committed.
