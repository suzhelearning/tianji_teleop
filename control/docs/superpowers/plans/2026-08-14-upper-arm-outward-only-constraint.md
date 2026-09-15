# Upper-Arm Outward-Only Constraint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an `outward_only` redundancy mode that ignores PICO arm-angle directions, preserves 6D end-effector tracking, and prevents each elbow from crossing inward of its shoulder plane.

**Architecture:** Compute a side-specific elbow lateral distance and Jacobian from existing MuJoCo shoulder/elbow geometry. Add one optional general inequality row to the existing 13-variable velocity and acceleration QPs, use first- and second-order control-barrier lower bounds, and retain free constraint bounds in legacy modes. Expose a three-way Viewer mode and compare all modes with identical recorded PICO packets.

**Tech Stack:** C++17, Eigen, MuJoCo, qpOASES, yaml-cpp, GoogleTest, Python 3 replay/analysis.

## Global Constraints

- Stay on `feature/pico-mujoco-teleop-v1` in `/home/zj/current_robotics/TJ_arm/TJ_arm_control`; do not create or switch worktrees.
- Do not use subagents.
- Preserve untracked `benchmark_results/`; never stage or delete it.
- Do not change PICO endpoint mapping, Position OTG, Cartesian gains, joint position/velocity/acceleration/jerk/braking limits, collision behavior, or QP decision variables.
- Left outward is world `+Y`; right outward is world `-Y`.
- Only lateral inward motion is constrained; elbow height and forward/backward motion remain free.
- Use test-first RED-GREEN cycles and commit each completed task independently.

---

### Task 1: Outward geometry, barrier bounds, and configuration

**Files:**
- Create: `include/tianji_qp_ik/upper_arm_outward.hpp`
- Create: `src/upper_arm_outward.cpp`
- Modify: `include/tianji_qp_ik/types.hpp`
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/test_upper_arm_outward.cpp`
- Test: `tests/test_config.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`

**Interfaces:**
- Consumes: `ArmSide`, shoulder/elbow positions and `Mat37` Jacobians, joint box bounds, `qdot`, and scalar `Jdot_h_qdot`.
- Produces: `LinearJointConstraint`, `UpperArmOutwardState`, `UpperArmOutwardDiagnostics`, `computeUpperArmOutwardState`, `makeVelocityOutwardConstraint`, and `makeAccelerationOutwardConstraint`.

- [ ] **Step 1: Write failing sign and barrier tests**

Add `tests/test_upper_arm_outward.cpp` with cases equivalent to:

```cpp
TEST(UpperArmOutward, UsesLeftPositiveYAndRightNegativeY) {
  const auto left = computeUpperArmOutwardState(
      ArmSide::kLeft, shoulder, shoulder + Eigen::Vector3d(0, 0.1, -0.2),
      J_shoulder, J_elbow, 0.0);
  const auto right = computeUpperArmOutwardState(
      ArmSide::kRight, shoulder, shoulder + Eigen::Vector3d(0, -0.1, -0.2),
      J_shoulder, J_elbow, 0.0);
  EXPECT_NEAR(left.distance_m, 0.1, 1e-12);
  EXPECT_NEAR(right.distance_m, 0.1, 1e-12);
}

TEST(UpperArmOutward, VelocityBarrierStopsInwardMotionAtPlane) {
  state.distance_m = 0.0;
  state.jacobian[2] = 0.5;
  const auto constraint = makeVelocityOutwardConstraint(state, bounds, config);
  EXPECT_TRUE(constraint.active);
  EXPECT_DOUBLE_EQ(constraint.requested_lower, 0.0);
  EXPECT_DOUBLE_EQ(constraint.lower, 0.0);
}

TEST(UpperArmOutward, AccelerationBarrierIncludesBiasAndDamping) {
  state.distance_m = 0.02;
  state.jacobian[0] = 1.0;
  qdot[0] = -0.3;
  const auto constraint = makeAccelerationOutwardConstraint(
      state, qdot, 0.4, bounds, config);
  EXPECT_NEAR(constraint.requested_lower,
              -0.4 - 20.0 * -0.3 - 100.0 * 0.02, 1e-12);
}
```

Also add tests for hanging/down state, inward negative distance, non-finite geometry, and clipping a requested lower bound to the maximum achievable value under box bounds.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
cmake --build build -j"$(nproc)" --target test_config
cmake --build build -j"$(nproc)" --target test_upper_arm_outward
```

Expected: target or compilation fails because the new header, types, and CMake target do not exist.

- [ ] **Step 3: Add minimal types and geometry implementation**

Add to `types.hpp`:

```cpp
struct LinearJointConstraint {
  bool active{false};
  Vec7 jacobian{Vec7::Zero()};
  double lower{-std::numeric_limits<double>::infinity()};
  double upper{std::numeric_limits<double>::infinity()};
  double requested_lower{-std::numeric_limits<double>::infinity()};
  bool feasibility_clipped{false};
};

struct UpperArmOutwardState {
  bool valid{false};
  double distance_m{0.0};
  Vec7 jacobian{Vec7::Zero()};
};

struct UpperArmOutwardDiagnostics {
  UpperArmOutwardState state;
  bool constraint_active{false};
  double requested_lower{0.0};
  double effective_lower{0.0};
  double achieved{0.0};
  double residual{0.0};
  bool feasibility_clipped{false};
};
```

Define `UpperArmOutwardConfig` defaults exactly as the design and parse the
`upper_arm_outward` YAML section. Validate finite/non-negative minimum distance,
positive velocity gain, and non-negative acceleration gains.

Implement:

```cpp
outward = side == ArmSide::kLeft ? UnitY() : -UnitY();
distance = outward.dot(elbow - shoulder) - minimum_distance;
jacobian = (elbow_jacobian - shoulder_jacobian).transpose() * outward;
```

Build each barrier lower bound and clip only against:

```text
maximum = sum_i jacobian_i * (jacobian_i >= 0 ? upper_i : lower_i)
effective_lower = min(requested_lower, maximum)
```

- [ ] **Step 4: Run focused tests and verify GREEN**

```bash
cmake --build build -j"$(nproc)" --target test_upper_arm_outward test_config
./build/test_upper_arm_outward
./build/test_config --gtest_filter='Config.*'
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit geometry and configuration**

```bash
git add CMakeLists.txt config/qp_ik_pico_teleop.yaml \
  include/tianji_qp_ik/types.hpp include/tianji_qp_ik/config.hpp \
  include/tianji_qp_ik/upper_arm_outward.hpp src/config.cpp \
  src/upper_arm_outward.cpp tests/test_upper_arm_outward.cpp tests/test_config.cpp
git diff --cached --check
git commit -m "feat: define outward-only upper-arm barrier"
```

---

### Task 2: Add the optional seventh qpOASES constraint row

**Files:**
- Modify: `include/tianji_qp_ik/hierarchical_qp.hpp`
- Modify: `include/tianji_qp_ik/acceleration_qp.hpp`
- Modify: `include/tianji_qp_ik/velocity_ik.hpp`
- Modify: `include/tianji_qp_ik/acceleration_ik.hpp`
- Modify: `include/tianji_qp_ik/hierarchical_qpoases_solver.hpp`
- Modify: `src/hierarchical_qp.cpp`
- Modify: `src/acceleration_qp.cpp`
- Modify: `src/hierarchical_qpoases_solver.cpp`
- Modify: `src/acceleration_qpoases_solver.cpp`
- Test: `tests/test_hierarchical_qp.cpp`
- Test: `tests/test_hierarchical_qpoases_solver.cpp`
- Test: `tests/test_acceleration_qp.cpp`
- Test: `tests/test_acceleration_qpoases_solver.cpp`

**Interfaces:**
- Consumes: `ArmIkInput::upper_arm_constraint` and `ArmAccelerationInput::upper_arm_constraint`.
- Produces: seven-row `A`, `constraint_lower`, and `constraint_upper` passed unchanged through both QP solvers.

- [ ] **Step 1: Write failing builder and solver tests**

Add active constraints with `jacobian[0] = 1`, `lower = 0.25`, `upper = +inf` and assert:

```cpp
EXPECT_TRUE(problem.A.row(6).head<kArmDof>().isApprox(jacobian.transpose()));
EXPECT_DOUBLE_EQ(problem.constraint_lower[6], 0.25);
EXPECT_TRUE(std::isinf(problem.constraint_upper[6]));
```

Add an inactive-row test asserting zero coefficients and `[-inf,+inf]`. Add a
solver test whose unconstrained optimum violates row 6 and assert the returned
solution satisfies it. Retain a legacy inactive-row solution and compare it to
the pre-change expected vector within `1e-9`.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
cmake --build build -j"$(nproc)" --target \
  test_hierarchical_qp test_hierarchical_qpoases_solver \
  test_acceleration_qp test_acceleration_qpoases_solver
```

Expected: compilation fails because problems still expose six equality rows only.

- [ ] **Step 3: Generalize fixed constraint storage**

Define:

```cpp
inline constexpr int kCartesianEqualities = 6;
inline constexpr int kQpConstraints = 7;
using Mat7x13 = Eigen::Matrix<double, kQpConstraints, kHierarchicalVariables>;
```

Replace six-row solver storage with seven rows and separate constraint lower and
upper vectors. Builders fill rows 0..5 from the unchanged Cartesian equality
and row 6 from the optional linear constraint. `AccelerationQpoasesSolver::adapt`
copies all rows and bounds.

Validation must check finite active coefficients, `lower <= upper`, exact
Cartesian equality residual on rows 0..5, and satisfaction of row 6 within
`safety.bound_tolerance`.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the four targets and binaries from Step 2. Expected: all pass, including
hotstart and inactive-row compatibility tests.

- [ ] **Step 5: Commit QP plumbing**

```bash
git add include/tianji_qp_ik/{hierarchical_qp.hpp,acceleration_qp.hpp,velocity_ik.hpp,acceleration_ik.hpp,hierarchical_qpoases_solver.hpp} \
  src/{hierarchical_qp.cpp,acceleration_qp.cpp,hierarchical_qpoases_solver.cpp,acceleration_qpoases_solver.cpp} \
  tests/{test_hierarchical_qp.cpp,test_hierarchical_qpoases_solver.cpp,test_acceleration_qp.cpp,test_acceleration_qpoases_solver.cpp}
git diff --cached --check
git commit -m "feat: support unilateral QP constraint row"
```

---

### Task 3: Integrate mode selection and controller constraints

**Files:**
- Modify: `include/tianji_qp_ik/arm_angle.hpp`
- Modify: `src/arm_angle.cpp`
- Modify: `include/tianji_qp_ik/mujoco_robot.hpp`
- Modify: `src/mujoco_robot.cpp`
- Modify: `include/tianji_qp_ik/controller.hpp`
- Modify: `src/controller.cpp`
- Modify: `include/tianji_qp_ik/acceleration_controller.hpp`
- Modify: `src/acceleration_controller.cpp`
- Test: `tests/test_arm_angle.cpp`
- Test: `tests/test_jdot_qdot.cpp`
- Test: `tests/test_controller.cpp`
- Test: `tests/test_acceleration_controller.cpp`

**Interfaces:**
- Consumes: `ArmAngleReferenceMode::kOutwardOnly`, current arm kinematics, reference motion state, and Task 1 barrier helpers.
- Produces: mode-aware controller overloads and `ArmKinematicBiasSample` containing Cartesian and shoulder/elbow `Jdot*qdot` terms.

- [ ] **Step 1: Write failing mode, bias, and controller tests**

Assert mode cycling:

```text
pico -> default_down -> outward_only -> pico
```

Add a central-difference test for shoulder/elbow translational `Jdot*qdot`.
Add velocity and acceleration controller tests that pass `kOutwardOnly` and
assert:

```cpp
EXPECT_FALSE(result.left.arm_angle_task_active);
EXPECT_TRUE(captured_problem.upper_arm_constraint.active);
EXPECT_GT(result.left.upper_arm_outward.distance_m, -1e-6);
```

Also prove PICO/default-down modes leave the new constraint inactive.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
cmake --build build -j"$(nproc)" --target \
  test_arm_angle test_jdot_qdot test_controller test_acceleration_controller
```

Expected: compilation fails because the third mode and controller interfaces do not exist.

- [ ] **Step 3: Add shared second-order kinematic bias**

Add:

```cpp
struct ArmKinematicBiasSample {
  Vec6 tcp_jdot_qdot{Vec6::Zero()};
  Eigen::Vector3d shoulder_jdot_qdot{Eigen::Vector3d::Zero()};
  Eigen::Vector3d elbow_jdot_qdot{Eigen::Vector3d::Zero()};
};
```

Compute all three from one `q +/- epsilon*qdot` sample pair. Keep
`tcpJacobianDotTimesVelocityWorld` as a compatibility wrapper.

- [ ] **Step 4: Integrate outward-only into both controllers**

Add mode-aware `step` overloads while preserving existing overloads. For each
arm in `kOutwardOnly`:

```text
arm_angle_task.active = false
state = computeUpperArmOutwardState(...)
input.upper_arm_constraint = velocity or acceleration barrier(state, bounds)
```

For acceleration mode, calculate `Jdot_h_qdot` from the shared bias sample.
For `pico` and `default_down`, preserve the existing arm-angle code and leave
the outward row inactive.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run the four targets and binaries from Step 2. Expected: all pass.

- [ ] **Step 6: Commit controller integration**

```bash
git add include/tianji_qp_ik/{arm_angle.hpp,mujoco_robot.hpp,controller.hpp,acceleration_controller.hpp} \
  src/{arm_angle.cpp,mujoco_robot.cpp,controller.cpp,acceleration_controller.cpp} \
  tests/{test_arm_angle.cpp,test_jdot_qdot.cpp,test_controller.cpp,test_acceleration_controller.cpp}
git diff --cached --check
git commit -m "feat: control upper arm with outward-only mode"
```

---

### Task 4: Viewer selection, telemetry, and integration diagnostics

**Files:**
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_snapshot_exchange.cpp`
- Modify: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- Consumes: controller outward diagnostics and `--arm-angle-mode` option.
- Produces: three-way `G` cycle, deterministic CLI selection, status text, headless summary, and per-arm CSV fields.

- [ ] **Step 1: Write failing Viewer integration assertions**

Extend the Python integration test to launch with:

```text
--arm-angle-mode outward_only
```

Require summary `arm_angle_mode=outward_only`, PICO endpoint errors that change
over time, arm-angle task inactive, and finite telemetry columns:

```text
left/right_upper_arm_outward_distance_m
left/right_upper_arm_outward_constraint_active
left/right_upper_arm_outward_requested_lower
left/right_upper_arm_outward_effective_lower
left/right_upper_arm_outward_achieved
left/right_upper_arm_outward_feasibility_clipped
left/right_upper_arm_outward_residual
```

- [ ] **Step 2: Run integration test and verify RED**

```bash
cmake --build build -j"$(nproc)" --target tianji_qp_ik_viewer
python3 tests/test_pico_viewer_integration.py \
  --viewer ./build/tianji_qp_ik_viewer --source-dir . \
  --control-level acceleration
```

Expected: CLI option is unknown and telemetry columns are missing.

- [ ] **Step 3: Wire Viewer and telemetry**

Parse `pico`, `default_down`, and `outward_only`; initialize the control-loop
mode from the option; pass it to the selected controller; cycle all three on
`G`; and append fields in identical header/value order. Keep startup `pico`
when no override is supplied.

- [ ] **Step 4: Run integration tests and verify GREEN**

```bash
cmake --build build -j"$(nproc)" --target \
  test_snapshot_exchange tianji_qp_ik_viewer
./build/test_snapshot_exchange
python3 tests/test_pico_viewer_integration.py \
  --viewer ./build/tianji_qp_ik_viewer --source-dir . \
  --control-level velocity
python3 tests/test_pico_viewer_integration.py \
  --viewer ./build/tianji_qp_ik_viewer --source-dir . \
  --control-level acceleration
```

Expected: all pass.

- [ ] **Step 5: Commit Viewer integration**

```bash
git add include/tianji_qp_ik/telemetry.hpp apps/run_qp_ik_viewer.cpp \
  tests/test_snapshot_exchange.cpp tests/test_pico_viewer_integration.py
git diff --cached --check
git commit -m "feat: expose outward-only teleoperation mode"
```

---

### Task 5: Identical-packet comparison and full verification

**Files:**
- Create: `scripts/compare_pico_arm_redundancy_modes.py`
- Create: `docs/verification/pico_outward_only_comparison.md`
- Test: `tests/test_compare_pico_arm_redundancy_modes.py`

**Interfaces:**
- Consumes: three telemetry CSV files aligned by `pico_sequence`.
- Produces: deterministic JSON/Markdown-compatible metrics for Cartesian tracking, outward distance, QP safety, and timing.

- [ ] **Step 1: Write failing analyzer test**

Create small synthetic CSV fixtures in a temporary directory and assert the
analyzer reports exact P50/P95/P99/max errors, minimum/P1 outward distance,
cycles below `-0.001 m`, control failures, deadline misses, feasibility clips,
and qdot/qddot ratios.

- [ ] **Step 2: Run analyzer test and verify RED**

```bash
python3 tests/test_compare_pico_arm_redundancy_modes.py
```

Expected: import/file failure because the analyzer does not exist.

- [ ] **Step 3: Implement read-only analyzer**

Use Python standard-library `csv`, `json`, and `statistics`; do not require
pandas. Align live accepted rows by `pico_sequence`, report each mode separately,
and calculate P95 regressions relative to `pico`.

- [ ] **Step 4: Run three identical 29.802-second replays**

Use distinct localhost UDP ports and output only under `/tmp`:

```bash
./build/tianji_qp_ik_viewer --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_pico_fast.xml --headless --duration 34 \
  --pico-teleop --pico-bind 127.0.0.1 --pico-port PORT \
  --control-level acceleration --arm-angle-mode MODE \
  --telemetry /tmp/pico_MODE_outward_comparison.csv

python3 /tmp/replay_pico_udp_trace.py \
  --input /tmp/pico_fast_motion_20260812_205428.tjvr --port PORT --lead 0.5
```

Run `pico`, `default_down`, and `outward_only`; retain logs in `/tmp`.

- [ ] **Step 5: Analyze and document evidence**

```bash
python3 scripts/compare_pico_arm_redundancy_modes.py \
  --pico /tmp/pico_pico_outward_comparison.csv \
  --default-down /tmp/pico_default_down_outward_comparison.csv \
  --outward-only /tmp/pico_outward_only_outward_comparison.csv
```

Write exact commands, file names, metrics, acceptance results, and the mapped-
packet limitation to `docs/verification/pico_outward_only_comparison.md`.

- [ ] **Step 6: Run full verification**

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -j"$(nproc)"
python3 tests/test_compare_pico_arm_redundancy_modes.py
git diff --check
git status --short --branch
```

Expected: all tests pass; only `benchmark_results/` remains untracked outside
the planned files.

- [ ] **Step 7: Commit analyzer and verification report**

```bash
git add scripts/compare_pico_arm_redundancy_modes.py \
  tests/test_compare_pico_arm_redundancy_modes.py \
  docs/verification/pico_outward_only_comparison.md
git diff --cached --check
git commit -m "test: compare PICO arm redundancy modes"
```
