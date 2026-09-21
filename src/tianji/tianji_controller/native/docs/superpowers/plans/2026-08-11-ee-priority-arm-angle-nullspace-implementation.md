# End-Effector Priority Arm-Angle Nullspace Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the 6D end-effector QP the primary task and apply the PICO arm-angle reference only through a bounded one-dimensional Jacobian-nullspace refinement, while removing the artificial 3 rad/s Cartesian angular clamp in the PICO MuJoCo profile.

**Architecture:** Both QP builders solve their existing Cartesian problem without an arm-angle rank-one objective. A shared arm-angle helper computes the 7DoF Jacobian nullspace with full-V SVD, solves a scalar bounded correction, and returns the primary solution unchanged on any invalid or degenerate input. Velocity and acceleration paths recompute slack after refinement so the QP equality remains exact.

**Tech Stack:** C++17, Eigen SVD, qpOASES, MuJoCo, Ruckig, GoogleTest, CMake/CTest, Python PICO UDP integration harness.

## Global Constraints

- Work only in `/home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1` on `feature/pico-mujoco-teleop-v1`; do not create or switch worktrees.
- Preserve the physical joint velocity limit of 3.1416 rad/s.
- Keep the controller at 200 Hz and retain all existing PICO stale, solver, joint-limit, braking, and reference-tracking watchdogs.
- Do not add Tianji SDK communication, raw torque output, or physical-robot K/D tuning.
- Keep the existing default-down arm reference when PICO is unavailable.
- Use test-first red-green-refactor for every behavior change.
- The worktree already contains arm-angle changes; stage only files listed by each task and inspect `git diff --cached` before every commit.

---

### Task 1: Bounded 7DoF nullspace refinement primitive

**Files:**
- Modify: `include/tianji_qp_ik/types.hpp`
- Modify: `include/tianji_qp_ik/arm_angle.hpp`
- Modify: `src/arm_angle.cpp`
- Modify: `tests/test_arm_angle.cpp`

**Interfaces:**
- Consumes: `Vec7`, `Mat67`, joint lower/upper vectors, and `ScalarJointTask`.
- Produces: `ArmAngleNullspaceResult refineArmAngleInNullspace(const Vec7&, const Mat67&, const ScalarJointTask&, const Vec7&, const Vec7&, double) noexcept`.

- [ ] **Step 1: Write failing nullspace behavior tests**

Add `#include <limits>` to `tests/test_arm_angle.cpp`, then append these tests:

```cpp
TEST(ArmAngleNullspace, TracksSecondaryWithoutChangingCartesianPrimary) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  ScalarJointTask task;
  task.active = true;
  task.jacobian[6] = 1.0;
  task.target = 0.4;
  task.activation = 1.0;

  const ArmAngleNullspaceResult result = refineArmAngleInNullspace(
      Vec7::Zero(), jacobian, task, Vec7::Constant(-1.0),
      Vec7::Constant(1.0), 1e-10);

  ASSERT_TRUE(result.active);
  EXPECT_NEAR(result.value[6], 0.4, 1e-12);
  EXPECT_TRUE((jacobian * result.value).isZero(1e-12));
  EXPECT_NEAR(result.cartesian_residual, 0.0, 1e-12);
  EXPECT_NEAR(result.after, task.target, 1e-12);
}

TEST(ArmAngleNullspace, ClampsScalarCorrectionToJointBounds) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  ScalarJointTask task;
  task.active = true;
  task.jacobian[6] = 1.0;
  task.target = 2.0;
  task.activation = 1.0;
  Vec7 upper = Vec7::Constant(1.0);
  upper[6] = 0.25;

  const ArmAngleNullspaceResult result = refineArmAngleInNullspace(
      Vec7::Zero(), jacobian, task, Vec7::Constant(-1.0), upper, 1e-10);

  ASSERT_TRUE(result.active);
  EXPECT_NEAR(result.value[6], 0.25, 1e-12);
  EXPECT_LE(result.value[6], upper[6]);
  EXPECT_TRUE((jacobian * result.value).isZero(1e-12));
}

TEST(ArmAngleNullspace, LeavesPrimaryUnchangedForDegenerateOrInvalidTask) {
  Mat67 jacobian = Mat67::Zero();
  jacobian.leftCols<6>() = Eigen::Matrix<double, 6, 6>::Identity();
  const Vec7 primary = Vec7::LinSpaced(-0.3, 0.3);
  ScalarJointTask task;
  task.active = true;
  task.jacobian[0] = 1.0;
  task.target = 1.0;
  task.activation = 1.0;

  const ArmAngleNullspaceResult degenerate = refineArmAngleInNullspace(
      primary, jacobian, task, Vec7::Constant(-1.0),
      Vec7::Constant(1.0), 1e-10);
  EXPECT_FALSE(degenerate.active);
  EXPECT_TRUE(degenerate.value.isApprox(primary, 0.0));

  task.target = std::numeric_limits<double>::quiet_NaN();
  const ArmAngleNullspaceResult invalid = refineArmAngleInNullspace(
      primary, jacobian, task, Vec7::Constant(-1.0),
      Vec7::Constant(1.0), 1e-10);
  EXPECT_FALSE(invalid.active);
  EXPECT_TRUE(invalid.value.isApprox(primary, 0.0));
}
```

- [ ] **Step 2: Build and run the focused test to verify red**

Run:

```bash
cmake --build build -j$(nproc) --target test_arm_angle
```

Expected: compilation fails because `ScalarJointTask::activation`, `ArmAngleNullspaceResult`, and `refineArmAngleInNullspace` do not exist.

- [ ] **Step 3: Add the nullspace API**

Add `activation` to `ScalarJointTask` in
`include/tianji_qp_ik/types.hpp`. Keep the existing `weight` member temporarily
so the not-yet-migrated velocity and acceleration paths continue to compile:

```cpp
struct ScalarJointTask {
  bool active{false};
  Vec7 jacobian{Vec7::Zero()};
  double target{0.0};
  double weight{0.0};  // Removed after both QP paths migrate in Task 3.
  double activation{0.0};
};
```

Add this declaration to `include/tianji_qp_ik/arm_angle.hpp`:

```cpp
struct ArmAngleNullspaceResult {
  Vec7 value{Vec7::Zero()};
  bool active{false};
  double alpha{0.0};
  double before{0.0};
  double after{0.0};
  double cartesian_residual{0.0};
};

ArmAngleNullspaceResult refineArmAngleInNullspace(
    const Vec7& primary, const Mat67& cartesian_jacobian,
    const ScalarJointTask& task, const Vec7& lower, const Vec7& upper,
    double tolerance) noexcept;
```

Implement it in `src/arm_angle.cpp` using `<Eigen/SVD>`:

```cpp
ArmAngleNullspaceResult refineArmAngleInNullspace(
    const Vec7& primary, const Mat67& cartesian_jacobian,
    const ScalarJointTask& task, const Vec7& lower, const Vec7& upper,
    double tolerance) noexcept {
  ArmAngleNullspaceResult result;
  result.value = primary;
  if (!task.active || !primary.allFinite() ||
      !cartesian_jacobian.allFinite() || !task.jacobian.allFinite() ||
      !lower.allFinite() || !upper.allFinite() ||
      !std::isfinite(task.target) || !std::isfinite(task.activation) ||
      !std::isfinite(tolerance) || tolerance <= 0.0 ||
      task.activation <= 0.0 || (lower.array() > upper.array()).any() ||
      (primary.array() < lower.array() - tolerance).any() ||
      (primary.array() > upper.array() + tolerance).any()) {
    return result;
  }

  Eigen::JacobiSVD<Mat67> svd(cartesian_jacobian, Eigen::ComputeFullV);
  if (svd.info() != Eigen::Success || svd.matrixV().cols() != kArmDof) {
    return result;
  }
  Vec7 nullspace = svd.matrixV().col(kArmDof - 1);
  if (!nullspace.allFinite() || nullspace.norm() <= tolerance) {
    return result;
  }
  nullspace.normalize();
  const double nullspace_residual =
      (cartesian_jacobian * nullspace).norm();
  const double sensitivity = task.jacobian.dot(nullspace);
  if (!std::isfinite(nullspace_residual) ||
      nullspace_residual > tolerance || !std::isfinite(sensitivity) ||
      std::abs(sensitivity) <= tolerance) {
    return result;
  }

  result.before = task.jacobian.dot(primary);
  double alpha = std::clamp(task.activation, 0.0, 1.0) *
                 (task.target - result.before) / sensitivity;
  double alpha_lower = -std::numeric_limits<double>::infinity();
  double alpha_upper = std::numeric_limits<double>::infinity();
  for (int joint = 0; joint < kArmDof; ++joint) {
    if (std::abs(nullspace[joint]) <= tolerance) {
      continue;
    }
    double one = (lower[joint] - primary[joint]) / nullspace[joint];
    double two = (upper[joint] - primary[joint]) / nullspace[joint];
    if (one > two) {
      std::swap(one, two);
    }
    alpha_lower = std::max(alpha_lower, one);
    alpha_upper = std::min(alpha_upper, two);
  }
  if (alpha_lower > alpha_upper + tolerance) {
    return result;
  }
  alpha = std::clamp(alpha, alpha_lower, alpha_upper);
  const Vec7 candidate = primary + alpha * nullspace;
  const double cartesian_residual =
      (cartesian_jacobian * (candidate - primary)).norm();
  if (!candidate.allFinite() || !std::isfinite(cartesian_residual) ||
      cartesian_residual > tolerance ||
      (candidate.array() < lower.array() - tolerance).any() ||
      (candidate.array() > upper.array() + tolerance).any()) {
    return result;
  }

  result.value = candidate;
  result.active = std::abs(alpha) > tolerance;
  result.alpha = alpha;
  result.after = task.jacobian.dot(candidate);
  result.cartesian_residual = cartesian_residual;
  return result;
}
```

- [ ] **Step 4: Verify green**

Run:

```bash
cmake --build build -j$(nproc) --target test_arm_angle
ctest --test-dir build -R '^test_arm_angle$' --output-on-failure
```

Expected: `test_arm_angle` passes, including the three new nullspace tests.

- [ ] **Step 5: Commit the primitive**

Run:

```bash
git add include/tianji_qp_ik/types.hpp include/tianji_qp_ik/arm_angle.hpp src/arm_angle.cpp tests/test_arm_angle.cpp
git diff --cached --check
git commit -m "feat: add bounded arm-angle nullspace refinement"
```

---

### Task 2: Make velocity QP Cartesian-primary

**Files:**
- Modify: `src/hierarchical_qp.cpp`
- Modify: `src/hierarchical_qp_ik.cpp`
- Modify: `src/controller.cpp`
- Modify: `tests/test_hierarchical_qp.cpp`
- Modify: `tests/test_controller.cpp`

**Interfaces:**
- Consumes: `refineArmAngleInNullspace` from Task 1 and the existing `ArmIkInput::arm_angle_task`.
- Produces: an `ArmIkResult` whose `qdot` includes only a bounded nullspace correction and whose slack is recomputed as `desired_twist - J*qdot`.

- [ ] **Step 1: Replace the weighted-objective test with a primary-invariance test**

Replace `HierarchicalQpBuilder.AddsArmAngleRankOneObjectiveExactly` in `tests/test_hierarchical_qp.cpp` with:

```cpp
TEST(HierarchicalQpBuilder, ArmAngleTaskDoesNotModifyPrimaryProblem) {
  ArmIkInput data = deterministicInput();
  const HierarchicalQpProblem baseline =
      HierarchicalQpBuilder(testConfig()).build(data);
  data.arm_angle_task.active = true;
  data.arm_angle_task.jacobian << 1.0, -2.0, 0.5, 0.0, 0.2, -0.3, 0.7;
  data.arm_angle_task.target = 1.25;
  data.arm_angle_task.weight = 0.05;  // Transitional red-test trigger.
  data.arm_angle_task.activation = 1.0;
  const HierarchicalQpProblem with_secondary =
      HierarchicalQpBuilder(testConfig()).build(data);

  EXPECT_TRUE(with_secondary.H.isApprox(baseline.H, 0.0));
  EXPECT_TRUE(with_secondary.g.isApprox(baseline.g, 0.0));
  EXPECT_TRUE(with_secondary.A.isApprox(baseline.A, 0.0));
  EXPECT_TRUE(with_secondary.equality.isApprox(baseline.equality, 0.0));
  EXPECT_TRUE(with_secondary.lower.isApprox(baseline.lower, 0.0));
  EXPECT_TRUE(with_secondary.upper.isApprox(baseline.upper, 0.0));
}
```

Add this fake solver beside `KnownSolutionSolver`:

```cpp
class ZeroPrimarySolver final : public IHierarchicalQpSolver {
 public:
  bool initialize(const HierarchicalQpProblem&) override { return true; }

  HierarchicalQpSolution solve(const HierarchicalQpProblem& problem) override {
    HierarchicalQpSolution solution;
    solution.status = SolverStatus::kSolved;
    solution.x.head<kArmDof>().setZero();
    solution.x.tail<6>() = problem.equality;
    solution.detail = "zero_primary";
    return solution;
  }

  void reset() override {}
};
```

Then add this backend-level test:

```cpp
TEST(HierarchicalQpIk, RefinesArmAngleOnlyInCartesianNullspace) {
  ArmIkInput input;
  input.jacobian.leftCols<6>() =
      Eigen::Matrix<double, 6, 6>::Identity();
  input.desired_twist << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  input.bounds.lower = Vec7::Constant(-1.0);
  input.bounds.upper = Vec7::Constant(1.0);
  input.limits.velocity = Vec7::Constant(2.0);
  input.arm_angle_task.active = true;
  input.arm_angle_task.jacobian[6] = 1.0;
  input.arm_angle_task.target = 0.4;
  input.arm_angle_task.activation = 1.0;
  HierarchicalQpIk7 backend(testConfig(), safetyConfig(),
                            std::make_unique<ZeroPrimarySolver>());

  const ArmIkResult result = backend.solve(input);

  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_NEAR(result.qdot[6], 0.4, 1e-12);
  EXPECT_TRUE((input.jacobian * result.qdot + result.slack)
                  .isApprox(input.desired_twist, 1e-12));
  EXPECT_TRUE((result.qdot.array() >= input.bounds.lower.array() - 1e-12)
                  .all());
  EXPECT_TRUE((result.qdot.array() <= input.bounds.upper.array() + 1e-12)
                  .all());
}
```

Delete `DualArmController.ZeroArmAngleWeightReportsTaskInactive` from
`tests/test_controller.cpp`; task activation is now geometric/configuration
driven, not controlled by a weighted-QP coefficient.

- [ ] **Step 2: Run the velocity tests to verify red**

Run:

```bash
cmake --build build -j$(nproc) --target test_hierarchical_qp test_controller
ctest --test-dir build -R '^(test_hierarchical_qp|test_controller)$' --output-on-failure
```

Expected: the primary-invariance test fails because the builder still adds the arm-angle Hessian/gradient, and the solver test fails because no post-QP nullspace correction exists.

- [ ] **Step 3: Remove arm angle from the velocity QP objective**

Delete the `if (input.arm_angle_task.active)` rank-one Hessian and gradient block from `src/hierarchical_qp.cpp`. Do not change the Cartesian equality, slack, posture, continuity, bounds, or solver validation.

- [ ] **Step 4: Refine the accepted velocity solution in nullspace**

In `HierarchicalQpIk7::solve`, after the primary solution passes validation:

```cpp
const ArmAngleNullspaceResult refinement = refineArmAngleInNullspace(
    solution.x.head<kArmDof>(), input.jacobian, input.arm_angle_task,
    input.bounds.lower, input.bounds.upper,
    safety_config_.bound_tolerance);
result.qdot = refinement.value;
result.slack = input.desired_twist - input.jacobian * result.qdot;
result.equality_residual =
    (input.jacobian * result.qdot + result.slack -
     input.desired_twist).norm();
```

Keep solver status, iterations, solve time, details, and active-bound diagnostics unchanged except that bound counting must run after assigning the refined `qdot`.

In `src/controller.cpp`, replace `velocityArmAngleTask` so it fills:

```cpp
ScalarJointTask result;
result.jacobian = task.jacobian;
result.target = std::clamp(config.kp_velocity * task.error_rad,
                           -config.max_velocity_rad_s,
                           config.max_velocity_rad_s);
result.activation = task.activation;
result.active = config.enabled && task.active && result.activation > 0.0;
```

Continue disabling the task for nullspace DLS. Preserve existing requested-rate and current-rate diagnostics.

- [ ] **Step 5: Verify velocity green**

Run:

```bash
cmake --build build -j$(nproc) --target test_hierarchical_qp test_controller
ctest --test-dir build -R '^(test_hierarchical_qp|test_controller)$' --output-on-failure
```

Expected: both test executables pass; the arm-angle QP input no longer changes the primary problem, and the accepted result uses the seventh-axis nullspace while preserving Cartesian equality.

- [ ] **Step 6: Commit the velocity path**

Run:

```bash
git add src/hierarchical_qp.cpp src/hierarchical_qp_ik.cpp src/controller.cpp tests/test_hierarchical_qp.cpp tests/test_controller.cpp
git diff --cached --check
git commit -m "fix: keep arm angle in velocity QP nullspace"
```

---

### Task 3: Make acceleration QP Cartesian-primary

**Files:**
- Modify: `src/acceleration_qp.cpp`
- Modify: `src/acceleration_controller.cpp`
- Modify: `tests/test_acceleration_qp.cpp`
- Modify: `tests/test_acceleration_controller.cpp`

**Interfaces:**
- Consumes: `refineArmAngleInNullspace` and `ArmAccelerationInput::arm_angle_task`.
- Produces: refined `qddot`, recomputed acceleration slack, and unchanged Cartesian equality to numerical tolerance.

- [ ] **Step 1: Write failing acceleration primary-invariance tests**

Replace `AccelerationQpBuilder.AddsArmAngleRankOneObjectiveExactly` with:

```cpp
TEST(AccelerationQpBuilder, ArmAngleTaskDoesNotModifyPrimaryProblem) {
  ArmAccelerationInput data = input();
  const AccelerationQpProblem baseline =
      AccelerationQpBuilder(config()).build(data);
  data.arm_angle_task.active = true;
  data.arm_angle_task.jacobian[6] = 1.0;
  data.arm_angle_task.target = -4.0;
  data.arm_angle_task.weight = 0.05;  // Transitional red-test trigger.
  data.arm_angle_task.activation = 1.0;
  const AccelerationQpProblem with_secondary =
      AccelerationQpBuilder(config()).build(data);

  EXPECT_TRUE(with_secondary.H.isApprox(baseline.H, 0.0));
  EXPECT_TRUE(with_secondary.g.isApprox(baseline.g, 0.0));
  EXPECT_TRUE(with_secondary.A.isApprox(baseline.A, 0.0));
  EXPECT_TRUE(with_secondary.equality.isApprox(baseline.equality, 0.0));
  EXPECT_TRUE(with_secondary.lower.isApprox(baseline.lower, 0.0));
  EXPECT_TRUE(with_secondary.upper.isApprox(baseline.upper, 0.0));
}
```

Add this acceleration-controller integration test. It forces a zero accepted
primary acceleration, requests a PICO elbow-plane change, and verifies that the
controller's nonzero correction stays in the observed Cartesian nullspace:

```cpp
TEST(AccelerationController, RefinesArmAngleOnlyInCartesianNullspace) {
  MujocoRobot robot(modelPath());
  initialize(robot);
  const DualArmReferences stationary = references(robot);
  DualArmDirectionReferences requested;
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmKinematicSample model =
        robot.armKinematicsAt(side, robot.armPosition(side));
    const Eigen::Vector3d axis =
        (model.wrist_position - model.shoulder_position).normalized();
    Eigen::Vector3d radial = model.elbow_position - model.shoulder_position;
    radial -= axis * axis.dot(radial);
    ArmDirectionReference reference;
    reference.valid = true;
    reference.direction =
        Eigen::AngleAxisd(side == ArmSide::kLeft ? 0.45 : -0.45, axis) *
        radial.normalized();
    reference.source = ArmDirectionReferenceSource::kPico;
    (side == ArmSide::kLeft ? requested.left : requested.right) = reference;
  }
  auto left = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  auto right = std::make_unique<FixedAccelerationSolver>(Vec7::Zero());
  FixedAccelerationSolver* left_observer = left.get();
  DualArmAccelerationController controller(robot, config(), std::move(left),
                                           std::move(right));

  const AccelerationControllerDiagnostics result =
      controller.step(stationary, requested, 0.005);

  ASSERT_TRUE(result.accepted) << toString(result.hold_reason);
  ASSERT_TRUE(result.left.arm_angle_task_active);
  ASSERT_EQ(left_observer->problems.size(), 1U);
  const AccelerationQpProblem& problem = left_observer->problems.front();
  EXPECT_GT(result.left.qp.qddot.norm(), 1e-8);
  EXPECT_LT((problem.A.leftCols<kArmDof>() * result.left.qp.qddot).norm(),
            1e-8);
  EXPECT_TRUE((result.left.qp.qddot.array() >=
               result.left.bounds.lower.array() - 1e-8)
                  .all());
  EXPECT_TRUE((result.left.qp.qddot.array() <=
               result.left.bounds.upper.array() + 1e-8)
                  .all());
  EXPECT_LT(result.left.qp.equality_residual, 1e-8);
}
```

Delete `AccelerationController.ZeroArmAngleWeightReportsTaskInactive` from
`tests/test_acceleration_controller.cpp`.

- [ ] **Step 2: Run acceleration tests to verify red**

Run:

```bash
cmake --build build -j$(nproc) --target test_acceleration_qp test_acceleration_controller
ctest --test-dir build -R '^(test_acceleration_qp|test_acceleration_controller)$' --output-on-failure
```

Expected: the builder invariance test fails on the rank-one objective and the controller test shows no bounded post-QP refinement.

- [ ] **Step 3: Remove arm angle from the acceleration QP objective**

Delete the `if (input.arm_angle_task.active)` Hessian and gradient block from `src/acceleration_qp.cpp`. Preserve regularization, jerk, posture, equality, slack, and all hard acceleration bounds.

- [ ] **Step 4: Apply bounded acceleration nullspace refinement**

In `src/acceleration_controller.cpp`, fill the scalar task without a weighted-QP weight:

```cpp
input.arm_angle_task.jacobian = arm_diagnostics.arm_angle.jacobian;
input.arm_angle_task.target = std::clamp(
    config_.arm_angle.kp_acceleration * arm_diagnostics.arm_angle.error_rad -
        config_.arm_angle.kd_acceleration *
            arm_diagnostics.arm_angle_current_rate,
    -config_.arm_angle.max_acceleration_rad_s2,
    config_.arm_angle.max_acceleration_rad_s2);
input.arm_angle_task.activation = arm_diagnostics.arm_angle.activation;
input.arm_angle_task.active = config_.arm_angle.enabled &&
                              arm_diagnostics.arm_angle.active &&
                              input.arm_angle_task.activation > 0.0;
```

After validating the primary QP solution, apply:

```cpp
const ArmAngleNullspaceResult refinement = refineArmAngleInNullspace(
    solution.x.head<kArmDof>(), input.jacobian, input.arm_angle_task,
    input.bounds.lower, input.bounds.upper,
    config_.safety.bound_tolerance);
arm_diagnostics.qp.qddot = refinement.value;
arm_diagnostics.qp.slack =
    problem.equality - input.jacobian * arm_diagnostics.qp.qddot;
arm_diagnostics.qp.equality_residual =
    (input.jacobian * arm_diagnostics.qp.qddot +
     arm_diagnostics.qp.slack - problem.equality).norm();
```

Count active acceleration bounds after refinement and integrate `qdot_ref` and
`q_ref` from the refined acceleration.

- [ ] **Step 5: Verify acceleration green**

Run:

```bash
cmake --build build -j$(nproc) --target test_acceleration_qp test_acceleration_controller
ctest --test-dir build -R '^(test_acceleration_qp|test_acceleration_controller)$' --output-on-failure
```

Expected: both tests pass; arm angle does not alter the acceleration QP matrices and the refined acceleration preserves Cartesian equality and hard bounds.

- [ ] **Step 6: Commit the acceleration path**

Run:

```bash
git add src/acceleration_qp.cpp src/acceleration_controller.cpp \
  tests/test_acceleration_qp.cpp \
  tests/test_acceleration_controller.cpp
git diff --cached --check
git commit -m "fix: keep arm angle in acceleration QP nullspace"
```

---

### Task 4: Remove obsolete weights and tune the PICO response profile

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `include/tianji_qp_ik/types.hpp`
- Modify: `src/config.cpp`
- Modify: `tests/test_config.cpp`
- Modify: `tests/test_hierarchical_qp.cpp`
- Modify: `tests/test_acceleration_qp.cpp`
- Modify: `config/qp_ik.yaml`
- Modify: `config/qp_ik_hierarchical.yaml`
- Modify: `config/qp_ik_qp_limit.yaml`
- Modify: `config/qp_ik_upper_limit.yaml`
- Modify: `config/qp_ik_cartesian_otg_velocity.yaml`
- Modify: `config/qp_ik_cartesian_otg_acceleration.yaml`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Modify: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- Removes: `ArmAngleConfig::velocity_weight` and `ArmAngleConfig::acceleration_weight` plus their YAML keys.
- Changes only PICO response values: Cartesian angular command limit and orientation slack penalties.

- [ ] **Step 1: Write failing config expectations**

Update `tests/test_config.cpp` so the PICO profile asserts:

```cpp
EXPECT_DOUBLE_EQ(config.cartesian_servo.max_angular_velocity, 6.0);
EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_weight_orientation, 3.0e4);
EXPECT_DOUBLE_EQ(config.acceleration_qp.slack_weight_orientation, 3.0e5);
```

Delete the two weight assertions from `Config.LoadsProjectDefaults` and delete
the `velocity_weight` and `acceleration_weight` replacement cases from
`Config.RejectsInvalidArmAngleConfiguration`. Do not change that test's
remaining max-rate, radius, or direction-rate validation cases.

- [ ] **Step 2: Run config test to verify red**

Run:

```bash
cmake --build build -j$(nproc) --target test_config
ctest --test-dir build -R '^test_config$' --output-on-failure
```

Expected: `Config.LoadsPicoTeleopProfile` fails because the loaded values are
still `3.0`, `3.0e3`, and `3.0e3`.

- [ ] **Step 3: Remove obsolete weighted-QP configuration**

Delete these fields from `ArmAngleConfig` and from parser/validation code:

```cpp
double velocity_weight{3.0};
double acceleration_weight{20.0};
```

Delete the transitional `weight` member from `ScalarJointTask`. Delete the
`data.arm_angle_task.weight = 0.05` lines from
`HierarchicalQpBuilder.ArmAngleTaskDoesNotModifyPrimaryProblem` and
`AccelerationQpBuilder.ArmAngleTaskDoesNotModifyPrimaryProblem`. At that point
no primary or secondary path consumes a weighted arm-angle coefficient.

Remove `velocity_weight` and `acceleration_weight` from every `arm_angle` YAML
section listed above. Keep `kp_velocity`, `max_velocity_rad_s`,
`kp_acceleration`, `kd_acceleration`, `max_acceleration_rad_s2`, radius values,
and direction rate limiting.

Run:

```bash
rg -n 'velocity_weight|acceleration_weight' include src tests config
```

Expected: no matches.

- [ ] **Step 4: Apply PICO-only response values**

Set these exact values in `config/qp_ik_pico_teleop.yaml`:

```yaml
cartesian_servo:
  max_angular_velocity: 6.0

acceleration_qp:
  slack_weight_orientation: 3.0e5

hierarchical_qp:
  slack_weight_orientation: 3.0e4
```

Do not alter the model joint velocity, `joint_limits.velocity_scale`, or
`joint_acceleration_limits.velocity_scale`.

- [ ] **Step 5: Extend the PICO replay harness for a fast orientation profile**

Change packet generation to accept orientation amplitude:

```python
def encode_packet(sequence, source_timestamp_ns, phase, orientation_amplitude):
    # Keep the existing positions and packet layout unchanged.
    left_quaternion = quaternion(
        (0.0, 0.0, 1.0), orientation_amplitude * math.sin(phase)
    )
    right_quaternion = quaternion(
        (0.0, 1.0, 0.0), -orientation_amplitude * math.sin(phase)
    )
```

Replace the hard-coded phase and call in `run_test` with:

```python
phase = (
    2.0
    * math.pi
    * arguments.motion_frequency
    * (sequence - 1)
    / arguments.send_rate
)
sender.sendto(
    encode_packet(
        sequence,
        source_timestamp_ns,
        phase,
        arguments.orientation_amplitude,
    ),
    ("127.0.0.1", port),
)
```

Add these parser arguments; their defaults preserve existing CTest behavior:

```python
parser.add_argument("--motion-frequency", type=float, default=0.5)
parser.add_argument("--orientation-amplitude", type=float, default=0.08)
parser.add_argument("--max-steady-orientation-error-mean", type=float)
```

After loading and validating telemetry rows, add the optional fast-profile
metric. The existing telemetry columns are `left_orientation_error_rad` and
`right_orientation_error_rad`:

```python
if arguments.max_steady_orientation_error_mean is not None:
    steady = [
        max(
            float(row["left_orientation_error_rad"]),
            float(row["right_orientation_error_rad"]),
        )
        for row in rows
        if row["pico_live"] == "1" and int(row["pico_sequence"]) >= 120
    ]
    if not steady:
        raise AssertionError("no live steady-state orientation samples")
    steady_mean = sum(steady) / len(steady)
    steady_peak = max(steady)
    if steady_mean > arguments.max_steady_orientation_error_mean:
        raise AssertionError(
            f"orientation error mean {steady_mean} exceeds "
            f"{arguments.max_steady_orientation_error_mean}"
        )
    summary["steady_orientation_error_mean"] = f"{steady_mean:.9f}"
    summary["steady_orientation_error_peak"] = f"{steady_peak:.9f}"
```

- [ ] **Step 6: Verify config and fast PICO response**

Run:

```bash
cmake --build build -j$(nproc)
ctest --test-dir build -R '^(test_config|pico_viewer_integration_velocity|pico_viewer_integration_acceleration)$' --output-on-failure
python3 tests/test_pico_viewer_integration.py \
  --viewer ./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_test.xml \
  --control-level velocity \
  --viewer-duration 4.1 \
  --send-duration 3.2 \
  --send-rate 72 \
  --motion-frequency 2.0 \
  --orientation-amplitude 0.4 \
  --max-steady-orientation-error-mean 0.118
python3 tests/test_pico_viewer_integration.py \
  --viewer ./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_test.xml \
  --control-level acceleration \
  --viewer-duration 4.1 \
  --send-duration 3.2 \
  --send-rate 72 \
  --motion-frequency 2.0 \
  --orientation-amplitude 0.4 \
  --max-steady-orientation-error-mean 0.115
```

Expected: no control failures, no jump rejection, both fast-response means are below their recorded baselines, and physical joint velocity limits remain active when required.

- [ ] **Step 7: Run complete verification**

Run:

```bash
ctest --test-dir build --output-on-failure
./build/tianji_cartesian_otg_benchmark \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_test.xml \
  --steps 600 \
  --output /tmp/tianji_pico_response_after_nullspace.csv
```

Expected: all CTest tests pass; benchmark reports zero failures and finite
metrics for direct velocity QP, OTG velocity QP, and OTG acceleration QP.

- [ ] **Step 8: Commit configuration and replay coverage**

Run:

```bash
git add include/tianji_qp_ik/config.hpp include/tianji_qp_ik/types.hpp \
  src/config.cpp tests/test_config.cpp tests/test_hierarchical_qp.cpp \
  tests/test_acceleration_qp.cpp \
  config/qp_ik.yaml config/qp_ik_hierarchical.yaml \
  config/qp_ik_qp_limit.yaml config/qp_ik_upper_limit.yaml \
  config/qp_ik_cartesian_otg_velocity.yaml \
  config/qp_ik_cartesian_otg_acceleration.yaml \
  config/qp_ik_pico_teleop.yaml tests/test_pico_viewer_integration.py
git diff --cached --check
git commit -m "fix: prioritize PICO end-effector orientation response"
```

---

### Task 5: Final regression audit

**Files:**
- Inspect: all files changed by Tasks 1-4
- Inspect: `docs/superpowers/specs/2026-08-11-ee-priority-arm-angle-nullspace-design.md`

**Interfaces:**
- Consumes: completed nullspace and response changes.
- Produces: a verified branch with no temporary instrumentation or unintended staged changes.

- [ ] **Step 1: Audit the final diff**

Run:

```bash
git status --short
git diff --check
git diff --stat 5515ef2..HEAD
rg -n 'velocity_weight|acceleration_weight|AddsArmAngleRankOneObjective' \
  include src tests config
```

Expected: no whitespace errors; obsolete weighted arm-angle objective names and
YAML keys are absent. Existing unrelated worktree changes remain visible but
unstaged unless explicitly included by Tasks 1-4.

- [ ] **Step 2: Re-run the critical suite from a fresh build invocation**

Run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

Expected: configure and build succeed and all tests pass.

- [ ] **Step 3: Record final evidence**

Report the complete CTest pass count, fast PICO velocity/acceleration mean and
peak orientation errors, active velocity/acceleration-bound percentages,
control failures, deadline misses, and the final commit hashes. Do not claim
hardware readiness; state that SDK impedance K/D and per-joint hardware
acceleration limits remain a separate integration step.
