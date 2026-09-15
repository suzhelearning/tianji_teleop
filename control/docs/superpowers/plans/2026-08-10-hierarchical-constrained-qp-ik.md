# Hierarchical Constrained QP-IK Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Viewer default with a 13-variable constrained QP-IK, add a bounded null-space DLS runtime baseline, and expose enough diagnostics for deterministic A/B verification.

**Architecture:** Keep target generation, world-frame kinematics, Cartesian servo, integration, and dual-arm safety in one shared controller shell. Delegate only arm joint-velocity calculation to `HierarchicalQpIk7` or `NullspaceDlsIk7`; retain the legacy 7-variable QP modules for regression until the new path is accepted.

**Tech Stack:** C++17, Eigen 3.4, MuJoCo 3.6, qpOASES 3.2, yaml-cpp 0.8, GLFW 3.4, GoogleTest, CMake/Ninja, Pixi.

## Global Constraints

- Work only in `/home/zj/current_robotics/TJ_arm/TJ_arm_control/.worktrees/qp-ik-v1` on `feature/mujoco-cpp-qp-ik-v1`.
- Keep the existing linked worktree and branch; do not merge, push, delete the branch, or remove the worktree.
- Preserve world-frame pose error paired with the validated world-frame MuJoCo TCP Jacobian.
- Use Cartesian ordering `[linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]` everywhere.
- Run the new QP and DLS paths at `200 Hz`, with `dt = 0.005 s`.
- Use qpOASES for the new 13-variable, 6-equality QP; do not add ProxSuite.
- Do not invent hard acceleration limits; use the continuity objective only.
- Keep joint position and velocity limits as hard constraints.
- Keep dual-arm all-or-nothing integration.
- Keep legacy OSQP, qpOASES, builder, benchmark, and tests during migration.
- Do not add VR input, SDK transport, collision constraints, dynamics, torque control, or coupled 14-DoF optimization.
- Add behavior test-first and observe every new test fail for the intended reason before implementation.
- Use plain-text formulas in user-facing documentation.

## File Structure

Create focused modules rather than expanding the legacy fixed-size QP files:

- `include/tianji_qp_ik/velocity_ik.hpp`: common algorithm enum support, input/result contract, joint-bound result, and backend interface.
- `src/velocity_ik.cpp`: shared hard-bound construction, result diagnostics, backend factory, and algorithm strings.
- `include/tianji_qp_ik/hierarchical_qp.hpp`: fixed 13-variable/6-equality problem and solution types plus builder declaration.
- `src/hierarchical_qp.cpp`: exact QP matrix construction and problem/result validation helpers.
- `include/tianji_qp_ik/hierarchical_qpoases_solver.hpp`: abstract 13-variable solver seam and qpOASES implementation declaration.
- `src/hierarchical_qpoases_solver.cpp`: persistent `SQProblem(13, 6)` initialization, hotstart, and result mapping.
- `include/tianji_qp_ik/hierarchical_qp_ik.hpp`: `HierarchicalQpIk7` arm-backend declaration.
- `src/hierarchical_qp_ik.cpp`: builder/solver orchestration and conversion to common diagnostics.
- `include/tianji_qp_ik/nullspace_dls.hpp`: bounded null-space DLS backend declaration.
- `src/nullspace_dls.cpp`: damped pseudoinverse, null-space posture, and uniform bound scaling.
- `tests/test_velocity_ik.cpp`: common bound construction and diagnostics tests.
- `tests/test_hierarchical_qp.cpp`: exact builder and validation tests.
- `tests/test_hierarchical_qpoases_solver.cpp`: equality, bound, slack, and hotstart tests.
- `tests/test_nullspace_dls.cpp`: DLS formula, singularity, and direction-preserving scaling tests.
- `tests/test_hierarchical_controller.cpp`: reference state, switching, failure, convergence, singularity, and unreachable integration tests.
- `apps/benchmark_hierarchical_ik.cpp`: deterministic QP/DLS A/B timing and tracking benchmark.

Modify:

- `include/tianji_qp_ik/config.hpp`, `src/config.cpp`, and all project YAML configurations for new algorithm/QP/DLS parameters.
- `include/tianji_qp_ik/controller.hpp` and `src/controller.cpp` to use the common backend contract and explicit reference state.
- `include/tianji_qp_ik/telemetry.hpp`, `src/telemetry.cpp`, and `apps/run_qp_ik_viewer.cpp` for algorithm commands and diagnostics.
- `CMakeLists.txt` for new library sources, tests, and benchmark.
- `pixi.toml`, `README.md`, and `docs/verification/hierarchical_qp_ik_results.md` for commands, controls, formulas, and verified results.

---

### Task 1: Add Configuration, Common IK Types, and Shared Hard Bounds

**Files:**
- Create: `include/tianji_qp_ik/velocity_ik.hpp`
- Create: `src/velocity_ik.cpp`
- Create: `tests/test_velocity_ik.cpp`
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `config/qp_ik.yaml`
- Modify: `config/qp_ik_upper_limit.yaml`
- Create: `config/qp_ik_hierarchical.yaml`
- Modify: `tests/test_config.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `Vec6`, `Vec7`, `Mat67`, `ArmLimits`, `SolverStatus`, and existing YAML loading helpers.
- Produces: `IkAlgorithm`, `HierarchicalQpConfig`, `DlsConfig`, `JointVelocityBounds`, `ArmIkInput`, `ArmIkResult`, `IArmVelocityIk`, `computeJointVelocityBounds`, `makeArmVelocityIk`, and algorithm string conversion.

- [ ] **Step 1: Write failing configuration and hard-bound tests**

Add tests that require the new schema and common bound behavior:

```cpp
TEST(Config, LoadsHierarchicalQpProfile) {
  const auto path = std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) /
                    "config" / "qp_ik_hierarchical.yaml";
  const QpIkConfig config = loadConfig(path.string());
  EXPECT_EQ(config.ik_algorithm, IkAlgorithm::kHierarchicalQp);
  EXPECT_DOUBLE_EQ(config.controller.rate_hz, 200.0);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_weight_position, 1.0e4);
  EXPECT_DOUBLE_EQ(config.hierarchical_qp.slack_weight_orientation, 3.0e3);
  EXPECT_DOUBLE_EQ(config.dls.damping, 1.0e-3);
}

TEST(VelocityIk, IntersectsVelocityAndOneStepPositionBounds) {
  JointLimitConfig config;
  config.margin_rad = 0.05;
  config.velocity_scale = 1.0;
  ArmLimits limits;
  limits.lower_position = Vec7::Constant(-1.0);
  limits.upper_position = Vec7::Constant(1.0);
  limits.velocity = Vec7::Constant(3.0);
  Vec7 q = Vec7::Zero();
  q[0] = 0.949;

  const JointVelocityBounds bounds =
      computeJointVelocityBounds(q, limits, config, 0.005);

  EXPECT_NEAR(bounds.upper[0], 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(bounds.lower[1], -3.0);
  EXPECT_TRUE(bounds.upper_source[0] == BoundSource::kPosition);
  EXPECT_TRUE(bounds.upper_source[1] == BoundSource::kVelocity);
}
```

Also add rejection tests for non-positive slack weights, negative continuity
weight, non-positive equality tolerance, non-positive DLS damping, and unknown
`ik.algorithm`.

- [ ] **Step 2: Run the focused tests and verify red**

Run:

```bash
cmake --build build --target test_config test_velocity_ik -j2
```

Expected: configuration compilation fails because the new types and YAML keys
do not exist; after adding the test target declaration, `test_velocity_ik`
fails to link because `computeJointVelocityBounds` is missing.

- [ ] **Step 3: Add the common configuration and IK contract**

Add to `config.hpp`:

```cpp
enum class IkAlgorithm { kHierarchicalQp, kNullspaceDls };

struct HierarchicalQpConfig {
  double lambda_reg{1e-4};
  double posture_weight{1e-3};
  double continuity_weight{1e-3};
  double nominal_gain{0.2};
  double slack_weight_position{1e4};
  double slack_weight_orientation{3e3};
  double equality_tolerance{1e-8};
};

struct DlsConfig {
  double damping{1e-3};
  double nominal_gain{0.2};
};
```

Add `ik_algorithm`, `hierarchical_qp`, and `dls` to `QpIkConfig`. Parse exact
strings `hierarchical_qp` and `nullspace_dls`. Require positive regularization,
slack weights, equality tolerance, and damping; allow non-negative posture and
continuity weights.

Define the common contract in `velocity_ik.hpp`:

```cpp
enum class BoundSource { kVelocity, kPosition };

struct JointVelocityBounds {
  Vec7 lower{Vec7::Zero()};
  Vec7 upper{Vec7::Zero()};
  std::array<BoundSource, kArmDof> lower_source{};
  std::array<BoundSource, kArmDof> upper_source{};
};

struct ArmIkInput {
  Vec7 q_measured{Vec7::Zero()};
  Vec7 q_ref{Vec7::Zero()};
  Vec7 qdot_prev{Vec7::Zero()};
  Vec6 slack_prev{Vec6::Zero()};
  Mat67 jacobian{Mat67::Zero()};
  Vec6 desired_twist{Vec6::Zero()};
  ArmLimits limits;
  JointVelocityBounds bounds;
  double dt{0.005};
};

struct ArmIkResult {
  SolverStatus status{SolverStatus::kInvalidInput};
  Vec7 qdot{Vec7::Zero()};
  Vec6 slack{Vec6::Zero()};
  int iterations{0};
  double solve_time_us{0.0};
  double equality_residual{0.0};
  int active_position_bound_count{0};
  int active_velocity_bound_count{0};
  double qdot_max_ratio{0.0};
  std::string_view detail{"not_solved"};
};

class IArmVelocityIk {
 public:
  virtual ~IArmVelocityIk() = default;
  virtual ArmIkResult solve(const ArmIkInput& input) = 0;
  virtual void reset() = 0;
  virtual IkAlgorithm algorithm() const noexcept = 0;
};
```

Implement the exact bound equations from the approved spec and preserve the
source that wins each `max`/`min` operation. Declare the factory now and define
it in Task 4 after both backends exist:

```cpp
std::unique_ptr<IArmVelocityIk> makeArmVelocityIk(
    IkAlgorithm algorithm, const QpIkConfig& config);
```

- [ ] **Step 4: Add complete YAML profiles**

Add `ik`, `hierarchical_qp`, and `dls` sections to both legacy profiles so all
files pass one strict schema. Create `qp_ik_hierarchical.yaml` with:

```yaml
controller:
  rate_hz: 200.0

ik:
  algorithm: hierarchical_qp

cartesian_servo:
  kp_position: [4.0, 4.0, 4.0]
  kp_orientation: [3.0, 3.0, 3.0]
  max_linear_velocity: 1.00
  max_angular_velocity: 3.14

hierarchical_qp:
  lambda_reg: 1.0e-4
  posture_weight: 1.0e-3
  continuity_weight: 1.0e-3
  nominal_gain: 0.2
  slack_weight_position: 1.0e4
  slack_weight_orientation: 3.0e3
  equality_tolerance: 1.0e-8

dls:
  damping: 1.0e-3
  nominal_gain: 0.2
```

Copy all existing legacy `qp`, joint-limit, solver, safety, and trajectory
sections unchanged into the new file.

- [ ] **Step 5: Build and run focused tests**

Run:

```bash
cmake --build build --target test_config test_velocity_ik -j2
./build/test_config
./build/test_velocity_ik
```

Expected: both executables report all tests passed.

- [ ] **Step 6: Commit Task 1**

```bash
git add CMakeLists.txt config include/tianji_qp_ik/config.hpp \
  include/tianji_qp_ik/velocity_ik.hpp src/config.cpp src/velocity_ik.cpp \
  tests/test_config.cpp tests/test_velocity_ik.cpp
git commit -m "feat: add shared velocity IK configuration and bounds"
```

---

### Task 2: Build the 13-Variable Hierarchical QP Exactly

**Files:**
- Create: `include/tianji_qp_ik/hierarchical_qp.hpp`
- Create: `src/hierarchical_qp.cpp`
- Create: `tests/test_hierarchical_qp.cpp`
- Modify: `include/tianji_qp_ik/safety.hpp`
- Modify: `src/safety.cpp`
- Modify: `tests/test_safety.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `HierarchicalQpConfig`, `ArmIkInput`, and `JointVelocityBounds` from Task 1.
- Produces: fixed-size `Vec13`, `Mat13`, `Mat6x13`, `HierarchicalQpProblem`, `HierarchicalQpSolution`, and `HierarchicalQpBuilder::build`.

- [ ] **Step 1: Write exact-expansion tests**

Define a deterministic nonzero Jacobian and assert every block:

```cpp
TEST(HierarchicalQpBuilder, ExpandsSlackPostureAndContinuityExactly) {
  HierarchicalQpConfig config;
  config.lambda_reg = 0.1;
  config.posture_weight = 0.2;
  config.continuity_weight = 0.3;
  config.nominal_gain = 0.4;
  config.slack_weight_position = 100.0;
  config.slack_weight_orientation = 30.0;
  HierarchicalQpBuilder builder(config);
  const ArmIkInput input = deterministicInput();

  const HierarchicalQpProblem problem = builder.build(input);

  EXPECT_TRUE(problem.H.topLeftCorner<7, 7>().isApprox(
      0.6 * Mat77::Identity(), 1e-14));
  EXPECT_TRUE(problem.H.topRightCorner<7, 6>().isZero(0.0));
  EXPECT_TRUE(problem.H.bottomRightCorner<6, 6>().diagonal().isApprox(
      (Vec6() << 100.0, 100.0, 100.0, 30.0, 30.0, 30.0).finished()));
  EXPECT_TRUE(problem.A.leftCols<7>().isApprox(input.jacobian));
  EXPECT_TRUE(problem.A.rightCols<6>().isApprox(
      Eigen::Matrix<double, 6, 6>::Identity()));
  EXPECT_TRUE(problem.equality.isApprox(input.desired_twist));
}
```

Add tests proving `qdot=0, slack=Vd` satisfies the equality for full-rank,
singular, and zero Jacobians; add non-finite and invalid-bound tests.

- [ ] **Step 2: Run the test and verify red**

Run:

```bash
cmake --build build --target test_hierarchical_qp -j2
```

Expected: compile failure because `HierarchicalQpBuilder` and fixed-size types
do not exist.

- [ ] **Step 3: Implement fixed-size problem types and builder**

Define:

```cpp
inline constexpr int kHierarchicalVariables = 13;
inline constexpr int kTrackingEqualities = 6;
using Vec13 = Eigen::Matrix<double, 13, 1>;
using Mat13 = Eigen::Matrix<double, 13, 13>;
using Mat6x13 = Eigen::Matrix<double, 6, 13>;

struct HierarchicalQpProblem {
  Mat13 H{Mat13::Zero()};
  Vec13 g{Vec13::Zero()};
  Mat6x13 A{Mat6x13::Zero()};
  Vec13 lower{Vec13::Zero()};
  Vec13 upper{Vec13::Zero()};
  Vec6 equality{Vec6::Zero()};
  JointVelocityBounds joint_bounds;
};

struct HierarchicalQpSolution {
  SolverStatus status{SolverStatus::kInvalidInput};
  Vec13 x{Vec13::Zero()};
  int iterations{0};
  double update_time_us{0.0};
  double solve_time_us{0.0};
  std::string_view detail{"not_initialized"};
};
```

Implement the approved objective exactly. Set slack variable bounds to
`-qpOASES::INFTY/+qpOASES::INFTY` only inside the solver copy step; keep the
logical problem bounds as IEEE infinities and make validation explicitly allow
infinity only for slack bounds.

- [ ] **Step 4: Implement problem and solution validation helpers**

Add functions:

```cpp
SafetyDecision validateHierarchicalProblem(
    const HierarchicalQpProblem& problem,
    const SafetyConfig& safety);

SafetyDecision validateHierarchicalSolution(
    const HierarchicalQpProblem& problem,
    const HierarchicalQpSolution& solution,
    const HierarchicalQpConfig& config,
    const SafetyConfig& safety);
```

Require finite `H`, `g`, `A`, equality, and qdot bounds; permit infinity only
in indices 7 through 12 of variable bounds. Check Hessian symmetry/positive
definiteness, solved status, finite `x`, qdot hard bounds, and:

```cpp
(problem.A * solution.x - problem.equality).norm()
    <= config.equality_tolerance
```

Add `HoldReason::kEqualityViolation`, its `toString` mapping, and safety tests.
Return that reason whenever an otherwise solved result violates the tracking
equality tolerance, so the Viewer can distinguish equality failure from hard
joint-bound failure.

- [ ] **Step 5: Run focused tests**

```bash
cmake --build build --target test_hierarchical_qp -j2
./build/test_hierarchical_qp
```

Expected: all builder and validation cases pass.

- [ ] **Step 6: Commit Task 2**

```bash
git add CMakeLists.txt include/tianji_qp_ik/hierarchical_qp.hpp \
  include/tianji_qp_ik/safety.hpp src/hierarchical_qp.cpp src/safety.cpp \
  tests/test_hierarchical_qp.cpp tests/test_safety.cpp
git commit -m "feat: build hierarchical slack QP problems"
```

---

### Task 3: Implement the Persistent 13x6 qpOASES Solver

**Files:**
- Create: `include/tianji_qp_ik/hierarchical_qpoases_solver.hpp`
- Create: `src/hierarchical_qpoases_solver.cpp`
- Create: `tests/test_hierarchical_qpoases_solver.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `HierarchicalQpProblem` and `HierarchicalQpSolution` from Task 2.
- Produces: `IHierarchicalQpSolver` and `HierarchicalQpoasesSolver` with persistent initialize/hotstart/reset behavior.

- [ ] **Step 1: Write solver equality, bound, slack, and hotstart tests**

Add deterministic tests:

```cpp
TEST(HierarchicalQpoasesSolver, SolvesTrackingEqualityWithSlack) {
  const HierarchicalQpProblem problem = fullRankTrackingProblem();
  HierarchicalQpoasesSolver solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(problem));
  const HierarchicalQpSolution result = solver.solve(problem);
  ASSERT_EQ(result.status, SolverStatus::kSolved) << result.detail;
  EXPECT_LT((problem.A * result.x - problem.equality).norm(), 1e-8);
  EXPECT_LE((result.x.head<7>() - problem.upper.head<7>()).maxCoeff(), 1e-9);
  EXPECT_LE((problem.lower.head<7>() - result.x.head<7>()).maxCoeff(), 1e-9);
}

TEST(HierarchicalQpoasesSolver, ZeroJacobianUsesFiniteSlack) {
  const HierarchicalQpProblem problem = zeroJacobianProblem();
  HierarchicalQpoasesSolver solver(accurateConfig());
  ASSERT_TRUE(solver.initialize(problem));
  const auto result = solver.solve(problem);
  ASSERT_EQ(result.status, SolverStatus::kSolved);
  EXPECT_TRUE(result.x.head<7>().isZero(1e-9));
  EXPECT_TRUE(result.x.tail<6>().isApprox(problem.equality, 1e-8));
}
```

Add a hotstart test that changes `H`, `A`, equality, and bounds while asserting
`setupCount() == 1` and `hotstartCount() >= 1`.

- [ ] **Step 2: Run the solver target and verify red**

```bash
cmake --build build --target test_hierarchical_qpoases_solver -j2
```

Expected: compile failure because the solver class is absent.

- [ ] **Step 3: Implement the solver seam and storage**

Define:

```cpp
class IHierarchicalQpSolver {
 public:
  virtual ~IHierarchicalQpSolver() = default;
  virtual bool initialize(const HierarchicalQpProblem& problem) = 0;
  virtual HierarchicalQpSolution solve(const HierarchicalQpProblem& problem) = 0;
  virtual void reset() = 0;
};
```

`HierarchicalQpoasesSolver` owns preallocated row-major arrays for 13x13 `H`,
13-vector `g/lower/upper`, 6x13 `A`, and 6-vector equality lower/upper. Create:

```cpp
qpOASES::SQProblem(kHierarchicalVariables, kTrackingEqualities,
                   qpOASES::HST_POSDEF)
```

Use `setToMPC`, no console printing, existing tight tolerances, configured nWSR,
and configured CPU limit.

- [ ] **Step 4: Implement initialize and full hotstart**

On initialize, call qpOASES `init` with `H`, `g`, `A`, variable bounds, and
equality lower/upper. Cache the initialized solution so the first identical
`solve` returns it without a second optimization, matching the legacy solver's
tested behavior.

On later cycles call the full `hotstart` overload with all updated matrices and
bounds. Map statuses identically to the legacy solver and return a zeroed
solution on failure.

- [ ] **Step 5: Run focused tests**

```bash
cmake --build build --target test_hierarchical_qpoases_solver -j2
./build/test_hierarchical_qpoases_solver
```

Expected: all equality, slack, bound, failure, initialization, and hotstart
tests pass.

- [ ] **Step 6: Commit Task 3**

```bash
git add CMakeLists.txt include/tianji_qp_ik/hierarchical_qpoases_solver.hpp \
  src/hierarchical_qpoases_solver.cpp tests/test_hierarchical_qpoases_solver.cpp
git commit -m "feat: solve hierarchical QP with qpOASES"
```

---

### Task 4: Add Hierarchical-QP and Null-Space-DLS Arm Backends

**Files:**
- Create: `include/tianji_qp_ik/hierarchical_qp_ik.hpp`
- Create: `src/hierarchical_qp_ik.cpp`
- Create: `include/tianji_qp_ik/nullspace_dls.hpp`
- Create: `src/nullspace_dls.cpp`
- Create: `tests/test_nullspace_dls.cpp`
- Modify: `tests/test_hierarchical_qp.cpp`
- Modify: `src/velocity_ik.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: common `IArmVelocityIk`, builder, validation, and hierarchical solver.
- Produces: `HierarchicalQpIk7`, `NullspaceDlsIk7`, and a complete `makeArmVelocityIk` factory.

- [ ] **Step 1: Write DLS formula and uniform scaling tests**

```cpp
TEST(NullspaceDlsIk, MatchesDampedNullspaceFormulaInsideBounds) {
  const ArmIkInput input = fullRankInputWithWideBounds();
  NullspaceDlsIk7 solver(dlsConfig());
  const ArmIkResult result = solver.solve(input);
  const Eigen::Matrix<double, 6, 6> inverse_term =
      (input.jacobian * input.jacobian.transpose() +
       dlsConfig().damping * dlsConfig().damping *
           Eigen::Matrix<double, 6, 6>::Identity())
          .inverse();
  const Eigen::Matrix<double, 7, 6> pinv =
      input.jacobian.transpose() * inverse_term;
  const Vec7 qdot_nominal = -dlsConfig().nominal_gain *
      (input.q_measured - 0.5 *
       (input.limits.lower_position + input.limits.upper_position));
  const Vec7 expected = pinv * input.desired_twist +
      (Mat77::Identity() - pinv * input.jacobian) * qdot_nominal;
  EXPECT_TRUE(result.qdot.isApprox(expected, 1e-10));
}
```

Add tests where one bound forces `alpha < 1`, verify every nonzero component has
the same `result.qdot[i] / raw[i]`, and verify singular/zero Jacobians remain
finite.

- [ ] **Step 2: Run DLS tests and verify red**

```bash
cmake --build build --target test_nullspace_dls -j2
```

Expected: compile failure because `NullspaceDlsIk7` does not exist.

- [ ] **Step 3: Implement DLS using a solve, not a matrix inverse**

Compute:

```cpp
const Eigen::Matrix<double, 6, 6> regularized =
    input.jacobian * input.jacobian.transpose() +
    damping_squared * Eigen::Matrix<double, 6, 6>::Identity();
const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> factorization(regularized);
const Eigen::Matrix<double, 7, 6> pinv =
    input.jacobian.transpose() * factorization.solve(
        Eigen::Matrix<double, 6, 6>::Identity());
```

Reject failed/non-finite factorization. Build the null-space term, then compute
one scalar `alpha` from all asymmetric lower/upper bounds. Return:

```cpp
result.slack = input.desired_twist - input.jacobian * result.qdot;
result.equality_residual =
    (input.jacobian * result.qdot + result.slack -
     input.desired_twist).norm();
```

Here `slack.norm()` is the DLS tracking deficit, while `equality_residual` is
the residual of the shared identity `J*qdot + slack = Vd` and should remain at
roundoff. Keep these two diagnostics distinct.

- [ ] **Step 4: Write a focused hierarchical backend test before implementation**

Use a fake `IHierarchicalQpSolver` returning a known 13-vector. Assert the
backend maps the first seven values to `qdot`, the final six to slack, reports
the equality residual, and counts active bound sources correctly.

Run the test and observe compile failure because `HierarchicalQpIk7` is absent.

- [ ] **Step 5: Implement `HierarchicalQpIk7` and the factory**

The class owns `HierarchicalQpBuilder`, `IHierarchicalQpSolver`, and an
initialized flag. It validates the problem, initializes once, solves/hotstarts,
validates the result, and maps diagnostics into `ArmIkResult`.

Complete:

```cpp
std::unique_ptr<IArmVelocityIk> makeArmVelocityIk(
    IkAlgorithm algorithm, const QpIkConfig& config) {
  if (algorithm == IkAlgorithm::kNullspaceDls) {
    return std::make_unique<NullspaceDlsIk7>(config.dls);
  }
  return std::make_unique<HierarchicalQpIk7>(
      config.hierarchical_qp, config.qpoases, config.safety);
}
```

- [ ] **Step 6: Run backend tests**

```bash
cmake --build build --target test_nullspace_dls test_hierarchical_qp -j2
./build/test_nullspace_dls
./build/test_hierarchical_qp
```

Expected: all backend, formula, scaling, singularity, and mapping tests pass.

- [ ] **Step 7: Commit Task 4**

```bash
git add CMakeLists.txt include/tianji_qp_ik/hierarchical_qp_ik.hpp \
  include/tianji_qp_ik/nullspace_dls.hpp src/hierarchical_qp_ik.cpp \
  src/nullspace_dls.cpp src/velocity_ik.cpp tests/test_nullspace_dls.cpp \
  tests/test_hierarchical_qp.cpp
git commit -m "feat: add hierarchical QP and nullspace DLS backends"
```

---

### Task 5: Refactor the Dual-Arm Controller Around Explicit Reference State

**Files:**
- Modify: `include/tianji_qp_ik/controller.hpp`
- Modify: `src/controller.cpp`
- Create: `tests/test_hierarchical_controller.cpp`
- Modify: `tests/test_controller.cpp`
- Modify: `tests/test_trajectory_regression.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `makeArmVelocityIk`, common bounds, and `IArmVelocityIk` from earlier tasks.
- Produces: default hierarchical controller behavior, explicit `q_ref` state, `setAlgorithm`, and enriched arm diagnostics.

- [ ] **Step 1: Write failing reference-state and switch tests**

Create a deterministic fake backend that returns configured velocities or
failures. Add tests:

```cpp
TEST(HierarchicalController, IntegratesReferenceRatherThanMeasuredState) {
  MujocoRobot robot(modelPath());
  initializeAtMidpoint(robot);
  auto left = std::make_unique<SequenceIk>(Vec7::Constant(0.1));
  auto right = std::make_unique<SequenceIk>(Vec7::Zero());
  DualArmController controller(robot, testConfig(),
                               IkAlgorithm::kHierarchicalQp,
                               std::move(left), std::move(right));
  const Vec7 start = robot.armPosition(ArmSide::kLeft);
  ASSERT_TRUE(controller.step(currentTargets(robot), 0.005).accepted);
  ASSERT_TRUE(controller.step(currentTargets(robot), 0.005).accepted);
  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(
      start + Vec7::Constant(0.001), 1e-12));
}

TEST(HierarchicalController, AlgorithmSwitchDoesNotJumpReference) {
  const Vec7 before = controller.reference(ArmSide::kLeft);
  controller.setAlgorithm(IkAlgorithm::kNullspaceDls);
  EXPECT_TRUE(controller.reference(ArmSide::kLeft).isApprox(before));
  EXPECT_TRUE(controller.previousVelocity(ArmSide::kLeft).isZero());
}
```

Add a test where the left backend solves and the right backend fails; assert
both references and both robot positions remain unchanged and both previous
velocities are zero.

- [ ] **Step 2: Run controller tests and verify red**

```bash
cmake --build build --target test_hierarchical_controller -j2
```

Expected: compile failure because the new constructor, accessors, diagnostics,
and algorithm switch do not exist.

- [ ] **Step 3: Refactor controller state and diagnostics**

Add:

```cpp
struct ArmReferenceState {
  Vec7 q_ref{Vec7::Zero()};
  Vec7 qdot_prev{Vec7::Zero()};
  Vec6 slack_prev{Vec6::Zero()};
};
```

Extend arm diagnostics with `ArmIkResult ik`, `q_ref`, and
`reference_error_max_abs`. Provide production and injection constructors:

```cpp
DualArmController(MujocoRobot& robot, QpIkConfig config);

DualArmController(MujocoRobot& robot, QpIkConfig config,
                  IkAlgorithm algorithm,
                  std::unique_ptr<IArmVelocityIk> left,
                  std::unique_ptr<IArmVelocityIk> right);
```

Add `setAlgorithm`, `algorithm`, `reference`, and `previousVelocity`.

- [ ] **Step 4: Implement the shared step pipeline**

For each cycle:

1. call `robot.forward()` once;
2. read measured positions, TCP poses, and world Jacobians;
3. compute desired twists with the shared Cartesian servo;
4. construct shared hard bounds from measured positions;
5. call both selected backends;
6. validate both results and candidate references;
7. integrate both references only when both arms pass;
8. write both references and update previous velocity/slack;
9. on failure, retain references and clear both previous velocities.

Do not force-clip candidate references after integration; reject the cycle if a
candidate lies outside the configured margin.

- [ ] **Step 5: Migrate legacy controller tests without weakening assertions**

Use `config/qp_ik_hierarchical.yaml`, `dt = 0.005`, and the new production
constructor for convergence, unreachable, and singular tests. Keep legacy
solver unit and crosscheck tests unchanged. Adapt the failure injection to
`IArmVelocityIk` rather than `IQpSolver7`.

- [ ] **Step 6: Run controller and trajectory tests**

```bash
cmake --build build --target test_controller test_hierarchical_controller \
  test_trajectory_regression -j2
./build/test_controller
./build/test_hierarchical_controller
./build/test_trajectory_regression
```

Expected: all tests pass, reachable position error is below `0.002 m`, reachable
orientation error is below one degree, and singular/unreachable cases remain
finite and bounded.

- [ ] **Step 7: Commit Task 5**

```bash
git add CMakeLists.txt include/tianji_qp_ik/controller.hpp src/controller.cpp \
  tests/test_controller.cpp tests/test_hierarchical_controller.cpp \
  tests/test_trajectory_regression.cpp
git commit -m "feat: integrate selectable velocity IK controller"
```

---

### Task 6: Integrate Viewer Commands, Telemetry, A/B Benchmark, and Acceptance

**Files:**
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `src/telemetry.cpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Create: `apps/benchmark_hierarchical_ik.cpp`
- Modify: `CMakeLists.txt`
- Modify: `pixi.toml`
- Modify: `README.md`
- Create: `docs/verification/hierarchical_qp_ik_results.md`
- Modify: `tests/test_snapshot_exchange.cpp`

**Interfaces:**
- Consumes: controller algorithm switching and diagnostics from Task 5.
- Produces: Viewer `Q`/`D` controls, headless algorithm stages, telemetry fields, deterministic benchmark, documentation, and final verification evidence.

- [ ] **Step 1: Write failing snapshot and command tests**

Add `ViewerCommandType::kSetIkAlgorithm` and `IkAlgorithm algorithm` to commands.
Define reusable snapshot diagnostics:

```cpp
struct ArmIkSnapshot {
  double position_error{0.0};
  double orientation_error{0.0};
  double slack_position_norm{0.0};
  double slack_orientation_norm{0.0};
  double equality_residual{0.0};
  double reference_error_max_abs{0.0};
  double qdot_max_ratio{0.0};
  double solve_time_us{0.0};
  int active_position_bounds{0};
  int active_velocity_bounds{0};
  int iterations{0};
};
```

Add a snapshot-exchange test proving the newest algorithm and slack fields are
returned while older snapshots are drained.

- [ ] **Step 2: Run snapshot tests and verify red**

```bash
cmake --build build --target test_snapshot_exchange -j2
```

Expected: compile failure because the new command and snapshot fields are
missing.

- [ ] **Step 3: Integrate command handling and overlay**

Map:

```text
Q -> kSetIkAlgorithm(hierarchical_qp)
D -> kSetIkAlgorithm(nullspace_dls)
```

Before sending either switch, cancel active marker drag. In the control thread,
call `controller->setAlgorithm(command.algorithm)` without resetting target or
nominal posture. Remove `O/P` from keyboard help and the scripted headless path.
Keep `--solver qpoases` accepted by the Viewer for compatibility with existing
launch commands. Reject `--solver osqp` with the exact message
`hierarchical QP supports qpOASES only`; it must not silently select or imply an
OSQP implementation.

Display algorithm, left/right slack norms, equality residuals, active position
and velocity bound counts, and solve time in the overlay.

- [ ] **Step 4: Extend telemetry without blocking control**

Copy `ArmIkSnapshot` data into `ViewerSnapshot` and `TelemetrySample`. Extend
the CSV header and rows with explicit left/right columns. Keep the existing
preallocated SPSC queue and writer thread; do not perform file I/O in the
control loop.

- [ ] **Step 5: Update the headless acceptance sequence**

The headless sequence must observe:

1. hierarchical QP active;
2. accepted manual left target;
3. circle, figure-eight, orientation, and combined modes;
4. DLS active;
5. hierarchical QP restored;
6. pause;
7. resume to hold;
8. nominal reset.

Final output must include:

```text
algorithm=hierarchical_qp
accepted=1
control_failures=0
command_failures=0
completed_stage=10
```

- [ ] **Step 6: Add the deterministic A/B benchmark**

`benchmark_hierarchical_ik.cpp` runs both algorithms from identical copied
MuJoCo states and target sequences for central, combined, fast, near-limit,
singular, and unreachable scenarios. Report CSV and console metrics:

```text
algorithm
scenario
position_rms_m
orientation_rms_rad
settling_time_s
qdot_variation_rms
maximum_velocity_ratio
slack_position_rms
slack_orientation_rms
active_position_bounds
active_velocity_bounds
solve_p50_us
solve_p95_us
solve_p99_us
failures
```

Return nonzero if either algorithm violates hard bounds, emits non-finite data,
or if hierarchical QP P99 is not below `5000 us`. Do not require QP to beat DLS
on every tracking metric; the benchmark must report evidence, not encode the
desired conclusion.

- [ ] **Step 7: Update CMake, Pixi, README, and verification documentation**

Add build targets and Pixi tasks:

```toml
viewer = "./build/tianji_qp_ik_viewer --config config/qp_ik_hierarchical.yaml"
ik-ab = "./build/tianji_hierarchical_ik_benchmark --config config/qp_ik_hierarchical.yaml"
```

Name the benchmark CMake executable target `tianji_hierarchical_ik_benchmark`.
Change the Viewer's compiled-in default config path to
`config/qp_ik_hierarchical.yaml`; an explicit `--config` continues to override
it.

README must use plain-text formulas, describe the 13-variable QP, explain that
it is not strict lexicographic HQP, document `Q`/`D`, and state the absence of
hard acceleration/collision/dynamics constraints.

- [ ] **Step 8: Run all focused and full verification**

Run:

```bash
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_hierarchical.yaml \
  --model models/marvin_m6_qp_test.xml \
  --headless --duration 4 \
  --telemetry /tmp/tianji_hierarchical_headless.csv
./build/tianji_hierarchical_ik_benchmark \
  --config config/qp_ik_hierarchical.yaml \
  --model models/marvin_m6_qp_test.xml \
  --output /tmp/tianji_hierarchical_ab.csv
git diff --check
```

Expected:

- all CTest executables pass;
- headless reports hierarchical QP, zero control/command failures, and final
  stage completion;
- all benchmark scenarios remain finite and inside hard bounds;
- unreachable QP scenarios show nonzero slack;
- hierarchical QP solve P99 is below `5000 us`;
- `git diff --check` emits no output.

- [ ] **Step 9: Perform GUI smoke and manual acceptance where display input is available**

Run:

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_hierarchical.yaml \
  --model models/marvin_m6_qp_test.xml
```

Verify marker drag, `Q`/`D` switch continuity, overlay diagnostics, fast-drag
smoothness, unreachable slack growth, and automatic recovery. If automated
mouse input is unavailable, record that GUI launch was smoke-tested and leave
the mouse checklist explicitly pending for the user rather than claiming it
passed.

- [ ] **Step 10: Record evidence and commit Task 6**

Write exact command outputs and benchmark metrics to
`docs/verification/hierarchical_qp_ik_results.md`, then commit:

```bash
git add CMakeLists.txt pixi.toml README.md apps/run_qp_ik_viewer.cpp \
  apps/benchmark_hierarchical_ik.cpp include/tianji_qp_ik/telemetry.hpp \
  src/telemetry.cpp tests/test_snapshot_exchange.cpp \
  docs/verification/hierarchical_qp_ik_results.md
git commit -m "feat: expose hierarchical QP IK in viewer"
```

---

## Final Review Checklist

- [ ] Compare every approved design section against Tasks 1-6.
- [ ] Confirm all new behavioral tests were observed red before implementation.
- [ ] Confirm legacy solver tests and benchmark still build and pass.
- [ ] Confirm no physical bound is represented only as a cost.
- [ ] Confirm no hard acceleration limit was invented.
- [ ] Confirm world-frame error/Jacobian ordering is unchanged.
- [ ] Confirm QP and DLS share target, servo, rate, bounds, and integration.
- [ ] Confirm QP failure and one-arm failure never integrate either reference.
- [ ] Confirm no unfinished markers or debug instrumentation remain.
- [ ] Confirm branch/worktree are retained and no merge or push occurred.
