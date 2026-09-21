# Cartesian OTG + Acceleration QP Implementation Plan

> Follow-up state-semantics update (2026-08-10): the nominal acceleration QP
> now evaluates FK, `J`, `Jdot*qdot`, Cartesian feedback, posture and predictive
> bounds from the integrated command model `q_ref/qdot_ref`. Actual `q/qdot`
> remain authoritative for physical-limit supervision, tracking watchdogs,
> telemetry and explicit re-synchronization. This supersedes references below
> to measured-state nominal kinematics; see
> `docs/superpowers/specs/2026-08-10-command-model-kinematics-design.md`.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the verified Cartesian OTG velocity branch with a 7-DoF acceleration-level QP and an A/B/C controller benchmark.

**Architecture:** Reuse the exact Cartesian OTG and direct/velocity controllers. Add measured qdot and model-based `Jdot*qdot`, a Cartesian acceleration servo, a separate qddot-QP backend, and second-order reference integration. Keep acceleration control selectable so A/B/C run from one branch.

**Tech Stack:** C++17, Eigen 3.4, Ruckig 0.19.4, MuJoCo 3.6, qpOASES, yaml-cpp, GoogleTest, CMake/Pixi.

## Global Constraints

- Create this branch from the fully verified tip of `feature/cartesian-otg-velocity-qp-v1`.
- Preserve direct and OTG velocity controllers unchanged for A/B/C comparison.
- Use measured `q`/`qdot` for kinematics and reference `q_ref`/`qdot_ref` for command integration.
- Treat qddot as an upper-level shaping reference, never as a torque or certified hardware command.
- Use world-frame `J`, twist, acceleration, and SO(3) error.
- Keep qpOASES problem size at 13 variables and 6 equalities per arm.
- Do not add strict HQP, MPC, collision constraints, 14-DoF coupling, or torque control.

---

### Task 1: Expose measured velocity and model Jdot*qdot

**Files:**
- Modify: `include/tianji_qp_ik/mujoco_robot.hpp`
- Modify: `src/mujoco_robot.cpp`
- Modify: `tests/test_mujoco_robot.cpp`
- Create: `tests/test_jdot_qdot.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
void setArmState(ArmSide side, const Vec7& position, const Vec7& velocity);
Vec7 armVelocity(ArmSide side) const;
Vec6 tcpJacobianDotTimesVelocityWorld(ArmSide side,
                                      const Vec7& q,
                                      const Vec7& qdot);
```

- [ ] **Step 1: Write failing qdot mapping tests**

Set distinct velocity values for both arms, read them back through mapped DOF
addresses, and prove left/right isolation.

- [ ] **Step 2: Write failing Jdot*qdot consistency test**

For random safe configurations and velocities, compare the model result with:

```cpp
const Vec6 numeric =
    (J(q + epsilon * qdot) * qdot - J(q - epsilon * qdot) * qdot) /
    (2.0 * epsilon);
EXPECT_LT((model - numeric).norm(), 1e-5);
```

- [ ] **Step 3: Implement state mapping and directional derivative**

Allocate one scratch `mjData` and scratch Jacobian arrays in `MujocoRobot`.
Evaluate Jacobians at `q +/- epsilon*qdot`, use a centered derivative, restore
no live state because only scratch data is mutated, and multiply by qdot.

- [ ] **Step 4: Run tests and commit**

Run:

```bash
cmake --build build --target test_mujoco_robot test_jdot_qdot -j
./build/test_mujoco_robot
./build/test_jdot_qdot
```

```bash
git add CMakeLists.txt include/tianji_qp_ik/mujoco_robot.hpp \
  src/mujoco_robot.cpp tests/test_mujoco_robot.cpp tests/test_jdot_qdot.cpp
git commit -m "feat: expose acceleration kinematics"
```

### Task 2: Add acceleration-control configuration and types

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Create: `include/tianji_qp_ik/acceleration_ik.hpp`
- Create: `config/qp_ik_cartesian_otg_acceleration.yaml`
- Modify: `tests/test_config.cpp`

**Interfaces:**
- Produces `CartesianAccelerationServoConfig`, `AccelerationQpConfig`,
  `JointAccelerationLimitConfig`, `ArmAccelerationInput`, and
  `ArmAccelerationResult`.

- [ ] **Step 1: Write failing config tests**

Assert exact initial values:

```cpp
EXPECT_EQ(config.control_level, ControlLevel::kAcceleration);
EXPECT_DOUBLE_EQ(config.cartesian_acceleration.kp_position, 100.0);
EXPECT_DOUBLE_EQ(config.cartesian_acceleration.kd_position, 20.0);
EXPECT_DOUBLE_EQ(config.cartesian_acceleration.linear_limit, 6.0);
EXPECT_DOUBLE_EQ(config.acceleration_qp.slack_linear_scale, 5.0);
EXPECT_DOUBLE_EQ(config.acceleration_qp.slack_angular_scale, 15.0);
```

- [ ] **Step 2: Define exact acceleration interfaces**

```cpp
struct ArmAccelerationInput {
  Vec7 q_measured;
  Vec7 qdot_measured;
  Vec7 q_ref;
  Vec7 qdot_ref;
  Vec7 qddot_previous;
  Mat67 jacobian;
  Vec6 jdot_qdot;
  Vec6 desired_acceleration;
  ArmLimits limits;
  JointAccelerationBounds bounds;
  double dt{0.005};
};

struct ArmAccelerationResult {
  SolverStatus status{SolverStatus::kInvalidInput};
  Vec7 qddot{Vec7::Zero()};
  Vec6 slack{Vec6::Zero()};
  double equality_residual{0.0};
  double solve_time_us{0.0};
  int iterations{0};
};
```

- [ ] **Step 3: Parse and validate the profile**

Reject non-positive gains, derivative limits, slack scales, qddot limits, jerk
limits, or watchdog ordering. Existing velocity profiles default to velocity
control.

- [ ] **Step 4: Run tests and commit**

```bash
cmake --build build --target test_config -j && ./build/test_config
git add include/tianji_qp_ik/config.hpp include/tianji_qp_ik/acceleration_ik.hpp \
  src/config.cpp config/qp_ik_cartesian_otg_acceleration.yaml tests/test_config.cpp
git commit -m "feat: configure acceleration-level QP"
```

### Task 3: Implement Cartesian acceleration servo

**Files:**
- Create: `include/tianji_qp_ik/cartesian_acceleration_servo.hpp`
- Create: `src/cartesian_acceleration_servo.cpp`
- Create: `tests/test_cartesian_acceleration_servo.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
Vec6 cartesianAccelerationCommand(
    const CartesianAccelerationServoConfig& config,
    const CartesianReference& reference,
    const Pose& measured_pose,
    const Vec6& measured_twist);
```

- [ ] **Step 1: Write failing feedforward/PD/frame tests**

Verify:

```cpp
A_des = A_ref + Kd*(V_ref - V_measured) + Kp*pose_error
```

with independent translation/orientation gains and post-composition norm
limits. Include zero-error feedforward and world-frame rotation cases.

- [ ] **Step 2: Implement and run green**

Run: `cmake --build build --target test_cartesian_acceleration_servo -j && ./build/test_cartesian_acceleration_servo`

- [ ] **Step 3: Commit**

```bash
git add CMakeLists.txt include/tianji_qp_ik/cartesian_acceleration_servo.hpp \
  src/cartesian_acceleration_servo.cpp tests/test_cartesian_acceleration_servo.cpp
git commit -m "feat: add Cartesian acceleration servo"
```

### Task 4: Build qddot hard bounds

**Files:**
- Create: `include/tianji_qp_ik/acceleration_bounds.hpp`
- Create: `src/acceleration_bounds.cpp`
- Create: `tests/test_acceleration_bounds.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
JointAccelerationBounds computeJointAccelerationBounds(
    const Vec7& q_measured, const Vec7& q_ref, const Vec7& qdot_ref,
    const Vec7& qddot_previous, const ArmLimits& limits,
    const JointAccelerationLimitConfig& config, double dt);
```

- [ ] **Step 1: Write one failing test per bound source**

Cover configured qddot, predicted qdot, second-order predicted position, soft
jerk disabled, hard jerk enabled, braking envelope, and infeasible intersection.

- [ ] **Step 2: Implement exact intersections**

Use:

```cpp
qddot_velocity_lower = (qdot_min - qdot_ref) / dt;
qddot_position_lower = 2.0 * (q_min_safe - q_ref - qdot_ref*dt) / (dt*dt);
qddot_jerk_lower = qddot_previous - jerk_max*dt;
qddot_brake_upper = (sqrt(2*a_brake*distance_upper) - qdot_ref) / dt;
```

Take the max of lower sources and min of upper sources without silently
dodging an infeasible hard intersection.

- [ ] **Step 3: Run tests and commit**

```bash
cmake --build build --target test_acceleration_bounds -j && ./build/test_acceleration_bounds
git add CMakeLists.txt include/tianji_qp_ik/acceleration_bounds.hpp \
  src/acceleration_bounds.cpp tests/test_acceleration_bounds.cpp
git commit -m "feat: constrain acceleration references"
```

### Task 5: Implement the acceleration slack QP and qpOASES backend

**Files:**
- Create: `include/tianji_qp_ik/acceleration_qp.hpp`
- Create: `src/acceleration_qp.cpp`
- Create: `include/tianji_qp_ik/acceleration_qpoases_solver.hpp`
- Create: `src/acceleration_qpoases_solver.cpp`
- Create: `tests/test_acceleration_qp.cpp`
- Create: `tests/test_acceleration_qpoases_solver.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces `AccelerationQpProblem`, `AccelerationQpBuilder`, and
  `AccelerationQpoasesSolver::solve`.

- [ ] **Step 1: Write failing builder tests**

Check Hessian diagonals, linear jerk/posture terms, equality
`[J I]x=A_des-Jdot_qdot`, qddot bounds, unbounded slack, and positive-definite
validation.

- [ ] **Step 2: Implement builder**

Use normalized slack diagonal:

```cpp
wp / (a_scale*a_scale)
wR / (alpha_scale*alpha_scale)
```

and qddot linear term:

```cpp
-jerk_weight*qddot_previous - posture_weight*qddot_posture
```

- [ ] **Step 3: Write solver tests**

Cover init, hotstart, equality residual, active bounds, infeasible input,
non-finite input, and reset after native failure.

- [ ] **Step 4: Implement qpOASES wrapper and run green**

Run:

```bash
cmake --build build --target test_acceleration_qp \
  test_acceleration_qpoases_solver -j
./build/test_acceleration_qp
./build/test_acceleration_qpoases_solver
```

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt include/tianji_qp_ik/acceleration_* \
  src/acceleration_* tests/test_acceleration_*
git commit -m "feat: solve acceleration-level slack QP"
```

### Task 6: Add the dual-arm acceleration controller

**Files:**
- Create: `include/tianji_qp_ik/acceleration_controller.hpp`
- Create: `src/acceleration_controller.cpp`
- Create: `tests/test_acceleration_controller.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces:

```cpp
class DualArmAccelerationController {
 public:
  DualArmAccelerationController(MujocoRobot&, QpIkConfig);
  AccelerationControllerDiagnostics step(const DualArmReferences&, double dt);
  void resetReferences();
};
```

- [ ] **Step 1: Write failing state-transition tests**

Cover qdot initialization, exact second-order integration, q/qdot candidate
validation, position and velocity watchdog scaling, failed-arm freeze, healthy
arm continuation, solver reset, and recovery.

- [ ] **Step 2: Implement per-arm solve flow**

For each arm: read measured q/qdot, compute FK/J/Jdotqdot, build A_des and
bounds, solve, validate, integrate candidates, run watchdog, and commit only
the accepted arm through `setArmState`.

- [ ] **Step 3: Run controller tests and inherited tests**

```bash
cmake --build build -j
ctest --test-dir build --output-on-failure
```

- [ ] **Step 4: Commit**

```bash
git add CMakeLists.txt include/tianji_qp_ik/acceleration_controller.hpp \
  src/acceleration_controller.cpp tests/test_acceleration_controller.cpp
git commit -m "feat: integrate acceleration QP references"
```

### Task 7: Expose acceleration control in viewer and telemetry

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `src/telemetry.cpp`
- Modify: `tests/test_snapshot_exchange.cpp`

- [ ] **Step 1: Write failing control-level/telemetry tests**

Prove snapshots retain qddot, acceleration slack, acceleration bounds,
qdot-reference error, controller level, and per-arm acceptance atomically.

- [ ] **Step 2: Add selectable control level**

Construct velocity or acceleration controller from config. Keep target manager
and Cartesian OTG shared. Add an explicit viewer key and headless stage for
velocity/acceleration switching with reference reinitialization and no q jump.

- [ ] **Step 3: Extend overlay and CSV**

Report qddot ratio, qddot/jerk active bounds, acceleration slack, equality
residual, q/qdot reference error, solve time, and failure reason.

- [ ] **Step 4: Run headless integration and commit**

```bash
cmake --build build --target tianji_qp_ik_viewer test_snapshot_exchange -j
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml \
  --headless --duration 4 \
  --telemetry /tmp/tianji_otg_acceleration_headless.csv
```

```bash
git add apps/run_qp_ik_viewer.cpp include/tianji_qp_ik/telemetry.hpp \
  src/telemetry.cpp tests/test_snapshot_exchange.cpp
git commit -m "feat: expose acceleration QP control"
```

### Task 8: Extend benchmark to A/B/C

**Files:**
- Modify: `apps/benchmark_cartesian_otg.cpp`
- Modify: `tests/test_cartesian_otg_benchmark.cpp`

- [ ] **Step 1: Add a failing C-controller benchmark test**

Require rows for direct velocity, OTG velocity, and OTG acceleration across all
existing scenarios. Assert finite metrics, respected hard bounds, and zero
acceleration-QP failures.

- [ ] **Step 2: Add acceleration metrics**

Record RMS/peak qddot and jerk, acceleration slack, q/qdot reference errors,
active qddot/qdot/q bounds, solve p99, phase lag, and tracking metrics.

- [ ] **Step 3: Run benchmark and commit**

```bash
cmake --build build --target tianji_cartesian_otg_benchmark \
  test_cartesian_otg_benchmark -j
./build/test_cartesian_otg_benchmark
./build/tianji_cartesian_otg_benchmark \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml \
  --output /tmp/tianji_otg_acceleration_abc.csv
```

```bash
git add apps/benchmark_cartesian_otg.cpp tests/test_cartesian_otg_benchmark.cpp
git commit -m "test: compare velocity and acceleration QP"
```

### Task 9: Final acceleration-branch verification and documentation

**Files:**
- Modify: `README.md`
- Create: `docs/verification/cartesian_otg_acceleration_qp_results.md`

- [ ] **Step 1: Document mathematical and plant limitations**

Document qddot QP equations, Jdotqdot method, q/qdot reference separation,
watchdogs, commands, A/B/C metrics, and that the kinematic qdot harness is not
an actuator-dynamics validation.

- [ ] **Step 2: Run complete verification**

```bash
pixi run configure
pixi run build
pixi run test
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml \
  --headless --duration 4 \
  --telemetry /tmp/tianji_otg_acceleration_final.csv
./build/tianji_cartesian_otg_benchmark \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml \
  --output /tmp/tianji_otg_acceleration_abc.csv
```

Expected: all tests pass; headless has zero command/control failures and
deadline misses; A/B/C has finite metrics, hard bounds respected, and zero
acceleration-QP failures.

- [ ] **Step 3: Record exact results and commit**

```bash
git add README.md docs/verification/cartesian_otg_acceleration_qp_results.md
git commit -m "docs: record acceleration QP results"
```
