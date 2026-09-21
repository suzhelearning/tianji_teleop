# MuJoCo C++ QP-IK V1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and validate a 1000 Hz C++17 dual-arm velocity-level QP-IK prototype in MuJoCo, then compare OSQP and qpOASES on identical workloads.

**Architecture:** A single controller builds fixed-size seven-variable bound-constrained QPs for each arm. Persistent OSQP and qpOASES adapters consume the same `QpProblem7`; the live viewer selects one adapter while a headless benchmark replays identical inputs through both. The control thread owns MuJoCo state at 1000 Hz and publishes snapshots to an independent GLFW render thread.

**Tech Stack:** C++17, CMake, Ninja, Pixi, MuJoCo C API, Eigen, libosqp C API, qpOASES, yaml-cpp, GLFW, GoogleTest.

## Global Constraints

- The complete dual-arm compute path targets 1000 Hz and a release-mode p99 below 1 ms on the development machine.
- Rendering runs independently at approximately 60 Hz and never blocks the control thread.
- Python and ProxSuite are not project, runtime, test, or benchmark dependencies.
- The original `marvin_m6_ccs` URDF and STL assets remain unchanged.
- Left and right arms use separate seven-variable QPs; no coupled 14-variable QP is introduced.
- Orientation error uses a world-frame SO(3) logarithm; Euler-angle subtraction is prohibited.
- Joint position and velocity limits are hard QP bounds; post-solve clipping is not the primary limit mechanism.
- Solver objects are initialized once, updated, and warm-started.
- NaN, infeasible bounds, rejected solver status, or material bound violation produces zero-velocity safe hold.
- No dynamics, actuator, PD, acceleration-limit, collision, HQP/WBC, Pico, or real-robot integration is included.

## File Map

- `pixi.toml`: reproducible C++ dependencies and build/test/view/benchmark tasks.
- `CMakeLists.txt`: project targets, dependency discovery, tests, and warning policy.
- `cmake/FindqpOASES.cmake`: imports the Pixi qpOASES library when no package config is exported.
- `models/marvin_m6_qp_test.xml`: validated runtime MJCF with TCP sites and target bodies.
- `config/qp_ik.yaml`: controller, QP, timing, safety, trajectory, and solver settings.
- `include/tianji_qp_ik/types.hpp`: fixed-size math and result types.
- `include/tianji_qp_ik/config.hpp`, `src/config.cpp`: typed YAML configuration.
- `include/tianji_qp_ik/so3.hpp`, `src/so3.cpp`: SO(3) logarithm and pose error.
- `include/tianji_qp_ik/mujoco_robot.hpp`, `src/mujoco_robot.cpp`: model ownership, mappings, FK, and Jacobians.
- `include/tianji_qp_ik/qp_builder.hpp`, `src/qp_builder.cpp`: objective and hard-bound construction.
- `include/tianji_qp_ik/qp_solver.hpp`: common solver contract.
- `include/tianji_qp_ik/osqp_solver.hpp`, `src/osqp_solver.cpp`: persistent OSQP backend.
- `include/tianji_qp_ik/qpoases_solver.hpp`, `src/qpoases_solver.cpp`: persistent qpOASES backend.
- `include/tianji_qp_ik/safety.hpp`, `src/safety.cpp`: validation and hold policy.
- `include/tianji_qp_ik/target_manager.hpp`, `src/target_manager.cpp`: scripted/manual target generation.
- `include/tianji_qp_ik/controller.hpp`, `src/controller.cpp`: one-cycle dual-arm orchestration and integration.
- `include/tianji_qp_ik/telemetry.hpp`, `src/telemetry.cpp`: bounded asynchronous diagnostics.
- `apps/inspect_model.cpp`: mapping and model report.
- `apps/run_qp_ik_viewer.cpp`: GLFW viewer and mouse/keyboard input.
- `apps/benchmark_solver.cpp`: deterministic backend and full-cycle timings.
- `tests/*.cpp`: focused GoogleTest suites.
- `README.md`: build, controls, tests, benchmark interpretation, and limitations.

---

### Task 1: Reproducible C++ Build and Loadable Model

**Files:**
- Create: `.gitignore`
- Create: `pixi.toml`
- Create: `CMakeLists.txt`
- Create: `cmake/FindqpOASES.cmake`
- Create: `apps/inspect_model.cpp`
- Create: `models/marvin_m6_qp_test.xml`
- Modify: `marvin_m6_ccs/README.md`

**Interfaces:**
- Consumes: supplied URDF/STL assets.
- Produces: `tianji_inspect_model <model-path>` and CMake targets linked to MuJoCo, Eigen, libosqp, qpOASES, yaml-cpp, GLFW, and GoogleTest.

- [x] **Step 1: Add the Pixi and CMake manifests**

Use only C/C++ packages in `pixi.toml`:

```toml
[workspace]
name = "tj-arm-control"
channels = ["conda-forge"]
platforms = ["linux-64"]

[dependencies]
cmake = ">=3.24"
ninja = ">=1.11"
cxx-compiler = ">=1.9"
libmujoco = ">=3.6,<4"
glfw = ">=3.4,<4"
eigen = ">=3.4,<4"
libosqp = ">=1.0,<2"
qpoases = ">=3.2,<4"
yaml-cpp = ">=0.8,<1"
gtest = ">=1.15,<2"

[tasks]
configure = "cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release"
build = "cmake --build build --parallel"
test = "ctest --test-dir build --output-on-failure"
inspect = "./build/tianji_inspect_model models/marvin_m6_qp_test.xml"
viewer = "./build/tianji_qp_ik_viewer --config config/qp_ik.yaml"
benchmark = "./build/tianji_qp_ik_benchmark --config config/qp_ik.yaml"
```

- [x] **Step 2: Install and prove all C/C++ dependencies are discoverable**

Run:

```bash
pixi install
pixi run cmake --version
```

Expected: dependency solve succeeds and CMake prints a version at least 3.24.

- [x] **Step 3: Compile the supplied MuJoCo URDF and create canonical MJCF**

First test the supplied `_mujoco.urdf` with `mj_loadXML`. If its `package://` names fail, mechanically create a local derivative under `models/` that retains all transforms/inertials/limits and replaces each mesh URI by the matching path under `../marvin_m6_ccs/meshes/`. Save the compiled form with `mj_saveLastXML`, then add:

```xml
<site name="tcp_L" pos="0 0 0" size="0.008" rgba="1 0.2 0.2 1"/>
<site name="tcp_R" pos="0 0 0" size="0.008" rgba="0.2 0.4 1 1"/>
<body name="target_L" mocap="true"><geom type="sphere" size="0.018" rgba="1 0.2 0.2 0.45" contype="0" conaffinity="0"/></body>
<body name="target_R" mocap="true"><geom type="sphere" size="0.018" rgba="0.2 0.4 1 0.45" contype="0" conaffinity="0"/></body>
```

Attach each TCP site to the corresponding TCP body and each mocap target directly under `worldbody`.

- [x] **Step 4: Add a model smoke executable and run it**

The executable must load with:

```cpp
char error[1024]{};
mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
if (!model) {
  std::cerr << error << '\n';
  return 1;
}
std::cout << "nq=" << model->nq << " nv=" << model->nv
          << " njnt=" << model->njnt << '\n';
```

Run `pixi run configure && pixi run build && pixi run inspect`.

Expected: model loads, `nq=14`, `nv=14`, and all 14 named arm joints and both TCP sites are present.

- [x] **Step 5: Commit the environment and assets**

```bash
git add .gitignore pixi.toml pixi.lock CMakeLists.txt cmake apps/inspect_model.cpp models marvin_m6_ccs
git commit -m "build: add reproducible C++ MuJoCo project"
```

### Task 2: Fixed Types, Configuration, and SO(3) Math

**Files:**
- Create: `include/tianji_qp_ik/types.hpp`
- Create: `include/tianji_qp_ik/config.hpp`
- Create: `src/config.cpp`
- Create: `include/tianji_qp_ik/so3.hpp`
- Create: `src/so3.cpp`
- Create: `config/qp_ik.yaml`
- Create: `tests/test_so3.cpp`
- Create: `tests/test_config.cpp`

**Interfaces:**
- Produces: `Pose`, `Vec7`, `Mat67`, `Mat77`, `QpProblem7`, `SolverResult7`, `QpIkConfig`, `so3Log`, and `poseErrorWorld`.

- [x] **Step 1: Write failing SO(3) and configuration tests**

Tests must assert:

```cpp
EXPECT_TRUE(so3Log(Eigen::Matrix3d::Identity()).isZero(1e-14));
EXPECT_NEAR(so3Log(Eigen::AngleAxisd(1e-9, Eigen::Vector3d::UnitX()).toRotationMatrix()).x(), 1e-9, 1e-12);
EXPECT_NEAR(so3Log(Eigen::AngleAxisd(M_PI - 1e-7, axis).toRotationMatrix()).norm(), M_PI - 1e-7, 1e-7);
EXPECT_DOUBLE_EQ(loadConfig(path).controller.rate_hz, 1000.0);
EXPECT_THROW(loadConfig(missing_required_path), std::runtime_error);
```

Run `pixi run build && ctest --test-dir build -R 'so3|config' --output-on-failure`.

Expected: compilation fails because the interfaces do not exist.

- [x] **Step 2: Define fixed-size shared types**

Use:

```cpp
using Vec7 = Eigen::Matrix<double, 7, 1>;
using Mat67 = Eigen::Matrix<double, 6, 7>;
using Mat77 = Eigen::Matrix<double, 7, 7>;
struct Pose { Eigen::Vector3d position; Eigen::Matrix3d rotation; };
struct QpProblem7 { Mat77 H; Vec7 g; Vec7 lower; Vec7 upper; };
enum class SolverStatus { kSolved, kMaxIterations, kInfeasible, kNumericalError, kInvalidInput };
struct SolverResult7 { SolverStatus status; Vec7 qdot; int iterations; double solve_time_us; };
```

- [x] **Step 3: Implement robust SO(3) logarithm and typed YAML parsing**

`so3Log` must clamp the trace-derived cosine, use the skew-vector series near zero, and use the dominant diagonal axis branch near pi. `loadConfig` validates positive rate/gains/weights, `0 < velocity_scale <= 1`, positive margins and finite values.

- [x] **Step 4: Run focused and full tests**

Run `pixi run build && pixi run test`.

Expected: SO(3) and configuration tests pass.

- [x] **Step 5: Commit**

```bash
git add config include/tianji_qp_ik/types.hpp include/tianji_qp_ik/config.hpp include/tianji_qp_ik/so3.hpp src/config.cpp src/so3.cpp tests/test_so3.cpp tests/test_config.cpp CMakeLists.txt
git commit -m "feat: add QP IK types configuration and SO3 math"
```

### Task 3: MuJoCo Name Mapping, FK, and Jacobian Validation

**Files:**
- Create: `include/tianji_qp_ik/mujoco_robot.hpp`
- Create: `src/mujoco_robot.cpp`
- Create: `tests/test_mujoco_robot.cpp`
- Create: `tests/test_jacobian.cpp`
- Modify: `apps/inspect_model.cpp`

**Interfaces:**
- Consumes: `Pose`, `Vec7`, and `Mat67`.
- Produces: `ArmSide`, `ArmMapping`, `MujocoRobot::setArmPosition`, `armPosition`, `tcpPose`, `tcpJacobianWorld`, and `forward`.

- [x] **Step 1: Write failing mapping and Jacobian tests**

For each side assert the exact names, revolute joint type, scalar qpos/DoF widths, finite limits, and unique addresses. For 20 deterministic random configurations compare analytic columns against:

```cpp
const Eigen::Vector3d dp = (plus.position - minus.position) / (2.0 * eps);
const Eigen::Vector3d dw = so3Log(plus.rotation * minus.rotation.transpose()) / (2.0 * eps);
EXPECT_LT((analytic.topRows<3>().col(i) - dp).cwiseAbs().maxCoeff(), 1e-5);
EXPECT_LT((analytic.bottomRows<3>().col(i) - dw).cwiseAbs().maxCoeff(), 1e-5);
```

Run `ctest --test-dir build -R 'mujoco_robot|jacobian' --output-on-failure`.

Expected: compilation fails because `MujocoRobot` does not exist.

- [x] **Step 2: Implement RAII model/data ownership and explicit mapping**

Resolve IDs with `mj_name2id`; derive qpos/DoF addresses from `jnt_qposadr` and `jnt_dofadr`; copy `jnt_range`; reject missing/duplicate/wrong-type entries. Call `mj_forward` after state updates.

- [x] **Step 3: Implement FK and analytic world-frame site Jacobian**

Read site `xpos` and row-major `xmat`. Allocate 3-by-`nv` scratch matrices once, call `mj_jacSite`, and copy only mapped DoF columns into `[Jp; Jr]`.

- [x] **Step 4: Run finite differences before any QP implementation**

Run `pixi run build && ctest --test-dir build -R 'mujoco_robot|jacobian' --output-on-failure`.

Expected: all mapping and both-arm finite-difference cases pass with maximum absolute error at most `1e-5`.

- [x] **Step 5: Commit**

```bash
git add include/tianji_qp_ik/mujoco_robot.hpp src/mujoco_robot.cpp tests/test_mujoco_robot.cpp tests/test_jacobian.cpp apps/inspect_model.cpp CMakeLists.txt
git commit -m "feat: validate MuJoCo kinematics and Jacobians"
```

### Task 4: QP Objective, Bounds, and Safety Policy

**Files:**
- Create: `include/tianji_qp_ik/qp_builder.hpp`
- Create: `src/qp_builder.cpp`
- Create: `include/tianji_qp_ik/safety.hpp`
- Create: `src/safety.cpp`
- Create: `tests/test_qp_builder.cpp`
- Create: `tests/test_safety.cpp`

**Interfaces:**
- Produces: `QpBuilder::build(J, desired_twist, q, limits, dt) -> QpProblem7` and `SafetyGuard::validateProblem/validateResult`.

- [x] **Step 1: Write failing objective and bound tests**

Tests must compare `H/g` against the expanded weighted least-squares formula, verify symmetry and positive eigenvalues, and assert for every joint:

```cpp
EXPECT_DOUBLE_EQ(problem.lower[i], std::max(-vmax[i], (qmin[i] + margin - q[i]) / dt));
EXPECT_DOUBLE_EQ(problem.upper[i], std::min( vmax[i], (qmax[i] - margin - q[i]) / dt));
```

Safety tests inject NaN, `lower > upper`, non-symmetric/indefinite Hessian, rejected status, and an out-of-bound result.

- [x] **Step 2: Implement the minimal fixed-size builder**

Use `H.noalias() = J.transpose() * W2 * J`, add diagonal regularization and nominal weight, compute `g`, and derive bound intersections. Do not clamp the solver result.

- [x] **Step 3: Implement structured validation**

Return `SafetyDecision {bool accepted; HoldReason reason;}` where reasons distinguish invalid input, infeasible bounds, invalid Hessian, solver failure, non-finite solution, and bound violation.

- [x] **Step 4: Run tests and commit**

Run `pixi run build && pixi run test`.

```bash
git add include/tianji_qp_ik/qp_builder.hpp include/tianji_qp_ik/safety.hpp src/qp_builder.cpp src/safety.cpp tests/test_qp_builder.cpp tests/test_safety.cpp CMakeLists.txt
git commit -m "feat: build constrained QP and safety checks"
```

### Task 5: Persistent OSQP Backend

**Files:**
- Create: `include/tianji_qp_ik/qp_solver.hpp`
- Create: `include/tianji_qp_ik/osqp_solver.hpp`
- Create: `src/osqp_solver.cpp`
- Create: `tests/test_osqp_solver.cpp`

**Interfaces:**
- Produces: `IQpSolver7` and `OsqpSolver7` with `initialize`, `solve`, and `reset`.

- [x] **Step 1: Write failing backend tests**

Use an unconstrained diagonal QP whose solution is `-H.inverse()*g`, then active-bound and repeated-update cases. Assert accepted OSQP statuses, residual at most `1e-8`, and that an internal setup counter remains one across repeated solves.

- [x] **Step 2: Implement a persistent OSQP 1.x workspace**

Store all CSC arrays as fixed member arrays. Use all 28 upper-triangular Hessian entries so the sparsity pattern never changes, and seven identity entries for `A`. Call `osqp_setup` once, then `osqp_update_data_mat`, `osqp_update_data_vec`, `osqp_warm_start`, and `osqp_solve`. Disable verbose output and dynamic adaptive features that create timing unpredictability; expose tolerances and maximum iterations from configuration.

- [x] **Step 3: Map every OSQP status explicitly**

Only `OSQP_SOLVED` is accepted in the default strict path. Preserve iteration count and measured update-plus-solve duration. Never return an uninitialized or previous solution after failure.

- [x] **Step 4: Run tests and commit**

Run `pixi run build && ctest --test-dir build -R osqp --output-on-failure`.

```bash
git add include/tianji_qp_ik/qp_solver.hpp include/tianji_qp_ik/osqp_solver.hpp src/osqp_solver.cpp tests/test_osqp_solver.cpp CMakeLists.txt
git commit -m "feat: add persistent OSQP backend"
```

### Task 6: Persistent qpOASES Backend and Cross-Solver Regression

**Files:**
- Create: `include/tianji_qp_ik/qpoases_solver.hpp`
- Create: `src/qpoases_solver.cpp`
- Create: `tests/test_qpoases_solver.cpp`
- Create: `tests/test_solver_crosscheck.cpp`

**Interfaces:**
- Produces: `QpoasesSolver7 : IQpSolver7` using `SQProblem(7, 0)` and the same result/status types as OSQP.

- [x] **Step 1: Write failing qpOASES and cross-check tests**

Repeat the analytic, active-bound, and Hessian-changing sequence. Generate 1000 deterministic strongly-convex problems and require both solvers to satisfy bounds within `1e-8`; compare infinity-norm solution and objective differences using configured solver tolerances.

- [x] **Step 2: Implement full-data hotstart**

Initialize once with dense row-major Hessian data and no general constraints. For later calls use the `SQProblem::hotstart(H_new, g_new, nullptr, lb_new, ub_new, nullptr, nullptr, nWSR, &cpu_time)` overload. Configure quiet MPC-oriented options, finite working-set limits, and explicit return-code mapping.

- [x] **Step 3: Verify Hessian updates really affect the answer**

The regression must solve two problems with identical `g/lb/ub` but different Hessians and assert distinct solutions matching the analytic reference. This prevents accidental use of the vector-only `QProblemB::hotstart` path.

- [x] **Step 4: Run tests and commit**

Run `pixi run build && ctest --test-dir build -R 'qpoases|solver_crosscheck' --output-on-failure`.

```bash
git add include/tianji_qp_ik/qpoases_solver.hpp src/qpoases_solver.cpp tests/test_qpoases_solver.cpp tests/test_solver_crosscheck.cpp CMakeLists.txt
git commit -m "feat: add qpOASES backend and solver cross-check"
```

### Task 7: Targets and Dual-Arm Kinematic Controller

**Files:**
- Create: `include/tianji_qp_ik/target_manager.hpp`
- Create: `src/target_manager.cpp`
- Create: `include/tianji_qp_ik/controller.hpp`
- Create: `src/controller.cpp`
- Create: `tests/test_target_manager.cpp`
- Create: `tests/test_controller.cpp`
- Create: `tests/test_trajectory_regression.cpp`

**Interfaces:**
- Produces: `TargetMode`, `TargetManager::sample(time)`, `DualArmController::step(targets, dt)`, and `ControllerDiagnostics`.

- [x] **Step 1: Write failing deterministic trajectory and controller tests**

Assert exact target values at selected times for hold, circle, figure-eight, and orientation-only modes. Controller tests cover zero error, reachable convergence, near-limit motion, unreachable target, singular posture, target return, NaN target, infeasible bounds, and injected solver failure.

- [x] **Step 2: Implement pose servo and target generation**

Targets are relative to captured initial TCP poses. Clamp translation and rotation twist norms independently. Scripted modes depend only on the passed control time.

- [x] **Step 3: Implement the complete dual-arm cycle**

One call performs one `mj_forward`, obtains both FK/Jacobians, builds and solves both QPs sequentially, validates both, and commits both integrated joint states only when the complete cycle is accepted. A failure produces zero velocity and leaves both positions unchanged.

- [x] **Step 4: Run all headless regressions and commit**

Run `pixi run build && pixi run test`.

```bash
git add include/tianji_qp_ik/target_manager.hpp include/tianji_qp_ik/controller.hpp src/target_manager.cpp src/controller.cpp tests/test_target_manager.cpp tests/test_controller.cpp tests/test_trajectory_regression.cpp CMakeLists.txt
git commit -m "feat: add dual-arm QP IK controller"
```

### Task 8: Deterministic Benchmark and Solver Selection

**Files:**
- Create: `apps/benchmark_solver.cpp`
- Create: `tests/test_benchmark_dataset.cpp`
- Modify: `config/qp_ik.yaml`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `tianji_qp_ik_benchmark --config ... --samples ... --output ...` and CSV/console statistics.

- [x] **Step 1: Write a deterministic dataset test**

Generate a fixed sequence containing nominal, active-limit, singular, unreachable, and continuous target cases. Assert identical hashes and category counts for repeated generation with the same seed.

- [x] **Step 2: Implement timing without rendering or logging in the measured region**

Use `std::chrono::steady_clock`, at least 1000 warm-up and 20000 measured samples, and sorted quantiles. Report separate setup, update, solve, update-plus-solve, single-arm, and complete dual-arm timings plus iterations, failures, residuals, and deadline misses.

- [x] **Step 3: Encode the default-solver rule**

Reject a backend from selection if any correctness case fails. Otherwise select lower dual-arm p99; use mean as tie-breaker. Print the chosen backend but keep configuration explicit rather than rewriting source files.

- [x] **Step 4: Run release benchmark and commit**

Run:

```bash
pixi run configure
pixi run build
./build/tianji_qp_ik_benchmark --config config/qp_ik.yaml --samples 20000 --output benchmark_results.csv
```

Expected: both backends have zero correctness failures; the report identifies the lower-p99 backend and reports whether full-cycle p99 is below 1000 microseconds.

```bash
git add apps/benchmark_solver.cpp tests/test_benchmark_dataset.cpp config/qp_ik.yaml CMakeLists.txt
git commit -m "perf: compare OSQP and qpOASES at 1000 Hz"
```

### Task 9: Non-Blocking Interactive MuJoCo Viewer

**Files:**
- Create: `include/tianji_qp_ik/telemetry.hpp`
- Create: `src/telemetry.cpp`
- Create: `apps/run_qp_ik_viewer.cpp`
- Create: `tests/test_snapshot_exchange.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `tianji_qp_ik_viewer`, bounded telemetry/snapshot exchange, and manual target updates.

- [x] **Step 1: Test snapshot and bounded telemetry exchange without OpenGL**

Run producer/consumer threads for deterministic sequence numbers. Assert snapshots are never torn, queue capacity is bounded, and the producer never blocks when the consumer is slow.

- [x] **Step 2: Implement absolute-deadline 1000 Hz control thread**

Use `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)` on Linux. The thread owns control `mjData` and controller objects, publishes state/diagnostics, and counts late deadlines. It performs no file or terminal I/O.

- [x] **Step 3: Implement the GLFW render/input main thread**

Create separate render `mjData`, copy the latest joint and target snapshot, call `mj_forward`, and render with MuJoCo visualization APIs. Use MuJoCo selection/perturbation helpers to translate or rotate selected mocap target bodies. Provide keys for arm selection, manual/circle/figure-eight/orientation/combined modes, pause, reset, backend reset/switch, and help.

- [x] **Step 4: Add on-screen diagnostics and graceful shutdown**

Show backend, target mode, position/rotation errors, solver status, current cycle microseconds, p99 estimate, and deadline misses. Join the control thread before freeing either model/data or terminating GLFW.

- [x] **Step 5: Build, run smoke test, and commit**

Run `pixi run build`, then run the viewer manually through every target mode and both backends. Exit normally and confirm no crash or solver failure.

```bash
git add include/tianji_qp_ik/telemetry.hpp src/telemetry.cpp apps/run_qp_ik_viewer.cpp tests/test_snapshot_exchange.cpp CMakeLists.txt
git commit -m "feat: add non-blocking MuJoCo QP IK viewer"
```

### Task 10: Documentation and Final Verification

**Files:**
- Create: `README.md`
- Create: `docs/verification/qp_ik_v1_results.md`
- Modify: `marvin_m6_ccs/README.md`

**Interfaces:**
- Consumes: all executables and tests.
- Produces: reproducible operator instructions and recorded acceptance evidence.

- [x] **Step 1: Write operator documentation**

Document Pixi setup, configure/build/test commands, exact Viewer controls, configuration fields, model provenance, solver-selection method, benchmark columns, safety behaviour, and V1 exclusions. State that all numeric limits are simulation defaults, not manufacturer-approved real-robot limits.

- [x] **Step 2: Run clean release verification**

Run:

```bash
pixi run configure
pixi run build
pixi run test
pixi run inspect
pixi run benchmark
```

Expected: build succeeds; every test passes; model maps 14 arm joints and two TCP sites; both solvers pass correctness checks; benchmark reports full-cycle p99 and a selected backend.

- [x] **Step 3: Record exact evidence**

Write compiler/library versions, test count, Jacobian maximum errors, solver residual/difference maxima, trajectory terminal errors, timing percentiles, selected backend, and any non-real-time OS scheduling caveat to `docs/verification/qp_ik_v1_results.md`.

- [x] **Step 4: Inspect repository hygiene**

Run `git status --short`, `git diff --check`, and verify no `.pixi`, `build`, caches, runtime logs, or benchmark CSV files are tracked.

- [x] **Step 5: Commit**

```bash
git add README.md marvin_m6_ccs/README.md docs/verification/qp_ik_v1_results.md
git commit -m "docs: add QP IK V1 usage and verification results"
```
