# MuJoCo C++ QP-IK V1 Design

Date: 2026-08-09

## 1. Goal

Build a reproducible C++17 prototype that validates velocity-level constrained QP inverse kinematics for both seven-degree-of-freedom arms in the Marvin M6-S-CCS-696 V4 model.

The complete control computation must target 1000 Hz. Rendering runs independently at approximately 60 Hz. The prototype compares OSQP and qpOASES on identical QPs and selects the faster backend only after both pass the same correctness checks.

## 2. Scope

V1 includes:

- CMake and Pixi based C++17 project; no Python runtime or test dependency.
- A loadable MuJoCo model derived from the supplied URDF and STL assets.
- Explicit name-based mappings for `Joint1_L` through `Joint7_L`, `Joint1_R` through `Joint7_R`, and both TCPs.
- World-frame TCP forward kinematics and 6-by-7 Jacobians.
- Central finite-difference validation of translational and rotational Jacobians.
- Independent left and right seven-variable velocity QPs.
- OSQP and qpOASES backends with persistent solver objects and warm starts.
- Joint-position and joint-velocity hard constraints.
- SO(3) logarithm orientation error, nominal-posture objective, Cartesian speed limits, and safe hold.
- A dual-arm GLFW viewer with scripted trajectories and mouse interaction.
- Headless regression tests and repeatable solver/control-loop benchmarks.

V1 excludes dynamics integration, MuJoCo actuators, PD gains, acceleration constraints without manufacturer data, dynamics identification, collision constraints, HQP/WBC, Pico input, and real-robot transport. Those are later phases.

## 3. Repository Layout

```text
TJ_arm_control/
├── CMakeLists.txt
├── pixi.toml
├── pixi.lock
├── marvin_m6_ccs/             # original URDF and STL assets, unchanged
├── models/                    # local URDF derivative and runtime MJCF
├── config/qp_ik.yaml
├── include/tianji_qp_ik/
├── src/
├── apps/                      # inspect_model, viewer, benchmark
├── tests/                     # GoogleTest tests
└── docs/
```

Generated build products, Pixi environments, runtime logs, benchmark CSV files, and temporary model-conversion outputs are ignored. The validated runtime MJCF is source-controlled.

## 4. Model Preparation

The supplied model has 20 links and 19 joints. Each arm has exactly seven revolute joints with matching limits. Both TCPs are fixed 0.095 m beyond Link7 using the supplied fixed-joint transform. All 20 referenced STL files exist.

The original files remain untouched. A local URDF derivative replaces `package://pico_body_tianji/...` mesh URIs with paths resolvable from the project. MuJoCo compiles that derivative and emits canonical MJCF. The runtime MJCF adds:

- named `tcp_L` and `tcp_R` sites at the TCP link origins;
- visible left and right mocap targets for manual interaction;
- visual-only target axes/geometries;
- no actuators in V1.

At startup the model adapter resolves every body, site, joint, qpos address, and DoF address by name. Missing names, wrong joint types, unexpected dimensions, invalid limits, or non-scalar revolute joints are fatal startup errors. No algorithm assumes contiguous or URDF-order indices.

## 5. Control Mathematics

For one arm, the decision variable is joint velocity `qdot` in R7. Current and desired TCP poses are `(p, R)` and `(p_d, R_d)` in the MuJoCo world frame.

```text
e_p = p_d - p
e_R = Log(R_d R^T)
V_d = [ clamp_norm(Kp e_p, v_max),
        clamp_norm(Kr e_R, w_max) ]
```

Euler-angle subtraction is prohibited. The SO(3) logarithm handles small angles with a series expansion and angles near pi with a numerically stable branch.

The objective is:

```text
min 0.5 ||Wt (J qdot - V_d)||^2
  + 0.5 lambda ||qdot||^2
  + 0.5 w_nom ||qdot - qdot_nom||^2

qdot_nom = -K_nom (q - q_nom)
H = J^T Wt^2 J + (lambda + w_nom) I
g = -J^T Wt^2 V_d - w_nom qdot_nom
```

`H` is symmetrized before solving. Positive regularization makes it positive definite within numerical tolerance.

The hard bounds are computed inside the QP:

```text
lb = max(-velocity_limit,
         (q_min + margin - q) / dt)
ub = min(+velocity_limit,
         (q_max - margin - q) / dt)
```

Post-solve clipping is not the primary limit mechanism. A tiny tolerance-only normalization may remove floating-point residue after the result has passed the constraint check; any material violation causes hold.

## 6. Components and Interfaces

- `MujocoModel`: owns `mjModel`, validates names and exposes immutable arm mappings and limits.
- `MujocoKinematics`: owns control `mjData`; performs one `mj_forward` per cycle, reads TCP poses, and extracts only the name-mapped Jacobian columns.
- `PoseServo`: computes SO(3)/position errors and saturated desired world-frame twist.
- `QpBuilder`: produces a fixed-size `QpProblem7 {H, g, lb, ub}` without heap allocation.
- `IQpSolver7`: common `initialize`, `solve`, `reset`, and diagnostics contract.
- `OsqpSolver7`: fixed full upper-triangular Hessian sparsity and `A=I`; updates numeric data and warm-starts the persistent workspace.
- `QpoasesSolver7`: persistent `SQProblem(7, 0)`; full-data hotstart is used because the Jacobian changes the Hessian every cycle.
- `SafetyGuard`: validates all inputs, bound feasibility, solver status, finite output, and final constraint residual.
- `DualArmController`: performs one kinematics update and two independent arm solves, then integrates accepted velocities.
- `TargetManager`: provides hold, circle, figure-eight, orientation-only, combined, and manual targets through one interface.
- `TelemetryBuffer`: preallocated single-producer/single-consumer ring for asynchronous CSV/log output.

The live viewer selects one backend through configuration. It never solves with both backends in the production control path. The benchmark executable replays the same recorded problems through both backends.

## 7. Threading and Timing

The background control thread owns control state and runs against `CLOCK_MONOTONIC` with absolute 1 ms deadlines. One cycle consists of target snapshot, `mj_forward`, both pose/Jacobian reads, both QP builds/solves, safety validation, and kinematic integration `q_next = q + qdot * dt`.

The GLFW main thread renders at display rate using a separate `mjData`. It consumes a joint-position snapshot, calls `mj_forward` on render data, and draws the scene. GUI input publishes target snapshots; it never mutates control `mjData`.

The initial implementation solves left and right QPs sequentially. Their combined timing is measured. Parallel solving is deferred unless measurement proves it necessary, because two worker threads can add scheduling jitter to such small QPs.

## 8. Viewer Interaction

The viewer shows actual TCP and desired target frames for both arms. Controls include:

- select left/right target;
- mouse translation using MuJoCo selection/perturbation utilities;
- modifier-drag target rotation;
- switch among manual, circle, figure-eight, orientation-only, and combined trajectories;
- pause/hold, reset nominal posture, and switch solver after a safe controller reset;
- on-screen pose errors, backend, solver status, cycle time, and deadline-miss count.

Scripted trajectories use simulation/control time rather than render time, so headless and viewer runs are reproducible.

## 9. Safety and Failure Behaviour

Every cycle checks finite state/target/Jacobian/QP data, `lb <= ub`, positive-definite objective within tolerance, accepted solver status, finite solution, and hard-bound residual. Any failure returns zero velocity for both arms for that cycle, records a structured reason, and requires subsequent valid cycles before motion resumes.

Target translation and rotation increments are rate-limited before reaching the Cartesian servo. Initial configurations must lie inside the configured safety margin. V1 never invents acceleration limits.

## 10. Tests

GoogleTest covers:

1. model load and exact joint/TCP name mapping;
2. limits and qpos/DoF address validity;
3. SO(3) logarithm at zero, small angle, generic rotations, near pi, and quaternion-sign-equivalent inputs;
4. central finite-difference position and rotation Jacobians for both arms over deterministic random configurations;
5. QP objective against unconstrained Eigen reference when bounds are inactive;
6. position and velocity hard bounds near every joint limit;
7. OSQP versus qpOASES solution, objective, KKT residual, and bound residual;
8. reachable, unreachable, singular, and target-return trajectories;
9. NaN, infeasible bound, forced solver failure, and safe-hold behaviour;
10. dual-arm scripted trajectory regression.

All randomized tests use printed deterministic seeds.

## 11. Benchmark and Solver Selection

Benchmarks run in release mode with logging and rendering disabled. They include warm-up followed by deterministic samples spanning nominal motion, active joint limits, singular configurations, unreachable targets, and continuous trajectories.

For each backend the report contains update time, solve time, combined solver time, complete single-arm time, complete dual-arm cycle time, iterations/working-set recalculations, failures, mean, p50, p95, p99, maximum, and deadline misses. Cold initialization is measured separately from the recurring 1000 Hz path.

Both backends must first meet correctness thresholds. Only then does lower dual-arm p99 latency choose the default; mean latency breaks a close tie. Solver settings and tolerances are recorded with results.

## 12. Acceptance Criteria

- Both arms have correct seven-joint mappings and TCP forward kinematics.
- Central finite-difference Jacobian maximum absolute error is at most `1e-5` for the validated samples.
- SO(3) tests pass without Euler-angle subtraction.
- OSQP and qpOASES satisfy hard bounds within `1e-8`, and their accepted solutions/objectives agree within configured numerical tolerances.
- Integrated joint positions stay inside safety-margin limits without post-solve clamping as the primary mechanism.
- Reachable scripted targets converge to at most 2 mm position error and 1 degree orientation error.
- Unreachable and singular cases remain finite and bounded.
- NaN, infeasible, and solver-failure cases enter safe hold.
- The measured complete dual-arm computation has p99 below 1 ms on the development machine in release mode; maximum latency and OS scheduling misses are reported separately.
- The interactive viewer and all scripted modes run without blocking the control loop.

## 13. Deferred Follow-up

The next phase may add MuJoCo actuators and realistic joint-level PD control. It must consume the same accepted `qdot`/integrated `q_cmd` interface so that actuator dynamics do not alter the validated QP formulation.
