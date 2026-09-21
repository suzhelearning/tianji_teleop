# PICO Arm-Angle QP Correctness Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the incorrect elbow-only arm-angle model with an exact analytic first-order model, keep Cartesian tracking dominant, and prove one-second convergence without projection flips or misleading telemetry.

**Architecture:** MuJoCo exposes shoulder, elbow, and wrist translational Jacobians at the command-model state. `ArmAngleTaskBuilder` differentiates the complete signed arm-angle expression, including shoulder-to-wrist-axis and projected-reference motion, and contributes that continuous full row as a low-weight secondary QP objective. Cartesian slack penalties remain dominant and regression tests impose explicit end-effector error limits.

**Tech Stack:** C++20, Eigen 3.4, MuJoCo 3.10, qpOASES, GoogleTest, CMake/CTest, Python integration tests.

## Global Constraints

- Keep the existing branches and linked worktrees; create no branch or worktree.
- Do not commit or push implementation changes unless the user requests it.
- Preserve all current PICO calibration, palm-offset, axis-mapping, protocol-V2, safety-bound, watchdog, and per-arm failure behavior.
- Keep joint position, velocity, acceleration, braking, and solver constraints unchanged.
- Arm-angle error must be below `0.10 rad` after `1.0 s` for the reachable fixed-pose regression.
- End-effector position error must remain below `0.001 m` in that regression.
- No heap allocation or finite-difference kinematics may be added to the 200 Hz control-loop path.

---

### Task 1: Exact arm-angle differential geometry

**Files:**
- Modify: `include/tianji_qp_ik/mujoco_robot.hpp`
- Modify: `src/mujoco_robot.cpp`
- Modify: `include/tianji_qp_ik/arm_angle.hpp`
- Modify: `src/arm_angle.cpp`
- Test: `tests/test_mujoco_robot.cpp`
- Test: `tests/test_arm_angle.cpp`

**Interfaces:**
- `ArmKinematicSample` produces shoulder, elbow, and wrist positions plus their `Mat37` translational Jacobians.
- `ArmAngleTaskBuilder::compute(...)` produces a `Vec7` current-arm-angle Jacobian satisfying `arm_angle_rate = jacobian.dot(qdot)`.

- [x] Write a MuJoCo central-difference test for all three point Jacobians.
- [x] Write a central-difference test comparing `ArmAngleTask::jacobian` with the derivative of the complete signed arm angle.
- [x] Run the focused tests and verify that they fail because shoulder/wrist Jacobians and the complete derivative are missing.
- [x] Add shoulder/wrist body Jacobians to `ArmKinematicSample`.
- [x] Implement the analytic chain rule for normalized shoulder-wrist axis, elbow radial direction, projected reference, and `atan2` error; expose the negative error derivative as current arm-angle rate.
- [x] Delete damped-nullspace/raw-row switching and remove the now-unused nullspace-damping configuration.
- [x] Run the focused tests and verify they pass.

### Task 2: Continuous projected references

**Files:**
- Modify: `include/tianji_qp_ik/arm_angle.hpp`
- Modify: `src/arm_angle.cpp`
- Modify: `src/controller.cpp`
- Modify: `src/acceleration_controller.cpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Test: `tests/test_arm_angle.cpp`
- Test: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- `ArmAngleTaskBuilder::compute(..., double dt)` limits the selected in-plane reference by `reference_rate_limit_rad_s * dt`.
- `ArmDirectionReferenceManager::reset()` seeds world-down state so a new tracking epoch cannot jump immediately.

- [x] Write a failing test with two nearly parallel world references on opposite sides of the shoulder-wrist axis and require the projected reference change to remain rate-limited.
- [x] Write a failing epoch-reset continuity test.
- [x] Add projected-reference angular limiting and a practical degeneracy threshold before normalization.
- [x] Reset the temporal arm-reference manager when the Viewer accepts a new tracking epoch.
- [x] Run focused unit and Viewer integration tests.

### Task 3: One-second velocity and acceleration convergence

**Files:**
- Modify: `tests/test_controller.cpp`
- Modify: `tests/test_acceleration_controller.cpp`
- Modify: `config/*.yaml`

**Interfaces:**
- Existing velocity and acceleration QP scalar-task interfaces consume the exact arm-angle row without solver changes.

- [x] Change fixed-pose convergence tests to exactly 200 cycles at `dt=0.005` and assert `abs(error)<0.10`, position error `<0.001 m`, and existing elbow-height behavior.
- [x] Run both tests and verify RED with the current tuning.
- [x] Tune arm-angle weight/gain/limits and strengthen Cartesian curvature as needed; do not weaken Cartesian or joint-safety weights.
- [x] Add a changing PICO-direction regression for both control levels.
- [x] Run both controller suites and verify GREEN.

### Task 4: Truthful diagnostics

**Files:**
- Modify: `include/tianji_qp_ik/controller.hpp`
- Modify: `include/tianji_qp_ik/acceleration_controller.hpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- Diagnostics expose `arm_angle_task_active`, requested velocity in `rad/s`, and requested acceleration in `rad/s^2` as distinct values.

- [x] Add failing tests proving disabled/DLS/zero-weight tasks report inactive and acceleration is not exported as velocity.
- [x] Split requested velocity and acceleration telemetry fields and report final solver task activation.
- [x] Run focused tests and verify GREEN.

### Task 5: Cross-repository verification

**Files:**
- Modify: `docs/superpowers/specs/2026-08-11-pico-arm-angle-qp-design.md`

**Interfaces:**
- The updated design records the approved accurate weighted-secondary-task behavior and removes the strict-nullspace/raw-fallback contradiction.

- [x] Update the design formulas, continuity rules, telemetry units, and one-second acceptance evidence.
- [x] Build the Viewer/QP project and run all CTest targets.
- [x] Build and run the full `pico_bridge` test suite.
- [x] Run a 10-second synthetic PICO headless test and record p99, deadline misses, control failures, Cartesian error, and arm-angle error.
- [x] Run `git diff --check` in both worktrees and inspect final status without committing or pushing.
