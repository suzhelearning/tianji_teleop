# Command-model kinematics implementation plan

> Execute this plan with test-driven development. Keep each branch buildable and
> commit the shared base implementation before merging it downstream.

**Goal:** Evaluate nominal Cartesian IK at the integrated command state while
retaining actual joint feedback for safety and tracking supervision.

**Architecture:** Add a non-mutating arbitrary-state kinematics query to
`MujocoRobot`. Feed its pose and Jacobian at `q_ref` into the controller and use
`q_ref` for recursive command bounds. Continue to read `q_actual` for tracking
scaling, physical-envelope validation, and diagnostics.

**Tech stack:** C++17, Eigen, MuJoCo, GoogleTest, CMake/CTest.

---

## Task 1: Add non-mutating command-state kinematics

**Files:**

- Modify: `include/tianji_qp_ik/mujoco_robot.hpp`
- Modify: `src/mujoco_robot.cpp`
- Test: `tests/test_mujoco_robot.cpp`

1. Add a failing test that requests left-arm kinematics at a different joint
   position and verifies both the returned pose changes and the main left/right
   `qpos` and TCP poses remain unchanged.
2. Build and run only `test_mujoco_robot`; verify the test fails because the API
   is absent.
3. Add `ArmKinematicSample` and
   `MujocoRobot::armKinematicsAt(ArmSide, const Vec7&)` backed by dedicated
   scratch `mjData`. Copy the current `mjData` state, replace the requested arm
   qpos, call `mj_forward`, and compute pose/Jacobian from scratch buffers.
4. Reject non-finite requested positions without modifying main state.
5. Rebuild and run `test_mujoco_robot`; verify all cases pass.

## Task 2: Switch nominal controller inputs to `q_ref`

**Files:**

- Modify: `include/tianji_qp_ik/controller.hpp`
- Modify: `src/controller.cpp`
- Test: `tests/test_hierarchical_controller.cpp`

1. Extend the recording fake IK test to inject actual-state lag below the
   warning threshold, then assert that pose error, Jacobian, `q_measured`, and
   velocity bounds are based on the preserved `q_ref`, not lagged feedback.
2. Run the focused test and verify it fails under current actual-state
   kinematics.
3. In `step`, read `q_actual`, evaluate command-model samples at each arm's
   `q_ref`, and use model pose/Jacobian for Cartesian servo and IK.
4. Set the nominal IK position (`q_measured` compatibility field) and recursive
   joint bounds from `q_ref`.
5. Keep reference tracking scale based on `q_ref - q_actual`; preserve per-arm
   freeze behavior.
6. Run the focused hierarchical-controller tests and verify they pass.

## Task 3: Separate model and actual diagnostics and validate feedback

**Files:**

- Modify: `include/tianji_qp_ik/controller.hpp`
- Modify: `src/controller.cpp`
- Test: `tests/test_hierarchical_controller.cpp`
- Modify if required: `src/telemetry.cpp`
- Modify if required: `include/tianji_qp_ik/telemetry.hpp`

1. Add failing assertions that diagnostics expose `q_actual`, `tcp_actual`, and
   command-model `current` independently under injected lag.
2. Add the diagnostic fields while retaining `current` as the command-model TCP
   pose for compatibility with existing consumers.
3. Add an actual joint physical-envelope check before solving. An arm with
   non-finite or out-of-model actual position must hold without changing
   `q_ref`; command-safe margin remains enforced on the integrated candidate.
4. If telemetry serializes controller diagnostics directly, append explicit
   model/actual tracking columns without renaming existing columns.
5. Run controller, telemetry, and trajectory tests.

## Task 4: Verify and commit the base implementation

1. Run formatting/static diff checks:

   ```bash
   git diff --check
   cmake --build build -j2
   ctest --test-dir build --output-on-failure
   ```

2. Run the headless acceptance sequence:

   ```bash
   ./build/tianji_qp_ik_viewer \
     --config config/qp_ik_hierarchical.yaml \
     --model models/marvin_m6_qp_test.xml \
     --headless --duration 4 \
     --telemetry /tmp/tianji_command_model_base.csv
   ```

3. Confirm zero control/command failures and a clean working tree after
   committing the implementation.

## Task 5: Propagate to Cartesian velocity OTG branch

**Worktree:**
`/home/zj/current_robotics/TJ_arm/TJ_arm_control_cartesian_otg_velocity_qp_v1`

1. Merge the verified base branch into
   `feature/cartesian-otg-velocity-qp-v1` without rebasing or changing
   worktrees.
2. Resolve only semantic extensions required by the velocity branch and rerun
   its full suite.
3. Add a failing orientation-settling regression that reproduces stale-target
   reversal/oscillation.
4. Replace the custom angular velocity/acceleration loop with tangent-space
   Ruckig position-interface tracking. Add a bounded settle condition that
   snaps only when orientation error, angular velocity, and angular acceleration
   are all below configured tolerances.
5. Verify orientation reference settles without repeated sign reversals, then
   commit the branch independently.

## Task 6: Propagate to Cartesian acceleration QP branch

**Worktree:**
`/home/zj/current_robotics/TJ_arm/TJ_arm_control_cartesian_otg_acceleration_qp_v1`

1. Merge the verified velocity branch into
   `feature/cartesian-otg-acceleration-qp-v1`.
2. Add failing tests proving acceleration mode evaluates `J` and
   `Jdot*qdot_ref` at `q_ref/qdot_ref`, while watchdogs still use actual
   `q/qdot`.
3. Implement isolated command-model acceleration kinematics without mutating
   actual MuJoCo state.
4. Run velocity/acceleration A/B/C benchmark tests, full CTest, and headless
   acceptance; commit independently.

## Task 7: Cross-branch verification

1. Confirm all three worktrees are on their intended branches and clean.
2. Record branch tips and exact test commands.
3. Compare telemetry for model-target error, actual-model tracking error,
   solver saturation, and orientation settling. Do not claim real-hardware
   improvement from simulation alone.

