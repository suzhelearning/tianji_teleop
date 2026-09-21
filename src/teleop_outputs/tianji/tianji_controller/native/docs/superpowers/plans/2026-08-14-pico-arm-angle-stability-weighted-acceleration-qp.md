# PICO Arm-Angle Stability and Weighted Acceleration-QP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove intermittent PICO elbow-angle twitch and reduce sustained arm-angle mismatch by conditioning degenerate skeleton/reference projections and solving the acceleration-level arm-angle task inside the existing QP.

**Architecture:** Preserve the PICO relative endpoint mapping, Position OTG, Cartesian acceleration equality/slack formulation, all joint hard bounds, and the velocity-level QP. Mark nearly straight human elbow-plane measurements invalid, hold the previous live direction, condition robot-side reference projection with hysteresis, and add a configurable scalar arm-angle least-squares term to the acceleration QP.

**Tech Stack:** C++17, Eigen, MuJoCo, qpOASES, yaml-cpp, GoogleTest, ROS 2/ament, Python viewer integration tests.

## Global Constraints

- Keep `/home/zj/current_robotics/TJ_arm/TJ_arm_control` on `feature/pico-mujoco-teleop-v1`; do not switch or clean its worktree.
- Preserve untracked `benchmark_results/`; never stage or delete recorded CSV data.
- Preserve all existing changes in `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1` on `feature/tianji-mujoco-teleop-v1`; only add the minimal geometry/test delta.
- Do not change the PICO UDP packet layout, endpoint relative mapping, endpoint orientation mapping, Position OTG parameters, collision behavior, joint speed/acceleration/braking/jerk limits, or velocity-level QP.
- Use `arm_angle.reference_projection_hold_enter: 0.05`, `arm_angle.reference_projection_hold_exit: 0.10`, and `acceleration_qp.arm_angle_weight: 10.0` in the PICO profile.
- Invalid human arm direction must not invalidate either end-effector target.
- Use test-first RED-GREEN cycles for every behavior change.

---

### Task 1: Reject unobservable human elbow-plane directions

**Files:**
- Modify: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/src/pico_bridge/src/tianji_teleop_geometry.cpp`
- Test: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/src/pico_bridge/test/test_tianji_teleop_geometry.cpp`

**Interfaces:**
- Consumes: `map_pico_palms_to_tianji(const PicoSkeletonFrame&, const PicoPositionRetargetingConfig&)`.
- Produces: `PicoShoulderMapResult::{left,right}_arm_direction.valid == false` when the corresponding shoulder-elbow radial projection about the shoulder-wrist axis is below `0.02 m`, while `result.valid` and both targets remain valid.

- [ ] **Step 1: Write the failing near-straight-arm test**

Add a case that starts from `skeletonWithBentElbows()`, moves only the left elbow to a point whose radial distance from the left shoulder-wrist axis is `0.01 m`, and asserts:

```cpp
const auto result = pb::map_pico_palms_to_tianji(frame);
ASSERT_TRUE(result.valid) << result.rejection_reason;
EXPECT_FALSE(result.left_arm_direction.valid);
EXPECT_TRUE(result.right_arm_direction.valid);
EXPECT_TRUE(result.left_target.matrix().allFinite());
EXPECT_TRUE(result.right_target.matrix().allFinite());
```

- [ ] **Step 2: Run the focused test and verify RED**

Run from the PICO workspace:

```bash
colcon build --packages-select pico_bridge --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select pico_bridge --ctest-args -R test_tianji_teleop_geometry
colcon test-result --verbose
```

Expected: the new test fails because the current threshold is `1.0e-4 m` and accepts the `0.01 m` projection.

- [ ] **Step 3: Raise only the human radial confidence threshold**

Change the local geometry constant to:

```cpp
constexpr double kMinimumElbowProjectionNorm = 0.02;
```

Do not alter segment-validity thresholds or endpoint target construction.

- [ ] **Step 4: Run the focused PICO test and verify GREEN**

Run the same three commands. Expected: the geometry target passes and existing protocol/layout tests remain unaffected.

- [ ] **Step 5: Preserve the dirty PICO worktree intentionally**

Run:

```bash
git diff --check -- src/pico_bridge/src/tianji_teleop_geometry.cpp src/pico_bridge/test/test_tianji_teleop_geometry.cpp
git status --short --branch
```

Expected: no whitespace errors. Do not commit this repository because these files depend on the user's pre-existing uncommitted PICO teleop implementation; report the two touched files explicitly at handoff.

---

### Task 2: Hold live invalid directions and condition robot projection

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `include/tianji_qp_ik/arm_angle.hpp`
- Modify: `src/arm_angle.cpp`
- Modify: `src/config.cpp`
- Test: `tests/test_arm_angle.cpp`
- Test: `tests/test_config.cpp`

**Interfaces:**
- Consumes: `ArmDirectionReferenceManager::update(...)` and `ArmAngleTaskBuilder::compute(...)`.
- Produces: `ArmAngleTask::{requested_reference_projection_norm,jacobian_norm,reference_projection_held}` and an extended constructor accepting `reference_projection_hold_enter` and `reference_projection_hold_exit`.

- [ ] **Step 1: Write failing reference-manager and projection-hysteresis tests**

Replace the old invalid-PICO fallback expectation with a test that first accepts a valid PICO direction and then sends an invalid live request:

```cpp
const auto output = manager.update({}, 0.1);
EXPECT_TRUE(output.left.direction.isApprox(Eigen::Vector3d::UnitY()));
EXPECT_EQ(output.left.source, ArmDirectionReferenceSource::kPrevious);
```

Keep the stale behavior test through `selectArmDirectionReferences(..., false, ...)`, which supplies a valid default-down request and therefore still rotates toward down.

Add builder tests using enter `0.05` and exit `0.10` that verify:

```cpp
EXPECT_TRUE(held.reference_projection_held);
EXPECT_EQ(held.reference_source, ArmDirectionReferenceSource::kPrevious);
EXPECT_TRUE(held.projected_reference.isApprox(before.projected_reference));
EXPECT_LT(held.jacobian_norm, 100.0);
```

Then provide a projection between enter/exit and verify it remains held, followed by a projection above exit and verify continuous rate-limited reacquisition.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
cmake --build build -j"$(nproc)" --target test_arm_angle test_config
./build/test_arm_angle --gtest_filter='ArmDirectionReferenceManager.*:ArmAngleTaskBuilder.*'
./build/test_config --gtest_filter='Config.*'
```

Expected: invalid live input falls toward world-down, the constructor/diagnostic fields do not exist, and config values are not parsed.

- [ ] **Step 3: Add configuration and diagnostics interfaces**

Extend `ArmAngleConfig`:

```cpp
double reference_projection_hold_enter{0.05};
double reference_projection_hold_exit{0.10};
```

Extend `ArmAngleTask`:

```cpp
double requested_reference_projection_norm{0.0};
double jacobian_norm{0.0};
bool reference_projection_held{false};
```

Extend `ArmAngleTaskBuilder` constructor/state with both thresholds and `bool reference_projection_held_{false}`. Parse both YAML keys, require `0 < enter < exit <= 1`, and reset the held state in `reset()`.

- [ ] **Step 4: Implement previous-direction hold and projection hysteresis**

In `ArmDirectionReferenceManager::updateOne`, distinguish invalid live requests from stale/default behavior using validity: if the request is invalid and a valid previous value exists, return the previous normalized direction with source `kPrevious`; if no previous value exists, return world-down. Valid `kDefaultDown` requests continue through rate limiting.

In `ArmAngleTaskBuilder::compute`:

```text
raw_norm = ||requested - axis * axis.dot(requested)||
if held:
    release only when raw_norm >= hold_exit
else:
    enter hold when raw_norm < hold_enter
```

When held and previous in-plane direction exists, use that normalized direction both as the projected target and as the differentiation reference. When no previous direction exists, retain the side-specific outward fallback. On release, reuse the existing angular rate limiter. Store raw norm, held state, and final Jacobian norm in the result.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run the commands from Step 2. Expected: all selected tests pass.

- [ ] **Step 6: Commit the conditioning behavior**

```bash
git add include/tianji_qp_ik/config.hpp include/tianji_qp_ik/arm_angle.hpp src/arm_angle.cpp src/config.cpp tests/test_arm_angle.cpp tests/test_config.cpp
git diff --cached --check
git commit -m "fix: condition PICO arm-angle references"
```

---

### Task 3: Add the weighted arm-angle acceleration-QP objective

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `src/acceleration_qp.cpp`
- Test: `tests/test_acceleration_qp.cpp`
- Test: `tests/test_config.cpp`

**Interfaces:**
- Consumes: `ArmAccelerationInput::arm_angle_task` (`ScalarJointTask`).
- Produces: `AccelerationQpConfig::arm_angle_weight` and the Hessian/gradient term for active finite tasks.

- [ ] **Step 1: Replace the legacy no-effect test with failing objective tests**

For a task with `j[6] = 2`, `target = -4`, `activation = 0.5`, and weight `10`, assert the delta against the inactive baseline:

```cpp
EXPECT_NEAR(with_task.H(6, 6) - baseline.H(6, 6), 20.0, 1e-14);
EXPECT_NEAR(with_task.g[6] - baseline.g[6], 40.0, 1e-14);
```

Add separate cases proving inactive tasks and `arm_angle_weight == 0` produce bitwise-equivalent problems. Add a config test for parsing `10.0` and rejecting negative values while allowing zero as rollback.
Add a non-finite task case with a NaN target and assert that its problem is
identical to the inactive baseline.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
cmake --build build -j"$(nproc)" --target test_acceleration_qp test_config
./build/test_acceleration_qp
./build/test_config --gtest_filter='Config.*'
```

Expected: the active task does not change Hessian/gradient and `arm_angle_weight` is unavailable.

- [ ] **Step 3: Implement the QP term and config validation**

Add `double arm_angle_weight{0.0};` to `AccelerationQpConfig`, parse it from `acceleration_qp.arm_angle_weight`, and validate it as finite/non-negative. In `AccelerationQpBuilder::build`, only when weight is positive and the task is active, finite, and has positive finite activation, add:

```cpp
const double weight = config_.arm_angle_weight * input.arm_angle_task.activation;
problem.H.topLeftCorner<kArmDof, kArmDof>().noalias() +=
    weight * input.arm_angle_task.jacobian *
    input.arm_angle_task.jacobian.transpose();
problem.g.head<kArmDof>().noalias() -=
    weight * input.arm_angle_task.target * input.arm_angle_task.jacobian;
```

Non-finite tasks are ignored for the arm-angle objective instead of poisoning the Cartesian problem.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the commands from Step 2. Expected: all tests pass, including zero-weight rollback.

- [ ] **Step 5: Commit the weighted objective**

```bash
git add include/tianji_qp_ik/config.hpp src/config.cpp src/acceleration_qp.cpp tests/test_acceleration_qp.cpp tests/test_config.cpp
git diff --cached --check
git commit -m "feat: solve arm angle inside acceleration QP"
```

---

### Task 4: Consume the QP result directly and expose diagnostics

**Files:**
- Modify: `src/controller.cpp`
- Modify: `src/acceleration_controller.cpp`
- Modify: `include/tianji_qp_ik/acceleration_controller.hpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Test: `tests/test_acceleration_controller.cpp`
- Test: `tests/test_pico_viewer_integration.py`
- Test: `tests/test_config.cpp`

**Interfaces:**
- Consumes: weighted QP solution and `ArmAngleTask` conditioning diagnostics.
- Produces: requested/achieved/residual acceleration and projection-conditioning fields in controller diagnostics, snapshots, and telemetry CSV.

- [ ] **Step 1: Write failing no-post-mutation and telemetry tests**

Replace `RefinesArmAngleOnlyInCartesianNullspace` with a fixed-solver regression that returns zero acceleration and asserts:

```cpp
EXPECT_TRUE(result.left.qp.qddot.isZero(0.0));
EXPECT_DOUBLE_EQ(result.left.arm_angle_achieved_acceleration, 0.0);
EXPECT_NEAR(result.left.arm_angle_acceleration_residual,
            result.left.arm_angle_requested_acceleration, 1e-12);
```

Also assert the captured QP problem contains a nonzero arm-angle Hessian contribution. Extend the viewer integration required-column list with, for each side:

```text
arm_angle_reference_projection_norm
arm_angle_jacobian_norm
arm_angle_projection_held
arm_angle_achieved_acceleration_rad_s2
arm_angle_acceleration_residual_rad_s2
```

- [ ] **Step 2: Run focused tests and verify RED**

```bash
cmake --build build -j"$(nproc)" --target test_acceleration_controller tianji_qp_ik_viewer
./build/test_acceleration_controller --gtest_filter='AccelerationController.*ArmAngle*'
python3 tests/test_pico_viewer_integration.py --viewer ./build/tianji_qp_ik_viewer --source-dir .
```

Expected: the controller mutates the solver output in post-QP nullspace refinement and the new telemetry fields are absent.

- [ ] **Step 3: Remove only acceleration-path post-QP refinement**

For accepted acceleration solves, assign:

```cpp
arm_diagnostics.qp.qddot = solution.x.head<kArmDof>();
```

Do not call `refineArmAngleInNullspace` in `DualArmAccelerationController`. Leave `HierarchicalQpIk` and the velocity-level controller unchanged. Compute after the final accepted/fallback acceleration:

```cpp
achieved = task.active ? task.jacobian.dot(qddot) : 0.0;
residual = task.active ? task.target - achieved : 0.0;
```

- [ ] **Step 4: Wire constructor/config and telemetry fields**

Pass both projection thresholds to every `ArmAngleTaskBuilder` in acceleration and velocity controllers. Extend `ArmAccelerationDiagnostics` and `ArmIkSnapshot`, copy fields in both control-level snapshot paths, append matching CSV headers/values in the same order, and require finite values in the viewer integration test. Set the PICO profile to:

```yaml
acceleration_qp:
  arm_angle_weight: 10.0
arm_angle:
  reference_projection_hold_enter: 0.05
  reference_projection_hold_exit: 0.10
```

- [ ] **Step 5: Run focused tests and verify GREEN**

Run the commands from Step 2 plus:

```bash
./build/test_config --gtest_filter='Config.LoadsPicoTeleopProfile'
```

Expected: all focused controller, config, and viewer tests pass.

- [ ] **Step 6: Commit controller/profile/telemetry integration**

```bash
git add src/controller.cpp src/acceleration_controller.cpp include/tianji_qp_ik/acceleration_controller.hpp include/tianji_qp_ik/telemetry.hpp apps/run_qp_ik_viewer.cpp config tests/test_acceleration_controller.cpp tests/test_pico_viewer_integration.py tests/test_config.cpp
git diff --cached --check
git commit -m "feat: integrate stable PICO arm-angle acceleration control"
```

---

### Task 5: Full verification and recorded-input comparison

**Files:**
- Modify only if a verification test exposes a regression; any fix requires a new RED-GREEN cycle in the owning task.
- Verify: `benchmark_results/pico_position_otg_live.csv` remains untracked and unchanged.

**Interfaces:**
- Consumes: completed PICO bridge and Tianji controller behavior.
- Produces: fresh build/test evidence and reproducible before/after metrics.

- [ ] **Step 1: Verify the complete Tianji suite**

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -j"$(nproc)"
```

Expected: all configured tests pass with zero failures.

- [ ] **Step 2: Verify the complete PICO bridge suite**

```bash
colcon build --packages-select pico_bridge --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select pico_bridge
colcon test-result --verbose
```

Expected: zero failed tests.

- [ ] **Step 3: Replay identical recorded PICO packets**

Run the 29.802 s, 2584-frame trace against a 32 s headless Viewer:

```bash
rm -f /tmp/pico_arm_angle_weighted_replay.csv
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_pico_fast.xml \
  --headless --duration 32 \
  --pico-teleop --pico-bind 127.0.0.1 --pico-port 15000 \
  --control-level acceleration \
  --telemetry /tmp/pico_arm_angle_weighted_replay.csv &
viewer_pid=$!
python3 /tmp/replay_pico_udp_trace.py \
  --input /tmp/pico_fast_motion_20260812_205428.tjvr \
  --port 15000 --lead 0.5
wait "$viewer_pid"
```

Compare `/tmp/pico_arm_angle_weighted_replay.csv` to the retained
`/tmp/pico_position_otg_after.csv`, aligned by `pico_sequence`, using a
read-only Python/pandas command. Do not write generated data into Git.

Expected acceptance:

```text
control_failures == 0
hard_bound_violations == 0
right arm-angle error P95 < 0.986 rad
right max adjacent-cycle arm-angle-rate change < 3.741 rad/s
position/orientation tracking error <= 1.20 * baseline
```

If the available recording contains mapped targets rather than raw skeleton arm directions, report that it validates the TJ/QP half only; do not claim it validates the new PICO human-radial gate.

- [ ] **Step 4: Check repository scope and commit any verification-only updates**

```bash
git diff --check
git status --short --branch
git -C /home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1 diff --check
git -C /home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1 status --short --branch
```

Expected: TJ changes are committed except untracked `benchmark_results/`; PICO existing dirty state is preserved with only the documented threshold/test additions.
