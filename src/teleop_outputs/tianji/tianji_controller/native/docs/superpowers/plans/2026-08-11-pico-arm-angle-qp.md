# PICO-Guided Arm-Angle QP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a PICO-corrected human elbow-plane reference as a soft null-space task in both Tianji QPs, with deterministic elbow-down standalone behavior.

**Architecture:** The PICO bridge extracts length-independent elbow radial directions from the already corrected shoulder/elbow/wrist positions and carries them in a backward-compatible UDP V2 frame. The Viewer selects and rate-limits human or default-down references, computes a scalar arm-angle row from MuJoCo command-model geometry, and injects that row into the velocity- and acceleration-level QP objectives without changing Cartesian equalities or safety bounds.

**Tech Stack:** ROS 2 Humble, C++17, Eigen 3.4, MuJoCo, qpOASES, Python/pytest, GoogleTest, CMake/CTest, colcon.

## Global Constraints

- Keep the existing branches `feature/tianji-mujoco-teleop-v1` and `feature/pico-mujoco-teleop-v1`; create no branch or worktree.
- Preserve all existing dirty PICO calibration, palm-offset, and end-effector-axis changes.
- Cartesian pose remains the primary task; arm angle is a soft task projected through the TCP null space.
- PICO palm `+X 0.10 m` translation compensation must not alter the arm-angle reference.
- No PICO, disabled/stale PICO, V1 packets, or invalid arm direction selects world `-Z` elbow-down behavior.
- Support both velocity and acceleration control levels; DLS remains unchanged.
- Keep all joint, acceleration, braking, watchdog, atomic dual-arm, and stale-input behavior.
- Final acceptance requires all tests green and PICO headless control p99 below `5000 us` with zero deadline misses.

---

### Task 1: Extract corrected human elbow-plane references

**Files:**
- Modify: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/src/pico_bridge/include/pico_bridge/tianji_teleop_geometry.hpp`
- Modify: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/src/pico_bridge/src/tianji_teleop_geometry.cpp`
- Test: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/src/pico_bridge/test/test_tianji_teleop_geometry.cpp`

**Interfaces:**
- Consumes: corrected SMPL indices left `(16,18,20)` and right `(17,19,21)` plus the existing `pico_shoulder_frame`.
- Produces: `PicoArmDirection { bool valid; Eigen::Vector3d direction; }` in `PicoShoulderMapResult::left_arm_direction` and `right_arm_direction`.

- [ ] **Step 1: Write failing geometry tests**

Add tests that construct bent corrected arms and assert:

```cpp
const auto mapped = pb::map_pico_palms_to_tianji(frame);
ASSERT_TRUE(mapped.left_arm_direction.valid);
EXPECT_TRUE(mapped.left_arm_direction.direction.isApprox(expected, 1e-12));
EXPECT_NEAR(mapped.left_arm_direction.direction.norm(), 1.0, 1e-12);
```

Also change `pico_world_x_offset_m` between `0.0` and `0.25` and assert both arm directions are identical, and make shoulder/elbow/wrist collinear to assert only that side is invalid while palm targets remain valid.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
cmake --build build/pico_bridge --target test_tianji_teleop_geometry -j2
```

Expected: compilation fails because `PicoArmDirection` and result fields do not exist.

- [ ] **Step 3: Implement corrected arm-direction extraction**

Add constants `kPicoLeftElbow=18`, `kPicoRightElbow=19`, `kPicoLeftWrist=20`, and `kPicoRightWrist=21`. Add a helper equivalent to:

```cpp
PicoArmDirection armDirection(const Eigen::Vector3d& shoulder,
                              const Eigen::Vector3d& elbow,
                              const Eigen::Vector3d& wrist,
                              const Eigen::Matrix3d& pico_to_robot) {
  PicoArmDirection result;
  const Eigen::Vector3d axis = wrist - shoulder;
  if (axis.norm() <= 1e-9) return result;
  const Eigen::Vector3d u = axis.normalized();
  const Eigen::Vector3d radial =
      (elbow - shoulder) - u * u.dot(elbow - shoulder);
  if (radial.norm() < 1e-4) return result;
  result.direction = (pico_to_robot * radial.normalized()).normalized();
  result.valid = result.direction.allFinite();
  return result;
}
```

Use `pico_shoulder_frame.linear().transpose()` as `pico_to_robot`. Do not pass `pico_world_x_offset_m` into this helper.

- [ ] **Step 4: Run geometry tests and verify GREEN**

Run:

```bash
cmake --build build/pico_bridge --target test_tianji_teleop_geometry -j2
build/pico_bridge/test_tianji_teleop_geometry
```

Expected: all `TianjiTeleopGeometry` tests pass.

- [ ] **Step 5: Commit the geometry checkpoint**

```bash
git add src/pico_bridge/include/pico_bridge/tianji_teleop_geometry.hpp \
  src/pico_bridge/src/tianji_teleop_geometry.cpp \
  src/pico_bridge/test/test_tianji_teleop_geometry.cpp
git commit -m "feat: map corrected PICO elbow directions"
```

---

### Task 2: Encode PICO UDP protocol V2

**Files:**
- Modify: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/src/pico_bridge/include/pico_bridge/tianji_teleop_protocol.hpp`
- Modify: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/src/pico_bridge/src/tianji_teleop_protocol.cpp`
- Test: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/src/pico_bridge/test/test_tianji_teleop_protocol.cpp`
- Test: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/src/pico_bridge/test/test_tianji_mujoco_teleop_bridge_runtime.py`

**Interfaces:**
- Consumes: `PicoShoulderMapResult` arm directions from Task 1.
- Produces: 208-byte protocol V2 with optional validity bits 4/5 and vectors at offsets 156/180.

- [ ] **Step 1: Write failing V2 wire tests**

Assert version `2`, declared/actual size `208`, arm-valid flags, vector doubles, and CRC at offset `204`. Extend the runtime test to unpack both vectors and assert the expected unit directions.

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
cmake --build build/pico_bridge --target test_tianji_teleop_protocol -j2
```

Expected: assertions or compilation fail against the current 160-byte V1 encoder.

- [ ] **Step 3: Implement V2 encoding**

Set:

```cpp
inline constexpr std::size_t kTianjiTeleopPacketSize = 208;
inline constexpr std::uint16_t kTianjiTeleopProtocolVersion = 2;
inline constexpr std::uint32_t kLeftArmDirectionValid = 1U << 4U;
inline constexpr std::uint32_t kRightArmDirectionValid = 1U << 5U;
```

Carry both `PicoArmDirection` values through `TianjiTeleopWireFrame`, write valid vectors at offsets 156 and 180, and write CRC32 at 204 over bytes `[0,204)`. Invalid vectors serialize as zero and leave the side bit clear.

- [ ] **Step 4: Run focused and runtime tests**

Run:

```bash
cmake --build build/pico_bridge --target test_tianji_teleop_protocol tianji_mujoco_teleop_bridge -j2
build/pico_bridge/test_tianji_teleop_protocol
pixi run pytest -q src/pico_bridge/test/test_tianji_mujoco_teleop_bridge_runtime.py --count=5
```

Expected: protocol tests pass and runtime passes 5/5.

- [ ] **Step 5: Commit the PICO V2 protocol**

```bash
git add src/pico_bridge/include/pico_bridge/tianji_teleop_protocol.hpp \
  src/pico_bridge/src/tianji_teleop_protocol.cpp \
  src/pico_bridge/test/test_tianji_teleop_protocol.cpp \
  src/pico_bridge/test/test_tianji_mujoco_teleop_bridge_runtime.py
git commit -m "feat: send PICO elbow references in teleop V2"
```

---

### Task 3: Decode V1/V2 and manage arm references in the Viewer

**Files:**
- Modify: `include/tianji_qp_ik/pico_teleop_protocol.hpp`
- Modify: `src/pico_teleop_protocol.cpp`
- Modify: `src/pico_udp_receiver.cpp`
- Create: `include/tianji_qp_ik/arm_angle.hpp`
- Create: `src/arm_angle.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/test_pico_teleop_protocol.cpp`
- Test: `tests/test_pico_udp_receiver.cpp`
- Create: `tests/test_arm_angle.cpp`

**Interfaces:**
- Produces: `ArmDirectionReference`, `DualArmDirectionReferences`, `ArmDirectionReferenceManager`, and V1/V2-decoded optional arm vectors.

- [ ] **Step 1: Write failing protocol and reference-manager tests**

Test V1 decode with invalid arm references, V2 decode with normalized vectors, malformed optional vectors degrading to invalid, and spherical rate limiting:

```cpp
ArmDirectionReferenceManager manager(config);
const auto first = manager.update({}, 0.005);
EXPECT_TRUE(first.left.direction.isApprox(-Vec3::UnitZ(), 1e-12));
const auto next = manager.update(human_references, 0.005);
EXPECT_LE(rotationAngle(first.left.direction, next.left.direction),
          config.reference_rate_limit_rad_s * 0.005 + 1e-12);
```

- [ ] **Step 2: Run focused tests and verify RED**

Run `cmake --build build --target test_pico_teleop_protocol test_arm_angle -j2`.

Expected: missing arm-angle types/target and V2 protocol assertions fail.

- [ ] **Step 3: Implement backward-compatible decoding and manager**

Accept exactly `(version=1,size=160)` or `(version=2,size=208)`. Keep V1 pose offsets unchanged. For V2 validate CRC at 204 and read optional vectors. Increase the receiver datagram buffer to `209` bytes. Implement a unit-vector manager that selects human vectors when valid, otherwise `-Z`, and advances by at most `reference_rate_limit_rad_s * dt` using axis-angle interpolation.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run:

```bash
cmake --build build --target test_pico_teleop_protocol test_pico_udp_receiver test_arm_angle -j2
./build/test_pico_teleop_protocol
./build/test_pico_udp_receiver
./build/test_arm_angle
```

Expected: all pass.

- [ ] **Step 5: Commit protocol decoding and reference management**

```bash
git add CMakeLists.txt include/tianji_qp_ik/pico_teleop_protocol.hpp \
  include/tianji_qp_ik/arm_angle.hpp src/pico_teleop_protocol.cpp \
  src/pico_udp_receiver.cpp src/arm_angle.cpp \
  tests/test_pico_teleop_protocol.cpp tests/test_pico_udp_receiver.cpp \
  tests/test_arm_angle.cpp
git commit -m "feat: decode PICO arm references"
```

---

### Task 4: Compute robot arm-angle geometry from the command model

**Files:**
- Modify: `include/tianji_qp_ik/mujoco_robot.hpp`
- Modify: `src/mujoco_robot.cpp`
- Modify: `include/tianji_qp_ik/arm_angle.hpp`
- Modify: `src/arm_angle.cpp`
- Test: `tests/test_mujoco_robot.cpp`
- Test: `tests/test_arm_angle.cpp`

**Interfaces:**
- Produces: expanded `ArmKinematicSample` with shoulder/elbow/wrist points and elbow translational Jacobian; `computeArmAngleTask(...) -> ArmAngleTask`.

- [ ] **Step 1: Write failing MuJoCo and arm-angle tests**

Assert body mapping uses Link1/Link4/Link5, the elbow Jacobian predicts finite-difference elbow motion, signed errors have opposite signs for opposite references, the default-down reference requests downward elbow motion, and radius fade is zero/full at `0.015/0.050 m`.

- [ ] **Step 2: Run tests and verify RED**

Run `cmake --build build --target test_mujoco_robot test_arm_angle -j2`.

Expected: missing geometry members and arm task function.

- [ ] **Step 3: Implement command-model geometry and scalar task**

Extend `armKinematicsAt` after `mj_forward` using `data->xpos` for Link1, Link4, Link5 and `mj_jacBody` for Link4. Implement:

```cpp
struct ArmAngleTask {
  bool active{false};
  Vec7 jacobian{Vec7::Zero()};
  double error_rad{0.0};
  double radius_m{0.0};
  double activation{0.0};
};
```

Compute the signed error and damped projector from the approved design. Store the scalar row transposed in `Vec7 jacobian`. If projection, LDLT solve, or radius is invalid, return inactive without throwing from the control loop.

- [ ] **Step 4: Run tests and verify GREEN**

Run:

```bash
cmake --build build --target test_mujoco_robot test_arm_angle -j2
./build/test_mujoco_robot
./build/test_arm_angle
```

Expected: all pass, including finite-difference Jacobian tolerance.

- [ ] **Step 5: Commit command-model arm geometry**

```bash
git add include/tianji_qp_ik/mujoco_robot.hpp src/mujoco_robot.cpp \
  include/tianji_qp_ik/arm_angle.hpp src/arm_angle.cpp \
  tests/test_mujoco_robot.cpp tests/test_arm_angle.cpp
git commit -m "feat: compute Tianji arm-angle task geometry"
```

---

### Task 5: Add velocity- and acceleration-level QP objectives

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: every `config/*.yaml`
- Modify: `include/tianji_qp_ik/velocity_ik.hpp`
- Modify: `include/tianji_qp_ik/acceleration_ik.hpp`
- Modify: `src/hierarchical_qp.cpp`
- Modify: `src/acceleration_qp.cpp`
- Modify: `src/controller.cpp`
- Modify: `src/acceleration_controller.cpp`
- Test: `tests/test_config.cpp`
- Test: `tests/test_hierarchical_qp.cpp`
- Test: `tests/test_acceleration_qp.cpp`
- Test: `tests/test_hierarchical_controller.cpp`
- Test: `tests/test_acceleration_controller.cpp`

**Interfaces:**
- Consumes: `ArmAngleTask` and `DualArmDirectionReferences` from Tasks 3/4.
- Produces: `ScalarJointTask { bool active; Vec7 jacobian; double target; double weight; }` in both QP inputs.
- Produces controller overloads:
  `DualArmController::step(const DualArmTargets&, const DualArmDirectionReferences&, double)`,
  `DualArmController::step(const DualArmReferences&, const DualArmDirectionReferences&, double)`, and
  `DualArmAccelerationController::step(const DualArmReferences&, const DualArmDirectionReferences&, double)`.

- [ ] **Step 1: Write failing config and QP matrix tests**

For `j=[1,0,...]`, target `2`, weight `0.05`, assert velocity QP adds `0.05` to `H(0,0)` and `-0.10` to `g(0)`. Repeat for acceleration QP. Add invalid config cases for negative gains/weights, zero limits/damping, and reversed radii.

- [ ] **Step 2: Run tests and verify RED**

Run `cmake --build build --target test_config test_hierarchical_qp test_acceleration_qp -j2`.

Expected: missing `arm_angle` config and scalar task fields.

- [ ] **Step 3: Implement config, scalar objective, and controller commands**

Add the exact `arm_angle` defaults from the design to every YAML. In each QP builder apply:

```cpp
if (input.arm_angle_task.active) {
  const Vec7& j = input.arm_angle_task.jacobian;
  problem.H.topLeftCorner<7, 7>().noalias() +=
      input.arm_angle_task.weight * j * j.transpose();
  problem.g.head<7>().noalias() -=
      input.arm_angle_task.weight * j * input.arm_angle_task.target;
}
```

Velocity target is clamped `kp_velocity*error`; acceleration target is clamped `kp_acceleration*error-kd_acceleration*(J_arm*qdot_ref)`. Multiply configured weight by radius activation.

Existing controller overloads without the new argument call the new overloads
with `defaultArmDirectionReferences()`, whose two valid directions are world
`-Z`. This preserves standalone tests and marker/scripted operation without a
Viewer-owned manager.

- [ ] **Step 4: Run QP and controller tests and verify GREEN**

Run:

```bash
cmake --build build --target test_config test_hierarchical_qp test_acceleration_qp test_hierarchical_controller test_acceleration_controller -j2
./build/test_config
./build/test_hierarchical_qp
./build/test_acceleration_qp
./build/test_hierarchical_controller
./build/test_acceleration_controller
```

Expected: all pass and Cartesian equality rows/bounds remain byte-for-byte unchanged in matrix tests.

- [ ] **Step 5: Commit both QP arm-angle objectives**

```bash
git add include/tianji_qp_ik/config.hpp src/config.cpp config \
  include/tianji_qp_ik/velocity_ik.hpp include/tianji_qp_ik/acceleration_ik.hpp \
  include/tianji_qp_ik/controller.hpp include/tianji_qp_ik/acceleration_controller.hpp \
  src/hierarchical_qp.cpp src/acceleration_qp.cpp src/controller.cpp \
  src/acceleration_controller.cpp tests/test_config.cpp \
  tests/test_hierarchical_qp.cpp tests/test_acceleration_qp.cpp \
  tests/test_hierarchical_controller.cpp tests/test_acceleration_controller.cpp
git commit -m "feat: constrain QP redundancy with arm angle"
```

---

### Task 6: Integrate Viewer selection, telemetry, and end-to-end behavior

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `include/tianji_qp_ik/controller.hpp`
- Modify: `include/tianji_qp_ik/acceleration_controller.hpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `src/telemetry.cpp`
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `tests/test_snapshot_exchange.cpp`

**Interfaces:**
- Consumes: decoded optional PICO vectors and `PicoTeleopFreshness`.
- Produces: per-cycle selected references and arm-angle diagnostics in snapshots/CSV.

- [ ] **Step 1: Write failing integration assertions**

Extend the test sender to emit V2 references. Add runs for human elbow-down and `--no-send`, then assert output fields include source, active, error, radius, shoulder Z, and elbow Z. Require `abs(error)<0.10` after one second and `elbow_z<=shoulder_z+0.02` for the selected reachable pose.

- [ ] **Step 2: Run integration tests and verify RED**

Run:

```bash
ctest --test-dir build -R 'pico_viewer_integration_(velocity|acceleration)|pico_viewer_monitor_no_input' --output-on-failure
```

Expected: V2/telemetry assertions fail.

- [ ] **Step 3: Wire per-cycle reference selection and diagnostics**

Retain the latest valid V2 vectors alongside the applied PICO pose. Each cycle, use them only when PICO is enabled and fresh; otherwise pass invalid inputs to the manager so it selects default-down. Feed the manager output to whichever controller level is active. Reset only temporal manager state on tracking-epoch reset. Add the approved diagnostics to snapshots, monitor output, and CSV without allocating in the control-loop hot path.

- [ ] **Step 4: Run integration tests and verify GREEN**

Run the same CTest regex. Expected: velocity, acceleration, default-profile, and no-input cases all pass.

- [ ] **Step 5: Commit Viewer arm-angle integration**

```bash
git add apps/run_qp_ik_viewer.cpp include/tianji_qp_ik/controller.hpp \
  include/tianji_qp_ik/acceleration_controller.hpp \
  include/tianji_qp_ik/telemetry.hpp src/telemetry.cpp \
  tests/test_pico_viewer_integration.py tests/test_snapshot_exchange.cpp
git commit -m "feat: apply PICO arm-angle references in Viewer"
```

---

### Task 7: Full cross-repository verification

**Files:**
- Verify all changed files in both existing worktrees.

- [ ] **Step 1: Verify PICO repository**

Run:

```bash
cd /home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1
pixi run colcon build --packages-select pico_bridge --symlink-install
pixi run colcon test --packages-select pico_bridge
pixi run colcon test-result --verbose
git diff --check
```

Expected: build exits 0, all tests pass, no whitespace errors.

- [ ] **Step 2: Verify Tianji repository**

Run:

```bash
cd /home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1
cmake --build build -j2
ctest --test-dir build --output-on-failure
git diff --check
```

Expected: build exits 0 and all CTest tests pass.

- [ ] **Step 3: Run the 10-second performance acceptance**

Run:

```bash
python3 tests/test_pico_viewer_integration.py \
  --viewer ./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_test.xml \
  --control-level velocity --viewer-duration 10 --send-duration 9
python3 tests/test_pico_viewer_integration.py \
  --viewer ./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_test.xml \
  --control-level acceleration --viewer-duration 10 --send-duration 9
```

The integration script must assert `float(summary["cycle_p99_us"]) < 5000`,
`int(summary["deadline_misses"]) == 0`, `control_failures == 0`, and matching
left/right source timestamps. Expected: both commands exit 0.

- [ ] **Step 4: Review repository state**

Run `git status --short --branch` in both worktrees. Confirm only intended feature files and the previously preserved PICO changes are present; do not push.
