# Joint4 Human-Elbow Limit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make both elbow Joint4 joints stop at the human-like straight-arm position of `0.0 rad` while preserving their existing `-2.5307 rad` lower limit.

**Architecture:** The checked-in robot descriptions remain the source of joint-limit truth. The runtime regression loads the MuJoCo MJCF through `MujocoRobot`, verifies the limits exposed to both IK backends, and the same values are synchronized into both source URDF variants and the local-path URDF.

**Tech Stack:** C++17, GoogleTest, MuJoCo MJCF, URDF XML, CMake/CTest

## Global Constraints

- `Joint4_L` and `Joint4_R` position range is exactly `[-2.5307, 0.0] rad`.
- Do not change Joint4 velocity, acceleration, or torque limits.
- Do not change any other joint's position range or any kinematic transform.
- Keep the current `feature/mujoco-cpp-qp-ik-v1` branch and existing worktree.

---

### Task 1: Enforce and verify the bilateral Joint4 range

**Files:**
- Modify: `tests/test_mujoco_robot.cpp`
- Modify: `tests/test_controller.cpp`
- Modify: `tests/test_benchmark_dataset.cpp`
- Modify: `apps/benchmark_hierarchical_ik.cpp`
- Modify: `include/tianji_qp_ik/benchmark_dataset.hpp`
- Modify: `src/benchmark_dataset.cpp`
- Modify: `models/marvin_m6_qp_test.xml`
- Modify: `models/marvin_m6_s_ccs_696_v4_local.urdf`
- Modify: `marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4_mujoco.urdf`
- Modify: `marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf`

**Interfaces:**
- Consumes: `MujocoRobot::mapping(ArmSide)` and `ArmMapping::limits` loaded from the runtime MJCF.
- Produces: bilateral Joint4 limits exposed as `lower_position[3] == -2.5307` and `upper_position[3] == 0.0`.

- [x] **Step 1: Write the failing runtime-model test**

Add this test to `tests/test_mujoco_robot.cpp`:

```cpp
TEST(MujocoRobot, Joint4UsesHumanLikeElbowRange) {
  MujocoRobot robot(modelPath());

  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    const ArmLimits& limits = robot.mapping(side).limits;
    EXPECT_NEAR(limits.lower_position[3], -2.5307, 1e-12);
    EXPECT_NEAR(limits.upper_position[3], 0.0, 1e-12);
  }
}
```

- [x] **Step 2: Build and run the focused test to verify RED**

Run:

```bash
cmake --build build --target test_mujoco_robot -j
./build/test_mujoco_robot --gtest_filter=MujocoRobot.Joint4UsesHumanLikeElbowRange
```

Expected: the test fails twice because both loaded upper limits are `1.0472`, not `0.0`.

- [x] **Step 3: Update all checked-in robot descriptions**

For both `Joint4_L` and `Joint4_R`, replace only the upper position limit:

```text
before: lower = -2.5307, upper = 1.0472
after:  lower = -2.5307, upper = 0.0
```

In the runtime MJCF this is:

```xml
range="-2.5307 0"
```

In each URDF this is:

```xml
<limit
  lower="-2.5307"
  upper="0"
  effort="66"
  velocity="3.1416" />
```

Preserve each file's existing formatting and every unrelated attribute.

- [x] **Step 4: Run the focused test to verify GREEN**

Run:

```bash
./build/test_mujoco_robot --gtest_filter=MujocoRobot.Joint4UsesHumanLikeElbowRange
```

Expected: one test passes with no failures.

- [x] **Step 5: Verify all four model representations are synchronized**

Run:

```bash
rg -n -A16 'name="Joint4_[LR]"' \
  models/marvin_m6_qp_test.xml \
  models/marvin_m6_s_ccs_696_v4_local.urdf \
  marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4_mujoco.urdf \
  marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf
```

Expected: all eight Joint4 declarations show lower `-2.5307` and upper `0` (or the equivalent MJCF range), while effort and velocity remain unchanged.

- [x] **Step 6: Run the complete automated regression suite**

Run:

```bash
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Expected: build succeeds and all 22 registered tests pass.

- [x] **Step 7: Run the headless hierarchical-QP acceptance sequence**

Run:

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_hierarchical.yaml \
  --model models/marvin_m6_qp_test.xml \
  --headless --duration 4 \
  --telemetry /tmp/tianji_joint4_limit_telemetry.csv
```

Expected: exit code zero and final output includes `algorithm=hierarchical_qp`, `accepted=1`, `control_failures=0`, `command_failures=0`, and `completed_stage=10`.

- [x] **Step 8: Keep diagnostic fixtures inside the new safe range**

Before final verification, keep tests that intentionally exercise solver
failure or singular kinematics inside the configured Joint4 safety margin:

```cpp
// Solver-failure fixture: use the model midpoint so bounds do not mask the
// injected backend failure.
robot.setArmPosition(side, midpoint(robot.mapping(side).limits));

// Singular fixture and benchmark share one measured rank-deficient pose.
// Its dedicated test verifies every configured safety margin, including
// Joint4, before verifying the Jacobian singular value.
const Vec7 singular = safeSingularBenchmarkPosition();
```

Retain the existing SVD assertion proving that the adjusted singular fixture
still has minimum singular value below `1e-4`.

- [x] **Step 9: Commit the implementation**

```bash
git add tests/test_mujoco_robot.cpp tests/test_controller.cpp \
  tests/test_benchmark_dataset.cpp \
  apps/benchmark_hierarchical_ik.cpp \
  include/tianji_qp_ik/benchmark_dataset.hpp src/benchmark_dataset.cpp \
  models/marvin_m6_qp_test.xml \
  models/marvin_m6_s_ccs_696_v4_local.urdf \
  marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4_mujoco.urdf \
  marvin_m6_ccs/urdf/marvin_m6_s_ccs_696_v4.urdf \
  docs/superpowers/plans/2026-08-10-joint4-human-elbow-limit.md
git commit -m "fix: prevent Joint4 elbow hyperextension"
```
