# Cartesian OTG + Velocity QP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a jerk-limited Cartesian reference generator in front of the existing 7-DoF velocity QP and provide a deterministic direct-vs-OTG comparison.

**Architecture:** `TargetManager` continues to own timestamp-aware raw VR targets. A new per-arm `CartesianReferenceGenerator` converts raw pose/twist into continuous pose/twist/acceleration references. The existing QP remains unchanged and consumes `V_ref + Kp*pose_error` through an explicit reference-servo path.

**Tech Stack:** C++17, Eigen 3.4, Ruckig 0.19.4, MuJoCo 3.6, qpOASES, yaml-cpp, GoogleTest, CMake/Pixi.

## Global Constraints

- Branch from `feature/mujoco-cpp-qp-ik-v1` and keep the baseline configs behaviorally unchanged.
- Run all real-time control and OTG updates at `dt = 0.005 s`.
- Use world-frame translation, rotation error, twist, and Jacobian conventions.
- Use Ruckig only for XYZ translation; orientation uses SO(3), not Euler angles.
- Keep velocity QP variables `[qdot(7), slack(6)]`, qpOASES hotstart, and all existing hard joint bounds.
- Hold or stale input must decelerate through OTG rather than zero derivative state discontinuously.
- One-arm failure must not stop the healthy arm.

---

### Task 1: Pin and expose the Ruckig dependency

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `pixi.toml`
- Modify: `pixi.lock`
- Test: `tests/test_cartesian_otg.cpp`

**Interfaces:**
- Produces: CMake target `ruckig::ruckig` and header `<ruckig/ruckig.hpp>`.

- [ ] **Step 1: Add a compile-smoke test target**

Create `tests/test_cartesian_otg.cpp` with:

```cpp
#include <gtest/gtest.h>
#include <ruckig/ruckig.hpp>

TEST(CartesianOtgDependency, ConstructsThreeDofGenerator) {
  ruckig::Ruckig<3> generator(0.005);
  (void)generator;
  SUCCEED();
}
```

- [ ] **Step 2: Run configure and verify the dependency is missing**

Run: `pixi run configure`

Expected: configure or compilation fails because Ruckig is not available.

- [ ] **Step 3: Add pinned FetchContent integration**

Add before `add_library(tianji_qp_ik ...)`:

```cmake
include(FetchContent)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  ruckig
  GIT_REPOSITORY https://github.com/pantor/ruckig.git
  GIT_TAG v0.19.4
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(ruckig)
```

Link `ruckig::ruckig` publicly to `tianji_qp_ik`, register
`test_cartesian_otg`, then regenerate the Pixi lock if dependency metadata
changes.

- [ ] **Step 4: Build the smoke test**

Run: `pixi run configure && cmake --build build --target test_cartesian_otg -j`

Expected: target builds.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt pixi.toml pixi.lock tests/test_cartesian_otg.cpp
git commit -m "build: add pinned Ruckig dependency"
```

### Task 2: Add OTG configuration and reference types

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `include/tianji_qp_ik/target_manager.hpp`
- Modify: `config/qp_ik_qp_limit.yaml`
- Create: `config/qp_ik_cartesian_otg_velocity.yaml`
- Modify: `tests/test_config.cpp`

**Interfaces:**
- Produces: `CartesianOtgConfig`, `CartesianReference`, and `DualArmReferences`.

- [ ] **Step 1: Write failing config tests**

Add assertions that the OTG profile loads:

```cpp
EXPECT_TRUE(config.cartesian_otg.enabled);
EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_velocity_max, 1.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_acceleration_max, 3.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_jerk_max, 20.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_velocity_max, 3.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_acceleration_max, 10.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_jerk_max, 60.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.orientation_gain, 10.0);
```

Also assert existing profiles default to `enabled == false`.

- [ ] **Step 2: Run the config test red**

Run: `cmake --build build --target test_config -j && ./build/test_config`

Expected: compile failure because `cartesian_otg` does not exist.

- [ ] **Step 3: Add exact types**

```cpp
struct CartesianOtgConfig {
  bool enabled{false};
  double translation_velocity_max{1.0};
  double translation_acceleration_max{3.0};
  double translation_jerk_max{20.0};
  double angular_velocity_max{3.0};
  double angular_acceleration_max{10.0};
  double angular_jerk_max{60.0};
  double orientation_gain{10.0};
};

struct CartesianReference {
  Pose pose;
  Vec6 twist{Vec6::Zero()};
  Vec6 acceleration{Vec6::Zero()};
  bool stale{false};
  bool valid{true};
};

struct DualArmReferences {
  CartesianReference left;
  CartesianReference right;
};
```

Parse an optional `cartesian_otg` YAML map and require every enabled limit and
gain to be finite and positive.

- [ ] **Step 4: Create the opt-in profile**

Copy the validated QP-limit profile, set direct `kff_linear` and
`kff_angular` to zero, and add:

```yaml
cartesian_otg:
  enabled: true
  translation_velocity_max: 1.0
  translation_acceleration_max: 3.0
  translation_jerk_max: 20.0
  angular_velocity_max: 3.0
  angular_acceleration_max: 10.0
  angular_jerk_max: 60.0
  orientation_gain: 10.0
```

- [ ] **Step 5: Run tests and commit**

Run: `cmake --build build --target test_config -j && ./build/test_config`

```bash
git add include/tianji_qp_ik/config.hpp include/tianji_qp_ik/target_manager.hpp \
  src/config.cpp config tests/test_config.cpp
git commit -m "feat: configure Cartesian OTG references"
```

### Task 3: Implement the per-arm Cartesian reference generator

**Files:**
- Create: `include/tianji_qp_ik/cartesian_otg.hpp`
- Create: `src/cartesian_otg.cpp`
- Modify: `tests/test_cartesian_otg.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `CartesianOtgConfig`, measured `Pose`, raw `Pose`, raw `Vec6`, stale flag, `dt`.
- Produces:

```cpp
class CartesianReferenceGenerator {
 public:
  explicit CartesianReferenceGenerator(CartesianOtgConfig config,
                                       double dt_seconds);
  void reset(const Pose& measured);
  CartesianReference update(const Pose& target, const Vec6& target_twist,
                            bool stale, double dt_seconds);
  const CartesianReference& state() const noexcept;
};
```

- [ ] **Step 1: Write failing behavior tests**

Tests must cover:

```cpp
TEST(CartesianOtg, ResetStartsAtMeasuredPoseWithZeroDerivatives);
TEST(CartesianOtg, TranslationRespectsVelocityAccelerationAndJerk);
TEST(CartesianOtg, OrientationUsesShortestWorldFrameRotation);
TEST(CartesianOtg, OrientationRespectsAngularDerivativeLimits);
TEST(CartesianOtg, StaleInputDeceleratesWithoutPoseJump);
TEST(CartesianOtg, RejectsNonFiniteInputWithoutMutatingState);
```

For derivative checks use:

```cpp
EXPECT_LE(reference.twist.head<3>().norm(), v_max + 1e-10);
EXPECT_LE(reference.acceleration.head<3>().norm(), a_max + 1e-10);
EXPECT_LE((a_now - a_previous).norm(), j_max * dt + 1e-9);
```

- [ ] **Step 2: Run tests red**

Run: `cmake --build build --target test_cartesian_otg -j`

Expected: missing `cartesian_otg.hpp`.

- [ ] **Step 3: Implement translation with Ruckig**

Use `ruckig::Ruckig<3>`, `InputParameter<3>`, and `OutputParameter<3>`.
Initialize current state from the stored reference, set target velocity to zero
when stale, call `update`, reject negative `Result`, and copy
`new_position/new_velocity/new_acceleration` to the reference.

- [ ] **Step 4: Implement SO(3) OTG**

Compute:

```cpp
const Eigen::Vector3d phi = so3Log(target.rotation * state.pose.rotation.transpose());
const Eigen::Vector3d w_goal = normLimit(config.orientation_gain * phi,
                                         config.angular_velocity_max);
const Eigen::Vector3d alpha_goal = normLimit(
    (w_goal - state.twist.tail<3>()) / dt,
    config.angular_acceleration_max);
const Eigen::Vector3d delta_alpha = normLimit(
    alpha_goal - state.acceleration.tail<3>(),
    config.angular_jerk_max * dt);
```

Integrate acceleration, angular velocity, and
`ExpSO3(w_ref * dt) * R_ref`, then re-orthonormalize only if the rotation error
exceeds numerical tolerance.

- [ ] **Step 5: Run OTG tests and commit**

Run: `cmake --build build --target test_cartesian_otg -j && ./build/test_cartesian_otg`

```bash
git add CMakeLists.txt include/tianji_qp_ik/cartesian_otg.hpp \
  src/cartesian_otg.cpp tests/test_cartesian_otg.cpp
git commit -m "feat: generate jerk-limited Cartesian references"
```

### Task 4: Add the explicit Cartesian reference servo

**Files:**
- Modify: `include/tianji_qp_ik/cartesian_servo.hpp`
- Modify: `src/cartesian_servo.cpp`
- Modify: `tests/test_cartesian_servo.cpp`

**Interfaces:**
- Produces:

```cpp
Vec6 cartesianReferenceServoTwist(const CartesianServoConfig& config,
                                  const CartesianReference& reference,
                                  const Pose& measured);
```

- [ ] **Step 1: Write a failing no-double-feedforward test**

```cpp
const Vec6 result = cartesianReferenceServoTwist(config, reference, measured);
const Vec6 expected = reference.twist +
    (Vec6() << config.kp_position.cwiseProduct(position_error),
               config.kp_orientation.cwiseProduct(rotation_error)).finished();
EXPECT_TRUE(result.isApprox(normLimited(expected, config), 1e-12));
```

Set legacy Kff to large values in the fixture and prove they do not alter this
new path.

- [ ] **Step 2: Run red, implement, run green**

Run: `cmake --build build --target test_cartesian_servo -j && ./build/test_cartesian_servo`

Implement world-frame pose error, add `reference.twist` exactly once, and apply
the existing linear/angular norm clamps after composition.

- [ ] **Step 3: Commit**

```bash
git add include/tianji_qp_ik/cartesian_servo.hpp src/cartesian_servo.cpp \
  tests/test_cartesian_servo.cpp
git commit -m "feat: track Cartesian OTG references"
```

### Task 5: Integrate OTG into the dual-arm control loop

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `include/tianji_qp_ik/controller.hpp`
- Modify: `src/controller.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `tests/test_controller.cpp`
- Modify: `tests/test_hierarchical_controller.cpp`
- Create: `tests/test_otg_controller.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Controller overload:

```cpp
ControllerDiagnostics step(const DualArmReferences& references, double dt);
```

- [ ] **Step 1: Write failing integration tests**

Cover reset at measured TCP, moving reference feedforward, Hold smooth
deceleration, stale target, one-arm invalid OTG output, and legacy direct path
compatibility.

- [ ] **Step 2: Run the new test red**

Run: `cmake --build build --target test_otg_controller -j`

- [ ] **Step 3: Add per-arm generators to the control thread**

Construct two generators from config. On OTG activation/reset, call `reset`
with measured TCP poses. Each cycle convert `DualArmTargets` to
`DualArmReferences`; if OTG is disabled, preserve the existing direct call.

- [ ] **Step 4: Route reference servo into the unchanged velocity QP**

Build `ArmIkInput` exactly as before except `desired_twist` comes from
`cartesianReferenceServoTwist`. Preserve all bounds, q-reference integration,
watchdog, per-arm commit, and solver history behavior.

- [ ] **Step 5: Extend telemetry**

Record per arm `v_ref`, `w_ref`, `a_ref`, `alpha_ref`, OTG validity, stale state,
and the existing QP diagnostics. Update CSV headers and overlay without doing
heap allocation in the control loop.

- [ ] **Step 6: Run integration and regression tests**

Run:

```bash
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 7: Commit**

```bash
git add apps include src tests CMakeLists.txt
git commit -m "feat: run velocity QP from Cartesian OTG"
```

### Task 6: Add deterministic A/B benchmark coverage

**Files:**
- Create: `apps/benchmark_cartesian_otg.cpp`
- Create: `tests/test_cartesian_otg_benchmark.cpp`
- Modify: `CMakeLists.txt`
- Modify: `pixi.toml`

**Interfaces:**
- Produces executable `tianji_cartesian_otg_benchmark` and CSV rows keyed by
  `controller=direct_velocity_qp|otg_velocity_qp`.

- [ ] **Step 1: Write benchmark acceptance test**

Invoke both controllers for 600 steps over straight, circle, reversal, stop,
jitter, orientation, near-limit, singular, unreachable, timestamp-jitter, and
dropped-frame scenarios. Assert finite metrics, hard bounds, and zero QP
failures; do not assert OTG always has lower tracking error.

- [ ] **Step 2: Run test red**

Run: `cmake --build build --target test_cartesian_otg_benchmark -j`

- [ ] **Step 3: Implement metrics and CSV**

Record position/orientation RMS and P95, peak error, lag estimate, reference
acceleration/jerk, qdot variation, qddot/jerk estimates, reference error,
slack, active bounds, solve p99, and failure counts.

- [ ] **Step 4: Run benchmark green and commit**

Run:

```bash
./build/tianji_cartesian_otg_benchmark \
  --config config/qp_ik_cartesian_otg_velocity.yaml \
  --model models/marvin_m6_qp_test.xml \
  --output /tmp/tianji_otg_velocity_ab.csv
```

```bash
git add CMakeLists.txt pixi.toml apps/benchmark_cartesian_otg.cpp \
  tests/test_cartesian_otg_benchmark.cpp
git commit -m "test: benchmark Cartesian OTG velocity QP"
```

### Task 7: Final velocity-branch verification and documentation

**Files:**
- Modify: `README.md`
- Create: `docs/verification/cartesian_otg_velocity_qp_results.md`

- [ ] **Step 1: Document architecture, commands, and limitations**

Document direct and OTG profiles, Ruckig/SO(3) split, Hold/stale behavior,
telemetry fields, and the fact that the current MuJoCo plant is kinematic.

- [ ] **Step 2: Run complete verification**

```bash
pixi run configure
pixi run build
pixi run test
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_cartesian_otg_velocity.yaml \
  --model models/marvin_m6_qp_test.xml \
  --headless --duration 4 \
  --telemetry /tmp/tianji_otg_velocity_headless.csv
./build/tianji_cartesian_otg_benchmark \
  --config config/qp_ik_cartesian_otg_velocity.yaml \
  --model models/marvin_m6_qp_test.xml \
  --output /tmp/tianji_otg_velocity_ab.csv
```

Expected: complete test suite passes; headless reports accepted, zero command
and control failures, zero deadline misses; benchmark reports finite metrics,
hard bounds respected, and zero QP failures.

- [ ] **Step 3: Record exact results and commit**

```bash
git add README.md docs/verification/cartesian_otg_velocity_qp_results.md
git commit -m "docs: record Cartesian OTG velocity QP results"
```
