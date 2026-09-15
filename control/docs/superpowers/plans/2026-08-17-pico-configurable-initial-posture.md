# PICO Configurable Initial Posture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Initialize and reset the PICO MuJoCo viewer to a configurable symmetric forward-reaching dual-arm posture while preserving midpoint startup for every other profile.

**Architecture:** Extend `ControllerConfig` with an opt-in dual-arm posture and centralize posture selection plus joint-range validation in the configuration module. The Viewer uses that single selector for startup and `N` reset before rebuilding targets and controllers, so MuJoCo, both controller reference states, Cartesian OTG, and SPARK guidance all inherit one coherent state.

**Tech Stack:** C++17, Eigen, yaml-cpp, MuJoCo, GoogleTest, CMake/CTest.

## Global Constraints

- Keep the current branch and worktree; do not create a worktree.
- Do not commit or push automatically.
- Only `config/qp_ik_pico_teleop.yaml` enables the configured posture.
- Keep midpoint initialization when the option is absent or disabled.
- Do not modify URDF zero positions, joint limits, QP equations, or motion limits.
- Reject malformed, non-finite, or out-of-range configured postures; never silently clamp.

---

### Task 1: Configuration contract and validated posture selection

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Test: `tests/test_config.cpp`

**Interfaces:**
- Produces: `ControllerConfig::{initial_posture_enabled,initial_left_q_rad,initial_right_q_rad}`.
- Produces: `Vec7 configuredInitialPosture(const ControllerConfig&, const ArmLimits&, ArmSide)`; returns the configured side when enabled, otherwise the joint-range midpoint, and throws `std::runtime_error` naming side and joint for an invalid enabled posture.

- [x] **Step 1: Write failing configuration tests**

Add tests that assert the PICO profile loads the exact left/right vectors, the standard profile remains disabled, malformed vectors fail during YAML loading, and `configuredInitialPosture` rejects an enabled value beyond its arm limit without clamping.

```cpp
EXPECT_TRUE(config.controller.initial_posture_enabled);
EXPECT_TRUE(config.controller.initial_left_q_rad.isApprox(
    (Vec7() << 1.10, -1.52, -1.52, -1.10, 0.0, 0.0, 0.0).finished()));
EXPECT_THROW(configuredInitialPosture(config.controller, limits,
                                      ArmSide::kLeft),
             std::runtime_error);
```

- [x] **Step 2: Verify RED**

Run: `cmake --build build -j"$(nproc)" --target test_config && ./build/test_config --gtest_filter='Config.*InitialPosture*'`

Expected: compilation fails because the new configuration fields and selector do not exist.

- [x] **Step 3: Implement the configuration contract**

Add the fields with disabled defaults, parse strict seven-element arrays only when present, reject non-finite enabled values during load, and implement the selector with explicit side/joint diagnostics.

```cpp
struct ControllerConfig {
  double rate_hz{1000.0};
  bool model_state_only{false};
  bool initial_posture_enabled{false};
  Vec7 initial_left_q_rad{Vec7::Zero()};
  Vec7 initial_right_q_rad{Vec7::Zero()};
};

Vec7 configuredInitialPosture(const ControllerConfig& config,
                              const ArmLimits& limits, ArmSide side);
```

- [x] **Step 4: Verify GREEN**

Run: `cmake --build build -j"$(nproc)" --target test_config && ./build/test_config --gtest_filter='Config.*InitialPosture*'`

Expected: all initial-posture tests pass.

### Task 2: PICO profile and Viewer startup/reset synchronization

**Files:**
- Modify: `config/qp_ik_pico_teleop.yaml`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Test: `tests/test_config.cpp`
- Test: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- Consumes: `configuredInitialPosture(...)` from Task 1.
- Produces: `setInitialConfiguration(MujocoRobot&, const QpIkConfig&)`, used for initial startup and `ViewerCommandType::kResetNominal`.

- [x] **Step 1: Add the enabled PICO profile values and integration assertions**

```yaml
controller:
  rate_hz: 200.0
  initial_posture_enabled: true
  initial_left_q_rad:  [1.10, -1.52, -1.52, -1.10, 0.0, 0.0, 0.0]
  initial_right_q_rad: [-1.10, -1.52, 1.52, -1.10, 0.0, 0.0, 0.0]
```

Extend the headless Viewer integration test to inspect first telemetry references and assert both configured vectors with zero initial velocity/acceleration.

- [x] **Step 2: Verify RED**

Run: `cmake --build build -j"$(nproc)" --target tianji_qp_ik_viewer test_config && ctest --test-dir build --output-on-failure -R 'pico_viewer|test_config'`

Expected: the Viewer integration assertion fails because startup still uses joint midpoints.

- [x] **Step 3: Route startup and reset through the selector**

Replace `setNominalConfiguration(robot)` with `setInitialConfiguration(robot, config)`. Set both arms with zero velocity, call `forward()`, then preserve the existing reset order: rebuild Cartesian targets, both controllers, OTG state, direct SPARK state, and plot differentiators.

```cpp
void setInitialConfiguration(MujocoRobot& robot, const QpIkConfig& config) {
  for (const ArmSide side : {ArmSide::kLeft, ArmSide::kRight}) {
    robot.setArmState(
        side,
        configuredInitialPosture(config.controller, robot.mapping(side).limits,
                                 side),
        Vec7::Zero());
  }
  robot.forward();
}
```

- [x] **Step 4: Verify focused behavior**

Run: `cmake --build build -j"$(nproc)" --target tianji_qp_ik_viewer test_config && ctest --test-dir build --output-on-failure -R 'pico_viewer|test_config'`

Expected: focused tests pass; PICO startup references equal the configured posture and non-PICO configuration behavior remains midpoint-based.

### Task 3: Regression verification

**Files:**
- Verify only: all modified files above

**Interfaces:**
- Consumes: completed implementation from Tasks 1-2.
- Produces: evidence that startup changes do not alter existing IK or QP behavior.

- [x] **Step 1: Build all targets**

Run: `cmake --build build -j"$(nproc)"`

Expected: successful build with no new warnings.

- [x] **Step 2: Run the complete test suite**

Run: `ctest --test-dir build --output-on-failure -j"$(nproc)"`

Expected: all tests pass.

- [x] **Step 3: Run a headless PICO smoke test**

Run: `./build/tianji_qp_ik_viewer --config config/qp_ik_pico_teleop.yaml --model models/marvin_m6_qp_pico_fast.xml --headless --duration 0.10 --control-level acceleration`

Expected: clean exit with no configuration, joint-limit, or solver initialization failure.

- [x] **Step 4: Inspect scope**

Run: `git diff --check && git diff -- include/tianji_qp_ik/config.hpp src/config.cpp apps/run_qp_ik_viewer.cpp config/qp_ik_pico_teleop.yaml tests/test_config.cpp tests/test_pico_viewer_integration.py`

Expected: no whitespace errors and no QP equations, URDF limits, or motion constraints changed.
