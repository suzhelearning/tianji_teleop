# PICO Position OTG Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reversible PICO translation Position-OTG mode that removes translation twist feedforward and second-order prediction while preserving the acceleration QP and all hard joint constraints.

**Architecture:** `CartesianOtgConfig` owns one optional `translation_position_mode` switch. `TargetManager` suppresses translation pose prediction while retaining timestamp-aware twist for stationary-hold intent detection, and `CartesianReferenceGenerator` defensively forces the translation Ruckig instance to Position interface with zero target velocity. The PICO profile enables it; omitted/false retains the existing path.

**Tech Stack:** C++17, Eigen, Ruckig, yaml-cpp, MuJoCo, GoogleTest, CMake/CTest, deterministic PICO UDP replay.

## Global Constraints

- Work on `feature/pico-mujoco-teleop-v1` in `/home/zj/current_robotics/TJ_arm/TJ_arm_control`.
- Preserve untracked `benchmark_results/` and do not modify other branches/worktrees.
- Do not change orientation OTG, acceleration-QP formulation, arm-angle task, solver settings, reference integration, or joint position/velocity/acceleration/jerk limits.
- `translation_position_mode: false` or an omitted field must preserve current behavior.
- Use tests-first RED/GREEN for every behavior change.

---

### Task 1: Configuration contract and PICO profile

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `tests/test_config.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`

**Interfaces:**
- Produces: `CartesianOtgConfig::translation_position_mode` as an optional YAML boolean defaulting to `false`.
- Consumes: the existing `cartesian_otg` YAML mapping.

- [ ] **Step 1: Write failing configuration tests**

Add assertions that the PICO profile enables the new mode and a configuration omitting the field leaves it false:

```cpp
EXPECT_TRUE(config.cartesian_otg.translation_position_mode);

QpIkConfig defaults = loadConfig(path_without_new_field);
EXPECT_FALSE(defaults.cartesian_otg.translation_position_mode);
```

- [ ] **Step 2: Run RED**

Run:

```bash
pixi run cmake --build build --target test_config -j"$(nproc)"
```

Expected: compilation fails because `translation_position_mode` does not exist.

- [ ] **Step 3: Implement the minimal configuration field**

Add:

```cpp
bool translation_position_mode{false};
```

Parse it with `optionalBool(...)`, and add this to the PICO YAML:

```yaml
translation_position_mode: true
```

- [ ] **Step 4: Run GREEN**

```bash
pixi run cmake --build build --target test_config -j"$(nproc)"
ctest --test-dir build -R '^test_config$' --output-on-failure
```

Expected: `test_config` passes.

### Task 2: Suppress translation pose prediction at the target boundary

**Files:**
- Modify: `src/target_manager.cpp`
- Modify: `tests/test_target_manager.cpp`

**Interfaces:**
- Consumes: `CartesianOtgConfig::translation_position_mode`.
- Produces: a raw mapped translation target while preserving linear twist for
  stationary-hold intent detection and angular twist for orientation handling.

- [ ] **Step 1: Write the failing target-manager test**

Create two timestamped manual frames with nonzero translation and rotation. In position mode, sample between frames and require no translation prediction while estimated linear and angular twist remain available for intent detection:

```cpp
target_config.cartesian_otg.enabled = true;
target_config.cartesian_otg.translation_position_mode = true;
target_config.cartesian_otg.translation_prediction_enabled = true;
const DualArmTargets output = manager.sample(0.015);
EXPECT_TRUE(output.left.position.isApprox(second.left.position, 1e-12));
EXPECT_TRUE(output.left_twist.head<3>().allFinite());
EXPECT_TRUE(output.left_twist.tail<3>().allFinite());
```

- [ ] **Step 2: Run RED**

```bash
pixi run cmake --build build --target test_target_manager -j"$(nproc)"
ctest --test-dir build -R '^test_target_manager$' --output-on-failure
```

Expected: position is still predicted.

- [ ] **Step 3: Implement pose-prediction suppression**

Make second-order translation prediction require `!translation_position_mode`.
Keep `manualTwist(...)` unchanged because stationary hold uses linear speed to
distinguish active motion from a quiet target; the Cartesian OTG is the sole
boundary that suppresses translation velocity feedforward.

- [ ] **Step 4: Run GREEN**

```bash
pixi run cmake --build build --target test_target_manager -j"$(nproc)"
ctest --test-dir build -R '^test_target_manager$' --output-on-failure
```

Expected: all target-manager tests pass.

### Task 3: Enforce Position interface inside Cartesian OTG

**Files:**
- Modify: `src/cartesian_otg.cpp`
- Modify: `tests/test_cartesian_otg.cpp`

**Interfaces:**
- Consumes: `CartesianOtgConfig::translation_position_mode` and any caller-provided target twist.
- Produces: Position-interface translation Ruckig input with zero target velocity when the mode is active.

- [ ] **Step 1: Write the failing defensive OTG test**

Reset at an identity pose, keep the target pose stationary, inject a nonzero linear target twist for multiple cycles, and require the reference to remain stationary:

```cpp
config.translation_position_mode = true;
Vec6 noisy_twist = Vec6::Zero();
noisy_twist.x() = 1.0;
for (int step = 0; step < 20; ++step) {
  ASSERT_TRUE(generator.update(identityPose(), noisy_twist, false, kDt).valid);
}
EXPECT_TRUE(generator.state().pose.position.isZero(1e-12));
EXPECT_TRUE(generator.state().twist.head<3>().isZero(1e-12));
```

Keep an adjacent compatibility assertion showing that velocity-tracking mode responds to the same twist.

- [ ] **Step 2: Run RED**

```bash
pixi run cmake --build build --target test_cartesian_otg -j"$(nproc)"
ctest --test-dir build -R '^test_cartesian_otg$' --output-on-failure
```

Expected: position mode moves because the current OTG consumes target twist.

- [ ] **Step 3: Implement the mode**

When active, force:

```cpp
translation_tracking_active = false;
translation_input_.control_interface = ruckig::ControlInterface::Position;
translation_feedforward.setZero();
translation_correction.setZero();
translation_input_.target_velocity = {0.0, 0.0, 0.0};
```

Leave target position and all Ruckig limits unchanged.

- [ ] **Step 4: Run GREEN and focused regression**

```bash
pixi run cmake --build build --target test_cartesian_otg test_target_manager test_config -j"$(nproc)"
ctest --test-dir build -R '^(test_cartesian_otg|test_target_manager|test_config)$' --output-on-failure
```

Expected: all three targets pass.

### Task 4: Documentation, complete regression, and recorded A/B

**Files:**
- Modify: `README.md`
- Generate only: `/tmp/pico_position_otg_after.csv`

**Interfaces:**
- Consumes: the PICO profile with position mode enabled and `/tmp/pico_fast_motion_20260812_205428.tjvr`.
- Produces: build/test evidence and replay metrics aligned by PICO sequence.

- [ ] **Step 1: Document the reversible mode**

Document that `translation_position_mode: true` means Position interface, zero translation target velocity, and no second-order translation prediction; false restores velocity tracking.

- [ ] **Step 2: Run full build and tests**

```bash
git diff --check
pixi run cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 3: Replay the identical PICO trace**

Run the Viewer and `replay_pico_udp_trace.py` in one shell, receive all 2584 datagrams, and write `/tmp/pico_position_otg_after.csv`. Require zero control failures.

- [ ] **Step 4: Compare acceptance metrics**

Align the baseline and new telemetry by `pico_sequence`. Require each arm to have no effective-lag regression beyond the 90 ms baseline, lower position-error P95, and lower greater-than-3-Hz actual-position RMS. Report orientation metrics without changing orientation code.

- [ ] **Step 5: Commit implementation**

```bash
git add include/tianji_qp_ik/config.hpp src/config.cpp src/target_manager.cpp \
  src/cartesian_otg.cpp tests/test_config.cpp tests/test_target_manager.cpp \
  tests/test_cartesian_otg.cpp config/qp_ik_pico_teleop.yaml README.md
git commit -m "fix: stabilize PICO translation with position OTG"
```
