# MuJoCo High-Response Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce Cartesian dynamic lag in the velocity and acceleration OTG branches without changing gains, endpoint settling, joint position/velocity limits, or feedback watchdogs.

**Architecture:** Raise fixed MuJoCo-only limits through every active stage rather than only at the OTG. Implement the shared velocity profile first, merge it into the acceleration branch, then apply acceleration-level command and qddot limits there. Use existing configuration tests and the deterministic A/B/C benchmark as the regression loop.

**Tech Stack:** C++17, yaml-cpp, GoogleTest, MuJoCo, Ruckig, qpOASES, CMake

## Global Constraints

- Both profiles use OTG limits `2.0`, `12.0`, `120.0`, `8.0`, `40.0`, `400.0` in linear velocity/acceleration/jerk and angular velocity/acceleration/jerk order.
- Keep Cartesian gains, joint position and velocity limits, watchdogs, and orientation settle thresholds unchanged.
- Velocity-level constraints use `60.0 rad/s^2` maximum acceleration and `45.0 rad/s^2` braking acceleration.
- Acceleration-level Cartesian command limits use `30.0 m/s^2` linear and `100.0 rad/s^2` angular.
- Acceleration-level joint constraints use `60.0 rad/s^2` maximum acceleration and `45.0 rad/s^2` braking acceleration.
- Every benchmark row must remain finite, hard-bound compliant, and free of solver failures.

---

### Task 1: Velocity-branch high-response profile

**Files:**
- Modify: `tests/test_config.cpp`
- Modify: `config/qp_ik_cartesian_otg_velocity.yaml`

**Interfaces:**
- Consumes: `loadConfig(const std::string&) -> QpIkConfig`
- Produces: the velocity branch's MuJoCo high-response YAML profile

- [ ] **Step 1: Change the velocity-profile configuration expectations first**

Set the existing `LoadsCartesianOtgVelocityProfile` assertions to:

```cpp
EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_velocity_max, 2.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_acceleration_max, 12.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_jerk_max, 120.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_velocity_max, 8.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_acceleration_max, 40.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_jerk_max, 400.0);
EXPECT_DOUBLE_EQ(config.cartesian_servo.max_linear_velocity, 2.5);
EXPECT_DOUBLE_EQ(config.cartesian_servo.max_angular_velocity, 8.0);
EXPECT_DOUBLE_EQ(config.joint_limits.max_acceleration_rad_s2, 60.0);
EXPECT_DOUBLE_EQ(config.joint_limits.braking_acceleration_rad_s2, 45.0);
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
cmake --build build --target test_config -j2 && \
./build/test_config --gtest_filter=Config.LoadsCartesianOtgVelocityProfile
```

Expected: FAIL because the YAML still contains the old `1/3/20`, `3/10/60`, `1/3`, and `20/15` limits.

- [ ] **Step 3: Update the velocity YAML**

Set exactly:

```yaml
cartesian_servo:
  max_linear_velocity: 2.5
  max_angular_velocity: 8.0
cartesian_otg:
  translation_velocity_max: 2.0
  translation_acceleration_max: 12.0
  translation_jerk_max: 120.0
  angular_velocity_max: 8.0
  angular_acceleration_max: 40.0
  angular_jerk_max: 400.0
joint_limits:
  max_acceleration_rad_s2: 60.0
  braking_acceleration_rad_s2: 45.0
```

- [ ] **Step 4: Run focused and full velocity verification**

Run:

```bash
cmake --build build -j2
./build/test_config --gtest_filter=Config.LoadsCartesianOtgVelocityProfile
ctest --test-dir build --output-on-failure
```

Expected: focused test and all velocity-branch tests PASS.

- [ ] **Step 5: Commit the velocity profile**

```bash
git add config/qp_ik_cartesian_otg_velocity.yaml tests/test_config.cpp
git commit -m "perf: raise MuJoCo velocity OTG bandwidth"
```

### Task 2: Acceleration-branch high-response profile

**Files:**
- Merge: `feature/cartesian-otg-velocity-qp-v1`
- Modify: `tests/test_config.cpp`
- Modify: `config/qp_ik_cartesian_otg_acceleration.yaml`

**Interfaces:**
- Consumes: Task 1's shared velocity profile and design history
- Produces: the acceleration branch's MuJoCo high-response YAML profile

- [ ] **Step 1: Merge the verified velocity branch**

```bash
git merge --no-edit feature/cartesian-otg-velocity-qp-v1
```

Expected: clean merge or conflicts resolved by preserving acceleration-only configuration assertions and files.

- [ ] **Step 2: Change acceleration-profile expectations first**

Add these assertions to `LoadsCartesianOtgAccelerationProfile`:

```cpp
EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_velocity_max, 2.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_acceleration_max, 12.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.translation_jerk_max, 120.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_velocity_max, 8.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_acceleration_max, 40.0);
EXPECT_DOUBLE_EQ(config.cartesian_otg.angular_jerk_max, 400.0);
EXPECT_DOUBLE_EQ(config.cartesian_acceleration.linear_limit, 30.0);
EXPECT_DOUBLE_EQ(config.cartesian_acceleration.angular_limit, 100.0);
EXPECT_DOUBLE_EQ(config.joint_acceleration_limits.max_acceleration_rad_s2, 60.0);
EXPECT_DOUBLE_EQ(config.joint_acceleration_limits.braking_acceleration_rad_s2, 45.0);
EXPECT_DOUBLE_EQ(config.joint_limits.max_acceleration_rad_s2, 60.0);
EXPECT_DOUBLE_EQ(config.joint_limits.braking_acceleration_rad_s2, 45.0);
```

- [ ] **Step 3: Run focused acceleration test and verify RED**

```bash
cmake --build build --target test_config -j2 && \
./build/test_config --gtest_filter=Config.LoadsCartesianOtgAccelerationProfile
```

Expected: FAIL on the old acceleration YAML limits.

- [ ] **Step 4: Update the acceleration YAML**

Use Task 1's OTG values and set:

```yaml
cartesian_acceleration:
  linear_limit: 30.0
  angular_limit: 100.0
joint_acceleration_limits:
  max_acceleration_rad_s2: 60.0
  braking_acceleration_rad_s2: 45.0
joint_limits:
  max_acceleration_rad_s2: 60.0
  braking_acceleration_rad_s2: 45.0
```

- [ ] **Step 5: Run focused and full acceleration verification**

```bash
cmake --build build -j2
./build/test_config --gtest_filter=Config.LoadsCartesianOtgAccelerationProfile
ctest --test-dir build --output-on-failure
```

Expected: focused test and all acceleration-branch tests PASS.

- [ ] **Step 6: Commit the acceleration profile**

```bash
git add config/qp_ik_cartesian_otg_acceleration.yaml tests/test_config.cpp
git commit -m "perf: raise MuJoCo acceleration OTG bandwidth"
```

### Task 3: Dynamic-response evidence and Viewer verification

**Files:**
- No tracked output files
- Output: `/tmp/tianji_high_response_*.csv`

**Interfaces:**
- Consumes: both committed high-response profiles
- Produces: before/after phase-lag, settling, bounds, solver and real-time evidence

- [ ] **Step 1: Run velocity benchmark and headless Viewer**

```bash
./build/tianji_cartesian_otg_benchmark \
  --config config/qp_ik_cartesian_otg_velocity.yaml \
  --model models/marvin_m6_qp_test.xml --steps 2000 \
  --output /tmp/tianji_high_response_velocity.csv
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_cartesian_otg_velocity.yaml \
  --model models/marvin_m6_qp_test.xml --headless --duration 4
```

Expected: finite benchmark rows, zero failures, hard bounds true; Viewer completes 800 cycles with zero control failures and deadline misses.

- [ ] **Step 2: Run acceleration benchmark and headless Viewer**

```bash
./build/tianji_cartesian_otg_benchmark \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml --steps 2000 \
  --output /tmp/tianji_high_response_acceleration.csv
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml --headless --duration 4
```

Expected: finite benchmark rows, zero failures, hard bounds true; Viewer completes 800 cycles with zero control failures and deadline misses.

- [ ] **Step 3: Compare against baseline**

Compare `otg_velocity_qp` and `otg_acceleration_qp` rows against
`/tmp/tianji_dynamic_response_baseline.csv`. Report straight/reversal/stop
phase lag and settling time, orientation RMS/P95 during motion, qddot peak,
active bounds, solver failures, and hard-bound status. Dynamic lag must improve
without regressing endpoint tests or safety acceptance.
