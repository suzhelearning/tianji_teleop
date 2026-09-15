# VR-Fast Scripted Viewer Trajectories Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Viewer keys `1` through `4` run a 1 Hz VR-speed stress trajectory and make each OTG branch load its own profile when the Viewer is launched without `--config`.

**Architecture:** Keep trajectory generation and amplitudes unchanged; raise only the branch-specific OTG profile frequency. Keep option parsing in the Viewer and expose the selected path in startup output so a process-level CTest can verify each branch's no-argument default without adding a test-only API.

**Tech Stack:** C++17, CMake/CTest, GoogleTest, yaml-cpp, MuJoCo, Ruckig, qpOASES.

## Global Constraints

- Work only in the existing velocity and acceleration OTG worktrees; do not modify the base `feature/mujoco-cpp-qp-ik-v1` worktree.
- Preserve keys `1`/`2`/`3`/`4`, manual marker behavior, trajectory amplitudes, gains, OTG limits, joint limits, watchdogs, and settling logic.
- Use exactly `trajectories.frequency_hz: 1.0` in both branch-specific OTG profiles.
- Velocity branch no-argument default: `config/qp_ik_cartesian_otg_velocity.yaml`.
- Acceleration branch no-argument default: `config/qp_ik_cartesian_otg_acceleration.yaml`.
- Explicit `--config FILE` remains authoritative.
- Treat these profiles as MuJoCo-only stress configurations, not real-hardware approval.

---

### Task 1: Velocity branch VR-fast profile and default launch

**Files:**
- Modify: `tests/test_config.cpp`
- Modify: `CMakeLists.txt`
- Modify: `config/qp_ik_cartesian_otg_velocity.yaml`
- Modify: `apps/run_qp_ik_viewer.cpp`

**Interfaces:**
- Consumes: `QpIkConfig loadConfig(const std::string&)` and the existing Viewer `Options::config_path`.
- Produces: a 1 Hz velocity OTG profile and startup output `viewer_config=<path>` used by CTest.

- [ ] **Step 1: Add the failing velocity-profile frequency assertion**

In `TEST(Config, LoadsCartesianOtgVelocityProfile)`, add:

```cpp
EXPECT_DOUBLE_EQ(config.trajectories.frequency_hz, 1.0);
```

- [ ] **Step 2: Build and run the focused test to verify RED**

Run:

```bash
.pixi/envs/default/bin/cmake --build build -j2 --target test_config
./build/test_config --gtest_filter=Config.LoadsCartesianOtgVelocityProfile
```

Expected: FAIL because the loaded value is `0.10`, not `1.0`.

- [ ] **Step 3: Add the failing process-level default-profile test**

Append inside `if(BUILD_TESTING)` in `CMakeLists.txt`:

```cmake
add_test(
  NAME test_viewer_default_profile
  COMMAND tianji_qp_ik_viewer
    --headless
    --duration 4.0
    --model ${CMAKE_CURRENT_SOURCE_DIR}/models/marvin_m6_qp_test.xml
)
set_tests_properties(test_viewer_default_profile PROPERTIES
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  PASS_REGULAR_EXPRESSION
    "viewer_config=config/qp_ik_cartesian_otg_velocity.yaml"
)
```

Reconfigure, build, and run:

```bash
.pixi/envs/default/bin/cmake -S . -B build -DBUILD_TESTING=ON
.pixi/envs/default/bin/cmake --build build -j2 --target tianji_qp_ik_viewer
.pixi/envs/default/bin/ctest --test-dir build -R '^test_viewer_default_profile$' --output-on-failure
```

Expected: FAIL because the Viewer neither selects nor prints the velocity OTG default.

- [ ] **Step 4: Apply the minimal velocity implementation**

In `config/qp_ik_cartesian_otg_velocity.yaml`, change only:

```yaml
trajectories:
  frequency_hz: 1.0
```

In `apps/run_qp_ik_viewer.cpp`, change `Options` to:

```cpp
struct Options {
  std::string config_path{"config/qp_ik_cartesian_otg_velocity.yaml"};
```

At the beginning of `run`, after parsing and before loading the configuration, add:

```cpp
const Options options = parseOptions(argc, argv);
std::cout << "viewer_config=" << options.config_path << '\n';
QpIkConfig config = loadConfig(options.config_path);
```

Do not change explicit `--config` parsing.

- [ ] **Step 5: Verify GREEN and both profile-selection contracts**

Run:

```bash
.pixi/envs/default/bin/cmake -S . -B build -DBUILD_TESTING=ON
.pixi/envs/default/bin/cmake --build build -j2 --target test_config tianji_qp_ik_viewer
./build/test_config --gtest_filter=Config.LoadsCartesianOtgVelocityProfile
.pixi/envs/default/bin/ctest --test-dir build \
  -R '^(test_viewer_default_profile|test_viewer_explicit_profile)$' \
  --output-on-failure
```

Expected: the focused GoogleTest and both process tests PASS. The no-argument
default-profile CTest requires only the exact startup line
`viewer_config=config/qp_ik_cartesian_otg_velocity.yaml`. The explicit-profile
process helper
independently requires `viewer_config=config/qp_ik_hierarchical.yaml`; its
short `--duration 0.1` child process may exit with the documented status `2`.
Full staged headless success is reserved for each branch's no-argument OTG
default in Task 3.

- [ ] **Step 6: Commit the velocity branch change**

```bash
git add tests/test_config.cpp CMakeLists.txt \
  config/qp_ik_cartesian_otg_velocity.yaml apps/run_qp_ik_viewer.cpp
git commit -m "feat: make velocity Viewer trajectories VR-fast"
```

---

### Task 2: Acceleration branch VR-fast profile and default launch

**Files:**
- Merge: `feature/cartesian-otg-velocity-qp-v1`
- Modify: `tests/test_config.cpp`
- Modify: `CMakeLists.txt`
- Modify: `config/qp_ik_cartesian_otg_acceleration.yaml`
- Modify: `apps/run_qp_ik_viewer.cpp`

**Interfaces:**
- Consumes: the velocity branch's startup-output contract and process-level CTest.
- Produces: a 1 Hz acceleration OTG profile whose argument-free Viewer loads the acceleration configuration.

- [ ] **Step 1: Merge the completed velocity branch**

Run in the acceleration worktree:

```bash
git merge --no-ff feature/cartesian-otg-velocity-qp-v1 \
  -m "Merge velocity VR-fast Viewer trajectory support"
```

Expected: the velocity implementation, design, and plan enter the acceleration branch. Resolve only genuine branch-specific conflicts and preserve existing acceleration-QP tests.

- [ ] **Step 2: Change tests first for the acceleration branch and verify RED**

In `TEST(Config, LoadsCartesianOtgAccelerationProfile)`, add:

```cpp
EXPECT_DOUBLE_EQ(config.trajectories.frequency_hz, 1.0);
```

In the `test_viewer_default_profile` CTest property, replace the velocity expected path with:

```cmake
"viewer_config=config/qp_ik_cartesian_otg_acceleration.yaml"
```

Run:

```bash
.pixi/envs/default/bin/cmake -S . -B build -DBUILD_TESTING=ON
.pixi/envs/default/bin/cmake --build build -j2 --target test_config tianji_qp_ik_viewer
./build/test_config --gtest_filter=Config.LoadsCartesianOtgAccelerationProfile
.pixi/envs/default/bin/ctest --test-dir build -R '^test_viewer_default_profile$' --output-on-failure
```

Expected: the GoogleTest fails at `0.10` versus `1.0`, and the process test fails because startup still reports the velocity profile.

- [ ] **Step 3: Apply the minimal acceleration implementation**

In `config/qp_ik_cartesian_otg_acceleration.yaml`, change only:

```yaml
trajectories:
  frequency_hz: 1.0
```

In `apps/run_qp_ik_viewer.cpp`, change only the default initializer to:

```cpp
std::string config_path{"config/qp_ik_cartesian_otg_acceleration.yaml"};
```

- [ ] **Step 4: Verify GREEN and both profile-selection contracts**

Run:

```bash
.pixi/envs/default/bin/cmake -S . -B build -DBUILD_TESTING=ON
.pixi/envs/default/bin/cmake --build build -j2 --target test_config tianji_qp_ik_viewer
./build/test_config --gtest_filter=Config.LoadsCartesianOtgAccelerationProfile
.pixi/envs/default/bin/ctest --test-dir build \
  -R '^(test_viewer_default_profile|test_viewer_explicit_profile)$' \
  --output-on-failure
```

Expected: the focused GoogleTest and both process tests PASS. The no-argument
default-profile CTest requires only the exact startup line
`viewer_config=config/qp_ik_cartesian_otg_acceleration.yaml`. The
explicit-profile process helper
independently requires `viewer_config=config/qp_ik_hierarchical.yaml`; its
short `--duration 0.1` child process may exit with the documented status `2`.
Full staged headless success is reserved for each branch's no-argument OTG
default in Task 3.

- [ ] **Step 5: Commit the acceleration branch change**

```bash
git add tests/test_config.cpp CMakeLists.txt \
  config/qp_ik_cartesian_otg_acceleration.yaml apps/run_qp_ik_viewer.cpp
git commit -m "feat: make acceleration Viewer trajectories VR-fast"
```

---

### Task 3: Full branch verification

**Files:**
- Verify only; no production changes expected.

**Interfaces:**
- Consumes: both completed branches.
- Produces: fresh build, test, headless, and analytical speed evidence.

The full staged headless checks in this task intentionally omit `--config`.
They are the only checks in this plan that require full staged headless
success, and therefore exercise each branch's no-argument OTG default.

- [ ] **Step 1: Verify the complete velocity branch**

Run in the velocity worktree:

```bash
.pixi/envs/default/bin/cmake -S . -B build -DBUILD_TESTING=ON
.pixi/envs/default/bin/cmake --build build -j2
.pixi/envs/default/bin/ctest --test-dir build --output-on-failure
./build/tianji_qp_ik_viewer \
  --headless --duration 4.0 \
  --model models/marvin_m6_qp_test.xml
git diff --check
git status --short
```

Expected: build exits zero, all CTests pass, startup prints the velocity OTG
configuration, headless output reports `accepted=1`, `deadline_misses=0`,
`control_failures=0`, `command_failures=0`, and the worktree is clean.

- [ ] **Step 2: Verify the complete acceleration branch**

Run the same commands in the acceleration worktree.

Expected: startup prints the acceleration OTG configuration; all other pass
conditions match the velocity branch.

- [ ] **Step 3: Re-run the original deterministic speed check**

Run:

```bash
for f in \
  /home/zj/current_robotics/TJ_arm/TJ_arm_control_cartesian_otg_velocity_qp_v1/config/qp_ik_cartesian_otg_velocity.yaml \
  /home/zj/current_robotics/TJ_arm/TJ_arm_control_cartesian_otg_acceleration_qp_v1/config/qp_ik_cartesian_otg_acceleration.yaml
do
  awk '/circle_radius:/ {r=$2} /angular_amplitude:/ {a=$2} \
       /frequency_hz:/ {f=$2} \
       END {period=1/f; v=2*3.141592653589793*f*r; \
            w=2*3.141592653589793*f*a; \
            printf "%s: period=%.2fs circle_peak=%.3fm/s angular_peak=%.3frad/s\\n", \
                   FILENAME,period,v,w; if (period > 2.0) exit 1}' "$f"
done
```

Expected for both profiles:

```text
period=1.00s circle_peak=0.377m/s angular_peak=2.199rad/s
```

- [ ] **Step 4: Confirm branch relationship and untouched base worktree**

Run:

```bash
git -C /home/zj/current_robotics/TJ_arm/TJ_arm_control \
  status --short
git -C /home/zj/current_robotics/TJ_arm/TJ_arm_control_cartesian_otg_acceleration_qp_v1 \
  merge-base --is-ancestor feature/cartesian-otg-velocity-qp-v1 HEAD
```

Expected: the base status is empty and the ancestor check exits zero.
