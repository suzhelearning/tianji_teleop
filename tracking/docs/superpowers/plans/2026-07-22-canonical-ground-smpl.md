# Canonical Ground-Aligned PICO SMPL Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish the APK payload unchanged on `/pico/smpl_raw` and publish `/pico/smpl` only after an A-triggered stable-foot calibration has translated both soles to world `Z=0`.

**Architecture:** Keep TCP decoding in `pico_bridge_node` and move ground calibration into a separate C++ node. Put orientation-aware sole geometry and the A/reset state machine in a header-only, ROS-independent unit so it can be exhaustively tested. Existing Odin, foot-IMU, recorder, and viewer consumers retain `/pico/smpl` and therefore automatically receive canonical `pico_ground` poses.

**Tech Stack:** ROS 2 Humble, C++17, `rclcpp`, `geometry_msgs`, `std_msgs`, Eigen-free quaternion math, `ament_cmake_gtest`, Python launch/source tests, pixi/colcon.

## Global Constraints

- Work directly on `feature/odin-pelvis-extrinsic-calibration`; do not create another branch or worktree.
- Preserve all unrelated dirty-worktree changes and stage only files named by each task.
- `/pico/smpl_raw` must be geometrically identical to the decoded 24-pose APK payload.
- `/pico/smpl` must remain silent before A and while no stable post-reset floor exists.
- Ground calibration changes position Z only; it preserves X/Y and every quaternion.
- Default foot half-extents are `[0.115, 0.050, 0.022]` metres, stable window is 30 frames, and tolerance is `0.02 m`.
- Canonical output uses `header.frame_id = "pico_ground"`.
- Odin calibration/runtime and foot-IMU fusion continue subscribing to `/pico/smpl` without remapping.

---

## File Structure

- Create `src/pico_bridge/include/pico_bridge/smpl_ground_alignment.hpp`: finite-pose validation, oriented sole-height calculation, rolling stable-floor lock, reset, and common-Z translation.
- Create `src/pico_bridge/src/pico_smpl_ground_node.cpp`: ROS parameters, `/pico/smpl_raw` and `/pico/world_reset` subscriptions, `/pico/smpl` publication, throttled waiting diagnostics.
- Create `src/pico_bridge/test/test_smpl_ground_alignment.cpp`: deterministic math/state tests.
- Create `src/pico_bridge/test/test_smpl_ground_integration.py`: source/launch contract tests that protect topic ownership and downstream topic names.
- Modify `src/pico_bridge/src/pico_bridge_node.cpp`: rename its body publisher to `/pico/smpl_raw` without changing payload assembly.
- Modify `src/pico_bridge/launch/start_pico_bridge.launch.py`: start both bridge and normalizer and expose validated calibration arguments.
- Modify `src/pico_bridge/CMakeLists.txt`: build/install/test the new component.
- Modify `README.md`, `README.zh-CN.md`, and `pico_full_test_commands.txt`: document A-first startup, raw/canonical topics, and verification commands while preserving existing unrelated edits.

---

### Task 1: Ground-alignment math and state machine

**Files:**
- Create: `src/pico_bridge/include/pico_bridge/smpl_ground_alignment.hpp`
- Create: `src/pico_bridge/test/test_smpl_ground_alignment.cpp`
- Modify: `src/pico_bridge/CMakeLists.txt`

**Interfaces:**
- Produces: `pico_bridge::SmplPose`, `pico_bridge::GroundAlignmentOptions`, and `pico_bridge::SmplGroundAlignment` with `reset()`, `observe(const std::vector<SmplPose>&)`, `locked()`, `floor_height()`, and `transform(const std::vector<SmplPose>&)`.
- Consumes: 24 checked poses in xyzw quaternion order; foot indices 10 and 11.

- [ ] **Step 1: Write failing unit tests**

Cover identity and pitched-foot sole height, invalid quaternion rejection, silence before reset, exactly 30 stable post-reset frames, unstable-window restart, common Z translation, unchanged X/Y/quaternions, and reset clearing an existing lock. Representative assertions:

```cpp
pico_bridge::SmplGroundAlignment alignment(options);
EXPECT_FALSE(alignment.observe(make_skeleton(-1.60)));
alignment.reset();
for (int i = 0; i < 29; ++i) EXPECT_FALSE(alignment.observe(make_skeleton(-1.60)));
EXPECT_TRUE(alignment.observe(make_skeleton(-1.60)));
ASSERT_TRUE(alignment.locked());
const auto output = alignment.transform(make_skeleton(-1.60));
EXPECT_NEAR(sole_height(output[10], options.foot_half_extents), 0.0, 1e-9);
EXPECT_DOUBLE_EQ(output[0].position.x(), input[0].position.x());
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
pixi run colcon test --packages-select pico_bridge \
  --ctest-args -R test_smpl_ground_alignment --output-on-failure
```

Expected: configure/build failure because `smpl_ground_alignment.hpp` and the test target do not exist.

- [ ] **Step 3: Implement the minimal ROS-independent alignment unit**

Use explicit scalar structs to avoid adding Eigen to `pico_bridge`. Normalize each xyzw quaternion, compute the vertical radius of an oriented box as
`abs(R20)*hx + abs(R21)*hy + abs(R22)*hz`, and define sole Z as `position.z - vertical_radius`. `observe()` accepts data only after `reset()`, maintains a bounded deque of paired sole heights, and locks their median only when the full flattened range is at most the tolerance. `transform()` throws unless locked and subtracts the locked floor from every Z.

- [ ] **Step 4: Build and run the focused unit test to verify GREEN**

Run:

```bash
pixi run build --packages-select pico_bridge
pixi run colcon test --packages-select pico_bridge \
  --ctest-args -R test_smpl_ground_alignment --output-on-failure
pixi run colcon test-result --test-result-base build --verbose
```

Expected: the new gtest passes with zero failures.

- [ ] **Step 5: Commit only the math/test files**

```bash
git add src/pico_bridge/include/pico_bridge/smpl_ground_alignment.hpp \
  src/pico_bridge/test/test_smpl_ground_alignment.cpp \
  src/pico_bridge/CMakeLists.txt
git commit -m "feat(pico_bridge): add SMPL ground alignment state"
```

---

### Task 2: Raw topic ownership and canonical ROS node

**Files:**
- Create: `src/pico_bridge/src/pico_smpl_ground_node.cpp`
- Create: `src/pico_bridge/test/test_smpl_ground_integration.py`
- Modify: `src/pico_bridge/src/pico_bridge_node.cpp`
- Modify: `src/pico_bridge/launch/start_pico_bridge.launch.py`
- Modify: `src/pico_bridge/CMakeLists.txt`

**Interfaces:**
- Consumes: `/pico/smpl_raw` (`geometry_msgs/msg/PoseArray`) and `/pico/world_reset` (`std_msgs/msg/Float32`).
- Produces: `/pico/smpl` (`geometry_msgs/msg/PoseArray`) with `frame_id=pico_ground` only when `SmplGroundAlignment::locked()` is true.

- [ ] **Step 1: Write failing integration/source-contract tests**

Assert that bridge source owns `/pico/smpl_raw` but not `/pico/smpl`, the new node defaults to the exact raw/reset/output topics, launch starts both executables, launch defaults equal the design values, and the existing foot-fusion/Odin sources still default to `/pico/smpl`.

```python
assert '"/pico/smpl_raw"' in bridge_source
assert 'executable="pico_smpl_ground"' in launch_source
assert 'default_value="30"' in launch_source
assert 'default_value="0.02"' in launch_source
assert '"/pico/smpl"' in fusion_source
assert '"/pico/smpl"' in odin_runtime_source
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```bash
pixi run python -m pytest -q src/pico_bridge/test/test_smpl_ground_integration.py
```

Expected: failures because the raw publisher, node, and launch action do not exist.

- [ ] **Step 3: Change bridge ownership and implement the ROS adapter**

Change only the bridge publisher topic from `/pico/smpl` to `/pico/smpl_raw`. In the adapter, declare and validate:

```cpp
raw_topic = "/pico/smpl_raw";
output_topic = "/pico/smpl";
world_reset_topic = "/pico/world_reset";
output_frame = "pico_ground";
foot_half_extents = {0.115, 0.050, 0.022};
stable_window_frames = 30;
stability_tolerance_m = 0.02;
require_world_reset = true;
```

The world-reset callback calls `alignment.reset()` before returning. The raw callback rejects arrays whose size is not 24, converts checked poses into the math type, feeds the state machine, and publishes a copied PoseArray with only position Z changed after lock. Log the locked raw floor height once and throttle invalid/waiting warnings.

- [ ] **Step 4: Wire build/install and launch**

Build the executable as `pico_smpl_ground`, install it under `lib/pico_bridge`, and start it next to `pico_bridge_node`. Pass all launch arguments as typed `ParameterValue` instances and keep `require_world_reset=true` by default.

- [ ] **Step 5: Run focused tests and package build**

Run:

```bash
pixi run python -m pytest -q src/pico_bridge/test/test_smpl_ground_integration.py
pixi run build --packages-select pico_bridge
pixi run colcon test --packages-select pico_bridge --event-handlers console_direct+
pixi run colcon test-result --test-result-base build --verbose
```

Expected: all `pico_bridge` tests pass, zero errors/failures.

- [ ] **Step 6: Commit only canonical-topic implementation files**

```bash
git add src/pico_bridge/src/pico_bridge_node.cpp \
  src/pico_bridge/src/pico_smpl_ground_node.cpp \
  src/pico_bridge/launch/start_pico_bridge.launch.py \
  src/pico_bridge/test/test_smpl_ground_integration.py \
  src/pico_bridge/CMakeLists.txt
git commit -m "feat(pico_bridge): publish canonical ground SMPL"
```

---

### Task 3: End-to-end reset semantics and downstream compatibility

**Files:**
- Modify: `src/pico_bridge/test/test_smpl_ground_integration.py`
- Modify: `src/pico_bridge/test/test_imu900_foot_integration.py`
- Modify: `src/pico_odin/test/test_launch_integration.py`
- Modify: `src/pico_odin/test/test_runtime_ros_integration.py`

**Interfaces:**
- Consumes: installed `pico_smpl_ground`, `/pico/smpl_raw`, and `/pico/world_reset`.
- Produces: executable evidence that no canonical frame leaks before reset/lock and that Odin receives positive-height canonical roots.

- [ ] **Step 1: Add a ROS integration test that drives the installed node**

Start `pico_smpl_ground` with a unique topic namespace. Publish stable 24-pose frames before reset and assert no output. Publish one reset plus 29 stable frames and assert no output; publish frame 30 and assert one output with `frame_id=pico_ground`, sole Z near zero, positive pelvis/head Z, and unchanged orientations. Publish another reset and assert output stops until a new 30-frame lock.

- [ ] **Step 2: Run the ROS integration test and verify any missing behavior fails**

Run:

```bash
pixi run build --packages-select pico_bridge
pixi run python -m pytest -q \
  src/pico_bridge/test/test_smpl_ground_integration.py -k ros_runtime
```

Expected before final wiring: failure on an unmet reset, QoS, or frame contract rather than a timeout with no diagnostic.

- [ ] **Step 3: Fix only behavior exposed by the integration test**

Use `rclcpp::SensorDataQoS()` for raw and canonical PoseArray streams and reliable depth-10 QoS for world-reset events. Ensure reset and pose callbacks serialize state access, and do not replay the 30 calibration frames after lock.

- [ ] **Step 4: Extend downstream contract tests**

Assert foot fusion publishes `pico_ground` when its input is canonical, Odin runtime's default `output_frame`/launch frame is `pico_ground`, and no consumer silently subscribes to `/pico/smpl_raw`. Update test fixtures that previously constructed `frame_id=pico` canonical messages.

- [ ] **Step 5: Run downstream focused suites**

Run:

```bash
pixi run build --packages-up-to pico_bridge pico_odin
pixi run python -m pytest -q src/pico_bridge/test/test_smpl_ground_integration.py
pixi run python -m pytest -q src/pico_bridge/test/test_imu900_foot_integration.py
pixi run python -m pytest -q src/pico_odin/test/test_launch_integration.py
pixi run python -m pytest -q src/pico_odin/test/test_runtime_ros_integration.py
```

Expected: all focused suites pass.

- [ ] **Step 6: Commit compatibility tests and minimal fixes**

```bash
git add src/pico_bridge/test/test_smpl_ground_integration.py \
  src/pico_bridge/test/test_imu900_foot_integration.py \
  src/pico_odin/test/test_launch_integration.py \
  src/pico_odin/test/test_runtime_ros_integration.py \
  src/pico_bridge/src/pico_smpl_ground_node.cpp \
  src/pico_bridge/src/pico_foot_imu_fusion_node.cpp \
  src/pico_odin/launch/odin_select.launch.py \
  src/pico_odin/src/odin_pelvis_runtime_node.cpp
git commit -m "test: verify ground SMPL downstream pipeline"
```

Only stage implementation files from the last list if the integration test required an actual change; never stage unrelated pre-existing edits.

---

### Task 4: Documentation and full verification

**Files:**
- Modify: `README.md`
- Modify: `README.zh-CN.md`
- Modify: `pico_full_test_commands.txt`

**Interfaces:**
- Documents: `/pico/smpl_raw`, A-first floor lock, canonical `/pico/smpl`, `pico_ground`, and Odin/IMU/MuJoCo consumption.

- [ ] **Step 1: Update English and Chinese topic/workflow documentation**

State explicitly that `/pico/smpl` is silent before A, takes about 30 stable frames to appear, uses foot-sole `Z=0`, and is the required Odin calibration input. Add diagnostic commands:

```bash
ros2 topic echo /pico/smpl_raw --once
# Press PICO controller A and stand still.
ros2 topic echo /pico/smpl --once
ros2 topic hz /pico/smpl
```

Remove claims that `/pico/smpl` is unchanged APK data; replace them with `/pico/smpl_raw`.

- [ ] **Step 2: Update the complete test-command guide**

Put A/reset and raw/canonical checks before foot-IMU and Odin calibration. Document that a missing `/pico/smpl` with a healthy `/pico/smpl_raw` means the ground lock has not completed.

- [ ] **Step 3: Run documentation and repository checks**

Run:

```bash
rg -n "/home/zj|/pico/smpl_raw|pico_ground" README.md README.zh-CN.md pico_full_test_commands.txt
git diff --check
```

Expected: no machine-specific path is introduced, canonical terms are present, and no whitespace errors exist.

- [ ] **Step 4: Run the full build and test suite**

Run:

```bash
pixi run build
pixi run test
pixi run colcon test-result --test-result-base build --verbose
```

Expected: all packages build; zero test errors and failures.

- [ ] **Step 5: Inspect the final diff without staging unrelated work**

Run:

```bash
git status --short
git diff -- src/pico_bridge src/pico_odin README.md README.zh-CN.md pico_full_test_commands.txt
```

Expected: every feature diff maps to this plan, and all pre-existing unrelated changes remain preserved.

- [ ] **Step 6: Commit documentation only if it can be isolated from pre-existing edits**

If the three documentation files contain earlier unrelated edits, leave them uncommitted and report that explicitly. Otherwise:

```bash
git add README.md README.zh-CN.md pico_full_test_commands.txt
git commit -m "docs: document canonical ground SMPL workflow"
```
