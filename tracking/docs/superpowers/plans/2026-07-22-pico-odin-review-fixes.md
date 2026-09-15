# PICO + Odin Lite Review Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the PICO ground-aligned skeleton, Odin Lite driver, pelvis calibration, and corrected skeleton runtime fail safely and behave deterministically during A-button reset and install-space execution.

**Architecture:** Keep the existing ROS nodes and extract only small policy helpers that can be tested without hardware. The calibration session owns the pending-start transition, ROS callbacks enforce the canonical frame contract, and the Odin driver resolves an explicit channel profile from the ament package share before opening streams.

**Tech Stack:** ROS 2 Humble, C++17/C++14, Eigen, rclcpp, ament_cmake, GoogleTest, pytest/launch, YAML-CPP, Pixi/colcon.

## Global Constraints

- `/pico/smpl_raw` remains the APK skeleton in frame `pico`.
- `/pico/smpl` remains the canonical ground-aligned skeleton in frame `pico_ground`.
- Saved extrinsics retain the `T_pelvis_odin` convention and `xyzw` quaternion order.
- Missing or invalid Odin Lite channel configuration must stop startup before streams are opened.
- Existing full-profile startup remains available only when selected explicitly.
- Existing untracked APK, recordings, and local test-instruction files must not be staged or modified.

---

### Task 1: Pending A-button calibration handoff

**Files:**
- Modify: `src/pico_odin/include/pico_odin/calibration_session.hpp`
- Modify: `src/pico_odin/src/odin_pelvis_calibrator_node.cpp`
- Test: `src/pico_odin/test/test_calibration_session.cpp`
- Test: `src/pico_odin/test/test_calibrator_ros_integration.py`

**Interfaces:**
- Consumes: `/pico/world_reset`, fresh canonical PICO samples, fresh Odin samples.
- Produces: `CalibrationPhase::kPendingStart`, `request_start()`, and a transition to `kNeutral` only after fresh inputs return.

- [ ] **Step 1: Write failing session tests**

Add tests that express the state contract:

```cpp
TEST(CalibrationSession, RequestedStartSurvivesInputPauseAndStartsWhenFresh) {
  po::CalibrationSession session;
  session.observe_valid_inputs(0.0);
  ASSERT_TRUE(session.request_start());
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kPendingStart);
  session.observe_invalid_inputs();
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kPendingStart);
  session.observe_valid_inputs(0.8);
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kNeutral);
}

TEST(CalibrationSession, SecondResetCancelsOnlyAnActiveAttempt) {
  po::CalibrationSession session;
  session.observe_valid_inputs(0.0);
  ASSERT_TRUE(session.request_start());
  session.on_world_reset();
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kPendingStart);
  session.observe_valid_inputs(1.0);
  session.on_world_reset();
  EXPECT_EQ(session.phase(), po::CalibrationPhase::kCancelled);
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
pixi run colcon test --packages-select pico_odin --ctest-args -R test_calibration_session --output-on-failure
```

Expected: compilation fails because `kPendingStart` and `request_start()` do not exist.

- [ ] **Step 3: Implement the pending transition**

Add `kPendingStart`; implement `request_start()` to clear samples and enter pending; make `observe_invalid_inputs()` preserve pending; make `observe_valid_inputs(now)` start the neutral phase from pending. Preserve `start(now)` for the keyboard fallback.

Update the A callback to call `request_start()`, clear PICO stationarity and its last-receipt marker, and leave repeated reset events pending. The timer must not use `max_input_age_sec` to cancel pending state.

- [ ] **Step 4: Add ROS source/integration assertions**

Extend the Python integration test to assert the A callback uses `request_start()`, resets PICO stationarity, and no longer consumes a `start_requested_` atomic in the timer.

- [ ] **Step 5: Run Task 1 tests and verify GREEN**

```bash
pixi run colcon build --packages-select pico_odin
pixi run colcon test --packages-select pico_odin --event-handlers console_direct+
pixi run colcon test-result --test-result-base build --verbose
```

Expected: all `pico_odin` tests pass.

- [ ] **Step 6: Commit Task 1**

```bash
git add src/pico_odin/include/pico_odin/calibration_session.hpp \
  src/pico_odin/src/odin_pelvis_calibrator_node.cpp \
  src/pico_odin/test/test_calibration_session.cpp \
  src/pico_odin/test/test_calibrator_ros_integration.py
git commit -m "fix: defer Odin calibration until ground skeleton resumes"
```

---

### Task 2: Canonical skeleton frame contract

**Files:**
- Modify: `src/pico_odin/src/odin_pelvis_calibrator_node.cpp`
- Modify: `src/pico_odin/src/odin_pelvis_runtime_node.cpp`
- Modify: `src/pico_odin/launch/odin_select.launch.py`
- Test: `src/pico_odin/test/test_calibrator_ros_integration.py`
- Test: `src/pico_odin/test/test_runtime_ros_integration.py`
- Test: `src/pico_odin/test/test_launch_integration.py`

**Interfaces:**
- Consumes: `expected_skeleton_frame` parameter, default `pico_ground`.
- Produces: strict rejection of empty, `pico`, or mismatched PoseArray frames before synchronization.

- [ ] **Step 1: Write failing wrong-frame ROS tests**

Publish otherwise valid skeletons using `frame_id="pico"` and verify no calibrated output is received; then publish `frame_id="pico_ground"` and verify the existing output path works. Update current test fixtures that incorrectly publish frame `pico` to use the canonical frame for success cases.

- [ ] **Step 2: Run focused ROS tests and verify RED**

```bash
pixi run colcon test --packages-select pico_odin \
  --pytest-args -k 'wrong_frame or runtime_ros_integration or calibrator_ros_integration' -vv
```

Expected: wrong-frame messages are currently accepted or the expected parameter is absent.

- [ ] **Step 3: Implement strict validation**

Declare and validate a non-empty parameter:

```cpp
expected_skeleton_frame_ =
  declare_parameter<std::string>("expected_skeleton_frame", "pico_ground");
if (expected_skeleton_frame_.empty()) {
  throw std::invalid_argument("expected_skeleton_frame must not be empty");
}
```

At the beginning of both skeleton callbacks, compare `message.header.frame_id`; on mismatch, issue a throttled warning and return before touching receipt timestamps, stationarity state, alignment buffers, or pending queues.

- [ ] **Step 4: Forward the parameter from launch**

Declare `expected_skeleton_frame` with default `pico_ground` in `odin_select.launch.py` and pass it to `odin_pelvis_runtime`. Also correct the accidental concatenated lookup:

```python
"extrinsics_file": LaunchConfiguration("pelvis_extrinsics_file").perform(context),
"expected_skeleton_frame": LaunchConfiguration(
    "expected_skeleton_frame"
).perform(context),
```

- [ ] **Step 5: Run Task 2 tests and verify GREEN**

```bash
pixi run colcon build --packages-select pico_odin
pixi run colcon test --packages-select pico_odin --event-handlers console_direct+
```

Expected: valid canonical-frame tests pass and wrong-frame tests produce no output.

- [ ] **Step 6: Commit Task 2**

```bash
git add src/pico_odin
git commit -m "fix: enforce canonical frame for Odin skeleton inputs"
```

---

### Task 3: Install-safe fail-closed Odin channel profiles

**Files:**
- Create: `src/odin/odin_ros_driver2/include/utility/channel_config.hpp`
- Modify: `src/odin/odin_ros_driver2/src/odin_ros_driver_node.cpp`
- Modify: `src/odin/odin_ros_driver2/CMakeLists.txt`
- Modify: `src/odin/odin_ros_driver2/package.xml`
- Create: `src/odin/odin_ros_driver2/test/test_channel_config.cpp`
- Modify: `src/pico_odin/test/test_launch_integration.py`

**Interfaces:**
- Consumes: a file name relative to `share/odin_ros_driver_rev1/config`, or an absolute YAML path.
- Produces: `ChannelEnableConfig load_channel_enable_config(path)` that returns six required channel booleans or throws `std::runtime_error`.

- [ ] **Step 1: Write failing config-loader tests**

Cover: odometry-only profile enables only odometry; absolute paths are accepted; missing files throw; missing any required key throws; non-scalar/invalid values throw.

```cpp
EXPECT_THROW(
  odin_ros_driver::load_channel_enable_config("/definitely/missing.yaml"),
  std::runtime_error);
```

- [ ] **Step 2: Run the focused test and verify RED**

```bash
pixi run colcon test --packages-select odin_ros_driver_rev1 \
  --ctest-args -R test_channel_config --output-on-failure
```

Expected: the test target/helper does not yet exist.

- [ ] **Step 3: Implement strict loader and package-share lookup**

Use `ament_index_cpp::get_package_share_directory("odin_ros_driver_rev1")` for relative names, preserve absolute paths, require all six channel keys, and propagate errors. `LoadChannelEnableConfig()` assigns members only after a complete config is returned, so partial parsing cannot enable streams.

- [ ] **Step 4: Install configuration and register dependencies/tests**

Add `ament_index_cpp` to `package.xml`/CMake, add the GoogleTest target, and change the install rule to:

```cmake
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})
```

- [ ] **Step 5: Verify source and install-space behavior**

```bash
pixi run colcon build --packages-select odin_ros_driver_rev1 --symlink-install
test -f install/odin_ros_driver_rev1/share/odin_ros_driver_rev1/config/control_command_odom.yaml
pixi run colcon test --packages-select odin_ros_driver_rev1 --event-handlers console_direct+
```

Expected: installed config exists and loader tests pass; a deliberately missing explicit profile exits non-zero before discovery/stream startup.

- [ ] **Step 6: Commit Task 3**

```bash
git add src/odin/odin_ros_driver2 src/pico_odin/test/test_launch_integration.py
git commit -m "fix: load Odin channel profiles from package share"
```

---

### Task 4: Bound point-frame allocation to one logical frame

**Files:**
- Modify: `src/odin/odin-sdk2/sdk/protocol/odinProtocol/OdinFrameAssembler.hpp`
- Modify: `src/odin/odin-sdk2/sdk/protocol/odinProtocol/OdinFrameAssembler.cpp`
- Modify: `src/odin/odin_ros_driver2/test/test_odin_frame_assembler.cpp`

**Interfaces:**
- Consumes: raw or SLAM point fragments.
- Produces: assembler capacity no greater than `MaxPointPayloadBytes(is_slam)` and unchanged oversized-gap drop behavior.

- [ ] **Step 1: Write a failing capacity test**

Declare `OdinFrameAssemblerTestPeer` as a friend of the assembler. Define that peer only in the test file, process the first raw fragment, and assert the raw payload capacity is at most the peer-computed logical-frame byte bound. This keeps the production public API unchanged.

- [ ] **Step 2: Run focused test and verify RED**

```bash
pixi run colcon test --packages-select odin_ros_driver_rev1 \
  --ctest-args -R test_odin_frame_assembler --output-on-failure
```

Expected: current reserve exceeds the logical-frame maximum.

- [ ] **Step 3: Apply the minimal reserve correction**

Replace:

```cpp
state.packet.payload.reserve(expected_packets * kUdpMaxDataSize);
```

with:

```cpp
state.packet.payload.reserve(max_payload_bytes);
```

- [ ] **Step 4: Run assembler tests and verify GREEN**

```bash
pixi run colcon build --packages-select odin_ros_driver_rev1
pixi run colcon test --packages-select odin_ros_driver_rev1 \
  --ctest-args -R test_odin_frame_assembler --output-on-failure
```

- [ ] **Step 5: Commit Task 4**

```bash
git add src/odin/odin-sdk2/sdk/protocol/odinProtocol/OdinFrameAssembler.* \
  src/odin/odin_ros_driver2/test/test_odin_frame_assembler.cpp
git commit -m "fix: bound Odin point-frame preallocation"
```

---

### Task 5: Complete bridge launch interface

**Files:**
- Modify: `src/pico_bridge/src/pico_bridge_node.cpp`
- Modify: `src/pico_bridge/launch/start_pico_bridge.launch.py`
- Create: `src/pico_bridge/test/test_launch_integration.py`
- Modify: `src/pico_bridge/CMakeLists.txt`

**Interfaces:**
- Consumes: launch arguments `smpl_raw_topic`, `smpl_topic`, `world_reset_topic`, `smpl_output_frame`.
- Produces: matching parameters for `pico_bridge_node` and `pico_smpl_ground`.

- [ ] **Step 1: Write failing launch-source tests**

Assert all four declarations exist. Assert the raw and reset topics are passed to the bridge, while all four values are passed to the ground node. Assert defaults are `/pico/smpl_raw`, `/pico/smpl`, `/pico/world_reset`, and `pico_ground`.

- [ ] **Step 2: Run focused test and verify RED**

```bash
pixi run colcon test --packages-select pico_bridge \
  --pytest-args -k launch_integration -vv
```

- [ ] **Step 3: Expose and forward launch values**

Pass:

```python
"raw_topic": LaunchConfiguration("smpl_raw_topic"),
"output_topic": LaunchConfiguration("smpl_topic"),
"world_reset_topic": LaunchConfiguration("world_reset_topic"),
"output_frame": LaunchConfiguration("smpl_output_frame"),
```

and declare the four arguments with the established defaults. In `pico_bridge_node.cpp`, declare `smpl_raw_topic` and `world_reset_topic` parameters, require both to be non-empty, and use them when creating the two publishers. Pass those two launch configurations to the bridge node; pass all four to the ground node.

- [ ] **Step 4: Run Task 5 tests and verify GREEN**

```bash
pixi run colcon build --packages-select pico_bridge
pixi run colcon test --packages-select pico_bridge --event-handlers console_direct+
```

- [ ] **Step 5: Commit Task 5**

```bash
git add src/pico_bridge/src/pico_bridge_node.cpp \
  src/pico_bridge/launch/start_pico_bridge.launch.py \
  src/pico_bridge/test/test_launch_integration.py src/pico_bridge/CMakeLists.txt
git commit -m "fix: expose PICO ground alignment launch topics"
```

---

### Task 6: Low-risk runtime cleanup

**Files:**
- Modify: `src/pico_odin/include/pico_odin/se3.hpp`
- Modify: `src/pico_odin/src/runtime_alignment.cpp`
- Modify: `src/pico_odin/src/odin_pelvis_runtime_node.cpp`
- Modify: `src/pico_odin/test/test_se3.cpp`

**Interfaces:**
- Consumes: `Eigen::Vector3d`.
- Produces: shared `Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d&)`, `SkeletonKind`, and `OdinRate`.

- [ ] **Step 1: Add failing SE(3) helper test**

```cpp
EXPECT_TRUE(
  po::skew_symmetric(Eigen::Vector3d(1.0, 2.0, 3.0)).isApprox(
    (Eigen::Matrix3d() << 0.0, -3.0, 2.0, 3.0, 0.0, -1.0, -2.0, 1.0, 0.0).finished()));
```

- [ ] **Step 2: Run focused test and verify RED**

```bash
pixi run colcon test --packages-select pico_odin \
  --ctest-args -R test_se3 --output-on-failure
```

- [ ] **Step 3: Add shared helper and replace duplicates**

Define the inline helper in `se3.hpp`; remove both anonymous-namespace copies. Rename `raw_pending_`/`raw_skeleton_received_sec_` to `canonical_pending_`/`canonical_skeleton_received_sec_`. Replace cross-cutting `bool fused` and `bool high_frequency` arguments with scoped enums while preserving topics and output behavior.

- [ ] **Step 4: Run `pico_odin` tests and verify GREEN**

```bash
pixi run colcon build --packages-select pico_odin
pixi run colcon test --packages-select pico_odin --event-handlers console_direct+
```

- [ ] **Step 5: Commit Task 6**

```bash
git add src/pico_odin
git commit -m "refactor: clarify Odin runtime stream types"
```

---

### Task 7: Full verification and handoff

**Files:**
- Verify: complete selected workspace
- Modify only if verification exposes a regression in files already covered above.

**Interfaces:**
- Consumes: all Task 1–6 commits.
- Produces: build/test evidence and exact true-hardware commands for the user.

- [ ] **Step 1: Check diff scope and whitespace**

```bash
git status --short
git diff HEAD~6 --check
git diff HEAD~6 --stat
```

- [ ] **Step 2: Run full build**

```bash
pixi run build
```

Expected: all selected packages complete with exit code 0.

- [ ] **Step 3: Run full tests and aggregate results**

```bash
pixi run test
pixi run colcon test-result --test-result-base build --verbose
```

Expected: zero errors, failures, and skipped tests.

- [ ] **Step 4: Verify install artifacts**

```bash
test -f install/odin_ros_driver_rev1/share/odin_ros_driver_rev1/config/control_command_odom.yaml
test -f install/pico_bridge/share/pico_bridge/launch/start_pico_bridge.launch.py
test -f install/pico_odin/share/pico_odin/launch/odin_select.launch.py
```

- [ ] **Step 5: Inspect final history without pushing**

```bash
git log --oneline --decorate -8
git status --short
```

Do not push unless the user separately requests it.
