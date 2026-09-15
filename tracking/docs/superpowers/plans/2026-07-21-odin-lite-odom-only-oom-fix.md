# Odin Lite Odom-Only OOM Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent Odin Lite pelvis calibration/runtime from exhausting host memory and harden point-cloud assembly against impossible UDP gaps.

**Architecture:** Add a dedicated odometry-only channel profile and make `pico_odin` select it for Odin Lite, because pelvis calibration and correction consume only odometry. Independently bound SDK point-cloud frame assembly so malformed or discontinuous packet sequences are dropped instead of producing oversized zero-filled frames.

**Tech Stack:** ROS 2 launch, YAML, C++14, GoogleTest, Odin SDK2.

## Global Constraints

- Preserve the existing full and SLAM profiles for explicit point-cloud use.
- Do not change raw or corrected ROS topic names.
- Do not commit or push the current dirty worktree.
- A malformed point-cloud frame must never allocate beyond one 256x192 frame.

---

### Task 1: Select an odometry-only profile for PICO/Odin pelvis use

**Files:**
- Create: `src/odin/odin_ros_driver2/config/control_command_odom.yaml`
- Modify: `src/pico_odin/launch/odin_select.launch.py`
- Modify: `src/pico_odin/test/test_launch_integration.py`

**Interfaces:**
- Consumes: Odin driver's existing `channel_config_file` parameter.
- Produces: Odin Lite driver with only `enable_odom: 1`; all point, image, and IMU streams disabled.

- [ ] Add a failing launch/config test asserting that Odin Lite selects `control_command_odom.yaml` and that only odometry is enabled.
- [ ] Run the focused pytest and confirm it fails because the profile does not exist and the launch still selects `control_command_slam.yaml`.
- [ ] Add the odometry-only YAML and select it in `odin_select.launch.py`.
- [ ] Run the focused pytest and confirm it passes.

### Task 2: Bound point-cloud UDP loss recovery

**Files:**
- Modify: `src/odin/odin-sdk2/sdk/protocol/odinProtocol/OdinFrameAssembler.hpp`
- Modify: `src/odin/odin-sdk2/sdk/protocol/odinProtocol/OdinFrameAssembler.cpp`
- Create: `src/odin/odin_ros_driver2/test/test_odin_frame_assembler.cpp`
- Modify: `src/odin/odin_ros_driver2/CMakeLists.txt`
- Modify: `src/odin/odin_ros_driver2/package.xml`

**Interfaces:**
- Consumes: fragmented `OdinPointCloudPacket` UDP data.
- Produces: complete frames no larger than 256x192 points; impossible sequence gaps drop and reset the active frame.

- [ ] Add a failing GoogleTest that feeds a packet gap larger than the remaining frame capacity and asserts no oversized callback is emitted.
- [ ] Run the focused test and confirm the current assembler emits an oversized frame.
- [ ] Validate fragment counts, cap payload growth, and reset malformed frames without synthesizing the impossible gap.
- [ ] Run the focused GoogleTest and confirm it passes.

### Task 3: Verification

**Files:**
- Modify: `README.md`
- Modify: `README.zh-CN.md`

**Interfaces:**
- Documents: Odin pelvis launch is odometry-only and point clouds require an explicit different profile.

- [ ] Build all seven ROS 2 packages.
- [ ] Run the complete test suite and require zero failures.
- [ ] Inspect the expanded launch parameters to confirm Odin Lite uses the odometry-only profile.
- [ ] Run the hardware launch while monitoring RSS and confirm memory remains bounded and `/raw/odom/odin_highfreq` publishes.
