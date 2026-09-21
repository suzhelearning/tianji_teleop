# PICO Foot IMU Pipeline Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct and verify the complete PICO + two-foot IMU900 calibration, fusion, and MuJoCo visualization pipeline.

**Architecture:** Extract production quaternion/fusion math into a C++ header tested by gtest. Keep `/pico/smpl_fused` globally framed and publish foot-only replacements plus a separate relative-ankle topic. Preserve the reference IMU driver behavior while correcting its runtime profile and readiness/reconnect integration; render foot orientation with explicit MuJoCo foot boxes.

**Tech Stack:** ROS 2 Humble, rclcpp, geometry_msgs, sensor_msgs, std_srvs, Eigen3, gtest, Python 3.11, MuJoCo, Pixi/colcon.

## Global Constraints

- `/pico/smpl` remains unchanged.
- `/pico/smpl_fused` contains 24 global poses and preserves its input header.
- Only foot indices 10 and 11 are replaced; ankle indices 7 and 8 remain PICO poses.
- IMU acceleration is never integrated for position.
- Invalid/non-finite quaternions are rejected.
- Calibration must reproduce the neutral PICO foot pose exactly.
- Hardware success is not claimed without a real PICO and two IMU900 devices.

---

### Task 1: Production C++ fusion math and regression tests

**Files:**
- Create: `src/pico_bridge/include/pico_bridge/foot_imu_fusion_math.hpp`
- Create: `src/pico_bridge/test/test_foot_imu_fusion_math.cpp`
- Modify: `src/pico_bridge/CMakeLists.txt`

**Interfaces:**
- Produces `pico_bridge::fusion::Quaternion`, `Vector3`, `normalize`, `multiply`, `inverse`, `rotate`, `relative_motion`, `aligned_foot_orientation`, and `reconstruct_foot_position`.

- [ ] Write gtests proving zero quaternion rejection, neutral foot pose preservation, known roll/pitch rotation, and local foot-offset reconstruction.
- [ ] Register `test_foot_imu_fusion_math` in CMake and run it to verify RED because the new header does not exist.
- [ ] Implement the header-only math. `aligned_foot_orientation` must return `pelvis * relative_motion(...) * inverse(pelvis0) * foot0`.
- [ ] Run the new gtest and existing `test_pico_frame`; expect all tests to pass.
- [ ] Commit the math and tests.

### Task 2: Correct C++ fusion node and calibration state

**Files:**
- Modify: `src/pico_bridge/src/pico_foot_imu_fusion_node.cpp`
- Create: `src/pico_bridge/test/test_foot_imu_fusion_state.cpp`
- Modify: `src/pico_bridge/CMakeLists.txt`

**Interfaces:**
- Consumes the Task 1 math header.
- Publishes `/pico/smpl_fused` and `/pico/ankle_relative`.

- [ ] Add state tests for fresh-input gating, reset gating, 24-pose validation, neutral calibration, and left/right ankle-relative ordering; verify they fail before refactoring.
- [ ] Replace local quaternion functions with the shared math header and reject invalid PICO/IMU quaternions.
- [ ] Store latest PICO receive time and require PICO plus both IMUs to be younger than `max_imu_age_sec` when calibration starts.
- [ ] Store neutral PICO foot orientations and compute local offsets using `inverse(q_foot0)`.
- [ ] Replace only foot indices 10/11; preserve ankle indices 7/8 and the input header.
- [ ] Publish relative ankle rotations as `inverse(q_knee) * q_foot`, left then right.
- [ ] Add configurable stability thresholds and throttled warnings for uncalibrated, stale, unstable, and invalid states.
- [ ] Run both fusion test executables and commit.

### Task 3: Restore reference-equivalent IMU900 runtime behavior

**Files:**
- Modify: `src/pico_imu900_driver/config/imu900_foot.yaml`
- Modify: `src/pico_imu900_driver/src/imu900_foot_node.cpp`
- Modify: `src/pico_imu900_driver/package.xml`
- Create: `src/pico_imu900_driver/test/test_imu900_profile.py`
- Modify: `src/pico_imu900_driver/CMakeLists.txt`

**Interfaces:**
- Publishes `/imu/left_feet`, `/imu/right_feet`, and `/driver/lower_foot_imu/health`.

- [ ] Add a profile test requiring IDs `[7,6]`, tags `[LL,RL]`, `startup/shutdown_command_append_crlf=false`, and absence of obsolete `left_tag/right_tag/left_topic/right_topic/frame_id` keys; verify RED.
- [ ] Correct YAML command suffix flags and remove obsolete keys.
- [ ] Change package license to Apache-2.0.
- [ ] Start the read/reconnect thread even when the first serial open fails; test this logic through a small extracted retry-state helper or deterministic unit test.
- [ ] Expose yaw-realignment readiness in DriverStatus and require 100 post-reset frames rather than the old 20-frame stable marker.
- [ ] Register tests, build the driver, and commit.

### Task 4: Render visible foot pitch/roll in MuJoCo

**Files:**
- Modify: `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`
- Modify: `src/pico_bridge/test/test_smpl_mujoco_geometry.py`

**Interfaces:**
- Consumes foot orientations at PoseArray indices 10/11.

- [ ] Add failing tests for converting ROS xyzw foot orientation to MuJoCo wxyz and for applying viewer yaw/scale without discarding roll.
- [ ] Store incoming pose orientations as well as positions.
- [ ] Add left/right oriented foot boxes with non-zero length, width, and height; update their free joints from fused foot quaternions.
- [ ] Preserve the existing capsule skeleton and stale coloring.
- [ ] Run geometry tests and commit.

### Task 5: Remove duplicate runtime and integrate build/test/docs

**Files:**
- Delete: `src/pico_bridge/scripts/pico_foot_imu_fusion.py`
- Delete: `src/pico_bridge/test/test_pico_foot_imu_fusion.py`
- Modify: `pixi.toml`
- Modify: `README.md`
- Modify: `docs/PICO_FOOT_IMU_FUSION.md`
- Modify: `docs/superpowers/plans/2026-07-20-pico-foot-imu-fusion.md`

**Interfaces:**
- `pixi run build` and `pixi run test` include `pico_imu900_driver`.

- [ ] Remove the duplicate Python fusion implementation after C++ tests cover its behavior.
- [ ] Add `pico_imu900_driver` to Pixi build/test package lists.
- [ ] Update usage docs with driver reset/readiness, calibration ordering, fused topic semantics, ankle-relative topic, and oriented foot visualization.
- [ ] Update the old plan to record the approved C++/vendored-driver scope change.
- [ ] Run `pixi run build`, `pixi run test`, direct fusion/visualizer tests, and `git diff --check`.
- [ ] Commit and push `feature/pico-foot-imu-fusion` to `personal`.
