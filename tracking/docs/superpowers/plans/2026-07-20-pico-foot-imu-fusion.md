# PICO Foot IMU900 Fusion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fuse PICO `/pico/smpl` with left/right foot IMU900 orientations and publish a calibrated `/pico/smpl_fused` stream suitable for MuJoCo visualization.

**Architecture:** Reuse the IMU900 driver from `/home/zj/sdk_test/catkin_exoskeleton_ws`, which publishes `/imu/left_feet` and `/imu/right_feet`. Add a Python ROS 2 fusion node in `pico_bridge` that uses PICO pelvis/ankle positions, calibrated IMU foot orientations, and the existing quaternion baseline formula. Preserve `/pico/smpl`; publish fused output on a new topic.

**Tech Stack:** Python 3.11, ROS 2 Humble `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_srvs`, NumPy, existing Pixi environment.

> **Scope update (2026-07-21):** The user approved replacing the Python runtime
> with a C++ fusion node and vendoring the reference IMU900 driver with a local
> `DriverStatus` message. The corrective implementation is tracked in
> `2026-07-21-pico-foot-imu-pipeline-fixes.md`; this original plan is retained as
> history rather than the current runtime specification.

## Global Constraints

- Keep `/pico/smpl` unchanged and never overwrite its raw PICO data.
- Publish fused data only as `/pico/smpl_fused` using `geometry_msgs/msg/PoseArray`.
- Treat IMU900 as an orientation source; do not integrate acceleration to estimate foot position.
- Use PICO ankle position and calibrated foot orientation to reconstruct foot position.
- Make left/right IMU mounting corrections independently configurable.
- Use receive-time freshness checks because PICO and IMU900 timestamps have different clock domains.
- Do not edit `/home/zj/sdk_test/catkin_exoskeleton_ws`; reuse its published ROS interfaces only.

## File Map

- Create `src/pico_bridge/scripts/pico_foot_imu_fusion.py`: quaternion math, calibration state, ROS subscriptions/services, fused PoseArray publisher.
- Create `src/pico_bridge/test/test_pico_foot_imu_fusion.py`: dependency-light tests for quaternion baseline fusion, mounting conjugation, foot offset reconstruction, and stale-input rejection.
- Modify `src/pico_bridge/CMakeLists.txt`: install the fusion executable and test resource.
- Modify `src/pico_bridge/package.xml`: declare `rclpy`, `sensor_msgs`, and `std_srvs` runtime dependencies.
- Modify `README.md`: document IMU900 prerequisites, calibration service, and fused visualization command.
- Create `docs/PICO_FOOT_IMU_FUSION.md`: hardware placement, calibration sequence, topic contract, and troubleshooting.

### Task 1: Add pure quaternion fusion primitives with failing tests

**Files:**
- Create `src/pico_bridge/test/test_pico_foot_imu_fusion.py`
- Create `src/pico_bridge/scripts/pico_foot_imu_fusion.py`

**Interfaces:**
- `quat_normalize_xyzw(q) -> numpy.ndarray`
- `quat_multiply_xyzw(a, b) -> numpy.ndarray`
- `quat_inverse_xyzw(q) -> numpy.ndarray`
- `mount_correct_xyzw(q_sensor, q_mount) -> numpy.ndarray`, implementing `q_mount * q_sensor * inverse(q_mount)`
- `relative_from_calibration(q_parent, q_child, q_parent0, q_child0) -> numpy.ndarray`
- `reconstruct_foot_position(ankle_position, foot_orientation, ankle_to_toe_local) -> numpy.ndarray`

- [ ] Write tests for identity calibration, a 90-degree yaw delta, mounting conjugation, foot-offset rotation, and invalid zero-norm quaternions.
- [ ] Run `pixi run python -m unittest src/pico_bridge/test/test_pico_foot_imu_fusion.py -v` and verify it fails because the new functions are absent.
- [ ] Implement only the pure functions, without importing `rclpy` or `sensor_msgs` at module import time.
- [ ] Run the same unittest command and verify all fusion math tests pass.

### Task 2: Implement the ROS2 fusion node

**Files:**
- Modify `src/pico_bridge/scripts/pico_foot_imu_fusion.py`

**Interfaces:**
- Subscribe to `/pico/smpl` with BEST_EFFORT QoS.
- Subscribe to `/imu/left_feet` and `/imu/right_feet` as `sensor_msgs/msg/Imu`.
- Publish `/pico/smpl_fused` as `geometry_msgs/msg/PoseArray` with BEST_EFFORT QoS.
- Provide `/pico_foot_imu_fusion/calibrate` (`std_srvs/srv/Trigger`).
- Provide `/pico_foot_imu_fusion/reset` (`std_srvs/srv/Trigger`).
- Parameters: `pico_topic`, `left_imu_topic`, `right_imu_topic`, `output_topic`, `max_imu_age_sec`, `calibration_samples`, `left_mount_quaternion`, `right_mount_quaternion`, `left_ankle_to_toe_m`, and `right_ankle_to_toe_m`.

- [ ] Add lock-protected latest PICO frame and latest left/right IMU samples with receive monotonic timestamps.
- [ ] Reject PICO frames that do not contain exactly 24 poses and reject IMU samples with invalid quaternions.
- [ ] On calibration service call, require a fresh PICO frame and both fresh IMUs, then collect the configured number of stable samples while the user holds a neutral standing pose.
- [ ] Capture `q_pelvis0`, left/right calibrated foot quaternions, and local ankle-to-toe vectors from the PICO baseline positions.
- [ ] At runtime calculate `q_rel = inverse(q_pelvis) * q_foot * inverse(q_foot0) * q_pelvis0`, then reconstruct global foot orientation and position.
- [ ] Copy the raw PICO PoseArray and replace only indices `7, 8, 10, 11` with fused left/right foot/ankle orientation and reconstructed positions.
- [ ] Publish nothing fused until calibration is complete and both IMUs are fresh; log the reason at a throttled rate.

### Task 3: Package integration and documentation

**Files:**
- Modify `src/pico_bridge/CMakeLists.txt`
- Modify `src/pico_bridge/package.xml`
- Modify `README.md`
- Create `docs/PICO_FOOT_IMU_FUSION.md`

- [ ] Install `pico_foot_imu_fusion.py` as `lib/pico_bridge/pico_foot_imu_fusion`.
- [ ] Install the Python test resource and add runtime dependencies for `rclpy`, `sensor_msgs`, and `std_srvs`.
- [ ] Document that the IMU900 workspace must publish `/imu/left_feet` and `/imu/right_feet` before fusion starts.
- [ ] Document neutral-pose calibration, independent mount quaternions, freshness behavior, and the distinction between foot pose and anatomical ankle pose.
- [ ] Document MuJoCo usage with `--topic /pico/smpl_fused`.

### Task 4: Verification with synthetic and real streams

**Files:**
- No additional source files; update tests only if a verified defect is found.

- [ ] Build `pico_bridge` and run all Python fusion tests plus existing C++ tests.
- [ ] Use a synthetic publisher/fixture to verify identity calibration reproduces PICO poses and a known foot rotation changes the fused foot orientation/position.
- [ ] Verify reset clears calibration and prevents `/pico/smpl_fused` publication until recalibration.
- [ ] Launch the IMU900 driver from the reference workspace and verify both IMU topics publish valid quaternions.
- [ ] Run the complete calibration sequence in a neutral standing pose and verify `/pico/smpl_fused` publishes 24 poses.
- [ ] Run MuJoCo with `--topic /pico/smpl_fused` and compare foot pitch/roll against the physical foot motion.
- [ ] Run `git diff --check`, commit the feature, and push `feature/pico-foot-imu-fusion` to `personal`.
