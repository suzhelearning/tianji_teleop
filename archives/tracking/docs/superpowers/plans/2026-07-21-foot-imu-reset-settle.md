# Foot IMU Reset Settling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wait one second after both IMU900 zero-Z acknowledgements, discard transition samples, and automatically calibrate from sixty stable PICO/IMU frames.

**Architecture:** Extend the existing generation-aware `AutoCalibrationState` with a settling phase, and let `pico_foot_imu_fusion` own a cancellable ROS wall timer. The timer callback rejects cached IMUs and transitions the same generation to fresh-sample waiting; the existing sampling and fusion path remains unchanged.

**Tech Stack:** ROS 2 Humble, C++17, rclcpp timers/services, ament gtest, Python launch tests.

## Global Constraints

- Hardware reset remains sequential `zero_z_axis`: left ACK, then right ACK.
- `imu_reset_settle_sec` defaults to `1.0` second.
- `calibration_samples` defaults to `60` frames.
- Messages received during settling cannot become calibration inputs.
- Repeated world reset and manual reset cancel the previous generation and timer.
- Manual calibration keeps its existing no-hardware-reset behavior.

---

### Task 1: Model the settling phase

**Files:**
- Modify: `src/pico_bridge/include/pico_bridge/auto_calibration_state.hpp`
- Modify: `src/pico_bridge/test/test_auto_calibration_state.cpp`

**Interfaces:**
- Consumes: existing generation returned by `begin()` and ACKs passed to `accept_zero_ack()`.
- Produces: `settling()`, `finish_settling(generation)`, and `awaiting_fresh_imus()` after a valid timer generation.

- [ ] **Step 1: Write failing tests** proving that both ACKs enter settling rather than fresh-data waiting, that `finish_settling()` advances the current generation, and that a stale generation cannot advance a restarted transaction.
- [ ] **Step 2: Run** `pixi run bash -lc 'colcon test --base-paths src --packages-select pico_bridge --ctest-args -R test_auto_calibration_state --event-handlers console_direct+'` and verify failure.
- [ ] **Step 3: Implement** `AutoCalibrationPhase::kSettling`, make the second ACK enter it, and add generation-checked `finish_settling(std::uint64_t)`.
- [ ] **Step 4: Rebuild and rerun the focused test**, expecting all state tests to pass.
- [ ] **Step 5: Commit** with `test: model IMU reset settling phase`.

### Task 2: Add the cancellable one-second timer

**Files:**
- Modify: `src/pico_bridge/src/pico_foot_imu_fusion_node.cpp`
- Modify: `src/pico_bridge/launch/start_pico_foot_fusion.launch.py`
- Modify: `src/pico_bridge/test/test_imu900_foot_integration.py`

**Interfaces:**
- Consumes: `AutoCalibrationState::settling()` and `finish_settling(generation)`.
- Produces: ROS parameter and launch argument `imu_reset_settle_sec` (`double`, default `1.0`).

- [ ] **Step 1: Add failing integration assertions** for `imu_reset_settle_sec`, default `1.0`, `create_wall_timer`, and launch parameter forwarding.
- [ ] **Step 2: Run** `pixi run python -m unittest src/pico_bridge/test/test_imu900_foot_integration.py -v` and verify failure.
- [ ] **Step 3: Implement** a `rclcpp::TimerBase::SharedPtr` that starts after the right ACK, captures the generation, waits the configured duration, rejects both cached IMUs, then calls `finish_settling(generation)`. Cancel the old timer on repeated world reset, ready failure, and manual reset. Reject negative settle durations during construction.
- [ ] **Step 4: Change the launch default** for `calibration_samples` from `90` to `60`, declare `imu_reset_settle_sec=1.0`, and pass both to the fusion node.
- [ ] **Step 5: Build and run focused tests**, expecting success.
- [ ] **Step 6: Commit** with `feat: settle IMU900 before foot calibration`.

### Task 3: Documentation and full verification

**Files:**
- Modify: `docs/PICO_FOOT_IMU_FUSION.md`
- Modify: `pico_full_test_commands.txt`

**Interfaces:**
- Documents: ACK → 1-second wait → fresh samples → 60-frame calibration.

- [ ] **Step 1: Update user instructions and expected logs** to describe the one-second settle phase and sixty-frame default.
- [ ] **Step 2: Run** `git diff --check`.
- [ ] **Step 3: Run** `pixi run build` and `pixi run test`.
- [ ] **Step 4: Run** `pixi run bash -lc 'source install/setup.bash && ros2 launch pico_bridge start_pico_foot_fusion.launch.py --show-args'` and verify `imu_reset_settle_sec=1.0` and `calibration_samples=60`.
- [ ] **Step 5: Commit** with `docs: update foot IMU settling workflow`.
- [ ] **Step 6: Push** `feature/pico-foot-imu-fusion` to the `personal` remote after all checks pass.
