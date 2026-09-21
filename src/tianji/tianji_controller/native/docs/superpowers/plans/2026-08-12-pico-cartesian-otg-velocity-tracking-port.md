# PICO Cartesian OTG Velocity Tracking Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the PICO MuJoCo Viewer use the benchmarked reduced-lag Cartesian Velocity-OTG and timestamp-aware frame-age prediction.

**Architecture:** Port only the clean core OTG/configuration/TargetManager files and their tests into the existing PICO worktree. Enable the opt-in switches only in the PICO profile, preserving Position-OTG as a fallback and leaving PICO transport, arm-angle, QP, telemetry, and Viewer code unchanged.

**Tech Stack:** C++20, Eigen, Ruckig Community, yaml-cpp, GoogleTest, CMake/CTest.

## Global Constraints

- Work only in `/home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1` on `feature/pico-mujoco-teleop-v1`.
- Preserve all pre-existing uncommitted changes and do not use subagents.
- Keep Cartesian acceleration and jerk limits active.
- Keep old Position-OTG selectable by configuration.
- Do not change orientation OTG or either QP formulation.

---

### Task 1: Lock the missing behavior with tests

**Files:**
- Modify: `tests/test_cartesian_otg.cpp`
- Modify: `tests/test_target_manager.cpp`
- Modify: `tests/test_config.cpp`

**Interfaces:**
- Consumes: `CartesianReferenceGenerator::update`, `TargetManager::setManualTarget`, `loadConfig`
- Produces: regression checks for tracking mode, second-order prediction, and PICO configuration

- [x] Add a moving-circle test with `translation_tracking_enabled = true` and `translation_tracking_gain = 30.0`; assert lower synchronous error than Position-OTG and bounded generated acceleration/jerk.
- [x] Add accelerating single-arm and bilateral source-frame tests; assert the between-frame pose uses `p + v*h + 0.5*a*h*h` and the twist uses `v + a*h`.
- [x] Add PICO YAML expectations for enabled tracking, gain `30`, prediction, and horizon `0.015`.
- [x] Build focused tests and verify RED because the new configuration fields do not exist.

### Task 2: Port the reduced-lag reference generator

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `src/cartesian_otg.cpp`

**Interfaces:**
- Produces: `translation_tracking_enabled`, `translation_tracking_gain`, and `translation_stationary_velocity_threshold`

- [x] Add backward-compatible defaults: `false`, `30.0`, and `1.0e-4`.
- [x] Parse optional YAML keys and validate positive gain/threshold.
- [x] Select Velocity mode only for live moving translation targets.
- [x] Command `clampNorm(v_target + gain * position_error, translation_velocity_max)` and retain configured acceleration/jerk bounds.
- [x] Return to Position mode for stationary/stale targets and run the focused OTG/config tests to GREEN.

### Task 3: Port timestamp-aware PICO prediction

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `include/tianji_qp_ik/target_manager.hpp`
- Modify: `src/config.cpp`
- Modify: `src/target_manager.cpp`

**Interfaces:**
- Produces: `translation_prediction_enabled`, `translation_prediction_horizon_seconds`, and per-arm `filtered_acceleration`

- [x] Add defaults `false` and `0.015`, optional YAML parsing, and positive-horizon validation.
- [x] On each strictly newer source timestamp, estimate filtered target acceleration from consecutive filtered linear velocities and clamp its norm.
- [x] Predict only translation position/twist by actual receive age capped by the configured horizon; leave orientation prediction unchanged.
- [x] Run TargetManager tests to GREEN, including PICO's bilateral atomic update path.

### Task 4: Enable and verify the PICO profile

**Files:**
- Modify: `config/qp_ik_pico_teleop.yaml`

**Interfaces:**
- Consumes: all new optional keys
- Produces: live PICO profile with reduced-lag tracking enabled

- [x] Add tracking enabled, gain `30.0`, stationary threshold `1.0e-4`, prediction enabled, and horizon `0.015` under `cartesian_otg`.
- [x] Build with `cmake --build build -j$(nproc)`.
- [x] Run `ctest --test-dir build --output-on-failure -R 'test_(cartesian_otg|target_manager|config)$'`.
- [x] Run the full relevant CTest suite and report the known pre-existing solver crosscheck failure separately.
- [x] Confirm `git diff --check` and verify no pre-existing PICO-modified file was overwritten.

### Task 5: Live A/B handoff

**Files:** none

**Interfaces:**
- Produces: reproducible Viewer and telemetry commands

- [x] Provide the exact build/source/launch/Viewer commands for the PICO worktree.
- [x] Provide old/new A/B switch values and a CSV comparison procedure for dynamic position error, frame age, and receive-to-control latency.
