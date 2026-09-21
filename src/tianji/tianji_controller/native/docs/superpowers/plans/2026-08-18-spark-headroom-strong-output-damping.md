# SPARK Headroom Strong Output Damping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Apply the replay-validated `1.0e5` velocity-continuity weight only to the SPARK headroom-feedforward velocity QP and verify it on the complete recorded PICO trajectory with visible MuJoCo playback.

**Architecture:** Keep the algorithm, QP variables, Cartesian servo, SPARK references, and all hard limits unchanged. Change one profile value, protect it with configuration coverage, and compare deterministic replay telemetry against the recorded baseline.

**Tech Stack:** YAML, C++ configuration tests, CMake/CTest, MuJoCo Viewer, TJVR replay, CSV/Numpy analysis.

## Global Constraints

- Change only `spark_headroom_feedforward_velocity_qp.joint_reference_smoothness_weight` from `3.0e4` to `1.0e5`.
- Keep `joint_reference_jerk_smoothness_weight` at `3.0e4`.
- Do not change joint position, velocity, acceleration, jerk, braking, or collision constraints.
- Do not change historical algorithms.
- Do not commit implementation without a separate user request.

---

### Task 1: Profile regression and minimal configuration change

**Files:**
- Modify: `tests/test_config.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`

**Interfaces:**
- Consumes: `loadConfig("config/qp_ik_pico_teleop.yaml")`.
- Produces: a profile whose headroom velocity-continuity weight is exactly `1.0e5` while jerk continuity remains `3.0e4`.

- [x] **Step 1: Add a failing assertion** to the real-profile configuration test requiring `joint_reference_smoothness_weight == 1.0e5` and `joint_reference_jerk_smoothness_weight == 3.0e4`.
- [x] **Step 2: Verify RED** with `cmake --build build -j12 --target test_config && ./build/test_config`; expect the new `1.0e5` assertion to fail against `3.0e4`.
- [x] **Step 3: Make the one-line YAML change** from `3.0e4` to `1.0e5` in the headroom-only section.
- [x] **Step 4: Verify GREEN** with the same `test_config` command.

### Task 2: Full visible replay and quantitative acceptance

**Files:**
- Input: `benchmark_results/pico_live/traces/output_continuity_retest.tjvr`
- Create: `benchmark_results/pico_live/strong_damping_retest.csv`
- Create: `benchmark_results/pico_live/strong_damping_retest_joints.csv`

**Interfaces:**
- Consumes: 9442 v4 TJVR packets over 106.887 seconds.
- Produces: visible MuJoCo playback plus Cartesian and joint telemetry for A/B comparison.

- [x] **Step 1: Launch the visible Viewer** with the headroom-feedforward algorithm, model-state-only control, skeleton overlay, and both telemetry outputs.
- [x] **Step 2: Replay the complete trace** into the Viewer with source timing preserved and wait for clean shutdown.
- [x] **Step 3: Verify runtime integrity**: 9442 datagrams, zero malformed/CRC/reordered packets, zero control failures, and no hard-bound violations.
- [x] **Step 4: Compare against baseline**: report low-intent QP-vs-SPARK acceleration residual, 2.5--5 Hz acceleration energy, jerk saturation ratio, Cartesian position/orientation error, and estimated tracking lag.
- [x] **Step 5: Run focused and full regression tests**, check `git diff --check`, and confirm no Viewer/replay process remains.
