# Recorded PICO Direct-QP A/B Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replay one recorded fast PICO trajectory through OTG+velocity-QP and direct velocity-QP, then compare both against the same original Cartesian targets.

**Architecture:** Extend viewer telemetry with observation-only target, Cartesian-reference, and actual TCP poses. Capture the existing ROS bag once through the production PICO bridge, rebase and replay the exact UDP packet stream into two headless viewer runs, and compute metrics from the resulting CSV files. No controller law or safety limit changes.

**Tech Stack:** C++17, Eigen, MuJoCo, ROS 2 rosbag2, Python 3, CSV.

## Global Constraints

- Keep the current branch and worktree.
- Do not use subagents.
- Do not modify controller gains, OTG limits, QP weights, or safety limits.
- Both runs must consume the identical timestamped PICO packet sequence.
- Cartesian errors must use original mapped PICO targets as the common reference.

---

### Task 1: Add common-reference pose telemetry

**Files:**
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Test: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- Consumes: `Pose desired`, `CartesianReference reference`, and `MujocoRobot::tcpPose()` in the control loop.
- Produces: CSV columns for target/reference/actual position XYZ and quaternion XYZW for each arm.

- [ ] Add an integration assertion requiring all new columns.
- [ ] Run the PICO viewer integration test and verify it fails because the columns are absent.
- [ ] Add pose fields to `TelemetrySample`, populate them without changing control flow, and serialize them.
- [ ] Rebuild and rerun the integration test until it passes.

### Task 2: Capture and replay the recorded fast PICO stream

**Files:**
- Input: `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1/recordings/pico_fast_motion_20260812_205428`
- Generate: `/tmp/pico_fast_motion_20260812_205428.tjvr`
- Generate: `/tmp/pico_fast_motion_otg.csv`
- Generate: `/tmp/pico_fast_motion_direct.csv`

**Interfaces:**
- Consumes: production ROS bridge UDP v2 packets.
- Produces: identical packet replay for both viewer runs.

- [ ] Replay the bag through the production bridge once and capture every accepted UDP datagram with relative source timing.
- [ ] Validate frame count, duration, monotonic timestamps, packet size, and CRC.
- [ ] Run headless velocity-QP with the normal OTG configuration.
- [ ] Generate a temporary config differing only in `cartesian_otg.enabled: false` and run direct velocity-QP.

### Task 3: Analyze the fair A/B result

**Files:**
- Generate: `/tmp/pico_fast_motion_direct_vs_otg_summary.csv`

**Interfaces:**
- Consumes: telemetry target/reference/actual poses from Task 1.
- Produces: per-arm RMS/P95/P99/max position and orientation error, lag, slack, active-bound rate, failures, and solver timing.

- [ ] Restrict analysis to the common live PICO sequence interval.
- [ ] Compute original-target-to-actual metrics for both runs.
- [ ] Compute target-to-reference and reference-to-actual decomposition for the OTG run.
- [ ] Verify both runs consumed the same PICO sequence range and report the conclusion.
