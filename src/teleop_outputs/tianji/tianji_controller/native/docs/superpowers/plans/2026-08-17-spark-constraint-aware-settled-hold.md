# SPARK Constraint-Aware Settled Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop constrained Cartesian endpoint limit cycles when headroom PICO control is stale or stationary without reducing active-motion response.

**Architecture:** Add a per-arm settled-hold state inside `DualArmSparkGuidance`. The state observes raw/SPARK motion and filtered constraint headroom, then overrides only the headroom algorithm's Cartesian reference and posture velocity while preserving the existing velocity QP and hard bounds.

**Tech Stack:** C++20, Eigen, qpOASES, GoogleTest, CMake/CTest, MuJoCo, Python telemetry analysis.

## Global Constraints

- Keep the velocity-QP decision variable and Cartesian servo unchanged.
- Keep joint position, velocity, acceleration, jerk, braking, outward, and collision constraints unchanged.
- Do not change fixed-feedforward or historical algorithm behavior.
- Do not reset QP derivative history when entering hold.
- Work in the current branch and worktree.

---

### Task 1: Configuration and validation

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Test: `tests/test_config.cpp`

**Interfaces:**
- Produces: `settled_hold_enabled`, `settled_hold_dwell_seconds`, and `settled_hold_headroom_enter` in `SparkHeadroomFeedforwardVelocityQpConfig`.

- [x] Add config parsing assertions and invalid-value tests first.
- [x] Run `cmake --build build -j$(nproc) --target test_config && ./build/test_config` and observe RED.
- [x] Add defaults `true`, `0.30`, and `0.05`, YAML parsing, and validation.
- [x] Re-run `test_config` and observe GREEN.

### Task 2: Per-arm settled-hold state machine

**Files:**
- Modify: `include/tianji_qp_ik/spark_guidance.hpp`
- Modify: `src/spark_guidance.cpp`
- Test: `tests/test_spark_guidance.cpp`

**Interfaces:**
- Produces: `SparkSettledHoldReason`, `settled_hold_active`, and `settled_hold_dwell_seconds` in per-arm diagnostics.
- Consumes: raw palm twist, SPARK palm twist, stale state, headroom scale, model TCP pose, and model joint state.

- [x] Add tests for immediate stale entry, delayed exhausted-headroom entry, healthy-headroom rejection, independent-arm state, and two-source release.
- [x] Run the focused tests and observe RED.
- [x] Add per-arm hold state and reset behavior.
- [x] In headroom mode, accumulate dwell in control time while source motion is held; stale enters immediately.
- [x] While holding, emit current model TCP, zero Cartesian twist, and zero posture target while retaining its soft weight.
- [x] On release, restart blend/feedforward from the current model state before accepting the live target.
- [x] Run `test_spark_guidance` and `test_spark_feedforward_reference` and observe GREEN.

### Task 3: Telemetry

**Files:**
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Test: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- Produces CSV fields `<side>_spark_settled_hold_active`, `<side>_spark_settled_hold_dwell_seconds`, and `<side>_spark_settled_hold_reason`.

- [x] Extend the integration test's required CSV columns first and observe RED.
- [x] Copy diagnostics into the Viewer snapshot and CSV writer.
- [x] Run both SPARK feedforward Viewer integration tests and observe GREEN.

### Task 4: Verification and visual replay

**Files:**
- Generate: `benchmark_results/headroom_settled_hold_20260817/`
- Update: `docs/superpowers/specs/2026-08-17-spark-constraint-aware-settled-hold-design.md`

**Interfaces:**
- Consumes: `pico_fast_motion_20260812_205428_v4.tjvr`.
- Produces: main telemetry, joint telemetry, and visual MuJoCo replay.

- [x] Build all targets and run full CTest.
- [x] Replay all 2134 frames with a 35-second Viewer duration.
- [x] Verify final stale two-second `|qdot| < 0.02 rad/s` and `|qddot| < 0.2 rad/s^2` for both arms.
- [x] Compare common live source frames against `headroom_dual_source_hold_20260817`; reject the change if position P95 regresses by more than 5%.
- [x] Record measured results in the design document and run `git diff --check`.
