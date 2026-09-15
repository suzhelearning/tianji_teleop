# Live Headroom, Intent, and Telemetry Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore useful constraint-aware feedforward and reliable stationary hold while making velocity-level dynamic telemetry truthful.

**Architecture:** Preserve the existing controller and QP. Correct Viewer diagnostics at their source, convert instantaneous jerk headroom into a persistent usage envelope, and use low-frequency intent only for hold entry while retaining full-band intent for release.

**Tech Stack:** C++17, Eigen, GoogleTest, Python integration tests, MuJoCo Viewer.

## Global Constraints

- Keep the current branch and worktree.
- Do not change joint position, velocity, acceleration, braking, or jerk limits.
- Do not create a Git commit.
- Do not change fixed-feedforward or non-SPARK algorithms.

---

### Task 1: Velocity-level dynamic telemetry

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- Consumes: `DualArmController::previousAcceleration(ArmSide)` and active `joint_limits`.
- Produces: valid `qddot_max_ratio` and finite velocity-mode jerk bound columns.

- [ ] Add integration assertions that velocity mode reports positive qddot usage during motion and finite jerk bounds.
- [ ] Run the focused integration test and observe RED.
- [ ] Select plot bounds by control level and calculate velocity-level acceleration usage.
- [ ] Re-run the integration test and observe GREEN.

### Task 2: Persistent jerk headroom

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Modify: `include/tianji_qp_ik/spark_constraint_headroom.hpp`
- Modify: `src/spark_constraint_headroom.cpp`
- Modify: `tests/test_config.cpp`
- Modify: `tests/test_spark_constraint_headroom.cpp`

**Interfaces:**
- Produces: `jerk_usage_attack_seconds` and `jerk_usage_release_seconds` with defaults 0.05 and 0.15 s.
- Produces: persistent `jerk_headroom` used by the existing minimum-headroom governor.

- [ ] Add config and governor tests for one-cycle immunity and sustained-saturation reduction; observe RED.
- [ ] Implement parsing, validation, state reset, and exponential jerk-usage filtering.
- [ ] Run focused config/headroom tests and observe GREEN.

### Task 3: Low-frequency hold entry and fast release

**Files:**
- Modify: `include/tianji_qp_ik/spark_palm_twist_estimator.hpp`
- Modify: `src/spark_palm_twist_estimator.cpp`
- Modify: `src/spark_guidance.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Modify: `tests/test_spark_palm_twist_estimator.cpp`
- Modify: `tests/test_spark_guidance.cpp`

**Interfaces:**
- Produces: `SparkPalmTwistDecision::low_frequency_twist`.
- Hold entry consumes low-frequency twist; release consumes full-band twist.

- [ ] Add estimator and guidance tests for noisy stationary entry and prompt corroborated release; observe RED.
- [ ] Expose low-frequency twist and split entry/release predicates.
- [ ] Set entry thresholds to 0.02 m/s and 0.10 rad/s without changing feedforward bandwidth.
- [ ] Run focused tests and observe GREEN.

### Task 4: Resynchronization and replay verification

**Files:**
- Modify only if tests expose a defect: `src/spark_guidance.cpp`
- Generate untracked: `benchmark_results/headroom_live_fix_20260817/`

**Interfaces:**
- Reuses existing model-state reset and 150 ms blend.

- [ ] Extend the resynchronization integration assertion for bounded q reference continuity and cleared feedforward history.
- [ ] Run all 76+ tests and `git diff --check`.
- [ ] Replay all 2134 frames headlessly, compare tracking P95, feedforward availability, solver failures, deadline misses, and all hard bounds.
- [ ] Run MuJoCo visualization if headless acceptance passes.
- [ ] Record results without committing.
