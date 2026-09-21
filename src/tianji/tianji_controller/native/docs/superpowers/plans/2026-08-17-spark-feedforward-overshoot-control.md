# SPARK Feedforward Overshoot Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce SPARK feedforward reference overshoot without increasing fixed-trace latency beyond 110 ms or changing the Velocity QP and hard constraints.

**Architecture:** Keep the existing Alpha-Beta estimator and jerk-limited reference state. Add source-direction confidence and fast stale-trend release in `acceptTarget()`, then add a target-relative braking envelope around the position correction in `step()`.

**Tech Stack:** C++20, Eigen, yaml-cpp, GoogleTest, CMake/CTest, existing deterministic PICO UDP replay and Python benchmark analyzer.

## Global Constraints

- Keep `spark_upper_qpoases_feedforward_velocity_qp` isolated from all historical algorithms.
- Do not modify Velocity-Level QP, Cartesian Servo, or any joint hard limit.
- Use the fixed `pico_fast_motion_20260812_205428_v4.tjvr` input for before/after comparison.
- Implement tests before production behavior.

---

### Task 1: Define Configuration and Validation

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`
- Test: `tests/test_config.cpp`

**Interfaces:**
- Produces: four scalar members on `SparkFeedforwardVelocityQpConfig`: `source_stationary_velocity_rad_s`, `velocity_reversal_decay`, `velocity_stationary_decay`, and `target_braking_acceleration_scale`.

- [x] Add config-loading assertions and invalid range cases to `tests/test_config.cpp`.
- [x] Run the focused build and confirm the new members initially fail compilation.
- [x] Add defaults, YAML parsing, and validation: threshold must be nonnegative, decay values must be in `[0,1]`, and braking scale must be in `(0,1]`.
- [x] Add the balanced values from the design to `config/qp_ik_pico_teleop.yaml`.
- [x] Rerun the Config tests and confirm they pass.

### Task 2: Release Stale Feedforward on Stop and Reversal

**Files:**
- Modify: `include/tianji_qp_ik/spark_feedforward_reference.hpp`
- Modify: `src/spark_feedforward_reference.cpp`
- Test: `tests/test_spark_feedforward_reference.cpp`

**Interfaces:**
- Produces: `SparkFeedforwardTargetDecision::feedforward_confidence` and per-joint internal `feedforward_confidence_` used by `step()`.

- [x] Add tests that establish positive estimated velocity, then send a stationary and a reversing source sample and assert old-direction feedforward decays immediately.
- [x] Run the focused test and confirm the original estimator fails the new assertions.
- [x] Compute raw source velocity only for valid timestamps; apply reversal decay when its sign conflicts with the previous estimate and stationary decay below the configured threshold.
- [x] Reset confidence with estimator state and preserve it across invalid timing without synthesizing a pulse.
- [x] Rerun the focused test and confirm it passes.

### Task 3: Add Target-Relative Braking Envelope

**Files:**
- Modify: `src/spark_feedforward_reference.cpp`
- Test: `tests/test_spark_feedforward_reference.cpp`

**Interfaces:**
- Consumes: `feedforward_confidence_`, `maximum_acceleration_`, `estimated_q_`, and the existing reference state.
- Produces: bounded `desired_velocity` while retaining the existing hard derivative envelopes.

- [x] Add a fast-approach test with a suddenly stationary target; assert overshoot is substantially reduced and all derivative limits remain satisfied.
- [x] Run the focused test and confirm it fails before the implementation.
- [x] Bound the position-correction speed by `sqrt(2 * braking_scale * a_max * abs(error))`; retain stateful source velocity to avoid a discontinuous feedforward switch.
- [x] Rerun all reference tests and confirm convergence and hard-bound tests still pass.

### Task 4: Fixed-Trace Benchmark and Final Verification

**Files:**
- Modify: `docs/verification/spark_feedforward_velocity_qp_results.md`

**Interfaces:**
- Consumes: fixed trace and existing four-way analysis scripts.
- Produces: recorded amplitude ratio, phase lag, lag-compensated error, derivative statistics, and acceptance decision.

- [x] Build and run the complete focused unit suite.
- [x] Replay the fixed trace into the feedforward mode and regenerate the four-way report.
- [x] Tune only the isolated estimator/reference parameters; reject settings with amplitude outside `0.98--1.01` or lag above 110 ms.
- [x] Update the verification document with exact metrics and accepted parameters.
- [x] Run `ctest --test-dir build --output-on-failure -j1` and confirm all tests pass.
- [x] Check `git diff --check` and ensure `benchmark_results/` remains untracked.
