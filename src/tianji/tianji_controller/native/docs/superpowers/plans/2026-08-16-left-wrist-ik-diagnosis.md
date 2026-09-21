# Left Wrist IK Diagnosis and Stabilization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reproduce the left-wrist anomaly from the recorded PICO trace, identify whether J3 saturation causes J5-J7 compensation, and apply the smallest evidence-backed stabilization without weakening Cartesian tracking or the new hard joint limits.

**Architecture:** The existing control loop already produces complete `JointKinematicsSample` values for the in-Viewer plot. Copy each sample into an optional second SPSC queue consumed by a dedicated CSV writer selected with `--joint-telemetry FILE`. A deterministic analysis script compares current J3 hard limits against a temporary symmetric-J3 model and reports wrist discontinuity metrics correlated with J3 boundary activity.

**Tech Stack:** C++20, MuJoCo, Eigen, qpOASES, CMake/CTest, Python 3 CSV analysis.

## Global Constraints

- Keep `Velocity-Level QP` and Cartesian OTG/servo architecture unchanged.
- Keep left J1 >= -1.5708, right J1 <= 1.5708, left J3 <= 0, and right J3 >= 0 in production models.
- Do not reduce Cartesian velocity, acceleration, or jerk limits as a wrist workaround.
- Preserve the current branch and worktree; do not commit or push.
- Do not use subagents.

---

### Task 1: Optional Joint Telemetry CSV

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `CMakeLists.txt`
- Create: `tests/assert_viewer_joint_telemetry.cmake`

**Interfaces:**
- Consumes: existing `JointKinematicsSample` generated once per control cycle.
- Produces: CLI option `--joint-telemetry FILE` and a CSV containing reference/actual q, qdot, qddot, jerk and effective limits for both arms and all seven joints.

- [ ] Add a failing CTest that invokes the Viewer with `--joint-telemetry` and requires finite left J1/right J7 samples and all position/velocity/acceleration/jerk bound columns.
- [ ] Run the focused test and verify it fails because the CLI option is unknown or the CSV is absent.
- [ ] Add `Options::joint_telemetry_path`, parse/help text, a dedicated `BoundedSpscQueue<JointKinematicsSample>`, writer thread, CSV serializer, and clean shutdown/error propagation.
- [ ] Run the focused test and existing joint-plot test until both pass.

### Task 2: Wrist A/B Analyzer

**Files:**
- Create: `scripts/analyze_pico_wrist_ab.py`
- Create: `tests/test_analyze_pico_wrist_ab.py`

**Interfaces:**
- Consumes: current-limit and symmetric-J3 joint telemetry CSV files.
- Produces: JSON metrics for J5-J7 q variation, qdot/qddot/jerk percentiles and maxima, wrap-sized jumps, J3 near-bound cycles, and wrist events correlated within 100 ms of J3 boundary activity.

- [ ] Write a synthetic failing test in which one dataset contains a J3-bound-correlated J6 jump and assert that the analyzer identifies it.
- [ ] Run the focused Python test and verify failure because the analyzer is missing.
- [ ] Implement deterministic CSV parsing and metrics without third-party Python packages.
- [ ] Run the focused Python test and verify it passes.

### Task 3: Recorded PICO Differential Reproduction

**Files:**
- Output: `benchmark_results/wrist_ab/current/`
- Output: `benchmark_results/wrist_ab/symmetric_j3/`
- Output: `benchmark_results/wrist_ab/report.json`

**Interfaces:**
- Consumes: `vr_data/converted_inputs/tjvr/pico_fast_motion_20260812_205428.tjvr`.
- Produces: sequence-compatible current/symmetric-J3 Cartesian and joint telemetry plus an automated red/green diagnostic verdict.

- [ ] Build the Viewer and create a temporary model that changes only left/right J3 back to symmetric ranges.
- [ ] Replay the same trace headlessly with current production limits and capture both telemetry streams.
- [ ] Replay the same trace headlessly with the temporary symmetric-J3 model and capture both telemetry streams.
- [ ] Run the analyzer and rank the causal hypotheses from measured wrist/J3 correlation and Cartesian tracking.

### Task 4: Minimal Evidence-Backed Stabilization

**Files:**
- Modify only the controller/config file implicated by Task 3.
- Test at the real call seam identified by Task 3.

**Interfaces:**
- Consumes: the red A/B reproduction and its dominant causal metric.
- Produces: stable left J5-J7 references while retaining hard J1/J3 limits and Cartesian tracking.

- [ ] Add a failing regression assertion using the recorded trace metric that captures the diagnosed wrist anomaly.
- [ ] Apply one fix only: nearest-branch DLS posture guide if branch selection is causal, or wrist-specific continuity weighting if boundary compensation is causal; do not combine them initially.
- [ ] Re-run the original trace and require reduced wrist discontinuity with no regression in Cartesian P95 tracking or control failures.
- [ ] Run the complete serial CTest suite, XML validation, and `git diff --check`.
