# SPARK qpOASES Direct and Velocity-QP Diagnosis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add isolated direct-qpos and velocity-QP diagnostic modes around the existing SPARK-scaled two-stage qpOASES position IK, then compare both modes with one deterministic PICO trace in MuJoCo.

**Architecture:** Reuse `SparkGuidance` for scaling and two-stage position IK. Add a small policy adapter that either atomically returns the bilateral `q_ik` for direct MuJoCo assignment or converts it to a bounded soft posture velocity for the existing velocity-level QP. Keep all existing control paths and hard-bound values unchanged.

**Tech Stack:** C++17, Eigen3, MuJoCo, Pinocchio, qpOASES, GoogleTest, Python 3 replay/CSV analysis.

## Global Constraints

- Keep the current branch and normal checkout.
- Do not use subagents.
- Do not commit or push unless the user explicitly requests it.
- Keep every existing algorithm selectable and behaviorally unchanged.
- Do not change joint position, velocity, acceleration, jerk, or braking values.
- Do not command physical hardware.

---

### Task 1: Algorithm identities and mode policy

**Files:**
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `tests/test_config.cpp`

**Interfaces:**
- Produces: `IkAlgorithm::kSparkUpperQpoasesDirect`
- Produces: `IkAlgorithm::kSparkUpperQpoasesVelocityQp`
- Produces parsing names `spark_upper_qpoases_direct` and `spark_upper_qpoases_velocity_qp`.

- [ ] Add config tests asserting parse, stringify, and Spark-mode classification for both names.
- [ ] Run `pixi run cmake --build build --target test_config -j4 && pixi run ctest --test-dir build -R '^test_config$' --output-on-failure`; verify RED because the enum names do not exist.
- [ ] Add both enum identities and precise predicates distinguishing direct-qpos from velocity-QP operation.
- [ ] Re-run the focused test and verify GREEN.

### Task 2: Direct bilateral q_ik adapter

**Files:**
- Create: `include/tianji_qp_ik/spark_qpoases_diagnostic.hpp`
- Create: `src/spark_qpoases_diagnostic.cpp`
- Create: `tests/test_spark_qpoases_diagnostic.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: accepted left/right `SparkGuidanceArmDiagnostics::q_ik`.
- Produces: `SparkQpoasesDirectCommand { bool accepted; Vec7 left; Vec7 right; std::string_view detail; }`.
- Produces: `makeDirectCommand(const SparkGuidanceDiagnostics&)` with atomic bilateral acceptance and finite-value checks.

- [ ] Add a test proving bilateral accepted finite q_ik is returned unchanged and either-arm rejection/non-finite data produces hold.
- [ ] Build/run `test_spark_qpoases_diagnostic`; verify RED because the adapter is absent.
- [ ] Implement only validation and atomic command extraction; do not add filtering or bounds that alter q_ik.
- [ ] Re-run the focused test and verify GREEN.

### Task 3: Velocity-QP posture adapter

**Files:**
- Modify: `include/tianji_qp_ik/spark_qpoases_diagnostic.hpp`
- Modify: `src/spark_qpoases_diagnostic.cpp`
- Modify: `tests/test_spark_qpoases_diagnostic.cpp`

**Interfaces:**
- Produces: `makePostureVelocity(q_ik, q_model, gain, velocity_limits)` returning `Kq*(q_ik-q_model)` element-wise clamped to existing velocity limits.

- [ ] Add tests for exact proportional output, positive/negative clamping, invalid inputs, and zero gain rejection.
- [ ] Run the focused test and verify RED for the missing function.
- [ ] Implement the minimal bounded posture-velocity conversion.
- [ ] Re-run the focused test and verify GREEN.

### Task 4: Viewer, MuJoCo direct assignment, and telemetry

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Direct mode: accepted bilateral q_ik is written atomically to both MuJoCo arm positions; rejected frames hold.
- Velocity-QP mode: existing Cartesian target/servo/QP remains active and receives the bounded Spark posture velocity as its soft posture reference.
- Both modes publish q_ik, commanded q, successive `delta_q_ik`, solver acceptance, and both skeleton overlays.

- [ ] Add headless integration assertions that direct mode reports no velocity-QP updates and that velocity-QP mode reports active posture guidance.
- [ ] Run both integration invocations and verify RED because the CLI names are unsupported.
- [ ] Wire direct mode before Cartesian controller execution, preserving atomic bilateral hold and reset synchronization.
- [ ] Wire velocity-QP mode through the existing Spark posture input without changing its Cartesian objective or hard constraints.
- [ ] Re-run both integration tests and existing three Spark-mode integration tests; verify GREEN.

### Task 5: Deterministic comparison and MuJoCo visualization

**Files:**
- Modify: `scripts/compare_spark_guided_velocity_qp.py`
- Modify: `tests/test_compare_spark_guided_velocity_qp.py`
- Create: `docs/verification/spark_qpoases_direct_velocity_qp_diagnosis.md`
- Generate: `benchmark_results/spark_qpoases_direct_velocity_qp_diagnosis/*`

**Interfaces:**
- Consumes the same v4 trace and identical initial model/configuration for both modes.
- Produces Cartesian residuals, q_ik discontinuity statistics, q/qdot/qddot/jerk audits, solver status, and cycle timing.

- [ ] Add analyzer tests for direct-mode q_ik continuity and mode-specific activity fields; verify RED.
- [ ] Implement the minimum analyzer extension and verify GREEN.
- [ ] Build all targets and run the full CTest suite with `--output-on-failure`.
- [ ] Replay `pico_fast_motion_20260812_205428_v4.tjvr` headlessly for direct mode; reject non-finite commands or malformed packets.
- [ ] If direct mode is valid, replay velocity-QP mode with the same trace and initial state.
- [ ] Run sequential graphical MuJoCo replays with corrected-PICO and processed-SPARK skeleton overlays.
- [ ] Record evidence and interpretation without claiming the velocity QP is causal unless direct mode is normal and the filtered mode degrades.
- [ ] Run `git diff --check` and verify no Viewer/replay process remains.
