# PICO Stream Resynchronization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reject isolated PICO pose jumps while accepting a stable same-epoch pose cluster after three mutually continuous frames and safely resetting Viewer target/OTG history.

**Architecture:** Extend `PicoTeleopStreamGate` with a three-frame candidate-cluster state machine and explicit discontinuity decision. Propagate that discontinuity through the local receiver frame into the existing `PicoTeleopSession::kResetEpochAndApply` path, so Viewer reuses its current TargetManager, OTG, and arm-direction reset behavior. Add receiver and telemetry counters for observable resynchronization without changing UDP V1/V2 packet layouts.

**Tech Stack:** C++20, Eigen, GoogleTest, UDP loopback integration tests, CMake/CTest.

## Global Constraints

- Work only on branch `feature/pico-mujoco-teleop-v1` in the current worktree.
- Preserve all pre-existing uncommitted V2 PICO protocol, arm-angle, MuJoCo, and telemetry changes.
- Keep jump thresholds at `0.15 m` position and `0.60 rad` orientation.
- Require exactly three mutually continuous candidate frames before same-epoch resynchronization.
- Do not alter QP, Cartesian OTG, SDK-limit, or collision-constraint parameters.
- Do not change UDP V1/V2 packet sizes or encoded fields.

---

### Task 1: Candidate-cluster stream gate

**Files:**
- Modify: `include/tianji_qp_ik/pico_teleop_protocol.hpp`
- Modify: `src/pico_teleop_protocol.cpp`
- Test: `tests/test_pico_teleop_protocol.cpp`

**Interfaces:**
- Consumes: `PicoTeleopFrame`, position and orientation jump thresholds.
- Produces: `PicoStreamDecision::stream_discontinuity`; `PicoTeleopFrame::stream_discontinuity`; three-frame candidate state in `PicoTeleopStreamGate`.

- [ ] **Step 1: Write failing gate tests**

Add tests proving: frames 2 and 3 of a stable new cluster are rejected, frame 4 is accepted with `stream_discontinuity`; an internal second jump restarts the three-frame count; return to the old anchor is accepted normally; candidate duplicate sequence is out-of-order; reset/new epoch clears candidates.

- [ ] **Step 2: Run the focused protocol tests and verify RED**

Run: `cmake --build build -j2 --target test_pico_teleop_protocol && ./build/test_pico_teleop_protocol`

Expected: compilation fails because `stream_discontinuity` and candidate-cluster behavior do not exist.

- [ ] **Step 3: Implement the minimal state machine**

Add a three-frame candidate counter. Compare normal frames against the last accepted anchor; compare candidate frames against the previous candidate. Track the latest observed same-epoch sequence even for rejected jump frames. Clear candidate state on normal acceptance, epoch transition, and `reset()`.

- [ ] **Step 4: Run the focused protocol tests and verify GREEN**

Run: `cmake --build build -j2 --target test_pico_teleop_protocol && ./build/test_pico_teleop_protocol`

Expected: all protocol tests pass.

### Task 2: Receiver propagation and observable counter

**Files:**
- Modify: `include/tianji_qp_ik/pico_udp_receiver.hpp`
- Modify: `src/pico_udp_receiver.cpp`
- Test: `tests/test_pico_udp_receiver.cpp`

**Interfaces:**
- Consumes: `PicoStreamDecision::stream_discontinuity`.
- Produces: accepted `PicoTeleopFrame::stream_discontinuity`; `PicoReceiverStats::resynchronizations`.

- [ ] **Step 1: Write a failing UDP loopback test**

Send one accepted anchor followed by three mutually continuous jumped frames. Assert that only the third candidate is published, it has `stream_discontinuity=true`, jump rejection count increases by two, accepted count increases once, and resynchronization count increases once.

- [ ] **Step 2: Run the focused receiver test and verify RED**

Run: `cmake --build build -j2 --target test_pico_udp_receiver && ./build/test_pico_udp_receiver`

Expected: compilation fails because receiver propagation and the counter are absent.

- [ ] **Step 3: Implement receiver propagation**

Before publishing an accepted decoded frame, copy the gate discontinuity flag and increment the atomic resynchronization counter when true. Add the counter to the `stats()` snapshot.

- [ ] **Step 4: Run the focused receiver test and verify GREEN**

Run: `cmake --build build -j2 --target test_pico_udp_receiver && ./build/test_pico_udp_receiver`

Expected: all receiver tests pass.

### Task 3: Session reset classification and Viewer telemetry

**Files:**
- Modify: `src/pico_teleop_session.cpp`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Test: `tests/test_pico_teleop_session.cpp`
- Test: `tests/test_snapshot_exchange.cpp`
- Test: `tests/test_pico_viewer_integration.py`

**Interfaces:**
- Consumes: accepted local frame discontinuity and receiver resynchronization count.
- Produces: `kResetEpochAndApply` for same-epoch discontinuities; CSV column `pico_resynchronizations`.

- [ ] **Step 1: Write failing session and telemetry tests**

Assert a fresh same-epoch frame with `stream_discontinuity=true` classifies as `kResetEpochAndApply`. Extend telemetry snapshot/copy and CSV header expectations with `pico_resynchronizations`.

- [ ] **Step 2: Run focused tests and verify RED**

Run: `cmake --build build -j2 --target test_pico_teleop_session test_snapshot_exchange && ./build/test_pico_teleop_session && ./build/test_snapshot_exchange`

Expected: session discontinuity assertion or telemetry compilation fails.

- [ ] **Step 3: Reuse the existing Viewer reset path**

Treat `frame.stream_discontinuity` like an epoch change in session classification. Copy `receiver.stats().resynchronizations` into snapshot, telemetry sample, CSV header/row, and Viewer diagnostics. Do not add a parallel reset implementation.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run: `cmake --build build -j2 --target test_pico_teleop_session test_snapshot_exchange tianji_qp_ik_viewer && ./build/test_pico_teleop_session && ./build/test_snapshot_exchange`

Expected: focused tests and Viewer build pass.

### Task 4: Regression verification

**Files:**
- Verify only; do not broaden implementation scope.

**Interfaces:**
- Consumes: all previous tasks.
- Produces: fresh evidence that the current dirty-worktree feature set still builds and passes.

- [ ] **Step 1: Run formatting/diff checks**

Run: `git diff --check`

Expected: no whitespace errors.

- [ ] **Step 2: Build all configured targets**

Run: `cmake --build build -j2`

Expected: exit code 0.

- [ ] **Step 3: Run the complete CTest suite**

Run: `ctest --test-dir build --output-on-failure`

Expected: all configured tests pass.

- [ ] **Step 4: Review scope**

Run: `git diff --stat && git status --short --branch`

Expected: implementation touches only the protocol gate, receiver/session propagation, telemetry/Viewer, tests, and this plan in addition to pre-existing user changes.
