# PICO Outward-Only Lock Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every PICO teleoperation run use `outward_only` arm redundancy control, independent of CLI or runtime mode toggles, while preserving non-PICO comparison modes.

**Architecture:** Keep all arm-angle modes and QP implementations intact. Enforce the policy only at the Viewer/PICO boundary: coerce startup mode to `outward_only` and ignore arm-angle mode toggles while PICO is configured. Continue transporting and rendering the PICO skeleton.

**Tech Stack:** C++20, MuJoCo Viewer, Python integration test, CTest.

## Global Constraints

- Preserve the current branch and dirty worktree.
- Do not change QP weights, OTG parameters, joint limits, or PICO packet/overlay behavior.
- Do not commit or push.

---

### Task 1: Lock PICO arm-angle mode

**Files:**
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `CMakeLists.txt`
- Modify: `apps/run_qp_ik_viewer.cpp`

**Interfaces:**
- Consumes: `Options::pico_teleop`, `ViewerCommandType::kTogglePicoArmAngleSource`.
- Produces: startup and runtime invariant `arm_angle_reference_mode == ArmAngleReferenceMode::kOutwardOnly` whenever PICO teleoperation is configured.

- [ ] **Step 1: Write the failing integration test**

Add an integration-test option that explicitly requests `pico_outward`, then assert the Viewer summary reports `outward_only`.

- [ ] **Step 2: Run test to verify it fails**

Run the dedicated CTest and expect `arm_angle_mode=pico_outward` instead of `outward_only`.

- [ ] **Step 3: Implement the minimal policy lock**

Select `kOutwardOnly` unconditionally for PICO startup and ignore runtime arm-angle toggles when PICO is configured. Keep existing behavior unchanged outside PICO mode.

- [ ] **Step 4: Update existing PICO integration expectations**

Assert `outward_only` mode and verify telemetry does not report the PICO skeleton as the QP arm-direction source.

- [ ] **Step 5: Run focused and full automated tests**

Build with `cmake --build build -j4`, run the PICO tests serially, then run the full CTest suite.

- [ ] **Step 6: Replay the recorded fast-motion trace**

Generate fresh Cartesian and joint telemetry, run `scripts/check_pico_wrist_stability.py`, and compare the detector result with the prior failing `pico_outward` run.
