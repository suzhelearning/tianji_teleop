# PICO TCP Two-stage Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split full per-side TCP calibration into four fixed-point rotation samples for translation and one explicit horizontal-arm reference sample for orientation.

**Architecture:** `PalmTcpCalibrator` becomes a two-stage state machine. The position stage solves and gates only `T_controller_palm.translation`; the orientation stage captures a fresh controller/HMD pair and solves only `T_controller_palm.rotation`, after which the complete candidate is atomically saved. Existing active artifacts are never overwritten by an incomplete or cancelled run.

**Tech Stack:** Python 3.11, ROS 2 Humble/rclpy, NumPy, unittest/pytest, Bash.

## Global Constraints

- The operator selects one side; the opposite straight arm is a physical pose reference only.
- Position uses four samples with a fixed palm reference point and varied controller orientations.
- Orientation uses a fifth explicit SPACE press with both arms forward and horizontal and palms facing each other.
- The fifth sample must not alter the accepted TCP translation.
- A candidate is saved only after both stages pass.
- The independent orientation-only service and script remain available.
- Wrist and arm-geometry artifacts remain valid because only the TCP calibration workflow changes.
- Do not commit these changes.

---

### Task 1: Lock the two-stage state machine with tests

**Files:**
- Modify: `src/pico_bridge/test/test_pico_palm_tcp_calibrator.py`

**Interfaces:**
- Consumes: `PalmTcpCalibrator.capture()` and `solve_tcp_rotation_for_hmd_reference()`.
- Produces: executable behavioral requirements for position acceptance, orientation capture, save, and shutdown.

- [ ] **Step 1: Replace the old fourth-sample auto-exit test**

Create a test that supplies four geometrically valid position samples and verifies that sample four changes the stage to `orientation`, preserves the solved translation, and does not save or shut down.

- [ ] **Step 2: Add the fifth-sample orientation test**

Supply a controller/HMD orientation different from sample four, call `capture()` once more, and verify the saved rotation comes from the fifth sample, the accepted translation is unchanged, and shutdown occurs exactly once.

- [ ] **Step 3: Run the focused test and confirm RED**

Run:

```bash
pixi run python -m pytest -q src/pico_bridge/test/test_pico_palm_tcp_calibrator.py
```

Expected: failure because the current implementation saves and exits on sample four.

---

### Task 2: Implement position-then-orientation TCP calibration

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_tcp_calibrator.py`
- Test: `src/pico_bridge/test/test_pico_palm_tcp_calibrator.py`

**Interfaces:**
- Consumes: four `(R_G_C, p_G_C)` position samples followed by one fresh `(R_G_C, R_G_V)` orientation sample.
- Produces: one complete `pico_palm_tcp_v2` artifact with `T_controller_palm` translation and rotation.

- [ ] **Step 1: Add an explicit calibration stage**

Initialize `_stage = "position"`. In the position stage, collect four samples, run the existing excitation, residual, and TCP-distance gates, store translation and quality metrics, clear position samples, then set `_stage = "orientation"` without saving or shutting down.

- [ ] **Step 2: Capture orientation independently**

In the orientation stage, use the current controller rotation and HMD rotation in `solve_tcp_rotation_for_hmd_reference()`. Do not append the sample to the position solver and do not modify `_tcp_position`.

- [ ] **Step 3: Save only the complete artifact**

After the orientation solution succeeds, set `_orientation_calibrated`, atomically save the candidate, stop the keyboard loop, and shut down rclpy. Add quality metadata `position_sample_count: 4`, `orientation_sample_count: 1`, and `orientation_capture_separate: true` without changing schema version 2.

- [ ] **Step 4: Preserve retry behavior**

If position gates fail, clear only the four rejected samples and remain in the position stage. If orientation capture fails, remain in the orientation stage and retain the accepted in-memory translation. Pressing `q` exits without writing the incomplete candidate.

- [ ] **Step 5: Run focused tests and confirm GREEN**

Run:

```bash
pixi run python -m pytest -q src/pico_bridge/test/test_pico_palm_tcp_calibrator.py
```

Expected: all tests pass.

---

### Task 3: Update operator interaction and documentation

**Files:**
- Modify: `scripts/calibrate_pico_arm.sh`
- Modify: `README.md`
- Modify: `README.zh-CN.md`
- Test: `src/pico_bridge/test/test_pico_calibration_menu.py`

**Interfaces:**
- Consumes: the two-stage terminal prompts emitted by `pico_palm_tcp_calibrator`.
- Produces: a clear Chinese workflow for ordinary operators.

- [ ] **Step 1: Update shell instructions**

Explain that spaces 1–4 calibrate position while the palm reference point stays fixed. Explain that after the position gate passes, the operator should extend both arms forward horizontally with palms facing each other and press SPACE once more to calibrate only the selected side's orientation.

- [ ] **Step 2: Update README workflows**

Remove every statement that sample four also supplies orientation. Document the separate fifth capture and clarify that the opposite arm is not measured by a one-side calibration.

- [ ] **Step 3: Run menu and documentation-related tests**

Run:

```bash
pixi run python -m pytest -q src/pico_bridge/test/test_pico_calibration_menu.py src/pico_bridge/test/test_pico_m0_scripts.py
```

Expected: all tests pass.

---

### Task 4: Build and regression verification

**Files:**
- Verify only; do not modify recordings or active calibration files.

**Interfaces:**
- Consumes: source tree after Tasks 1–3.
- Produces: install-space scripts matching source and a regression result.

- [ ] **Step 1: Build the package**

Run the repository's existing pixi/colcon build command for `pico_bridge`.

- [ ] **Step 2: Run the complete package test suite**

Run all `src/pico_bridge/test` tests and confirm zero failures and zero errors.

- [ ] **Step 3: Check patch hygiene**

Run:

```bash
git diff --check
```

Expected: no output.

- [ ] **Step 4: Report the manual test command**

The handoff must state that `./scripts/calibrate_pico_arm.sh left tcp` now expects five SPACE presses split across the two stages, and that no source commit was created.
