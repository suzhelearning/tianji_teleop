# PICO Left-arm Quick Geometry Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a one-person, left-arm-only calibration that estimates individual forearm and upper-arm lengths from neutral-down, straight-forward and static approximately 90-degree elbow poses while eliminating the unknown shoulder anchor, then makes the existing palm-constrained skeleton filter consume the validated lengths.

**Architecture:** A ROS-independent NumPy solver eliminates the unknown shoulder from neutral-down, repeated straight-forward, and static approximately 90-degree palm-derived wrist poses. Raw PICO joints provide diagnostic action-angle evidence only and never define length or veto otherwise valid independent-palm geometry. A guided ROS 2 calibrator time-pairs `/pico/smpl_raw` with `/pico/palm_left`, derives the wrist from the calibrated palm-local +X offset, uses only stable stage tails, archives raw palm pose plus raw/derived wrists, writes an immutable NPZ capture plus v3 YAML candidate, and rejects weak palm geometry. The existing runtime filter optionally loads the accepted left geometry artifact while leaving the right side on its current raw-SMPL baseline path.

**Tech Stack:** Python 3.11, NumPy, ROS 2 Humble/rclpy, geometry_msgs, PyYAML, Pytest, Colcon/Pixi.

## Global Constraints

- Only the left arm participates in calibration and corrected-length activation.
- Controller-to-palm TCP and wrist-to-palm calibration are reused exactly once.
- Wrist position is `p_W = p_P - R_P [d_WP, 0, 0]`; palm-local +X points from wrist to palm.
- Raw PICO shoulder and elbow are weak seeds/branch hints, never accepted bone-length truth.
- Right-arm geometry remains unchanged unless separately calibrated later.
- After preflight, the default launch starts automatically; all stage countdowns, capture, solve, and save are automatic. An explicit `start_mode:=enter` remains available when desired.
- Tracking epoch must be nonzero, explicit, and unchanged throughout capture.
- Candidate artifacts are fail-closed and are never activated when a gate fails.
- Existing dirty changes and recordings are preserved; no Git commit is created in this implementation session.

---

### Task 1: Pure left-arm geometry solver

**Files:**
- Create: `src/pico_bridge/scripts/pico_left_arm_geometry_core.py`
- Create: `src/pico_bridge/test/test_pico_left_arm_geometry_core.py`

**Interfaces:**
- Consumes: straight-stage shoulder/wrist samples and flex-stage shoulder/wrist/raw-elbow samples as finite NumPy arrays.
- Produces: `LeftArmTwoPoseResult` and `solve_left_arm_two_pose_geometry(...)`.

- [ ] Write synthetic failing tests for two-pose recovery, impossible diagonal geometry, wrong elbow pose, uncertainty, and deterministic output.
- [ ] Run `python -m pytest src/pico_bridge/test/test_pico_left_arm_geometry_core.py -q` and confirm missing-module failure.
- [ ] Implement repeated straight-pose consistency, static three-pose equations, raw-joint angle diagnostics, deterministic bootstrap covariance, and fixed independent-palm gates.
- [ ] Re-run the focused test and require all cases to pass.

### Task 2: Artifact contract and runtime override

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_skeleton_filter_node.py`
- Modify: `src/pico_bridge/launch/start_pico_palm_skeleton_filter.launch.py`
- Modify: `src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py`
- Modify: `src/pico_bridge/test/test_launch_integration.py`

**Interfaces:**
- Consumes: `pico_left_arm_geometry_quick_v3.yaml` with `valid: true`, positive revision, left side, exact TCP/wrist-artifact SHA-256 and TCP revision, finite lengths, and passed gates.
- Produces: left `SideCalibration` override and status fields `geometry_source`, `geometry_revision`, `upper_length_m`, and `forearm_length_m`.

- [ ] Add failing loader tests for a valid artifact and rejection of wrong side/schema/invalid status/revision mismatch.
- [ ] Add a failing launch-contract test for `left_arm_geometry_artifact` and `require_left_arm_geometry_artifact`.
- [ ] Implement strict loading and initialize only the left side from the artifact; keep the right baseline-learning behavior unchanged.
- [ ] Run the node and launch test files and require them to pass.

### Task 3: Guided automatic ROS 2 calibrator

**Files:**
- Create: `src/pico_bridge/scripts/pico_left_arm_geometry_calibrator.py`
- Create: `src/pico_bridge/test/test_pico_left_arm_geometry_calibrator.py`
- Modify: `src/pico_bridge/CMakeLists.txt`

**Interfaces:**
- Subscribes: `PoseArray /pico/smpl_raw`, `PoseStamped /pico/palm_left`, `UInt64 /pico/tracking_epoch`, and `String /pico/tracking_epoch/status`.
- Loads: left TCP artifact and left wrist-pivot artifact.
- Writes: `<output-dir>/capture.npz`, `<output-dir>/stage_ranges.yaml`, `<output-dir>/pico_left_arm_geometry_candidate.yaml`, and `<output-dir>/gate_report.json`.

- [ ] Add failing tests for stage sequencing, nearest-time pairing, wrist +X reconstruction, epoch transition rejection, artifact serialization, and rejected-candidate preservation.
- [ ] Implement automatic stages: neutral 3 s, two 6 s static straight reaches, a 7 s static approximately 90-degree elbow pose, straight validation 6 s, and neutral return 3 s. Select the longest suffix satisfying the static-motion gate.
- [ ] Eliminate the unknown shoulder point algebraically: straight-minus-right-angle estimates upper-arm length and right-angle-minus-neutral estimates forearm length; select the closest two of three straight captures and record the discarded outlier.
- [ ] Implement atomic output writes and always preserve the NPZ evidence on solver rejection or interruption.
- [ ] Install the executable and test through CMake.
- [ ] Run focused calibrator tests and require them to pass.

### Task 4: Launch workflow and operator documentation

**Files:**
- Create: `src/pico_bridge/launch/calibrate_pico_left_arm_geometry.launch.py`
- Modify: `README.zh-CN.md`
- Modify: `README.md`
- Modify: `src/pico_bridge/test/test_launch_integration.py`

**Interfaces:**
- Produces one calibration command and one runtime command that loads the resulting left geometry artifact.

- [ ] Add a failing launch test for topic/artifact/output arguments.
- [ ] Add the calibrator launch wrapper and forward typed parameters.
- [ ] Document the exact solo actions, output files, status checks, runtime launch, and MuJoCo comparison command.
- [ ] Run launch integration tests and syntax checks.

### Task 5: Verification gate

**Files:** No production-file changes.

- [ ] Run all `pico_bridge` Python tests.
- [ ] Run `pixi run build` or the repository's package build command for `pico_bridge`.
- [ ] Run `colcon test --packages-select pico_bridge` and inspect `colcon test-result --verbose`.
- [ ] Run `git diff --check` and `python -m py_compile` on new scripts.
- [ ] Confirm the working tree contains no modified recordings and no commit was created.
- [ ] Provide clean-start, calibration, runtime, status, and MuJoCo commands for hardware testing.
