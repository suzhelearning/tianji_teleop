# MuJoCo Diagnostic Environment and Ground Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a reference-style MuJoCo scene, lock the physical floor from stable PICO foot poses, and display world plus pelvis coordinate axes without changing Odin transforms.

**Architecture:** Keep the feature in `smpl_mujoco_visualizer.py`, with pure quaternion/sole-height helpers and a thread-safe `GroundPlaneEstimator` separated from ROS/MuJoCo orchestration. ROS callbacks feed the estimator (raw PICO when enabled, otherwise primary); the render loop applies the locked height and renders diagnostic pelvis frames.

**Tech Stack:** Python 3, NumPy, ROS 2 Humble, MuJoCo MJCF, unittest/pytest

## Global Constraints

- Preserve the existing CLI and primary/raw skeleton rendering behavior.
- Prefer raw PICO foot poses for the floor whenever `--show-raw` is enabled.
- Lock from 30 complete two-foot frames with at most `0.02 m` peak-to-peak sole-height variation.
- Hide the floor and world axes before lock; never move them after lock until `/pico/world_reset`.
- Do not change Odin calibration or runtime transforms.
- Leave feature implementation uncommitted for user review.

---

### Task 1: Oriented foot-sole height and stable floor estimator

**Files:**
- Modify: `src/pico_bridge/test/test_smpl_mujoco_geometry.py`
- Modify: `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`

**Interfaces:**
- Produces: `foot_sole_height(position, quaternion_wxyz, half_extents=FOOT_HALF_EXTENTS) -> float`
- Produces: `GroundPlaneEstimator(window_size=30, stability_tolerance=0.02)` with `observe(...)`, `reset()`, and `height`

- [x] **Step 1: Write failing tests** for identity/oriented foot boxes, stable lock, unstable rejection, immutability after lock, and reset.
- [x] **Step 2: Run** `pixi run python -m pytest -q src/pico_bridge/test/test_smpl_mujoco_geometry.py -k 'sole or ground_plane_estimator'` and verify failures are caused by missing interfaces.
- [x] **Step 3: Implement minimal helpers** using the world-Z support radius `sum(abs(R[2, :]) * half_extents)` and a locked rolling-window median estimator.
- [x] **Step 4: Re-run the focused tests** and require zero failures.

### Task 2: Reference environment and diagnostic coordinate frames

**Files:**
- Modify: `src/pico_bridge/test/test_smpl_mujoco_geometry.py`
- Modify: `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`

**Interfaces:**
- Consumes: `_make_xml(show_raw: bool = False) -> str`
- Produces: named `ground_geom`, `world_axis_*`, `pelvis_axis_*`, and optional `raw_pelvis_axis_*` geometries

- [x] **Step 1: Write failing XML tests** for checker texture/material, skybox, light/headlight, initially hidden ground/world axes, and distinct primary/raw pelvis axes.
- [x] **Step 2: Run the XML-focused tests** and verify RED.
- [x] **Step 3: Extend `_make_xml`** with the reference scene assets, lighting, axis bodies, and hidden initial alpha values.
- [x] **Step 4: Re-run all geometry tests** and require zero failures.

### Task 3: ROS reset, source selection, and render integration

**Files:**
- Modify: `src/pico_bridge/test/test_smpl_mujoco_geometry.py`
- Modify: `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`

**Interfaces:**
- Consumes: `/pico/world_reset` as `std_msgs/msg/Float32`
- Consumes: transformed primary/raw foot positions and orientations
- Produces: one locked MuJoCo ground body height and primary/raw pelvis-frame poses

- [x] **Step 1: Write failing behavioral tests** proving raw callbacks feed the floor in overlay mode, primary callbacks feed it otherwise, reset clears it, and scene update applies a locked height once.
- [x] **Step 2: Run the integration-focused unit tests** and verify RED.
- [x] **Step 3: Wire the estimator into callbacks and render state** by subscribing to `/pico/world_reset`, caching MuJoCo IDs, revealing the ground/world axes after lock, and updating pelvis-axis free joints from pose index 0.
- [x] **Step 4: Run the complete viewer test file** and require zero failures.

### Task 4: Project verification

**Files:**
- Verify only; no additional feature scope.

- [x] **Step 1: Run** `pixi run build` and require exit code 0.
- [x] **Step 2: Run** `pixi run test` and require exit code 0.
- [x] **Step 3: Run** `source install/setup.bash && colcon test-result --test-result-base build --verbose` and require zero errors/failures.
- [x] **Step 4: Run** `git diff --check` and inspect the scoped diff.
- [x] **Step 5: Report the real-device behavior** with the exact command and explain floor locking and axis colors without claiming the upstream Odin pose is fixed.
