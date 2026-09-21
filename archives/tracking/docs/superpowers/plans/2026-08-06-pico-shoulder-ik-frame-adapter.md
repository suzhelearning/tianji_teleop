# PICO Shoulder IK Frame Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a position-preserving PICO skeleton whose left and right shoulder orientations use the YuShu IK joint-frame convention.

**Architecture:** Keep `/pico/smpl_palm_corrected` unchanged. Derive `/pico/smpl_palm_corrected_ik` from each corrected frame by right-multiplying only the left shoulder by local `Rx(+pi/2)` and the right shoulder by local `Rx(-pi/2)`. The M0 viewer consumes the derived topic so the adjusted axes can be inspected without changing palm geometry.

**Tech Stack:** ROS 2 Humble, Python 3.11, NumPy, pytest, MuJoCo.

## Global Constraints

- Do not modify any joint position.
- Do not modify elbow, wrist, hand, TCP, wrist-pivot, or bone-length semantics.
- Apply the fixed offsets exactly once and in the shoulder-local frame.
- Preserve `/pico/smpl_palm_corrected` for existing consumers.
- Do not enable active control.

---

### Task 1: Pure shoulder-frame adapter

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_skeleton_filter_core.py`
- Test: `src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py`

**Interfaces:**
- Consumes: corrected `(24, 4)` XYZW joint orientations.
- Produces: `adapt_shoulders_to_ik_frames(orientations_xyzw, left_offset_rad, right_offset_rad) -> np.ndarray`.

- [ ] Add a failing test proving identity shoulders become local `Rx(+pi/2)` and `Rx(-pi/2)` while every non-shoulder orientation remains byte-for-byte equivalent.
- [ ] Add a failing test with a non-identity shoulder proving the offset is right-multiplied (`R_out = R_in @ Rx`) rather than left-multiplied.
- [ ] Implement the pure adapter with finite-value, shape, and angle validation.
- [ ] Run `pytest -q src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py` and require zero failures.

### Task 2: Publish the explicit IK-frame topic

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_skeleton_filter_node.py`
- Modify: `src/pico_bridge/launch/start_pico_palm_skeleton_filter.launch.py`
- Test: `src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py`
- Test: `src/pico_bridge/test/test_launch_integration.py`

**Interfaces:**
- Consumes: the fully corrected frame already published on `/pico/smpl_palm_corrected`.
- Produces: `/pico/smpl_palm_corrected_ik` with identical header and positions, and shoulder-only frame offsets.

- [ ] Add failing tests for the new publisher, default topic, default `+pi/2`/`-pi/2` parameters, identical positions, and unchanged original output.
- [ ] Add parameters `ik_output_topic`, `left_shoulder_local_x_offset_rad`, and `right_shoulder_local_x_offset_rad`.
- [ ] Publish a deep-copied, adapted PoseArray after publishing the unchanged corrected PoseArray.
- [ ] Run the node and launch tests and require zero failures.

### Task 3: Select the IK-frame skeleton in M0 viewer

**Files:**
- Modify: `src/pico_bridge/scripts/start_pico_m0_pixi.sh`
- Test: `src/pico_bridge/test/test_pico_m0_scripts.py`

**Interfaces:**
- Consumes: `/pico/smpl_palm_corrected_ik`.
- Produces: the existing MuJoCo viewer with adjusted shoulder axes and unchanged geometry.

- [ ] Add a failing script-contract test requiring the viewer topic to be `/pico/smpl_palm_corrected_ik`.
- [ ] Change only the viewer topic; keep raw overlay, palms, controllers, rate, and offsets unchanged.
- [ ] Run `pytest -q src/pico_bridge/test/test_pico_m0_scripts.py` and require zero failures.

### Task 4: Verification

**Files:**
- Verify only; do not commit or push.

**Interfaces:**
- Consumes: the completed working tree.
- Produces: evidence that the adapter is isolated and regression-free.

- [ ] Run Python compile checks and `bash -n` on modified scripts.
- [ ] Build `pico_bridge` in the clean Pixi environment.
- [ ] Run the complete `pico_bridge` test suite and require zero errors and failures.
- [ ] Run `git diff --check` and inspect `git diff --stat` plus `git status -sb`.
