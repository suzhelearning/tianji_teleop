# PICO Elbow IK Frame Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make each IK-output elbow frame use local X along the forearm and local Y as the elbow flexion axis without changing any skeleton position.

**Architecture:** Extend the pure IK-frame adapter so it consumes corrected joint positions and orientations, preserves the existing shoulder basis offsets, and reconstructs only the two elbow quaternions. The geometric flexion-plane normal defines elbow Y when observable, body-left selects its sign, and the previous valid elbow Y preserves the sign when body-left is numerically ambiguous. For straight or nearly straight arms, projected body-left is the deterministic fallback. Only `/pico/smpl_palm_corrected_ik` uses the adapted frames; a degenerate IK frame is skipped without interrupting `/pico/smpl_palm_corrected`.

**Tech Stack:** Python 3.11, NumPy, ROS 2 Humble `geometry_msgs/PoseArray`, pytest/ament.

## Global Constraints

- `/pico/smpl_palm_corrected` must remain byte-semantically unchanged by this adapter.
- All 24 positions in `/pico/smpl_palm_corrected_ik` must equal the corrected input positions exactly.
- Left and right elbow local X must point from elbow to wrist.
- Elbow local Y must represent the shoulder-elbow-wrist flexion axis when that plane is observable.
- Local Z must complete a right-handed orthonormal frame.
- Shoulder local-X offsets remain left `+pi/2` and right `-pi/2`.
- No commit or push is part of this task.

---

### Task 1: Reconstruct IK elbow frames without moving joints

**Files:**
- Modify: `src/pico_bridge/scripts/pico_palm_skeleton_filter_core.py`
- Modify: `src/pico_bridge/scripts/pico_palm_skeleton_filter_node.py`
- Modify: `src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py`
- Modify: `src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py`
- Modify: `README.md`
- Modify: `README.zh-CN.md`

**Interfaces:**
- Consumes: corrected `(24, 3)` positions and `(24, 4)` global xyzw orientations.
- Produces: adapted `(24, 4)` orientations; positions are not returned or mutated.

- [x] **Step 1: Write failing pure-geometry tests**

Add tests proving both elbow local X axes equal normalized `wrist - elbow`, elbow Y matches the observable flexion-plane normal with its sign selected toward body-left, the frame is orthonormal/right-handed, a straight or nearly straight arm uses the projected body-left fallback, and inputs are not mutated.

- [x] **Step 2: Run the focused core test and verify RED**

Run through the clean Pixi/colcon environment. Expected: failure because the current adapter changes shoulders only.

- [x] **Step 3: Implement the minimal pure elbow-frame adapter**

Build X from the forearm and Y from the upper/forearm plane. Select the Y sign against the body-left direction derived from right shoulder to left shoulder. For straight or nearly straight arms, fall back to body-left projected onto the plane normal to the forearm, then compute `Z = X cross Y`.

- [x] **Step 4: Write and run the node-copy test**

Assert all 24 positions and non-arm orientations remain exactly equal while shoulder and elbow orientations use the new IK semantics.

- [x] **Step 5: Wire the adapter into the independent IK output**

Update the node helper only; the primary corrected publisher must remain before and independent from IK adaptation.

- [x] **Step 6: Update documentation and status semantics**

Document shoulder and elbow frame conventions and report `ik_elbow_frame_semantics` in the status payload.

- [x] **Step 7: Verify focused and full regression suites**

Run `git diff --check`, compile checks, shell syntax checks, then the complete `pico_bridge` colcon test suite. Expected: 0 failures.
