# PICO Palm-Constrained Skeleton Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with a verification checkpoint after each task.

**Goal:** Publish a complete 24-joint PICO skeleton whose left/right upper limbs are reconstructed from calibrated TCP palm poses while preserving the original PICO torso, head, and lower body.

**Architecture:** Add a ROS-independent NumPy geometry core, then wrap it in a small `rclpy` node. The node consumes `/pico/smpl_raw` and the two calibrated `/pico/palm_*` topics, automatically learns per-side arm geometry from paired frames, and publishes `/pico/smpl_palm_corrected` in the same `pico` frame. Existing raw, ground-aligned, fused, TCP-calibration, and external exoskeleton paths remain unchanged.

**Tech Stack:** Python 3.11, NumPy, ROS 2 Humble `rclpy`, `geometry_msgs`, `std_msgs`, pytest/ament pytest, Pixi/colcon.

## Global Constraints

- Use `/pico/smpl_raw` because it and `/pico/palm_left/right` are both `frame_id=pico`.
- Never silently combine `pico` with `pico_ground`.
- Preserve all non-arm poses byte-for-byte from the triggering 24-pose input.
- Process left and right arms independently; stale or invalid one-side input falls back to that side's raw four poses.
- HAND indices 22/23 are the calibrated palm targets in the corrected experimental topic.
- Never modify `/pico/smpl_raw`, `/pico/smpl`, `/pico/smpl_fused`, or TCP artifact schema.
- Do not add external exoskeleton/Odin/ESEKF/QP dependencies.
- Do not publish an active-control topic; this is a shadow/visualization experiment only.
- Preserve the user's existing uncommitted TCP calibrator edits.

## File Map

- Create `src/pico_bridge/scripts/pico_palm_skeleton_filter_core.py`: finite NumPy geometry, calibration, triangle elbow solve, and per-side correction.
- Create `src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py`: deterministic unit tests for every core failure and success path.
- Create `src/pico_bridge/scripts/pico_palm_skeleton_filter_node.py`: ROS subscriptions, timestamp pairing, automatic calibration, fallback publication, and JSON status.
- Create `src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py`: node-level conversion and pairing tests using mocked callbacks; no hardware required.
- Modify `src/pico_bridge/CMakeLists.txt`: install the two scripts and register both pytest modules.
- Create `src/pico_bridge/launch/start_pico_palm_skeleton_filter.launch.py`: optional launch wrapper with safe defaults.
- Modify `docs/PICO_Streaming_Guide.md` or add `docs/PICO_PALM_SKELETON_FILTER.md`: build, run, topics, frame contract, and viewer command.

---

### Task 1: Implement and test the pure geometry core

**Files:**
- Create: `src/pico_bridge/scripts/pico_palm_skeleton_filter_core.py`
- Test: `src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py`

**Interfaces:**

```python
@dataclass(frozen=True)
class SideCalibration:
    upper_length_m: float
    forearm_length_m: float
    palm_to_wrist_offset_m: np.ndarray      # legacy position fallback, palm frame
    palm_to_wrist_quat_xyzw: np.ndarray     # legacy/deprecated; never rotates TCP wrist

@dataclass(frozen=True)
class SideCorrection:
    positions: np.ndarray                   # shape (24, 3), copied input with one side changed
    orientations_xyzw: np.ndarray           # shape (24, 4), copied input with one side changed
    reach_clamped: bool
    wrist_palm_position_residual_m: float

def fit_side_calibration(
    skeleton_positions: np.ndarray,       # (N, 24, 3), frame=pico
    skeleton_orientations_xyzw: np.ndarray,# (N, 24, 4)
    palm_positions: np.ndarray,            # (N, 3), frame=pico
    palm_orientations_xyzw: np.ndarray,   # (N, 4)
    side: str,                             # "left" or "right"
) -> SideCalibration:

def correct_side(
    skeleton_positions: np.ndarray,       # (24, 3)
    skeleton_orientations_xyzw: np.ndarray,# (24, 4)
    palm_position: np.ndarray,             # (3,)
    palm_orientation_xyzw: np.ndarray,    # (4,)
    calibration: SideCalibration,
    side: str,
    previous_elbow_position: np.ndarray | None = None,
    ratio_max_stretch: float = 0.995,
    epsilon_m: float = 1.0e-6,
) -> SideCorrection:
```

- [ ] **Step 1: Write failing tests for validation and calibration.**

Add tests that construct 24-pose arrays with identity quaternions and known left/right chains. Assert that `fit_side_calibration` returns the known shoulder-elbow and elbow-wrist lengths, and that malformed shapes, non-finite values, zero quaternions, wrong side names, and fewer than 24 joints raise `ValueError`.

- [ ] **Step 2: Run the focused test and verify it fails.**

Run from the PICO repository root:

```bash
pixi run python -m pytest -q src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py
```

Expected: collection or import failure because the core module does not yet exist.

- [ ] **Step 3: Implement quaternion and calibration primitives.**

Implement normalized xyzw quaternion helpers (`quat_to_matrix`, `matrix_to_quat`, `quat_apply`, `quat_multiply`, `quat_inverse`, `quat_from_two_vectors`) and `fit_side_calibration` using:

```python
upper = median(norm(P[:, shoulder] - P[:, elbow]))
forearm = median(norm(P[:, elbow] - P[:, wrist]))
offset_i = R_H_i.T @ (P[i, wrist] - palm_position[i])
rotation_i = R_H_i.T @ R_W_i
```

Use component-wise medians for offset and hemisphere-aligned normalized quaternion mean for rotation. Reject non-positive lengths and non-finite samples instead of fabricating defaults.

- [ ] **Step 4: Write failing tests for runtime reconstruction.**

Cover: exact palm hand output when reachable, fixed upper/forearm lengths on every frame, palm-local positive-X wrist reconstruction, endpoint residual when the target is infeasible, elbow branch continuity using an original elbow reference, max-reach diagnostics, opposite-vector quaternion handling, left-side-only modification, and preservation of all non-arm indices.

- [ ] **Step 5: Implement `correct_side`.**

Use the fixed palm-to-wrist transform:

```python
p_wrist_requested = p_palm - R_palm @ [wrist_to_palm_distance_m, 0, 0]
R_wrist = R_palm

When no wrist-pivot artifact is available, the paired-skeleton position offset is
allowed as an explicit degraded fallback, but the TCP palm orientation remains
authoritative and is copied unchanged to the reconstructed wrist.
```

Clamp shoulder-to-wrist distance to `[abs(Lu-Lf)+epsilon, (Lu+Lf)*ratio_max_stretch]` for both the artifact and legacy fallback paths. Existing vector artifacts are converted to a scalar distance by their norm; their XYZ direction is never reused. Solve the elbow on the two-sphere intersection circle, rotate shoulder and elbow minimally toward corrected bone directions, set wrist and HAND orientation to the TCP palm orientation, and reconstruct HAND from the final wrist along palm-local positive X. Return the clamp flag and the distance from the reconstructed HAND to the requested TCP palm.

- [ ] **Step 6: Run the focused tests and commit the core.**

Run:

```bash
pixi run python -m pytest -q src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py
```

Expected: all core tests pass. Commit only the new core and core tests:

```bash
git add src/pico_bridge/scripts/pico_palm_skeleton_filter_core.py src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py
git commit -m "feat: add palm-constrained skeleton geometry core"
```

---

### Task 2: Add the ROS 2 filter node and node tests

**Files:**
- Create: `src/pico_bridge/scripts/pico_palm_skeleton_filter_node.py`
- Test: `src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py`

**Interfaces:**

- Inputs: `PoseArray /pico/smpl_raw`, `PoseStamped /pico/palm_left`, `PoseStamped /pico/palm_right`.
- Outputs: `PoseArray /pico/smpl_palm_corrected`, `String /pico/smpl_palm_corrected/status`.
- Parameters: `raw_topic`, `left_palm_topic`, `right_palm_topic`, `output_topic`, `status_topic`, `max_skew_s` default `0.03`, `min_calibration_samples` default `60`, `palm_cache_size` default `120`, `ratio_max_stretch` default `0.995`.

- [ ] **Step 1: Write failing node tests for topic-independent callback behavior.**

Use lightweight fake message objects and call the node's `_raw_callback` and `_palm_callback` directly. Assert:

```python
assert node._nearest_palm_stamp(1.000, "left").stamp == pytest.approx(1.010)
assert node._nearest_palm_stamp(1.000, "left") is None  # when skew > max_skew_s
```

Also assert that a 24-pose raw message is copied before modification, that `pico_ground` is rejected, and that one-side fallback leaves the opposite side and all torso/lower-body poses unchanged.

- [ ] **Step 2: Run the focused node test and verify it fails.**

Run:

```bash
pixi run python -m pytest -q src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py
```

Expected: import failure until the node exists.

- [ ] **Step 3: Implement timestamped palm caches.**

Store each `PoseStamped` as `(stamp_ns, position, quaternion, frame_id)` in a bounded deque. On every raw skeleton callback, select the minimum absolute source-stamp difference and reject it when the difference exceeds `max_skew_s`. Do not substitute receive time. Validate exactly 24 raw poses, `frame_id == "pico"`, finite positions, and normalizable quaternions.

- [ ] **Step 4: Implement automatic per-side baseline collection.**

For each side, collect paired raw skeleton/palm samples until `min_calibration_samples`. Call `fit_side_calibration` when enough samples exist. Publish raw skeleton copies during collection with status `baseline_ready=false`; do not block or hold the stream. Keep the calibration in memory only.

- [ ] **Step 5: Implement continuous correction and independent fallback.**

After a side is ready, call `correct_side`. Start from a full copy of the input PoseArray. Apply only indices `(16,18,20,22)` or `(17,19,21,23)`. Catch geometry errors per side, retain that side's four raw poses, increment a reason counter, and continue publishing the other side and all remaining joints. Keep the previous accepted elbow position per side for branch continuity.

- [ ] **Step 6: Implement status publication.**

Publish compact JSON containing `baseline_ready`, `corrected`, `fallback_reason`, `time_skew_ms`, and `reach_clamped` for both sides. Throttle warning logs, but publish a status message on every raw frame so replay can audit behavior.

- [ ] **Step 7: Run node tests and commit.**

Run:

```bash
pixi run python -m pytest -q src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py
```

Expected: all tests pass. Commit only the node and node tests:

```bash
git add src/pico_bridge/scripts/pico_palm_skeleton_filter_node.py src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py
git commit -m "feat: publish palm-constrained pico skeleton"
```

---

### Task 3: Install, launch, visualize, and run integration checks

**Files:**
- Modify: `src/pico_bridge/CMakeLists.txt`
- Create: `src/pico_bridge/launch/start_pico_palm_skeleton_filter.launch.py`
- Create or modify: `docs/PICO_PALM_SKELETON_FILTER.md`
- Modify: `src/pico_bridge/test/test_launch_integration.py`

- [ ] **Step 1: Add installation and pytest registration.**

Add both scripts to `install(PROGRAMS ...)`, add both Python tests to `install(FILES ...)`, and register both with `ament_add_pytest_test`. Do not change existing executable targets or topic defaults.

- [ ] **Step 2: Add the optional launch file.**

The launch file shall start only `pico_palm_skeleton_filter_node` and expose the parameters listed in Task 2. Defaults must be exactly `/pico/smpl_raw`, `/pico/palm_left`, `/pico/palm_right`, `/pico/smpl_palm_corrected`, and `/pico/smpl_palm_corrected/status`.

- [ ] **Step 3: Add documented run and inspection commands.**

Document:

```bash
ros2 launch pico_bridge start_pico_palm_skeleton_filter.launch.py
ros2 topic hz /pico/smpl_raw
ros2 topic hz /pico/smpl_palm_corrected
ros2 topic echo /pico/smpl_palm_corrected/status --once
```

For MuJoCo comparison, use the existing visualizer with corrected output as primary and raw as overlay:

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_palm_corrected \
  --show-raw --raw-topic /pico/smpl_raw \
  --palm-topic /pico/palm_left \
  --right-palm-topic /pico/palm_right
```

State explicitly that the corrected topic is `pico` frame and must not be mixed with `/pico/smpl` until a separate ground adapter is added.

- [ ] **Step 4: Build and run static/integration checks.**

Run:

```bash
pixi run build
pixi run python -m pytest -q src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py src/pico_bridge/test/test_pico_palm_tcp_calibrator.py
git diff --check
```

Expected: build succeeds, focused tests pass, and no whitespace errors occur. Run existing `test_launch_integration.py` to confirm existing launch defaults remain unchanged.

- [ ] **Step 5: Commit integration packaging and docs.**

```bash
git add src/pico_bridge/CMakeLists.txt src/pico_bridge/launch/start_pico_palm_skeleton_filter.launch.py src/pico_bridge/test/test_launch_integration.py docs/PICO_PALM_SKELETON_FILTER.md
git commit -m "feat: package pico palm skeleton filter"
```

---

### Task 4: Hardware-free replay gate and handoff

- [ ] **Step 1: Run a 1000-frame synthetic replay.**

Feed deterministic generated raw skeleton/palm pairs through the core and assert: no NaN, exactly 1000 outputs, all non-arm poses unchanged, arm lengths within `1e-9 m`, and unclamped HAND position error below `1e-6 m`.

- [ ] **Step 2: Run live PICO shadow validation.**

With the existing bridge and both TCP calibrators publishing, start the new launch, perform neutral standing followed by slow arm forward, side raise, elbow flexion, and hands-together motions. Verify status reaches `baseline_ready=true` independently for both sides, corrected output remains continuous, and raw/corrected/palm markers can be viewed together.

- [ ] **Step 3: Preserve source changes and report artifacts.**

Do not stage or modify the pre-existing dirty TCP calibrator files. Report the corrected topic, status topic, frame contract, build/test results, and any observed side fallback reasons. Do not claim active-control readiness.
