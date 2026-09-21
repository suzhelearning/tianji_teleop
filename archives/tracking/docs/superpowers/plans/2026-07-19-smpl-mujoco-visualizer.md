# SMPL MuJoCo Geometry Visualizer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a ROS 2 executable that subscribes to `/pico/smpl` and renders a 24-joint MuJoCo geometric skeleton in real time.

**Architecture:** A Python ROS 2 node stores the newest `PoseArray` in a lock-protected buffer while a MuJoCo passive viewer loop renders at a fixed rate. The scene uses free-jointed sphere bodies for joints and capsule bodies for bones; each frame updates their free-joint poses from the configured parent-child topology.

**Tech Stack:** Python 3.11, ROS 2 Humble `rclpy`, `geometry_msgs`, MuJoCo Python bindings, NumPy, `unittest`.

## Global Constraints

- Keep the existing `/pico/smpl` message type (`geometry_msgs/msg/PoseArray`) unchanged.
- Render only geometric joints and bones; do not add an SMPL mesh or dynamics model.
- Default to 24 joints and the topology defined in the approved design.
- Do not modify the existing TCP protocol or `pico_bridge_node` behavior.
- Preserve unrelated untracked user files in the workspace.

## File Map

- Create `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`: ROS node, MuJoCo scene construction, pose conversion, render loop, CLI.
- Create `src/pico_bridge/test/test_smpl_mujoco_geometry.py`: dependency-light tests for topology, transforms, and capsule pose math.
- Modify `src/pico_bridge/CMakeLists.txt`: install the visualizer executable and test file.
- Modify `src/pico_bridge/package.xml`: declare Python runtime dependencies used by the executable.
- Modify `pixi.toml`: add the MuJoCo Python package to the project environment.
- Update `pixi.lock` with `pixi install` after the manifest change.

### Task 1: Add geometry and topology primitives

**Files:**
- Create `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`
- Create `src/pico_bridge/test/test_smpl_mujoco_geometry.py`

**Interfaces:**
- `JOINT_NAMES: tuple[str, ...]` contains the 24 canonical names.
- `BONE_EDGES: tuple[tuple[int, int], ...]` contains 23 `(parent_index, child_index)` edges.
- `transform_position(position: Sequence[float], scale: float, yaw: float) -> numpy.ndarray` returns a transformed 3-vector.
- `capsule_pose(parent: Sequence[float], child: Sequence[float]) -> tuple[numpy.ndarray, numpy.ndarray, float]` returns center, MuJoCo `wxyz` quaternion, and length.

- [x] Write tests covering 24 unique names, 23 edges rooted at Pelvis, identity transform, scale/yaw, zero-length capsule, and a vertical capsule.
- [x] Run the geometry unittest suite and verify it passes.
- [x] Implement constants and pure math functions without importing `rclpy` or `mujoco` at module import time.

### Task 2: Implement the MuJoCo scene and ROS subscriber

**Files:**
- Modify `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`

**Interfaces:**
- `SmplMujocoVisualizer(Node)` subscribes to `geometry_msgs.msg.PoseArray` and exposes `run() -> None`.
- CLI arguments are `--topic`, `--scale`, `--rate`, `--timeout`, and `--yaw`.

- [x] Add lazy imports for `rclpy`, `geometry_msgs`, and `mujoco`, with a clear missing-package error.
- [x] Build the MJCF scene with a ground plane, 24 free-joint spheres, and 23 free-joint capsules.
- [x] Store the latest valid 24-pose message under a lock and record its receive time.
- [x] Update geometry through free-joint `data.qpos` values and call `mujoco.mj_forward`.
- [x] Run ROS spinning in a daemon thread while `mujoco.viewer.launch_passive` owns rendering.
- [x] Apply timeout coloring and throttled warnings for invalid or stale data.
- [x] Keep the viewer responsive and shut down ROS cleanly.

### Task 3: Package and dependency integration

**Files:**
- Modify `src/pico_bridge/CMakeLists.txt`
- Modify `src/pico_bridge/package.xml`
- Modify `pixi.toml`
- Update `pixi.lock`

- [x] Install `smpl_mujoco_visualizer.py` into `lib/pico_bridge` as an executable.
- [x] Install the Python test file with the package test resources.
- [x] Add `rclpy` runtime declaration while retaining existing C++ dependencies.
- [x] Add `mujoco` to Pixi and regenerate the lockfile.
- [x] Build only `pico_bridge` successfully.

### Task 4: End-to-end verification

**Files:**
- No new source files; update documentation only if command names differ from the approved design.

- [x] Verify `ros2 run pico_bridge smpl_mujoco_visualizer --help` lists all five options.
- [x] Start the visualizer without a publisher and verify it remains open without crashing.
- [x] Feed a synthetic 24-pose `PoseArray` directly to the scene update and verify MuJoCo state changes.
- [ ] Verify nonzero `/pico/smpl` rate with real PICO body frames when available.
- [x] Run the existing C++ tests and the new Python geometry tests.
- [x] Run `git diff --check` and inspect the final package status.
