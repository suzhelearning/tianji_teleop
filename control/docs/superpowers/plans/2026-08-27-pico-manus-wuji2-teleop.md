# PICO + Manus Wuji Hand 2 Teleoperation Implementation Plan

> **For agentic workers:** Implement this plan task-by-task with a fresh test
> cycle at each task. This plan intentionally contains no commit step because
> the user asked to keep all changes uncommitted.

**Goal:** Add Manus-to-Wuji Hand 2 teleoperation to the current PICO arm
controller while preserving the existing arm algorithm and using the official
`/home/zj/git_src/TJ_arm_control/wuji-retargeting` Hand 2 retargeting model,
configuration, and optimizer.

**Architecture:** The Manus collector emits raw skeleton frames together with
semantic chain/joint metadata. A semantic adapter resolves those frames to the
21-point MediaPipe order used by `wuji-retargeting` and publishes the established
ROS2 `/hand_input` contract. An optional ROS2 retarget/output bridge runs the
official Hand 2 `Retargeter`, sends named 20-joint commands over a small binary
UDP protocol, and the existing C++ MuJoCo controller applies those commands to
the combined 54-DOF model by joint name. PICO remains on its current UDP/QP
path.

**Tech Stack:** C++17, MuJoCo, Eigen, GoogleTest, Python 3, NumPy, PyYAML,
Pinocchio, NLopt, ROS2 `rclpy`, `std_msgs/Float32MultiArray`, Manus Integrated
SDK, binary UDP on localhost.

## Global Constraints

- Do not run `git commit`, amend, reset, clean, or discard user changes.
- Build and run the C++ project only with its checked-in Pixi environment.
- Use `models/marvin_m6_wuji2.xml` for the combined MuJoCo model; do not change
  the default PICO model unless explicitly requested.
- Keep the existing seven-joint arm limits, velocities, PICO receiver, and QP/
  IK algorithm flow unchanged.
- Resolve both arm and hand joints by XML/URDF names, never by assumed qpos
  indices.
- The official `wuji-retargeting` Hand 2 configuration and Hand 2 assets are the
  source of truth for retargeting semantics and the 20-joint order.
- A stale or malformed hand frame must never overwrite a newer frame or create
  an out-of-range MuJoCo qpos.

---

### Task 1: Make the Manus raw stream carry stable Hand 2 input semantics

**Files:**
- Modify: `/home/zj/git_src/TJ_arm_control/manus/rawviz.cpp`
- Modify: `/home/zj/git_src/TJ_arm_control/manus/zenoh_pub.py`
- Create: `/home/zj/git_src/TJ_arm_control/manus/tests/test_wuji2_hand_input.py`

**Interfaces:**
- Raw producer adds one backward-compatible metadata record per node:
  `NODE <glove_id> <array_index> <node_id> <parent_id> <chain_type> <side> <finger_joint_type>`.
- Existing `HAND`, `EDGE`, and `POSE` records remain valid for existing tools.
- The semantic adapter consumes node metadata and returns a `(21, 3)` float32
  array in exactly the `wuji_teleop` MediaPipe order.

- [ ] **Step 1: Write failing parser tests.**

  Test a 25-node frame whose numeric node order is deliberately shuffled. The
  expected output must still be `[wrist, thumb mcp/pip/dip/tip, index pip/ip/
  dip/tip, ..., pinky pip/ip/dip/tip]`, with the four non-thumb metacarpal
  nodes omitted. Add tests that duplicate a semantic key and omit a required
  key; both must raise a mapping error.

- [ ] **Step 2: Run the focused test and verify the missing adapter failure.**

  Run:

  ```bash
  python3 -m pytest -q /home/zj/git_src/TJ_arm_control/manus/tests/test_wuji2_hand_input.py
  ```

  Expected result before implementation: import or attribute failure because
  the new semantic adapter does not exist yet.

- [ ] **Step 3: Extend `rawviz.cpp` metadata output.**

  Read `NodeInfo::chainType`, `NodeInfo::side`, and
  `NodeInfo::fingerJointType` from the existing SDK call. Emit one `NODE` line
  for every raw node before the first `POSE` frame. Preserve the raw array index
  so the downstream parser can associate the semantic node with `POSE` values
  without trusting numeric node IDs.

- [ ] **Step 4: Add the Python semantic resolver.**

  Add a pure-Python resolver with these public functions:

  ```python
  def resolve_wuji2_keypoints(
      nodes: list[dict], poses: "np.ndarray"
  ) -> "np.ndarray": ...

  def parse_rawviz_line(line: str) -> dict | None: ...
  ```

  Normalize the SDK enum names/integers to `hand`, `thumb`, `index`, `middle`,
  `ring`, `pinky` and `wrist`, `mcp`, `pip`, `ip`, `dip`, `tip`. Reject duplicate
  or incomplete semantic frames. Apply the existing VUH-to-retarget input
  conversion `(x, y, z) -> (x, -y, z)` exactly once.

- [ ] **Step 5: Extend `zenoh_pub.py` without breaking raw consumers.**

  Store the `NODE` metadata and include a `node_semantics` array in each JSON
  raw-skeleton payload. Leave the existing positions/quaternions/edge keys
  unchanged. Binary mode remains unchanged until a versioned binary format is
  explicitly needed.

- [ ] **Step 6: Run the focused tests and syntax checks.**

  Run the pytest command from Step 2 and:

  ```bash
  python3 -m py_compile /home/zj/git_src/TJ_arm_control/manus/zenoh_pub.py
  ```

  Expected result: all semantic ordering, duplicate, missing, and coordinate
  conversion tests pass.

---

### Task 2: Publish the established ROS2 `/hand_input` contract from Manus

**Files:**
- Create: `/home/zj/git_src/TJ_arm_control/manus/manus_hand_input.py`
- Create: `/home/zj/git_src/TJ_arm_control/manus/tests/test_manus_hand_input_node.py`
- Modify: `/home/zj/git_src/TJ_arm_control/manus/README.md`

**Interfaces:**
- Input: rawviz lines on stdin, including `NODE` and `POSE` records.
- Output: `std_msgs/msg/Float32MultiArray` on `/hand_input`.
- Payload: one hand is 63 floats (`21 x 3`); two hands are 126 floats in
  right-hand then left-hand order, matching `wuji_teleop`.
- Parameters/options: `--right-glove`, `--left-glove`, `--topic`, and
  `--publish-rate-hz`. A frame is published only when all required semantic
  nodes for that side are present.

- [ ] **Step 1: Write failing node-assembly tests.**

  Feed parsed right-only and right-plus-left fixtures into a pure assembly
  function and assert 63/126 payload lengths, right-then-left ordering, and
  latest-frame replacement. Assert that an incomplete side is omitted rather
  than padded with zeros.

- [ ] **Step 2: Verify the tests fail for the missing assembly function.**

  Run:

  ```bash
  python3 -m pytest -q /home/zj/git_src/TJ_arm_control/manus/tests/test_manus_hand_input_node.py
  ```

- [ ] **Step 3: Implement the stdin-to-ROS2 node.**

  Keep stdin parsing and ROS2 publishing separate so the parser/assembler can
  run without `rclpy`. Use sensor-data QoS and publish only the latest complete
  pair. Preserve per-side sequence/timestamp metadata internally for the next
  bridge stage; do not add timestamps into the established float payload.

- [ ] **Step 4: Add launch/usage documentation and run tests.**

  Document the executable pipeline:

  ```bash
  ./rawviz.out | python3 manus_hand_input.py
  ```

  Run the focused pytest and `python3 -m py_compile` on the new node.

---

### Task 3: Expose the official Hand 2 retargeter as a ROS2-to-command bridge

**Files:**
- Create: `/home/zj/git_src/TJ_arm_control/wuji-retargeting/tests/test_tj_wuji2_hand_bridge.py`
- Create: `/home/zj/git_src/TJ_arm_control/wuji-retargeting/example/tj_wuji2_hand_bridge.py`
- Use: `/home/zj/git_src/TJ_arm_control/wuji-retargeting/wuji_retargeting/wuji-description/hand2/hand2_beta1/body/urdf/{left,right}.urdf`

**Interfaces:**
- ROS2 input: `/hand_input`, `Float32MultiArray`, one or two `(21, 3)` sides.
- Retarget core: `wuji_retargeting.Retargeter.from_yaml(...)` with the official
  Hand 2 left/right YAML files and initialized official Hand 2 submodule assets.
- Command output: a pure function returns a `HandCommandFrame` containing
  `sequence`, `source_timestamp_ns`, and two optional 20-element float64 arrays
  in the official Hand 2 joint order.
- UDP output is implemented by a small sender class but is tested separately
  from ROS2 callbacks.

- [ ] **Step 1: Initialize and verify the official retargeting submodule.**

  Run inside the official repository:

  ```bash
  git submodule update --init --recursive
  ```

  Verify that both configured Hand 2 URDF paths exist and that each resolves to
  20 scalar joints. Do not commit the submodule pointer or any generated files.

- [ ] **Step 2: Write failing bridge tests.**

  Test pure helpers for:

  ```python
  split_hand_input(values: Sequence[float]) -> dict[str, np.ndarray]
  clamp_hand_qpos(side: str, qpos: np.ndarray, limits: np.ndarray) -> np.ndarray
  build_hand_command(sequence: int, timestamp_ns: int, values: dict[str, np.ndarray]) -> HandCommandFrame
  ```

  Cover 63/126 lengths, invalid lengths, NaN/Inf rejection, per-side limit
  clipping, and official 20-joint names. Use a deterministic fake retargeter
  only for transport tests; add one integration test that instantiates the
  official Hand 2 `Retargeter` when the initialized assets and Python
  dependencies are available.

- [ ] **Step 3: Verify focused bridge tests fail for missing helpers.**

  Run:

  ```bash
  python3 -m pytest -q /home/zj/git_src/TJ_arm_control/wuji-retargeting/tests/test_tj_wuji2_hand_bridge.py
  ```

- [ ] **Step 4: Implement the bridge around the existing official API.**

  Do not copy the first-generation C++ retarget path from `wuji_teleop`. Load
  the official Hand 2 YAML for each side, call `Retargeter.retarget(points)`,
  preserve the official 20-joint order, and reset the filter on stream gaps.
  Make ROS2 imports optional at module import time so pure helper tests do not
  require a sourced ROS2 shell.

- [ ] **Step 5: Add the UDP sender and launch instructions.**

  Use the command packet layout defined in Task 4. Publish at the configured
  rate, use latest-only input, reject non-monotonic sequences, and stop sending
  a side after the configurable stale timeout. Document:

  ```bash
  ros2 run ... tj_wuji2_hand_bridge --ros-args -p udp_port:=16000
  ```

- [ ] **Step 6: Run focused Python verification.**

  Run the bridge tests, syntax compilation, and an offline fixture through both
  official Hand 2 retargeters. Record any unavailable dependency as an explicit
  environment blocker; do not substitute a system MuJoCo build.

---

### Task 4: Add a versioned C++ Hand 2 UDP protocol and receiver

**Files:**
- Create: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/include/tianji_qp_ik/wuji_hand_teleop_protocol.hpp`
- Create: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/src/wuji_hand_teleop_protocol.cpp`
- Create: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/include/tianji_qp_ik/wuji_hand_udp_receiver.hpp`
- Create: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/src/wuji_hand_udp_receiver.cpp`
- Create: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/tests/test_wuji_hand_teleop_protocol.cpp`
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/CMakeLists.txt`

**Interfaces:**
- Packet magic: ASCII `TJH2`.
- Packet fields: version, declared byte size, side-valid flags, uint64 sequence,
  int64 source timestamp, 20 float64 joint positions for each side in the
  official Hand 2 order, and CRC32.
- Decoder rejects wrong size/magic/version/CRC, non-finite qpos, and invalid
  side flags.
- Receiver stores only the latest accepted frame and exposes atomically safe
  statistics plus a configurable stale timeout.

- [ ] **Step 1: Write failing codec tests.**

  Add round-trip, wrong magic, wrong version, wrong declared size, CRC mismatch,
  NaN, and invalid flag tests. Assert the exact packet size constant and that
  both side arrays survive encode/decode without precision loss.

- [ ] **Step 2: Run the focused C++ test and verify the codec is missing.**

  Configure/build the one test through the project Pixi CMake environment and
  observe the expected missing-header/link failure.

- [ ] **Step 3: Implement codec and receiver.**

  Follow the existing PICO protocol/receiver conventions, but keep this stream
  independent so hand packets cannot affect PICO sequence gates. Use a bounded
  `LatestSpscExchange<HandTeleopFrame>` and reject out-of-order frames.

- [ ] **Step 4: Build and run focused tests with Pixi.**

  Use the project’s `.pixi/envs/default/bin/cmake`, compiler, and MuJoCo paths;
  do not call `/home/zj/.mujoco/...` tools.

---

### Task 5: Map official Hand 2 commands into the combined MuJoCo model

**Files:**
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/include/tianji_qp_ik/mujoco_robot.hpp`
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/src/mujoco_robot.cpp`
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/tests/test_mujoco_robot.cpp`

**Interfaces:**
- Add `HandMapping` with 20 XML joint names, IDs, qpos addresses, and lower/
  upper ranges for each side.
- Add `const HandMapping& handMapping(ArmSide) const noexcept` and
  `void setHandPosition(ArmSide, const Vec20&)`.
- `setHandPosition` validates finite values and clamps only to the MuJoCo XML
  range after the receiver has applied the official retargeting limits.

- [ ] **Step 1: Add failing full-model tests.**

  Load `models/marvin_m6_wuji2.xml`, assert all 40 expected named hand joints
  are resolved, assert left/right mappings are distinct, and assert a command
  changes only the selected hand qpos while arm qpos remain unchanged.

- [ ] **Step 2: Run the focused tests and verify the new API is absent.**

- [ ] **Step 3: Implement name-based mappings and setters.**

  Reuse the existing scalar-joint validation pattern from `buildMapping`; do
  not assume the hand qpos are contiguous with either arm.

- [ ] **Step 4: Run the focused MuJoCo tests with the Pixi MuJoCo library.**

---

### Task 6: Integrate hand commands into the current viewer without changing PICO

**Files:**
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/apps/run_qp_ik_viewer.cpp`
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/include/tianji_qp_ik/telemetry.hpp`
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/config/qp_ik_pico_teleop.yaml`
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/tests/test_pico_teleop_session.cpp`
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/tests/test_mujoco_robot.cpp`

**Interfaces:**
- New options: `--hand-teleop|--no-hand-teleop`, `--hand-bind`, and
  `--hand-port`, with hand teleop disabled by default.
- The control thread applies the latest hand frame after the existing arm
  controller step, forwards MuJoCo, and copies hand qpos into the render
  snapshot. Arm targets, PICO state, and controller references remain unchanged.
- Headless output reports hand datagrams, accepted frames, stale state, and
  per-side sequence numbers.

- [ ] **Step 1: Write failing integration tests.**

  Test option parsing and a headless full-model run with a synthetic hand UDP
  frame. Assert zero hand decode failures, a nonzero selected-hand qpos change,
  unchanged arm mapping, and arm/PICO control failure counters matching the
  no-hand baseline.

- [ ] **Step 2: Run focused tests and capture the expected missing option/
  integration failure.**

- [ ] **Step 3: Integrate receiver construction and lifecycle.**

  Start/stop the hand receiver alongside the existing PICO receiver. Do not
  reuse the PICO port or packet decoder.

- [ ] **Step 4: Apply hand frames by name and add stale handling.**

  On a fresh frame, call `setHandPosition` for valid sides. On timeout, hold the
  last valid qpos and mark the stream stale; never write zeros. On a sequence
  rollback, keep the current command and increment a reject counter.

- [ ] **Step 5: Synchronize render snapshots and status output.**

  Add hand qpos/status to `ViewerSnapshot`; render both hand branches in the
  separate viewer `MujocoRobot` instance.

- [ ] **Step 6: Build and run the focused/headless integration tests with Pixi.**

---

### Task 7: End-to-end replay and regression verification

**Files:**
- Create: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/tests/fixtures/wuji2_hand_input_right.json`
- Create: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/tests/fixtures/wuji2_hand_input_bimanual.json`
- Modify: `/home/zj/git_src/TJ_arm_control/TJ_arm_control/README.md`
- Modify: `/home/zj/git_src/TJ_arm_control/manus/README.md`
- Modify: `/home/zj/git_src/TJ_arm_control/wuji-retargeting/README.md`

- [ ] **Step 1: Add deterministic replay fixtures.**

  Store one neutral and one flexed semantic 21-point frame per side. Fixtures
  must not contain hardware identifiers or calibration secrets.

- [ ] **Step 2: Run the official retargeter offline.**

  Generate the expected 20-joint outputs from the official Hand 2 configurations
  and assert finite, in-range output for both sides.

- [ ] **Step 3: Run the current C++ unit suite and full-model headless test.**

  Verify the default `marvin_m6_qp_pico_fast.xml` path remains unchanged and the
  explicit `--model models/marvin_m6_wuji2.xml` path loads 54 scalar DOFs.

- [ ] **Step 4: Run `git diff --check` and inspect status.**

  Report every modified external-project file and confirm that no commit was
  created.
