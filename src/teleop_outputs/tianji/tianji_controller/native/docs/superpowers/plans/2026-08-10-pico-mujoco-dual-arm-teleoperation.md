# PICO corrected-IK to Tianji MuJoCo dual-arm teleoperation implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive both Tianji MuJoCo TCP targets from the corrected PICO palm skeleton with absolute 1:1 shoulder-frame mapping, atomic UDP transport, and stable 200 Hz Cartesian OTG/QP control.

**Architecture:** A C++ ROS 2 node in `PICO_tracker` pairs corrected skeleton and status messages, maps both calibrated PICO hand poses into the fixed Tianji shoulder-midpoint frame, and sends one versioned 160-byte UDP datagram per accepted source frame. A separate Tianji UDP thread decodes and validates the latest bilateral frame, hands it to the 200 Hz control thread through an independent overwrite-capable three-buffer SPSC exchange, and updates both TargetManager sides atomically before the existing Cartesian OTG and velocity/acceleration QP path.

**Tech Stack:** C++17, ROS 2 Humble/rclcpp, geometry_msgs, std_msgs, Eigen 3.4, yaml-cpp, POSIX UDP sockets, MuJoCo, Ruckig Cartesian OTG, qpOASES, GoogleTest, CTest, colcon, Pixi.

## Global Constraints

- Work only in Tianji branch `feature/pico-mujoco-teleop-v1` at `/home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1` and PICO branch `feature/tianji-mujoco-teleop-v1` at `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1`.
- Do not modify or switch the existing base, velocity-OTG, acceleration-OTG, or `feat/pico-palm-tcp-v2` worktrees.
- Consume only `/pico/smpl_palm_corrected_ik` plus matching `/pico/smpl_palm_corrected/status`; reject raw or partially corrected fallback arms.
- Use SMPL indices `SPINE2=6`, `LEFT_SHOULDER=16`, `RIGHT_SHOULDER=17`, `LEFT_HAND=22`, and `RIGHT_HAND=23`.
- Map with `T_W_target = T_W_M * inverse(T_G_S) * T_G_hand`, where `o_M=(0,0,1.121) m`, `R_W_M=Identity`, and position scale is exactly `1.0`.
- Use the PICO hand quaternion directly after the shoulder-frame transform. Do not add clutching, first-frame orientation alignment, current-robot pose substitution, or a hand-to-TCP orientation offset.
- Encode one atomic bilateral UDP packet per accepted source frame; default destination/bind endpoint is `127.0.0.1:15000`.
- Keep ROS 2 out of the Tianji build and keep UDP off the QP/control thread.
- Preserve the existing UI SPSC queue as single-producer; UDP uses a separate latest-only SPSC exchange.
- Estimate twist only on accepted new source timestamps after shoulder mapping. Keep the existing 15 Hz filter and do not add a second feedforward path when Cartesian OTG is enabled.
- Use a 50 ms external-target timeout, freeze the last pose, zero target twist, and let OTG decelerate smoothly.
- Keep both velocity and acceleration QP modes available through the existing `V` and `A` Viewer controls.
- This feature remains MuJoCo-only and must not add a real-robot output path.

---

## File map

### PICO repository

- `src/pico_bridge/include/pico_bridge/tianji_teleop_geometry.hpp`: ROS-independent SMPL shoulder-frame and bilateral hand mapping contract.
- `src/pico_bridge/src/tianji_teleop_geometry.cpp`: exact shoulder-frame construction, degeneracy checks, and absolute 1:1 mapping.
- `src/pico_bridge/include/pico_bridge/tianji_teleop_protocol.hpp`: status, matched-frame, wire-frame, packet constants, CRC, and encoder interfaces.
- `src/pico_bridge/src/tianji_teleop_protocol.cpp`: strict status parsing, frame matching, explicit little-endian encoding, and CRC32.
- `src/pico_bridge/src/tianji_mujoco_teleop_bridge_node.cpp`: thin rclcpp subscriber/UDP publisher and diagnostics node.
- `src/pico_bridge/launch/start_tianji_mujoco_teleop.launch.py`: bridge launch arguments and node wiring.
- `src/pico_bridge/test/test_tianji_teleop_geometry.cpp`: geometry, common-body-motion cancellation, and exact orientation tests.
- `src/pico_bridge/test/test_tianji_teleop_protocol.cpp`: status pairing, validity, golden bytes, CRC, sequence, and cache tests.
- `src/pico_bridge/test/test_tianji_mujoco_teleop_bridge_runtime.py`: ROS-to-UDP runtime test with paired PoseArray/status messages.
- `src/pico_bridge/test/test_launch_integration.py`: launch contract assertions.
- `src/pico_bridge/CMakeLists.txt`, `src/pico_bridge/package.xml`, `pixi.toml`: dependencies, targets, installs, and tests.
- `README.md`, `README.zh-CN.md`: bridge launch, topic, endpoint, and diagnostics instructions.

### Tianji repository

- `include/tianji_qp_ik/pico_teleop_protocol.hpp`: decoded target frame, decode errors, stream gate, and fixed packet contract.
- `src/pico_teleop_protocol.cpp`: little-endian decoder, CRC/quaternion checks, sequence/epoch/jump gate.
- `include/tianji_qp_ik/pico_udp_receiver.hpp`: receiver options, statistics snapshot, and receiver lifecycle.
- `src/pico_udp_receiver.cpp`: blocking socket thread, latest-only publication, and atomic statistics.
- `include/tianji_qp_ik/pico_teleop_session.hpp`: control-thread enable/epoch/freshness state machine.
- `src/pico_teleop_session.cpp`: deterministic classification of received frames into ignore/apply/reset-and-apply actions.
- `include/tianji_qp_ik/target_manager.hpp`, `src/target_manager.cpp`: strict atomic bilateral timestamped target update.
- `include/tianji_qp_ik/config.hpp`, `src/config.cpp`: jump-limit configuration.
- `include/tianji_qp_ik/telemetry.hpp`, `src/telemetry.cpp`: PICO state and transport counters in snapshots/telemetry.
- `apps/run_qp_ik_viewer.cpp`: CLI, receiver lifecycle, control-loop application, `P` toggle, overlay, CSV, and headless behavior.
- `config/qp_ik_pico_teleop.yaml`: high-response acceleration profile with 50 ms timeout.
- `tests/test_pico_teleop_protocol.cpp`, `tests/test_pico_udp_receiver.cpp`, `tests/test_pico_teleop_session.cpp`: transport and state tests.
- `tests/test_pico_viewer_integration.py`: synthetic UDP-to-headless-Viewer test in both velocity and acceleration QP modes.
- `tests/test_target_manager.cpp`, `tests/test_config.cpp`, `tests/test_snapshot_exchange.cpp`: atomic update/config/SPSC regressions.
- `tests/assert_viewer_config.cmake`, `CMakeLists.txt`, `pixi.toml`, `README.md`: build/test wiring and user commands.

---

### Task 1: Implement the PICO shoulder-frame and direct palm mapping core

**Repository:** `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1`

**Files:**
- Create: `src/pico_bridge/include/pico_bridge/tianji_teleop_geometry.hpp`
- Create: `src/pico_bridge/src/tianji_teleop_geometry.cpp`
- Create: `src/pico_bridge/test/test_tianji_teleop_geometry.cpp`
- Modify: `src/pico_bridge/CMakeLists.txt`
- Modify: `src/pico_bridge/package.xml`

**Interfaces:**
- Consumes: 24 finite PICO poses represented by `PicoSkeletonFrame`.
- Produces: `PicoShoulderMapResult map_pico_palms_to_tianji(const PicoSkeletonFrame&)` with fixed robot midpoint, exact PICO hand rotations, and explicit rejection reasons.

- [ ] **Step 1: Add the failing geometry tests and test target**

Define the public types in the test before implementation expectations:

```cpp
using PicoSkeletonFrame = std::array<PicoSkeletonPose, 24>;

PicoSkeletonFrame neutralSkeleton()
{
  PicoSkeletonFrame frame{};
  for (auto & pose : frame) {
    pose.orientation = Eigen::Quaterniond::Identity();
  }
  frame[6].position = {0.0, 0.0, 1.0};
  frame[16].position = {0.0, 0.2, 1.5};
  frame[17].position = {0.0, -0.2, 1.5};
  frame[22].position = {0.5, 0.4, 1.4};
  frame[23].position = {0.5, -0.4, 1.4};
  return frame;
}

TEST(TianjiTeleopGeometry, MapsShoulderMidpointToFixedRobotMidpointAtUnitScale)
{
  const auto result = map_pico_palms_to_tianji(neutralSkeleton());
  ASSERT_TRUE(result.valid) << result.rejection_reason;
  EXPECT_TRUE(result.left_target.translation().isApprox(
    Eigen::Vector3d(0.5, 0.2, 1.021), 1e-12));
  EXPECT_TRUE(result.right_target.translation().isApprox(
    Eigen::Vector3d(0.5, -0.2, 1.021), 1e-12));
}

TEST(TianjiTeleopGeometry, UsesPicoHandOrientationWithoutOffset)
{
  auto frame = neutralSkeleton();
  frame[22].orientation = Eigen::Quaterniond(
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()));
  frame[23].orientation = Eigen::Quaterniond(
    Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ()));
  const auto result = map_pico_palms_to_tianji(frame);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.left_target.linear().isApprox(
    frame[22].orientation.toRotationMatrix(), 1e-12));
  EXPECT_TRUE(result.right_target.linear().isApprox(
    frame[23].orientation.toRotationMatrix(), 1e-12));
}
```

Also test rigidly rotating/translating every PICO pose leaves both mapped targets unchanged, shoulder separation below `0.10 m` rejects with `shoulder_separation_out_of_range`, and collinear SPINE2/shoulders rejects with `spine_direction_degenerate`.

- [ ] **Step 2: Run the focused build and verify the test fails**

Run:

```bash
pixi run build-core
```

Expected: compilation fails because `pico_bridge/tianji_teleop_geometry.hpp` or `map_pico_palms_to_tianji` does not exist.

- [ ] **Step 3: Implement the minimal geometry core**

Expose exactly:

```cpp
inline constexpr std::size_t kPicoSmplJointCount = 24;
inline constexpr std::size_t kPicoSpine2 = 6;
inline constexpr std::size_t kPicoLeftShoulder = 16;
inline constexpr std::size_t kPicoRightShoulder = 17;
inline constexpr std::size_t kPicoLeftHand = 22;
inline constexpr std::size_t kPicoRightHand = 23;

struct PicoSkeletonPose
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

using PicoSkeletonFrame = std::array<PicoSkeletonPose, kPicoSmplJointCount>;

struct PicoShoulderMapResult
{
  bool valid{false};
  std::string rejection_reason;
  Eigen::Isometry3d pico_shoulder_frame{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d left_target{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_target{Eigen::Isometry3d::Identity()};
};

PicoShoulderMapResult map_pico_palms_to_tianji(
  const PicoSkeletonFrame & skeleton);
```

Implement these exact equations and constants:

```cpp
const Eigen::Vector3d origin = 0.5 * (left_shoulder + right_shoulder);
const Eigen::Vector3d y_axis = (left_shoulder - right_shoulder).normalized();
const Eigen::Vector3d z_raw = origin - spine2;
Eigen::Vector3d z_axis = z_raw - y_axis * y_axis.dot(z_raw);
z_axis.normalize();
Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized();
z_axis = x_axis.cross(y_axis).normalized();

Eigen::Isometry3d T_G_S = Eigen::Isometry3d::Identity();
T_G_S.linear().col(0) = x_axis;
T_G_S.linear().col(1) = y_axis;
T_G_S.linear().col(2) = z_axis;
T_G_S.translation() = origin;

Eigen::Isometry3d T_W_M = Eigen::Isometry3d::Identity();
T_W_M.translation() = Eigen::Vector3d(0.0, 0.0, 1.121);

result.left_target = T_W_M * T_G_S.inverse() * T_G_left_hand;
result.right_target = T_W_M * T_G_S.inverse() * T_G_right_hand;
```

Normalize accepted quaternions, reject norms below `1e-9`, require shoulder separation in `[0.10, 0.60] m`, require projected spine norm at least `0.05 m`, and require rotation determinant positive with orthogonality error at most `1e-9`.

- [ ] **Step 4: Run the focused geometry test**

Run:

```bash
pixi run build-core
pixi run bash -lc './build/pico_bridge/test_tianji_teleop_geometry'
```

Expected: all `TianjiTeleopGeometry` tests pass.

- [ ] **Step 5: Commit the geometry core**

```bash
git add src/pico_bridge/CMakeLists.txt src/pico_bridge/package.xml \
  src/pico_bridge/include/pico_bridge/tianji_teleop_geometry.hpp \
  src/pico_bridge/src/tianji_teleop_geometry.cpp \
  src/pico_bridge/test/test_tianji_teleop_geometry.cpp
git commit -m "feat: map PICO palms into Tianji shoulder frame"
```

### Task 2: Add strict corrected-status pairing and the PICO UDP encoder

**Repository:** `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1`

**Files:**
- Create: `src/pico_bridge/include/pico_bridge/tianji_teleop_protocol.hpp`
- Create: `src/pico_bridge/src/tianji_teleop_protocol.cpp`
- Create: `src/pico_bridge/test/test_tianji_teleop_protocol.cpp`
- Modify: `src/pico_bridge/CMakeLists.txt`
- Modify: `src/pico_bridge/package.xml`

**Interfaces:**
- Consumes: `PicoSkeletonFrame`, corrected-status JSON, source timestamp, and mapped `PicoShoulderMapResult` from Task 1.
- Produces: `CorrectedIkStatus`, `TianjiTeleopBridgeCore`, `TianjiTeleopWireFrame`, `encode_tianji_teleop_packet`, and the exact 160-byte protocol consumed by Tianji Task 4.

- [ ] **Step 1: Write failing status, pairing, and golden-packet tests**

Use this exact valid status fixture:

```cpp
const std::string status_json = R"({
  "tracking_epoch":9,
  "stream_valid":true,
  "source_frame_id":"pico",
  "source_stamp_ns":11,
  "ik_frame_valid":true,
  "left":{"corrected":true},
  "right":{"corrected":true}
})";
```

Assert malformed JSON, zero epoch, wrong frame, false stream/IK validity, either uncorrected side, and a status stamp without a cached skeleton produce no frame and an exact rejection reason. Assert status-before-skeleton and skeleton-before-status both produce exactly one frame when the second half arrives; a repeated pair never sends twice; an eight-frame cache evicts its oldest stamp.

Use this fixed encoder fixture:

```text
sequence=7
tracking_epoch=9
source_timestamp_ns=11
bridge_send_monotonic_ns=13
flags=15
left=(1,2,3, 0,0,0,1)
right=(-1,-2,-3, 0,0,1,0)
```

The complete expected packet hex is:

```text
544a56520100a000070000000000000009000000000000000b000000000000000d000000000000000f000000000000000000f03f00000000000000400000000000000840000000000000000000000000000000000000000000000000000000000000f03f000000000000f0bf00000000000000c000000000000008c000000000000000000000000000000000000000000000f03f00000000000000003740fb80
```

Assert size `160`, magic bytes `TJVR`, version `1`, encoded size `160`, required flags `0x0f`, and CRC32 `0x80fb4037`.

- [ ] **Step 2: Run the focused build and verify it fails**

Run:

```bash
pixi run build-core
```

Expected: compilation fails because `tianji_teleop_protocol.hpp` and its functions are missing.

- [ ] **Step 3: Implement strict parser, matcher, and encoder**

Expose these signatures:

```cpp
inline constexpr std::size_t kTianjiTeleopPacketSize = 160;
inline constexpr std::uint16_t kTianjiTeleopProtocolVersion = 1;
inline constexpr std::uint32_t kTianjiTeleopRequiredFlags = 0x0f;

struct CorrectedIkStatus
{
  bool valid{false};
  std::string rejection_reason;
  std::int64_t source_stamp_ns{0};
  std::uint64_t tracking_epoch{0};
};

struct TianjiTeleopWireFrame
{
  std::uint64_t sequence{0};
  std::uint64_t tracking_epoch{0};
  std::int64_t source_timestamp_ns{0};
  std::int64_t bridge_send_monotonic_ns{0};
  Eigen::Isometry3d left_target{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_target{Eigen::Isometry3d::Identity()};
};

struct TianjiTeleopBridgeOutcome
{
  std::optional<TianjiTeleopWireFrame> frame;
  std::string rejection_reason;
};

CorrectedIkStatus parse_corrected_ik_status(std::string_view json_text);

class TianjiTeleopBridgeCore
{
public:
  explicit TianjiTeleopBridgeCore(std::size_t cache_capacity = 8);
  TianjiTeleopBridgeOutcome ingest_skeleton(
    std::int64_t source_stamp_ns, const PicoSkeletonFrame & skeleton);
  TianjiTeleopBridgeOutcome ingest_status(const CorrectedIkStatus & status);
};

std::uint32_t tianji_teleop_crc32(const std::uint8_t * data, std::size_t size);
std::array<std::uint8_t, kTianjiTeleopPacketSize> encode_tianji_teleop_packet(
  const TianjiTeleopWireFrame & frame);
```

Parse JSON with `YAML::Load` because JSON is valid YAML and yaml-cpp is already a project dependency. Validate all named fields explicitly. Store at most eight skeletons and eight statuses by `source_stamp_ns`; only erase a pair after matching or definitive rejection. Increment sequence only when a valid bilateral frame is produced.

Encode fields at fixed offsets `0,4,6,8,16,24,32,40,44,68,100,124,156`; write each integer and IEEE-754 double explicitly little-endian; compute IEEE CRC32 over bytes `[0,156)` and write it at offset `156`.

- [ ] **Step 4: Run the protocol tests**

Run:

```bash
pixi run build-core
pixi run bash -lc './build/pico_bridge/test_tianji_teleop_protocol'
```

Expected: all status, pairing, cache, golden-byte, and CRC tests pass.

- [ ] **Step 5: Commit the protocol core**

```bash
git add src/pico_bridge/CMakeLists.txt src/pico_bridge/package.xml \
  src/pico_bridge/include/pico_bridge/tianji_teleop_protocol.hpp \
  src/pico_bridge/src/tianji_teleop_protocol.cpp \
  src/pico_bridge/test/test_tianji_teleop_protocol.cpp
git commit -m "feat: encode atomic Tianji teleop packets"
```

### Task 3: Publish matched PICO targets through the ROS 2 UDP bridge

**Repository:** `/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1`

**Files:**
- Create: `src/pico_bridge/src/tianji_mujoco_teleop_bridge_node.cpp`
- Create: `src/pico_bridge/launch/start_tianji_mujoco_teleop.launch.py`
- Create: `src/pico_bridge/test/test_tianji_mujoco_teleop_bridge_runtime.py`
- Modify: `src/pico_bridge/test/test_launch_integration.py`
- Modify: `src/pico_bridge/CMakeLists.txt`
- Modify: `README.md`
- Modify: `README.zh-CN.md`

**Interfaces:**
- Consumes: ROS PoseArray/status topics and Task 2 `TianjiTeleopBridgeCore`/encoder.
- Produces: `tianji_mujoco_teleop_bridge`, UDP datagrams, `/pico/tianji_mujoco_teleop/status`, and launch file parameters.

- [ ] **Step 1: Add failing launch and runtime tests**

Extend launch tests to require exactly these arguments and defaults:

```python
assert {
    "skeleton_topic",
    "status_topic",
    "diagnostics_topic",
    "destination_address",
    "destination_port",
    "cache_capacity",
} <= arguments

assert defaults == {
    "skeleton_topic": "/pico/smpl_palm_corrected_ik",
    "status_topic": "/pico/smpl_palm_corrected/status",
    "diagnostics_topic": "/pico/tianji_mujoco_teleop/status",
    "destination_address": "127.0.0.1",
    "destination_port": "15000",
    "cache_capacity": "8",
}
```

The runtime test binds a localhost UDP socket on an ephemeral port, launches the bridge against unique ROS topics, publishes one valid 24-pose skeleton and matching valid status, then asserts one 160-byte datagram arrives with sequence `1`, epoch/source stamp preserved, and no second datagram arrives without a new source pair. Publish an uncorrected-left status at the next stamp and assert no datagram. Also assert no datagram for a PoseArray whose `frame_id != "pico"`, pose count is not exactly 24, timestamp is non-positive, pose contains a non-finite scalar, or quaternion norm is below `1e-9`.

- [ ] **Step 2: Run launch/runtime tests and verify failure**

Run:

```bash
pixi run build-core
pixi run bash -lc 'source install/setup.bash && pytest -q \
  src/pico_bridge/test/test_launch_integration.py \
  src/pico_bridge/test/test_tianji_mujoco_teleop_bridge_runtime.py'
```

Expected: tests fail because the launch file and executable do not exist.

- [ ] **Step 3: Implement the thin C++ ROS/UDP node**

Use a single-threaded rclcpp executor and these callback rules:

```cpp
void on_skeleton(const geometry_msgs::msg::PoseArray & message)
{
  const std::int64_t stamp_ns =
    static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(message.header.stamp.nanosec);
  const auto frame = convert_pose_array(message);
  handle_outcome(core_.ingest_skeleton(stamp_ns, frame));
}

void on_status(const std_msgs::msg::String & message)
{
  handle_outcome(core_.ingest_status(parse_corrected_ik_status(message.data)));
}

void handle_outcome(const TianjiTeleopBridgeOutcome & outcome)
{
  if (!outcome.frame.has_value()) {
    record_rejection(outcome.rejection_reason);
    return;
  }
  TianjiTeleopWireFrame frame = *outcome.frame;
  frame.bridge_send_monotonic_ns = monotonic_now_ns();
  const auto packet = encode_tianji_teleop_packet(frame);
  const ssize_t sent = sendto(
    socket_fd_, packet.data(), packet.size(), 0,
    reinterpret_cast<const sockaddr *>(&destination_), sizeof(destination_));
  record_send(sent == static_cast<ssize_t>(packet.size()), frame);
}
```

Use sensor-data QoS depth one for PoseArray and reliable depth ten for status, matching their existing publishers, and a one-second diagnostics timer only for JSON status. The pose path remains callback-driven with no pose publishing timer. Before handing a skeleton to the core, require `header.frame_id == "pico"`, exactly 24 poses, a positive source timestamp, finite position/quaternion components, and quaternion norm at least `1e-9`; normalize only accepted quaternions. Validate destination IPv4 and port `[1,65535]` at startup. Close the exact socket in the destructor.

- [ ] **Step 4: Run PICO bridge tests and package tests**

Run:

```bash
pixi run build-core
pixi run bash -lc 'source install/setup.bash && colcon test \
  --base-paths src --packages-select pico_bridge --event-handlers console_direct+'
pixi run bash -lc 'colcon test-result --test-result-base build/pico_bridge --verbose'
```

Expected: bridge runtime/launch tests pass and all existing pico_bridge tests pass in the repository-standard Pixi/install layout.

- [ ] **Step 5: Commit the PICO bridge**

```bash
git add src/pico_bridge/CMakeLists.txt \
  src/pico_bridge/src/tianji_mujoco_teleop_bridge_node.cpp \
  src/pico_bridge/launch/start_tianji_mujoco_teleop.launch.py \
  src/pico_bridge/test/test_tianji_mujoco_teleop_bridge_runtime.py \
  src/pico_bridge/test/test_launch_integration.py README.md README.zh-CN.md
git commit -m "feat: stream PICO targets to Tianji MuJoCo"
```

### Task 4: Decode and gate PICO teleoperation packets in Tianji

**Repository:** `/home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1`

**Files:**
- Create: `include/tianji_qp_ik/pico_teleop_protocol.hpp`
- Create: `src/pico_teleop_protocol.cpp`
- Create: `tests/test_pico_teleop_protocol.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: the exact 160-byte packet from PICO Task 2.
- Produces: `decodePicoTeleopPacket` and `PicoTeleopStreamGate::evaluate` for the UDP receiver.

- [ ] **Step 1: Add failing decoder and stream-gate tests**

Decode the same golden packet hex from Task 2 and assert:

```cpp
ASSERT_EQ(result.error, PicoPacketError::kNone);
ASSERT_TRUE(result.frame.has_value());
EXPECT_EQ(result.frame->sequence, 7U);
EXPECT_EQ(result.frame->tracking_epoch, 9U);
EXPECT_EQ(result.frame->source_timestamp_ns, 11);
EXPECT_TRUE(result.frame->left.position.isApprox(
  Eigen::Vector3d(1.0, 2.0, 3.0)));
EXPECT_TRUE(result.frame->right.position.isApprox(
  Eigen::Vector3d(-1.0, -2.0, -3.0)));
EXPECT_TRUE(result.frame->left.rotation.isApprox(Eigen::Matrix3d::Identity()));
```

Flip one payload bit and expect `kCrcMismatch`; test wrong magic/version/size/flags, NaN position, zero quaternion, and non-unit quaternions outside tolerance `1e-3`.

For `PicoTeleopStreamGate(0.15, 0.60)`, assert the first frame of an epoch is accepted, duplicate/out-of-order sequence is rejected, a `0.151 m` side jump is rejected, a `0.601 rad` side jump is rejected, the first frame of a strictly newer non-zero epoch is accepted with `epoch_changed=true`, and an old epoch arriving after that transition is rejected as rollback.

- [ ] **Step 2: Build and verify the focused test fails**

Run:

```bash
pixi run build
```

Expected: compilation fails because `pico_teleop_protocol.hpp` is missing.

- [ ] **Step 3: Implement the decoder and deterministic stream gate**

Expose exactly:

```cpp
inline constexpr std::size_t kPicoTeleopPacketSize = 160;

enum class PicoPacketError
{
  kNone,
  kWrongSize,
  kWrongMagic,
  kWrongVersion,
  kWrongDeclaredSize,
  kInvalidFlags,
  kCrcMismatch,
  kInvalidMetadata,
  kNonFinitePose,
  kInvalidQuaternion,
};

struct PicoTeleopFrame
{
  std::uint64_t sequence{0};
  std::uint64_t tracking_epoch{0};
  std::int64_t source_timestamp_ns{0};
  std::int64_t bridge_send_monotonic_ns{0};
  std::int64_t receive_monotonic_ns{0};
  Pose left;
  Pose right;
};

struct PicoPacketDecodeResult
{
  PicoPacketError error{PicoPacketError::kWrongSize};
  std::optional<PicoTeleopFrame> frame;
};

PicoPacketDecodeResult decodePicoTeleopPacket(
  const std::uint8_t * bytes, std::size_t size) noexcept;

enum class PicoStreamRejectReason
{
  kNone,
  kZeroEpoch,
  kEpochRollback,
  kOutOfOrder,
  kPositionJump,
  kOrientationJump,
};

struct PicoStreamDecision
{
  bool accepted{false};
  bool epoch_changed{false};
  PicoStreamRejectReason reason{PicoStreamRejectReason::kNone};
};

class PicoTeleopStreamGate
{
public:
  PicoTeleopStreamGate(double max_position_jump_m,
                       double max_orientation_jump_rad);
  PicoStreamDecision evaluate(const PicoTeleopFrame & frame);
  void reset() noexcept;
};
```

Decode every scalar explicitly little-endian at the Task 2 offsets. Reject zero sequence/epoch and non-positive source/send timestamps as `kInvalidMetadata`. Construct Eigen quaternions as `(w,x,y,z)` from wire `xyzw`, normalize only after verifying `abs(norm-1)<=1e-3`, and call `isProperRotation`. Compare jumps independently for left and right using Euclidean position distance and `rotationDistance`. Epochs are monotonic in PICO's persistent tracking-epoch store, so accept an epoch transition only when `frame.tracking_epoch > current_epoch`; delayed packets from an older epoch cannot reset the stream.

- [ ] **Step 4: Run the decoder tests and full Tianji tests**

Run:

```bash
pixi run build
./build/test_pico_teleop_protocol
pixi run test
```

Expected: protocol tests and the existing 34-test suite pass.

- [ ] **Step 5: Commit the Tianji protocol decoder**

```bash
git add CMakeLists.txt include/tianji_qp_ik/pico_teleop_protocol.hpp \
  src/pico_teleop_protocol.cpp tests/test_pico_teleop_protocol.cpp
git commit -m "feat: decode and gate PICO teleop packets"
```

### Task 5: Add the independent latest-only Tianji UDP receiver

**Repository:** `/home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1`

**Files:**
- Create: `include/tianji_qp_ik/pico_udp_receiver.hpp`
- Create: `src/pico_udp_receiver.cpp`
- Create: `tests/test_pico_udp_receiver.cpp`
- Modify: `CMakeLists.txt`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `tests/test_snapshot_exchange.cpp`

**Interfaces:**
- Consumes: Task 4 decoder/gate and a dedicated `LatestSpscExchange<PicoTeleopFrame>`.
- Produces: `PicoUdpReceiver`, a true latest-only non-blocking SPSC handoff, and atomic `PicoReceiverStats` snapshots for the Viewer control thread.

- [ ] **Step 1: Add failing localhost receiver and burst tests**

Test lifecycle and ephemeral binding:

```cpp
LatestSpscExchange<PicoTeleopFrame> exchange;
PicoUdpReceiver receiver(
  PicoUdpReceiverOptions{"127.0.0.1", 0U, 0.15, 0.60}, exchange);
receiver.start();
ASSERT_NE(receiver.boundPort(), 0U);
sendGoldenPacket(receiver.boundPort(), 1U, 9U, 1000000000LL);
ASSERT_TRUE(waitForLatest(exchange, frame, std::chrono::milliseconds(100)));
EXPECT_EQ(frame.sequence, 1U);
receiver.stop();
```

Also send malformed CRC, duplicate sequence, over-threshold jump, newer epoch, and epoch-rollback packets and assert exact counters. Send 32 valid frames before one consumer read and assert `tryReadLatest` returns sequence 32 and `superseded == 31`; the exchange must never share the UI command queue.

- [ ] **Step 2: Build and verify failure**

Run:

```bash
pixi run build
```

Expected: compilation fails because `PicoUdpReceiver` is missing.

- [ ] **Step 3: Implement receiver lifecycle and atomic statistics**

Expose:

```cpp
struct PicoUdpReceiverOptions
{
  std::string bind_address{"127.0.0.1"};
  std::uint16_t port{15000U};
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
};

struct PicoReceiverStats
{
  std::uint64_t datagrams{0};
  std::uint64_t accepted{0};
  std::uint64_t malformed{0};
  std::uint64_t crc_failures{0};
  std::uint64_t reordered{0};
  std::uint64_t jump_rejections{0};
  std::uint64_t superseded{0};
  std::uint64_t epoch_resets{0};
  std::uint64_t tracking_epoch{0};
  std::uint64_t sequence{0};
  std::int64_t latest_receive_monotonic_ns{0};
  double input_frequency_hz{0.0};
};

class PicoUdpReceiver
{
public:
  PicoUdpReceiver(PicoUdpReceiverOptions options,
                  LatestSpscExchange<PicoTeleopFrame> & exchange);
  ~PicoUdpReceiver();
  void start();
  void stop() noexcept;
  std::uint16_t boundPort() const noexcept;
  PicoReceiverStats stats() const noexcept;
};
```

Add this separate exchange next to the existing queue-backed snapshot exchange; do not alter UI/telemetry queue behavior:

```cpp
enum class LatestPublishResult { kPublished, kSuperseded };

template <typename T>
class LatestSpscExchange
{
public:
  LatestPublishResult publish(const T & value) noexcept;
  bool tryReadLatest(T & value) noexcept;
};
```

Implement it as three owned payload slots plus one atomic middle-slot state with a dirty bit. The producer writes only its private slot and atomically exchanges it with the middle slot; the consumer exchanges its private slot with a dirty middle slot before copying. This prevents concurrent access to the same non-atomic Eigen payload, never blocks the 200 Hz consumer, always preserves the newest publication, and returns `kSuperseded` when an unread middle value was replaced.

Create one UDP socket, set `SO_REUSEADDR`, bind the exact IPv4 endpoint, and run one receiver thread using `poll` with a 20 ms timeout. Stamp accepted frames with `CLOCK_MONOTONIC`, pass them through `PicoTeleopStreamGate`, and publish to the dedicated exchange. Store every counter as an atomic scalar; count overwritten unread frames as `superseded` and accepted newer epochs as `epoch_resets`. Compute input frequency from the median of the last 32 positive source-timestamp gaps inside the receiver thread, then publish the resulting double atomically. `stop()` sets the flag, joins the exact thread, closes the exact socket, and is idempotent.

- [ ] **Step 4: Run receiver and regression tests**

Run:

```bash
pixi run build
./build/test_pico_udp_receiver
./build/test_snapshot_exchange
pixi run test
```

Expected: localhost, burst, malformed, epoch, shutdown, and existing SPSC tests pass.

- [ ] **Step 5: Commit the UDP receiver**

```bash
git add CMakeLists.txt include/tianji_qp_ik/pico_udp_receiver.hpp \
  include/tianji_qp_ik/telemetry.hpp \
  src/pico_udp_receiver.cpp tests/test_pico_udp_receiver.cpp \
  tests/test_snapshot_exchange.cpp
git commit -m "feat: receive latest PICO teleop target"
```

### Task 6: Make bilateral target updates atomic and add the PICO profile

**Repository:** `/home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1`

**Files:**
- Modify: `include/tianji_qp_ik/target_manager.hpp`
- Modify: `src/target_manager.cpp`
- Modify: `tests/test_target_manager.cpp`
- Modify: `include/tianji_qp_ik/config.hpp`
- Modify: `src/config.cpp`
- Modify: `tests/test_config.cpp`
- Create: `config/qp_ik_pico_teleop.yaml`
- Modify: `tests/assert_viewer_config.cmake`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: bilateral poses and one source/receive timestamp from a decoded frame.
- Produces: `TargetManager::setManualTargets`, `PicoTeleopConfig`, and a 50 ms high-response profile used by the Viewer.

- [ ] **Step 1: Add failing atomic-update and config tests**

Add tests proving both sides update from the same source frame and neither side changes if the right pose is invalid or the timestamp is duplicate:

```cpp
ASSERT_TRUE(manager.setManualTargets(
  requested.left, requested.right, 10.0, 0.0));
const auto accepted = manager.sample(0.0);
EXPECT_TRUE(accepted.left.position.isApprox(requested.left.position));
EXPECT_TRUE(accepted.right.position.isApprox(requested.right.position));

Pose invalid_right = requested.right;
invalid_right.rotation(0, 0) = 2.0;
EXPECT_FALSE(manager.setManualTargets(
  newer_left, invalid_right, 10.01, 0.01));
const auto unchanged = manager.sample(0.01);
EXPECT_TRUE(unchanged.left.position.isApprox(accepted.left.position));
EXPECT_TRUE(unchanged.right.position.isApprox(accepted.right.position));
```

Load `config/qp_ik_pico_teleop.yaml` and assert:

```cpp
EXPECT_DOUBLE_EQ(config.controller.rate_hz, 200.0);
EXPECT_EQ(config.control_level, ControlLevel::kAcceleration);
EXPECT_TRUE(config.cartesian_otg.enabled);
EXPECT_DOUBLE_EQ(config.cartesian_servo.target_timeout_seconds, 0.050);
EXPECT_DOUBLE_EQ(config.pico_teleop.max_position_jump_m, 0.15);
EXPECT_DOUBLE_EQ(config.pico_teleop.max_orientation_jump_rad, 0.60);
```

Reject non-positive jump limits.

Add a timestamp-aware bilateral twist test with two targets 14 ms apart and assert the raw delta uses `0.014`, not the 200 Hz control period. Add a timeout boundary test proving the last pose is frozen and both twists are zero/stale when target age reaches exactly `0.050 s`.

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
pixi run build
```

Expected: compilation fails because `setManualTargets` and `PicoTeleopConfig` do not exist.

- [ ] **Step 3: Implement strict two-phase bilateral commit and config parsing**

Add:

```cpp
bool TargetManager::setManualTargets(
  const Pose & left_requested,
  const Pose & right_requested,
  double source_timestamp_seconds,
  double receive_time_seconds);
```

Copy `manual_`, `left_manual_state_`, and `right_manual_state_` into local candidates. Validate finite timestamps, both rotations/positions, and strictly newer source timestamps for both sides. Estimate each filtered twist from consecutive absolute mapped targets using the common source `dt`, then store both requested poses unchanged in the candidates. Assign all candidates to members only after both sides succeed. Do not call `limitManualIncrement` during PICO ingestion: the existing sample-level safety increment and Cartesian OTG already smooth the transition, while the stored target must remain the absolute 1:1 PICO target. Treat age `>= target_timeout_seconds` as stale and zero twist at that boundary. Keep the existing single-side overload unchanged for marker compatibility.

Add:

```cpp
struct PicoTeleopConfig
{
  double max_position_jump_m{0.15};
  double max_orientation_jump_rad{0.60};
};
```

Parse an optional `pico_teleop` YAML map so old profiles retain defaults. Create `qp_ik_pico_teleop.yaml` by copying the current acceleration profile exactly, changing only `cartesian_servo.target_timeout_seconds` to `0.050`, and appending the two jump limits.

- [ ] **Step 4: Run target/config and full tests**

Run:

```bash
pixi run build
./build/test_target_manager
./build/test_config
ctest --test-dir build -R 'viewer_(default|explicit)_profile' --output-on-failure
pixi run test
```

Expected: atomicity, 50 ms profile, old-profile compatibility, and all existing tests pass.

- [ ] **Step 5: Commit atomic targets and profile**

```bash
git add include/tianji_qp_ik/target_manager.hpp src/target_manager.cpp \
  tests/test_target_manager.cpp include/tianji_qp_ik/config.hpp src/config.cpp \
  tests/test_config.cpp config/qp_ik_pico_teleop.yaml \
  tests/assert_viewer_config.cmake CMakeLists.txt
git commit -m "feat: update PICO dual-arm targets atomically"
```

### Task 7: Integrate PICO session state, diagnostics, and controls into Viewer

**Repository:** `/home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1`

**Files:**
- Create: `include/tianji_qp_ik/pico_teleop_session.hpp`
- Create: `src/pico_teleop_session.cpp`
- Create: `tests/test_pico_teleop_session.cpp`
- Create: `tests/test_pico_viewer_integration.py`
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_snapshot_exchange.cpp`
- Modify: `CMakeLists.txt`
- Modify: `pixi.toml`

**Interfaces:**
- Consumes: Task 5 receiver/exchange/stats and Task 6 atomic TargetManager update.
- Produces: deterministic teleoperation actions, `P` enable control, Viewer overlay/CSV/headless metrics, and clean receiver startup/shutdown.

- [ ] **Step 1: Add failing session and Viewer contract tests**

Define and test these transitions:

```cpp
PicoTeleopSession session(0.050);
session.setEnabled(true);
auto first = session.classify(frameAt(9U, 1U, 1000000000LL), 1000000000LL);
EXPECT_EQ(first.action, PicoTeleopAction::kResetEpochAndApply);

auto next = session.classify(frameAt(9U, 2U, 1014000000LL), 1015000000LL);
EXPECT_EQ(next.action, PicoTeleopAction::kApply);

session.setEnabled(false);
EXPECT_EQ(session.classify(frameAt(9U, 3U, 1028000000LL), 1029000000LL).action,
          PicoTeleopAction::kIgnoreDisabled);

session.setEnabled(true);
EXPECT_EQ(session.classify(frameAt(9U, 3U, 1028000000LL), 1080000000LL).action,
          PicoTeleopAction::kIgnoreStale);

const auto freshness = session.freshness(1080000000LL);
EXPECT_FALSE(freshness.live);
EXPECT_TRUE(freshness.stale);
EXPECT_GT(freshness.frame_age_seconds, 0.050);
```

Add source assertions that Viewer help contains `P PICO teleop`, CLI help contains `--pico-teleop --pico-bind --pico-port --control-level`, and Telemetry/ViewerSnapshot expose the exact PICO fields listed below.

- [ ] **Step 2: Run focused tests and verify failure**

Run:

```bash
pixi run build
```

Expected: compilation fails because `PicoTeleopSession` and telemetry fields are missing.

- [ ] **Step 3: Implement the session state and snapshot fields**

Expose:

```cpp
enum class PicoTeleopAction
{
  kIgnoreDisabled,
  kIgnoreStale,
  kIgnoreAlreadyApplied,
  kApply,
  kResetEpochAndApply,
};

struct PicoTeleopClassification
{
  PicoTeleopAction action{PicoTeleopAction::kIgnoreDisabled};
  double frame_age_seconds{0.0};
};

struct PicoTeleopFreshness
{
  bool has_applied_frame{false};
  bool live{false};
  bool stale{false};
  double frame_age_seconds{0.0};
};

class PicoTeleopSession
{
public:
  explicit PicoTeleopSession(double timeout_seconds);
  void setEnabled(bool enabled) noexcept;
  bool enabled() const noexcept;
  PicoTeleopClassification classify(
    const PicoTeleopFrame & frame, std::int64_t now_monotonic_ns);
  PicoTeleopFreshness freshness(std::int64_t now_monotonic_ns) const noexcept;
};
```

`classify` records the receive time only for a fresh frame selected for application. `freshness` is called every 200 Hz cycle even when no datagram arrives; once the age reaches 50 ms it reports stale. Disabling reports neither live nor stale, and re-enabling requires a fresh frame before returning live.

Add these fields to both `TelemetrySample` and `ViewerSnapshot`:

```cpp
bool pico_configured{false};
bool pico_enabled{false};
bool pico_live{false};
bool pico_stale{false};
std::uint64_t pico_tracking_epoch{0};
std::uint64_t pico_sequence{0};
std::uint64_t pico_datagrams{0};
std::uint64_t pico_accepted{0};
std::uint64_t pico_malformed{0};
std::uint64_t pico_crc_failures{0};
std::uint64_t pico_reordered{0};
std::uint64_t pico_jump_rejections{0};
std::uint64_t pico_superseded{0};
std::uint64_t pico_epoch_resets{0};
double pico_input_frequency_hz{0.0};
double pico_frame_age_ms{0.0};
double pico_receive_to_control_us{0.0};
double pico_bridge_to_control_us{0.0};
```

- [ ] **Step 4: Wire receiver ownership and atomic control-loop application**

Extend `Options` with:

```cpp
bool pico_teleop{false};
std::string pico_bind{"127.0.0.1"};
std::uint16_t pico_port{15000U};
std::optional<ControlLevel> control_level_override;
```

Parse `--control-level velocity|acceleration` as an optional repeatable-test override applied after loading YAML; reject any other value. Validate `--pico-bind` as IPv4 and `--pico-port` in `[1,65535]`. Interactive `V`/`A` switching remains unchanged. In `run`, create `LatestSpscExchange<PicoTeleopFrame> pico_frames` and a `PicoUdpReceiver` only when `--pico-teleop` is present. Start it before the control thread and stop/join it on every normal and exceptional exit.

The control loop always drains the newest PICO frame so disabled input cannot backlog. Apply this exact action sequence:

```cpp
if (pico_frames != nullptr && pico_frames->tryReadLatest(pico_frame)) {
  const auto classification = pico_session.classify(pico_frame, monotonic_now_ns());
  if (classification.action == PicoTeleopAction::kResetEpochAndApply) {
    const DualArmTargets current = currentTargets(robot);
    targets = TargetManager(config, current);
    targets.setMode(TargetMode::kManual, control_time);
    left_otg.reset(current.left);
    right_otg.reset(current.right);
  }
  if (classification.action == PicoTeleopAction::kApply ||
      classification.action == PicoTeleopAction::kResetEpochAndApply) {
    targets.setMode(TargetMode::kManual, control_time);
    const double source_seconds =
      static_cast<double>(pico_frame.source_timestamp_ns) * 1.0e-9;
    (void)targets.setManualTargets(
      pico_frame.left, pico_frame.right, source_seconds, control_time);
  }
}
```

Add `ViewerCommandType::kSetPicoTeleopEnabled`; `P` toggles it. Disabling sets TargetMode Hold. While enabled, marker-generated `kSetManualTarget` commands are ignored so UI and UDP cannot fight. `V`/`A`, pause, reset, camera, and rendering controls remain unchanged. CLI-enabled PICO starts enabled; no frame is applied until a fresh accepted datagram arrives.

Write all PICO fields into snapshots, overlay, terminal headless summary, and telemetry CSV. A frame is live only when enabled and age is less than 50 ms; age `>= 50 ms` is stale. On stale input, TargetManager emits zero twist/stale flags and OTG decelerates without reusing prediction.

When both `--headless` and `--pico-teleop` are set, use a dedicated monitor-only headless loop. It must not enqueue or require completion of the existing 13 scripted `1/2/3/4`, `V/A`, pause, and reset stages. It reads snapshots until duration expires, prints PICO/control metrics, and succeeds when the control loop stayed healthy even if no PICO packet arrived or the final frame is intentionally stale.

- [ ] **Step 5: Add the synthetic UDP-to-headless-Viewer integration test**

Create a Python CTest that reserves a localhost UDP port, starts the Viewer in PICO headless monitor mode, and sends correctly encoded 160-byte bilateral frames at 72 Hz. Use smooth bounded sinusoidal translations and orientations so every inter-frame change remains below the jump limits. Parameterize the test over:

```text
--control-level velocity
--control-level acceleration
```

For each mode, send for at least one second, stop sending at least 75 ms before Viewer exit, and assert: process exit code zero; accepted sequence advances; both arms carry the same source timestamp; `pico_live` becomes true; `pico_stale` becomes true 50--60 ms after the last receive; no malformed/CRC/reordered/jump errors occur; no control failure occurs; and telemetry contains finite bridge-to-control, receive-to-control, control-cycle, and bilateral pose-error columns.

- [ ] **Step 6: Run session, integration, Viewer, and full tests**

Run:

```bash
pixi run build
./build/test_pico_teleop_session
./build/test_snapshot_exchange
ctest --test-dir build -R pico_viewer_integration --output-on-failure
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_test.xml \
  --pico-teleop --headless --duration 2
pixi run test
```

Expected: session tests and both synthetic velocity/acceleration Viewer cases pass; the no-input monitor run reports `pico_configured=1`, `pico_live=0`, exits cleanly without control failures; all existing Viewer and 34 baseline tests remain passing.

- [ ] **Step 7: Commit Viewer integration**

```bash
git add include/tianji_qp_ik/pico_teleop_session.hpp src/pico_teleop_session.cpp \
  tests/test_pico_teleop_session.cpp include/tianji_qp_ik/telemetry.hpp \
  apps/run_qp_ik_viewer.cpp tests/test_snapshot_exchange.cpp \
  tests/test_pico_viewer_integration.py \
  CMakeLists.txt pixi.toml
git commit -m "feat: drive Tianji Viewer from PICO teleop"
```

### Task 8: Verify the cross-repository pipeline and document operation

**Repositories:** both isolated worktrees.

**Files:**
- Modify: Tianji `README.md`
- Create: Tianji `docs/verification/pico_mujoco_teleop_results.md`
- Modify: PICO `README.md`
- Modify: PICO `README.zh-CN.md`

**Interfaces:**
- Consumes: completed PICO publisher and Tianji receiver/Viewer.
- Produces: reproducible startup/test commands and measured acceptance evidence.

- [ ] **Step 1: Add exact user-facing startup and diagnostics instructions**

Document these terminal commands.

PICO terminal:

```bash
cd /home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1
pixi shell
export ROS_DOMAIN_ID=120 ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1
./scripts/start_pico_m0.sh
ros2 launch pico_bridge start_tianji_mujoco_teleop.launch.py \
  destination_address:=127.0.0.1 destination_port:=15000
```

Tianji terminal:

```bash
cd /home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1
pixi run configure
pixi run build
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_test.xml \
  --pico-teleop --pico-bind 127.0.0.1 --pico-port 15000
```

Document `P` enable, `V` velocity QP, `A` acceleration QP, and these checks:

```bash
ros2 topic hz /pico/smpl_palm_corrected_ik
ros2 topic echo /pico/smpl_palm_corrected/status --once
ros2 topic echo /pico/tianji_mujoco_teleop/status --once
```

- [ ] **Step 2: Run complete PICO verification**

Run:

```bash
cd /home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1
pixi run build-core
pixi run bash -lc 'source install/setup.bash && colcon test \
  --base-paths src --packages-select pico_bridge --event-handlers console_direct+'
pixi run bash -lc 'colcon test-result --test-result-base build/pico_bridge --verbose'
```

Expected: all pico_bridge tests pass with zero failures.

- [ ] **Step 3: Run complete Tianji verification**

Run:

```bash
cd /home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1
pixi run configure
pixi run build
pixi run test
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_test.xml \
  --pico-teleop --headless --duration 10 \
  --telemetry /tmp/tianji_pico_teleop.csv
```

Expected: all Tianji tests pass; no-input headless mode exits cleanly; telemetry schema includes all PICO columns.

- [ ] **Step 4: Run live or synthetic end-to-end measurement**

Start PICO M0/bridge and the Tianji headless Viewer concurrently for at least 60 seconds. Record the PICO diagnostics JSON and Tianji telemetry. Compute and write these measured values to `docs/verification/pico_mujoco_teleop_results.md`:

```text
corrected_ik median frequency Hz
bridge accepted/output frequency Hz
Viewer PICO input frequency Hz
same-host send-to-control latency p50/p95/p99 ms
control cycle p50/p95/p99 us
deadline misses
malformed/CRC/reordered/jump/superseded/epoch-reset counts
50 ms stale-transition result
left/right source timestamp mismatch count
velocity-QP and acceleration-QP control failure counts
```

Run the automated synthetic headless case once per control level using `--control-level velocity` and `--control-level acceleration`; if live PICO hardware is available, repeat the same two runs with the real bridge and record those separately. The no-input and post-input timeout portions use PICO monitor-only headless mode, not the Viewer's legacy scripted-stage harness.

Acceptance requires PICO rate near its observed nominal rate, bridge output equal to accepted source frames, send-to-control p99 below `5 ms`, control p99 below `5000 us`, zero normal-run deadline misses, and zero left/right timestamp mismatches. The configured freshness boundary is exactly `50 ms`; the stream must become stale on the first 200 Hz control sample at or after that boundary (nominal observed transition age `50--55 ms`, with scheduler jitter reported separately).

- [ ] **Step 5: Commit documentation and verification evidence in each repository**

Tianji:

```bash
git add README.md docs/verification/pico_mujoco_teleop_results.md
git commit -m "docs: verify PICO MuJoCo teleoperation"
```

PICO:

```bash
git add README.md README.zh-CN.md
git commit -m "docs: document Tianji MuJoCo teleop bridge"
```

- [ ] **Step 6: Final clean-tree and branch audit**

Run in both worktrees:

```bash
git status --short
git log --oneline --decorate -8
```

Expected: both worktrees are clean; only `feature/pico-mujoco-teleop-v1` and `feature/tianji-mujoco-teleop-v1` contain the new commits; the existing base/velocity/acceleration/PICO branches retain their original heads.
