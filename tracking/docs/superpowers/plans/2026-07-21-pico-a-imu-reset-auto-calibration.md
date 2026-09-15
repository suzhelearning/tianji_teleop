# PICO A 键触发 IMU900 清零与自动融合标定 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 PICO 右手柄 A 键在世界坐标重置后，自动清零双脚 IMU900 的 Z 轴并在双侧 ACK 成功后开始安全的软件融合标定。

**Architecture:** `pico_bridge_node` 将 TCP `0x06` 转为 `/pico/world_reset`。`pico_foot_imu_fusion` 订阅该事件并以 generation 保护异步的左右 zero-Z 服务响应；两侧成功后丢弃旧 IMU 样本，等待新的 ready/fresh 样本，再复用已有稳定采样逻辑完成标定。

**Tech Stack:** ROS 2 Humble、C++17、rclcpp、std_msgs、std_srvs、gtest、ament/colcon、Pixi。

## Global Constraints

- 触发源只能是 `TYPE_WORLD_RESET (0x06)`，不能使用 `/pico/record_flag`。
- 任一 zero-Z 调用失败、超时或服务不可用时必须保持无融合基线。
- 两侧 zero-Z 都成功后必须丢弃 reset 前 IMU 样本。
- 手动 `/calibrate` 不发送 IMU 命令；手动 `/reset` 取消自动事务。
- 保留 `/pico/smpl` 原始话题、`/pico/smpl_fused` 和 `/pico/ankle_relative` 的既有消息类型。

---

### Task 1: 发布 PICO 世界重置事件

**Files:**
- Modify: `src/pico_bridge/include/pico_bridge/pico_frame.hpp`
- Modify: `src/pico_bridge/src/pico_bridge_node.cpp`
- Modify: `src/pico_bridge/test/test_pico_frame.cpp`

**Interfaces:**
- Produces: `/pico/world_reset` with type `std_msgs/msg/Float32`; `data` is the little-endian yaw carried by `0x06`.

- [ ] **Step 1: Write the failing yaw parser test**

```cpp
TEST(PicoFrame, ParsesWorldResetYaw) {
  const float expected = 1.25F;
  uint8_t payload[sizeof(float)];
  std::memcpy(payload, &expected, sizeof(expected));
  float yaw{};
  ASSERT_TRUE(pico_bridge::parse_world_reset_payload(payload, sizeof(payload), yaw));
  EXPECT_FLOAT_EQ(yaw, expected);
  EXPECT_FALSE(pico_bridge::parse_world_reset_payload(payload, sizeof(payload) - 1, yaw));
}
```

- [ ] **Step 2: Run test red**

Run: `pixi run build`

Expected: `test_pico_frame.cpp` cannot resolve `parse_world_reset_payload`.

- [ ] **Step 3: Implement parser and publisher**

```cpp
inline bool parse_world_reset_payload(const uint8_t *payload, size_t len, float &yaw) {
  if (len < sizeof(float)) return false;
  std::memcpy(&yaw, payload, sizeof(float));
  return true;
}
```

Create `world_reset_pub_` as `create_publisher<std_msgs::msg::Float32>("/pico/world_reset", 10)` and replace the ignored `TYPE_WORLD_RESET` branch with parser validation plus `world_reset_pub_->publish`.

- [ ] **Step 4: Run focused test green**

Run: `pixi run test`

Expected: `test_pico_frame` passes.

- [ ] **Step 5: Commit**

```bash
git add src/pico_bridge/include/pico_bridge/pico_frame.hpp src/pico_bridge/src/pico_bridge_node.cpp src/pico_bridge/test/test_pico_frame.cpp
git commit -m "feat: publish PICO world reset events"
```

### Task 2: 加入可测试的自动标定事务状态

**Files:**
- Create: `src/pico_bridge/include/pico_bridge/auto_calibration_state.hpp`
- Create: `src/pico_bridge/test/test_auto_calibration_state.cpp`
- Modify: `src/pico_bridge/CMakeLists.txt`

**Interfaces:**
- Produces: `pico_bridge::AutoCalibrationState` with `begin()`, `accept_zero_ack(generation, left_side, success)`, `fail()`, `cancel()`, and `awaiting_fresh_imus()`.

- [ ] **Step 1: Write failing state transition tests**

```cpp
TEST(AutoCalibrationState, RequiresBothAcksBeforeFreshImus) {
  pico_bridge::AutoCalibrationState state;
  const auto generation = state.begin();
  EXPECT_FALSE(state.accept_zero_ack(generation, true, true));
  EXPECT_TRUE(state.accept_zero_ack(generation, false, true));
  EXPECT_TRUE(state.awaiting_fresh_imus());
}

TEST(AutoCalibrationState, IgnoresStaleGenerationAndCancelsOnFailure) {
  pico_bridge::AutoCalibrationState state;
  const auto old_generation = state.begin();
  const auto new_generation = state.begin();
  EXPECT_FALSE(state.accept_zero_ack(old_generation, true, true));
  state.fail(new_generation);
  EXPECT_FALSE(state.awaiting_fresh_imus());
}
```

- [ ] **Step 2: Run test red**

Run: `pixi run build`

Expected: compilation fails because `auto_calibration_state.hpp` is missing.

- [ ] **Step 3: Implement minimal state class**

```cpp
enum class AutoCalibrationPhase { kIdle, kAwaitingAcks, kAwaitingFreshImus, kFailed };
```

Store monotonic generation plus left/right ACK flags. `accept_zero_ack` returns true only when the current generation has both successful ACKs. `fail` and `cancel` move the current generation out of the active phase.

- [ ] **Step 4: Run focused test green**

Run: `pixi run test`

Expected: `test_auto_calibration_state` passes.

- [ ] **Step 5: Commit**

```bash
git add src/pico_bridge/include/pico_bridge/auto_calibration_state.hpp src/pico_bridge/test/test_auto_calibration_state.cpp src/pico_bridge/CMakeLists.txt
git commit -m "feat: model automatic foot calibration state"
```

### Task 3: 融合节点调用双 IMU zero-Z 并自动采样

**Files:**
- Modify: `src/pico_bridge/src/pico_foot_imu_fusion_node.cpp`
- Modify: `src/pico_bridge/launch/start_pico_foot_fusion.launch.py`
- Modify: `src/pico_bridge/test/test_imu900_foot_integration.py`
- Modify: `docs/PICO_FOOT_IMU_FUSION.md`
- Modify: `pico_full_test_commands.txt`

**Interfaces:**
- Consumes: `/pico/world_reset`, `/im900/left_foot/zero_z_axis`, `/im900/right_foot/zero_z_axis`, and ready topics.
- Produces: auto calibration only after two successful ACKs and post-ACK IMU samples.

- [ ] **Step 1: Write failing integration contract**

```python
def test_fusion_auto_calibrates_from_world_reset(self):
    source = (ROOT / "src/pico_foot_imu_fusion_node.cpp").read_text()
    self.assertIn('"/pico/world_reset"', source)
    self.assertIn('"/im900/left_foot/zero_z_axis"', source)
    self.assertIn('"/im900/right_foot/zero_z_axis"', source)
    self.assertIn("async_send_request", source)
    self.assertIn("auto_calibrate_on_world_reset", source)
```

- [ ] **Step 2: Run contract red**

Run: `pixi run python -m unittest src/pico_bridge/test/test_imu900_foot_integration.py -v`

Expected: FAIL because the event subscription and service clients are absent.

- [ ] **Step 3: Implement asynchronous orchestration**

Add `std_msgs/msg/float32.hpp`, `rclcpp/client.hpp`, and `auto_calibration_state.hpp`. Declare `auto_calibrate_on_world_reset` and left/right zero-Z service names. On event clear baseline/samples, reject both cached IMUs, begin a new generation, then send both Trigger requests. Each callback rejects stale generations; it fails on unavailable or unsuccessful response; after both success it rejects both IMUs again and waits for fresh data. In `pico_callback`, when the state awaits fresh IMUs and both are ready/fresh, start the existing sample collection. Manual reset calls `cancel()`.

- [ ] **Step 4: Run focused tests green**

Run: `pixi run build && pixi run python -m unittest src/pico_bridge/test/test_imu900_foot_integration.py -v`

Expected: build succeeds and all integration tests pass.

- [ ] **Step 5: Document and commit**

Document the A-key sequence, ACK logs, manual fallback, and the requirement to remain still until `PICO foot IMU calibration complete` appears.

```bash
git add src/pico_bridge/src/pico_foot_imu_fusion_node.cpp src/pico_bridge/launch/start_pico_foot_fusion.launch.py src/pico_bridge/test/test_imu900_foot_integration.py docs/PICO_FOOT_IMU_FUSION.md pico_full_test_commands.txt
git commit -m "feat: calibrate foot fusion from PICO world reset"
```

### Task 4: 全量验证与真机验收

**Files:**
- Modify: `pico_full_test_commands.txt`

- [ ] **Step 1: Run automated verification**

```bash
git diff --check
pixi run build
pixi run test
pixi run bash -lc 'colcon test-result --test-result-base build --verbose'
pixi run bash -lc 'source install/setup.bash && ros2 launch pico_bridge start_pico_foot_fusion.launch.py --show-args'
```

Expected: all packages build, all tests pass, and launch shows the new auto-calibration parameters.

- [ ] **Step 2: Run real-device acceptance**

Start PICO bridge and foot fusion, press right-controller A once while standing still, then observe both zero-Z service confirmations and `PICO foot IMU calibration complete`. Verify `/pico/smpl_fused` begins only after completion. Disconnect one IMU, press A again, and verify no completion log or fused output occurs.

- [ ] **Step 3: Commit verification documentation**

```bash
git add pico_full_test_commands.txt
git commit -m "docs: add A-triggered IMU calibration checks"
```
