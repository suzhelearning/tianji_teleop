# 扩展 PICO 流协议：全身追踪 + A 键录制开关 设计文档

**日期：** 2026-07-09
**状态：** 已确认（用户批准）

## 背景

新版 `PicoStreamingServer`（参考 `/home/eai/Downloads/pico_stream_record_receiver.py`）在原有 8 种帧类型之外新增了 10 种：

| 类型 | 内容 | payload |
|---|---|---|
| `0x07` / `0x08` | 左/右手柄按键状态 | 未文档化（参考端仅存 hex） |
| `0x09` | A 键录制开关 | 1 字节，非 0 = 开始 |
| `0x20` / `0x21` | 脚踝 0/1 | 7×float32（同现有 pose） |
| `0x22` / `0x23` | 膝盖 0/1 | 同上 |
| `0x24` / `0x25` | 手肘 0/1 | 同上 |
| `0x26` | 骨盆 | 同上 |

当前仓库对这些帧走 unknown-type 路径直接丢弃。本设计把身体点和录制开关接入 ROS 采集链路。

## 决策（已与用户确认）

1. **0x09 接入自动录制**：pico_bridge 发布 `/pico/record_flag`，data_collector 订阅后自动 start/stop，键盘 s/d 保留。
2. **0x07/0x08 静默跳过**：登记常量、不再刷 unknown 警告，不发布（payload 格式无文档）。`0x06` 世界重置同样静默跳过。
3. **7 个身体点默认写入 collect_config.yaml 并启用**：不用 Motion Tracker 时手动注释对应条目。
4. **不重复造轮子**：身体点复用 `parse_pose_payload` / `publish_pose` / `PoseStampedProcessor`，零新增解析/存储逻辑。

## 设计

### pico_frame.hpp
新增常量：`TYPE_WORLD_RESET=0x06`、`TYPE_CTRL_LEFT=0x07`、`TYPE_CTRL_RIGHT=0x08`、`TYPE_RECORD_FLAG=0x09`、`TYPE_ANKLE0=0x20`、`TYPE_ANKLE1=0x21`、`TYPE_KNEE0=0x22`、`TYPE_KNEE1=0x23`、`TYPE_ELBOW0=0x24`、`TYPE_ELBOW1=0x25`、`TYPE_PELVIS=0x26`。无新解析函数。

### pico_bridge_node.cpp
- 身体点用 `type → PoseStamped publisher` 查表（`std::unordered_map` 或等价结构），与现有头/手柄 pose 走同一个 `publish_pose`；话题命名 `/pico/pose/{ankle0, ankle1, knee0, knee1, elbow0, elbow1, pelvis}`（0/1 下标与参考接收端一致；0x20 左右归属官方未文档化，暂不命名 left/right）。
- `/pico/record_flag`：`std_msgs/Bool`，RELIABLE + VOLATILE + depth 10。事件语义，不用 transient_local，避免采集端后启动时被过期开关误触发。
- `0x06/0x07/0x08`：静默跳过；其余未知类型保留 WARN。

### data_collector
- yaml 顶层可选键 `record_flag_topic`（默认缺省 = 不订阅）。配置后订阅 `std_msgs/Bool`：`True → start_collecting()`，`False → stop_collecting()`。
- `collect_config.yaml`：新增 7 个 pose dataset（`PoseStampedProcessor`）+ `topics` 条目 + `record_flag_topic: /pico/record_flag`。
- 已知取舍：A 键触发的 `stop_collecting()` 在 rclpy 回调线程执行落盘（耗时数秒、期间回调暂停），此时采集已停，行为与键盘路径一致，不做额外改造。

### mock_pico_server.py
- 每帧循环追加 7 个身体位姿帧（不同 offset）。
- 新增 `--record-cycle N`（默认 0 = 关闭）：每 N 秒翻转一次 0x09。

### 测试与文档
- `test_pico_frame.cpp`：新增类型常量断言（防止常量与 wire 协议漂移）。
- 更新 `docs/PICO_Streaming_Guide.md` 帧类型表、`CLAUDE.md` 话题表与 data_collector 说明。

## 验证

1. `pixi run build` + `colcon test --packages-select pico_bridge`。
2. 集成冒烟：`mock_pico_server.py --record-cycle 5` + pico_bridge，确认 `/pico/pose/pelvis` 等话题有数据、`/pico/record_flag` 周期翻转。
3. data_collector 用仅含 pico 话题的临时配置跑一轮 A 键自动录制，检查 HDF5 内出现 7 个身体点 group（`pos/quat_xyzw/ts_ms`）。
