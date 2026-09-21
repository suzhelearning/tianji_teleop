# PICO A 键触发 IMU900 清零与自动融合标定设计

## 目标

右手柄 A 键完成 PICO 世界坐标重置后，系统自动清零左右脚 IMU900 的 Z 轴；只有两路设备均返回成功 ACK 且产生新的有效姿态后，才开始 PICO + IMU900 软件融合标定。

## 触发事件

`pico_bridge_node` 将 TCP `TYPE_WORLD_RESET (0x06)` 发布为 `/pico/world_reset`。该事件与 `/pico/record_flag` 分离，避免把录制状态当作标定完成信号。消息使用 `std_msgs/msg/Float32` 保存 PICO 提供的 reset yaw。

## 状态流程

1. 收到新的 `/pico/world_reset` 后立即清除融合基线、停止当前标定并暂停 `/pico/smpl_fused`。
2. 异步调用 `/im900/left_foot/zero_z_axis` 和 `/im900/right_foot/zero_z_axis`。
3. 只有两个 `std_srvs/srv/Trigger` 响应均为 `success=true` 时才进入等待新 IMU 数据状态。
4. 两路 ACK 完成后丢弃 reset 前缓存的 IMU 姿态，要求左右脚分别收到新的 ready、有效且新鲜的四元数。
5. 条件满足后自动进入现有 90 帧稳定采样流程；骨盆、左右 PICO 脚或左右 IMU 任一移动超阈值时重新采样。
6. 标定完成后恢复 `/pico/smpl_fused` 和 `/pico/ankle_relative`。

重复收到 A 键事件时，用递增 generation 取消旧事务，只允许最新事件的服务响应改变状态。

## 失败处理

- 任一 zero-Z 服务不存在、调用异常、超时或返回失败：保持无基线状态，不进行软件标定，并输出包含左右侧和失败原因的错误日志。
- IMU reset 成功但新姿态未到达：继续等待，不复用 reset 前样本。
- IMU 在自动标定期间失效或断线：停止采样；ready 恢复后仍需最新 A 键事务或手动标定。
- 手动 `/pico_foot_imu_fusion/reset` 取消自动事务；手动 `/calibrate` 保留现有行为，不向 IMU900 发送命令。

## 接口与配置

- 新话题：`/pico/world_reset`，类型 `std_msgs/msg/Float32`。
- 使用服务：`/im900/left_foot/zero_z_axis`、`/im900/right_foot/zero_z_axis`。
- 新参数：`auto_calibrate_on_world_reset`，默认 `true`。
- 新参数：左右 zero-Z 服务名，保留覆盖能力。
- 安装四元数与该事务独立；每次改变安装四元数后仍需重新触发 A 键或手动标定。

## 测试与验收

- frame/bridge 测试验证 `0x06` 不再被静默丢弃并发布 yaw。
- 状态机测试覆盖双 ACK 成功、单侧失败、旧 generation 响应、旧 IMU 样本丢弃和手动 reset 取消。
- 完整构建及 ROS 测试必须通过。
- 实机验收顺序为：按 A、观察两侧 zero-Z ACK、观察自动标定完成日志、验证 yaw/pitch/roll 方向及断线失败行为。
