# PICO 掌心运行时 Publisher 一键启动设计

## 目标

将交互式 TCP 标定与日常掌心发布解耦。标定器只在重新采样时运行；日常掌心约束骨架测试由一个 launch 自动启动左右掌心 publisher 和骨架 filter。

## 方案比较

1. **继续启动两个 calibrator**：改动最小，但需要两个终端，存在误按空格覆盖 artifact 的风险。
2. **在 filter 内直接读取左右 TCP artifact**：进程少，但混合原始 controller 与 canonical palm 语义，使 filter 重复承担 TCP bridge 职责。
3. **独立无交互 publisher（采用）**：左右各自加载 artifact，显式发布 `/pico/palm_left/right`，filter 保持单一职责，TCP exactly-once 数据流清晰。

## 数据流

```text
/pico/pose/left_hand  + left T_controller_palm  -> /pico/palm_left
/pico/pose/right_hand + right T_controller_palm -> /pico/palm_right
                                              -> pico_palm_skeleton_filter
                                              -> /pico/smpl_palm_corrected
```

## 组件

### `pico_palm_tcp_runtime.py`

ROS 无关的运行时核心：

- 校验 `valid=true`、side、`pose_semantics=controller_pose` 和 `transform_convention=T_controller_palm`；
- 读取平移与 xyzw 四元数；
- 计算 `T_G_H = T_G_C · T_C_H`；
- 拒绝非有限位置、零四元数和方向错误的 artifact。

### `pico_palm_tcp_publisher.py`

- CLI：`--side left|right`、`--artifact PATH`；
- 输入：`/pico/pose/<side>_hand`，`geometry_msgs/PoseStamped`；
- 输出：`/pico/palm_<side>`，保留 controller source timestamp，`frame_id=pico`；
- 无键盘线程、无采样、绝不写 artifact；
- artifact 缺失或非法时启动失败，不发布伪造掌心。

### `start_pico_palm_skeleton_filter.launch.py`

新增：

- `start_palm_publishers:=true`；
- `left_tcp_artifact`；
- `right_tcp_artifact`。

默认启动左右 publisher 和原有 filter。需要手动运行 calibrator 时显式设置 `start_palm_publishers:=false`，避免同一掌心 topic 出现两个发布者。

## 测试与验收

- identity、平移和旋转组合测试；
- artifact side/方向/四元数校验测试；
- 输出保持 source timestamp 和 `pico` frame；
- launch 默认包含左右 publisher 和 filter；
- `start_palm_publishers=false` 时只启动 filter；
- `pico_bridge` 全包构建与测试通过；
- 运行时 `/pico/palm_left/right` 各恰好一个 publisher，filter 状态进入 `baseline_ready=true`、`corrected=true`。
