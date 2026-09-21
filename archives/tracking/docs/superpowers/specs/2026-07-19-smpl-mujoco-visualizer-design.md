# SMPL MuJoCo 几何骨架可视化设计

## 目标

新增一个独立的 ROS 2 可视化节点，订阅 `/pico/smpl`（`geometry_msgs/msg/PoseArray`），使用 MuJoCo Viewer 实时显示 PICO 全身追踪骨架。

第一阶段只显示几何骨架，不加载 SMPL 人体网格或动力学模型，以便先验证关节数据、坐标系和实时性。

## 范围

包含：

- 订阅 `/pico/smpl`；
- 显示 24 个关节球；
- 显示 23 根父子关节之间的胶囊骨骼；
- 支持坐标缩放、水平旋转、刷新频率和超时配置；
- 在无 PICO 时支持测试消息驱动；
- 数据中断时保留最后一帧并提示超时。

不包含：

- SMPL 人体 mesh、蒙皮或动力学控制；
- 修改 PICO TCP 协议；
- 修改 `/pico/smpl` 的消息定义或发布逻辑；
- 同时支持多个可视化后端。

## 架构与数据流

```text
pico_wholebody_stream.apk
        ↓ TCP 9999
adb forward tcp:9999 tcp:9999
        ↓
pico_bridge_node
        ↓ /pico/smpl
smpl_mujoco_visualizer
        ↓
MuJoCo Viewer
```

可视化节点包含两个独立循环：

1. ROS 订阅回调保存最新的 `PoseArray`；
2. MuJoCo 渲染循环按目标频率读取最新数据并更新场景。

ROS 回调不直接操作 Viewer，从而避免渲染线程和 ROS executor 之间的线程耦合。

## 关节拓扑

关节顺序与 `pico_bridge` 的 24 个关节定义一致：

```text
Pelvis
├── LEFT_HIP → LEFT_KNEE → LEFT_ANKLE → LEFT_FOOT
├── RIGHT_HIP → RIGHT_KNEE → RIGHT_ANKLE → RIGHT_FOOT
└── SPINE1 → SPINE2 → SPINE3
              ├── NECK → HEAD
              ├── LEFT_COLLAR → LEFT_SHOULDER → LEFT_ELBOW → LEFT_WRIST → LEFT_HAND
              └── RIGHT_COLLAR → RIGHT_SHOULDER → RIGHT_ELBOW → RIGHT_WRIST → RIGHT_HAND
```

每根骨骼的几何参数由父子关节实时计算：

- 中心点：`(parent + child) / 2`；
- 长度：`norm(child - parent)`；
- 方向：`child - parent`；
- 胶囊姿态：将 MuJoCo 胶囊的长轴旋转到父子方向。

## MuJoCo 场景

场景包含：

- 24 个带 free joint 的球体 body；
- 23 个带 free joint 的 capsule body；
- 地面和世界坐标轴参考。

使用 free joint 更新几何体的位置和姿态，避免修改只读的 MuJoCo 派生几何数据。关节球半径和骨骼半径使用固定的可配置视觉尺寸；骨骼长度由实时父子距离决定。

## 坐标系

坐标转换保持为独立函数并通过参数控制：

- MuJoCo 默认使用 Z 轴向上；
- 初始版本保留 PICO 消息的原始轴顺序；
- `scale` 控制米/毫米等单位差异；
- `yaw` 控制人体水平朝向；
- 后续可扩展 `flip-x`、`flip-y`、`flip-z`。

不在实现中假定未经验证的 PICO 轴向；通过参数和简单测试发布器校准。

## 启动接口

计划提供：

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0 \
  --yaw 0
```

参数含义：

- `topic`：输入 PoseArray 话题，默认 `/pico/smpl`；
- `scale`：坐标缩放，默认 `1.0`；
- `rate`：MuJoCo 刷新频率，默认 `60 Hz`；
- `timeout`：数据过期阈值，默认 `1.0 s`；
- `yaw`：整体水平旋转角，默认 `0`。

## 异常处理

- `poses` 少于 24 个时丢弃该帧并记录警告；
- 无消息时保持最后一帧；
- 超过超时阈值时在终端提示，并将骨架显示为灰色；
- 父子关节距离接近零时隐藏对应骨骼；
- ROS 或 Viewer 退出时释放资源并正常关闭线程。

## 测试策略

### 单元级

- 验证 24 个名称到索引的映射；
- 验证父子拓扑边数量和根节点；
- 验证端点到胶囊中心、长度和旋转的计算；
- 验证缩放、yaw 和坐标轴转换。

### 集成级

使用测试发布器发送 24 个关节的 `PoseArray`，验证：

1. Viewer 能启动；
2. 24 个关节和 23 根骨骼可见；
3. 关节运动会实时更新；
4. 消息停止后触发超时状态；
5. 真实 PICO 数据接入后骨架稳定显示。

## 验收标准

- 在 ROS 2 环境中能通过一条命令启动可视化器；
- 收到完整 `/pico/smpl` 后，24 个关节和 23 根骨骼持续更新；
- PICO 断流时不会崩溃，并明确提示数据超时；
- 不影响现有 `pico_bridge`、录制器和其他 ROS 话题。
