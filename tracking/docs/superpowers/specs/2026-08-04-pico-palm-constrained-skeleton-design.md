# PICO 掌心约束骨架快速验证设计

## 1. 目标

在不依赖外骨骼、Odin、人体尺寸标定或完整上肢 IK 的前提下，快速验证：

- 现有 PICO controller-to-palm TCP 标定能否提供稳定的左右掌心位姿；
- 能否参考 PICO 自带 24 点骨架，仅用掌心观测修正左右上肢；
- 修正后是否能同时满足掌心位置、固定上臂长度、固定前臂长度和连续肘部分支。

该版本参考 `/home/zj/下载/myfilter_standalone.py` 的固定骨长、三角形解析肘部和最小旋转更新方法，但把原脚本内部生成的腕部端点替换为 TCP 标定后的独立掌心观测。

## 2. 范围与非目标

### 2.1 本轮实现

- 继续使用已有左右 PICO TCP artifact 和 `/pico/palm_left`、`/pico/palm_right`。
- 使用 PICO 原始 24 点骨架作为基线。
- 自动估计左右上臂长度和前臂长度；腕掌 artifact 只提供标量距离。
- 按侧独立重建肩、肘、腕和掌心。
- 持续发布完整 24 点修正骨架。
- 支持在现有 MuJoCo viewer 中与原始 PICO 骨架叠加比较。

### 2.2 明确不做

- 不接入外骨骼、Odin、ESEKF、QP 或人体全身 IK。
- 不重新估计骨盆、躯干、头部或下肢。
- 不增加人工上臂/前臂长度标定动作。
- 不写入正式人体模型或外骨骼 operator profile。
- 不覆盖 `/pico/smpl_raw`、`/pico/smpl` 或现有掌心 topic。
- 不用于 active control。

## 3. 坐标和消息契约

### 3.1 输入

| Topic | Type | Frame | 语义 |
|---|---|---|---|
| `/pico/smpl_raw` | `geometry_msgs/msg/PoseArray` | `pico` | APK 原始 24 点 PICO 骨架 |
| `/pico/palm_left` | `geometry_msgs/msg/PoseStamped` | `pico` | TCP 标定后的左掌心位姿 |
| `/pico/palm_right` | `geometry_msgs/msg/PoseStamped` | `pico` | TCP 标定后的右掌心位姿 |

必须使用 `/pico/smpl_raw`，不能在本节点中直接使用 `/pico/smpl`。后者已经经过地面 Z 平移并使用 `pico_ground`，而当前掌心 topic 仍使用 `pico`。节点不猜测或隐式补偿两个 frame 之间的变换。

24 点索引沿用现有协议：

- 左肩、左肘、左腕、左手：`16, 18, 20, 22`；
- 右肩、右肘、右腕、右手：`17, 19, 21, 23`。

### 3.2 输出

| Topic | Type | Frame | 语义 |
|---|---|---|---|
| `/pico/smpl_palm_corrected` | `geometry_msgs/msg/PoseArray` | `pico` | 掌心约束后的完整 24 点骨架 |
| `/pico/smpl_palm_corrected/status` | `std_msgs/msg/String` | 不适用 | JSON 格式的快速验证状态 |

输出 header 使用触发计算的 `/pico/smpl_raw` header。输出必须始终包含 24 个 pose。除左右上肢索引外，其余 pose 与输入逐字段一致。

## 4. 自动基线

节点启动后不要求额外人工标定动作。用户只需保持 PICO 正常跟踪约 2 秒。

每侧从时间配对成功的骨架和掌心样本中估计：

- 固定上臂长度 `L_upper`：肩到肘距离的稳健中值；
- 固定前臂长度 `L_forearm`：肘到腕距离的稳健中值；
- 兼容腕掌 artifact 的三维向量模长 `d_WH`；
- 固定掌心位于腕/掌心姿态局部 `+X`，并令 `R_G_W = R_G_H`。

对每个基线样本：

```text
d_WH = norm(wrist_to_palm_m)
p_G_W* = p_G_H - R_G_H [d_WH, 0, 0]^T
R_G_W* = R_G_H
```

其中：

- `G` 是 `pico`；
- `H` 是 TCP 掌心；
- `W` 是 PICO 骨架腕部。

长度使用中值，平移使用逐轴中值，旋转使用带四元数同半球处理的归一化均值。单侧至少需要 60 个有效配对样本；左右侧独立进入 ready。

基线样本必须满足：

- 骨架严格为 24 点；
- 所有相关位置和四元数有限；
- 四元数可归一化；
- frame 均为 `pico`；
- 掌心与骨架时间差不超过 30 ms；
- 上臂和前臂长度均在 `[0.10, 0.50] m`；
- 单帧长度相对当前中值偏差不超过 15%。

本快速版本不持久化自动基线。节点重启后重新收集，避免把尚未验证的数据误当作正式人体尺寸 artifact。

## 5. 运行时重建

每个新的 `/pico/smpl_raw` frame 到达时，分别处理左右侧。

### 5.1 掌心反算腕部

使用同侧最新且时间匹配的掌心观测：

```text
p_G_W* = p_G_H + R_G_H r_H_W
R_G_W* = R_G_H R_H_W
```

HAND pose 直接使用 TCP 掌心位姿：

```text
T_G_HAND* = T_G_H
```

因此输出索引 22/23 的语义在该实验 topic 中明确为掌心，而不是原始 PICO HAND 估计。

### 5.2 可达范围

肩部位置保留原始 PICO 骨架值。令肩到目标腕部的距离为 `d`：

```text
d_min = abs(L_upper - L_forearm) + epsilon
d_max = 0.995 (L_upper + L_forearm)
```

所有路径都沿肩到目标腕部方向投影到最近可达边界，并在 status 中记录 `reach_clamped=true`。这样上臂和前臂长度始终固定；不可达时用最终腕点沿掌心局部 `+X` 重建 HAND，并将其与 TCP 掌心强观测的距离记录为 `wrist_palm_position_residual_m`。TCP 掌心仍作为独立观测保留，不能被静默改写。

### 5.3 肘部解析解

在肩和目标腕部确定后，肘部位于两球交圆上。沿用参考脚本的三角形解析方法：

- 两条边固定为 `L_upper` 和 `L_forearm`；
- 首选原始 PICO 肘部在交圆平面内的投影作为 branch reference；
- 原始肘部退化时使用上一帧已接受的修正肘部；
- 两者都退化时，该侧回退到原始 PICO 上肢。

该方法不求解完整 7DoF 关节角，只修正几何骨架，因此不存在 DLS/QP 不收敛导致整帧停止的问题。

### 5.4 姿态更新

- 肩部姿态：将原肩到肘方向最小旋转到修正方向，再左乘原肩姿态；
- 肘部姿态：将原肘到腕方向最小旋转到修正方向，再左乘原肘姿态；
- 腕部姿态：使用 `R_G_W*`；
- HAND/掌心姿态：直接使用 `R_G_H`。

所有输出四元数必须归一化，并保持与上一输出同半球，避免符号翻转。

## 6. 时间同步与持续发布

节点以 `/pico/smpl_raw` 为触发源，并为左右掌心分别维护有限长度缓存。

- 每侧选择 corrected/source timestamp 与骨架最接近的掌心样本；
- 最大允许差为 30 ms；
- 不使用 ROS receive time替代 source stamp；
- 不跨时间倒退复用样本；
- 左右掌心不要求同一帧同时有效。

每个有效的 24 点骨架输入都必须产生一个输出：

- 某侧 ready 且掌心新鲜：输出该侧修正结果；
- 某侧未 ready、超时、frame 不匹配或计算退化：该侧四个 pose 原样保留；
- 另一侧、躯干和下肢不受影响。

该策略保证掌心或单侧重建失败不会冻结整套 PICO 骨架。

## 7. 状态与故障处理

status JSON 至少包含：

```json
{
  "left": {
    "baseline_ready": true,
    "corrected": true,
    "fallback_reason": "",
    "time_skew_ms": 4.2,
    "reach_clamped": false
  },
  "right": {
    "baseline_ready": true,
    "corrected": false,
    "fallback_reason": "palm_stale",
    "time_skew_ms": 48.0,
    "reach_clamped": false
  }
}
```

失败必须按侧 fail-open 到原始 PICO 上肢，但不能输出 NaN、部分 PoseArray 或陈旧修正结果。frame 不一致属于配置错误，节点持续发布原始骨架并节流报警。

## 8. 实现边界

快速版在 `pico_bridge` 中新增独立 Python ROS 2 节点，不修改：

- `pico_bridge_node`；
- `pico_smpl_ground_node`；
- TCP 标定器的求解和 artifact schema；
- 现有 raw/canonical/fused topic；
- 外骨骼仓库代码。

核心几何函数与 ROS 节点分离，使固定骨长、腕部反算、交圆肘部解和姿态更新可以不启动 ROS 直接测试。

## 9. 测试

### 9.1 单元测试

- 掌心到腕部刚性变换正向/反向组合一致；
- 左右侧索引映射正确；
- 肩肘、肘腕长度在重建后保持固定；
- HAND position/orientation 与输入掌心一致；
- 原始肘部选择的 branch 不发生镜像翻转；
- 超出最大可达范围时腕部正确 clamp；
- 肩腕重合、共线、零四元数和非有限输入安全失败；
- 最小旋转在同向和反向向量下均有限；
- 左侧失败不改变右侧结果；
- 非上肢 16 个 pose 逐字段不变。

### 9.2 节点测试

- 24 点骨架加双侧同步掌心产生 24 点修正输出；
- 30 ms 内样本被接受，超出门限按侧回退；
- `pico_ground` 掌心或骨架被拒绝；
- 基线不足时持续转发原始骨架；
- 单侧掌心 dropout 不阻塞另一侧和躯干/下肢；
- 输出 header 与触发骨架完全一致；
- 每个有效骨架输入都有且仅有一个输出。

## 10. 快速验收

软件验收：

- 全部新增测试通过；
- 现有 `pico_bridge` 测试无回归；
- 1000 帧合成回放无 NaN、无长度漂移、无输出丢帧；
- 掌心未 clamp 时，HAND 与 TCP 掌心的位置误差不超过 `1e-6 m`，姿态误差不超过 `1e-6 rad`；
- 非上肢 pose 与输入严格一致。

真机快速验收：

- 正常站立、双臂前伸、侧举、屈肘和双手合拢；
- 原始与修正骨架同时显示；
- 修正 HAND 与左右 TCP 掌心标记重合；
- 上臂和前臂长度在动作过程中不发生可见伸缩；
- 肘部不发生左右镜像翻转；
- 遮挡一侧 controller 时，另一侧及躯干/下肢继续更新。

推荐可视化方式：主 topic 使用 `/pico/smpl_palm_corrected`，raw overlay 使用 `/pico/smpl_raw`，并保留左右掌心球和坐标轴。

## 11. 成功后的下一步

只有该快速实验确认 TCP 掌心能明显改善 PICO 上肢骨架后，才考虑：

- 把自动基线升级为可持久化人体尺寸 artifact；
- 将 corrected raw skeleton 接到现有 ground alignment；
- 与外骨骼建立显式 frame adapter；
- 再评估完整 IK、ESEKF 或 exo soft prior。
