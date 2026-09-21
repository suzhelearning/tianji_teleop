# Shared-root TJVR 输入契约（Phase A）

本契约替代旧稿关于“原 PICO 世界系”和“独立解剖 hand”的假设，不修改发布端或线协议。
配置证据见 [输入 artifact](../config/shared_root_tjvr_input_contract.yaml) 和
[机器人 artifact](../config/shared_root_robot_geometry.yaml)。

## 来源与单一所有权

| 输出 | 来源 / 处理 |
|---|---|
| 左肩肘腕掌 | `PicoTeleopFrame.upper_limb_skeleton` 点索引 0、1、2、3 |
| 右肩肘腕掌 | 同字段索引 4、5、6、7 |
| `*_root_Ct` | adapter 从以上绝对点扣除一次 `[0,0,1.121] m`；后续模块不再扣除 |
| control / shape proxy | 同一 M0 重建掌心样本，明确是代理，不是两份观测 |
| 掌旋转 | skeleton hand rotation 后乘一次左右 EE basis；不读取 `frame.left/right` |
| 肩肘腕旋转 | skeleton 对应旋转，已在 bridge frame 中；不乘掌 basis |
| 序列/源时间/接收时间/epoch/generation/discontinuity | 同名 `PicoTeleopFrame` 字段原样透传 |

`bridge_robot_midpoint_frame` 是本地契约名称，不是 TJVR 携带的 frame ID。
bridge 利用双肩及 spine2 构造轴，再平移到高度 1.121 m；此契约不声称其 Z 轴
就是原世界竖直。控制端不重建世界 frame，不重新取当前肩中点作为平移。

`correct_side()` 同时生成重建掌心位置及 `palm_quat`；IK-frame 适配只调整
肩/肘姿态。bridge 将位置与旋转作相同刚体变换，其 skeleton rotation 不应用 EE basis。
报文的 `left/right` 是另一条目标构造支路，可能包含 reach scale / X offset，不能
当成 skeleton 掌姿态的备用字段。

bridge 的状态配对要求同源时间戳、合法 epoch、双侧 `corrected=true`。
M0 的 raw/hold 回退不会通过该状态门。这是仓库发布端实现保证，不是报文对任意
外部发布者的身份认证；artifact 哈希不能证明现场正在运行的远端程序版本。

## 对称骨架

依据用户明确要求，保留既有 `symmetric_max`。M0 可将左右已标定的上臂、前臂
长度分别替换为共同最大值，bridge 对 skeleton 仅做刚体变换，不再缩放骨段。
新估计器应读取这些**米制有效骨长**，不恢复原始不对称观测，不重复对称化。
这不是宣称修正后的长度是未经处理的真实人体测量。

## 机器人几何

共同 B 使用现有 MuJoCo world / Pinocchio universe 对齐后的模型坐标。
固定根方向来自模型基座轴和肩中心关系，不来自随 J1 转动的 Link1 局部姿态。
零位、配置初始姿态和两组非零关节姿态由 `test_shared_root_geometry` 对照。

SPARK 将 MuJoCo 的 Link7→tcp 固定变换传给 Pinocchio；求解 TCP 是 `tcp_L/R`，
不是 flange TCP。当前独立共享根模型的腕中心→TCP 距离为 0.1615 m；
旧模型的 0.1315 m 不作为当前值。Link5 局部向量随腕关节变化；
artifact 中局部向量只是 reference q 的 shape 参考，不得当作固定安装外参。

## 当前接线状态

已实现纯 C++ adapter、尺度估计、目标构造/滤波/intent、连续性管理和 guidance 离线接线。
Viewer 已接通共享根 SPARK、Ceres 和 Franka DLS 分支；模型参考回放和真实输入仿真
均有历史记录，但不能据此宣称完整阶段验收、动力学或真机验收通过。
实验 profile 默认关闭；只有声明 `ConfigConsumer::kSharedRootAware` 的入口可加载
开启的完整契约。其他入口 fail fast，不会静默走 legacy 冒充新算法。
实验模式禁止 joint command export，也禁止会话内切换 IK/控制级别；不改变现有路线。
当前入口与验收边界见 [交互仿真说明](verification/ceres_interactive_sim.md)；
[guidance 报告](archive/2026-09-shared-root/shared_root_phase_a_guidance.md)仅为历史记录。
离线 validator 只验证工作区证据与配置一致性，不授予运动权限，也不等于阶段 6 验收。

连续性模块的 `accept(sequence, epoch, generation)` 只能由最终控制参考接受路径调用，
不能仅因为生成目标、IK 成功或 guidance 返回 `accepted` 就调用。
shared-root guidance 已消费恢复期间普通 stationary hold 的抑制和历史重置信号；
安全保持、stale、Home、失能和故障仍然优先。该接线有离线回归，不代表现场验收。

## zhoujie 简化对称配置

除既有 measured/symmetric_max 来源外，输入契约也允许显式声明的
`left_measured_symmetric_local_y_height_template`：左侧实测 TCP，右侧采用声明的镜像模型，
双侧臂长由身高模板生成。简化 bundle v2 的腕掌距离也由身高估计，v1 才使用左腕实测；
它是有效模型几何，不是双侧独立实测。入口见[简化标定](../../docs/pico-simple-calibration.md)。
控制端仍只消费 M0/bridge 输出，不再镜像 TCP、不再生成一次骨长，也不读取个人文件热更新。

本轮离线输入依据为 `profiles/zhoujie/pico-simple/cal-91c71441c51b47db9366390c96c96199`，
身高 1.62 m。其后已有使用该配置的[动作录制证据](archive/2026-09-shared-root/shared_root_actions_20260918.md)。
历史录制与文件存在不等于当前人员指针已发布；启动时必须验证 `pico-simple/active.json`。
原生合成测试不能证明右侧物理轴、SDK 输出或换人后的现场跟踪效果。
