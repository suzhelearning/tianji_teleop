# Shared-root：映射设计与实现边界

实验配置：`control/config/qp_ik_pico_shared_root.yaml`，默认 `enabled: false`。
关闭时直接进入原 legacy SPARK；不改变 mapped-palm、PICO2 裸手或手部来源。
当前不是现场或真机验收许可；共享根 SPARK 实验 Viewer 仍禁止导出关节命令。
`teleop.sh --sim`、`--real` 和 `--data` 默认后端均为共享根 Franka DLS＋Ruckig；
仅真机执行入口显式启用受保护的 DLS loopback 参考导出，仍由独立执行器授权和保护设备。
本页说明共用映射及 SPARK 分支，不是默认后端选择说明。
运行入口、导出限制与验收边界见[交互仿真说明](verification/ceres_interactive_sim.md)。
[R3 阶段交付索引](archive/2026-09-shared-root/shared_root_r3_handoff.md)保留历史指纹与测试，不代表当前文件指纹。

R3 状态：闭合几何 artifact v2、读取核验和纯几何闭合函数已实现，
详见 [R3 几何验证](archive/2026-09-shared-root/shared_root_r3_geometry.md)。
TargetBuilder 的 raw/filtered、恢复 blend 和控制分支接受现已接入闭合函数。
掌心目标不变，由固定腕掌外参及两连杆闭合得到肘腕；只有同周期双侧外部参考
接受才提交肘分支历史。详见 [R3 接线验证](archive/2026-09-shared-root/shared_root_r3_wiring.md)。
上述 SPARK 实验配置维持默认关闭；DLS 启动器在运行副本中启用共享根。软件与离线通过不替代代表性动作覆盖率及设备验收。
连续失效时长、分侧几何原因和未标注动作边界见
[R3 覆盖率补充](archive/2026-09-shared-root/shared_root_r3_coverage.md)。
人工动作区间的格式与离线命令见
[分动作统计入口](verification/shared_root_action_coverage.md)。

## 数据链路

### 掌心 TCP（2026-09-18）

经用户静态观察确认，新路线采用左右 Link7 局部位置
`[0, -0.1615, 0] m`，姿态不变。独立模型为
`control/models/marvin_m6_wuji2_shared_root.xml`；仅移动 TCP sites、显示坐标轴
和连接标记，不移动手部网格、手指或关节。
原模型 `marvin_m6_wuji2.xml` 的 0.1315 m TCP 保持不变。

启用 shared-root 时 Viewer 默认加载独立模型；显式传入不匹配模型将拒绝启动。
关闭 shared-root 不改变原模型选择。离线审计从 geometry artifact 加载新模型，
Pinocchio 使用该模型相对 Link7 的 TCP 外参，臂形末段和机器人 reach 同步更新。
这会改变人体到机器人 reach 尺度；旧 TCP 的误差统计不能作为本版本结果。
本次定位来自静态观察，不代表精密掌心标定或设备跟踪验收。

TJVR 已有双肩参考系骨架 → `TjvrSharedRootInputAdapter` → morphology →
target builder/filter → continuity → SPARK guidance → 原姿态 IK 与 velocity-QP。

- bridge 肩中点 `[0,0,1.121]` 只由 adapter 扣除一次，不重算 yaw/世界坐标。
- 骨架点 3/7 是 M0 重建掌心，control 与 shape proxy 来自同一观测。
- 姿态 basis 仅在 adapter 应用一次，不混用报文另一套掌姿态作为 fallback。
- 机器人几何来自冻结的 URDF/MuJoCo artifact，不依赖用户启动姿势。
- 尺度使用双侧有效骨长；沿用上游对称骨架，不在控制端再次对称化。
- 当前使用共同各向异性尺度；统一尺度仅有离线消融，不增加现场切换参数。
- 已按用户要求删除双掌间距 1.6 m 门控；中心、修正量、有限值、新鲜度、
  身份、授权、故障以及关节限位保护保留。10 mm 仅为诊断参考。

## 恢复与命令所有权

短异常保持最后映射，长异常使目标无效；不回退旧 SPARK。
恢复需唯一 fresh 源帧、同步 blend，以及同周期双侧控制参考接受。
恢复期间普通静止保持不阻止到达新目标，但 Home/失能/故障始终优先。
不能用“姿态 IK accepted”代替外部参考确认。

## 合并后的实现对应关系

| 部分 | 当前实现与不能省略的约束 |
|---|---|
| 输入适配 | `shared_root_input.*`；固定原点、单次 basis、元数据透传，不使用另一掌姿态支路 fallback |
| 尺度估计 | `shared_root_morphology.*`；有效米制骨长、唯一源帧、PROVISIONAL/CONFIDENT，不把重复控制周期当新样本 |
| 目标与闭合 | `shared_root_target_builder.*`、`shared_root_closure.*`；唯一掌 pose、固定肩点/骨长、两球交圆选肘 |
| 恢复与提交 | `shared_root_continuity.*`、pipeline/guidance；HOLD 冻结历史，恢复重新闭合，同周期双侧控制参考 ACK 才提交 |
| 诊断与证据 | 原生 audit、动作采集/统计；区分映射、IK 目标、整形后命令、模型参考和 actual |

几何 artifact v2 冻结肩/肘/腕位置 frame、两段长度、Link7→TCP 与 TCP→腕中心外参。
WristCenter 是原点在腕中心、轴平行 TCP 的虚拟坐标系，不是固定的 Link5 姿态。
恢复时插值掌 pose 和肘偏好，再重新闭合；不能插值骨段并拉伸骨长。
跨模型 FK/Jacobian 一致性容差与目标链代数闭合容差是不同口径，不互相替代。
raw/filtered 只读分支历史；恢复以同一模型快照的 FK 肘点为参考。
HOLD 诊断可能显示当前模型 TCP，不能拿它当原始映射候选统计误差。

### 不可达近似与统计口径

原拒绝配置不变；reachable、DLS/Ceres 配置按显式选项采用有界共同平移近似。
它同时约束左右 raw/filtered 腕点的外可达球、修正幅度与修正速度，保持双掌相对
位置和姿态不变；不修改骨长、肩点。内半径、肘分支、闭合残差及安全门仍需通过。
有限预算无法满足约束时继续拒绝，不保证完整关节空间可达或全局最优。
实现细节和被拒绝样本见[投影记录](archive/2026-09-shared-root/shared_root_reachable_projection.md)。

覆盖率按唯一源帧计算，失效时长按录制接收单调时间及 freshness 计算；两者不能互换。
未标注片段单列，不猜动作；闭合未评估也不算几何不可达。坏包/倒序不能跳过后再宣称高覆盖率。
standalone mapping、200 Hz latest-only 回放、IK 已接受子集的分母不同。
七类动作已有使用者确认的[录制证据](archive/2026-09-shared-root/shared_root_actions_20260918.md)，
早期“没有动作标注”已不是当前结论，但这些证据不等于启用门限或真机验收。

## 离线验证入口

### 仅映射可视化（机械臂静止）

```bash
.pixi/envs/default/bin/python control/scripts/view_shared_root_mapping.py \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr
```

使用原生审计工具的 `--mapping-frames` 输出，播放 FILTERED 层，不含 guidance
恢复 blend、IK 或动力学。机器人固定 J2=-90°、其余零；不接设备或发送命令。
橙色/绿色为左右闭合臂形，白球和 RGB 轴为唯一掌目标；新 R3 中骨架末点
与白球重合，旧版紫色间距线应为零长度。
P 暂停/继续、R 从头播放、Q 退出；播完停在末帧。无窗口校验加 `--check`。
无效帧隐藏目标。静态模型的 TCP 和动态期望目标不重合，不应当作跟踪误差。

构建 `tianji_shared_root_trace_audit` 后，以 `PROFILE TRACE` 为参数：

| 可选参数 | 验证内容 |
|---|---|
| 不加参数 | 固定模型、逐源帧姿态 IK 和历史权重诊断 |
| `--mapping-only` | 不运行 IK；检查 raw 双掌关系/中心/姿态、raw/filtered 肩点和骨长，并报告掌目标与臂形末点间距 |
| `--reference-loop` | legacy/shared-root 原权重，命令模型参考更新与恢复确认 |
| `--reference-loop-200hz` | 固定 5 ms 合成时钟，按录制到达时间 latest-only 消费源帧 |
| `--scale-ablation` | 同帧各向异性/统一尺度，filtered target 的姿态 IK 对照 |

这些工具没有硬件运动权限，不产生执行端关节发布。reference 不是真实 actual。
统一尺度对照共用 morphology 样本；两组分别保留 filter 与 IK seed 历史，
比较同一有效帧集合。每侧求解预算取 profile 值，不能把该实验当端到端周期预算验收。
200 Hz 回放保留原接收时间，不因控制周期重复刷新 freshness；被 latest-only 覆盖的
帧计为 superseded，不声称网络丢包。无睡眠等待，不构成操作系统实时调度验收。

详细证据：

- [0.1615 m TCP 映射层复核](archive/2026-09-shared-root/shared_root_tcp1615_mapping.md)

- [输入契约](shared_root_input_contract.md)
- [阶段 1 审计](archive/2026-09-shared-root/shared_root_phase_a_stage1.md)
- [guidance 验证](archive/2026-09-shared-root/shared_root_phase_a_guidance.md)
- [原录制诊断](archive/2026-09-shared-root/shared_root_zhoujie_trace_20260918.md)
- [参考接线回放](archive/2026-09-shared-root/shared_root_reference_replay_20260918.md)
- [尺度与回归补充](archive/2026-09-shared-root/shared_root_scale_regression_20260918.md)

设备、动力学、碰撞安全、真实反馈和真机仍需另行授权与验收，不从离线通过推断。
