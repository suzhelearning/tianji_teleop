# Shared-root 掌优先权重实验

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

独立完整配置：`control/config/qp_ik_pico_shared_root_palm_priority.yaml`。
默认关闭；保留原 `qp_ik_pico_shared_root.yaml`，不影响其他路线。

只修改 Stage 2 四项：

| 权重 | 原值 | 实验值 |
|---|---:|---:|
| upper_direction | 5 | 0.05 |
| forearm_direction | 5 | 0.05 |
| elbow_position | 5 | 0.25 |
| wrist_position | 5 | 0.25 |

掌位置/姿态、Stage 1、迭代次数、收敛阈值、预算、所有限位和安全参数不变。
新增 profile 测试逐项比较完整 YAML，断言除此四项外完全一致。

同一 zhoujie 录制使用 `tianji_shared_root_trace_audit PROFILE TRACE --reference-loop-200hz`。
原 profile SHA256：`f9262d6186315341e1daeb9693b16eebfd14c7d6dd38f997099a0e29be97e9ca`。
实验 profile SHA256：`b7ee71bada2829a95fa6e5458781399009b95a4c97ad7100394d80f0b0048aeb`。
trace SHA256：`8d92f338056a40f73602e6d029c807d807dea29cc6e378b15946d85d3354ab62`。

## 首轮结果

**后续审计更正：本节两组都缺少 Viewer 实际使用的 `updateHeadroomFeedback` 接线，
因此只能视为旧离线工具结果，不能据此判定完整 Viewer 链路的权重收益。
已修复离线工具；新结果在文末追加。生产 Viewer 本身已有这段反馈，未修改。**

实验组 9999 控制周期全部下游参考接受，映射拒绝和预算耗尽均为 0，4 次恢复确认、
9828 周期 TRACKING。44 周期为原 feedforward stopping，仍有受约束参考输出。

| 分层指标 | 原配置 | 权重实验 |
|---|---:|---:|
| pose IK 掌位置 p90 | 66.059 mm | 40.956 mm |
| pose IK 掌姿态 p90 | 0.009784 rad | 0.002353 rad |
| pose IK 双掌向量误差 p90 | 124.873 mm | 78.780 mm |
| controller reference 掌位置 p90 | 161.223 mm | 165.903 mm |

不能把旧的固定模型 IK-only 实验约 10 mm 结果直接套用到本闭环：现在模型与 seed
随控制参考变化，且经过完整 guidance 的 blend、hold、feedforward 接线。
新权重改善了姿态 IK 的任务折中，但没有改善本次下游参考误差。
目前不替换默认配置，不宣称全链路跟踪改善，也不据此放宽关节约束。
该录制不是物理实际反馈；未验证现场关节抖动、动作主观效果或碰撞安全。

验证：profile 隔离与 contract 共 9 项 Python 测试通过（0.28 s），artifact 验证通过。
回放逐周期 finite/关节限位检查未失败。本轮未启动设备/Viewer，未提交或 push。

## 补齐 headroom 反馈后的重跑

离线工具现在按 Viewer 的方法把 accepted、qdot、上一参考加速度与任务缩放反馈给
`updateHeadroomFeedback`；本机时间的新鲜度参与反馈有效性。现场已有该接线，未改。
新增自动断言要求 reference-loop 的 feedback_cycles 与控制周期数一致，防止遗漏回归。

| 指标 | 原配置 | 四项新权重 |
|---|---:|---:|
| IK 掌位置 p90 | 66.059 mm | 40.944 mm |
| IK 掌姿态 p90 | 0.009651 rad | 0.002258 rad |
| IK 双掌向量 p90 | 124.823 mm | 78.733 mm |
| reference 掌位置 p90 | 160.379 mm | 163.960 mm |

两组均 9999 周期参考接受、0 输入拒绝、0 预算耗尽、2 次恢复确认、9917 周期 TRACKING。
18 周期为受约束 stopping，无新 pose IK；不叫作求解失败。
补齐反馈后仍未观察到参考跟踪收益；不能把姿态 IK 改善等同于端到端改善。
后续只做分层诊断，不擅自削弱速度/加速度/jerk 或限位保护。

新权重分层探针（同一固定 200 Hz 回放）：

- feedback_cycles=9999，所有控制周期都执行反馈。
- 受约束 feedforward.q 的 TCP 与掌目标误差 p90=187.580 mm。
- 最终 reference TCP 与 feedforward.q TCP 的差 p90=90.691 mm。
- 19962 个有效侧样本中，14438 个 headroom.scale<0.5（约 72.3%）。
- headroom 主导源：task_scaling 9151、velocity 7378、jerk 3218、acceleration 211、none 4。

这是同一时刻不同层的误差统计，不可相加，也不是相位延迟秒数。
说明大误差已经存在于受约束前馈参考；下游参考还包含 Cartesian 任务反馈，
并非机械地复制 feedforward.q。主导源计数不是“应删除该约束”的证据，
需用保留硬边界的独立消融才能区分软平滑、参考跟随增益与物理速度包络的因果作用。

修复工具后重新构建成功，trace/CLI/contract/profile 共 41 项测试通过（90.55 s），
包含逐源帧及 200 Hz 回放反馈次数断言；git diff --check 通过。
