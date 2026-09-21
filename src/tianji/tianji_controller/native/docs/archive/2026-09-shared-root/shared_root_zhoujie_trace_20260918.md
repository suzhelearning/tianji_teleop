# zhoujie 50 秒真实输入：首轮逐源帧 IK 对照

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

## 当前约束变更（用户确认，2026-09-18）

删除 shared-root 原始及滤波目标的双掌间距 1.6 m 拒绝条件，配置键
`maximum_relation_norm_m` 一并删除。不同臂展不再仅因双掌间距触发拒绝。
保留有限数值检查、中心/修正量门控、输入新鲜度、连续性管理及 IK/执行层关节限位。
10 mm 仅作为描述性统计，不再作为验收阻塞条件；不为追求该精度增加现场迭代次数。
现场迭代上限、收敛阈值及权重保持原配置，离线组 3～5 不是现场推荐参数。
以下为删除门限前的历史结果，不能作为修改后的运行结果。

修改后固定配置完整回放成功：4388 帧输入，shared-root 原权重组输入拒绝为 0
（原为 669），4384 帧可统计、4 帧未纳入 IK 统计；没有预算耗尽。
掌位置 p90 为 65.619 mm，仅如实记录，不据此继续增加迭代或判定验收失败。
由于不再回滚宽臂展帧的尺度估计，提交尺度下最大双掌间距为 1.81548 m，
不应与删除门限前的 1.76916 m 候选统计直接混用。
此次 profile SHA256：`f9262d6186315341e1daeb9693b16eebfd14c7d6dd38f997099a0e29be97e9ca`。

验证：全量 C++ 构建成功；15 组相关 CTest 通过（17.19 s），包括新增 1.8 m
原始/滤波目标通过、中心和非有限值仍拒绝、安全/控制器/原 SPARK 回归；
Python 契约及 PICO2 回归 66 项通过（4.59 s），trace/CLI 12 项通过（0.36 s）。
首次回放因补写配置注释触发配置变更检查而中断，固定配置重跑退出码为 0。
未连接设备，未完成现场验收，未提交或 push。

## 数据和执行范围

录制：`recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr`。
SHA256：`8d92f338056a40f73602e6d029c807d807dea29cc6e378b15946d85d3354ab62`。
4388 帧，epoch 333；同目录有完整性状态、启动日志、源码哈希与 calibration 快照。
双侧骨长为 0.25252884000000003 / 0.24776442 / 0.05999994 m。
录制后几何校验全部通过；骨长来自有效模型，不是独立实测人体长度。

按用户要求，录制会话的 driver/M0/bridge 先发送 Ctrl-C，确认进程退出后删除其空 tmux
会话；没有停止其他工程进程。此后不启动设备、Viewer 或执行器。

新增 `tianji_shared_root_trace_audit`：直接读取 TJVT，复用原生 TJVR 解码器及现有 guidance，
不使用 socket、不调用 mj_step、不导出关节命令。各组从相同模型关节限位中点初始化，
固定模型快照，不用 IK 输出冒充 actual；内部 IK seed 依原 guidance 更新。
使用记录的接收间隔（夹在 1～100 ms），不是 200 Hz 调度回放。
shared-root 不伪造外部参考接受确认，处于 RECOVERING/保持状态，不能替代完整控制闭环。
当前未复现 receiver-local stream gate/generation，也没有测试真实时序抢占或动态约束。

## 对照结果

所有组均使用 profile 原有 4.5 ms IK 预算，未发生预算耗尽。
统计针对各组实际通过目标/双侧 IK 检查的帧，位置样本为双侧合计。

| 组 | IK 可统计帧 | 排除帧 | 掌位置 p90 | 姿态 p90 | B 系双掌向量误差 p90 |
|---|---:|---:|---:|---:|---:|
| 0：legacy + 原权重 | 4388 | 0 | 5.711 mm | 0.001677 rad | 13.044 mm |
| 1：shared-root + 原权重 | 3690 | 698 | 63.310 mm | 0.009574 rad | 100.777 mm |
| 2：shared-root，Stage 2 肘/腕位置权重 0.25 | 3690 | 698 | 62.211 mm | 0.009538 rad | 97.841 mm |
| 3：组 2 + Stage 2 骨段方向权重 0.05 | 3690 | 698 | 10.199 mm | 0.001477 rad | 10.930 mm |

组 1/2/3 的输入候选均有 669 帧被 target_gate 拒绝；其他 29 帧不能被算作成功帧。
组 3 的 7380 个侧样本中仍有 781 个位置误差 >10 mm。
不能只报告排除后的 p90、不报告覆盖率；也不能把 legacy 和 shared-root 的不同目标语义
或不同样本集合当作同一个外部 ground truth。所有 actual 指标 unavailable。

独立 mapping-only 探针同样记录到 target_gate 669 帧。
以**提交后的尺度估计**重算，最大中心偏移约 0.729359 m（门限 1 m），最大双掌间距
约 1.75162 m（门限 1.6 m），661 帧超过间距门限。这不是门内试算尺度：被拒帧会回滚
尺度窗口，因此不能把 661 与 669 强行等同，也不能据此声称逐帧归因已完全闭合。
未放宽目标门限，未更改现场权重或默认开关。

## 可重复命令和验证

```bash
pixi run cmake --build control/build --target tianji_shared_root_trace_audit -j 2
pixi run control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root.yaml \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py control/tests/test_shared_root_trace_audit.py \
  control/tests/test_shared_root_contract.py pico2_hands/tests -q
```

原生工具构建及完整录制运行成功；Python 78 项通过（4.74 s），含原生读文件负例：
错误头部、空数据、过量条数、截断、非法报文。未把这些测试通过称为跟踪验收通过。

## 尚未完成

阶段 6 不验收通过。组 3 尚略高于 10 mm p90 门限，且存在目标门限排除；不能从单条
录制冻结参数。下一步需要门限拒绝的精确候选尺度归因、共同有效集合对照、分动作统计、
完整参考接受/调度离线闭环、尺度消融及负例集合。保留 CauseUndetermined，不能仅凭
加权残差认定全局不可达。输入的物理镜像轴仍需现场独立验证。
新功能继续默认关闭；未提交、merge 或 push。

## 后续诊断：门控与求解停止原因（2026-09-18）

本轮只扩展离线 audit，不修改现场 profile、求解器、驱动或保护条件。

### 候选尺度门控

离线工具独立调用现有 morphology/builder，保留失败回滚和中断后恢复语义，逐帧核对
其 valid/detail 与 pipeline 一致；遇到本探针未支持的 invalid/rebuild 上下文立即报错。
它是该录制的诊断探针，不是替代生产 pipeline 的通用实现。

在门控实际试算的候选尺度下，669 帧全部超过双掌间距 1.6 m 门限，最大约 1.76916 m；
中心门限拒绝为 0。此前提交后尺度得到的 661 帧不能用作门控当时的逐帧原因。
本结论说明软件拒绝的直接条件，不证明门限应该放宽，也不证明这些目标全都可达。

### 收敛与残差

`spark_upper_qpoases_ik.cpp` 的 Stage 2 QP 同时优化掌与臂形任务，线搜索要求掌主任务
改善，但停止条件使用总加权误差变化量。`converged` 不代表掌位置达到 10 mm。
原 shared-root 组 7380 个侧样本中 7333 个标记 converged，其中 5990 个仍超过 10 mm。

| 组 | 掌位置 p90 | >10 mm 侧样本 | 最坏位置误差 |
|---|---:|---:|---:|
| 3：降低方向与肘腕位置权重，Stage 2 上限 10 | 10.199 mm | 781/7380 | 106.512 mm |
| 4：组 3，Stage 2 上限 50 | 9.447 mm | 660/7380 | 58.819 mm |
| 5：组 4，convergence_delta 从 1e-4 改成 1e-7 | 8.817 mm | 521/7380 | 48.480 mm |

convergence_delta 为 Stage 1/2 共用参数，组 5 不能归因成仅改变 Stage 2。
组 5 中 3035 个侧样本以 stage_solved 结束且达到 Stage 2 的 50 次上限。
这些组本次均无预算耗尽；不能据此宣称现场 200 Hz 达标。

组 5 超过 10 mm 的 521 个侧样本中，仅 70 个距离任一安全关节界限不足 0.01 rad
（安全界限包含现有 joint_limit_margin_rad）。因此不能把全部残差归因于撞限位；
这也不是奇异性、不可达性或约束活跃集的完整判定。
组 5 最坏样本：sequence 7736，右侧，48.480 mm。
原 legacy 组最大误差 806.138 mm，sequence 5042 左侧；其较小 p90 不意味着没有极端值，
需要分开检查初始 seed/恢复阶段，不能宣称 legacy 在所有帧均优于新映射。

结论：方向任务权重、迭代上限和收敛停止条件都影响本录制残差；尚不能认定根因完全
闭合，更不能只凭本次调参冻结现场配置。下一步优先围绕上述源序列分析目标、seed、
任务残差和恢复阶段，并补全参考接受的离线闭环。

本轮重新构建 audit 并运行完整录制成功；相关 CLI/trace Python 测试 12 项通过
（0.37 s），git diff --check 通过。第 6 阶段仍未验收，未运行设备或提交代码。
