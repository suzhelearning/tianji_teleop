# 分动作链路诊断与任务冲突对照

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

## 本轮结论

本轮完成用户批准的两个离线任务，没有实施生产参数优化。
七类已标注动作中，已求解样本的 IK 最大位置残差为 0.666 mm；35 个超过 10 mm
的侧周期样本全部位于开头未标注的 0.055～0.340 s。它们对应 14 个不同的
源序号/侧组合，不是 35 个独立输入帧。不能把这些启动样本当成稳定动作的共同瓶颈。

稳定动作中，双手共同运动的控制参考位置误差 p90 为 98.889 mm，且全部处于
TRACKING、无 settled HOLD、无恢复 blend。当前优化优先级应转向受约束前馈和
最终参考跟踪，而不是普遍调整 IK 权重。以下均为离线命令模型参考，不是真实反馈。

## 工具与口径

- C++ 审计新增 `--layer-diagnostics`：完整运行既有 mode=0/1 的 200 Hz 参考闭环，
  输出 mode=1 的逐侧逐周期链路指标，并在回放结束后隔离运行任务消融。
- `report_shared_root_layers.py` 校验原始录制、配置及标注指纹，复用已有动作区间
  校验，按动作、状态、动作×状态和左右侧汇总。缺失值是 null，不填零。
- 逐源帧投影修正成本单列；它来自 standalone mapping，不能冒充经过恢复的控制目标。
- 动作归属使用相对首帧接收时刻的**控制评估时间**，raw 日志保留所消费源帧时间。
  区间采用半开边界；末接收时刻保留，超出末帧的最后一个 200 Hz tick 单列。
- 状态保留 shared-root state、每侧 settled HOLD 与 blend 三个维度，不猜测人工动作。

各误差对应的两个量固定如下：

| 输出 | 比较的量 |
|---|---|
| shift_m | 映射器原始偏好与共同平移后目标的修正幅度 |
| ik_m / ik_rad | FK(q_ik) 与同周期实际 solver input |
| ff_command_m | FK(受约束 feedforward.q) 与 post-HOLD command |
| reference_ff_m | 最终控制参考 FK 与 feedforward.q FK |
| reference_command_m | 最终控制参考 FK 与 post-HOLD command |
| reference_ik_target_m | 最终控制参考 FK 与 HOLD 前 solver input |
| ik_command_m | HOLD 前 solver input 与 post-HOLD command 的差 |

这些范数/分位数不能相加，不直接解释成时间延迟。HOLD 后命令误差接近零，不代表
跟到了原输入：settled HOLD 的 TRACKING 子集，reference_command p90 约 7.7e-8 m，
而 reference_ik_target p90 为 52.476 mm；两者都保留。

## 分动作结果

10001 控制周期，20002 侧周期样本；19848 个纳入 IK 指标、154 个停止/未求解样本
排除并保留原因。参考 step 均接受，不等于掌目标全部准确跟踪。原始录制有 4427
唯一源帧，4093 投递、334 latest-only superseded，不能与侧周期分母混用。

表内位置误差单位为 mm，分位数采用 nearest-rank。

| 动作 | 已求解 / 总侧周期 | IK 最大值 | FF→命令 p90 | 最终参考→FF p90 | 最终参考→命令 p90 |
|---|---:|---:|---:|---:|---:|
| 自然前伸 | 2378 / 2398 | 0.666 | 82.102 | 64.249 | 72.309 |
| 双手靠近 | 2400 / 2400 | 0.014 | 30.629 | 15.891 | 26.669 |
| 胸前交叉 | 2384 / 2400 | 0.014 | 46.714 | 31.759 | 48.155 |
| 一高一低 | 2400 / 2400 | 0.016 | 23.892 | 38.916 | 33.966 |
| 单手工作 | 2400 / 2400 | 0.035 | 83.632 | 64.859 | 72.740 |
| 双手共同运动 | 2400 / 2400 | 0.008 | 120.144 | 76.302 | 98.889 |
| 伸直边界 | 2400 / 2400 | 0.010 | 47.784 | 25.386 | 37.037 |
| 未标注 | 3084 / 3202 | 44.434 | 103.233 | 65.552 | 100.937 |
| 末帧之后的 tick | 2 / 2 | 0.020 | 7.072 | 49.767 | 48.433 |

总体 reference_command p90 为 71.819 mm，p95 为 92.241 mm。最大值 670.151 mm
发生于 0.200 s 的右侧恢复混合；此离线模型从关节区间中点初始化，不是真实设备初态。
该极值不能隐去，也不应与稳定动作混为一谈。
未 settled HOLD 的 TRACKING 子集有 19059 侧周期，reference_command p90 为
70.432 mm，仍明显高于 IK 残差。3519 个参考误差 >50 mm 的侧样本中，3331 个在该子集。
50 mm 仅为排查阈值，不是启用门限。

双手共同运动有 1310/2400 个侧周期参考误差 >50 mm。headroom 主导源计数为
jerk 1706、task_scaling 486、velocity 201、acceleration 7；1469 个侧周期的 scale<0.5。
这是约束活动的相关证据，不是应删除或放宽该约束的因果证明。
该动作的 standalone mapping 538 帧全部无需共同平移，继续扩大平移范围没有现成收益依据。

修正成本：自然前伸最大共同平移 14.177 mm；未标注片段最大 22.908 mm，并有两帧
拒绝、修正量记为缺失。其余六类动作本次 standalone mapping 修正均为零。

## 任务冲突对照

选择原参考回放中 IK 位置残差 >10 mm 的全部 35 个侧周期，逐个固定完整目标及
原求解结果 q 作为共同初值，使用新建冷 solver。不是从原历史输入 seed 重现；
baseline 同样冷启动以供成对比较。保留关节限位、0.05 rad 裕量、信赖域及阻尼，
不将任何结果回灌控制器。每个样本运行七组，共 245 次；全部 accepted。

| 对照组 | 唯一变化 |
|---|---|
| baseline | 无 |
| shape_soft | Stage 1/2 臂段方向、Stage 2 肘腕位置权重乘 0.01 |
| pose_only | 上述臂形权重清零，保留掌位置和姿态 |
| position_only | 在 pose_only 上再去掉两阶段掌姿态权重 |
| iterations_only | 两阶段迭代上限从 10 改为 100 |
| precision_only | convergence_delta 从 1e-4 改为 1e-8 |
| precision_iterations | 同时应用上述迭代与精度变化 |

消融 wall-clock deadline 明确为 unbounded；有限迭代不是现场 4.5 ms 预算认证。
尤其 position_only 的 Stage 1 已没有非零任务，只用于隔离诊断，不是可上线的推荐配置。

| 对照组 | 位置残差 p90 / mm | 姿态残差 p90 / rad | 位置 <10 mm 的样本 |
|---|---:|---:|---:|
| baseline | 44.434 | 0.01489 | 2 / 35 |
| shape_soft | 37.894 | 0.01134 | 3 / 35 |
| pose_only | 37.098 | 0.01064 | 4 / 35 |
| position_only | 5.386 | 0.28612 | 33 / 35 |
| iterations_only | 44.434 | 0.01489 | 4 / 35 |
| precision_only | 44.404 | 0.01435 | 2 / 35 |
| precision_iterations | 44.404 | 0.01435 | 4 / 35 |

原最坏样本 cycle=50/sequence=2554/right，去掉臂形后为 37.097 mm；再去掉姿态要求
后为 5.015 mm，但姿态误差变为 0.28610 rad（约 16.4°）。这提供了位置/姿态与限位
相互制约的证据，不能把牺牲姿态后的结果宣称为原位姿目标已可达。
仅提高精度和迭代预算时，该样本仍约 44.404 mm。有限局部求解不能证明全局不可达；
不少对照仍达到迭代上限，结论保留这一限制。

## 本轮验证与复现

```bash
.pixi/envs/default/bin/cmake --build control/build --target tianji_shared_root_trace_audit -j 2
.pixi/envs/default/bin/python control/scripts/report_shared_root_layers.py \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr \
  --annotations recordings/shared_root/zhoujie_actions_20260918_041020/actions-confirmed.json \
  --output-dir recordings/shared_root/zhoujie_actions_20260918_041020/layer-diagnostics-v1
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py control/tests/test_shared_root_layers.py \
  control/tests/test_shared_root_actions.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_layers.py -q
git diff --check
git diff --exit-code -- pico2_hands pico2_sim.sh real_robot control/models/marvin_m6_wuji2.xml
```

构建与完整录制诊断成功。联合回归 100 passed（133.20 s）；随后新增 native 前 720 帧
集成校验，专项 23 passed（28.42 s），两组有重叠，不相加冒充独立测试数；均无跳过。
覆盖缺失/重复消融配对、时间边界、非有限值、缺失值与有效性掩码、指纹、无标签、
分母、权限标记，以及 native 分层 p90 与原总汇总一致性。
剔除新增行和计时行，原有 47 行汇总与上轮日志在六位有效数字舍入精度内一致。
没有重跑完整 CTest。旧路线差异检查和 `git diff --check` 通过。

输出目录已存在，重跑需换一个新目录；工具拒绝覆盖。`native.txt` 保存逐周期与逐组
原始证据，`report.json` 为可检查汇总。原 trace、profile、标注及历史日志均未修改。
trace SHA256：`20340067256a3a78205c78a0d5ba6fc01f818d14da5e1e57f116391451df2f2b`。
profile SHA256：`7925a889075b062d95cd5e79bbb5fed6fc553fbe6097a11bb94c11d6ee550da6`。
全部数据保持 `actual=unavailable`、`motion_authorized=false`、`phase_a_accepted=false`。

## 建议的下一轮

优先以“双手共同运动”为主集合，做保持硬约束不变的前馈/headroom/最终参考接线
对照，检查软平滑及反馈配合是否引入可避免的滞后；其他六类动作必须同时做回归。
启动大残差单独处理，先核对初始化与恢复目标，不用降低整个会话的姿态要求来解决。
本轮没有授权或实施线上参数替换、门限冻结、设备试验、提交或 push。
