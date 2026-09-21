# Phase A 后续交付复核：动作证据与 IK 审计修正

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

当前用户要求先收尾映射、后置 IK：[映射拒绝与连续性复核](shared_root_mapping_transitions_20260918.md)
已完成，两帧为速度保护与外可达约束冲突，未放宽保护，未改生产映射或 IK/QP。

最新进展：[分动作链路诊断与任务冲突对照](shared_root_layers_20260918.md)已完成。
七类动作已求解样本的 IK 最大残差不足 0.67 mm；大 IK 残差全部在开头未标注片段。
后续优化重点转向持续动作的受约束前馈及参考跟踪，未修改生产参数或启用配置。

日期：2026-09-18。对应实施方案 V2.1(5) 阶段 1R～6；未进入 Phase B。
当前 HEAD 为 `b0713576724499d229e12155d7db1daed096e9fe`，包含保留的未提交工作。

## 当前结论

50 秒动作提示、真实接收时间标记、七类动作录制及用户确认已完成，
见[动作证据](shared_root_actions_20260918.md)。首次交付时“没有动作标注”的阻塞已解除。
用户明确要求不可达时生成可达近似目标，是对原方案不投影规则的授权扩展；
[独立投影配置](shared_root_reachable_projection.md)保留原拒绝配置，整体默认关闭。

逐源帧几何覆盖从 4120/4427 提升为 4425/4427；七类已标注动作的几何帧有效率
均为 100%，但交叉动作仍有 6.783559 ms 接收过期，两帧未标注片段仍被拒绝。
几何有效不等于完整关节位姿可达、QP 准确跟踪、无碰撞或真机可用。

## 本轮修正

200 Hz 离线审计此前将 pose IK 的关节解与随后 settled HOLD 改写后的命令目标
比较，混用了两个阶段。当前 `shared_root_trace_audit.cpp` 改为与
`latestTargets()` 中同周期的实际 IK 输入配对，覆盖位置、姿态及双掌关系指标。
参考跟踪指标仍使用 post-HOLD 命令目标，不与 IK 指标混淆。
新增 `ik_target_pairing` 输出及独立 FK 与 solver 自报残差的 10 um 一致性检查。
新增真实录制前 720 帧回归，确保确实经历超过 100 mm 的 HOLD 目标差，同时
正确配对的残差一致性通过。无本地录制或构建产物时此项明确 skip；本轮未跳过。

重跑同一原始录制、相同初态与 legacy 权重，两种共享根配置的 mode=1 结果：

| 指标 | 原共享根 | 共同平移近似 |
|---|---:|---:|
| 已求解控制周期 | 9257 | 9924 |
| IK 位置残差 p90 | 0.009010 mm | 0.010662 mm |
| IK 位置残差大于 10 mm | 35/18514 侧样本 | 35/19848 侧样本 |
| IK 最大位置残差 | 44.434 mm | 44.434 mm |
| 参考掌位置残差 p90 | 69.6436 mm | 71.8194 mm |

旧“投影最大 IK 残差 172.668 mm”结论撤回；该数是 IK 输入与 HOLD 命令目标的
差异，不是对应 IK 求解失败。修正后仍有真实大残差，不能宣称全部精确跟踪。
35 个大残差侧样本中 31 个接近安全关节限位，只是关联证据，根因仍未确定。
两组纳入样本集合不同，不作为严格成对精度比较；`actual=unavailable`。
原 solver、QP、权重、迭代预算、HOLD、限位与发布权限均未更改。

录制目录 `recordings/shared_root/zhoujie_actions_20260918_041020/` 中新增：

- `reference-baseline-200hz-paired.txt`
- `reference-reachable-200hz-paired.txt`

旧未带 `paired` 的日志保留历史，其 IK 指标已被替代。原包、标注、profile
和 geometry 未改写；指纹详见 paired 日志与投影报告。

## 本轮实际验证

仓库根目录执行：

```bash
.pixi/envs/default/bin/cmake --build control/build --target tianji_shared_root_trace_audit -j 2
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py control/tests/test_shared_root_actions.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest pico2_hands/tests -q
control/build/tianji_shared_root_trace_audit control/config/qp_ik_pico_shared_root.yaml \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr --reference-loop-200hz
control/build/tianji_shared_root_trace_audit control/config/qp_ik_pico_shared_root_reachable.yaml \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr --reference-loop-200hz
git diff --check
git diff --exit-code -- pico2_hands pico2_sim.sh real_robot control/models/marvin_m6_wuji2.xml
```

构建成功；审计与动作回归 65 passed（114.55 s），旧裸手回归 58 passed（4.36 s），
两组完整离线参考闭环退出码均为 0。差异检查通过，指定旧路线相对 HEAD 无差异。
本轮没有重跑完整 CTest；此前 98/98 结果属于投影实现轮，不作为本轮结果。
没有启动设备、真实输入仿真或真机，也没有提交或 push。

## 尚未完成的验收

按方案 §16.1，IK 残差目前用于诊断，不据此擅自更改生产 QP。
启用门限尚未由用户评审冻结，包括允许的共同平移成本、逐动作失效时长和
参考跟踪偏差。30 mm / 0.5 m/s 仍只是隔离实验参数，不是启用批准。
后续现场仿真需要明确设备会话授权；真机验收另行授权，不能由软件或离线通过替代。
当前保持 `motion_authorized=false`、`phase_a_accepted=false`，默认启动路线不变。

## 后续轮：最坏 IK 样本定位

本节是随后一次“继续推进”的实际结果，不替换上轮记录。
审计新增 `worst_ik_sample`：保存与最大 FK 残差同周期同侧的 solver 结果、
安全限位余量、停止原因、两阶段迭代次数/上限、时间预算状态和加权误差。
仅增补离线输出，不修改生产 solver 或配置。

投影配置完整 200 Hz 参考重跑，最坏样本为 cycle=50、sequence=2554、右侧。
周期从零开始，故相对录制首帧为 0.250 s，位于开头未标注片段，不归到七类动作。

| 最坏样本证据 | 结果 |
|---|---:|
| 独立 FK 掌心残差 | 44.434 mm |
| solver 自报掌心残差 | 44.4347 mm |
| 掌姿态残差 | 0.0149085 rad |
| 到最近安全关节边界余量 | -1.80411e-16 rad（浮点舍入量级，边界上） |
| stage1 迭代次数 / 上限 | 3 / 10 |
| stage2 迭代次数 / 上限 | 2 / 10 |
| 时间预算耗尽 | false |
| 最终停止原因 | spark_upper_ik_converged |
| stage2 加权误差 | 0.443365 |

`spark_upper_qpoases_ik.cpp::solveStage` 在加权误差 <=1e-6 **或**本次加权误差
改善量小于 `convergence_delta` 时标记 converged；当前配置后者为 1e-4。
本样本最终加权误差明显大于 1e-6，因此触发的是改善量条件，不是绝对误差达标。
两阶段均未到迭代上限，亦未耗尽时间预算。`accepted/solved` 表示产生可接受的
局部约束求解结果，不保证掌心误差小于 10 mm。

后续已完成[16 初值隔离对照](shared_root_multiseed_20260918.md)：最佳位置残差
44.1831 mm，未实质消除该样本的大残差。以下“尚未进行多初值”保留为本节当轮状态；
最新试验范围及局限以该对照报告为准。

结论：已确认最坏样本在安全限位处以改善量停止；不能由此宣称整个关节空间内
无解，也不能断言只有关节限位一个原因。尚未进行多初值或目标分量反事实试验，
保留 `classification=CauseUndetermined`。提高最大迭代次数本身不会移除此样本
已经触发的提前停止条件。本轮没有以减小限位裕量或修改收敛条件来换取误差降低。

新证据保存在原录制目录 `reference-reachable-200hz-worst-sample.txt`，保留旧日志。
几何覆盖与上一轮一致，IK p90 为 0.010662 mm；参考跟踪 p90 仍为 71.8194 mm。
本次只重跑投影 profile（内含 mode=0/1），没有重跑原拒绝 profile，不将其历史
44.434 mm 数值列作本轮新结果。

本轮实际执行上述 audit 构建命令、投影 profile 完整参考重跑及
`test_shared_root_trace_cli.py` / `test_shared_root_actions.py`：65 passed（111.17 s），
无跳过。回归核对最坏样本与汇总的 sequence/side/error 一致、独立 FK 与 solver
残差一致、迭代次数不超过预算。`git diff --check` 与指定旧路线零差异检查通过。
没有重跑完整 CTest，没有启动设备、改权重或提交/push。

最终启用仍需按方案 §16.1 冻结逐动作覆盖率及最长连续失效门限，并评审近似
目标的修正成本；软件回归或设备试验均不能绕过这一门。门限评审与现场会话
授权是两项独立条件，本次“继续推进”不视为任何门限的批准。
