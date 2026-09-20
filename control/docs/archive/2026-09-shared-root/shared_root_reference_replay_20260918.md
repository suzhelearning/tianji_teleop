# Phase A 阶段 6：真实输入的离线控制参考接线验证

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

后续审计更正：此报告的历史参考回放漏接 `updateHeadroomFeedback`，不等同于
完整 Viewer 参考链路。工具已补齐，最新对照见
[权重实验报告](shared_root_palm_priority_trial.md)。此处结果保留为历史记录。

## 范围

使用 zhoujie 50 秒 TJVR 录制，增加独立 `--reference-loop` 诊断入口，保留原
pose-level IK-only 对照。只调用现有 guidance、velocity-QP、FK 和控制参考更新，
不启动设备、socket、Viewer、mj_step 或真实执行器。不修改生产控制参数。

该模式按源帧接收间隔推进（dt 夹在 1～100 ms），不是 200 Hz 调度复现。
模型跟随控制器参考，是命令模型的离线运动学更新，不是物理反馈。
控制器中名称为 actual 的内部读数在本模式下来自该模型，不能用作真实跟踪验收；
报告 actual=unavailable。不模拟接收器 stream gate、动力学、通信抖动或 Home 操作。

## 可重复运行

```bash
pixi run cmake --build control/build --target tianji_shared_root_trace_audit -j 2
pixi run control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root.yaml \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr \
  --reference-loop
```

trace SHA256：`8d92f338056a40f73602e6d029c807d807dea29cc6e378b15946d85d3354ab62`。
profile SHA256：`f9262d6186315341e1daeb9693b16eebfd14c7d6dd38f997099a0e29be97e9ca`。
使用原权重、原迭代上限与收敛阈值；未采用之前离线精度消融组 3～5 的参数。
双方从同一关节限位中点模型初始化，逐帧使用各自控制器接受后的参考。

## 实际结果

| 指标 | legacy | shared-root |
|---|---:|---:|
| 输入源帧 | 4388 | 4388 |
| 输入映射拒绝 | 0 | 0 |
| 可统计双侧 pose IK 帧 | 4388 | 4372 |
| 未纳入 pose IK 统计 | 0 | 16 |
| 下游控制参考接受 | 4388 | 4388 |
| 下游控制参考拒绝 | 0 | 0 |
| shared-root 恢复确认 | 不适用 | 4 |
| shared-root TRACKING 帧 | 不适用 | 4313 |
| pose IK 掌位置 p90 | 4.766 mm | 65.695 mm |
| reference 掌位置 p90 | 161.135 mm | 156.483 mm |

shared-root 的 16 帧原因均为 `spark_feedforward_stopping/not_solved/not_solved`，
这些帧没有新的 pose IK 样本，但其受约束停止输出仍被控制器接受。
参考和 IK 误差不是同一层指标；两种映射的目标也不同，不能仅凭此表评价绝对优劣。
10 mm 仅为诊断统计，不作为本轮阻塞项。

逐周期检查参考 q/qdot/qddot 有限、q 不越模型硬关节限位；未发生检查失败。
姿态级 IK 预算耗尽为 0，不代表现场实时达标。
新输入映射链 `updateSharedRootFrame` 共 4388 次，p99 4.847 us、最大 10.862 us；
这是本机单次测量，不含 IK、下游 QP、调度与模型计算，不是端到端实时认证。

## 诊断工具自身的修正

首轮实现把“无新 pose IK 样本”误用为跳过下游控制器的条件，导致停止期间的速度
历史被冻结，出现反复 reference reset failed。已修正为：独立统计 pose IK，
只要 guidance 输出 accepted，就交给原控制器；恢复确认仍必须有同周期双侧接受。
修正后本录制无 reference reset failed。没有绕过保护或修改生产 guidance。

## 当前结论和剩余项

本轮验证：16 组 CTest 通过（18.49 s）；契约、PICO2 与 trace/CLI 共 84 项
Python 测试通过（5.12 s）。随后新增真实录制参考恢复回归，重跑 trace/CLI 共
19 项通过（19.15 s），断言完整参考接受、进入 TRACKING、恢复确认和无 reset_failed；
该录制回归在缺少本地录制时明确 skip，不冒充合成测试。
`git diff --check` 通过，`pico2_hands/`、`pico2_sim.sh`、`real_robot/` 相对 HEAD
零差异。实际基线 SHA：`b0713576724499d229e12155d7db1daed096e9fe`。

本录制证明：删除双掌间距门限后，原权重下可以贯通真实输入、shared-root guidance、
下游 QP、控制参考接受与恢复确认。不是仅“位置 IK 返回成功”。

阶段 6 尚不能整体宣布完成：还需统一尺度/各向异性尺度的跟踪与饱和离线消融、
有明确动作标签的限位/奇异与中断集合，以及现有线程/清理的大范围回归汇总。
200 Hz 调度精确回放未完成；设备仿真、actual、dynamics 和真机不在本轮授权范围。
默认仍关闭，保留现有路线；不进入 Phase B，不提交或 push。

后续进展：同一真实录制的 paired filtered-target 尺度/IK 消融和全量回归已补齐，
见 [尺度与回归补充](shared_root_scale_regression_20260918.md)。这不补造动作标签，
也不改变上述实际反馈/动力学/调度复现的范围限制。
