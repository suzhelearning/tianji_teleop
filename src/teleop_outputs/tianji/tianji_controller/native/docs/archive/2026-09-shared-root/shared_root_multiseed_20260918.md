# 最坏样本多初值对照（离线）

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

## 范围与方法

接续[最坏样本定位](shared_root_phase_a_followup_20260918.md)，检查初值变化是否能
消除投影分支 sequence=2554、右侧、0.250 s 的 44.434 mm 位置残差。
新增审计参数 `--worst-ik-multiseed`，先原样运行 200 Hz 参考闭环，再针对每个
mode 的最坏样本固定同周期 IK 完整目标、模型、TCP、权重、限位及每阶段 10 次预算。
所有重试使用新建的独立 solver，不共享热启动状态，不将结果写回 guidance/controller。

每个 mode 固定 16 次试验：

- trial 0：从原求解结果 q 冷启动；不是原求解输入 seed，也不是完全相同的历史热启动重现。
- trial 1：安全关节区间中点。
- trial 2～15：从中点依次对关节 0～6 加/减该关节安全区间宽度的 25%。

安全边界使用原配置的 0.05 rad 裕量，未放宽。输出各次 FK 位置/姿态残差、
限位余量、停止原因、迭代次数及相对原解的关节距离。
有限试验不是全局搜索；不同初值不保证达到相同局部解或完成相同收敛程度。

## 结果

投影 mode=1 的 16 次结果均 accepted，均未耗尽时间预算，但无一次位置残差低于
10 mm。所有结果的最近安全关节边界余量处于浮点零附近。

| 初值 | 位置残差 | 姿态残差 | 相对原解关节距离（L2） |
|---|---:|---:|---:|
| 原参考闭环结果（对照） | 44.434 mm | 0.0149085 rad | 0 |
| trial 0：原解重新冷启动 | 44.4336 mm | 0.0148883 rad | 0.0000984 rad |
| trial 1：安全中点 | 210.226 mm | 0.0678222 rad | 3.23436 rad |
| trial 9：关节 3 正偏移，最低位置残差 | 44.1831 mm | 0.0110663 rad | 0.0389536 rad |

最优样本仅改善约 0.251 mm，以 `spark_upper_ik_no_improvement` 停止。
14 个中点偏移试验均用满 stage1 的 10 次预算，stage2 未用满预算；不能由此
排除更多迭代或其他初值的影响。原样本自身则是 3/2 次即按改善量停止。
mode=0 对照最坏目标为不同的 sequence=2549，16 次最佳为 31.7396 mm；不能把
不同 mode 的这两个目标当成严格成对对照。

结论：在当前目标、保护和求解参数下，这组有限初值重试没有实质解决大残差，
不支持据此加入在线多初值切换。结果与关节边界受限相符，但不能证明全局无解，
也不能排除软任务冲突或局部求解因素，分类仍为 `CauseUndetermined`。
关节距离仅是差异指标，不是可安全切换的轨迹或碰撞检查。

## 复现与证据

```bash
.pixi/envs/default/bin/cmake --build control/build --target tianji_shared_root_trace_audit -j 2
control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root_reachable.yaml \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr --worst-ik-multiseed
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py control/tests/test_shared_root_actions.py -q
```

完整输出保存在录制目录 `reference-reachable-200hz-multiseed.txt`，旧日志未覆盖。
trace SHA256 为 `20340067256a3a78205c78a0d5ba6fc01f818d14da5e1e57f116391451df2f2b`，
profile SHA256 为 `7925a889075b062d95cd5e79bbb5fed6fc553fbe6097a11bb94c11d6ee550da6`。
剔除新 `ik_multiseed` 行及计时行后，完整输出与上轮 worst-sample 日志逐行一致，
确认隔离诊断未改变参考闭环数值。

本轮构建成功（最终构建无警告），完整离线诊断退出码 0；上述回归 **72 passed**
（133.48 s），没有跳过。覆盖新参数的畸形文件拒绝、16 个试验身份/限位校验、
原参考模式不输出多初值结果，以及既有 HOLD 配对和动作统计。
`git diff --check` 通过，指定旧路线 `pico2_hands`、`pico2_sim.sh`、`real_robot`、
原 `marvin_m6_wuji2.xml` 相对 HEAD 无差异。本轮没有重跑完整 CTest。

本轮未修改生产 solver、参数或默认入口，未启动设备、未提交/push。
没有真实反馈、完整空间可达性证明或真机验收，不冻结门限、不自动启用。
