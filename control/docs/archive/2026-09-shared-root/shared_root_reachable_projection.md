# 共享根：有界共同平移近似目标

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

后续已完成[两帧拒绝与连续性复核](shared_root_mapping_transitions_20260918.md)：
两帧均为外可达约束与修正速度约束不相容，非迭代预算不足；保护拒绝保持不变。

按用户“不可达时生成可达近似目标，再交给 QP 跟踪”的要求，新增独立实验选项。
原配置不增加该项、行为保持拒绝；新增配置
`control/config/qp_ik_pico_shared_root_reachable.yaml` 打开投影选项，但仍保持
`spark_shared_root.enabled:false`。没有启用现场会话、机器人或更改默认启动入口。

## 实现与保护

`SharedRootTargetBuilder` 在已有输入、形态、中心和形状门控之后、精确骨架闭合之前，
求一个共同三维平移 delta。同时约束原始/滤波目标的左右四个目标腕点位于各自
两连杆外可达球内，并约束 delta 的总幅度和相对上个已提交候选的变化速度。
两侧掌心共同平移，双掌相对位置向量、掌姿态和相对姿态不变。骨长、肩点不变。

使用固定容量六球 Dykstra 投影，固定 64 轮，无热路径 I/O 或无界分配/重试。
它是有限预算近似，不宣称求得全局最优的完整关节可达目标。
预算结束仍不满足约束则以 `ReachableProjectionInfeasible` 拒绝。
两连杆内半径、肘分支、闭合残差及原有中心/形状门仍检查；不能用外球投影绕过。
该实现不处理所有内半径不可达或双手共同平移约束冲突的情形。

实验参数：

```yaml
spark_shared_root:
  # 其余完整契约配置见独立实验 profile
  reachable_projection:
    enabled: true
    maximum_translation_m: 0.03
    maximum_speed_m_s: 0.5
```

30 mm / 0.5 m/s 是本次离线实验参数，不是已冻结启用门限。
速度约束作用于附加平移，采用唯一源帧 dt；并非整个掌目标的速度限制。
首次输入或恢复重建没有旧平移速度参考，仍由原 continuity 恢复混合和下游关节
约束限制控制输出。任何不成功的双侧候选均不提交滤波或投影历史。
有平移修正及其退出过渡时，禁用未修正输入的意图前馈证据，避免把原目标速度
直接用到修正目标；其余 tracking/hold/recovery/ACK、身份、新鲜度和关节保护不变。

原始数据与语义：`raw_preference` / `filtered_preference` 保存投影前目标；
`reachable_translation` 保存修正。启用时 `raw` / `filtered` 是修正并闭合后的目标。
原始 affine 中心不再强制与输出中心相等；审计分别记录平移，检查扣除该平移后
的代数残差。双掌关系代数不变。动作统计报告新增 `target_semantics` 与
`reachable_projection` 字段，以免把近似后有效率冒充原始映射有效率。
`closure_side` / `mapped_palm_outside_workspace_ratio` 仍统计投影前 preference；
其中 filtered preference 随接受/恢复历史变化，不是两种配置完全相同的集合。

闭合目标经既有 SharedRootPipeline → continuity → DualArmSparkGuidance 的
SPARK 位姿 IK → velocity-QP 路径消费，不新增运动发布入口。
两连杆几何可达只是必要条件，不保证关节限位下的位姿解、碰撞安全或实际跟踪。

## 当前录制对照

录制：`zhoujie_actions_20260918_041020/input.tjvr`，4427 唯一帧，使用用户确认标注。
原录制 SHA256：`20340067256a3a78205c78a0d5ba6fc01f818d14da5e1e57f116391451df2f2b`。
实验 profile SHA256：`7925a889075b062d95cd5e79bbb5fed6fc553fbe6097a11bb94c11d6ee550da6`。
原 profile / geometry / trace 未改写；未修改任何标定或录制原包。

| 逐源帧几何指标 | 原映射 | 共同平移近似 |
|---|---:|---:|
| 有效帧 | 4120 / 4427 | 4425 / 4427 |
| 有效率 | 93.0653% | 99.9548% |
| 前伸有效帧 | 420 / 529 | 529 / 529 |
| 累计失效 | 3.427991 s | 0.026351 s |
| 最长连续失效 | 1.070074 s | 0.010431 s |
| 发生修正的已接受帧 | 0 | 310 |
| 最大共同平移 | 0 | 22.907736 mm |

七类已标注动作均为 100% 几何帧有效率；交叉段原有 6.783559 ms 接收过期保留。
两帧投影拒绝在录制相对时间 0.641163127 s 和 48.951062519 s，均未标注。
未放宽约束将它们强行纳入。共同平移导致的双掌位置关系误差为零（浮点代数
残差上限约 2.4e-16 m）；掌姿态不变。原始中心变化量必须作为修正成本保留。

## 200 Hz 离线 IK/QP 参考闭环

同一录制、相同模型初态、相同 legacy 权重和既有 solver，两种 profile 分别执行
`--reference-loop-200hz`。这里只比较各自输出中的 mode=1；mode=0 是旧路线对照。
每组 10001 控制周期，4093 投递源帧、334 latest-only superseded 源帧。

| 指标 | 原共享根 | 共同平移近似 |
|---|---:|---:|
| 已求解/纳入 IK 指标的周期 | 9257 | 9924 |
| 停止/未求解而排除的周期 | 744 | 77 |
| TRACKING 周期 | 9035 | 9668 |
| 输入候选拒绝 | 289 | 2 |
| IK 掌心位置残差 p90 | 0.009010 mm | 0.010662 mm |
| IK 位置残差 >10 mm 的侧样本 | 35 / 18514 | 35 / 19848 |
| IK 最大位置残差 | 44.434 mm | 44.434 mm |
| 参考掌位置残差 p90 | 69.6436 mm | 71.8194 mm |
| 参考命令校验接受 / 拒绝 | 10001 / 0 | 10001 / 0 |

两组已纳入的样本集合不同，不能把上述误差当作严格成对精度比较。
近似分支纳入了此前被几何门排除的更多边界目标，仍有较大 IK 残差；
不能因为参考命令校验接受或 p90 较小，就宣称所有目标准确跟踪。
`actual=unavailable`；参考值不是真实反馈。残差原因仍为 CauseUndetermined，
本轮没有改权重、迭代次数、收敛门限或关节保护来掩盖结果。

2026-09-18 审计修正：旧统计把 IK 解与后续 settled HOLD 改写后的命令目标比较，
因此旧版投影最大值 172.668 mm 及两组超 10 mm 样本数不能作为 IK 残差证据。
现在 IK 位置、姿态及双掌关系统一与同周期 solver input 配对；参考跟踪指标仍与
post-HOLD command 比较。两组独立 FK 残差与 solver 自报残差最大差均约 1.128 um，
低于跨模型核验容差 10 um。修正后两组最大值均为 44.434 mm（右侧 sequence 2554）；
35 个超 10 mm 侧样本中 31 个距安全关节限位不足 0.01 rad，仅说明相关性，未确认根因。
修复仅涉及审计口径，没有改变生产 IK、QP、HOLD 或权重。

修正后重跑：投影分支映射 p99 11.951 us、最大 19.295 us；基线 p99 10.881 us、最大 23.095 us。
本轮有并行软件验证负载，这些仅是离线观测，不是实时最坏时间保证。
两组 legacy mode=0 数值输出一致（时间测量不纳入数值契约）。

证据位于录制目录：`coverage-reachable-experiment.json`、
`reference-baseline-200hz-paired.txt`、`reference-reachable-200hz-paired.txt`。
旧 `reference-baseline-200hz.txt`、`reference-reachable-200hz.txt` 保留历史，
其 IK 残差指标已被以上 paired 结果替代，不能继续据此推断投影导致 173 mm IK 误差。

## 复现与验收边界

仓库根目录执行：

```bash
.pixi/envs/default/bin/cmake --build control/build -j 2
.pixi/envs/default/bin/ctest --test-dir control/build --output-on-failure -j 1
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_actions.py control/tests/test_shared_root_contract.py \
  control/tests/test_shared_root_palm_priority_profile.py control/tests/test_shared_root_trace_cli.py -q
control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root_reachable.yaml \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr --reference-loop-200hz
.pixi/envs/default/bin/python control/scripts/report_shared_root_actions.py \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr \
  --profile control/config/qp_ik_pico_shared_root_reachable.yaml \
  --annotations recordings/shared_root/zhoujie_actions_20260918_041020/actions-confirmed.json
```

新增回归覆盖共同关系/姿态和固定骨长、修正幅度/速度、不相容双臂目标拒绝、
失败不提交历史、已可达目标保持一致、旧配置省略新选项和参数校验。
投影实现轮的历史 Python 回归为 76 passed（94.74 s）。当轮完整原生 CTest 为 98/98 passed（135.76 s），
包括旧路线、协议、状态机及本机合成输入无窗口集成。`git diff --check` 通过。
未运行新分支真实输入仿真或真机验收，仍默认关闭、门限未冻结，未提交/push。
最新审计修复轮的命令和验证结果见[交付复核](shared_root_phase_a_followup_20260918.md)，
上述历史完整回归不冒充修复后的本轮结果。
