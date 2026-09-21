# Shared-root Phase A：阶段 6 统一交付审查报告

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

> 历史报告：本文件主体对应早期 R2，保留用于追溯，不再是当前统一验收结论。
> 当前 R3 交付索引见 [R3 阶段交付](shared_root_r3_handoff.md)。
> R3 使用完整闭合 artifact v2 和 0.1615 m TCP；下文旧几何哈希、旧回放数字
> 及“12 mm 仍是阻塞门”的判断均不适用于当前方案。按现行第 16.1 节，
> 这些 IK 精度指标为诊断，启用仍受代表性动作覆盖率等硬门约束。

最新 TCP 已改为独立 shared-root 模型的 0.1615 m，见
[新 TCP 映射层复核](shared_root_tcp1615_mapping.md)。下文 0.1315 m 几何哈希
及数值保留为历史记录，不能当作新 TCP 的验证结果。

后续更正：下文旧 200 Hz 参考回放未反馈 headroom，不能作为完整 Viewer 一致性
结论。已修复离线工具，最新结果见 [权重实验报告](shared_root_palm_priority_trial.md)。
这个缺口不在现场 Viewer 中，不能当作现场故障根因。

日期：2026-09-18。实现和离线验证汇总，**不代表现场/真机验收通过**。
基线：`b0713576724499d229e12155d7db1daed096e9fe`。新功能默认关闭，未提交/push。
此前各报告保留为历史证据，本报告统一描述最新范围。

## 实施对应关系

| 阶段 | 当前产物与证据 |
|---|---|
| 1R 契约/模型 | input contract、robot geometry、完整实验 profile；源码与模型哈希校验 |
| 2 输入适配 | 固定偏移扣除一次、固定姿态来源及 basis；无二次世界系 canonicalization |
| 3 尺度/滤波 | PROVISIONAL/CONFIDENT、唯一帧计数、异常不污染、固定容量窗口 |
| 4 目标/intent | palm 与 shape proxy 分离、固定 intent scale、原始/滤波/混合层验证 |
| 5 连续性 | fresh 唯一帧恢复、旧 ack 拒绝、丢失后静止新目标恢复、授权优先 |
| 6 接线 | guidance 与下游参考接受；真实录制逐源帧与 200 Hz 离线回放、尺度消融、回归 |

代码位置：`control/{include/tianji_qp_ik,src}/shared_root_*`、
`spark_guidance.*`、`config.*`、`apps/run_qp_ik_viewer.cpp` 的隔离分支，及
`apps/shared_root_trace_audit.cpp`、`tests/test_shared_root_*`。
文档/配置路径见 [实现说明](../../spark_shared_root_retarget.md)。
tracking/profile 的现有未提交改动来自另行授权的简化标定，不能混称为本轮控制改造。

## 固定输入

- input contract SHA256：`68304d5b76b063996bdf386b28d103af1d998f7f9a801acb5a55960aaa2fccda`
- robot geometry SHA256：`be93c26db544ad40c96275f49478015256d916657457a0ab31b14c736a56ccd8`
- profile SHA256：`f9262d6186315341e1daeb9693b16eebfd14c7d6dd38f997099a0e29be97e9ca`
- trace SHA256：`8d92f338056a40f73602e6d029c807d807dea29cc6e378b15946d85d3354ab62`

用户已确认删除固定双掌间距门限；掌位置 10 mm 只统计，不作为阻塞项。
关节限位、其他输入/执行保护保留；不提高现场迭代次数，不修改旧算法权重。

## 本轮新增验证

1. 七组带标签的**合成几何**：靠近、分开、交叉、前后错位、高低错位、单手静止、
   共同平移；断言原始共同变换关系、中心、首帧 raw/filtered 一致。
   不是给真实录制补造动作标签，不证明接触或碰撞安全。
2. 伸直姿态检查：J2=-90°、其余零，归一化雅可比平移行除以机器人 reach。
   左右 sigma_min=0.0200272，sigma_min/sigma_max=0.00982417（条件数约 102）。
   是数值条件较差样本，**不是严格秩亏样本**；accepted IK 输出有限且满足安全限位。
   初次把该姿态假定为精确奇异的测试失败，测量后纠正了测试命名和证据定义，
   没有修改算法以迁就测试。
3. `--reference-loop-200hz`：合成 5 ms 控制时钟，按录制 receive timeline 消费最新帧。
   原接收时间保持不变；没有新源帧不更新尺度，不把重复控制周期当作恢复证据。

```bash
pixi run cmake --build control/build --target \
  test_shared_root_guidance test_shared_root_target_builder tianji_shared_root_trace_audit -j 2
pixi run control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root.yaml \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr \
  --reference-loop-200hz
pixi run ctest --test-dir control/build --output-on-failure -R '^test_shared_root_'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py control/tests/test_shared_root_trace_audit.py \
  control/tests/test_shared_root_contract.py -q
```

200 Hz shared-root 结果：

- 4388 个录制源帧；3709 个被控制循环消费，679 个被 latest-only 覆盖。
- 9999 个控制周期，全部下游参考接受；输入映射拒绝 0，预算耗尽 0。
- 4 次恢复确认；9828 个周期为 TRACKING。
- 44 个周期没有新 pose IK 样本，原受约束停止输出继续被接受。
- pose IK 掌位置 p90=66.059 mm，最大 263.626 mm；reference 掌位置 p90=161.223 mm。
- B 系双掌向量误差 p90=124.873 mm；这些精度数据不可隐藏或说成全部目标精确跟踪。
- updateSharedRootFrame p99=10.970 us、最大 96.218 us（当时存在并行测试负载）；
  不含下游 IK/QP，不是端到端实时认证。

所有参考逐周期检查 finite/硬限位，actual=unavailable；命令模型跟随参考不是真实反馈。
该回放未复现生产 receiver stream gate，也未注入系统抢占/传输延迟，不能称现场精确复现。

## 验证状态与验收边界

本轮 shared-root 8 组 CTest 通过（18.74 s），包含更新后的目标构造和伸直姿态测试。
trace/CLI/contract 40 项通过（59.84 s），包括两种参考回放的自动检查、四种入口
的坏文件拒绝。git diff --check 通过，PICO2/real_robot 隔离路径相对 HEAD 零差异。
此前全量 96 组 CTest、346 项主工程 Python 以及 ROS 加载器补测结果，见
[尺度/全量回归报告](shared_root_scale_regression_20260918.md)；不冒充本轮重跑。

新功能实现、接线及主要离线测试已有交付证据，但**不将阶段 6 自动标为全部验收通过**：

- 当前闭环双掌关系精度没有达到原方案的 12 mm 初始目标，单掌 10 mm 已由用户豁免，
  不能默认为其他指标也已经达标；本轮不为此自行改权重/迭代。
- 严格秩亏边界、带人工动作标签的真实录制覆盖仍不完整，不能用近奇异或合成样本冒充。
- 设备/动力学/碰撞安全/真实 actual/真机尚未验证；应在另获授权后执行。

保持 `enabled: false`，不进入 Phase B、不启动现场输入、不创建分支或提交/push。
