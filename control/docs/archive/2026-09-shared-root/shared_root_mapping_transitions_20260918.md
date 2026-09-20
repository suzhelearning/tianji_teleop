# 映射收尾：两帧拒绝与投影／肘分支连续性

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

本轮只检查映射。没有修改生产 target builder、IK、QP、限位或实验参数，也没有
启动设备。新增工具仅用于离线证据，原有效率和两帧拒绝均保持不变。

## 两帧拒绝已定位

原理：共同平移 delta 必须同时处于 raw/filtered × 左/右四个外可达球、30 mm
总平移球，以及以旧 delta 为中心、半径为 `0.5 m/s × source_dt` 的速度球内。
若任意两个球的中心距离大于半径之和，整组约束必然无共同解。

| 相对接收时间 | 源序号 | 冲突约束 | dt | 允许修正步长 | 满足单侧外可达所需步长下界 | 缺口 |
|---|---:|---|---:|---:|---:|---:|
| 0.641163127 s | 2589 | raw 左侧外可达球 / 速度球 | 10 ms | 5.000 mm | 7.245 mm | 2.245 mm |
| 48.951062519 s | 6863 | raw 右侧外可达球 / 速度球 | 8 ms | 4.000 mm | 4.477 mm | 0.477 mm |

上述下界是满足对应单球约束的必要条件，不冒充所有约束的最优共同平移。
诊断原样重算 64 轮，delta 与 production builder 的结果误差不超过 1e-12 m；
延长到 4096 轮仍不满足约束。仅在隔离诊断中去掉速度球后，4096 轮找到满足其余
五球的结果，最大残差不超过 1.12e-16 m。没有把此结果提交给映射器或控制器。

因此这两帧不是 64 轮计算不足，也不是 IK 失败，而是当前“保持双掌关系的共同平移”
与附加平移速度上限发生冲突。继续保留拒绝，不能通过多算几轮消除，也不放宽速度
保护来追求 100% 总有效率。两帧均在未标注片段；七类动作的原统计不变。

## 投影进入与退出

新增 `--mapping-transitions` 在原 standalone mapping replay 中旁路观察，不改历史。
4425 个有效输出包括 4 次历史起始和 4421 对连续有效输出；3 个重建间隔单列，
不把 beginRecovery 清空速度历史之后的输出冒充连续过渡。

- 连续历史下 3 次进入、3 次退出；重建起始不计入 enter/exit。
- 全部连续有效输出的附加平移速度不超过 0.5 m/s。
- 有修正或上一帧仍有修正时，`intent_evidence_valid` 必须为 false；本次全部通过。
- 六次进入/退出事件中最大修正步长约 1.370 mm（6.545639987 s 的退出）。

0.5 m/s 限制的是附加 delta，不是整个掌目标的速度。固定骨长、固定肩点、末端闭合
与双掌相对向量代数检查仍通过。保持本录制的 4425/4427 有效帧、累计失效
26.351160 ms、最长失效 10.430654 ms，未把输入过期时段从统计中删除。

## 肘分支连续性

比较两个已定义肘点时，先计算肩—腕轴和两球交圆的径向，再用最小旋转将旧交圆
平面传输到新平面。直接把旧肘点投影到新交圆会在旧帧完全伸直时制造一个假方向，
因此不采用该口径。

诊断将交圆半径 <1 um 或肩腕轴退化的情况单列，此阈值不修改生产闭合规则。
还保留每层每侧最后一个已定义径向，检查经过退化区后重新出现的方向；不跨越
builder 重建或输入身份重置延续这份诊断历史。

| 连续历史层 / 侧 | 相邻对数 | 退化、未做相邻方向比较的对数 | 最大掌步长 | 最大肘步长 | 已定义径向反向事件 |
|---|---:|---:|---:|---:|---:|
| raw 左 | 4421 | 104 | 49.675 mm | 54.113 mm | 0 |
| raw 右 | 4421 | 91 | 67.107 mm | 61.065 mm | 0 |
| filtered 左 | 4421 | 40 | 33.623 mm | 33.040 mm | 0 |
| filtered 右 | 4421 | 109 | 37.497 mm | 41.423 mm | 0 |

可比较的相邻径向最小 cosine 为 0.974107，未发现大于 90° 的方向反转。
跨退化区两端共比较 33 段，最小 cosine 为 0.716407，反向事件 0、轴反向未决 0。
这不证明全工作空间无翻支，也不把“未反向”解释为所有目标步长足够小或运动安全。
三个重建间隔的 filtered 最大掌步长约 42.254 mm、最大肘步长约 58.328 mm，
明确保留；这些不是未经恢复整形直接执行的轨迹。

以上是逐源帧 standalone mapping，没有伪造执行器接受历史。连续性状态机和
双侧提交语义由本轮相应 native 回归检查，不以本报告代替真实输入仿真或设备验收。

## 实现、证据与验证

新增 `control/apps/shared_root_mapping_probe.hpp`，仅被离线审计及测试使用。
生产映射文件和两份 profile 未改变，原 profile 保留拒绝策略，实验 profile 保持默认关闭。
完整输出在录制目录 `recordings/shared_root/zhoujie_actions_20260918_041020/mapping-transitions.txt`。
trace SHA256：`20340067256a3a78205c78a0d5ba6fc01f818d14da5e1e57f116391451df2f2b`。
profile SHA256：`7925a889075b062d95cd5e79bbb5fed6fc553fbe6097a11bb94c11d6ee550da6`。

```bash
.pixi/envs/default/bin/cmake --build control/build \
  --target tianji_shared_root_trace_audit test_shared_root_coverage -j 2
control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root_reachable.yaml \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr --mapping-transitions
.pixi/envs/default/bin/ctest --test-dir control/build --output-on-failure \
  -R '^test_shared_root_(coverage|closure|target_builder|continuity|pipeline|options|input|morphology|geometry)$'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py -k 'mapping or native_trace_reader or duplicate_trace' -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest control/tests/test_shared_root_actions.py -q
git diff --check
git diff --exit-code -- pico2_hands pico2_sim.sh real_robot control/models/marvin_m6_wuji2.xml
```

最终构建无警告。Native CTest **9/9**（3.71 s）；CLI 映射/坏包回归 **57 passed**、
4 deselected（1.23 s）；动作统计 **24 passed**（0.47 s），没有 skip。
新增合成检查可以识别人工构造的“弯曲—伸直—反向弯曲”，也验证重建不伪造分支连续性；
真实录制回归核对两帧速度约束冲突、修正速度、意图抑制及跨退化区计数。
四个含 IK/参考闭环的 CLI 测试本轮主动不选，未重跑完整 CTest；没有进行 IK/QP 优化。
差异检查通过，未启动设备、未提交/push。

当前结论：本录制未发现需要修改映射算法的新缺陷，两帧是已解释的保护拒绝。
映射软件可按这一边界阶段性收尾；门限评审和现场映射体验/物理方向确认仍独立待验，
不得据此自动启用。IK 与下游跟踪按用户要求后置。

## 后续交付检查：纯映射回放待人工评审

随后一轮只做入口检查和文档收尾，未新增算法、未打开窗口或运行上述历史回归。
执行：

```bash
.pixi/envs/default/bin/python control/scripts/view_shared_root_mapping.py \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr \
  --profile control/config/qp_ik_pico_shared_root_reachable.yaml --check
```

结果：4427 帧，退出码 0；filtered mapping only，无 guidance blend、IK、physics 或 socket。
如需人工查看同一录制，使用上述命令去掉 `--check`；必须保留实验 profile 参数，
因为脚本默认仍是原拒绝配置。背景机器人固定不动，橙色为左侧映射骨架、绿色为右侧，
白球及 RGB 轴为掌目标。P 暂停、R 重播、Q 退出；拒绝帧目标隐藏，不伪装成有效帧。

待使用者查看并反馈，尚未勾选通过：

- 左右、前后、上下和掌朝向是否符合这次录制动作；机械背景姿态不用于判断 IK 效果。
- 双手靠近、交叉、一高一低时的相对关系和尺寸比例是否符合预期。
- 接近伸直和回到静止时，是否看到不符合预期的肘形变化或目标跳动。

回放观感不能独立证明现场物理轴、整个工作空间、碰撞安全或最终跟踪效果。
逐动作最小几何有效率、最长连续失效允许值，以及近似修正成本仍待评审；
不从单段录制倒推“恰好通过”的阈值，不擅自填入 95%/99%。
按方案 §19/20 在此等待用户验收方向，不自动进入设备试验、Phase B 或恢复 IK 优化。
本轮 `git diff --check` 通过，没有提交/push。
