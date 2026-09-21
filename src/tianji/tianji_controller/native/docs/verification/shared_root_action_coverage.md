# Shared-root：按动作离线统计入口

本工具补充方案第 16.1 节的分动作证据。不修改控制热路径、QP、协议、录制或驱动。
它只调用原生 `--mapping-frames`，没有 Viewer、设备连接或运动发布功能。

## 不带标注先检查现有录制

在仓库根目录运行：

```bash
.pixi/envs/default/bin/python control/scripts/report_shared_root_actions.py \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr
```

输出 JSON 到标准输出，不覆盖或生成录制文件。默认实验配置仍为
`control/config/qp_ik_pico_shared_root.yaml`，可用 `--profile` 显式选择离线配置。
报告记录 trace/profile/geometry 指纹、全部及分动作统计、未标注部分和缺失动作。
不带标注时所有帧为 unannotated，各动作均未覆盖，不推测用户当时的动作。

## 提供真实动作区间

新的带动作提示采集入口见[50 秒动作采集](shared_root_action_capture.md)。
其候选区间只证明提示发出时刻，采集后仍需佩戴者/观察者确认，不能直接当成动作证据。

确认录制中的真实动作时间后，在用户自选 JSON 文件中填写：

```json
{
  "schema_version": 1,
  "trace_sha256": "替换为报告中的实际录制哈希",
  "time_basis": "nanoseconds_since_first_receive",
  "segments": []
}
```

每个 segment 包含 `action`、整数 `start_ns`、整数 `end_ns`；时间为距离
**第一条接收时间**的纳秒数，不是 PICO 源时间或壁钟。上述空数组是未标注状态，
不是验收样例；不要凭本文给现有录制编造区间。

允许的动作标签：

| 标签 | 动作 |
|---|---|
| natural_reach | 双手自然前伸 |
| hands_approach | 双手靠近 |
| crossing | 双手交叉 |
| unequal_height | 一高一低 |
| single_hand | 单手工作 |
| bilateral_motion | 双手共同运动 |
| extension_boundary | 伸直边界 |

```bash
.pixi/envs/default/bin/python control/scripts/report_shared_root_actions.py \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr \
  --annotations /实际路径/actions.json
```

区间必须排序、不重叠且在录制范围内。使用 `[start_ns,end_ns)`，唯独录制最后
一帧归入以 EOF 为终点的区间。间隙保留为 unannotated；连续相邻同类区间合并
计算最长失效，不能靠拆分标注缩短结果。不同时间段之间不拼接失效时长。
区间内无源帧时，覆盖率为 null，该动作仍列为 missing，即使标注跨过一段断流。

统计会裁剪跨越动作边界的无效时间；超过 freshness 的无帧区间计入失效。
EOF 后不可观测时间不延长。条件分母与原生工具相同：完整、有序、唯一源帧且
morphology 可评估的录制；不支持的坏输入会报错，不通过静默跳过来抬高覆盖率。
动作标注来自使用者，不代表工具独立确认了动作内容。

## 验收边界和本轮验证

任何输出始终为 `thresholds_frozen:false`、`phase_a_accepted:false`、
`motion_authorized:false`。本工具不提供调低门限、写配置启用或运动授权操作。
只有分动作数据经评审后才能冻结门限。下文保留本工具首次交付时的历史状态；
后续七类动作录制及用户确认标注已完成，见[动作证据](../archive/2026-09-shared-root/shared_root_actions_20260918.md)。
用户授权的近似目标分支已完成[拒绝与连续性复核](../archive/2026-09-shared-root/shared_root_mapping_transitions_20260918.md)，
当前仍待人工评审和启用门限冻结，不再缺本次七类动作标注。

首次交付所用的无标签 zhoujie 录制结果为 4388 源帧、4376 有效帧，最长连续映射
无效约 62.382 ms；未标注 4388 帧、缺少全部七类动作的独立验收证据。
原生结果与 Python 区间时间计算的交叉校验已加入集成测试。

测试命令：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_actions.py \
  control/tests/test_shared_root_contract.py \
  control/tests/test_shared_root_palm_priority_profile.py -q
```

本轮结果：36 passed（0.86 s），其中新工具测试 24 项；包含真实录制交叉校验。
`git diff --check` 通过；PICO2 目录及入口、real_robot、原模型相对 HEAD 零差异。
本轮没有重新运行全部 C++ 和设备测试，不引用旧轮结果作为本轮通过证据。

本轮未改变现场默认行为；未启动设备、Viewer、真机或提交/push。
