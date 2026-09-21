# 2026-09-18 真实输入动作覆盖记录

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

录制：`recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr`。
SHA256：`20340067256a3a78205c78a0d5ba6fc01f818d14da5e1e57f116391451df2f2b`。
本次为 PICO＋VR 手柄输入观察采集，不含机器人执行或实际反馈。

使用 zhoujie 简化标定 `cal-91c71441c51b47db9366390c96c96199`，桥接 X 偏移 +0.20 m。
现场启动日志、源码/标定快照、原始包、提示时刻和失败状态均保留在录制目录。
首次尝试 `zhoujie_actions_20260918_040855` 因 M0 输入未就绪而超时，保留为失败，
未与成功录制合并。重启本次 M0 后成功采集；本次创建的输入会话已停止。

## 动作标注来源

用户在被询问是否全部跟上提示、包括 32 秒换手后，回复“都完成了”。
据此以实际提示发出时刻确认七类动作区间；不以计划整秒数替换实际时间戳。
这属于使用者确认，不是工具独立识别动作或精确测量人体动作起止。
`actions-confirmed.json` 独占生成，`action-candidates.json` 和原空标注保留。
正式标注 SHA256：`805bcf15817f5d7a4505260ba02961347b0fb621252c8160a4bf86574ea8e2db`。

## 本轮离线结果

| 动作 | 有效帧 / 总帧 | 几何帧有效率 | 累计失效 / s | 最长连续失效 / s |
|---|---:|---:|---:|---:|
| 自然前伸 | 420 / 529 | 79.40% | 1.23405 | 0.62921 |
| 双手靠近 | 538 / 538 | 100% | 0 | 0 |
| 胸前交叉 | 520 / 520 | 100% | 0.00678 | 0.00678 |
| 一高一低 | 531 / 531 | 100% | 0 | 0 |
| 单手工作 | 525 / 525 | 100% | 0 | 0 |
| 双手共同运动 | 538 / 538 | 100% | 0 | 0 |
| 伸直边界 | 531 / 531 | 100% | 0 | 0 |
| 未标注（含开头/结尾静止时段） | 517 / 715 | 72.31% | 2.18716 | 1.07007 |
| 全部 | 4120 / 4427 | 93.07% | 3.42799 | 1.07007 |

七类动作均有确认区间和评估帧，`missing_actions=[]`；这只补齐本次动作证据，
不代表全面覆盖工作空间。帧有效率与时间连续性为不同指标，交叉段虽有 100%
帧有效率，时间统计仍存在约 6.78 ms 失效，不能隐去。
总体最长失效位于未标注部分，不能归到某个已标注动作；七类动作中自然前伸
最值得继续定位。本轮没有据此判断必须调尺度、滤波或 QP。

输入几何检查：4427 帧、单一 epoch 338，`geometry_consistent=true`，错误计数为空。
这是录制骨长及原点与标定的一致性，不证明镜像物理轴、控制跟踪或碰撞安全。
末帧为 49.997071457 s；50 秒窗口最后约 2.929 ms 不可观测，单独保留。

实际执行命令（仓库根目录）：

```bash
.pixi/envs/default/bin/python control/scripts/record_shared_root_actions.py confirm-prompts \
  --session recordings/shared_root/zhoujie_actions_20260918_041020 --reviewer zhoujie
.pixi/envs/default/bin/python control/scripts/report_shared_root_actions.py \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr \
  --annotations recordings/shared_root/zhoujie_actions_20260918_041020/actions-confirmed.json
.pixi/envs/default/bin/python control/scripts/audit_shared_root_trace.py \
  --calibration-dir profiles/zhoujie/pico-simple/cal-91c71441c51b47db9366390c96c96199 \
  --trace recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr
```

结果保存在录制目录的 `coverage-confirmed.json` 和 `geometry-audit.json`。
本轮标注后未修改运行逻辑，未重跑完整软件回归；没有运行真实输入机器人仿真或真机。
仍为 `thresholds_frozen=false`、`phase_a_accepted=false`、`motion_authorized=false`，
启用配置保持关闭。下一步是定位自然前伸及未标注时段失效、评审门限，
不能因为动作标签齐全就自动启用。
