# SPARK 源模型限位与在线计算接线

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

2026-09-19，参考源提交 `f615b8c2931601957315d1d3ea7f8aad8bb369a6`。

用户要求将当前共享根 SPARK 与 Ceres 的限位统一。已更新
`config/qp_ik_pico_shared_root_reachable.yaml`，不批量改变其他历史 SPARK 配置。

- 两条当前配置使用同一冻结几何 artifact、源 URDF 和 Ceres 专用 MuJoCo XML。
  文件名含 `ceres` 是历史命名，不表示模型只能用于 Ceres。
- 双臂 J2 为 `[-1.76, 2.0944]`，J3 为 `[-3.1, 3.1]`，J4 为 `[-2.5307, 2.5307]` rad；其余关节同 Ceres。安全余量 0.05 rad。
- 模型关节速度限制 4 rad/s；关节加速度上限前三轴 60、后四轴 90 rad/s²；jerk 上限前三轴 3000、后四轴 4500 rad/s³。速度 QP 开启硬 jerk 约束，初始姿态与 Ceres/源配置一致。
- 保留 SPARK 的软权重、前馈/headroom、停止保护和速度 QP；不额外串联 Ruckig。未启用的历史 `dls_posture_ruckig` 块不是这条路线的平滑器，保持不动。统一硬限值不等于两条路线全部控制语义相同。
- artifact 白名单允许两个已支持后端使用源限位版本，未移除模型/TCP/指纹验证。

## 在线执行，不预计算轨迹

Viewer 每个控制周期获取当前输入及模型参考状态，调用 `stepSharedRoot()`，随后调用控制器 `step()`。

- SPARK：新输入触发 SPARK 姿态 IK；参考处理和速度 QP 在控制循环中执行。
- Ceres：控制循环内求 Ceres 近似关节目标，并立即调用 Ruckig 更新下一步参考。

SPARK 对同一输入序号复用已有 IK 结果是实时控制缓存，不是离线生成轨迹。录制若作为输入源，也只是按时发送原始输入；以上计算仍在 Viewer 运行时完成。此前固定周期离线诊断只用于分析，不是现场控制路径。

配置继续默认关闭、仅模型参考模式、禁止关节导出。本文未授权或启动现场输入/真机会话；尚无统一限位后的真实输入仿真效果结论。启用时须使用该配置对应的 artifact 模型，不能显式传入旧共享根 XML。

## 本轮验证

构建成功；完整 Ceres ON CTest 102/102 通过（139.02 秒）。新增配置一致性测试后单独重建并执行 `test_shared_root_options`，11/11 通过。
`git diff --check` 通过。日志 `/tmp/spark_source_limits_tests.log`、`/tmp/spark_source_limits_options_test.log`。
本轮未重建 Ceres OFF，未启动真实输入或真机；软件通过不能替代现场验收。

用户随后明确 home 为左 `[55,-65,-70,-60,60,0,0]°`、右 `[-55,-65,70,-60,-60,0,0]°`。
两条配置的初始姿态已采用此值，Ceres 的 `home_left_rad/home_right_rad` 也相同。
SPARK 以该初始姿态启动，随后使用上一已接受的关节解作为 seed；没有为它新增 Ceres 式 home 吸引项。
