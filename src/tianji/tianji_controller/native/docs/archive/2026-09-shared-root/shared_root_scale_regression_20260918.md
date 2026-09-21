# 阶段 6：尺度消融与回归补充

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

本轮不修改现场权重/迭代、不启动真实设备、不提交或 push。
原录制与 profile 哈希见参考接线回放报告，输入文件保持不变。

```bash
pixi run cmake --build control/build --target tianji_shared_root_trace_audit -j 2
pixi run control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root.yaml \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr \
  --scale-ablation
```

## 相同有效帧集合的尺度对照

| 指标 | diag(reach,lateral,reach) | reach × I |
|---|---:|---:|
| 共同有效帧 | 4388 | 4388 |
| 排除帧 / 求解拒绝 / 预算耗尽 | 0 / 0 / 0 | 0 / 0 / 0 |
| 掌位置 p90 | 65.589 mm | 67.955 mm |
| 掌姿态 p90 | 0.009636 rad | 0.011055 rad |
| B 系双掌向量误差 p90 | 122.659 mm | 118.637 mm |
| 骨段方向残差组合范数 p90 | 0.040315 | 0.046748 |
| 距安全关节边界 <0.01 rad 的侧样本 | 1106 / 8776 | 1102 / 8776 |

两侧每个 accepted 解均检查 finite 和含 margin 的关节限位。
近限位计数不是 QP 活跃约束证明；骨段方向残差不是肘角误差。
同一滤波模块、同一权重、同一初始 seed，只改变共同 lateral scale。
本测试直接比较 filtered targets，不含恢复 blend、velocity-QP 或 actual。
两种尺度使用相同 morphology 样本流；本录制无无效候选，不能据此验证失败时
生产 pipeline 的尺度回滚语义，那部分由独立 pipeline 测试覆盖。

统一尺度的双掌向量指标略好，掌位姿和臂形指标略差，没有明显整体优势。
暂时保留现有各向异性实验选择，不引入新运行时选项，也不为毫米级精度调参。
这不是对任意人体动作的可达性保证，精度只作统计。

## 验证边界

新增双臂各关节上下边界附近的 14 组 FK 目标压力用例，检查 accepted 输出的
有限性和安全关节范围；不要求这些映射后目标无残差，也不把它叫作奇异性证明。
原有已知 FK、冲突消融、预算、无新动作恢复、重复帧、epoch 与授权撤销测试保留。

方案中未完成的证据仍明确保留：真实动作没有人工阶段标签，不能声称每种指定动作
都有现场覆盖；200 Hz 调度精确复现、actual、dynamics、真机未验证。

## 本轮命令与结果

```bash
pixi run ctest --test-dir control/build --output-on-failure
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  pico2_hands/tests tests sim/tests real_robot/tests -q
pixi run cmake --build control/build --target test_shared_root_guidance -j 2
pixi run control/build/test_shared_root_guidance \
  --gtest_filter=SharedRootGuidance.JointBoundaryTargetsStayFiniteAndInsideSafeLimits
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py control/tests/test_shared_root_trace_audit.py \
  control/tests/test_shared_root_contract.py -q
```

- 96/96 CTest 通过（121.47 s），包含原生线程交换、录制、旧 Viewer headless
  合成输入集成测试、暂停/恢复及默认配置；没有连接真实 PICO 或机器人。
- Python 主工程 346 项通过、1 项跳过（14.33 s）。
- 用 `-rs` 复跑为 346 通过、1 跳过（14.09 s），跳过原因是默认 Python 无兼容
  rclpy ABI。切换 tracking Python 但未 source ROS 时仍跳过；随后执行
  `source .pixi/envs/tracking/setup.bash`，用该环境运行
  `tests/test_pico_simple_ros_loaders.py`，2 项通过（0.26 s），只测试加载器、不启动节点。
- 新增关节边界测试单独构建并运行通过（5.457 s）；它在上述全量 CTest 的该项
  已执行后加入，因此单独列明，不把旧全量结果说成包含新增用例。
- trace/CLI/contract 33 项通过（19.51 s），包括三个离线 CLI 分支的坏文件拒绝
  以及真实录制参考恢复回归。尺度消融另以完整录制直接运行成功。
- 所有硬件驱动、协议及现场参数未因本轮更改，未提交/push。
