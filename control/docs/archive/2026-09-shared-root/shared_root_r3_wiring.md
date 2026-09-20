# R3 唯一掌心与闭合控制接线

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

日期：2026-09-18。仅离线软件验证；默认关闭，无设备、真机、发布或提交授权。
本报告接续 [几何证据](shared_root_r3_geometry.md)，取代其“尚未接线”的进度说明。

## 实现范围

- 原生配置读取器将 artifact v2 的左右闭合几何注入 TargetBuilder 与 continuity。
  纯代数测试允许不注入几何；正式配置没有跳过闭合的选项。
- 保留人体臂形作为 raw/filtered preference，不将其末点作为另一掌目标。
  raw/filtered/control 均从唯一 palm pose 闭合；`hand == palm.position`。
- 恢复插值只提供掌位姿与肘偏好，每周期重新闭合，不线性插值骨段后拉伸。
- 不可达或分支退化整对拒绝，不改变 affine 掌目标；双侧接受前不写肘历史。
  分支跳变上限复用现有 `pico_teleop.max_position_jump_m`，不另加现场参数。
- raw/filtered 读取历史但不提交；HOLD 冻结；恢复采用同一模型快照的 FK 肘点。
  同周期源帧已消费、双侧 IK/前馈接受及外部控制参考接受后，才原子提交历史。
  `accept()` 返回值仍表示“恢复转入 tracking”，不表示每次历史提交。
- shared-root 恢复不再依赖未使用的 Ruckig 姿态参考器包络；该模式只允许
  headroom/feedforward velocity-QP，实际外部控制器的限位与状态验证不变。
- settled-hold 的每侧诊断显示完整模型 FK 与同一 TCP，不混用旧 shape 末点。
  此诊断与仍保留的闭合 mapped candidate 是不同层，不得用它统计映射误差。

## 数据与初步几何结果

沿用 zhoujie 的 4388 帧录制，SHA256：
`8d92f338056a40f73602e6d029c807d807dea29cc6e378b15946d85d3354ab62`。
模型、几何 artifact 及实验配置指纹见几何报告与审计输出。

- 仅映射：4376 有效 / 12 几何不可达，约 99.73% 有效。
- raw 双掌关系最大残差 `2.42843e-16 m`。
- raw/filtered 固定骨长最大残差 `3.88578e-16 m`，末点/掌心间距为零。
- 当前任一 raw 或 filtered 失败都会拒绝整个候选，所以不能将此前“独立
  filtered 有 4383 帧有效”的诊断直接当作当前接受帧数。
- 这些是本段录制的几何覆盖率，不是按动作冻结的启用门槛；不能宣布 Phase A
  最终验收或现场可用。闭合几何也不保证关节空间可达、无碰撞或实际跟踪精度。

## 验证命令与结果

最终构建成功；C++ 相关回归 **14/14 组通过**（23.90 s），包含 shared-root、
旧 SPARK guidance/retarget、配置、安全和 controller；Python **51 项通过**
（87.83 s，无跳过），包含逐源帧及 200 Hz 的真实录制离线参考回放。命令：

```bash
cmake --build control/build --parallel 4
ctest --test-dir control/build \
  -R 'shared_root|test_spark_guidance|test_config|test_spark_upper_retarget|test_safety|test_controller$' \
  --output-on-failure
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py \
  control/tests/test_shared_root_contract.py \
  control/tests/test_shared_root_palm_priority_profile.py -q
```

回放断言未放宽：逐源帧及 200 Hz 参考循环仍要求每个控制周期被参考控制器
接受、无 reset failure，且保留关节限位检查。审计还逐有效周期检查闭合骨长
与末点残差 ≤1e-9 m。接受安全停止周期不等于持续跟踪目标。

本轮发现并修复的验证失败：

- 首轮参考回放在 4388/9999 个周期中分别少接受 2/1 个周期，原因是 shared-root
  恢复错误调用未使用的 Ruckig reference reset；移除该无关包络依赖后，原断言通过。
- 新增严格端点断言发现 settled-hold 只替换 palm 的旧诊断逻辑；新模式现显示整条
  held-model FK。闭合候选仍独立检查 1e-9 m，不将模型采样误差当闭合误差。

`git diff --check` 通过；`pico2_hands/`、`pico2_sim.sh`、`real_robot/` 和原 XML
相对 HEAD 零差异。现有其余未提交修改保留，未做全仓库测试或清理。

未做：新设备录制、Viewer 播放、动力学/真机验收、代表性分动作覆盖率冻结、
QP 精度优化、Phase B、commit/push。PICO2、旧模型与驱动不属于本轮修改范围。
