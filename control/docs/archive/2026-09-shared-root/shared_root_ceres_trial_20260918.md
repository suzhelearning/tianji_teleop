# 当前共享根映射 → 外部 Ceres LM + Ruckig 离线试验

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

2026-09-18。已完成两组窗口回放；这是原工程后端的隔离试验，**尚未将 Ceres 接入当前现场运行入口**。不修改默认 SPARK 路线，不代表真机验收。

## 输入与兼容性

- 当前 `qp_ik_pico_shared_root_reachable.yaml` 和冻结模型保持不变。使用 `zhoujie_actions_20260918_041020/input.tjvr`，导出当前映射经过滤波及可达投影、进入 guidance blend/hold/IK 之前的双末端目标。
- EE CSV 无法表达无效帧，故只选最长连续有效段：0.650300074～48.9443457 秒，4273 帧；排除154帧（包含无效帧及该段以外的有效帧），没有跨无效区间补点。统计取录制5～47秒，每组8400控制采样，覆盖七个计划动作区间；不是整条原始录制的有效率验收。
- 后端为显式指定的 `TJ_arm_control_pico_ee_ik` 原二进制、`pico_ee_franka_ceres_lm` + Ruckig。源版本及关键源码指纹见[机器可读结果](shared_root_ceres_trial_20260918.json)。Ceres LM 是非线性最小二乘 IK，不是 QP IK。
- 使用源工程支持的 MuJoCo FK fallback，避免默认 Pinocchio TCP 与当前冻结 TCP 不一致。源后端要求14自由度，因此派生只固定40个手指关节的模型。32组随机双臂姿态检查中，关节范围一致、TCP位置及旋转矩阵分量最大差为0；没有宣称动力学等价。
- 初始关节姿态使用当前工程值，Ceres home/正则项保留源配置；不是原工程全部配置原封不动搬运。
- 统计段内 Viewer 的目标与导出目标逐时间插值校验：位置最大差 < 5.1e-12 m，旋转最大差 < 1.9e-12 rad。未以冻结目标代替映射目标降低误差。

## 结果

以下均为5～47秒采样的 P90；跟踪误差是控制器本周期求解前、平滑关节参考 FK 对目标的误差，不是真机反馈误差。

| 指标 | 当前 jerk 限制 左/右 | 源配置 jerk 限制 左/右 |
|---|---:|---:|
| Ceres 求解后位置残差 | 0.453 / 0.455 mm | 0.455 / 0.456 mm |
| Ruckig 参考末端位置跟踪误差 | 41.85 / 50.18 mm | 25.86 / 31.34 mm |
| Ruckig 参考末端姿态跟踪误差 | 0.0578 / 0.0648 rad | 0.0372 / 0.0404 rad |
| 单臂 Ceres 求解耗时 | 2.23 / 2.28 ms | 2.26 / 2.57 ms |

两组均全程固定 Ceres，control_failures=0；统计段目标 stale/held 均为0，轨迹接受率100%。这不等于每帧残差都达标：Ceres位置残差最大值分别为18.24/17.81 mm和22.86/16.46 mm；平滑后位置误差最大值分别150.30/156.58 mm和114.57/109.62 mm。

当前 jerk 为1000/1500 rad/s³，源配置为3000/4500 rad/s³（按关节分组）；源配置更激进。跟踪改善伴随更高运动限制，不能直接作为真机默认值。两组各运行一次，求解具有时间预算，残差/耗时可能受调度影响。

结论：当前映射目标可以交给 Ceres 求解，大多数样本求解残差较小；平滑与动态跟踪层仍产生明显误差。结果不能证明人体到机器人映射语义完全正确，也不能与先前带 guidance hold/blend 的 SPARK 数据直接作公平算法优劣比较。下一步若接入当前工程，应新增独立后端并保留输入新鲜度、身份、故障、限位和命令所有权门控，再在同输入、同运动限制下对比。

## 复现

在当前 `tianji_teleop` 根目录执行准备工具；输出目录必须不存在。工具不启动设备或 Viewer，外部工程路径由参数显式提供。

```bash
.pixi/envs/default/bin/python control/scripts/prepare_shared_root_ceres_trial.py \
  --source-root /home/zj/current_robotics/TJ_arm_control_pico_ee_ik \
  --trace recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr \
  --output /tmp/shared_root_ceres_trial_new
```

进入显式指定的源工程，分别运行下列命令，将 `matched_jerk` 改为 `source_limits`、输出名改为 `source_complete` 即为第二组：

```bash
DISPLAY=:1 build-ceres/tianji_qp_ik_viewer \
  --config /tmp/shared_root_ceres_trial_new/matched_jerk.yaml \
  --model /tmp/shared_root_ceres_trial_new/shared_root_arms_only.xml \
  --algorithm pico_ee_franka_ceres_lm --no-pico-teleop --model-state-only \
  --duration 70 \
  --telemetry /tmp/shared_root_ceres_trial_new/matched_complete.csv \
  --joint-telemetry /tmp/shared_root_ceres_trial_new/matched_complete_joints.csv
```

不要追加普通 `--headless`：该入口自动切换算法做冒烟测试，不适合本比较。70秒是墙钟运行时间；检查回放源时间确实覆盖目标段，不能只相信运行时长。

回到当前工程运行：

```bash
.pixi/envs/default/bin/python control/scripts/report_shared_root_ceres_trial.py /tmp/shared_root_ceres_trial_new
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest control/tests/test_prepare_shared_root_ceres_trial.py -q
```

新增准备工具测试6/6通过。禁用 pytest 自动插件发现是为了避开环境中 ROS launch_testing 缺少 lark 的无关插件错误，未安装或更改系统依赖。

扩大验证命令：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest control/tests/test_prepare_shared_root_ceres_trial.py control/tests/test_shared_root_trace_cli.py -q`，本轮67/67通过（134.29秒）。本轮未重跑全部 CTest；未修改当前原生运行逻辑。报告工具在两份完整输出上实际执行成功。

本轮原始输出位于 `/tmp/shared_root_ceres_trial_20260918_v3`；正式统计仅使用 `matched_complete.csv` 与 `source_complete.csv`。早期14自由度/CSV兼容性失败、被停止的自动切算法 headless 输出，以及未覆盖全段的50秒输出均不计入结果。临时文件可能被系统清理，仓库 JSON 保留指标及指纹，原始录制未修改。

未运行真实输入采集、未连接真实执行器、未提交或 push。没有把本次离线模型参考回放视为硬件或动力学验收。
