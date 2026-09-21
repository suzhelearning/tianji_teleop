# Shared-root：Viewer 末端跟踪参数消融

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

2026-09-18。本轮完成基线和五个调整组的真实录制 UDP 无窗口仿真回放。
六组均发送并收到全部 4427 帧。没有发现能同时改善双臂且保持稳定性的配置，
因此未替换候选或默认参数。详细数值见 [统计数据](shared_root_tracking_tuning_20260918.json)。

## 本轮对先前解释的修正

- 上轮完整诊断中首个有效目标在 control_time=4.525 s，但匹配 TJVR sequence 后，
  对应录制时间仅 0.049 s。4.5 s 包含进程启动和等待发送，不能解释为形态初始化。
  上轮直接按 control_time 切动作区间的统计也不能用于动作归因。
- headroom.scale 缩放 Cartesian 前馈 twist，不是整体关节速度的比例。
  小 scale 与大跟踪误差相关，但仅凭它不能认定限速就是根因。
- 微米级数值是姿态 IK 对其求解目标的残差，不是人手到机器人映射准确度。
- settled hold 会把下游 command 改成当前模型 TCP。command 误差变小可能是目标
  被保持机制替换，不能独立证明更好地跟踪原始映射目标。

## 同口径比较

基线为 `qp_ik_pico_shared_root_reachable.yaml`；临时副本仅开启 shared-root，
将两个 artifact 路径指向原仓库文件。每个实验只在这个基线上改变表中软参数。
Viewer 使用配置初始姿态、model_state_only、原硬限位和硬 jerk 接线；joint export=0。
没有使用原离线参考工具结果混入比较：该工具从限位中点初始化，接线口径不同。

通过 TJVR 的 epoch、sequence 将每个控制周期定位到真实录制时间。
主指标取录制 5～47 s、双侧目标有效且 pico_live 的周期，避免启动等待污染；
动作分段使用录制中的 actions-confirmed.json。按控制周期加权，未做相位补偿。
不同运行有调度和 latest-frame supersession 差异，因此这是单次筛选实验，非确定性验收。

| 组别 | 调整 | 左位置 P90 | 右位置 P90 | 控制失败 |
|---|---|---:|---:|---:|
| baseline | 无 | 137.1 mm | 89.4 mm | 0 |
| smoothness | 速度平滑权重 1e5 → 1e4 | 132.8 mm | 90.8 mm | 11 |
| jerk_soft | 加速度趋势软权重 3e4 → 3e3 | 134.2 mm | 89.0 mm | 0 |
| joint_weight | 关节参考权重 3e4 → 3e3 | 116.1 mm | 105.2 mm | 0 |
| position_gain | 近目标位置增益 [4,4,4] → [10,10,10] | 133.9 mm | 84.1 mm | 0 |
| combined | joint_weight + position_gain | 107.2 mm | 102.8 mm | 0 |

这些位置误差比较的是 **下游 command 与模型 TCP**，不是原始映射目标或真实设备反馈。
统计没有排除 QP 失败周期；失败组明确保留在表中。
smoothness 组为左 7 次、右 4 次 solver_failure/numerical_error，不采用。
其全程每周期最大关节加速度的 P90 从基线 23.94 增至 60 rad/s²。
jerk_soft 收益很小，而 jerk 同口径 P90 从 1368 增至 1500 rad/s³，不采用。

position_gain 的左/右 settled-hold 占比从基线 13.06%/1.77% 增至 17.21%/7.52%，
不能将小幅 command 误差收益当作无条件改善。
combined 虽改善左臂，但右臂位置 P90 恶化约 15%；不作为推荐配置。
joint_weight 和 combined 的关节加速度/jerk P90 下降，但这也不能抵消右臂跟踪退化。

## 验证、证据及复现

实际构建命令：`cmake --build control/build --target tianji_qp_ik_viewer -j2`，
结果 `ninja: no work to do`。六组 Viewer 各运行 57 秒，replay lead=2 秒，原始 packet 不改写。
每组 receiver 都接收 4427 帧；CSV 保留 source sequence、command/model pose、QP 状态和关节导数。
逐关节检查 reference q/qdot/qddot/jerk 对 telemetry 中上下界，容差 1e-7；
导数只统计对应 valid 标记有效的周期。六组均无非有限值及界外样本。
这不意味着没有瞬态失败，smoothness 的 11 次失败由现有处理路径接住。

本机原始证据目录：`/tmp/shared-root-tuning.2xeplM/`，包含每组配置、
`*.viewer.log`、`*.csv`、`*.joints.csv`、`run.py`、`analyze.py`。
该目录为临时工作证据，长期结论和参数差异保存在同目录链接的仓库 JSON 中。
复现时从基线按 JSON 的 changes 生成独立配置，保持 artifact 指向原文件；
启用 shared-root 仅用于本次仿真，使用空闲本地端口。启动命令在 control 目录执行：

```bash
build/tianji_qp_ik_viewer --config /absolute/path/trial.yaml \
  --pico-bind 127.0.0.1 --pico-port 25839 --headless --duration 57 \
  --model-state-only --joint-command-port 0 \
  --telemetry /fresh/output/trial.csv --joint-telemetry /fresh/output/trial.joints.csv
```

确认 receiver 启动后，另一终端在工程根目录执行：

```bash
python3 control/scripts/replay_pico_udp_trace.py \
  --input recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr \
  --host 127.0.0.1 --port 25839 --lead 2
```

未修改控制运行逻辑，本轮没有补跑全量 CTest，也不沿用旧测试数作为本轮结果。
本轮没有新开 GUI 或操作设备；结果仅为录制驱动的模型状态仿真。

## 后续优化边界

下一步应把原始映射目标、hold 后 command、IK FK 和模型 TCP 在同一 source sequence 下
并列输出，保留目标无效/保持周期，再做左右臂和动作段的比较。
优先确认是否存在有残余误差却提前进入 settled hold 的情况，以及软关节参考目标与
Cartesian 纠偏的冲突；不得通过关闭硬限位或删去失败样本制造收益。
现阶段证据支持继续定位这两项，尚不足以宣称 headroom 是唯一根因。
