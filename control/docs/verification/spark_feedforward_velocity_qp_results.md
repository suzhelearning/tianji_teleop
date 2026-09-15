# SPARK Feedforward Velocity QP验证结果

## 验证对象

- 算法：`spark_upper_qpoases_feedforward_velocity_qp`
- 输入：`vr_data/converted_inputs/tjvr/pico_fast_motion_20260812_205428_v4.tjvr`
- 规模：2134个PICO源帧，源周期中位数11 ms，控制频率200 Hz
- 模型反馈：`--model-state-only`
- 对比：SPARK + Velocity QP、Cartesian OTG + Velocity QP、Joint Ruckig、新Feedforward Velocity QP
- 共同窗口：2070个源帧，剔除启动与重同步窗口`1:30,807:840`

## 实施中修复的缺陷

1. 新模式最初同时通过Cartesian twist和旧`kSparkJointReference`路径注入`J*qdot`，造成前馈重复。新增`kSparkFeedforwardJointReference`，只保留同相位软关节目标。
2. 初版参考只积分估计速度，静态`q_ik`不会收敛。改为临界阻尼的位置/速度联合跟踪，再施加速度、加速度与jerk硬边界。
3. 单帧关节跳变拒绝后会永久与旧目标比较并冻结。改为原始连续目标保留、每源周期限幅重新捕获。
4. 异常源帧曾污染两阶段IK热启动。现在只有双臂参考均接受时才提交热启动状态；异常帧保持上一有效参考。

## 参数扫描

| 版本 | alpha/beta | 位置增益 | attack | 位置延迟 | 位置均值 | jerk中位数 | 结论 |
|---|---:|---:|---:|---:|---:|---:|---|
| 稳健低带宽 | 0.65/0.12 | 8 | 80 ms | 121 ms | 119.75 mm | 147.58 rad/s³ | 平滑但延迟偏高 |
| 激进高带宽 | 0.80/0.25 | 16 | 30 ms | 110 ms | 120.42 mm | 1000.00 rad/s³ | 长时间顶jerk边界，拒绝作为默认 |
| 最终平衡 | 0.70/0.18 | 10 | 40 ms | 99 ms | 110.48 mm | 285.86 rad/s³ | 最低延迟且典型jerk略低 |

## 四算法结果

| 算法 | 位置均值 | 位置延迟 | 延迟补偿RMSE | 姿态均值 | 幅值保持率 |
|---|---:|---:|---:|---:|---:|
| SPARK + Velocity QP | 100.92 mm | 110 ms | 71.88 mm | 0.3263 rad | 0.893 |
| Cartesian OTG + Velocity QP | 116.17 mm | 143 ms | 77.59 mm | 0.4355 rad | 0.879 |
| SPARK + Joint Ruckig | 113.23 mm | 132 ms | 71.83 mm | 0.3829 rad | 0.961 |
| SPARK + Feedforward Velocity QP | 110.48 mm | 99 ms | 97.02 mm | 0.3739 rad | 1.019 |

新算法相对直接Velocity QP降低11 ms相位延迟，相对Joint Ruckig降低33 ms；但零时移位置均值增加9.55 mm，延迟补偿路径RMSE也更高。因此它是“更跟手”的候选，不是当前精度最优算法。

## 关节连续性与安全

- 新算法双臂`q/qdot/qddot/jerk`边界违规均为0。
- `qddot`相邻周期最大变化7.5 rad/s²，对应既有1500 rad/s³ jerk上限和5 ms周期。
- 新算法全部回放`control_failures=0`；控制周期P99约0.60–0.70 ms。
- 平衡参数运行出现1次调度deadline miss，但该运行算法周期P99仅0.603 ms；不同重复运行在0–3次之间波动，属于普通Linux调度抖动，不能声明严格零miss。

## 产物

- 跟踪报告：`benchmark_results/spark_four_way_20260817/report_balanced/README.md`
- 三维轨迹：`report_balanced/trajectory_3d.png`
- 时序、CDF、延迟扫描和性能汇总：`report_balanced/*.png`
- 左右臂4×7连续性图：`report_balanced/joint_continuity/*_all_joints_4x7.png`
- 精确连续性统计：`report_balanced/joint_continuity/joint_continuity_summary.csv`

## 结论

新算法已作为独立模式完成，历史算法未被替换。它满足低于100 ms的离线相位延迟目标、零控制失败和零硬边界违规；尚未满足“位置/姿态误差不得恶化”和“qddot/jerk P95降低20%”两个严格门槛。真机前应先做MuJoCo交互回放，再以相同输入进行实机小速度A/B，不能仅凭离线相位结果设为默认。

## 2026-08-17 防超调优化

三维轨迹检查表明旧平衡版仍有约1.9%的幅值超调。分解`raw -> reference -> actual`后确认QP对内部参考的幅值误差很小，主要问题是Alpha-Beta速度在停止和反向时保留旧趋势。

最终仅增强新算法参考生成器：

- 有效源时间戳下检测逐关节静止与反向，轻度衰减旧速度趋势，不瞬时切换速度符号。
- 位置校正速度使用目标距离与参考加速度构造连续制动包络。
- `beta`由0.18降至0.16，降低速度估计的高频增益；位置增益和QP硬限制不变。
- 最终参数为静止阈值0.002 rad/s、单源帧反向总保留率0.7225、静止总保留率0.9025、制动加速度比例0.75。预测前和Alpha-Beta更新后各应用一次平方根，使配置值等于完整源帧的总保留率。

同一2134帧固定快速轨迹的前后结果：

| 指标 | 旧平衡版 | 优化终版 |
|---|---:|---:|
| 位置均值 | 110.48 mm | 107.69 mm |
| 位置P95 | 342.65 mm | 331.87 mm |
| 位置相位延迟 | 99 ms | 99 ms |
| 延迟补偿位置RMSE | 97.02 mm | 93.20 mm |
| 姿态均值 | 0.3739 rad | 0.3679 rad |
| 幅值保持率 | 1.0188 | 0.9932 |
| control failures | 0 | 0 |
| deadline misses（本次运行） | 1 | 0 |

优化终版双臂关节位置、速度、加速度和jerk硬边界违规均为0。最终两次等价参数回放中，综合qddot P50为`8.18--8.75 rad/s²`，旧版为`8.66 rad/s²`；综合jerk P50为`282.05--306.16 rad/s³`，旧版为`286.16 rad/s³`。这说明典型导数水平与旧版相当，但UDP源帧相对200 Hz控制周期的落点会影响离散导数统计，不能据单次回放声明jerk稳定下降。后续频响测试应使用同步激励或多次平均。

最终未采用“达到预测停车距离后强制最大减速度”的试验实现：它虽能进一步抑制单步超调，却会在切换面反复触发jerk上限。最终实现只保留连续包络。

最终产物位于：

- `benchmark_results/spark_feedforward_overshoot_20260817/report_final/trajectory_3d.png`
- `benchmark_results/spark_feedforward_overshoot_20260817/report_final/tracking_timeseries.png`
- `benchmark_results/spark_feedforward_overshoot_20260817/report_final/lag_scan.png`
- `benchmark_results/spark_feedforward_overshoot_20260817/report_final/joint_continuity/`

## 2026-08-17 Raw-Palm v2同源前馈

本轮修复了原Feedforward模式的数据源耦合：Cartesian pose不再取
`FK(q_ff)`，Cartesian twist也不再取`J(q_ff) qdot_ff`。现在主任务始终使用
同一条最新SPARK palm轨迹：

```text
T_target = T_spark_palm_latest
V_ff = timestamp_aware_SE3_difference(T_spark_palm)
V_des = Kp * Log(T_target * T_model^-1) + V_ff_shaped
```

两阶段qpOASES的`q_ik`仍经过原有速度、加速度和jerk受限参考生成器，但仅作为
低权重关节姿态任务，不再替换Cartesian主目标。重复sequence、非单调时间戳、
median-dt异常、epoch切换和stream discontinuity均由独立palm twist估计器处理；
100 Hz源帧之间的200 Hz控制周期保持最近twist，不预测新的pose。

为避免完整速度前馈在1--2 Hz造成幅值峰值，最终使用互补前馈：0.8 Hz低通
分离低/高频速度，位置低频/高频增益为0.28/0.28，姿态为0.30/0.70。
这只改变当前Feedforward模式，不修改Cartesian Servo形式、Velocity-Level QP
决策变量或硬约束。

固定2134帧快速轨迹最终结果：

| 指标 | 旧Feedforward终版 | Raw-Palm v2终版 |
|---|---:|---:|
| 位置均值 | 107.69 mm | 97.48 mm |
| 位置P95 | 331.87 mm | 302.62 mm |
| 位置相位延迟 | 99 ms | 88 ms |
| 延迟补偿位置RMSE | 93.20 mm | 97.02 mm |
| 姿态均值 | 0.3679 rad | 0.3233 rad |
| 姿态相位延迟 | 未单列 | 99 ms |
| 幅值保持率 | 0.9932 | 0.9029 |
| control failures | 0 | 0 |
| deadline misses | 0 | 0 |

Raw-Palm v2相对同次历史SPARK + Velocity QP基线，将位置延迟从110 ms降到
88 ms，位置均值从102.31 mm降到97.48 mm；但位置P95仍由284.76 mm升到
302.62 mm，说明快速轨迹尾部误差尚未全面优于无前馈基线。

最终84工况FRF结果：

| 范围 | 低频增益 | -3 dB带宽 | 1--5 Hz群时延 | 共振峰 |
|---|---:|---:|---:|---:|
| 双臂全部通道 | 1.0117 | 3.125 Hz | 34.82 ms | +0.479 dB |
| 位置XYZ | 1.0167 | 3.320 Hz | 40.39 ms | +0.488 dB |
| 姿态RxRyRz | 0.9877 | 3.125 Hz | 28.12 ms | +0.445 dB |

相对旧Feedforward FRF，全部通道群时延由53.69 ms降到34.82 ms，低频增益
由1.053收敛到1.0117，峰值由+0.918 dB降到+0.479 dB。姿态达到不高于
30 ms的目标；位置仍为40.39 ms，未达到原定30 ms目标。继续压低位置时延需要
提高1--2 Hz速度前馈，而实测会重新超过+0.5 dB峰值，因此本轮选择平滑性优先，
不通过放大Kff伪装达标。

最终验证：74/74 CTest通过；FRF 84/84工况完成且无失败。FRF和快速轨迹中
control failures均为0；最终快速回放参考`q/qdot/qddot/jerk`相对记录上下界的
违规数均为0。

最终产物：

- `benchmark_results/spark_feedforward_raw_palm_v2_20260817/report_final/`
- `benchmark_results/spark_feedforward_raw_palm_v2_20260817/feedforward_raw_palm_v2_final.csv`
- `benchmark_results/spark_feedforward_raw_palm_v2_20260817/feedforward_raw_palm_v2_final_joint.csv`
- `benchmark_results/cartesian_frf_feedforward_raw_palm_v2_final_20260817/`

## 2026-08-17 约束余量感知前馈

新增独立算法
`spark_upper_qpoases_headroom_feedforward_velocity_qp`。旧的固定前馈算法保持不变；
新算法读取上一周期已接受Velocity QP的`qdot`、`qddot`、由相邻`qddot`
得到的jerk以及位置/姿态task scaling，分别换算成0--1余量。每条手臂独立取
四项最小值，经快降慢升滤波和五次smoothstep后，只调度Cartesian速度前馈；
Cartesian位置反馈、SPARK两阶段IK弱姿态参考和全部硬约束始终保留。

默认余量阈值为0.10/0.30，下降/恢复时间常数为20/100 ms。没有有效PICO帧、
没有连续加速度历史、QP失败或输入非有限时，前馈比例安全退化到0。Viewer CSV
新增左右臂velocity/acceleration/jerk/task/raw/filtered余量、最终scale、状态和
主导瓶颈字段。

固定v4快速PICO轨迹在相同1807个有效共同源帧上的结果：

| 指标 | 无前馈Velocity QP | 固定前馈 | 余量感知前馈 |
|---|---:|---:|---:|
| 位置均值 | 103.00 mm | 100.03 mm | 99.53 mm |
| 位置P95 | 286.77 mm | 306.73 mm | 294.47 mm |
| 位置相位延迟 | 110 ms | 88 ms | 99 ms |
| 延迟补偿位置RMSE | 72.95 mm | 91.88 mm | 77.22 mm |
| 幅值保持率 | 0.879 | 0.950 | 0.953 |
| 控制周期P99 | 1088.2 us | 668.1 us | 662.4 us |
| control failures | 0 | 0 | 0 |
| deadline misses | 2 | 0 | 0 |

相对固定前馈，余量感知版本用11 ms相位延迟换取12.26 mm的P95改善和
14.66 mm的延迟补偿RMSE改善；位置均值再改善0.49 mm。相对无前馈，它仍降低
11 ms延迟和3.46 mm均值误差，但P95高7.70 mm。因此它是固定前馈与纯反馈之间
更均衡的候选，不应表述为所有指标都优于无前馈。

本次快速轨迹中，左/右臂平均前馈比例为0.087/0.208；主导瓶颈主要是jerk，
说明调度器确实在固定前馈最容易撞动态边界的区间撤掉了前馈。关节连续性报告中
双臂`q/qdot/qddot/jerk`相对记录有效上下界的违规数均为0，重置次数均为2。

验证与产物：

- 全量CTest：76/76通过。
- 五方案严格同源报告：
  `benchmark_results/spark_headroom_feedforward_20260817/report_five_way/`
- 固定前馈遥测：
  `benchmark_results/spark_headroom_feedforward_20260817/fixed_feedforward.csv`
- 余量感知前馈遥测：
  `benchmark_results/spark_headroom_feedforward_20260817/headroom_feedforward.csv`
- 关节连续性与边界统计：
  `report_five_way/joint_continuity/joint_continuity_summary.csv`
