# Cartesian经验频响基准设计

## 目标

新增独立、可重复的MuJoCo小信号经验频响基准，用统一激励比较四条Cartesian IK/控制路线：

1. Direct Velocity QP。
2. SPARK + Velocity QP。
3. SPARK + Cartesian OTG + Velocity QP。
4. 当前优化后的SPARK + Feedforward Velocity QP。

基准用于解释跟手性、幅值超调和相位延迟随频率与工作姿态的变化，并为Feedforward模式调参。它不修改任何控制算法、QP结构或硬约束。

## 方法选择

首版采用离线MuJoCo小信号chirp，而不是直接从VR快速轨迹或人工PICO扫频估计频响。固定激励具有确定的频率覆盖和可重复工作点，能区分系统响应与操作者输入差异。已有VR数据继续作为大信号瞬态验证，不替代小信号FRF。

系统包含QP约束、运动学姿态变化和参考整形，因此报告的是各工作点附近的经验频响，不把完整系统声明为全局线性时不变系统。

## 对比算法

命令行算法名称固定为：

```text
hierarchical_qp
spark_upper_qpoases_velocity_qp
spark_upper_qpoases_cartesian_otg_velocity_qp
spark_upper_qpoases_feedforward_velocity_qp
```

`hierarchical_qp`作为Direct Velocity QP基线。四种算法使用同一模型、初始状态、工作点、控制周期、激励序列和安全限制。

## 工作点

每条手臂分别测试七个代表性TCP工作点：

```text
center
x_negative, x_positive
y_negative, y_positive
z_negative, z_positive
```

偏移以可达中心姿态为基准，第一版位置偏移为50 mm。工作点姿态固定为中心姿态。生成数据前先验证IK可达、关节限位余量大于0.10 rad，且静态Cartesian误差小于2 mm；不满足时该工况明确标记为无效，不以临近工况替代。

左右臂单独激励，另一条手臂保持中心目标。这样避免双臂同步输入混入交叉耦合，同时仍保留完整双臂模型和安全约束。

## 激励

每个工作点依次激励六个Cartesian通道：

```text
translation: X, Y, Z
orientation: Rx, Ry, Rz
```

输入为对数扫频chirp：

```text
frequency: 0.2 -> 15 Hz
warmup: 2 s
chirp: 20 s
settling hold: 2 s
control dt: 0.005 s
synthetic PICO source dt: 0.010 s
position amplitude: 0.005 m
orientation amplitude: 1 degree
```

姿态激励使用SO(3)指数映射：

```text
R_target(t) = Exp(axis * angle(t)) * R_center
```

不使用Euler角差分。激励相位由固定时间索引生成，不使用墙钟时间，确保不同算法得到逐采样相同输入。
SPARK算法每两个控制周期更新一次合成corrected skeleton与PICO pose，严格复现100 Hz输入、200 Hz控制的多速率链路；中间控制周期保持最近一帧并继续运行guidance与QP。

## 数据流

```text
fixed Cartesian working point
  -> deterministic logarithmic chirp
  -> selected algorithm
  -> existing Cartesian Servo / OTG / Velocity QP path
  -> model-state reference integrator
  -> MuJoCo/Pinocchio model TCP
  -> synchronized target/reference/actual telemetry
```

采集字段至少包括：

- 算法、手臂、工作点、激励通道和采样时间。
- 输入Cartesian位移或SO(3)旋转向量。
- target、reference和actual TCP位置与姿态。
- Cartesian误差。
- q、qdot、qddot和jerk。
- QP求解状态、周期耗时和各硬约束激活计数。

## FRF估计

分析脚本从同步输入`u`与输出`y`计算Welch谱和H1估计：

```text
H1(f) = S_yu(f) / S_uu(f)
coherence(f) = |S_yu(f)|^2 / (S_uu(f) * S_yy(f))
magnitude_db(f) = 20 * log10(|H1(f)|)
phase_deg(f) = unwrap(angle(H1(f)))
group_delay(f) = -d phase_rad / d omega
```

位置通道直接分析对应轴。姿态输出先相对工作点计算SO(3)对数映射，再投影到激励轴。预热和结束保持不进入谱估计。

使用固定Welch窗口、50%重叠和Hann窗。窗口长度由测试确定，使0.2 Hz附近具有足够分辨率，并在报告中记录。输入功率低于该工况峰值PSD的-40 dB，或coherence低于0.8的频点只显示为低可信区域，不进入带宽和群时延汇总。

## 指标

每个算法、手臂、工作点和通道输出：

- 低频增益：0.2--0.5 Hz有效频点的中位增益。
- 共振峰：低频归一化后0.2--15 Hz最大增益。
- `-3 dB`带宽。
- 相位首次达到-45°与-90°的频率。
- 1--5 Hz有效频点群时延中位数和P95。
- coherence有效频率占比。
- 关节位置、速度、加速度和jerk违规数。
- QP约束激活率、失败数和计算时间P99。

七个工作点汇总使用逐频率P10/P50/P90包络，不能只报告平均工作点。

## 输出

结果目录结构固定为：

```text
benchmark_results/cartesian_frf_<timestamp>/
  raw/
  summary.csv
  figures/
    <arm>__<channel>__bode.png
    <arm>__<channel>__coherence.png
    working_point_envelopes.png
  README.md
```

每张Bode图同时叠加四种算法，包含幅频、相频和群时延三个子图；coherence单独成图，避免低可信频点被误读为控制器特性。

## 验收标准

当前Feedforward算法的目标是：

- 0.2--0.5 Hz低频增益位于0.98--1.02。
- 可信频段内归一化共振峰不超过+1 dB。
- coherence不低于0.8的频点才参与结论。
- 1--5 Hz群时延低于旧Feedforward基线，或在误差范围内不恶化。
- 所有关节位置、速度、加速度和jerk硬边界违规为零。
- 控制失败为零。

Direct Velocity QP可能仍具有更小零时移误差，不能因为Feedforward延迟更低而隐藏该事实。报告同时列出带宽、相位、误差与连续性，不用单一指标宣布总体最优。

## 测试策略

### 单元测试

- 对已知一阶低通系统恢复增益、相位和`-3 dB`带宽。
- 对已知纯延迟恢复群时延。
- 对不相关噪声得到低coherence并排除带宽结论。
- SO(3)小角度投影恢复输入旋转轴和幅值。
- chirp在固定采样率下可重复且频率单调。

### 集成测试

- 一个工作点、一个通道、两个算法的缩短测试能生成原始CSV、汇总和图片。
- 相同算法重复运行输入序列逐采样一致。
- 无效工作点明确失败或标记，不静默输出错误FRF。
- 完整扫描结束后四算法均无硬边界违规和控制失败。

## 非目标

- 首版不连接真机SDK，不评估EtherCAT、UDP或位置伺服传输延迟。
- 首版不使用碰撞激活区做小信号线性结论。
- 首版不自动修改控制参数。
- 首版不把FRF结果推广到工作点范围之外。
- 首版不删除或替换现有圆轨迹和VR快速回放基准。
