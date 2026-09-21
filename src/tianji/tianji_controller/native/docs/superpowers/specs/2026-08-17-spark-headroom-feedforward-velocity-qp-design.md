# SPARK约束余量感知前馈Velocity QP设计

## 目标

新增独立算法`spark_upper_qpoases_headroom_feedforward_velocity_qp`。它在现有
Raw-Palm SPARK速度前馈路径上，根据上一控制周期真实QP输出的关节运动学余量和
task scaling状态调节下一周期Cartesian前馈增益，保留普通运动的相位收益，同时
降低快速运动、停止和反向时因持续撞限造成的P95误差与超调。

## 约束与隔离

- 保留`spark_upper_qpoases_feedforward_velocity_qp`的行为、CLI和配置不变。
- 不修改Velocity-Level QP的决策变量、目标函数和求解次数。
- 不修改关节位置、速度、加速度、jerk、制动、outward-only和碰撞硬约束。
- 不修改SPARK缩放、两阶段qpOASES位置IK、Relative Mapping和模型状态反馈。
- 新模式失败时安全退化为零Cartesian前馈，但保留原位置反馈和弱关节姿态参考。

## 方案选择

采用上一周期反馈式余量调度。相对于QP前Jacobian预测，它不需要额外近似；相对于
二次QP，它不增加求解开销。一个5 ms周期的因果延迟由快速衰减时间常数覆盖。

## 数据流

```text
Raw SPARK palm pose/twist
  -> existing feedforward decomposition
  -> previous-cycle QP headroom governor
  -> scaled Cartesian velocity feedforward
  -> existing Cartesian Velocity Servo
  -> existing Velocity-Level QP
  -> qdot, task scale and hard-bound diagnostics
  -> update governor for next cycle
```

每条手臂拥有独立状态，左右臂不得共享最小余量。

## 余量定义

上一周期已接受QP结果给出`qdot`。控制器保存的上一周期`qdot_previous`和
`qddot_previous`用于计算：

```text
velocity_usage = max_i(abs(qdot_i) / velocity_limit_i)
acceleration_usage = max_i(abs(qddot_i) / acceleration_limit_i)
jerk_usage = max_i(abs((qddot_i-qddot_previous_i)/dt) / jerk_limit_i)

h_velocity = clamp(1 - velocity_usage, 0, 1)
h_acceleration = clamp(1 - acceleration_usage, 0, 1)
h_jerk = clamp(1 - jerk_usage, 0, 1)
h_task = clamp((min(beta_position,beta_orientation)-beta_min)
               / (1-beta_min), 0, 1)

h_raw = min(h_velocity, h_acceleration, h_jerk, h_task)
```

无有效历史、Hold、epoch重置或QP失败时，`h_raw=0`。这不会冻结位置反馈，只关闭
尚未证明安全的速度前馈。

## 平滑调度

余量映射使用五次smoothstep：

```text
u = clamp((h_filtered - low_headroom) /
          (full_headroom - low_headroom), 0, 1)
kff_scale = u^3 * (10 - 15*u + 6*u^2)
```

默认参数：

```yaml
spark_headroom_feedforward_velocity_qp:
  enabled: true
  low_headroom: 0.10
  full_headroom: 0.30
  reduction_time_seconds: 0.020
  recovery_time_seconds: 0.100
  task_scaling_min_position: 0.75
  task_scaling_min_orientation: 0.75
```

余量下降使用20 ms时间常数，余量恢复使用100 ms时间常数。接近约束时快速撤掉
前馈，离开约束后缓慢恢复，避免边界附近反复开关。最终位置和姿态前馈均乘同一
手臂尺度，保证6D运动方向不被调度器自身扭曲。

## 状态与异常处理

- 首次Active周期从`kff_scale=0`启动，随后按恢复时间平滑启用。
- QP接受后更新余量；QP拒绝、fallback、stale或epoch变化立即请求快速降到零。
- 非有限输入直接输出零比例并记录`invalid_feedback`，不得传播NaN。
- 当前算法的attack/release、时间戳验证和palm twist估计保持原样。

## 遥测

新模式增加每臂：

- `headroom_raw`、`headroom_filtered`和`kff_scale`。
- velocity、acceleration、jerk和task四项余量。
- 主导约束名称与governor状态。

现有CSV列和旧算法语义保持兼容，新列对旧模式填中性值或`inactive`。

## 测试与验收

单元测试覆盖余量计算、最小项选择、smoothstep边界、快降慢升、无效反馈、左右臂
状态隔离和重置。配置、枚举、CLI、遥测与Viewer集成必须有回归测试。

固定v4快速PICO回放与当前固定前馈算法A/B，要求：

- control failures和deadline misses为0。
- 关节位置、速度、加速度和jerk违规为0。
- 位置相位延迟不超过99 ms。
- 平均位置误差不超过102.31 mm。
- 位置P95低于当前固定前馈的302.62 mm。

如果P95目标未达到，保留独立模式和完整数据，但不得用提高硬限制或修改历史算法
来伪造通过；后续只调整新模式的余量阈值和时间常数。
