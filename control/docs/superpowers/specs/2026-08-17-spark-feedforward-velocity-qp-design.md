# SPARK一致参考前馈Velocity QP设计

## 目标

新增独立算法`spark_upper_qpoases_feedforward_velocity_qp`，在不改变任何历史算法行为的前提下，同时提高PICO遥操作的末端跟手性和关节平滑性。

历史基线`spark_upper_qpoases_velocity_qp`已经具有速度、加速度、jerk、制动和关节位置硬约束。新算法不重复包装这些约束，而是改善进入Velocity-Level QP之前的SPARK参考生成：从同一条连续关节参考生成Cartesian pose、Cartesian twist前馈和关节速度软目标。

## 非目标

- 不修改Velocity-Level QP的决策变量、目标结构和硬约束。
- 不修改SPARK骨长缩放或两阶段qpOASES位置IK。
- 不修改Relative Mapping、PICO协议或模型状态反馈策略。
- 不让正常运动全程通过Cartesian OTG或Joint Ruckig。
- 不改变已有算法的枚举值、CLI名称、配置和遥测语义。

## 独立算法与隔离边界

新增CLI名称：

```text
spark_upper_qpoases_feedforward_velocity_qp
```

新算法拥有独立枚举、独立`SparkPostureGuideMode`和独立配置段：

```yaml
spark_feedforward_velocity_qp:
  enabled: true
  alpha: 0.70
  beta: 0.16
  dt_median_window: 31
  dt_min_ratio: 0.5
  dt_max_ratio: 1.5
  maximum_joint_jump_rad: 0.35
  stale_velocity_decay_seconds: 0.02
  reference_velocity_scale: 1.0
  reference_acceleration_scale: 1.0
  reference_jerk_scale: 1.0
  reference_position_gain: 10.0
  position_feedforward_gain: 1.0
  orientation_feedforward_gain: 1.0
  joint_position_gain: 4.0
  attack_seconds: 0.04
  release_seconds: 0.05
```

现有`kJointReferenceVelocity`、`kOtgConsistentJointReferenceVelocity`及其代码路径保持不变。新模块失败时只让新算法安全保持，不回退到另一个算法，也不改变历史模式状态。

## 数据流

```text
PICO corrected skeleton
  -> SPARK bone retargeting
  -> two-stage qpOASES position IK
  -> q_ik_raw
  -> branch continuity and timestamp validation
  -> q_ik_continuous
  -> timestamp-aware alpha-beta joint velocity estimator
  -> jerk-limited reference shaping
  -> q_ref_ik, qdot_ref_ik, qddot_ref_ik
     |-> FK(q_ref_ik) -> Cartesian reference pose
     |-> J(q_ref_ik) qdot_ref_ik -> Cartesian twist feedforward
     `-> qdot_nom = qdot_ref_ik + Kq (q_ref_ik - q_model)
  -> existing Cartesian velocity servo
  -> existing Velocity-Level QP
  -> existing reference integrator
```

模型参考状态`q_model/p_model/R_model`继续作为外层Cartesian Servo反馈，不改用滞后的真机反馈。

## 时间戳与连续性

估计器只在收到新的PICO/SPARK IK结果时更新。200 Hz控制循环遇到重复目标时不得再次差分。

以最近31个有效源周期的中位数`median_dt`为基准：

```text
valid_dt =
  0.5 * median_dt <= dt_source <= 1.5 * median_dt
```

时间戳重复、倒退或掉帧时不使用异常`dt`差分。估计器保持上一速度，并在`stale_velocity_decay_seconds`内因果衰减到零。

关节角先按最短角差连续化：

```text
delta_q = wrapToPi(q_ik_raw - q_ik_previous)
q_ik_unwrapped = q_ik_previous + delta_q
```

若任一关节单帧变化超过`maximum_joint_jump_rad`，原始跳变不会直接进入参考；连续目标每个源周期最多向新候选移动该阈值。这样既阻止单帧翻轴脉冲，也避免拒绝器永远与旧分支比较而永久冻结。PICO会话和历史算法不受影响。

## Alpha-Beta速度估计

第一版不预测未来位置。Alpha-Beta估计器只生成低噪声、低延迟的关节速度：

```text
q_predict = q_est_previous + qdot_est_previous * dt
residual = q_ik_unwrapped - q_predict
q_est = q_predict + alpha * residual
qdot_est = qdot_est_previous + beta * residual / dt
```

这样比固定低截止频率滤波更少拖慢快速主动运动，也避免直接差分把SPARK帧间噪声放大。

## 轻量jerk受限参考整形

参考整形运行在200 Hz控制周期。它只生成QP软参考，不替代QP硬可行域：

```text
qdot_track = qdot_est + 0.5 * reference_position_gain
                         * (q_est - q_ref_previous)

qddot_requested = 2 * reference_position_gain
                    * (qdot_track - qdot_ref_previous)

qddot_lower = max(-a_ref_max,
                   qddot_ref_previous - j_ref_max * control_dt)
qddot_upper = min(+a_ref_max,
                   qddot_ref_previous + j_ref_max * control_dt)

qddot_ref = clamp(qddot_requested, qddot_lower, qddot_upper)
qdot_ref = clamp(qdot_ref_previous + qddot_ref * control_dt,
                 -v_ref_max, +v_ref_max)
q_ref = q_ref_previous + qdot_ref * control_dt
```

默认参考上限等于现有QP硬上限，不降低机器人能力：

```text
acceleration = [60, 60, 60, 90, 90, 90, 90] rad/s^2
jerk = [1000, 1000, 1000, 1500, 1500, 1500, 1500] rad/s^3
```

## 一致Cartesian与关节参考

平滑关节参考生成两类完全同相位的任务：

```text
T_ref = FK(q_ref)
Vff = J(q_ref) * qdot_ref

V_position = Kp_position * (p_ref - p_model)
             + Kff_position * Vff_position

V_orientation = Kp_orientation * Log(R_ref * R_model^T)
                + Kff_orientation * Vff_orientation

qdot_nom = qdot_ref + Kq * wrapToPi(q_ref - q_model)
```

Cartesian pose、twist前馈和SPARK关节软目标均来自同一`q_ref/qdot_ref`，避免原始掌心目标与滞后关节参考处于不同动态相位。

## Velocity-Level QP保持不变

QP继续使用原决策变量和约束：

```text
decision:
  qdot, Cartesian slack, task-scaling beta

objective:
  Cartesian velocity tracking
  + SPARK joint velocity reference tracking
  + qdot continuity
  + acceleration-trend continuity
  + slack and task-scaling costs

hard constraints:
  joint position
  joint velocity
  joint acceleration
  joint jerk
  braking
  outward-only
  collision when enabled
```

## 状态转换和失败处理

状态为`Hold`、`Active`和`Stopping`：

- `Hold -> Active`：将`q_ref`同步到`q_model`，速度和加速度清零；任务权重按`attack_seconds`平滑启用。
- `Active -> Stopping`：保持最终关节目标，按参考jerk边界将速度平滑降到零；任务权重按`release_seconds`释放。
- tracking epoch变化或重同步：不得跨epoch差分；参考重新同步到模型状态。
- 单次估计或IK帧失败：保持上一有效参考，不推进异常目标。
- 单周期QP失败：不推进既有reference integrator；新估计器保持可恢复状态。
- 连续失败或PICO stale：进入`Stopping`，完成jerk受限停止后进入`Hold`。

正常Active运动不调用Ruckig。第一版也不启用未来预测，避免未验证的超调。

## 遥测

为新算法增加独立诊断字段，至少包括：

- 原始、连续化和整形后的`q_ik`。
- `qdot_est/qdot_ref/qddot_ref/jerk_ref`。
- `dt_source/median_dt/dt_valid`。
- 拒绝跳变、拒绝时间戳和重置计数。
- `Vff`线速度与角速度模长。
- attack/release phase与状态。

原有遥测列保持兼容。

## 测试与验收

### 单元测试

- 重复、倒退和掉帧时间戳不产生速度脉冲。
- 角度跨越正负pi时参考连续。
- 跳变参考被拒绝。
- 参考速度、加速度和jerk不越界。
- Hold、Active、Stopping和epoch重置正确。
- FK pose、Jacobian twist和关节参考来自同一状态。

### 集成与回归

- 新CLI算法独立启动并产生有效遥测。
- 历史算法路径和测试保持不变。
- PICO stale后参考停止且不持续积分。
- 完整CTest通过，控制失败和headless deadline miss为零。

### 四路离线对比

使用同一条`pico_fast_motion_20260812_205428_v4.tjvr`比较：

1. SPARK + Velocity QP。
2. SPARK + Cartesian OTG + Velocity QP。
3. SPARK + Joint Ruckig。
4. SPARK + Feedforward Velocity QP。

输出原始意图到TCP的三维轨迹、位置/姿态误差、离散延迟扫描、幅值保持率、q/qdot/qddot/jerk统计、约束激活和求解周期。

### 验收门槛

- 位置相位延迟目标不高于100 ms，硬性不得超过历史Velocity QP的110 ms。
- 位置和姿态误差不得比历史Velocity QP恶化。
- qddot和jerk P95至少降低20%。
- 启动、停止、重同步不出现IK分支翻转或持续末端振荡。
- 停止后三秒TCP峰峰值不高于历史Velocity QP。
- control failure与headless deadline miss均为零。

若跟手指标提高但平滑性未达到门槛，保留新算法和数据，不修改历史算法；后续只在新配置段内调整Alpha-Beta和参考整形参数。

## 2026-08-17固定快速轨迹实测结论

初始平衡参数为`alpha=0.70`、`beta=0.18`、`reference_position_gain=10.0`、`attack_seconds=0.04`。后续防超调优化将`beta`降至0.16，并增加轻触式停止/反向趋势释放和连续目标制动包络。在2134帧固定PICO快速轨迹上：位置相位延迟保持99 ms，幅值保持率由1.0188改善为约0.993，位置均值由110.48 mm改善到107.69--108.68 mm。全部关节位置、速度、加速度和jerk硬边界违规数为零，控制失败为零。

因此新算法是“更低相位延迟、略低典型jerk、略高路径误差”的独立实验模式，不替换历史默认算法。严格的“误差不得恶化”和“qddot/jerk P95降低20%”门槛尚未同时满足。
