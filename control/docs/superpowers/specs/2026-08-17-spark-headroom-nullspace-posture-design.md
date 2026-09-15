# SPARK余量感知Velocity QP静止姿态稳定设计

## 问题与证据

实时遥测中的严重末端震荡窗口具有以下特征：

- 实际送入Cartesian/QP的SPARK palm twist接近零；
- 原始corrected PICO palm速度估计却出现最高`3 m/s`和`12 rad/s`尖峰；
- 原实现只用原始PICO palm判断是否允许更新SPARK关节参考；
- 因此手柄到位时仍持续接受骨架冗余变化，姿态速度目标达到`4 rad/s`。

静止判据和生成`q_ik`的目标不在同一数据域，是本次偶发震荡的直接原因。

## 被实验否决的方案

以下两种方案均使用同一快速PICO轨迹做过固定回放，不能保留：

1. QP内严格零空间姿态目标：位置误差和task-scaling饱和均恶化。
2. `posture_activation = feedforward_activation * headroom_scale`：姿态关闭后
   构型漂移，进一步耗尽余量，形成自锁。

硬拒绝超过`maximum_joint_jump_rad`的IK目标同样不可用；部分高位姿变化是持续
存在的新可行解，硬拒绝会永久停留在旧构型。

## 最终设计

保留现有Velocity QP、全关节SPARK软姿态目标、权重和全部硬约束。只增强
headroom算法的静止参考门控：

```text
raw_stationary   = raw corrected PICO palm twist低于阈值
spark_stationary = retargeted SPARK palm twist低于阈值

hold_joint_reference =
    headroom_mode
    and reference_initialized
    and (raw_stationary or spark_stationary)
```

只有两个独立数据域都确认正在运动时，才允许新两阶段qpOASES解更新关节参考。

- raw palm异常、SPARK目标静止：保持，屏蔽corrected pose尖峰；
- raw palm静止、SPARK骨架冗余抖动：保持，屏蔽臂形噪声；
- 两者都运动：正常更新，保留快速遥操响应。

原始motion-intent estimator继续保留在遥测中，便于诊断，但不再单独决定是否
更新姿态参考。

## 隔离与约束

- 仅影响`spark_upper_qpoases_headroom_feedforward_velocity_qp`。
- 固定前馈及历史算法行为不变。
- 不修改QP决策变量、Cartesian servo、task scaling和求解次数。
- 不修改关节位置、速度、加速度、jerk、制动、outward或碰撞约束。
- 不修改姿态目标权重，也不在QP后处理`qdot`。

## 验证结果

- 12项SPARK guidance和8项reference测试通过。
- 固定快速轨迹2134帧、90.9 Hz输入，控制P99约`0.662 ms`，零deadline miss、
  零control failure、零telemetry drop。
- 快速运动段相对旧基线基本持平；左臂位置P95由`326.0 mm`降至`320.4 mm`，
  最大位置误差由`489.3 mm`降至`458.8 mm`，最大position slack由`4.83`降至
  `3.98`。该轨迹主要验证不损害高速响应。
- 最终是否消除现场到位震荡，需要用真实PICO静止保持数据复测。
