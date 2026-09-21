# SPARK Feedforward参考防超调设计

## 问题与证据

固定快速PICO轨迹显示，`spark_upper_qpoases_feedforward_velocity_qp`的模型TCP对内部参考跟踪良好，但内部参考相对原始SPARK意图存在明显超调和相位滞后。平衡参数的左右臂`actual/reference`幅值比分别约为1.0006和1.0057，而`reference/raw`约为1.0154和1.0171。因此超调主要来自Alpha-Beta速度估计与jerk受限参考整形，而不是Velocity QP。

## 约束

- 不修改Velocity-Level QP、Cartesian Servo、SPARK缩放和两阶段qpOASES位置IK。
- 不修改关节位置、速度、加速度、jerk、制动、outward-only或碰撞硬约束。
- 不修改历史算法行为。
- 不预测未来位姿，不用整体降低带宽换取稳定。

## 方案

仅增强`SparkFeedforwardReference7`：

1. 对每个关节计算当前源速度，并根据它与上一估计速度的方向一致性生成前馈置信度。
2. 正向持续运动时使用原Alpha-Beta更新；源速度减小、静止或反向时快速释放旧速度，防止旧趋势继续推动参考越过目标。
3. 参考整形的位置校正速度增加目标相对制动包络：

```text
error = q_est - q_ref
v_correction = 0.5 * position_gain * error
v_brake = sqrt(2 * a_reference * abs(error))
v_correction = clamp(v_correction, -v_brake, +v_brake)
v_des = confidence * qdot_est + v_correction
```

4. 原有速度、加速度和jerk限制继续作为最终硬包络。制动包络只约束趋近目标的软参考速度，不直接裁剪`q/qdot/qddot`状态，因此不会制造导数跳变。

## 参数

在独立配置段新增：

```yaml
source_stationary_velocity_rad_s: 0.002
velocity_reversal_decay: 0.7225
velocity_stationary_decay: 0.9025
target_braking_acceleration_scale: 0.75
```

衰减系数位于`[0,1]`。最终采用轻触式衰减：它只削弱旧趋势，不瞬间把速度切换到新方向。制动加速度比例位于`(0,1]`，只用于连续的目标相对速度包络，不改变硬限制。

## 验收

- 单元测试证明反向和静止输入会快速释放旧速度。
- 快速趋近静止目标时，参考不出现显著越过目标，同时保持原速度、加速度和jerk限制。
- 固定快速PICO回放的幅值保持率目标为`0.98--1.01`，相位延迟不超过110 ms。
- 完整CTest通过，历史算法不回归。
