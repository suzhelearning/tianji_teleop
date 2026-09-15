# Velocity QP 固定目标停稳设计与实测结论

## 问题

PICO 快速轨迹结束后，Cartesian target 与 reference 已冻结，但左臂在受限构型附近持续形成约 1.5--3.6 Hz 极限环。基线回放中关节速度、加速度和 jerk 反复触及既有硬边界，左臂最后三秒 TCP 三维峰峰值达到 68.45 mm。

该现象不是 PICO 数据继续抖动，也不是 SPARK 掌心坐标错误：stale 段的目标和参考位姿保持不变。根因是高 Cartesian Kp 与 Velocity-Level QP 的速度、加速度和 jerk 可行域共同形成饱和极限环。

## 约束

- 保持 Velocity-Level QP 的决策变量、目标和硬约束结构不变。
- 保持 SPARK 两阶段位置 IK、强关节参考和 outward-only 逻辑不变。
- 不降低关节速度、加速度或 jerk 上限。
- 使用模型参考状态闭环，不切换到实机反馈。
- 仅改变 PICO profile；其他 benchmark/profile 保持历史行为。

## 单变量实验

### 关闭 task scaling

关闭 task scaling 后固定目标振荡恶化，左右臂最后三秒 TCP 峰峰值分别约为 215 mm 和 66 mm，因此 task scaling 必须保留。

### 全程模型速度阻尼

测试了：

```text
V_des = V_ref + Kp * e + Kd * (V_ref - J*qdot_ref)
```

在 `Kd=1` 时左臂仍振荡，并使原本稳定的右臂进入约 76 mm 的固定目标振荡。该方案会在运动阶段重复放大 `V_ref`，已否决，代码中不保留此功能。

### 全程低增益

将位置/姿态 Kp 从 15/10 降为 4/3 可以完全停稳，但快速段 P50/P95 跟踪误差有所增加，不适合作为全局配置。

### 平滑自适应 Kp

最终采用误差调度：

```text
u = clamp((||e|| - e_start) / (e_end - e_start), 0, 1)
s = u*u*(3 - 2*u)
Kp(e) = Kp_near + s*(Kp_far - Kp_near)

V_des = V_ref + Kp(e)*e
```

PICO profile 参数：

```yaml
adaptive_gain_enabled: true
kp_position: [15.0, 15.0, 15.0]
kp_position_near: [4.0, 4.0, 4.0]
position_gain_transition_start_m: 0.02
position_gain_transition_end_m: 0.22

kp_orientation: [10.0, 10.0, 10.0]
kp_orientation_near: [3.0, 3.0, 3.0]
orientation_gain_transition_start_rad: 0.03
orientation_gain_transition_end_rad: 0.15
```

窄位置过渡带 0.01--0.05 m 仍会在误差增大时把 Kp 快速推回高值，无法消除左臂极限环；最终使用 0.02--0.22 m 的宽 smoothstep 过渡。

## 离线验证

输入：

```text
/home/zj/current_robotics/TJ_arm/vr_data/converted_inputs/tjvr/
pico_fast_motion_20260812_205428_v4.tjvr
```

算法：`spark_upper_qpoases_velocity_qp`，Velocity-Level、model-state-only、200 Hz。

结果：

| 指标 | 高增益基线 | 自适应 Kp |
|---|---:|---:|
| 左臂最后 3 s TCP 峰峰值 | 68.45 mm | 约 1e-7 mm |
| 右臂最后 3 s TCP 峰峰值 | 0 mm | 0 mm |
| 左臂 live 位置误差 P50 | 102.06 mm | 107.18 mm |
| 左臂 live 位置误差 P95 | 309.03 mm | 301.89 mm |
| 右臂 live 位置误差 P50 | 56.01 mm | 58.00 mm |
| 右臂 live 位置误差 P95 | 276.19 mm | 280.73 mm |

动态 P50 退化约 3.6%--5.0%，P95 基本持平；固定目标极限环被消除。

参考关节轨迹仍满足原配置：

```text
|qdot|  <= 4 rad/s
|qddot| <= [60, 60, 60, 90, 90, 90, 90] rad/s^2
|jerk|  <= [1000, 1000, 1000, 1500, 1500, 1500, 1500] rad/s^3
```

完整串行 CTest 为 65/65 通过，回放中 `control_failures=0` 且无 deadline miss。

## 已知剩余问题

`spark_upper_qpoases_velocity_qp` 当前直接使用 SPARK palm pose，同时以高权重追踪两阶段 qpOASES 的 `q_ik`。该模式绕过 Cartesian OTG，记录中的 Cartesian reference twist 始终为零，因此快速运动仍只能依靠位姿误差反馈追赶目标。

下一阶段应从同一 `q_ik` 通过 FK 生成一致的末端 pose，并从连续 `q_ik` 序列生成 Cartesian twist 前馈，使末端任务与关节参考动态一致。该优化不属于本次停稳修复。
