# SPARK Cartesian OTG 同相位 Velocity QP 设计

## 目标

新增独立算法 `spark_upper_qpoases_cartesian_otg_velocity_qp`，用于和现有 `spark_upper_qpoases_velocity_qp` 做公平 A/B。新算法必须让 Cartesian reference pose、Cartesian reference twist 和 SPARK 关节软参考处于同一 OTG 时间相位，减少快速移动时的相位滞后、双目标竞争和欠阻尼。

现有算法、配置和快捷键行为保持不变。新增模式失败时不能改变现有模式的结果。

## 当前问题

现有 `spark_upper_qpoases_velocity_qp` 使用 SPARK palm pose 作为 Cartesian 目标，同时使用两阶段 qpOASES 输出的 `q_ik` 作为高权重关节速度参考。该模式显式绕过 Cartesian OTG，且 Cartesian reference twist 为零：

```text
SPARK palm pose ----------------> Cartesian Servo（twist = 0）
SPARK two-stage q_ik -----------> high-weight joint reference
```

快速运动时，Cartesian Servo 只能通过 `Kp*pose_error` 追赶目标。`q_ik-q_model` 又可能很大，关节软目标频繁达到速度边界，因此 Cartesian 任务、关节参考和硬动态边界之间容易形成相位差和竞争。

## 新数据流

```text
PICO corrected upper skeleton
        |
        v
SPARK bone retargeting
        |
        +---- raw arm-shape targets
        |     upper direction / forearm direction / elbow
        |
        v
raw SPARK palm pose
        |
        v
Cartesian OTG
        |
        +---- T_palm_otg
        +---- V_palm_otg
        |
        v
OTG-consistent two-stage qpOASES IK
        |
        +---- Stage 1: solve T_tcp(q) = T_palm_otg
        +---- Stage 2: preserve TCP and match SPARK arm shape
        |
        v
q_ik_otg
        |
        +--------------------------+
        |                          |
        v                          v
Cartesian Servo              joint soft reference
V_des = V_otg                qdot_posture =
      + Kp(e)*e              Kq*(q_ik_otg-q_model)
        |                          |
        +-------------+------------+
                      v
               Velocity-Level QP
                      |
                      v
                    qdot
```

## Cartesian OTG

OTG 输入是当前 SPARK palm pose。OTG 状态以当前模型 TCP pose/twist 初始化，输出连续 reference：

```text
T_otg[k]
V_otg[k]
A_otg[k]
```

继续使用现有 Cartesian OTG 的速度、加速度、jerk 限制和 stale 行为，不新增第二套运动限制。

## OTG 一致位置级 IK

### 初值

每周期使用上一周期结果作为初值：

```text
q_seed = q_ik_otg[k-1]
```

首次启用、算法切换、PICO resynchronization 或 reset 时：

```text
q_ik_otg = q_model
```

不得从 nominal posture 或未经约束的解析解重新起步，以避免 IK 分支跳变。

### Stage 1：OTG TCP 主任务

定义：

```text
e_p = p_otg - p_tcp(q)
e_R = Log(R_otg * R_tcp(q)^T)
e_tcp = [e_p, e_R]
```

迭代求解：

```text
minimize
    ||J_tcp*delta_q - K_tcp*e_tcp||^2_W
  + lambda*||delta_q||^2
  + w_continuity*||q + delta_q - q_ik_otg_previous||^2
```

subject to：

```text
q_min + margin <= q + delta_q <= q_max - margin
-trust_region <= delta_q <= trust_region
```

Stage 1 以 TCP pose 为唯一主目标，必须优先达到位置和姿态容差。

### Stage 2：TCP 零空间内恢复 SPARK 臂型

Stage 2 在 Stage 1 解附近优化：

```text
J_tcp*delta_q_null = 0
```

软目标包括：

```text
upper-arm direction -> SPARK upper direction
forearm direction   -> SPARK forearm direction
elbow position      -> scaled SPARK elbow target
q                    -> q_ik_otg_previous
```

同时保留：

```text
joint position envelope
left/right J1 one-sided envelope
left/right J3 one-sided envelope
upper-arm outward-only envelope
branch continuity
```

如果严格 `J_tcp*delta_q_null=0` 因数值秩或关节边界不可行，则 Stage 2 放弃该步并返回 Stage 1 解，不得牺牲 TCP 主任务。

最终要求：

```text
FK(q_ik_otg).pose ~= T_palm_otg
```

## Velocity QP 输入

Cartesian Servo 使用同一 OTG reference：

```text
V_des = V_palm_otg + Kp_adaptive(e)*e
```

关节软参考使用同一时刻求出的 `q_ik_otg`：

```text
qdot_posture = clamp(
    Kq*(q_ik_otg-q_model),
    joint_velocity_limits)
```

Velocity QP 的决策变量、task scaling、slack、关节位置/速度/加速度/jerk/制动和 outward-only 约束保持不变。

## 连续性与失败策略

- `q_ik_otg` 以上一周期解热启动。
- 单周期 `q_ik_otg` 变化受 trust region 和 joint envelope 限制。
- OTG reference 无效时保持上一有效 reference。
- Stage 1 失败时保持上一 `q_ik_otg`，并让现有 reference/watchdog 决定保持或降速。
- Stage 2 失败时继续使用 Stage 1 解。
- PICO stale 时停止更新 raw target，现有 OTG 完成受约束停稳。
- 新模式不能回退到另一 IK 分支或直接提交 raw `q_ik`。

## 代码隔离

- 新增 `IkAlgorithm::kSparkUpperQpoasesCartesianOtgVelocityQp`。
- 新模式单独命名、单独 CLI 解析、单独 telemetry algorithm 字段。
- 现有 `kSparkUpperQpoasesVelocityQp` 继续绕过 OTG，作为历史基线。
- 尽量复用 `CartesianOtgController`、`SparkUpperQpoasesIk`、Velocity QP 和 telemetry，不复制求解器。
- OTG 一致 IK 的接口应显式接收 `T_palm_otg`，避免通过全局状态隐式覆盖 raw SPARK target。

## A/B 验证

固定输入：

```text
/home/zj/current_robotics/TJ_arm/vr_data/converted_inputs/tjvr/
pico_fast_motion_20260812_205428_v4.tjvr
```

比较：

```text
A: spark_upper_qpoases_velocity_qp
B: spark_upper_qpoases_cartesian_otg_velocity_qp
```

两者必须保持相同模型、初始姿态、PICO 映射、SPARK 权重、Velocity QP 参数、关节硬限制和 model-state feedback。

记录并比较：

- TCP 位置/姿态误差 P50、P95、P99、最大值。
- 对目标轨迹做时间对齐前后的误差，区分形状误差和纯延迟。
- Cartesian reference pose/twist 连续性。
- `FK(q_ik_otg)` 与 `T_palm_otg` 的位置/姿态残差。
- `q/qdot/qddot/jerk` 的 P50、P95、P99、最大值。
- QP slack、task scaling、active bounds、solver failure。
- stale 后最后三秒 TCP 峰峰值。
- SPARK 肘部/臂角与机器人结果的偏差。

## 验收标准

- 新模式无 IK 分支翻转和关节跳变。
- `FK(q_ik_otg)` 对 OTG reference 的位置残差 P95 小于 2 mm。
- `FK(q_ik_otg)` 对 OTG reference 的姿态残差 P95 小于 0.02 rad。
- 关节位置、速度、加速度和 jerk 不突破现有硬限制。
- `control_failures=0`，无 deadline miss。
- stale 最后三秒 TCP 峰峰值不超过 1 mm。
- 相比当前模式，动态误差或平滑性至少一项显著改善；若 tracking 延迟增加，则必须同时报告时间对齐误差，不以单一未对齐误差下结论。
- 完整 CTest 通过。
