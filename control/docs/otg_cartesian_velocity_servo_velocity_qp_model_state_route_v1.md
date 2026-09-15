# OTG + Cartesian Velocity Servo + Velocity QP 技术路线（模型状态闭环）

版本：v1  
日期：2026-08-16  
状态：已实现并通过固定 PICO 快动记录回放验证

## 1. 目标与边界

本文定义 PICO VR 遥操作天机双臂时的速度级 IK 主线：

```text
PICO 目标位姿
    -> Cartesian OTG
    -> Cartesian Velocity Servo
    -> Velocity-Level QP
    -> 关节参考积分
    -> 天机位置接口
```

核心原则：

1. Cartesian OTG 负责生成连续、有限速的笛卡尔参考轨迹。
2. Cartesian Velocity Servo 负责形成笛卡尔闭环速度命令。
3. Velocity-Level QP 负责把末端速度分配为满足约束的关节速度。
4. 控制计算统一使用内部模型状态 `q_model`，不使用真实机械臂反馈修正正常运行中的控制状态。
5. PICO corrected IK 骨架仅用于生成臂角方向参考；末端主任务仍使用 PICO 手柄位姿。
6. `pico_outward` 同时提供臂角软任务和禁止上臂内收的硬约束。
7. 保持输出为关节位置参考，不在本阶段改为真机速度或力矩接口。

## 2. 总体数据流

```text
PICO corrected pose + corrected upper-body skeleton
                         |
                         v
              Timestamp / Validity Gate
                         |
                         v
                 Relative Mapping
              (clutch + motion scaling)
                         |
                         v
                 Cartesian Target
              T_target, V_target_est
                         |
                         v
                  Cartesian OTG
       T_otg_ref, V_otg_ref, A_otg_ref
                         |
                         |          q_model
                         |             |
                         |             v
                         |      FK(q_model), J(q_model)
                         |             |
                         +-------------+
                         |
                         v
            Cartesian Velocity Servo
          V_des = V_otg_ref + Kp * e_pose
                         |
                         v
                Velocity-Level QP
       Cartesian task scaling + joint hard bounds
       + weighted arm-angle task
       + upper-arm outward hard constraint
       + observable elbow-branch hard constraint
                         |
                         v
                      qdot
                         |
                         v
       q_model[k+1] = q_model[k] + dt*qdot[k]
                         |
                         v
                Tianji Position Interface
                         |
                         v
                       Robot

真实反馈 q_actual（如果接入）
    -> 只读显示、日志、独立安全系统和性能评估
    -> 不进入 OTG、Servo、QP、积分器或自动重同步
```

## 3. 状态定义

系统必须区分以下状态：

```text
T_target
    PICO相对映射得到的最终目标位姿。

T_otg_ref, V_otg_ref, A_otg_ref
    Cartesian OTG生成的连续参考位姿、速度和加速度。

q_model
    控制器内部维护的机器人关节参考状态，也是运动学模型状态。

T_model = FK(q_model)
J_model = J(q_model)
    Cartesian Servo和Velocity QP共同使用的模型位姿与Jacobian。

qdot_model
    Velocity QP求得的关节速度。

q_actual
    真机或动力学仿真的实际关节反馈；不参与正常控制计算。
```

同一个控制周期内，Cartesian Servo、Jacobian、关节约束和参考积分必须使用同一个 `q_model`，不能混用 `q_actual`。

## 4. PICO 输入与 Relative Mapping

输入包括：

```text
手柄位姿：
    p_pico, R_pico

corrected IK骨架：
    p_shoulder, p_elbow, p_wrist

数据质量：
    timestamp, sequence, validity, stale
```

相对位置映射：

```text
p_target = p_robot_anchor
         + S_position * R_map
         * (p_pico - p_pico_anchor)
```

相对姿态映射：

```text
R_target = R_robot_anchor * R_relative_mapped
```

继续保留已有的 clutch、左右臂坐标变换、运动缩放和工作空间映射，不在本路线中修改 Relative Mapping 的总体形式。

## 5. Cartesian OTG

### 5.1 职责

Cartesian OTG回答：

```text
“从当前参考状态到新的PICO目标，参考轨迹应该怎样连续运动？”
```

它不是机器人末端反馈控制器，也不负责消除机器人相对参考轨迹的跟踪误差。

### 5.2 输入输出

输入：

```text
T_target
V_target_estimated
target_stale
dt
```

内部状态和输出：

```text
T_otg_ref = (p_ref, R_ref)
V_otg_ref = (v_ref, w_ref)
A_otg_ref = (a_ref, alpha_ref)
```

OTG对平移和旋转参考施加速度、加速度和jerk限制：

```text
||v_ref|| <= v_max
||a_ref|| <= a_max
||j_ref|| <= j_max

||w_ref|| <= w_max
||alpha_ref|| <= alpha_max
||angular_jerk_ref|| <= angular_jerk_max
```

### 5.3 当前速度级配置基线

```text
控制频率：200 Hz

平移：
    velocity     = 3.0 m/s
    acceleration = 12.0 m/s^2
    jerk         = 120.0 m/s^3

旋转：
    velocity     = 12.0 rad/s
    acceleration = 40.0 rad/s^2
    jerk         = 400.0 rad/s^3

短时预测上限：15 ms
```

这些参数是仿真和算法上限测试基线，不自动代表真机安全参数。

## 6. Cartesian Velocity Servo

### 6.1 必要性

OTG只生成参考轨迹。如果直接把 `V_otg_ref` 送给Velocity QP，系统是开环速度跟踪：初始位姿误差、QP受限造成的落后量以及离散积分误差都不会被主动消除。

Cartesian Velocity Servo回答：

```text
“内部机器人模型应该以多大的末端速度追上OTG参考？”
```

### 6.2 模型反馈

反馈状态固定为：

```text
T_model = FK(q_model)
```

禁止在正常控制过程中改为：

```text
T_actual = FK(q_actual)
```

### 6.3 控制律

位置误差：

```text
e_p = p_otg_ref - p_model
```

姿态误差：

```text
e_R = LogSO3(R_otg_ref * R_model^T)
```

期望笛卡尔速度：

```text
v_des = v_otg_ref + Kp_position .* e_p
w_des = w_otg_ref + Kp_orientation .* e_R
```

限幅：

```text
||v_des|| <= max_linear_velocity
||w_des|| <= max_angular_velocity
```

当前基线：

```text
Kp_position    = [15, 15, 15]
Kp_orientation = [10, 10, 10]

max_linear_velocity  = 3.0 m/s
max_angular_velocity = 12.0 rad/s
```

Servo只是计算QP右端项，不增加第二次优化，其计算开销相对运动学和QP求解可忽略。

## 7. Velocity-Level QP

### 7.1 决策变量

```text
x = [qdot(7), slack_position(3), slack_orientation(3), beta_p, beta_R]
x in R^15
```

运动决策变量仍然是 `qdot`，没有改成 acceleration-level QP。`beta_p` 与
`beta_R` 只是位置和姿态任务的无量纲幅值缩放变量。

### 7.2 主任务等式

```text
J_position(q_model) * qdot + slack_position
    = beta_p * V_des_position

J_orientation(q_model) * qdot + slack_orientation
    = beta_R * V_des_orientation

0.75 <= beta_p <= 1
0.75 <= beta_R <= 1
```

其中：

```text
V_des = [v_des, w_des]
```

### 7.3 目标函数

忽略与决策变量无关的常数项后：

```text
min

0.5 * lambda_reg * ||qdot||^2

+ 0.5 * posture_weight
      * ||qdot - qdot_nominal||^2

+ 0.5 * continuity_weight
      * ||qdot - qdot_previous||^2

+ 0.5 * jerk_weight
      * ||qdot - (qdot_previous + qddot_previous*dt)||^2

+ 0.5 * slack_weight_position
      * ||slack_position / position_scale||^2

+ 0.5 * slack_weight_orientation
      * ||slack_orientation / orientation_scale||^2

+ 0.5 * task_scaling_weight_position * (beta_p - 1)^2

+ 0.5 * task_scaling_weight_orientation * (beta_R - 1)^2

+ 0.5 * arm_angle_weight
      * activation * weight_scale
      * (J_arm*qdot - arm_rate_target)^2
```

当完整笛卡尔速度与关节动态边界、禁止内收和臂角目标冲突时，QP优先降低
位置/姿态速度幅值，而不是任意改变各自的三维方向。Cartesian slack仍保留，
用于保证不可达或奇异情况下问题可解。

名义关节速度：

```text
q_mid = 0.5 * (q_min + q_max)
qdot_nominal = -nominal_gain * (q_model - q_mid)
```

当前基线权重：

```text
lambda_reg               = 1e-4
posture_weight           = 2e-2
continuity_weight        = 1e-3
jerk_weight              = 2e-2
nominal_gain             = 2.0
arm_angle_weight         = 10.0
slack_weight_position    = 3e4
slack_weight_orientation = 3e4
position_scale           = 1.0
orientation_scale        = 2.0
task_scaling_min_p       = 0.75
task_scaling_min_R       = 0.75
task_scaling_weight_p    = 8000
task_scaling_weight_R    = 3000
```

### 7.4 关节速度可行域

每个关节的最终速度上下界由以下约束求交：

```text
qdot_lower = max(
    velocity_lower,
    one_step_position_lower,
    acceleration_lower,
    braking_lower
)

qdot_upper = min(
    velocity_upper,
    one_step_position_upper,
    acceleration_upper,
    braking_upper
)
```

一步位置约束：

```text
qdot_position_lower = (q_min + margin - q_model) / dt
qdot_position_upper = (q_max - margin - q_model) / dt
```

相邻周期加速度约束：

```text
qdot_acc_lower = qdot_previous - qddot_max * dt
qdot_acc_upper = qdot_previous + qddot_max * dt
```

制动约束近似：

```text
qdot_brake_upper ~= sqrt(2 * a_brake * distance_upper)
qdot_brake_lower ~= -sqrt(2 * a_brake * distance_lower)
```

当前Velocity QP没有关节jerk硬约束。Cartesian OTG的笛卡尔jerk约束不能等价替代关节jerk约束，因此 `q/qdot/qddot/jerk` 必须继续记录和评估。

## 8. pico_outward 臂角路线

`pico_outward`由两个互补部分组成：

```text
PICO骨架臂角软参考
    +
禁止上臂向身体内侧运动的硬约束
```

### 8.1 PICO臂角参考

人体方向：

```text
u_human = normalize(p_elbow - p_shoulder)
f_human = normalize(p_wrist - p_elbow)
```

使用机器人骨长形成肩腕轴线近似：

```text
a_des = normalize(
    L_upper_robot * R_map * u_human
    + L_forearm_robot * R_map * f_human
)
```

期望肘平面方向：

```text
r_des = normalize(
    R_map * u_human
    - a_des * dot(a_des, R_map * u_human)
)
```

机械臂当前肩腕轴和肘平面方向：

```text
a_current = normalize(p_wrist_robot - p_shoulder_robot)

r_current = normalize(
    p_elbow_robot - p_shoulder_robot
    - a_current
      * dot(a_current, p_elbow_robot - p_shoulder_robot)
)
```

臂角误差：

```text
e_arm = atan2(
    dot(a_current, cross(r_current, r_des)),
    dot(r_current, r_des)
)
```

接近手臂伸直、肘平面投影不可观测时，必须降低任务激活度并保持上一次可靠方向，禁止直接使用噪声方向翻转臂角分支。

### 8.2 同QP臂角任务与分支保护

弱臂角速度目标：

```text
arm_rate_target = clamp(
    continuity_kp_velocity * e_arm,
    -continuity_max_velocity,
    +continuity_max_velocity
)
```

启用 task scaling 的 PICO 速度主线把臂角目标直接放入同一个 QP：

```text
cost_arm = 0.5 * w_arm * activation * weight_scale
         * (J_arm*qdot - arm_rate_target)^2
```

因此臂角、笛卡尔幅值缩放、slack、关节动态边界和两条安全约束由一次
qpOASES求解共同决定，不再在QP后修改 `qdot`。task scaling关闭的旧配置仍保留
严格Cartesian零空间臂角路径，以保证旧的13变量行为语义不变。

肩腕轴接近直线时，普通带符号臂角不可观测。当前PICO配置区分两个半径：

```text
minimum_radius_m    = 0.015 m  # 几何任务失效阈值
branch_lock_radius_m = 0.040 m # 分支保护目标
full_weight_radius_m = 0.050 m # 分支约束开始激活
```

在半径低于 `full_weight_radius_m` 时，保存最后可靠的肘平面方向 `r_last`，
并加入有限距离分支约束：

```text
d_branch = dot(r_last, radial_elbow) - branch_lock_radius_m
J_branch*qdot >= -K_branch*d_branch
```

这样不会等到1.5 cm几何退化点才采取动作。若恢复要求超出当前关节速度盒，
约束保留5%内部可行域余量，避免把qpOASES钉死在七个关节边界的唯一顶点。

### 8.3 禁止内收硬约束

左右臂外侧方向：

```text
left_outward  = [0, +1, 0]
right_outward = [0, -1, 0]
```

上臂外向距离：

```text
d_out = outward^T * (p_elbow - p_shoulder) - d_min
```

速度Jacobian：

```text
J_out = outward^T * (J_elbow - J_shoulder)
```

Velocity QP硬约束：

```text
J_out * qdot >= max(
    -K_out * d_out,
    -d_out / dt
)
```

第一项是平滑速度barrier，第二项保证下一参考周期不跨过肩部内侧平面。如果该约束与关节速度盒冲突，应保持当前参考，不允许以跨越内收边界换取QP可行。

## 9. 参考积分与位置接口

QP输出：

```text
qdot_model[k]
```

模型状态积分：

```text
q_model[k+1] = q_model[k] + dt * qdot_model[k]
```

发送给位置接口：

```text
q_command[k+1] = q_model[k+1]
```

用于可视化和日志的期望导数：

```text
qddot_model[k] =
    (qdot_model[k] - qdot_model[k-1]) / dt

jerk_model[k] =
    (qddot_model[k] - qddot_model[k-1]) / dt
```

正常运行时禁止执行：

```text
q_model = q_actual
```

因为自动重同步会在参考状态中产生不连续。纯模型测试模式从配置的安全初始关节角初始化；如果真机接入需要一次性对齐，必须只在控制器未激活时完成，激活后不再用反馈修正模型。

## 10. 真实反馈的严格边界

真实反馈不允许进入：

```text
Cartesian OTG状态
Cartesian Velocity Servo误差
QP的FK或Jacobian
QP关节约束状态
q_model积分器
运行中的自动同步
```

真实反馈可以作为控制链外只读信息用于：

```text
UI显示
日志记录
离线跟踪性能比较
驱动器状态和故障信息
独立急停与硬件安全系统
```

这些只读信息不得改变 `T_otg_ref`、`V_des`、QP约束或 `q_model`。

采用该策略后，底层驱动器必须独立承担实际关节限位、超速、跟踪失败、通信超时和急停保护。模型状态闭环不能替代硬件安全链。

## 11. 控制周期伪代码

```text
initialize:
    q_model = configured_safe_initial_q
    qdot_previous = 0
    qddot_previous = 0
    OTG.reset(FK(q_model))

loop at 200 Hz:
    pico_sample = receivePicoSample()

    target = relativeMapping(
        pico_sample.pose,
        clutch_state,
        motion_scaling
    )

    arm_direction = computePicoArmDirection(
        pico_sample.shoulder,
        pico_sample.elbow,
        pico_sample.wrist
    )

    otg_ref = cartesian_otg.update(
        target.pose,
        target.twist_estimate,
        target.stale,
        dt
    )

    model = kinematics(q_model)
    T_model = model.tcp_pose
    J_model = model.tcp_jacobian

    e_position = otg_ref.position - T_model.position
    e_orientation = LogSO3(
        otg_ref.rotation * T_model.rotation^T
    )

    V_des.linear = clampNorm(
        otg_ref.linear_velocity
        + Kp_position .* e_position,
        max_linear_velocity
    )

    V_des.angular = clampNorm(
        otg_ref.angular_velocity
        + Kp_orientation .* e_orientation,
        max_angular_velocity
    )

    velocity_bounds = computeVelocityBounds(
        q_model,
        qdot_previous,
        joint_limits,
        dt
    )

    outward_constraint = computeOutwardConstraint(
        model.shoulder,
        model.elbow,
        model.shoulder_jacobian,
        model.elbow_jacobian,
        dt
    )

    arm_angle_task = computeArmAngleTask(
        model,
        arm_direction
    )

    branch_constraint = computeArmBranchConstraint(
        arm_angle_task,
        velocity_bounds
    )

    qp_result = solveVelocityQp(
        J_model,
        V_des,
        velocity_bounds,
        outward_constraint,
        branch_constraint,
        arm_angle_task,
        qdot_previous
    )

    qdot_model = qp_result.qdot
    beta_p = qp_result.beta_position
    beta_R = qp_result.beta_orientation

    validate(
        qdot_model,
        velocity_bounds,
        outward_constraint,
        branch_constraint,
        J_model*qdot_model + slack
            - [beta_p*V_des_position,
               beta_R*V_des_orientation]
    )

    if accepted:
        qddot_model = (qdot_model - qdot_previous) / dt
        jerk_model = (qddot_model - qddot_previous) / dt
        q_model = q_model + dt * qdot_model
        sendPositionCommand(q_model)
        qdot_previous = qdot_model
        qddot_previous = qddot_model
    else:
        holdModelReference()

    logModelReferenceAndOptionalActualFeedback()
```

## 12. 异常处理

### PICO超时或无效

```text
停止推进PICO目标；
OTG平滑减速到保持；
模型参考保持连续；
不使用真实反馈重置q_model。
```

### QP不可行或求解失败

```text
拒绝当前候选；
保持上一q_model；
清理或安全重建QP warm start；
记录失败原因和约束激活状态。
```

### outward一步可行性无法满足

```text
保持对应机械臂模型参考；
禁止跨过肩部内侧平面；
不通过放松outward硬约束继续积分。
```

### 模型参考达到关节边界

```text
由位置、速度、加速度和制动边界收紧qdot可行域；
允许笛卡尔slack增加；
禁止积分越过安全关节边界。
```

## 13. 测试与验收路线

第一阶段只验证模型输出，不考虑真机伺服性能。

### 13.1 确定性笛卡尔轨迹

对比：

```text
Direct Velocity QP
OTG + Velocity QP
OTG + Acceleration QP（仅作为历史基线）
```

记录：

```text
末端位置RMS/P95/P99/最大误差
姿态RMS/P95/P99/最大误差
时间延迟
QP耗时、失败和重置次数
slack
active bounds
outward最小距离
```

### 13.2 PICO快速运动离线回放

使用统一 `vr_data` 中的快速运动记录，比较：

```text
PICO目标轨迹
OTG参考轨迹
模型末端轨迹
模型关节q/qdot/qddot/jerk
PICO骨架臂角与机器人臂角
```

### 13.3 实时PICO + MuJoCo Viewer

检查：

```text
快速大范围移动的跟手性
到位后的末端震荡
手柄静止时的关节速度和加速度波动
手超过肩高时的肘部分支稳定性
左右臂禁止内收
目标超时与重新接入行为
```

### 13.4 关节连续性

必须同时绘制：

```text
q
qdot
qddot
jerk
```

并叠加：

```text
位置上下界
速度上下界
有效加速度上下界
QP active-bound时刻
启动、停止和QP失败时刻
```

注意：Velocity QP当前没有关节jerk硬约束，因此jerk统计属于必要验收项，不能仅检查OTG输出的笛卡尔jerk。

## 14. 当前实测基线

固定输入：

```text
vr_data/converted_inputs/tjvr/pico_fast_motion_20260812_205428.tjvr
PICO live控制周期：5867
控制：200 Hz，model-state-only，velocity，pico_outward
```

最终候选输出：

```text
                              左臂              右臂
末端位置参考跟踪 RMS          0.1066 m          0.0815 m
末端位置参考跟踪 P95          0.2619 m          0.1646 m
末端姿态参考跟踪 RMS          0.1465 rad        0.1306 rad
末端姿态参考跟踪 P95          0.3536 rad        0.3031 rad
臂角误差 RMS                  0.5620 rad        0.3399 rad
臂角误差 P95                  1.5743 rad        0.6987 rad
最小肘平面半径                0.0349 m          0.0910 m
平均 beta_position            0.8733            0.9119
平均 beta_orientation         0.8765            0.9044
```

```text
QP/control failure：0 / 5867
控制周期P99：约0.67 ms（普通Linux离线回放，不作为硬实时保证）
```

同一记录的历史OTG + Acceleration QP位置RMS约为左0.1220 m、右0.0988 m，
姿态RMS约为左0.1569 rad、右0.1472 rad。因此最终速度级候选在这条快动记录上
保持了更低的末端参考跟踪误差，同时臂角RMS处于同一量级。

确定性圆轨迹仍用于毫米级稳态精度回归；PICO快动记录用于暴露高位姿态、
近直臂分支、输入重同步和动态边界同时激活的问题，两者不能互相替代。

## 15. 实施原则

1. 不推翻现有 Cartesian OTG、Velocity Servo 和 Velocity QP 模块边界。
2. 先审计当前代码是否已全部使用 `q_model/q_ref`，只做最小必要修改。
3. 为模型状态模式提供明确配置和遥测字段，避免依赖隐含行为。
4. 测试通过前不提前提交实验性控制修改。
5. 测试CSV、图片和大体积回放结果默认不提交，只提交必要汇总文档。
6. 本路线验证完成后，再决定是否增加关节jerk硬约束或独立安全过滤QP。

## 16. 结论

### 16.1 同配置A/B测试指令

为避免复制配置后参数漂移，模型状态模式与旧模式使用同一个配置文件，仅切换命令行状态源。

模型状态闭环：

```bash
OMP_WAIT_POLICY=ACTIVE \
OMP_PROC_BIND=close \
OMP_PLACES=cores \
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_pico_fast.xml \
  --pico-teleop \
  --pico-bind 127.0.0.1 \
  --pico-port 15000 \
  --control-level velocity \
  --arm-angle-mode pico_outward \
  --model-state-only \
  --telemetry benchmark_results/pico_otg_velocity_model_state_live.csv
```

启动时必须输出：

```text
control_state_source=model_reference
```

旧的实际反馈保护模式：

```bash
OMP_WAIT_POLICY=ACTIVE \
OMP_PROC_BIND=close \
OMP_PLACES=cores \
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_pico_fast.xml \
  --pico-teleop \
  --pico-bind 127.0.0.1 \
  --pico-port 15000 \
  --control-level velocity \
  --arm-angle-mode pico_outward \
  --actual-feedback-control \
  --telemetry benchmark_results/pico_otg_velocity_actual_guarded_live.csv
```

启动时必须输出：

```text
control_state_source=actual_feedback_guarded
```

两条命令除了状态源开关和遥测输出文件名外完全相同，因此适合直接比较跟踪误差、QP输出以及 `q/qdot/qddot/jerk`。

### 16.2 最终主线

最终主线固定为：

```text
PICO corrected pose/skeleton
    -> Relative Mapping
    -> Cartesian OTG
    -> Cartesian Velocity Servo based on FK(q_model)
    -> 15-variable Velocity QP based on J(q_model)
       (qdot + Cartesian slack + beta_p/beta_R)
    -> weighted PICO arm-angle task in the same QP
    -> outward no-adduction + observable-branch hard constraints
    -> integrate q_model
    -> Tianji position command
```

其中：

```text
OTG负责参考轨迹连续性；
Cartesian Velocity Servo负责模型位姿闭环；
Velocity QP负责关节约束下的速度分配；
pico_outward负责人体臂角趋势、禁止内收和近直臂分支保护；
真实机械臂反馈不参与正常控制计算。
```
