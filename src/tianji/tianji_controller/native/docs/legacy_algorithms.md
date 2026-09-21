# 历史算法与回归入口

本文保存 TJ Arm MuJoCo QP-IK 工程的历史算法、对照基准和旧交互入口，仅用于回归、
A/B 对比和理解演进过程。当前 `main` 推荐的
`spark_upper_qpoases_headroom_feedforward_velocity_qp` 使用方法见
[根 README](../README.md)。本文命令均不是当前默认启动方式。

## 构建

```bash
pixi install
pixi run configure
pixi run build
pixi run test
```

## Hierarchical constrained QP

显式启动传统 hierarchical profile：

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_hierarchical.yaml \
  --model models/marvin_m6_qp_test.xml \
  --no-pico-teleop --no-pico-skeleton-overlay \
  --control-level velocity \
  --algorithm hierarchical_qp
```

无显示器验收：

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_hierarchical.yaml \
  --model models/marvin_m6_qp_test.xml \
  --no-pico-teleop --no-pico-skeleton-overlay \
  --headless --duration 4 \
  --telemetry /tmp/hierarchical_telemetry.csv
```

历史 Viewer 脚本覆盖手动目标、圆、8 字、纯姿态、组合轨迹、速度/加速度控制层切换、
DLS、暂停、恢复和名义位复位。

## Null-space DLS

零空间 DLS 基线使用阻尼伪逆和名义位零空间速度：

```text
J_pinv = J^T * inverse(J*J^T + damping^2*I)
N      = I - J_pinv*J

qdot_raw = J_pinv*Vd + N*qdot_nominal
qdot     = alpha*qdot_raw
```

`alpha` 是所有关节共享的 `[0,1]` 缩放，不做逐关节裁剪。该模式用于与 QP 比较任务
缺口、关节速度变化和限位行为，不是当前 PICO 遥操默认算法。

## Cartesian OTG velocity profile

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_cartesian_otg_velocity.yaml \
  --model models/marvin_m6_qp_test.xml \
  --no-pico-teleop --no-pico-skeleton-overlay \
  --control-level velocity \
  --algorithm hierarchical_qp
```

历史数据流：

```text
raw target pose/twist
→ translation Ruckig + orientation SO(3) jerk limiter
→ Cartesian reference (T_ref, V_ref, A_ref)
→ Vd = V_ref + Kp*Log(T_ref*T_measured^-1)
→ velocity-level constrained QP
```

## Cartesian OTG acceleration profile

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml \
  --no-pico-teleop --no-pico-skeleton-overlay \
  --control-level acceleration \
  --algorithm hierarchical_qp
```

历史加速度伺服：

```text
pose_error = Log(T_ref*T_measured^-1)
A_des = A_ref + Kd*(V_ref - J*qdot_measured) + Kp*pose_error
```

启用 task scaling 时，单臂决策变量为：

```text
x = [qddot(7), slack_acceleration(6), beta_position, beta_orientation]
```

## 历史 SPARK A/B 模式

这些模式仍可显式运行，但不作为推荐入口：

- `spark_guided_velocity_qp`：SPARK 两阶段 IK 只提供软姿态参考。
- `spark_direct_velocity_qp`：以 `q_ik-q_model` 直接形成软姿态速度。
- `spark_pose_velocity_qp`：只使用 SPARK 重建掌心位姿，不添加 SPARK 关节任务。
- `spark_upper_qpoases_direct`：直接采用 SPARK 两阶段位置 IK 输出。
- `spark_upper_qpoases_velocity_qp`：SPARK 关节参考进入 velocity QP。
- `spark_upper_qpoases_cartesian_otg_velocity_qp`：SPARK 与 Cartesian OTG 一致路径。
- `spark_upper_qpoases_feedforward_velocity_qp`：固定前馈，不做 Headroom 衰减。

示例：

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_pico_fast.xml \
  --pico-teleop --pico-skeleton-overlay \
  --pico-bind 127.0.0.1 --pico-port 15000 \
  --control-level velocity \
  --algorithm spark_guided_velocity_qp \
  --model-state-only
```

替换 `--algorithm` 即可在同一 PICO/TJVR 输入上进行 A/B 对比。

## 历史 Velocity QP 形式

启用 task scaling 时，每臂决策变量为：

```text
x = [qdot(7), slack(6), beta_position, beta_orientation]
```

任务等式：

```text
J_position*qdot + slack_position = beta_position*Vd_position
J_orientation*qdot + slack_orientation = beta_orientation*Vd_orientation
```

目标函数包含笛卡尔 slack、正则、名义姿态、连续性、jerk 趋势、任务缩放和臂角软
任务：

```text
minimize
    0.5*slack^T*W_slack*slack
  + 0.5*lambda_reg*||qdot||^2
  + 0.5*posture_weight*||qdot-qdot_nominal||^2
  + 0.5*continuity_weight*||qdot-qdot_prev||^2
  + 0.5*jerk_weight*||qdot-(qdot_prev+qddot_prev*dt)||^2
  + 0.5*w_beta_p*(beta_position-1)^2
  + 0.5*w_beta_R*(beta_orientation-1)^2
  + 0.5*w_arm*(J_arm*qdot-arm_rate_target)^2
```

硬边界取模型速度、一步位置、加速度和制动距离包络的交集：

```text
position_lower_i = (q_min_i + margin - q_i)/dt
position_upper_i = (q_max_i - margin - q_i)/dt

acceleration_lower_i = qdot_prev_i - qddot_max_i*dt
acceleration_upper_i = qdot_prev_i + qddot_max_i*dt

braking_upper_i =  sqrt(2*a_brake_i*distance_upper_i)
braking_lower_i = -sqrt(2*a_brake_i*distance_lower_i)
```

优先关系：

```text
硬关节与上臂安全
> task scaling / 高权重笛卡尔 slack
> 臂角、姿态偏好、连续性和正则
```

这是带软任务等式的单层约束 QP，不是严格词典序 HQP。

## 历史 Acceleration QP 形式

```text
J*qddot + slack_acceleration
  = [beta_position*A_des_position,
     beta_orientation*A_des_orientation] - Jdot*qdot
```

`Jdot*qdot` 使用中心方向差分。加速度硬边界为配置加速度、下一拍速度、二阶位置预测、
可选 jerk 和制动包络的交集：

```text
qdot_next = qdot_ref + qddot*dt
q_next    = q_ref + qdot_ref*dt + 0.5*qddot*dt^2
qdot_next^2 <= 2*a_brake*(q_safe_limit-q_next)
```

## 历史 Benchmark

Hierarchical QP 与 DLS：

```bash
pixi run ik-ab
# 或
./build/tianji_hierarchical_ik_benchmark \
  --config config/qp_ik_hierarchical.yaml \
  --model models/marvin_m6_qp_test.xml \
  --output /tmp/hierarchical_ik_ab.csv
```

Cartesian OTG velocity/acceleration：

```bash
./build/tianji_cartesian_otg_benchmark \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml \
  --steps 600 \
  --output /tmp/cartesian_otg_acceleration_abc.csv
```

旧 7 变量 OSQP/qpOASES 对照仍可通过 `pixi run benchmark` 运行。

## 完整旧 Viewer 操作

| 输入 | 功能 |
|---|---|
| `L` / `R` | 选择左/右目标 |
| 左键拖动 XYZ 箭头或旋转环 | 平移或旋转 marker |
| 左键拖动中心球 | 在相机视平面平移目标 |
| `W` | 切换 world/local marker 坐标系 |
| `0` 或 `M` | 手动模式 |
| `1` / `2` / `3` / `4` | 圆 / 8 字 / 纯姿态 / 组合轨迹 |
| `H` | 保持模式 |
| `Q` / `D` | hierarchical QP / null-space DLS |
| `V` / `A` | velocity / acceleration 控制层 |
| `P` | 启用或关闭 PICO 输入 |
| `G` | 循环切换臂角参考模式 |
| `F2` | 显示或隐藏关节曲线 |
| `F3` | 切换 `q`、`dq`、`ddq`、`jerk` |
| `F4` | 切换并锁定左/右臂曲线 |
| `F5` | 曲线恢复跟随当前选择臂 |
| `Space` | 暂停或恢复 |
| `N` | 回到名义位并重置引用 |
| `F1` / `Esc` | 显示帮助 / 退出 |

## 历史验证文档

- [Hierarchical QP IK](verification/hierarchical_qp_ik_results.md)
- [QP IK V1](verification/qp_ik_v1_results.md)
- [Cartesian OTG velocity QP](verification/cartesian_otg_velocity_qp_results.md)
- [Cartesian OTG acceleration QP](verification/cartesian_otg_acceleration_qp_results.md)
- [PICO MuJoCo teleoperation](verification/pico_mujoco_teleop_results.md)
- [SPARK feedforward velocity QP](verification/spark_feedforward_velocity_qp_results.md)
- [SPARK guided velocity QP](verification/spark_guided_velocity_qp_results.md)
- [SPARK velocity QP three-way](verification/spark_velocity_qp_three_way_results.md)

以上内容用于历史回归和对照。当前运行与回放请返回[根 README](../README.md)。
