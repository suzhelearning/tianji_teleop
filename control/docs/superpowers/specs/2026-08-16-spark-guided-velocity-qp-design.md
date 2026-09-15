# Spark-Guided Velocity QP 设计

## 目标

在不替换当前 Cartesian OTG、Cartesian Velocity Servo 和 Velocity-Level QP 主线的前提下，引入 `TJ_arm_control_DLS_IK` 已验证的 Spark 上肢缩放与位置级 IK，为七自由度冗余构型提供完整、连续、符合人体上肢形态的关节参考。

新模式命名为：

```text
spark_guided_velocity_qp
```

现有算法模式保持可用，便于使用相同输入做确定性 A/B 对比。

## 问题定义

当前 PICO 遥操固定使用 `outward_only` 后，QP 不再消费 PICO 标量臂角，因此消除了接近 `+/-pi` 时的臂角分支翻转。但 `outward_only` 只提供以下信息：

- 禁止上臂向身体内侧越界；
- 尽量维持当前臂平面；
- 没有指定完整的七关节人形构型。

在快速、大范围或接近工作空间边界的运动中，Cartesian 任务可以沿冗余流形把 J3、J5、J6、J7 推向硬限位。现有回放中已经观察到多关节同时达到 `4 rad/s`，并达到配置允许的 `60/90 rad/s^2`，因此机械臂会产生肉眼可见的乱动。该现象不是 QP 超过约束，而是 QP 在缺少稳定冗余目标时选择了不希望的可行解。

## 总体架构

```text
PICO corrected upper-limb skeleton + palm rotations
        |
        v
Timestamp / Validity / Atomic Dual-Arm Gate
        |
        v
Spark Robot-Length Skeleton Scaling
        |
        +-------------------------------+
        |                               |
        v                               v
Spark Position IK                  Spark Palm Pose
        |                               |
        v                               v
Joint Reference Ruckig             Cartesian OTG
        |                               |
        v                               v
q_spark_ref, qdot_spark_ref        Cartesian Velocity Servo
        |                               |
        +------------+                  v
                     |               V_des
                     v                  |
              Posture Task              |
                     +---------> Velocity-Level QP
                                      |
                                      v
                          qdot -> Reference Integrator
                                      |
                                      v
                                  MuJoCo/Tianji
```

Spark 负责选择连续的人形 IK 分支；Velocity QP 仍是最终运动求解器和硬约束执行层。

## 输入与 Spark 缩放

输入使用 TJVR v4 中 corrected IK 的左右肩、肘、腕、掌心关键点及各段旋转。每一帧必须是双臂原子帧，并通过现有时间戳、有限值、CRC、重排和跳变检查。

缩放规则复用 `TJ_arm_control_DLS_IK` 中的 `UpperSparkSkeletonScaler`：

1. 机器人肩点固定为 Tianji 模型肩点；
2. 人体肩到肘方向映射到机器人上臂方向，长度替换为机器人上臂长度；
3. 人体肘到腕方向映射到机器人前臂方向，长度替换为机器人前臂长度；
4. 腕到掌心使用机器人对应段长；
5. 掌心姿态沿用 corrected PICO 掌心姿态；
6. 关键点位置与段旋转冲突时，沿用 Spark 基线的旋转优先、位置方向回退和旋转跳变拒绝规则。

缩放后的掌心位置是本模式唯一的 Cartesian 位置目标。不得同时使用旧 Relative Mapping 的平移目标，否则肘、腕和掌心目标会在几何上互相冲突。现有 Relative Mapping 模式保留给其他算法，不删除。

## Spark 位置 IK

移植经过验证的 Pinocchio + 两阶段 qpOASES 位置 IK。每条手臂独立维护 Stage 1 和 Stage 2 热启动求解器。

Stage 1 任务：

```text
上臂方向
前臂方向
掌心姿态
```

Stage 2 在保留 Stage 1 任务的基础上增加：

```text
肘点位置
腕点位置
掌心位置
```

位置 IK 每周期从上一帧有效 `q_spark` 开始，使用关节安全范围、单次 trust region、LM 阻尼、回溯线搜索和总计算时间预算。候选必须改善掌心主误差，不能为了中间关键点降低而明显恶化末端目标。

IK 输出是绝对关节目标 `q_spark`，不直接写入 MuJoCo 或机器人接口。

## Spark 参考轨迹

`q_spark` 输入独立的七轴位置接口 Ruckig，生成：

```text
q_spark_ref
qdot_spark_ref
qddot_spark_ref
```

该 Ruckig 只平滑 QP 的姿态参考，不是最终命令输出。它使用 SDK 对齐的关节动态限制，并在初次接入、PICO epoch 变化、stream resynchronization 或 Hold 到 Active 时从当前模型状态重新同步。

姿态速度目标定义为：

```text
qdot_posture =
    qdot_spark_ref
    + K_posture * (q_spark_ref - q_model)
```

`qdot_posture` 按每关节速度上限裁剪；当 Spark 参考接近安全关节边界时，使用现有 interior margin 和关节中心恢复逻辑降低该任务激活度。

## Cartesian 参考

缩放后的 Spark 掌心目标进入现有 Cartesian OTG：

```text
Spark palm pose
    -> Cartesian OTG
    -> Cartesian Velocity Servo
    -> V_des
```

Cartesian Servo 的模型反馈仍使用 `q_model` 的 FK，不使用真机反馈关节位置闭合外层 IK。真机反馈只进入现有 Reference Tracking Watchdog，不改变本设计的模型状态求解语义。

## Velocity-Level QP

QP 决策变量保持当前结构，不改为加速度级：

```text
qdot
slack_position
slack_orientation
task scaling variables（若当前配置启用）
```

主任务保持：

```text
J_position * qdot + slack_position
    = beta_position * V_position

J_orientation * qdot + slack_orientation
    = beta_orientation * V_orientation
```

增加 Spark 姿态软目标：

```text
0.5 * spark_posture_weight
    * ||W_joint * (qdot - qdot_posture)||^2
```

任务关系为：

1. Cartesian 位置跟踪最高；
2. Cartesian 姿态跟踪次之；
3. Spark 完整关节构型用于冗余选支，并在 Cartesian 能力不足时让步；
4. 常规连续性和正则化保持最低层。

Spark 姿态目标不能作为等式硬约束，也不能覆盖 Cartesian 任务。`outward_only` 的上臂单边约束继续作为硬安全约束，防止 Spark 输入异常时产生内收。

## 硬约束

保持当前固定顺序：

```text
Joint Position
Joint Velocity
Joint Acceleration
Joint Jerk
Braking
Upper-Arm Outward
Collision（启用后）
```

本功能不修改现有速度、加速度、jerk 数值，也不通过放宽约束改善跟踪。

## 状态、重置和异常处理

### 初次接入与重同步

初次获得有效 Spark 骨架、PICO epoch 变化或 stream resynchronization 时：

1. 从当前 `q_model` 计算肩、肘、腕和 TCP 状态；
2. Spark 目标使用五次 smoothstep 渐入；
3. Spark IK 种子和关节 Ruckig 同步到当前模型状态；
4. Cartesian OTG 保持连续，不从测量反馈瞬移；
5. 渐入期间仍执行完整 QP 硬约束。

### 输入无效或 stale

- 短时无效：保持最后有效 Spark 几何目标和关节参考；
- 超过现有 PICO timeout：Cartesian 目标 Hold，Spark Ruckig 制动到零速度；
- 恢复时从当前模型状态重新渐入，不追赶过期轨迹。

### Spark IK 失败

- 保留上一帧有效 `q_spark`；
- Spark 姿态任务冻结并逐步降低激活度；
- Cartesian QP 可继续短时跟踪；
- 连续失败超过阈值时进入 Hold，不切换到另一 IK 分支。

### QP 失败

沿用当前有界回退和双臂事务策略。Spark 关节目标不能绕过 QP 直接提交。

## 模式隔离

新增独立模式 `spark_guided_velocity_qp`，不修改以下模式的行为：

```text
hierarchical_qp
nullspace_dls
spark_upper_qpoases
其他 Cartesian OTG velocity/acceleration 模式
```

Viewer、CLI、遥测和 benchmark 显式记录算法名称，避免测试结果混淆。PICO 骨架 overlay 继续显示缩放后的 Spark 骨架与原始 corrected 骨架，便于检查映射是否合理。

## 遥测

除现有 q/qdot/qddot/jerk、Cartesian 误差、QP bounds 和求解时间外，增加：

- Spark 输入有效性、数据年龄和重同步次数；
- 缩放后的肩、肘、腕、掌心位置；
- 上臂/前臂方向误差；
- Spark Stage 1/2 状态、迭代、回溯和耗时；
- `q_spark`、`q_spark_ref`、`qdot_spark_ref`；
- Spark posture task 激活度和每关节误差；
- 多关节同时速度饱和占比；
- 每关节贴近硬限位的持续时间；
- PICO reset 前后最大目标与关节参考变化。

## 测试与 A/B 验收

### 单元测试

- Spark 缩放后段长与机器人段长一致；
- 段旋转优先、位置回退和异常旋转拒绝；
- Pinocchio 关键点 FK/Jacobian 与有限差分及 MuJoCo 对齐；
- 两阶段 IK 保持掌心主误差改善；
- Spark 参考 Ruckig 不超过配置的速度、加速度和 jerk；
- Spark posture objective 的 Hessian/gradient 与定义一致；
- PICO stale、epoch reset 和恢复不会导致关节参考跳变；
- `outward_only` 硬约束仍生效。

### 确定性离线对比

同一条 PICO 快速移动数据分别运行：

1. 当前 `outward_only Velocity QP`；
2. `TJ_arm_control_DLS_IK` 的 Spark 基线；
3. 新 `spark_guided_velocity_qp`。

比较：

- 末端位置/姿态误差 P50、P95、P99、最大值；
- 多关节同时达到速度上限的时间占比；
- J3、J5、J6、J7 距离硬限位小于 `0.01 rad` 的时间占比；
- 每关节 q、qdot、qddot、jerk 及相邻周期变化；
- 肘高于肩、上臂内收和 IK 分支异常事件；
- QP/Spark solver 失败次数；
- 控制周期 P50、P95、P99 和 deadline miss。

### 首轮验收标准

- 不超过当前配置的关节位置、速度、加速度和 jerk 硬约束；
- 不发生上臂内收；
- 不发生持续的腕部贴限位或肘部翻支；
- 相比当前 `outward_only Velocity QP`，多关节速度饱和占比和腕部贴限位占比均明显下降；
- 末端跟踪误差不得劣于 Spark 基线，并记录与当前速度 QP 的差异；
- 双臂控制周期 P99 目标不超过 `5 ms`；若实时预算未满足，保持功能模式隔离，不降低安全约束。

## 实施边界

- 复用 DLS/Spark 工程已验证的实现，不重新定义 Spark 几何语义；
- 优先移植最小闭环，不同时加入碰撞 QP；
- 第一阶段只验证 MuJoCo 和离线回放，不涉及真机伺服；
- 不删除当前算法、配置或历史 benchmark；
- 不提交或推送，除非用户明确要求。
