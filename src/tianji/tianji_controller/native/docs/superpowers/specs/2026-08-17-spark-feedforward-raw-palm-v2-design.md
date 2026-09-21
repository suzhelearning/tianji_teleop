# SPARK Feedforward Raw-Palm v2设计

## 目标

直接增强现有`spark_upper_qpoases_feedforward_velocity_qp`，不新增算法名、不修改Velocity-Level QP决策变量、不改变Relative Mapping和Cartesian Servo总体形式。目标是保留`SPARK + Velocity QP`对最新末端目标的低延迟反馈路径，同时增加同源Cartesian速度前馈；两阶段位置IK的噪声只能影响低权重姿态参考，不能替换或拖慢Cartesian主目标。

## 方案选择

采用以下方案：

```text
latest valid SPARK palm pose
    + timestamp-aware SE(3) palm twist
    + jerk-limited q_ik posture reference
                         -> Velocity-Level QP
```

拒绝两个替代方案：

1. 不使用`(q_ik[k]-q_ik[k-1])/dt`直接生成前馈，因为位置IK噪声和分支变化会放大为速度、加速度和jerk尖峰。
2. 不使用`FK(q_ff)`作为Cartesian pose target，因为关节参考的Alpha-Beta、加速度和jerk平滑会把延迟传入末端主任务。

## 数据流

```text
PICO corrected skeleton
        |
        v
SPARK skeleton scaling / retarget
        |
        +---- latest valid palm pose --------------------+
        |                                                |
        +---- source timestamp + palm history            |
        |          |                                     |
        |          v                                     |
        |     robust SE(3) twist estimator               |
        |          |                                     |
        |          v                                     |
        |     bounded Cartesian feedforward              |
        |                                                |
        +---- two-stage qpOASES q_ik                     |
                   |                                     |
                   v                                     |
             branch/jump validation                      |
                   |                                     |
                   v                                     |
             jerk-limited joint reference                |
                   |                                     |
                   v                                     |
             low-weight posture task                     |
                                                        v
V_des = Kp * Log(T_spark * T_model^-1) + Kff * V_spark
                         |
                         v
                  Velocity-Level QP
                         |
                         v
                 qdot -> reference integrator
```

## Cartesian主参考

每个有效PICO源帧更新最新SPARK palm：

```text
T_target = T_spark_palm_latest
```

200 Hz控制中间周期保持最新目标。不得将主目标替换为：

```text
FK(q_ff)
```

Cartesian反馈使用模型参考状态，不使用真机反馈闭环：

```text
e = Log(T_target * T_model^-1)
```

## 同源SE(3)速度前馈

只在新源帧到达时，用源时间戳计算：

```text
dt_source = timestamp[k] - timestamp[k-1]
v_raw = (p[k] - p[k-1]) / dt_source
w_raw = Log(R[k] * R[k-1]^T) / dt_source
```

要求：

- 重复sequence、非单调timestamp和超出median-dt范围的帧不更新速度。
- epoch切换、stream discontinuity和目标跳变时清零速度并重新同步。
- 速度估计使用有限带宽滤波与置信度门控；停止和反向时衰减旧趋势。
- 200 Hz中间周期保持最近速度，可做不超过12 ms的速度保持，不预测新的pose target。
- 线速度和角速度分别受现有Cartesian上限约束。

最终主任务：

```text
V_des = Kp * e + activation * Kff * V_spark_filtered
```

## q_ik姿态参考隔离

两阶段qpOASES输出的`q_ik`保留，但不对原始序列求导。现有`SparkFeedforwardReference7`继续负责角度unwrap、异常跳变限幅、加速度和jerk受限的连续关节参考。它的输出只用于：

```text
min w_posture * ||qdot - qdot_posture||^2
```

并满足：

```text
w_cartesian >> w_posture
```

禁止通过下式重复注入Cartesian主任务：

```text
V_des += J * qdot_posture
```

`q_ik`拒绝、超时或Ruckig参考无效时，保持或平滑释放姿态任务；只要SPARK palm仍有效，Cartesian主任务继续运行。

## 状态与故障处理

- `Hold -> Active`：主pose同步到当前有效SPARK palm；前馈从零attack；姿态参考从当前模型关节状态启动。
- `Active`：pose每源帧直接更新，twist按时间戳更新，姿态参考独立连续化。
- `Active -> Stopping`：Cartesian前馈快速平滑衰减，pose保持最后有效目标，姿态参考按现有release停止。
- epoch/reset：清空pose twist历史、前馈激活和q_ik连续化历史，防止旧速度跨epoch传播。
- 非法palm pose：本周期不提交新目标，不污染历史。

## 代码边界

预期只修改：

- `include/tianji_qp_ik/spark_guidance.hpp`
- `src/spark_guidance.cpp`
- 新增独立的SPARK palm twist estimator头文件、实现和单元测试，或复用边界清晰的现有估计模块。
- `tests/test_spark_guidance.cpp`
- 必要的telemetry字段和验证文档。

不修改：

- Hierarchical/Velocity QP决策变量和硬约束。
- `SparkFeedforwardReference7`的q/qdot/qddot/jerk边界语义。
- 其他SPARK算法模式。
- 真机SDK和通信链路。

## 测试与验收

单元测试必须覆盖：

1. 静态`q_ik`抖动不会改变Cartesian pose target。
2. palm以恒速移动时，输出twist方向、量级和时间戳一致。
3. source掉帧、重复帧、epoch切换和反向运动不会生成速度尖峰。
4. q_ik姿态参考不再改变Cartesian target pose，也不重复注入`J*qdot_posture`。
5. q/qdot/qddot/jerk硬边界保持通过。

离线验证使用相同2134帧快速轨迹和Cartesian FRF矩阵。相对当前Feedforward版本要求：

- 快速轨迹位置延迟不高于99 ms，且尽量接近或优于SPARK + Velocity QP的110 ms。
- 小信号1--5 Hz群时延明显低于当前约54 ms，目标不高于30 ms。
- 低频增益保持0.98--1.02，共振峰不超过+0.5 dB。
- 控制失败和硬边界违规为0。
- 若跟手性与平滑性不能同时改善，保留原始数据并明确报告取舍，不覆盖历史基线。
