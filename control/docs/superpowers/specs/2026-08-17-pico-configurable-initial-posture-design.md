# PICO 可配置初始姿态设计

## 目标

将 PICO MuJoCo 遥操作的双臂初始姿态设为接近用户给出的自然前伸姿态，减少 SPARK 首次接管时模型姿态与人体骨架之间的差值，从而降低启动阶段的饱和和抖动。

## 配置

只在 `qp_ik_pico_teleop.yaml` 中启用以下初始关节角，单位为 rad：

```text
left  = [ 1.10, -1.52, -1.52, -1.10, 0.0, 0.0, 0.0 ]
right = [-1.10, -1.52,  1.52, -1.10, 0.0, 0.0, 0.0 ]
```

两组角度均位于当前 MuJoCo/URDF 关节范围内。配置加载时必须校验数组长度、有限值及模型安全限位；无效配置必须报错，不能静默裁剪。

## 数据流

Viewer 启动或收到 `N` 重置命令时：

```text
读取配置初始姿态
  -> 设置 MuJoCo 双臂 q，qdot=0
  -> forward kinematics
  -> 重建 Cartesian 初始目标
  -> 同步 Velocity/Acceleration Controller 参考状态
  -> 重置 Cartesian OTG
  -> 重置 SPARK guidance 和两阶段位置 IK seed
```

PICO 数据开始后，现有 `0.8 s` joint-reference smooth attack 保持不变。

## 隔离范围

- 只修改 PICO profile 的默认值。
- `qp_ik_hierarchical.yaml`、圆轨迹 benchmark 和其他算法配置继续使用原来的关节中点。
- 不修改 URDF 零位、MuJoCo joint range、速度/加速度/jerk/制动限制。
- 不修改 Direct、Velocity QP 或 Acceleration QP 的数学形式。

## 接口

在控制器配置下增加可选项：

```yaml
controller:
  initial_posture_enabled: true
  initial_left_q_rad:  [ 1.10, -1.52, -1.52, -1.10, 0.0, 0.0, 0.0 ]
  initial_right_q_rad: [-1.10, -1.52,  1.52, -1.10, 0.0, 0.0, 0.0 ]
```

未启用或字段不存在时保持历史行为：使用每个关节安全范围的中点。

## 验收

1. PICO Viewer 启动后的左右关节位置与配置一致。
2. `N` 重置后恢复同一配置姿态，且 qdot/qddot 清零。
3. SPARK guidance 的初始 seed 与 MuJoCo/Controller q_ref 一致。
4. 配置姿态违反任一安全关节范围时启动失败并指出手臂和关节编号。
5. Direct 与 `spark_upper_qpoases_velocity_qp` 的现有集成测试通过。
6. 全量 CTest 通过，现有硬约束数值无变化。
