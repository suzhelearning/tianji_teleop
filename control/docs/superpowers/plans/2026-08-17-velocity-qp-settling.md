# Velocity QP Settling 实施与验证记录

**目标：** 在不修改 Velocity QP 和关节运动硬限制的前提下，消除首次到位及 PICO stale 后的固定目标极限环，并尽量保留快速段响应。

## 最终实施范围

- [x] 在 `CartesianServoConfig` 增加可选的近端 Kp 和误差过渡区间。
- [x] 在 Cartesian reference servo 中使用 smoothstep 调度位置和姿态 Kp。
- [x] 仅在 `config/qp_ik_pico_teleop.yaml` 启用自适应 Kp。
- [x] 保持 Velocity QP 决策变量和目标结构不变。
- [x] 保持关节位置、速度、加速度、jerk、制动和 outward-only 约束不变。
- [x] 增加自适应增益近端、中间和远端单元测试。

## 已测试但未采用

- [x] 关闭 task scaling：固定目标振荡明显恶化，已恢复启用。
- [x] 全程 `Kd*(V_ref-J*qdot)`：使右臂也发生固定目标振荡，已从最终实现删除。
- [x] 固定低 Kp=4/3：能停稳，但快速段响应退化，未作为最终配置。
- [x] 窄位置调度区间 0.01--0.05 m：左臂仍会重新进入高增益极限环。
- [x] 宽位置调度区间 0.02--0.22 m：满足停稳和动态误差验收，作为最终配置。

没有实施 stale 强制锁存。自适应 Kp 已使 stale 后模型在原目标处自然受约束停稳，因此无需引入额外状态跳转或改变最后目标。

## 验证命令

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure -j1
```

结果：

```text
100% tests passed, 0 tests failed out of 65
Total Test time (real) = 55.28 sec
```

并行 CTest 曾出现两个资源/首采样时序相关失败；两个测试单独串行复测通过，完整串行测试也通过。产品回放验证使用固定输入和独立 UDP 端口执行，不依赖这些测试的并行调度。

## 回放验收

回放数据：

```text
/home/zj/current_robotics/TJ_arm/vr_data/converted_inputs/tjvr/
pico_fast_motion_20260812_205428_v4.tjvr
```

最终结果：

- [x] 左臂最后三秒 TCP 峰峰值由 68.45 mm 降至约 1e-7 mm。
- [x] 右臂最后三秒保持静止。
- [x] live 位置误差 P50 退化不超过 5%。
- [x] live 位置误差 P95 基本持平。
- [x] `qdot/qddot/jerk` 不超过原配置硬限制。
- [x] `control_failures=0`。
- [x] 无 deadline miss。

## 提交状态

实现基线已提交：

```text
d5e0f708de2a8500a8b007fd8123fd1f00c681be
feat: add SPARK-guided PICO velocity QP teleoperation
```

## 后续任务

本次仅解决固定目标停稳。动态阶段仍存在以下接口问题：

```text
SPARK palm pose ---------> Cartesian Velocity Servo（twist=0）
两阶段 qpOASES q_ik ----> 高权重关节速度参考
```

`spark_upper_qpoases_velocity_qp` 还会绕过 Cartesian OTG。后续应让 FK(`q_ik`) pose、连续 `q_ik` 导出的 Cartesian twist 和关节软参考来自同一条轨迹，再进行离线 A/B；不得把该问题与本次已验证的停稳修复混在同一提交中。
