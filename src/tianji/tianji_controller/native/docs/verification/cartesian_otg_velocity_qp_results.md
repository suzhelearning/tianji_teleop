# Cartesian OTG + Velocity QP Verification

验证日期：2026-08-10

分支：`feature/cartesian-otg-velocity-qp-v1`

## 实现范围

- 200 Hz 双臂 Cartesian reference generation。
- XYZ 平移：Ruckig v0.19.4，位置接口，速度/加速度/jerk 限制。
- 姿态：世界系 SO(3) 最短旋转，角速度/角加速度/角 jerk 限制。
- timestamp-aware 原始目标 twist；OTG 路径不使用旧目标位姿预测。
- `Vd = V_ref + Kp*pose_error`，参考 twist 只叠加一次。
- 原 13 变量 velocity QP、加速度/制动/速度/位置硬边界、qpOASES hotstart、关节引用 watchdog 与单臂失败隔离均保留。
- CSV 与 overlay 包含 `v_ref/w_ref/a_ref/alpha_ref`、valid、stale。

## 完整验证

```text
pixi run configure: PASS
pixi run build: PASS
pixi run test: PASS, 26/26
```

4 秒 headless：

```text
sequence=799
accepted=1
cycle_p99_us=224.174
deadline_misses=0
control_failures=0
command_failures=0
completed_stage=10
snapshot_drops=0
telemetry_drops=0
```

命令：

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_cartesian_otg_velocity.yaml \
  --model models/marvin_m6_qp_test.xml \
  --headless --duration 4 \
  --telemetry /tmp/tianji_otg_velocity_headless.csv
```

## Direct / OTG A/B

每种控制器 11 个场景、每场景 600 步。两种控制器均为：

```text
QP failures: 0
hard-bound violations: 0
non-finite metric rows: 0
```

跨场景简单均值（仅用于回归观察，不代表场景加权性能结论）：

| controller | mean position RMS (m) | mean orientation RMS (rad) | mean qdot variation | max solve p99 (us) |
|---|---:|---:|---:|---:|
| direct_velocity_qp | 0.260087402 | 0.054391366 | 0.031589212 | 16.670 |
| otg_velocity_qp | 0.271959908 | 0.154312345 | 0.018888370 | 8.453 |

不可达场景的米级误差主导跨场景位置均值。OTG 的目标是约束参考导数并降低关节速度变化，不保证相对无平滑 direct 路径降低瞬时目标误差；结果符合该验收原则。

## 限制

MuJoCo 当前通过关节位置引用验证运动学一致性，并未建模真实执行器动力学。这里的 OTG 和 QP 参数不能直接作为实机安全认证值；实机部署仍需碰撞约束、执行器限制、通信 watchdog 和硬件急停链路。
