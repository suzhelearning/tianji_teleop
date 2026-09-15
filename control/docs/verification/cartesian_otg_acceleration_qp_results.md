# Cartesian OTG + Acceleration QP Verification

验证日期：2026-08-10

分支：`feature/cartesian-otg-acceleration-qp-v1`

## 实现范围

- 继承已验证的 200 Hz Cartesian OTG：平移 Ruckig v0.19.4，姿态为世界系 SO(3) jerk limiter。
- 笛卡尔加速度伺服：`A_des = A_ref + Kd*(V_ref-V_measured) + Kp*pose_error`。
- 每臂 13 变量 acceleration slack QP：`x=[qddot(7), slack(6)]`，等式为 `J*qddot+slack=A_des-Jdot*qdot`。
- `Jdot*qdot` 由 MuJoCo Jacobian 的中心方向差分获得，差分计算使用独立 scratch state，不污染控制状态。
- `q_ref/qdot_ref` 二阶积分，硬约束覆盖加速度、预测速度、预测位置、可选 jerk 与递归可行的制动包络。
- 独立的位置/速度引用 watchdog、qpOASES hotstart、单臂失败隔离、加速度级 overlay 与 69 列 CSV telemetry。
- Viewer 支持 `V/A` 在 velocity/acceleration QP 间切换；切换时从当前 `q/qdot` 重建引用和 hotstart，headless 序列覆盖双向切换。

## 完整验证

```text
configure/build: PASS
ctest: PASS, 32/32
```

4 秒 headless：

```text
sequence=800
algorithm=hierarchical_qp
control_level=acceleration
accepted=1
cycle_p99_us=809.361
deadline_misses=0
control_failures=0
command_failures=0
completed_stage=12
snapshot_drops=0
telemetry_drops=0
```

Telemetry 检查：69 列，801 行数据，缺列 0，非有限值行 0。

命令：

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml \
  --headless --duration 4 \
  --telemetry /tmp/tianji_otg_acceleration_headless.csv
```

## Direct / OTG velocity / OTG acceleration A/B/C

每种控制器 12 个场景、每场景 600 步，36 个结果行全部满足：

```text
QP failures: 0
hard-bound violations: 0
non-finite metric rows: 0
CSV field-count errors: 0 (26 fields/row)
```

跨场景简单均值（不可达场景会主导位置均值，仅用于确定性回归观察）：

`phase_lag` 以双臂目标/实测位置序列在 `[0, 0.25] s` 内最小均方差的离散时移估计；`settling_time` 是误差此后持续低于 `max(0.01 m, 5% peak)` 的最早时刻。二者是离线确定性回归指标，不等同于真实 VR、网络和执行器端到端延迟。

离线 A/B/C 将 qpOASES 单次预算下限放宽到 20 ms，避免并行主机调度抢占被误记为算法失败；CSV 仍记录实际 solve mean/P99。Viewer/headless 继续使用 profile 中的 350 us 实时预算，实时期限只由 headless 结果验收。

| controller | mean position RMS (m) | mean orientation RMS (rad) | mean phase lag (s) | mean qddot RMS | mean joint jerk RMS | max solve p99 (us) |
|---|---:|---:|---:|---:|---:|---:|
| direct_velocity_qp | 0.237492573 | 0.044900316 | 0.120416667 | 6.020318800 | 828.428556570 | 16.436 |
| otg_velocity_qp | 0.249636083 | 0.129988710 | 0.135833333 | 4.199371504 | 151.661854533 | 9.622 |
| otg_acceleration_qp | 0.248209593 | 0.191692079 | 0.135833333 | 3.891902979 | 216.860243800 | 12.066 |

命令：

```bash
./build/tianji_cartesian_otg_benchmark \
  --config config/qp_ik_cartesian_otg_acceleration.yaml \
  --model models/marvin_m6_qp_test.xml \
  --steps 600 \
  --output /tmp/tianji_otg_acceleration_abc.csv
```

OTG 路径为参考导数整形，不能保证相对无平滑 direct 路径降低瞬时目标误差。加速度 QP 的 joint jerk 均值高于 OTG velocity QP，因为当前 jerk 是软目标而非硬约束；这是一项明确的后续调参/实机约束选择，不影响本轮硬加速度、速度、位置和制动边界验收。

## 限制

MuJoCo harness 直接写入运动学 `q/qdot`，未包含电机、减速器、位置伺服时延、力矩/电流限制和结构柔性。因此结果证明的是运动学闭环、QP 可行性与参考积分一致性，不证明真实执行器能复现这些加速度。实机部署前仍需碰撞约束、硬件参数标定、通信 watchdog 和急停链路。
