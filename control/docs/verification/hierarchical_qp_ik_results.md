# Hierarchical Constrained QP-IK 验收结果

验收日期：2026-08-10

分支：`feature/mujoco-cpp-qp-ik-v1`

配置：`config/qp_ik_hierarchical.yaml`

控制周期：200 Hz，`dt = 0.005 s`

## 自动化测试

执行：

```bash
cmake --build build -j2
.pixi/envs/default/bin/ctest --test-dir build --output-on-failure
```

结果：22/22 测试通过，总耗时约 5.71 秒。覆盖内容包括：

- 13 变量 QP 的 Hessian、梯度、等式和边界精确展开；
- 任意 Jacobian 秩下的 slack 可行性；
- qpOASES 初始化解复用和完整 `H/A/Vd/bounds` hotstart；
- DLS 解析公式、统一比例限速及奇异/零 Jacobian；
- `q_ref` 独立积分、stale measured state、双臂 all-or-nothing 和 Q/D 切换连续性；
- 可达、不可达、奇异和双臂组合轨迹；
- 快照/telemetry 非阻塞交换；
- 六场景 QP/DLS 端到端 benchmark。

控制器回归中的最终误差：

| 场景 | 结果 | 阈值 |
|---|---:|---:|
| 2 cm 可达位置目标 | `6.19e-6 m` | `< 0.002 m` |
| 不可达目标返回后的恢复误差 | `4.01e-6 m` | `< 0.003 m` |
| 奇异初态姿态目标 | `1.15e-5 rad` | `< 1 degree` |
| 双臂组合轨迹最大位置误差 | `5.98e-8 m` | `< 0.002 m` |
| 双臂组合轨迹最大姿态误差 | `1.67e-6 rad` | `< 1 degree` |

## Headless Viewer

执行：

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_hierarchical.yaml \
  --model models/marvin_m6_qp_test.xml \
  --headless --duration 4 \
  --telemetry /tmp/tianji_hierarchical_headless.csv
```

结果：

```text
sequence=800
algorithm=hierarchical_qp
mode=hold
accepted=1
cycle_p99_us=204.371
deadline_misses=0
control_failures=0
command_failures=0
completed_stage=10
snapshot_drops=0
telemetry_drops=0
```

阶段 0–10 覆盖 hierarchical QP、手动目标、圆、8 字、纯姿态、组合、DLS、恢复 QP、暂停、恢复和名义位复位。Telemetry CSV 包含每臂 slack、等式残差、引用误差、速度比、活跃边界、求解时间、迭代数和状态。

兼容参数检查：

```bash
./build/tianji_qp_ik_viewer --solver osqp --headless
```

以退出码 1 返回：

```text
viewer error: hierarchical QP supports qpOASES only
```

`--solver qpoases` 仍可用于旧启动命令。

## QP 与 DLS A/B

执行：

```bash
./build/tianji_hierarchical_ik_benchmark \
  --config config/qp_ik_hierarchical.yaml \
  --model models/marvin_m6_qp_test.xml \
  --output /tmp/tianji_hierarchical_ab.csv
```

每个算法和场景运行 600 个周期。两种算法从相同关节初态跟踪相同目标函数。

| 算法 | 场景 | position RMS (m) | orientation RMS (rad) | qdot variation RMS | max velocity ratio | slack pos RMS | slack rot RMS | solve p99 (us) | failures |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| QP | central | `0.006155` | `5.29e-9` | `0.01041` | `0.1572` | `2.17e-7` | `2.23e-8` | `4.23` | 0 |
| QP | combined | `0.008017` | `0.03549` | `0.01648` | `0.3624` | `7.33e-7` | `5.99e-8` | `4.77` | 0 |
| QP | fast | `0.05900` | `0.2004` | `0.05669` | `0.8108` | `3.70e-6` | `7.53e-7` | `4.55` | 0 |
| QP | near_limit | `0.02587` | `0.02053` | `0.08082` | `1.0000` | `0.09618` | `0.06763` | `5.28` | 0 |
| QP | singular | `7.29e-7` | `0.02366` | `0.005431` | `0.1106` | `8.34e-7` | `4.48e-8` | `4.35` | 0 |
| QP | unreachable | `2.5437` | `0.1655` | `2.5399` | `1.0000` | `0.8940` | `0.5500` | `7.65` | 0 |
| DLS | central | `0.006155` | `1.47e-8` | `0.01041` | `0.1572` | `1.94e-6` | `5.99e-8` | `0.45` | 0 |
| DLS | combined | `0.008017` | `0.03549` | `0.01648` | `0.3623` | `6.88e-6` | `1.64e-7` | `0.48` | 0 |
| DLS | fast | `0.05900` | `0.2004` | `0.05667` | `0.8107` | `3.34e-5` | `2.04e-6` | `0.47` | 0 |
| DLS | near_limit | `0.03758` | `2.33e-5` | `0.01912` | `0.2593` | `0.1478` | `6.94e-5` | `0.43` | 0 |
| DLS | singular | `2.01e-6` | `0.02366` | `0.005431` | `0.1106` | `8.39e-6` | `1.22e-7` | `0.42` | 0 |
| DLS | unreachable | `2.7422` | `9.16e-5` | `1.9891` | `1.0000` | `0.9401` | `3.20e-4` | `0.53` | 0 |

汇总：

```text
benchmark_complete accepted=1
qp_solve_p99_max_us=7.58687
rows=12
```

所有场景均保持有限且位于硬关节边界内。`accepted` 还要求中心、组合和奇异场景的最终误差低于 `0.002 m / 1 degree`，并要求不可达 QP 的 slack 高于数值噪声。不可达 QP 的非零位置/姿态 slack 表明不可行任务由 slack 吸收。DLS 求解更快；在中心、组合和快速场景中两者跟踪 RMS 接近。近极限和不可达场景体现了两种方法不同的速度分配与任务牺牲方式，结果只作为证据，不据此宣称某算法在所有指标上占优。

## GUI 烟测与人工项

在 `DISPLAY=:1` 下执行 1 秒真实 GLFW Viewer，窗口成功创建、渲染并以退出码 0 关闭。

自动化无法判断主观手感，以下项目留给操作者确认：

- XYZ 箭头/旋转环拖动连续性；
- 快速拖动是否存在可见抖动；
- `Q/D` 切换时是否主观可见跳变；
- 拖向不可达区域时 slack 增长及返回可达区域后的恢复手感。

## 安全边界

本次验收证明的是 MuJoCo 运动学实现与配置下的数值行为，不是实机安全认证。当前没有硬加速度、碰撞、自碰撞、双臂耦合、动力学、力矩、执行器、VR/SDK 或硬件急停约束。
