# SPARK 三方案跟手性 Benchmark 设计

## 目标

对同一条 TJVR v4 快速 PICO 轨迹比较：

1. SPARK 两阶段位置 IK + 7轴关节空间 Ruckig；
2. SPARK 两阶段位置 IK + Velocity QP；
3. SPARK + Cartesian OTG + OTG-consistent 位置 IK + Velocity QP。

比较对象是原始 SPARK palm 意图到模型 TCP 的端到端跟手性，而不是各算法
对自身平滑参考的跟踪误差。

## 数据口径

- 三次运行使用同一个 TJVR 文件、200 Hz 控制周期和 model-reference 状态。
- 每个 `pico_sequence` 取最后一个 200 Hz 控制样本。
- 共同真值使用 Velocity QP telemetry 中的 `*_target_p*` 和 `*_target_q*`。
- 启动和重同步 blend 窗口显式排除。
- Ruckig 基线独立生成的 SPARK target 与共同真值位置相差超过 1 mm 的样本不参与比较。
- 左右臂作为同口径观测合并统计，同时保留分臂三维图。

## 指标

- 零时移位置/姿态误差：mean、P50、P95、RMSE、maximum。
- 在 0–80 个 PICO frame 内扫描最佳位置和姿态时移；使用源时间戳计算毫秒延迟。
- 最佳时移后的 RMSE，用于区分纯延迟与几何/幅值失真。
- TCP 运动幅值保持率：去均值后实际轨迹 RMS 半径 / 目标 RMS 半径。
- 控制周期 mean/P99、控制失败、deadline miss。
- Velocity QP 两方案从 joint telemetry 统计 qdot/qddot/jerk；Joint Ruckig 使用其内建动态上限 ratio。

## 输出

- `summary.csv` 和 `README.md`；
- 原始意图与实际 TCP 的三维轨迹；
- 精选高动态窗口位置时间序列；
- 误差 CDF；
- 时移扫描曲线；
- 跟手性指标汇总图。

所有原始数据和生成结果保存在 `benchmark_results/`，保持为不提交的大体积产物。

