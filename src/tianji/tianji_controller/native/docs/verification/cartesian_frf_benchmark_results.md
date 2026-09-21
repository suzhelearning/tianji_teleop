# Cartesian经验频响基准验证结果

## 测试范围

正式结果位于`benchmark_results/cartesian_frf_full_v3_20260817`。测试矩阵为4算法 × 2手臂 × 7工作点 × 6通道，共336工况。每个工况使用200 Hz控制、100 Hz corrected-skeleton输入、2 s预热、20 s对数chirp和2 s结束保持。位置幅值5 mm，姿态幅值1 degree。

```bash
python3 scripts/run_cartesian_frf_benchmark.py \
  --jobs 12 \
  --output-root benchmark_results/cartesian_frf_full_v3_20260817
```

所有SPARK工况先用连续位置IK生成与目标TCP一致的肩/肘/腕/掌骨架，再进入现有SPARK重定向和QP链路。抽查左臂中心X通道时，corrected skeleton输入标准差达到理想chirp的98.5%，避免旧测试只激励到约20 µm的问题。

## 汇总结果

下表为84工况中位数。统计只使用coherence >= 0.8且输入PSD不低于峰值-40 dB的频点。当前20 s对数chirp的有效频率上限中位数约8.8--9.0 Hz，因此本轮不能断言9--15 Hz性能。

| 算法 | 低频增益 | -3 dB带宽 | 1--5 Hz群时延 | 共振峰 | 有效频点占比 |
|---|---:|---:|---:|---:|---:|
| Direct Velocity QP | 0.970 | 2.246 Hz | 19.27 ms | 0.00 dB | 0.600 |
| SPARK + Velocity QP | 0.969 | 2.246 Hz | 22.74 ms | 0.00 dB | 0.593 |
| SPARK + Cartesian OTG + Velocity QP | 无有效FRF | 无 | 无 | 无 | 0.000 |
| SPARK + Feedforward Velocity QP | 1.053 | 3.516 Hz | 53.69 ms | +0.918 dB | 0.587 |

位置/姿态拆分：

| 算法 | 位置带宽 | 姿态带宽 | 位置群时延 | 姿态群时延 |
|---|---:|---:|---:|---:|
| Direct Velocity QP | 2.734 Hz | 1.758 Hz | 20.70 ms | 17.91 ms |
| SPARK + Velocity QP | 2.734 Hz | 1.758 Hz | 26.37 ms | 20.43 ms |
| SPARK + Feedforward Velocity QP | 3.711 Hz | 3.320 Hz | 55.25 ms | 52.58 ms |

## 结论

当前参数下，Direct Velocity QP时延最低；加入SPARK姿态引导后带宽基本不变，群时延增加约3--6 ms。Feedforward模式将位置带宽从2.73 Hz提高到3.71 Hz、姿态带宽从1.76 Hz提高到3.32 Hz，但低频增益偏高，存在约+0.92 dB峰值，而且1--5 Hz群时延增加到约54 ms。因此它提高了快速变化的通过能力，却尚未达到“更跟手且同样平滑”的综合目标。

Cartesian OTG模式在5 mm/1 degree小信号下被当前stationary-hold迟滞锁住：位置退出误差阈值为20 mm，姿态退出误差阈值为0.035 rad，低频输入不足以退出Hold。因此该模式没有可辨识的线性FRF。这是当前配置真实的非线性死区表现，不应解释为普通线性低通。

四种算法的控制失败、关节位置违规、速度违规、加速度违规和启用的hard-jerk违规均为0。QP求解时间P99中位数依次约为13.5 us、14.9 us、14.1 us和15.8 us，远低于5 ms控制周期。

## 后续调参方向

1. 若目标是跟手且平滑，先保留`SPARK + Velocity QP`作为稳定基线。
2. Feedforward优先减少参考估计/平滑链路相位滞后，同时把低频增益压回0.98--1.02、峰值压到+0.5 dB以内；不能只继续放大前馈。
3. OTG应让Hold退出逻辑与激励幅值/运动意图相关，或在小信号基准中显式关闭stationary hold；两种结果必须分别标记。
4. 若必须覆盖15 Hz，应延长高频停留时间或改用逐频正弦，不能从本轮输入功率不足的9--15 Hz区间推断。

## 结果文件

- `summary.csv`：336工况指标。
- `metrics.json`：逐工况机器可读指标。
- `raw/*.csv`：同步输入、目标、TCP、q/qdot/qddot/jerk及约束统计。
- `figures/<arm>__<channel>__bode.png`：四算法幅值、相位、群时延和coherence叠加图。
- `manifest.json`：并行执行工况数和失败清单。
