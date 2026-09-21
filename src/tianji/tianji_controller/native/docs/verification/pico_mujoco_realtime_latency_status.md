# PICO MuJoCo 遥操作实时性现状与暂缓决定

日期：2026-08-11

状态：功能链路已完成；普通 Linux 硬实时优化暂缓，当前不实施控制频率、线程调度或
操作系统配置修改。

## 当前结论

PICO 双掌到天机 MuJoCo 双臂的原子输入、50 ms stale/hold、velocity-level QP、
acceleration-level QP、tracking epoch reset 和诊断链路均已实现并通过自动化测试。

当前 200 Hz 控制循环具有充足的平均计算裕量，但在普通 Ubuntu generic 内核下不能
严格保证：

```text
send-to-control latency 永远小于 5 ms
deadline misses 永远为 0
```

这里的主要限制不是 velocity QP 持续计算过慢，而是 200 Hz 异步采样的理论相位等待
与非实时线程的调度抖动。

## 已测结果

合成输入频率为 72 Hz，Viewer 控制频率为 200 Hz。完整数据与测试范围见
[`pico_mujoco_teleop_results.md`](pico_mujoco_teleop_results.md)。

| 指标 | Velocity 第一次 | Velocity 复测 | Acceleration |
|---|---:|---:|---:|
| send-to-control p99 | 5.310 ms | 5.115 ms | 4.967 ms |
| cycle p99 | 0.861 ms | 0.545 ms | 1.053 ms |
| cycle max | 7.176 ms | 3.231 ms | 3.479 ms |
| deadline misses | 20 | 1 | 0 |
| control failures | 0 | 0 | 0 |
| 左右源时间戳不一致 | 0 | 0 | 0 |
| 左右 stale 不一致 | 0 | 0 | 0 |

速度级复测的唯一 deadline miss 当拍计算时间为 0.476 ms。第一次运行的大多数 miss
同样发生在 0.4--0.7 ms 计算拍，另有 4 次 5.7--7.2 ms 尖峰。这说明 deadline miss
主要来自控制线程晚唤醒，少数来自系统级计算尖峰，并非 QP 的稳定计算时间超过 5 ms。

最终 10 秒无输入监测完成 2000 个控制周期，deadline miss 和 control failure 均为 0。

## 5 ms 指标的结构性限制

当前控制频率与周期为：

```text
control_rate = 200 Hz
control_period = 1 / 200 = 5 ms
```

PICO/UDP 帧异步到达，只有控制线程在下一拍读取并应用。因此即使接收线程没有任何
调度延迟，输入等待下一控制拍的时间仍近似分布在：

```text
phase_wait in [0, 5 ms)
```

send-to-control p99 理论上已经接近 4.95 ms，再叠加 UDP 接收、线程唤醒、时间戳和
交换开销后，稳定满足 p99 小于 5 ms 的余量不足。保持 200 Hz 时，继续微调 QP 权重
不会消除这个结构性上限。

## 当前主机环境

诊断时主机状态为：

```text
kernel: Ubuntu 6.8.0-136-generic
preemption: PREEMPT_DYNAMIC
user rtprio limit: 0
SCHED_FIFO permission: unavailable
memlock limit: 2002372 KiB
CPU governor: powersave on 24 logical CPUs
energy performance preference: performance
isolated CPUs: none
nohz_full CPUs: none
```

当前控制线程由普通 `std::thread` 创建，使用 `SCHED_OTHER`，没有显式 CPU affinity、
`mlockall` 或实时优先级。控制循环已经使用 `CLOCK_MONOTONIC` 和绝对时间
`clock_nanosleep(..., TIMER_ABSTIME, ...)`，因此周期基准方式本身是合理的。

## 已考虑但暂不实施的方案

### 方案一：提高速度级控制频率

将 velocity-level PICO profile 提升到 400 Hz：

```text
control_period = 2.5 ms
observed velocity QP cycle p99 ~= 0.55 ms at 200 Hz
```

这样可为异步相位等待、控制计算与调度抖动预留更多余量。加速度级应单独测量后决定
保持 200 Hz，还是提升到 250--300 Hz。

### 方案二：增加显式实时运行模式

候选接口：

```text
--realtime
--control-cpu CPU
--receiver-cpu CPU
--rt-priority PRIORITY
```

候选行为：

```text
control thread: SCHED_FIFO, priority 80, isolated CPU
UDP receiver:   SCHED_FIFO, priority 70, separate CPU
telemetry/UI:   SCHED_OTHER, housekeeping CPUs
memory:         mlockall(MCL_CURRENT | MCL_FUTURE), prefaulted stack
```

请求实时模式但权限、CPU affinity 或内存锁定失败时应直接启动失败，不允许静默退回
普通线程。

### 方案三：部署 PREEMPT_RT 与 CPU 隔离

候选系统措施包括：

- PREEMPT_RT 内核；
- 受限的 `rtprio` 与 `memlock` 权限；
- cpuset/nohz_full/RCU callback 隔离；
- 将设备 IRQ、桌面、日志和磁盘任务移出控制 CPU；
- 在目标机器上评估深度 C-state、CPU 迁移和频率变化；
- 使用实时压力测试验证，而不是仅依赖空载 Viewer 测试。

PREEMPT_RT 与 CPU 隔离可以显著降低内核调度噪声，但仍不能消除 BIOS SMI、硬件和
固件导致的所有不可控延迟。若“永远小于 5 ms”是形式化硬实时要求，应评估 RTOS、
实时协处理器或具备明确最坏执行时间保证的控制平台。

## 后续需要增加的诊断

当前 `deadline_misses` 同时包含晚唤醒和计算完成超时。恢复实时化工作时，应拆分：

```text
wake_lateness_us = actual_cycle_start - scheduled_cycle_start
compute_time_us = control_finish - actual_cycle_start
completion_lateness_us = control_finish - scheduled_deadline
```

同时记录最大值、p99、p99.9、CPU migration、page fault 和 context switch。硬实时验收
不能只使用平均值或 p99。

## 恢复实施的建议顺序

1. 增加 wake/compute/completion 三类时延诊断，不改变控制行为。
2. 新增独立的 400 Hz velocity profile，并先进行纯 MuJoCo 压力测试。
3. 增加显式实时线程配置及 fail-fast 权限检查。
4. 在 PREEMPT_RT 与隔离 CPU 环境进行 10 分钟、1 小时及后台压力测试。
5. 接入真实 PICO，分别测量 corrected_ik、bridge、UDP 和 control 各段时延。
6. 根据部署要求决定采用统计性 SLO，还是迁移到真正的硬实时平台。

## 当前保留的验收口径

在实时化工作恢复前，当前工程按以下口径使用：

```text
功能正确性：已通过
控制失败：0
左右目标原子性：已通过
50 ms stale/hold：已通过
普通 Linux 低延迟：已验证到统计指标
严格 <5 ms、零 miss 硬保证：未满足，暂不作为当前完成条件
```

该暂缓决定不影响现有 PICO→MuJoCo 功能测试和算法效果评估，但现有参数与测量结果
不得作为实机硬实时或安全认证依据。
