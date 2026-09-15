# PICO 双臂遥操作 MuJoCo 验收记录

日期：2026-08-11

普通 Linux 实时性限制、暂缓决定与后续候选方案单独记录在
[`pico_mujoco_realtime_latency_status.md`](pico_mujoco_realtime_latency_status.md)。

## 结论

PICO `corrected_ik` 到天机 MuJoCo 双臂的功能链路已经实现：双掌相对实时双肩中点
进行 1:1 映射，一个 160 字节 UDP 包原子携带左右掌目标，Viewer 以 latest-only
方式交给 200 Hz 控制线程，并可接入 velocity-level 或 acceleration-level
Cartesian OTG/QP。tracking epoch 变化会同步重置两臂参考，输入在 50 ms 边界
进入 stale/hold；PICO 模式会强制该 50 ms 安全值，不受其他 YAML profile 的
`target_timeout_seconds` 影响。

功能测试全部通过。合成 72 Hz 输入下，两种控制层均为 0 控制失败、0 左右时间戳
不一致、0 左右 stale 不一致，CRC/乱序/跳变拒绝均为 0。加速度级 60 秒运行满足
send-to-control p99 小于 5 ms 且 0 deadline miss；速度级两次运行的计算 p99 均远低于
5 ms，但普通 Linux 调度造成 20 次和 1 次绝对周期 miss，send-to-control p99 为
5.31 ms 和 5.12 ms。因此当前结果证明计算量有充分裕量，但不构成硬实时保证。

## 测试范围

本机没有连接 PICO 头显，长时测试由 Python 以 72 Hz 向 Viewer 的 UDP 边界注入与
正式 bridge 完全相同的原子包。它覆盖 UDP receiver、latest-only exchange、50 ms
freshness、双臂 TargetManager、Cartesian OTG、两种 QP、MuJoCo 状态更新和完整 CSV
诊断。真实 `/pico/smpl_palm_corrected_ik` 频率及 ROS 2 bridge 实机输出频率本次未测，
不能用合成 72 Hz 数字代替；PICO bridge 的映射、协议、配对、launch 与 runtime
路径由软件包测试覆盖。

## 自动化验证

PICO bridge 聚焦构建与测试：

```bash
cd /home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1
pixi run --frozen bash -lc \
  'colcon build --base-paths src --symlink-install --packages-select pico_bridge \
   --cmake-args -DPython_ROOT_DIR=$CONDA_PREFIX -DPython_FIND_VIRTUALENV=ONLY'
pixi run --frozen bash -lc \
  'source install/setup.bash && colcon test --base-paths src \
   --packages-select pico_bridge --event-handlers console_direct+'
pixi run --frozen bash -lc \
  'colcon test-result --test-result-base build/pico_bridge --verbose'
```

结果：29 个 CTest 目标、364 个断言/测试，0 error、0 failure、0 skipped。

天机工程：

```bash
cd /home/zj/current_robotics/TJ_arm/TJ_arm_control_pico_mujoco_teleop_v1
pixi run configure
pixi run build
pixi run test
```

结果：42/42 通过。新增覆盖包括两种控制层、普通 75 ms profile 被 PICO 安全契约
覆盖为 50 ms、无包 monitor、双臂源时间戳一致、双臂 stale 一致，以及被拒绝目标不
刷新 freshness。

## 60 秒合成测量

每组 Viewer 时长 60.8 s，发包时长 60.1 s，源频率 72 Hz。时延分位数按每个首次
应用的唯一 `pico_sequence` 计算，避免 200 Hz CSV 对同一个源帧重复采样造成偏置。

| 指标 | Velocity 第一次 | Velocity 复测 | Acceleration |
|---|---:|---:|---:|
| Viewer 输入频率中位数 | 72.000 Hz | 72.000 Hz | 72.000 Hz |
| 应用原子帧 | 4325 | 4326 | 4325 |
| send-to-control p50 | 2.462 ms | 2.342 ms | 2.602 ms |
| send-to-control p95 | 4.740 ms | 4.615 ms | 4.858 ms |
| send-to-control p99 | 5.310 ms | 5.115 ms | 4.967 ms |
| cycle p50 | 0.427 ms | 0.421 ms | 0.856 ms |
| cycle p95 | 0.543 ms | 0.516 ms | 0.975 ms |
| cycle p99 | 0.861 ms | 0.545 ms | 1.053 ms |
| cycle max | 7.176 ms | 3.231 ms | 3.479 ms |
| deadline misses | 20 | 1 | 0 |
| control failures | 0 | 0 | 0 |
| 左右源时间戳不一致 | 0 | 0 | 0 |
| 左右 stale 不一致 | 0 | 0 | 0 |

所有运行的 `malformed`、`crc_failures`、`reordered`、`jump_rejections` 和
`superseded` 均为 0，tracking epoch reset 均为 1。停止发包后的 stale 转换年龄为
51.26 ms、51.69 ms 和 51.37 ms，符合“第一个位于 50 ms 边界之后的 200 Hz 控制拍”
预期。

速度级复测唯一 deadline miss 当拍的控制计算时间仅 0.476 ms，说明该次是线程晚唤醒，
不是 QP 计算超过 5 ms。第一次运行的大多数 miss 同样发生在 0.4--0.7 ms 计算拍，
另有 4 次 5.7--7.2 ms 尖峰。当前主机不允许无特权进程设置 `SCHED_FIFO`，因此不能
把普通内核调度结果解释为硬实时保证。若实机要求严格零 miss，应使用 PREEMPT_RT、
授予受控的实时调度权限、隔离控制 CPU，并在目标部署机上重新执行同一 60 秒测试。

## 无输入监测

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_qp_test.xml \
  --pico-teleop --pico-bind 127.0.0.1 --pico-port 15000 \
  --headless --duration 10 \
  --telemetry /tmp/tianji_pico_no_input_final_10s.csv
```

结果：2000 个控制周期，`pico_configured=1`、`pico_enabled=1`、`pico_live=0`、
`pico_stale=0`、0 deadline miss、0 control failure。未收到过任何有效帧时保持 monitor
状态，不误报 stale。

## 实机联调检查

连接 PICO 后同时记录：

```bash
ros2 topic hz /pico/smpl_palm_corrected_ik
ros2 topic echo /pico/smpl_palm_corrected/status --once
ros2 topic echo /pico/tianji_mujoco_teleop/status --once
```

Viewer 使用 README 中的 `--pico-teleop` 命令启动，并增加 `--telemetry FILE`。至少运行
60 秒，分别按 `V` 和 `A` 测试快速平移、快速旋转、停止、恢复跟踪和 P 键关闭/重启。
应确认 corrected_ik 与 bridge accepted/output 频率一致、左右时间戳不一致为 0、
50--55 ms 内进入 stale、控制失败为 0；硬实时部署还应要求目标系统的 deadline miss
为 0。
