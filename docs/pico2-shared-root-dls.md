# PICO2 身高＋C＋共享根掌心映射／DLS-Ruckig

## 2026-09-20 提交前复核

补齐了接收器断开门控：断开事件会立即取消自动续接候选、清除上一有效时间，
并在快速重连竞态下先制动到 HOLD；恢复后必须人工按 S。重新建立求解 epoch
只在静止、显式 S 时执行，避免源时间戳重启导致 worker 退出，也不会在运动中重置速度历史。
P／空格可取消 H 的回程（退出中的 Q／Ctrl+C 除外），取消后保持在当前姿态并需重新 S。

本轮复核结果：`pixi run --locked build` 成功，原生 CTest **107/107**，
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pixi run --locked python -m pytest pico2_hands/tests sim/tests -q`
**133 passed**，`git diff --check` 和 Shell 语法检查通过。未连接真实设备、未启动现场会话；
MuJoCo 窗口的实际闪帧效果仍需重启后的人工观察。

## 后续：短时输入失效的有界恢复

窗口后续由 300 ms 调整为 1 秒，其余门控不变。本次复测
`test_dropout_recovery.py`、`test_shared_root.py`、`test_sim_launcher.py`：
25 passed（11.03 秒），新增 600 ms 断流可恢复和接近 1 秒但稳定采样不足时拒绝的测试。
下方 125 项是此前初始恢复实现的验证记录，不是此次重新运行的数量。

仅 PICO2 shared-root 模式新增恢复候选。失效仍先原生制动；从最后有效输入起
1 秒内须恢复同一连接的 100 ms、至少 5 个独立有效帧，帧间隔不超过 45 ms，
映射有效且 C 未失效。满足后等待原生 HOLD 静止和新帧，走原 S 的软启动路径。
不重置运动中的关节速度／加速度，不继续推进旧目标，不在制动中提前恢复。
超过窗口或资格建立后再次失鲜，均取消；TCP 断开、来源变更、IK／映射拒绝、
P/空格/H/Q/C/R/S 也取消候选。人工暂停／Home 不自动接管。

本轮命令：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pixi run --locked python -m pytest pico2_hands/tests sim/tests -q`，
**125 passed，18.63 秒**，包含真实本地 DLS/Ruckig worker 合成输入的
制动→HOLD→软启动续接和运动步长检查，以及短／长断流、重复帧、稳定性中断、
重连、无效跟踪、映射拒绝和人工操作取消的确定性测试。
本轮未改 C++ 算法或限值，未运行真实输入／设备，未提交或 push。

## 后续：PICO2 仿真接近速度调整

按用户要求，S 接近阶段改为 1.4 rad/s、3 rad/s²、12 rad/s³。
通过仿真专用调用参数传给同一 Ruckig limiter，仍与正常限值取较小值；
默认调用保留 0.35／0.5／2，0.5 秒恢复过程、Home 和真实执行配置不变。
需重启会话生效，本轮未启动或停止现场会话。

本次 `pixi run --locked build` 成功；PICO2 `test_shared_root.py`、
`test_sim_launcher.py` 共 11 项通过（8.83 秒）。原生 CTest 筛选
`test_(ceres_trajectory_limiter|simulation_recovery|franka_dls_controller|ceres_controller|franka_dls_viewer|ceres_viewer)$`
6/6 通过（32.46 秒），包含新增的自定义上限／加速度／jerk 与默认不变测试。
`git diff --check` 通过，模型、配置及真机目录无改动。

## 初始接线验证记录

2026-09-20，新增独立仿真模式；操作见 [PICO2 README](../pico2_hands/README.md)。

```bash
pixi run --locked build
bash pico2_sim.sh --mapping-mode shared-root --height-m 1.62
```

## 实际复用范围

- 人体肩宽、上臂、前臂、腕掌距离来自显式身高比例；未假定有实测肩肘。
- C 采集双腕与头显，建立水平根轴、头到根偏移、腕到机器人掌心旋转。
- 腕到掌心中心只补偿一次，沿 wrist→middle proximal；不再使用 palm 点加偏移。
- 掌心目标按主工程共享根仿射公式生成；读取相同机器人几何 artifact，并校验 URDF/MJCF 指纹。
- IK 直接调用主工程 `DualArmController`，不是另行抄写 DLS；同 profile、Pinocchio、模型掌心 TCP、限位、在线 Ruckig。
- 使用 `SimulationRecovery` 和软启动，H/Q 平滑回双臂、手指保持；失鲜制动 HOLD 后需 S。
- 旧 V131 默认及其 C/XZ、手部 retarget、协议和现场驱动不改动。

这不是完整 VR 输入链路移植：没有 M0 人体骨架重建、shape guidance、
完整闭合投影或人体骨架叠加；没有凭模板生成“实测臂角”。
根随头显平移但不随头部转动，无法区分头部平移与躯干移动；转身／明显弯腰后需重做 C。
这项人体假设及关键点轴向仍需真实输入验收。

## 本轮软件验证

| 命令 | 结果 |
|---|---|
| `pixi run --locked build` 配置；随后 `pixi run --locked cmake --build control/build --target pico2_dls_worker --parallel 2` | 最终构建通过 |
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pixi run --locked python -m pytest pico2_hands/tests sim/tests -q` | 113 passed，17.11 秒 |
| `pixi run --locked test-native` | 106/106，166.72 秒 |
| `bash -n pico2_sim.sh`、`git diff --check`、README 本地文件链接检查 | 通过 |
| `git diff --exit-code -- real_robot tracking manus control/config control/models` | 无改动 |

首轮新 worker 编译因缺少 Eigen Geometry 头失败，补齐后重建通过。
新测试覆盖：1.45/1.62/1.85/2.00 米等比人体、多初始朝向、转头不改变根轴、
无双重掌心补偿、C 缺失／重复／移动／失败、重新连接清标定、DLS 软启动、
禁止运动中 reset、失鲜制动、Home、手指保持，以及 worker/显示掌心 TCP 一致性。

合成 TCP 端到端测试覆盖未标定不接管、原始包记录、身高与后端事件、
定时回 Home 退出、录制排空后 `complete=true`。原始包和状态事件 schema 保持兼容。
测试未连接真实设备，没有执行 ADB、真实输入仿真或真机；未验收实时调度、
不同体型的实际误差、实际握姿轴向和长期跟踪效果。
