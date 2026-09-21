# 三条输入路线

工作目录：`~/syz/tianji_teleop`。PICO 手柄 + Manus、PICO 手柄 + 外骨骼两条组合路线共用 `bash bash/run_teleop.sh`；PICO 裸手由 `bash bash/run_pico2_sim.sh` 独立运行，仅支持仿真。输入侧物理目录只保留 `src/teleop_inputs/` 下的 `pico_controller`、`manus`、`exoskeleton`、`pico_hand` 四个主输入包（ROS 包名仍为 `pico_bridge`、`manus_bridge`、`exoskeleton_bridge`、`pico2_hands`）；正式 schema-v1 采集器独立于输入目录。

环境由 Pixi 管理，统一为 ROS 2 Jazzy + Fast DDS：`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_DOMAIN_ID=120`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`（只支持同机采集与共享单调时钟）。大多数 `bash/` 入口会自行进入 Pixi 环境并 `source bash/environment.sh`，确认 `ROS_DISTRO=jazzy`、Python 3.12，并清掉旧 shell 可能残留的 Humble／旧 `tracking/install` overlay；`bash bash/build.sh` 与 `bash bash/test_native.sh` 例外，它们要求调用者已经在 Pixi 环境内（推荐用 `pixi run build`／`pixi run test-native`）。不要手动 source 旧环境。

配置只有一个来源：`config/robot.json` 保存设备身份、端口、安全边界、控制器 profile 名与模型路径；`config/collect_real.json` 保存相机角色、序列号和 `state_rate_hz`（整数 `0` 表示该角色禁用）。相机序列号不另存第二份。

## 安装、构建与环境检查

| 目的 | 命令 |
|---|---|
| 裸 shell 完整安装（不接触硬件） | `bash bash/install.sh`：全部锁定环境、外骨骼／PICO2、default／policy overlay、control 原生目标与 Manus 扩展 |
| 只校验前置条件（不下载、不构建） | `bash bash/install.sh --check` |
| 只安装外骨骼环境 | `bash bash/install.sh --exoskeleton` |
| 重建 control 原生目标与当前环境的 ROS 包 | `pixi run build`（default）；`pixi run -e policy build`（policy 独立 overlay） |
| 校验环境（Python 3.12 / Jazzy / Fast DDS） | `pixi run check-env`（等价 `python bash/check_env.py`） |
| 原生 CTest（control core 与 mapped_palm） | `pixi run test-native` |

构建输出按环境分离：`build/<环境>`、`install/<环境>`、`log/<环境>`。colcon 只扫描 `src/`；control 的 core／mapped-palm 分开构建，原生可执行文件安装到 `install/control/bin/` 或 `install/control/lib/mapped_palm/`，统一由资源 helper 定位。不再构建辅助定位、外接 IMU 或旧 PICO 专用录制包。

模型及共用部署 Home 属于 `src/teleop_outputs/tianji/tianji_description/`，运行时用 `tianji_runtime.package_share()` 读取安装资源；native 程序用 `native_executable()`，不指向旧构建树。根 config／profiles／vendor 由 Pixi 注入的 `TIANJI_WORKSPACE` 定位，不能依赖 cwd。DLS／Ceres 的 Home 保持各自配置。缺少消息包 overlay 的节点必须先构建，不靠源码路径注入或旧环境回退。

环境保持隔离：default 为 Jazzy／Python 3.12／Fast DDS，control 为无 ROS 的原生工具链，cameras 为官方 RealSense 4.58.3，manus 为无 ROS 的 Python 3.12／Pinocchio 3.8。policy 使用同一 Jazzy／Python ABI，但拥有自己的 overlay，锁定 CPU torch 2.10.0＋Zenoh；模型权重、CUDA wheel／驱动需要显式准备，CPU 验证不等于 GPU 验收。

## 外骨骼 + PICO → Tianji + Wuji

```text
PICO → UDP :15000 → 双臂控制
外骨骼 → bash/run_exoskeleton.sh（采集、重定向、TJH2 直发）→ UDP :16000 → 双手控制
双臂／双手控制 → bash/run_teleop.sh 选择仿真或真机
```

```bash
# 终端 1：PICO（--user zj 仅为示例，必须替换为实际佩戴者已标定的档案）
bash bash/run_pico.sh --user zj

# 终端 2：外骨骼发送器
bash bash/run_exoskeleton.sh

# 终端 3：仿真
bash bash/run_teleop.sh --sim
```

- 外骨骼实现、标定配置、模型和官方求解器位于 `src/teleop_inputs/exoskeleton/`，不依赖外部 `~/syz/data_glove_wuji_teleop`。`bash bash/run_exoskeleton.sh` 使用该目录自己的 `pixi.toml`／`.pixi` 环境（独立 Python 3.12）；缺少环境时先执行 `bash bash/install.sh --exoskeleton`。
- 该入口默认发送双手；附加参数原样传给发送器（单手 `--hand left`／`--hand right`；离线检查 `--check-config`，只检查配置、不发送数据）。默认已带 `--confirm-send --commission-directions`：允许采集与 UDP 发送，不修改方向验收标记，也不代表真机方向与动作已验证。
- 每帧四连杆／21 通道机构换算和独立 MANO 手型拟合在 `src/teleop_inputs/exoskeleton/native/` 的 C++17 扩展中完成，无 Python 回退路径；修改 C++ 后重跑 `bash bash/install.sh --exoskeleton`。
- 此路径不运行 `bash bash/run_manus.sh`；外骨骼使用自己的设备身份、零位和方向档案。设备与零位档案在 `src/teleop_inputs/exoskeleton/config/dataglove/devices/`，任务绑定在 `src/teleop_inputs/exoskeleton/config/teleoperation/`，传给入口的相对配置路径以该目录（ROS 包 `exoskeleton_bridge`）为基准。
- 默认直接向 `127.0.0.1:16000` 发送控制器的 TJH2 v2 数据，不需要独立桥接进程；源时间使用同机单调时钟，只允许回环 IPv4 目的地址。每侧保留原始帧读取完成时的本机时间，另一侧更新不刷新旧姿态；没有新结果不发包，失败侧清除缓存，退出不补零。

## Manus + PICO → Tianji + Wuji

```text
PICO → UDP :15000 → 双臂控制
Manus → bash/run_manus.sh（采集、重定向和 Hand2 UDP 桥）→ UDP :16000 → 双手控制
双臂／双手控制 → bash/run_teleop.sh 选择仿真或真机
```

```bash
# 终端 1：PICO
bash bash/run_pico.sh --user zj

# 终端 2：Manus，使用同一实际佩戴者的档案
bash bash/run_manus.sh --user zj

# 终端 3：仿真
bash bash/run_teleop.sh --sim
```

- `bash bash/run_manus.sh` 需要且只需要一个人名档案：`--user NAME`（或 `--calibration-user NAME` 直接使用已有标定）；`--list-users`／`--list-calibration-users` 列出可用档案。传给发送器的 Manus 参数原样透传。
- 此路径不运行 `bash bash/run_exoskeleton.sh`；该入口已管理 Manus 采集、手部适配和 Hand2 UDP 桥。
- Manus 桥入口运行在 `default`（Jazzy）环境；手部重定向所用的 `manus` Pixi 环境独立、无 ROS，Pinocchio 固定 3.8。桥输入通过既有私有进程协议进入重定向端，再以 TJH2 输出，不跨环境导入 `rclpy` 或注入 site-packages。源码改动后用 `bash bash/build_manus.sh` 重建。

## 共用执行模式与切换

| 模式 | 命令 |
|---|---|
| 有窗口仿真 | `bash bash/run_teleop.sh --sim` |
| 无窗口 SPARK 仿真 | `bash bash/run_teleop.sh --sim --ik-backend spark --headless`（默认 DLS direct 不支持 headless） |
| 真机执行 | `bash bash/run_teleop.sh --real` |
| 真机 + schema-v1 采集 | `bash bash/run_teleop.sh --data` |

`--sim`／`--real`／`--data` 只能选一个，其余参数原样传给执行器。`--data` 在真机执行的同时按需管理相机与采集节点，把原始 RGB、双臂 14 维和双手 40 维写入数据集；数据集目录由 `--dataset` 给出、任务标签由 `--task` 记录，相机或采集未就绪时不会连接设备。

切真机前先停止仿真，并完成只读设备预检：

```bash
pixi run bash -c 'source bash/environment.sh; python -m tianji_controller.run_teleop --devices all --inspect'
```

`--inspect` 与 `--confirm-real` 互斥，所以预检必须直接调用执行器（`run_teleop.sh` 会固定加上 `--confirm-real`），并先载入 workspace overlay；未构建时先 `pixi run build`。确认设备身份、标定、方向和安全空间后，才启动真机模式并按交互提示逐步授权；只读预检通过不等于动作安全已验证。

**同一时刻只运行一套 PICO、一种手部输入和一个执行模式。**外骨骼与 Manus 不可同时向手部控制端发送，仿真与真机也不可同时占用相同输入端口。切换路径时先停止执行端，再停止旧手部输入，切换并检查后重新启动执行端。

回 Home 只操作双臂：`bash bash/run_home.sh`（默认附加 `--confirm-real`，`--dry-run` 只做无硬件检查）。PICO2 裸手仿真为 `bash bash/run_pico2_sim.sh`，默认 V131；`--mapping-mode shared-root --height-m HEIGHT` 选择身高模板＋C 标定＋DLS/Ruckig。新模式先 C 后 S，H 只回双臂、手指保持，P／空格可取消 H 回程；两种模式均不驱动真机、也不在遥操作端口上发布。首次构建 PICO2 原生依赖用 `bash src/teleop_inputs/pico_hand/build_native.sh`；共享根 worker 另由 `pixi run build` 安装。

## 相机与数据采集（ROS 节点）

相机使用官方 `realsense2_camera` 节点，由 `bash bash/run_cameras.sh` 启动。驱动只跑在独立的 `cameras` Pixi 环境里（ABI 隔离：它是唯一需要 `ros2-distro-mutex >=0.16` 的组件），与其余进程只通过 DDS 通信，不要把它的 site-packages 注入 `default` 环境。

| 目的 | 命令 |
|---|---|
| 启动相机驱动 | `bash bash/run_cameras.sh` |
| 只读 RGB 预览（只订阅，不打开 pipeline） | `bash bash/preview_cameras.sh` |
| 设备枚举与目标模式预检 | `pixi run inspect-cameras` |
| 独立采集节点（不连机器人、不开相机） | `bash bash/run_data_collector.sh` |
| 对应的 Pixi 任务 | `pixi run cameras`／`pixi run preview`／`pixi run collect` |

- 每个启用角色一个驱动节点，私有 RGB 话题统一为 `/cameras/<role>/color/image_raw`，同目录有 `camera_info` 与 metadata。
- 采集节点只订阅相机图像和 `/tianji/feedback/*`、`/tianji/executor/state`；它不打开相机、不连接机器人。`--data` 管理本次创建的相机／collector，已运行节点只有精确配置、身份和 ready 一致才复用；退出只停止本次拥有的进程。
- DDS 服务只能启停录制，永远不能获得使能或运动权限。

## 其余 Pixi 任务

`pixi run` 提供：`test-native`、`test-native-core`、`test-native-mapped-palm`、`test-sim`、`test-collection`、`test-controller`、`test-interfaces`、`test-pico`、`test-pico2`、`test-ros`（domain 121 真实 DDS）、`test-retargeting`、`test-manus`；Mocap 回归用 `pixi run -e policy test-mocap`。

输入任务有 `setup-pico`（`bash/run_setup_pico.sh`，简化人员标定向导）、`stop-pico`（`bash/run_stop_pico.sh`）、`test-pico-simple`／`test-sim-user`，以及 `sim`／`real`／`home`。

Mocap 六个任务保留 `h5_sim`／`h5_real`／`rl_infer`／`rl_live_infer`／`regrind_real`／`regrind_hand_sim`。统一 `tianji mocap` 分发到 `bash/run_mocap.sh`：`infer`／`live`／`regrind-real`／`regrind-hand-sim` 自动进入 policy，其余进入 default。完整流程见 `README-mocap.md`；安装／仿真／合成 DDS 不代表真实输入、相机、机器人、Motive 或 GPU／模型现场验收，最新证据见 `docs/migration-verification-status.md`。
