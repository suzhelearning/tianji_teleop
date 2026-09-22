# 三条输入路线

工作目录：`~/syz/tianji_teleop-ros2`。PICO 手柄 + 外骨骼使用 `bash bash/run_teleop.sh`；Manus 已迁移为独立 ROS Hand2 目标发布链，当前止于发布，尚未接入仿真／真机执行器；PICO 裸手由 `bash bash/run_pico_hand_sim.sh --height-m HEIGHT` 独立运行，仅支持仿真。输入侧物理目录只保留 `src/teleop_inputs/` 下的 `pico_controller`、`manus`、`exoskeleton`、`pico_hand` 四个主输入包（ROS 包名仍为 `pico_bridge`、`manus_bridge`、`exoskeleton_bridge`、`pico2_hands`）；正式 schema-v1 采集器独立于输入目录。

环境由 Pixi 管理，统一为 ROS 2 Jazzy + Fast DDS：`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_DOMAIN_ID=120`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`（只支持同机采集与共享单调时钟）。大多数 `bash/` 入口会自行进入 Pixi 环境并 `source bash/environment.sh`，确认 `ROS_DISTRO=jazzy`、Python 3.12，并清掉旧 shell 可能残留的 Humble／旧 `tracking/install` overlay；`bash bash/build.sh` 与 `bash bash/test_native.sh` 例外，它们要求调用者已经在 Pixi 环境内（推荐用 `pixi run build`／`pixi run test-native`）。不要手动 source 旧环境。

配置只有一个来源：`config/robot.json` 保存设备身份、端口、安全边界、控制器 profile 名与模型路径；`config/collect_real.json` 保存相机角色、序列号和 `state_rate_hz`（整数 `0` 表示该角色禁用）。相机序列号不另存第二份。

## 本分支唯一遥操作算法基线

用户已指定：本分支后续遥操作实现只采用 main 中已确认的 **共享根掌心映射 + Franka DLS IK + Ruckig** 双臂链路，不再新增、切换或推广其他双臂后端。

- 双臂后端固定为 `franka-dls`，算法标识为 `pico_ee_franka_dls`；配置通过 `controller_profile("qp_ik_pico_shared_root_dls.yaml")` 定位，源码位于 `src/teleop_outputs/tianji/tianji_controller/native/config/`。Ruckig 负责双臂参考轨迹的速度、加速度和 jerk 约束。
- PICO 手柄侧负责人员标定、TCP／骨架校正与坐标变换，不是机器人 IK 求解器。
- Manus 双手使用 **手部骨架 → 21 点适配 → Wuji Hand2 重定向 → 双手关节目标**，不套用双臂 DLS／Ruckig。其他输入路线保留自身输入适配与手部重定向，后续双臂实现仍统一到上述 DLS／Ruckig 主线。
- 标准仿真使用 `direct`／model-reference：直接显示关节目标，不做动力学积分，不发送真机指令。真机执行仍须独立的现场授权与验收。
- `--real`／`--data` 已统一到该 DLS／Ruckig 主线，`--ik-backend` 只接受 `franka-dls`；旧 real 的 SPARK／mapped-palm 选项和专属 C 标定／断流宽限选项已移除。原生子进程仅通过执行器传入的 `--franka-dls-executor` 启用受限回环 TJRC 输出；它不授予使能或运动权限，仍须经过 Python 的反馈、限位、失鲜和分阶段 Enter 授权门控。仅修改 profile 不能把普通仿真变成真机出口。
- 不得为迁移、仿真、headless 或输入接入改用 SPARK、Ceres LM、mapped-palm 或 V131 双臂后端；所需功能应在上述主线上实现并验证，不增加第二套算法实现。
- 后文列出的其他后端和默认值仅描述尚存的历史功能，不是后续实现选项。此约束不表示存量入口已全部完成切换，也不授权擅自删除保留的离线审计、benchmark 或历史对照工具；如需统一存量入口，应按此基线明确迁移并完成行为验收。

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

环境保持隔离：default 为 Jazzy／Python 3.12／Fast DDS，新 Manus 链在此使用锁定 `wuji-sdk==2026.8.31` 的纯求解 `RetargetSession`，不连接 Wuji 硬件；control 为无 ROS 的原生工具链，cameras 为官方 RealSense 4.58.3。保留的 manus/Python 3.12/Pinocchio 3.8 环境只服务历史离线重定向工具，不再属于新 Manus 生产链。policy 使用同一 Jazzy／Python ABI，但拥有自己的 overlay，锁定 CPU torch 2.10.0＋Zenoh；模型权重、CUDA wheel／驱动需要显式准备，CPU 验证不等于 GPU 验收。

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

## Manus → 独立 Hand2 ROS 目标发布

```text
Manus SDK → /manus/raw/{left,right} → 21 点适配
  → /manus/landmarks/{left,right} → Wuji SDK RetargetSession
  → /wuji/{left,right}_hand/joint_commands（每手 20 维 rad）
```

```bash
# NAME 换成实际佩戴者；只采集和发布目标，不连接 Wuji、不发送 UDP
bash bash/run_manus.sh --user NAME
# 已进入 Pixi 并 source bash/environment.sh 时的等价入口
ros2 launch manus_bridge manus_hand2.launch.py user:=NAME
```

- 入口需要且只需要一个人员档案：`--user NAME`（或 `--calibration-user NAME`）；`--list-users`／`--list-calibration-users` 列出完整双侧档案。后续参数是 ROS launch 的 `name:=value`，旧 `--check`、`--host`、`--port` 已移除。
- 使用 `tianji_interfaces` 的 `ManusGlove`、`HandLandmarks`、`HandJointCommand`。每侧保留 glove/side/session/boot/sequence 和 SDK 回调处的 CLOCK_MONOTONIC 源时间；不能用发布或求解时间刷新旧样本。BEST_EFFORT/KEEP_LAST1/VOLATILE；默认失鲜 250ms，失效发布 valid=false 和不可执行数值，禁止补零冒充目标。
- SDK 原始世界坐标为右手 VUH XFromViewer/Z-up/米；适配器仅执行一次 `(x,-y,z)`。输出固定为 thumb/index/middle/ring/pinky 各 S1–S4，SDK 已做重定向，不再套旧 Hand2 关节重排或双臂 IK。
- `start_acquisition:=false` 只启动适配和求解，用于独立调试。合成数据只在 source 环境后显式设置的隔离域 121 发布。显示／调试不参与生产进程的生存条件。
- 包仍是 `manus_bridge`，现使用 ament_cmake_python；`bash bash/build_manus.sh` 构建 ROS 包及消息。rawviz 文本管道、私有 worker、旧 Manus TJH2 输出已退役，无旧算法回退。
- 本次用户限定只完成目标发布：**执行器、安全授权和采集系统未改，新目标不会被当前 `run_teleop.sh --data` 执行。** 后续接入时须由现有执行器唯一写入硬件，逐条录制确认开始后才放行 Hand2 目标，保留最小执行安全门控，采集继续记录实测反馈。不得启动参考仓库自动使能的独立手部驱动。

## 共用执行模式与切换

| 模式 | 命令 |
|---|---|
| 有窗口仿真 | `bash bash/run_teleop.sh --sim` |
| 无窗口 SPARK 仿真 | `bash bash/run_teleop.sh --sim --ik-backend spark --headless`（默认 DLS direct 不支持 headless） |
| 真机执行 | `bash bash/run_teleop.sh --real` |
| 真机 + schema-v1 采集 | `bash bash/run_teleop.sh --data` |

`--sim`／`--real`／`--data` 只能选一个，其余参数原样传给执行器。`--data` 在真机执行的同时按需管理相机与采集节点，把原始 RGB、双臂 14 维和双手 40 维写入数据集；数据集目录由 `--dataset` 给出、任务标签由 `--task` 记录，相机或采集未就绪时不会连接设备。

两种执行模式共用 `qp_ik_pico_shared_root_dls.yaml`，运行副本开启共享根并固定 `pico_ee_franka_dls`＋Ruckig；默认模型为冻结的 `marvin_m6_wuji2_shared_root_ceres.xml`，文件名中的 `ceres` 不代表使用 Ceres 算法。`controller_velocity_scale` 同时约束运行副本的 Ruckig 速度上限。禁止用旧后端或旧配置回退绕过启动拒绝。双臂仍从只读实测姿态初始化，采集服务永远不能获得运动授权。

切真机前先停止仿真，并完成只读设备预检：

```bash
pixi run bash -c 'source bash/environment.sh; python -m tianji_controller.run_teleop --devices all --inspect'
```

`--inspect` 与 `--confirm-real` 互斥，所以预检必须直接调用执行器（`run_teleop.sh` 会固定加上 `--confirm-real`），并先载入 workspace overlay；未构建时先 `pixi run build`。确认设备身份、标定、方向和安全空间后，才启动真机模式并按交互提示逐步授权；只读预检通过不等于动作安全已验证。

**同一时刻只运行一套 PICO、一种手部输入和一个执行模式。**外骨骼与 Manus 不可同时向手部控制端发送，仿真与真机也不可同时占用相同输入端口。切换路径时先停止执行端，再停止旧手部输入，切换并检查后重新启动执行端。

回 Home 只操作双臂：`bash bash/run_home.sh`（默认附加 `--confirm-real`，`--dry-run` 只做无硬件检查）。PICO 裸手入口为 `bash bash/run_pico_hand_sim.sh --height-m HEIGHT`，只保留身高模板＋C 标定＋共享根 DLS/Ruckig，必须显式给出实际身高（米，1.0～2.4）；旧 V131、旧入口和 `--mapping-mode` 已删除，不保留别名或回退。先 C 后 S，H 只回双臂、手指保持，P／空格可取消 H 回程，Q 回 Home 后退出；不驱动真机，也不在遥操作端口上发布。`--self-test` 走实际 DLS/Hand2 的 C/S/H/S/Q 离线流程，仍须传身高。Hand2 原生依赖由 `bash src/teleop_inputs/pico_hand/build_native.sh` 构建，双臂 DLS worker 由 `pixi run build` 安装。Python/ROS 包和 `pico2_sim_session_v1` 录制 schema 不改名。

裸手入口的主职责是给 SPD 仿真提供 ROS command，普通启动默认发布 `/spd/tianji_wuji2/v1/joint_command`（`tianji_spd_interfaces/msg/JointCommand`，54 维 rad，Fast DDS、domain 120、LOCALHOST、BEST_EFFORT/KEEP_LAST1/VOLATILE，最多 60 Hz）。原生 Viewer 只辅助观察同一份目标，`--headless` 不关闭发布。消息来自 `core.tick()` 控制目标，不从 Viewer／qpos 反取；ready 按有效目标、保持意图和源时间生成，不能沿用显示用固定 flags=7。C／源身份／epoch 变化建立新会话，SPD 仍需本地显式授权；P/H/Q 的受控制动／回程继续发布，最后一帧本地 publish 不代表 SPD 物理 Home 确认。`--self-test` 强制使用隔离域 121。消息包在本工作区按同一固定 schema 编译，不注入 SPD 的 Python overlay。此链不连接机器人、不接入 TJRC 真机输出，也不改 Manus／外骨骼＋PICO 手柄的真机路线。

裸手普通启动在默认 `127.0.0.1:10002`／`localhost:10002` 上自动检查 ADB 设备与转发，缺失时以 `--no-rebind` 补建，匹配时复用；离线／未授权、多设备未选择或转发冲突在启动发布器／Viewer 前拒绝。多设备可用 `ANDROID_SERIAL` 指定。自测及自定义 host／port 不操作 ADB；转发不在退出时删除，不自动覆盖其他设备的映射。

## 相机与数据采集（ROS 节点）

相机使用官方 `realsense2_camera` 节点，由 `bash bash/run_cameras.sh` 启动。驱动只跑在独立的 `cameras` Pixi 环境里（ABI 隔离：它是唯一需要 `ros2-distro-mutex >=0.16` 的组件），与其余进程只通过 DDS 通信，不要把它的 site-packages 注入 `default` 环境。

| 目的 | 命令 |
|---|---|
| 启动相机驱动 | `bash bash/run_cameras.sh` |
| 只读 RGB 预览（只订阅，不打开 pipeline） | `bash bash/preview_cameras.sh` |
| top RGB 传到 PICO（只订阅，不录制） | `bash bash/run_pico_camera.sh`／`pixi run pico-camera` |
| 设备枚举与目标模式预检 | `pixi run inspect-cameras` |
| 独立采集节点（不连机器人、不开相机） | `bash bash/run_data_collector.sh` |
| 对应的 Pixi 任务 | `pixi run cameras`／`pixi run preview`／`pixi run collect` |

- 每个启用角色一个驱动节点，私有 RGB 话题统一为 `/cameras/<role>/color/image_raw`，同目录有 `camera_info` 与 metadata。
- 采集节点只订阅相机图像和 `/tianji/feedback/*`、`/tianji/executor/state`；它不打开相机、不连接机器人。`--data` 管理本次创建的相机／collector，已运行节点只有精确配置、身份和 ready 一致才复用；退出只停止本次拥有的进程。
- DDS 录制操作只使用 `/start_collect`（StartCollect）与 `/stop_collect`（StopCollect），旧 RecordingCommand 已移除。r 等待实际 RECORDING 的服务结果及匹配条目状态后放行遥操作；s 调 stop(save=true)，d 调 stop(save=false)，确认本条完成且受控制动结束后才回双臂 Home。首次本地授权不变；服务永远不能获得使能或运动权限。
- 请求用规范 UUID request_id 绑定 session／phase_revision；stop 还绑定 episode_id（start 的 request_id）。同进程同请求复用终态结果或等待同一操作，改参复用 ID 拒绝；缓存不跨采集器重启，不自动重试未知结果。内部故障 stop(abort=true,save=false) 保留 partial，不等同 d 的删除。服务 success 是实际完成，不是入队确认；连续反馈／状态仍走 Topic。

PICO 视频串流运行在 default，只订阅唯一配置中的 top RGB，不打开相机 pipeline，不影响 Manus／外骨骼或真机执行。先运行官方相机节点；头显 PC 视频源地址为 127.0.0.1。视频脚本只管理 reverse 13579 与 forward 12345，复用匹配映射、冲突拒绝、退出只清理本次新建且仍匹配的映射，绝不修改裸手 10002。使用现有 PICO OPEN/CLOSE_CAMERA 控制协议及大端长度头 H.264、同源双眼 SBS，无录制；无相机、失鲜或背压不能用重复旧帧掩盖。物理相机与头显显示的现场验收独立于合成 ROS／编码／协议验证。

## 其余 Pixi 任务

`pixi run` 提供：`test-native`、`test-native-core`、`test-native-mapped-palm`、`test-sim`、`test-collection`、`test-controller`、`test-interfaces`、`test-pico`、`test-pico2`、`test-ros`（domain 121 真实 DDS）、`test-retargeting`、`test-manus`；Mocap 回归用 `pixi run -e policy test-mocap`。

输入任务有 `setup-pico`（`bash/run_setup_pico.sh`，简化人员标定向导）、`stop-pico`（`bash/run_stop_pico.sh`）、`test-pico-simple`／`test-sim-user`，以及 `sim`／`real`／`home`。

Mocap 六个任务保留 `h5_sim`／`h5_real`／`rl_infer`／`rl_live_infer`／`regrind_real`／`regrind_hand_sim`。统一 `tianji mocap` 分发到 `bash/run_mocap.sh`：`infer`／`live`／`regrind-real`／`regrind-hand-sim` 自动进入 policy，其余进入 default。完整流程见 `README-mocap.md`；安装／仿真／合成 DDS 不代表真实输入、相机、机器人、Motive 或 GPU／模型现场验收，最新证据见 `docs/migration-verification-status.md`。
