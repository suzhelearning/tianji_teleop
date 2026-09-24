# 完整操作参考

本页记录当前 DLS/Ruckig 链路的标定、安装、采集与安全操作；旧后端实现和说明已删除，历史由 Git 保存。
新人员简化标定和默认 DLS 仿真请优先按[当前快速开始](README.md)操作。
以下各路线的标定、环境和授权要求不同，不要交叉执行；历史测试数字不代表本轮重新验证。
当前工作空间为 `bash/`、`config/`、`src/` 和 `vendor/`；默认使用 Jazzy／Python 3.12／Fast DDS。
本轮安装与图形窗口证据及仍待完成项统一见[迁移验证状态](docs/migration-verification-status.md)。
本文历史实机、微基准和回归记录保留原日期与范围，不作为迁移后的硬件或全量测试通过声明。

> 双臂通信更新：标准 DLS 仿真及真机执行使用 `pico_arm_input` → `/pico/arm_input`
> → `tianji_arm_ros`（同一 DLS/Ruckig 核心）→ `/tianji/controller/joint_targets`。
> 业务 UDP 15000／17000 已从生产路径移除；双臂不再提供其他 IK 后端或自动回退。
> 新核心在隔离 `arm-ros` 环境构建，`pixi run build-arm-ros` 不连接设备。

> 四终端恢复（2026-09-24）：PICO、Manus、`run_camera_views.sh`、`run_teleop.sh --data`
> 分别运行；具体命令和当前 r/s/d 授权流程以 [四终端启动](README.md#picomanus-双臂双手遥操) 为准。
> Manus ROS 目标已接入默认仿真和 Python 安全执行器；输入节点本身不连接 Wuji 硬件。
> 本页旧 `/hand_input`、私有 worker、`--check`／`--port`、Manus UDP 命令及历史验收仅作历史记录。

# 天机遥操数据采集

PICO 裸手使用独立[仿真入口 `bash/run_pico_hand_sim.sh --height-m HEIGHT`](src/teleop_inputs/pico_hand/README.md)，只保留共享根 DLS/Ruckig，旧 V131 和模式选择已删除。
此前已通过合成输入／假 TCP 测试，用户已进行真实 PICO 输入仿真并录制；
跟踪抖动仍是已知限制，不能视为性能验收完成；不支持真机。
现有 VR、Manus、外骨骼输入入口保留；默认仿真以及 `--real`／`--data` 均采用共享根 Franka DLS＋Ruckig。真机执行器只接受 `franka-dls`，不再提供旧 real 后端选择。


以 **人员标定 → PICO／Manus 输入 → C++ 重定向与控制 → 安全执行 → 数据落盘** 为主链。
标定、Hand2 retargeting、双臂控制、模型和必要的原生依赖源码均位于本仓库，不需要克隆其他业务仓库或初始化子模块。
硬件 SDK、ROS、MuJoCo、Pinocchio 等通用库仍须安装；厂商二进制和私钥不由重构替代。

> **三种模式必须区分，且不能同时占用相同输入端口。**
> - `pixi run sim`（或 `bash bash/run_teleop.sh --sim`）：仅 Franka DLS＋Ruckig direct 仿真，按 S 接入；默认接收 Manus ROS 双手目标，纯双臂使用 `--no-hand-teleop`，外骨骼须显式 `--hand-source exoskeleton`。
> - `bash bash/run_teleop.sh --real`：共享根 DLS＋Ruckig 双臂与 Manus ROS 双 Hand2 真机执行，保留实测／目标窗口；独立预检及逐步授权不可省略。
> - `bash bash/run_teleop.sh --data --task TASK`：先启动 `bash bash/run_camera_views.sh` 并等待就绪；执行器管理独立 DDS 采集器，默认写入 `/data/TianjiData/raw/YYYYMMDD/`，可用 `--dataset PATH` 指定根目录。
> - 当前真机支持双臂与左右两只 Hand2；`all` 表示双臂＋双手，`hands` 表示仅双手。
> - 历史离线测试和设备身份读取不能替代当前只读预检；尚不能宣称迁移后带运动实机闭环已验收。

## 新用户：左侧标定＋镜像 TCP＋身高骨长（可选）

新增独立入口，不替换旧双侧实测流程。只采左侧掌心 TCP 位置/姿态，
右侧由显式对称模型生成；上臂/前臂按身高模板生成，腕掌距离为身高 × 0.037037。
无需腕部旋转采样。已有用户配置不自动迁移。
模型采用对应局部 Y 轴镜像与对称握持假设，首次使用仍须核对实际轴向。

推荐单终端向导（先停止旧遥操和 PICO 输入，连接 USB 并授权）：

```bash
pixi run build
pixi run setup-pico --user NEW_USER --height-m 1.70
```

按提示输入人员名、身高并完成 TCP 采集，展示结果后提示“回车确认使用并显示骨架，输入 q 取消”。
回车即发布本次结果并自动切换到 MuJoCo 骨架显示；q 取消，其他输入重新提示。
不会启动机械臂执行器；检查骨架后另开终端运行 `pixi run sim`。
也可预填 `setup-pico --user NEW_USER --height-m 1.70`（替换实际人员名与身高）。
向导和下方手动流程二选一，不同时运行。

原始 PICO driver 单独运行时，在另一终端执行（身高为米，示例需替换）：

```bash
pixi run -e default bash -c 'source bash/environment.sh; exec python src/teleop_inputs/pico_controller/scripts/calibrate_pico_simple.py --user NEW_USER --height-m 1.70 --accept-symmetric-model'
```

按提示完成左侧掌心 TCP 采集，检查身高派生参数，输入 `publish` 后保存到独立 `pico-simple` 指针。
停止原始 driver 后，用 `bash bash/run_pico.sh --user NEW_USER` 选择该简化模型。
首次或更新后执行 `pixi run build`；原始 driver 入口为 `pixi run -e default bash bash/start_pico_driver.sh`。
标定和输入使用 default Jazzy／Python 3.12，机械臂另开终端运行 `pixi run sim`。
该入口仅启动 PICO 输入，不启动执行器；完整双侧实测档案使用下文显式 `--calibration-dir` 流程，
不把独立 `pico-simple` 指针误当成完整 `profile.yaml`。
完整设备准备、操作及取消说明见[简化标定](docs/pico-simple-calibration.md)。
新软件链路已做离线验证，尚未完成真实输入验收；不要直接将其视为真机动作放行。

### 换身高、换体型时的适配边界

共享根映射会根据输入骨架的肩宽和臂展分别计算横向、前后／上下尺度，
并非固定使用 1.62 m 人员的比例。简化标定的骨长仍是身高模板估计，不是实际臂长测量。

2026-09-20 用 1.62 m 录制的 4471 帧构造了 11 组合成人体离线测试：
等比例身高 1.45～1.95 m 的机械臂掌心目标与基线几乎一致；单独改变臂长／肩宽时，
目标出现厘米级差异。全部组最终几何闭合，但不代表 IK、碰撞或真人换人验收通过。
该结果仅针对当前共享根映射；软件几何验证不等于实际佩戴、碰撞或真机验收。

换人须以新的人员名和实际身高重新标定、明确发布，再检查 MuJoCo 骨架的掌心位置、
转腕方向及伸臂动作；不要复用上一人的 TCP 或手工改派生骨长文件。
完整步骤及仍需检查的体型差异见[简化标定说明](docs/pico-simple-calibration.md#换人身高与臂长适配)。

当前仿真只使用 DLS/Ruckig：S 接入、H 回双臂 Home、P／空格停止跟随。实际证据见迁移验证状态。

## PICO＋VR 手柄：DLS/Ruckig 与人员骨架

`bash bash/run_teleop.sh --sim` 只使用 Franka DLS＋Ruckig 的 direct/model-reference 显示。
`--real`／`--data` 使用同一双臂算法，但运动授权仍只属于 Python 安全执行器。

```bash
pixi run build
pixi run sim --user YOUR_USER --no-hand-teleop
pixi run test-native
pixi run test-sim
```

`YOUR_USER` 必须换成实际佩戴者的已发布标定；已有输入会话需工作区、人员和指纹一致，
且通过新鲜 ROS 输入检查。不会静默换人，退出只停止本次拥有的输入会话。
双臂 ROS 核心位于独立 arm-ros 环境，普通 control 环境保留 DLS worker 和只读显示工具。
人员骨长策略属于输入标定，不是另一种 IK 后端。需要对称快照时仍可用以下显式流程。

### 1. 环境和构建

在本工程根目录操作。便捷包装器自动激活环境；下文需要直接调用 Python／底层脚本的终端先执行：

```bash
pixi shell -e default
source bash/environment.sh
```

首次安装先按下文安装流程准备依赖；代码更新后统一构建原生后端和 `src/` ROS 包：

```bash
pixi run build
```

### 2. 终端一：启动对称骨架输入

使用 VR 手柄路线的头显 APK。`YOUR_USER` 必须替换成实际佩戴者的已发布档案名。
切换前先退出旧执行器及旧 PICO 输入会话，不能重复启动或同时占用端口。

```bash
python -m tianji profile --list-users
PICO_PROFILE_DIR=$(python -m tianji profile --user YOUR_USER --component pico) &&
PICO_SYMMETRIC_DIR=$(python src/teleop_inputs/pico_controller/scripts/pico_symmetric_profile.py --source "$PICO_PROFILE_DIR" --resolve) &&
bash bash/start_tianji_pico_teleop.sh --calibration-dir "$PICO_SYMMETRIC_DIR" --pico-world-x-offset 0.20
```

此入口显式选择对称快照，bridge 的 PICO 世界 X 偏移为 +0.20 m。
`--resolve` 会校验快照；未生成或已过期时拒绝，不静默退回原配置。
已有完整标定但没有快照时，可先离线生成（会新增快照与指针），然后重新运行上述解析与启动：

```bash
PICO_PROFILE_DIR=$(python -m tianji profile --user YOUR_USER --component pico) &&
python src/teleop_inputs/pico_controller/scripts/pico_symmetric_profile.py --source "$PICO_PROFILE_DIR"
```

若需 A 键地面初始化，应在真机未使能前完成；执行中不要重置 PICO 坐标系。
保持 PICO 输入运行，再从下面两种执行模式中选择一种。

### 3A. 终端二：仿真（不连接真机）

```bash
bash bash/run_teleop.sh --sim --no-hand-teleop
```

在机器人窗口按 S 接入、H 受控回双臂 Home、P／空格停止跟随。回位中不排队接管，
普通仿真不导出硬件目标；不加 `--no-hand-teleop` 时默认接收 Manus ROS 手部目标。

### 3B. 终端二：真机仅双臂

先停止仿真，确认急停、设备身份和运动空间，再做只读预检：

```bash
pixi run bash -c 'source bash/environment.sh; python -m tianji_controller.run_teleop --devices arms --inspect'
```

现场确认通过后才启动 `bash bash/run_teleop.sh --real --devices arms`。
第一次 Enter 慢速对齐，到 READY 后第二次 Enter 开始跟随；TELEOP 中第三次 Enter
受控回 Home 并失能。对齐／回位中的 Enter 或 Ctrl+C 直接停止，不追加 Home。
这不是图形窗口的 S/H/P 授权，也不驱动手部。软件验证不等于现场动作验收。


## 目录

`src/teleop_inputs/` 收录人手侧输入，`src/teleop_outputs/tianji/` 与 `src/teleop_outputs/wuji/`
收录机器人侧的目标映射、控制、模型与适配。

| 目录 | 用途 |
| --- | --- |
| `bash/` | 安装、构建、环境激活和日常操作包装器 |
| `config/` | 机器人、设备身份、唯一采集／相机配置及 Fast DDS 配置 |
| `src/teleop_inputs/` | 人手侧输入：PICO 手柄、Manus、外骨骼和 PICO 裸手四种主输入；正式观测采集独立位于 `src/data_collector/` |
| `src/teleop_outputs/tianji/tianji_cmd_pub/` | 机器人侧：PICO 人体目标映射与原子 `PicoArmInput` ROS 发布 |
| `src/teleop_outputs/tianji/tianji_controller/` | 机器人侧：原生控制算法、Python 真机执行器和反馈发布 |
| `src/teleop_outputs/tianji/tianji_description/` | 机器人侧：安装到 share 的机器人模型、网格及共用 Home |
| `src/teleop_outputs/wuji/` | 机器人侧：Hand2 硬件适配与重定向 |
| `src/cameras/` | 官方 RealSense 驱动的启动／检查／订阅预览，以及可选鱼眼节点 |
| `src/data_collector/` | 独立 DDS 观察采集、HDF5 段生命周期、离线压缩与浏览 |
| `src/interfaces/` | ROS 接口与轻量 `tianji_runtime` 契约、资源定位 |
| `src/simulation/`、`src/inference/` | 仿真与 Mocap／Regrind 执行 |
| `src/tools/tianji_tools/` | `tianji` CLI、`teleop_profile` 及数据路径策略 |
| `profiles/` | 实际人员档案、PICO 标定版本及 `profiles/<user>/manus/` 双手标定；不包含机器人硬件配置 |
| `vendor/` | 厂商 SDK／第三方资源；许可证和私有凭证边界保留 |
| `src/teleop_outputs/tianji/tianji_controller/native/third_party/ruckig/` | 固定版本离线轨迹生成源码及许可证 |

### C++ 与 Python 边界

- PICO／Manus 采集、共享根映射、Franka DLS 和 Ruckig 在 C++ 原生代码中执行；
  双臂输入输出使用原子 ROS 消息，没有第二条硬件控制通道。
- 新 Manus 使用 SDK Hand2 RetargetSession；其余仍被使用的离线手部工具保持独立。
- Pinocchio FK／Jacobian、双侧安全提交、恢复和轨迹约束保留，不因删除旧后端改变数值约定。
- Python 保留标定交互、配置、SDK 编排、安全状态机和数据段生命周期；NumPy、h5py／HDF5、OpenCV 本身调用原生库。
  不为了换语言重写这些边界，也不改变 schema-v1、人工授权或过期输入拒绝策略。

历史原生化离线微基准（非本次迁移复测）：左右 Hand2 实际模型共 240 组状态（含 alpha 端点、thumb mask、正则和耦合项），
新旧目标函数及梯度一致；含 FK 的目标函数约 **255 μs → 90–91 μs，2.8×**。
CRC 对 0–656 字节随机数据及标准校验向量一致，360／464／652 字节包约 **4.3×**。
这是局部计算测量，不代表真机采集帧率或完整闭环延迟提高同样倍数。

## 通用准备

所有命令从仓库根目录开始。ROS Python 包由 ament／colcon 安装到对应环境的 overlay；模型从 `tianji_description` 的 share 资源定位，原生程序从 `install/control/bin` 定位。`TIANJI_WORKSPACE` 由 Pixi 包装器固定到 checkout，用于 config、profiles、vendor 和日志，不根据当前目录猜资源，不是脱离 checkout 的独立 wheel。

### 精简副本：一键安装与编译

本仓库保留精简副本的源码、模型、厂商 SDK 和现有标定，不包含原仓库历史、旧工程、
Python／Pixi 环境、构建产物、日志、录制数据及私钥。
Manus SDK 动态库使用 Git LFS 管理；克隆时请先安装 Git LFS，再获取真实库文件：

```bash
git lfs install
git clone git@github.com:suzhelearning/tianji_teleop.git
cd tianji_teleop
git lfs pull
```

如果使用的是已经包含真实 SDK 动态库的精简压缩包，则无需执行 Git LFS 命令。

支持 Linux x86_64，首次安装需要联网。先按
[Pixi 官方安装说明](https://pixi.sh/latest/installation/) 安装 Pixi。
Ubuntu／Debian 系统依赖由管理员安装一次：

```bash
sudo apt-get update
sudo apt-get install build-essential libudev1 libusb-1.0-0 zlib1g adb tmux
```

然后在本目录执行：

```bash
bash bash/install.sh --check  # 只检查基础前置条件，不下载、不编译
bash bash/install.sh          # 安装锁定环境并编译，不连接硬件
```

脚本按根 `pixi.lock` 安装 default、control、cameras、manus、policy 全部锁定环境；
另安装 `src/teleop_inputs/exoskeleton/` 和 PICO2 原生工具的独立锁定环境。
default 为 Jazzy／Python 3.12／Fast DDS；control 为原生控制工具链；cameras 单独运行官方 RealSense 4.58.3；
manus 为 ROS-free Python 3.12／Pinocchio 3.8 重定向；policy 与 default 同 Jazzy/Python ABI，
但有自己的 `install/policy` overlay，锁定 CPU PyTorch 2.10 和 Zenoh。GPU／模型权重须另行显式准备。
完整安装构建 default/policy ROS 包、control 原生程序、Manus 及 PICO2 原生组件；不创建旧 `.venv`，
不加载 Humble／tracking overlay，也不把一个环境的 site-packages 注入另一个环境。
失败时立即停止，修复报错后可重新运行。脚本不会自动调用 sudo，不启动或使能硬件。

请为依赖、下载缓存和构建产物预留充足磁盘空间，不能只按源码包大小预留。
安装后不要随意移动或删除 checkout，symlink-install、外骨骼 editable 包和配置资源仍依赖它。
USB 权限、设备网络地址及个人标定仍须按后续文档配置；已有标定不适用于任意新操作者。

已有主项目环境，仅补装外骨骼输入时执行 `bash bash/install.sh --exoskeleton`。
该分支安装 `src/teleop_inputs/exoskeleton/.pixi/envs/default` 并重建其扩展，不重装主环境、不编译控制器／Manus／ROS，也不启动硬件。编译需要 `/usr/bin/gcc`、`/usr/bin/g++`（Ubuntu/Debian 的 `build-essential`）。

当前构建、无硬件验证及未验收范围以[迁移验证状态](docs/migration-verification-status.md)为准。

### 手动分步安装

```bash
pixi install --locked --all
pixi run --locked check-env
pixi run --locked build
pixi run --locked -e policy build
pixi run prepare-manus
bash bash/install.sh --exoskeleton
pixi install --locked --manifest-path src/teleop_inputs/pico_hand/tools/wuji_hand_native/pixi.toml
bash src/teleop_inputs/pico_hand/build_native.sh
```

`build` 只扫描 `src/`，默认输出为 `build/default`、`install/default`、`log/default`；
policy 有独立 overlay。DLS 原生目标在 `build/control/core`，ROS 适配核心在
`build/arm-ros/core`，可执行文件安装到 `install/control/bin/`。Manus ROS 链在 default
运行；不同环境不互相注入 Python 或数值库路径。

系统还需 `adb`、`tmux`、USB／网络权限。必需的厂商库通过本仓库 LFS 提供。
运行包装器不需要手动激活；需要下面的 `python -m tianji` 等直接模块命令时，在该终端先执行：

```bash
pixi shell -e default
source bash/environment.sh
```

环境激活只 source 当前环境的 `install/<环境>/local_setup.bash`。不要设置旧 `ROS_SETUP`，
不要使用根 `.venv` 或跨环境 `TIANJI_PYTHON`；外骨骼和相机包装器自行切到各自环境。

```bash
python -m tianji --help
python -m tianji profile --list-users
# PICO 简化标定与 Manus 分别选择，必须对应实际佩戴者。
bash bash/run_pico.sh --user NEW_USER
bash bash/run_manus.sh --calibration-user MANUS_USER
# 在独立交互终端选择以下一种模式，不要同时运行。
bash bash/run_teleop.sh --sim
bash bash/run_teleop.sh --real
bash bash/run_teleop.sh --data --task pick_hammer --dataset /data/tianji
# 离线工具无需机器人或输入设备。
python -m tianji compress /data/tianji /data/tianji_jpeg50
python -m tianji visualize /data/tianji_jpeg50
```

`real`／`home` CLI 默认不使能，须明确传入 `--confirm-real`；便捷 shell 入口沿用既有显式授权语义。
`collect` 仅运行独立 DDS 观察采集器，无 SDK／运动授权；`bash bash/run_teleop.sh --data` 才管理真机采集，仍受 TTY、设备、相机预检和人工 Enter 门控，不能无人值守使能。

- 首次克隆后运行 `git lfs install` 和 `git lfs pull`，获取由 Git LFS 管理的 Manus SDK 动态库。

真机运行前置条件（本次离线验证不代表已连接硬件）：

- 已完成 PICO、双臂控制工程和 Manus 采集器的安装、构建。
- PICO 已通过 USB 连接并授权，头显动作流应用在前台，追踪设备正常。
- 双臂遥操前，左右侧 **TCP → 手腕 → 骨长** 标定均有效；首次标定命令见文末。
- 手部遥操前，Manus 手套已开机、配对，且选择实际佩戴者对应的标定用户。
- 图形仿真需在本机桌面终端运行。

默认通道：

| 通道 | 默认设置 |
| --- | --- |
| ROS 2 | Jazzy／Fast DDS，Domain `120`，`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`；清除 `ROS_LOCALHOST_ONLY` |
| PICO 双臂输入 | ROS `/pico/arm_input`，`PicoArmInput` |
| 默认手部输入（Manus） | ROS `/wuji/{left,right}_hand/joint_commands`，`HandJointCommand` |
| 显式手部输入（外骨骼） | `--hand-source exoskeleton`，TJH2 UDP `127.0.0.1:16000` |
| 控制器 → 真机执行器 | ROS `/tianji/controller/joint_targets`，`ControllerJointTargets`，同机 boot/时钟 |

**同一时刻只保留一套 PICO 驱动、一套 Manus 采集链路和一个机器人控制器。**
ROS 订阅不代表拥有硬件；执行器和 Home 通过本机互斥租约排他，不能换 domain/topic 绕过。

## RealSense RGB 相机

`pixi run cameras`（或 `bash bash/run_cameras.sh`）从唯一配置 `config/collect_real.json`
读取启用角色和 serial，在独立 cameras 环境启动官方 `realsense2_camera` 4.58.3，
固定请求 **1280×720、30 FPS、RGB8**。不启用深度流，不连接机械臂／灵巧手，不降分辨率。
普通 RGB sensor 使用 `rgb_camera.color_profile/color_format`，D405 使用对应 `depth_module` 参数。
preflight 枚举目标模式但不打开 pipeline；实际 ready 要求参数读回、Image 与 Metadata 配对及新鲜度全部通过。

```bash
# 在允许访问相机后执行；不会连接机器人。
pixi run inspect-cameras
pixi run cameras
# 另一个桌面终端：只订阅，退出不停止相机发布者。
pixi run preview
```

默认 ROS Domain 为 `120`。每路发布 `/cameras/<role>/color/image_raw`（`rgb8`），
同目录 `camera_info` 与 `metadata`；驱动使用 SENSOR_DATA，消费者使用 BEST_EFFORT。
官方节点拥有唯一 SDK pipeline，内参／畸变使用官方 camera_info，不把 D405 畸变手工冒充 `plumb_bob`。
monitor 校验 serial、目标 profile／format、源帧号、发布者身份与配对流；不以约 30 Hz 替代模式验收。
相机更换发布者、参数改变或断流会撤销 ready；不会重打时间戳或自动重启后拼接原录制。
第二套相同 serial 的 launch 会被会话锁拒绝。预览和采集可同时订阅同一发布者。

### top RGB → PICO 有线画面

`bash bash/run_pico_camera.sh`（`pixi run pico-camera`）是 default 环境内的纯订阅者，
读取同一配置的 top 角色和 `/cameras/top/color/image_raw`，不创建 SDK pipeline。
先运行上述官方相机节点；在 PICO 软件启用 PC 视频源并将地址设为 `127.0.0.1`。
启动先等待有效 RGB，再检查 ADB 视频映射：reverse `13579` 为相机控制、
forward `12345` 为 H.264 接收；不改变裸手 `10002`。只删除本次创建且仍归本连接所有的规则。

控制帧沿用 `OPEN_CAMERA`／`CLOSE_CAMERA`，视频为 4 字节大端长度＋完整 AnnexB access unit。
固定软件 libx264、无 B 帧、SPS/PPS 与周期关键帧、左右眼重复同一 top 图像；
尺寸取 PICO 请求，帧率不超过 30，默认 4M 码率，不写图像或视频文件。
仅在编码前选择最新 RGB；编码后不随意丢 P 帧，超龄／背压时关闭连接以免破坏解码或播放旧画面。
启动图像等待默认 15 秒，可用 `--timeout` 调整；`--duration` 限定 ready 后的服务时长。
现有 manager 已管理映射时用 `--no-adb`，多设备用 `ANDROID_SERIAL`。
细节和验证边界见[首页串流说明](README.md#top-相机画面传到-pico)。

## 观测数据集采集（R / S / D）

采集契约见 [schema-v1.md](schema-v1.md)。只保存实际双臂关节位置（14 维）、
实际双手关节位置（40 维）和 RGB；**不保存 actions 或控制目标**。
状态以 120 Hz 为读取目标，每路 RGB 配置为 1280×720@30 FPS；采集直接保存未压缩 uint8 RGB 像素，不进行 JPEG 编码。
各流使用独立、以 episode 起点为零的主机单调时间戳；实际采样频率以文件时间戳为准。
训练时的时间偏移和 state/action 配对由训练端自行完成。

当前 `config/collect_real.json` 是相机角色／序列号的唯一配置来源：

| 视角 | 序列号 |
| --- | --- |
| top | `409122274035` |
| left_wrist | `409122274621` |
| right_wrist | `409122273692` |

先按真机章节完成只读预检，启动所需的 PICO 和 Manus／外骨骼输入。日常一体化入口：

```bash
bash bash/run_teleop.sh --data --task pick_hammer
# 可选：指定原始数据根目录，仍按日期保存。
bash bash/run_teleop.sh --data --task pick_hammer --dataset /data/pick_hammer
```

`--data` 在连接硬件前启动或验证相机与独立 collector，确认 profile、配置及 writer 已 prepared；
连接后等待真实反馈 inputs_ready，才进入原有人工授权流程。缺相机／目录不可写时不会先连接设备。
collector 只通过 DDS 观察实际反馈、执行器状态及 RGB/Metadata，不加载硬件 SDK，也不重复打开相机。
若需独立运行，可在不同终端先 `pixi run cameras` 与
`pixi run collect --task pick_hammer`，再运行相同配置／task／dataset 的 `--data`。
只有身份、配置与就绪状态验证匹配才复用；冲突拒绝，不偷偷接管。退出仅停止本次创建的进程，
复用的 collector 只结束本执行会话的录制，硬件停止／释放先于自建采集进程的关闭等待。
普通 `--real` 不要求相机或 collector，collector 故障不授予或撤销硬件运动权限。

默认输出到 `/data/TianjiData/raw/`（可用 `TIANJI_DATASET` 或 `--dataset` 覆盖），
启动前必须以实测反馈确认双臂已在 Home（配置的关节对齐容差内），否则拒绝使能；初次 Enter 授权本场使能并保持。若需外层 console／退出日志，激活 default overlay 后使用
`python -m tianji real --devices all --confirm-real --collection --task pick_hammer`，日志保存至 `logs/real/`。
`--data` 使用逐条遥操采集：机器人整场只使能一次，条目之间回 Home 保持，不跟随输入。
按键在执行器终端直接输入，不追加 Enter：

| 按键 | 数据操作 | 机器人行为 |
| --- | --- | --- |
| `r` | 在 `HOME_READY` 申请新条目 | 冻结当前目标、慢速对齐；确认实际 `RECORDING` 后开始跟随 |
| `s` | 截止当前条，保存为 `success=true` | 停止跟随并受控制动；保存确认且实测静止后回双臂 Home，保持使能 |
| `d` | 丢弃当前条，不删除已保存文件 | 同样制动、回 Home，保持使能 |
| `q` | 在 Home 结束整场；录制中则先保存当前条 | 正常回 Home 完成后失能退出 |

必须等 `HOME_READY` 才能开始下一条，保存、制动、对齐和回程中不排队开始键。
双手在回程中保持，不自动张开；下一条重新对齐后才接入 Manus。对齐和 Home 过程不录入任务片段。
脚踏板应配置成键盘单次 `r`／`s`，不连发、不附带 Enter；当前依赖执行器终端焦点。
按键经有界队列异步发送 ROS 服务请求，不阻塞控制循环。start/save/discard 必须匹配当前健康、
新鲜的 real/TELEOP session 与 phase_revision；服务 accepted 只是入队，不表示写盘完成。
S/D 的冻结截止时刻是 collector 接受有效请求时，不是按键发出时；超时结果未知时查看状态，不自动重发。
`--data` 使能后的 Enter、Ctrl+C 或关闭机器人窗口立即安全停止，不自动 Home；普通 `--real` 的原 Enter 流程不变。
采集拒绝、超时、失鲜、反馈／限位或执行器异常均结束本场并走安全清理，不强行回 Home、不自动重试或重新使能。
未完成的录制保留 `.partial.h5`；已成功保存的条目不会因后续回程故障被删除。保存成功不等于 Home 或失能已确认。
独立 collector 服务仍没有运动权限；由执行器协调这些状态，不能用 DDS 录制请求触发机器人使能。
录制中即使进程仍存活，真实反馈超过 300 ms 或有效 Image/Metadata 配对超过 2 s 未更新也会判为采集故障；
结束当前段时再次检查并冻结该段状态。按 `s` 截止后的相机故障不否决已结束的正常录制，
但截止前的采集故障及该段 HDF5 写入/校验错误仍会阻止成功发布。
写入者关闭超时会进入 `FAILED`，不允许再次按 `r`，退出时重试清理；
只有确认关闭后才释放文件管理权。初始化超时或中断后，阻塞的文件操作一旦返回，
后台写入者会关闭并清理未开始录制的文件；无法强制中断尚未返回的文件系统调用。

目录和文件名沿用 mocap 的 `YYYYMMDD/YYYYMMDD_HHMMSS_ffffff_takeNNN.h5`，
例如 `20260913/20260913_143025_123456_take001.h5`。使用本地时间、六位微秒和进程内递增的 take 序号；
时间在文件准备时生成（第一段在硬件连接前预留），丢弃或失败不会回收序号。
完整文件包含 `observations/arms`、`observations/hands` 和
`images/top`、`images/left_wrist`、`images/right_wrist`，各相机包含 `timestamp_ns` 和 `rgb[N,720,1280,3]`。单个后台 HDF5 写入者处理有界队列和校验，
控制线程不等待写盘；保存失败保持 `.partial.h5`，不覆盖旧文件。
`dataset_config.json` 写在每个 `raw/YYYYMMDD/` 日期目录内，与该日 HDF5 文件同级。不同日期可使用不同配置；同一天内关节/相机/编码契约不匹配时拒绝追加，不覆盖已有配置。`raw/` 根目录不再写共享配置。
已有 JPEG 数据集保留原有契约和数据，不覆盖配置、不补造相机图片。
原始 RGB 采集请使用新默认目录或另一个空目录，不向旧 JPEG 数据集追加。
三路 30 Hz 原始图像约 **249 MB/s、14.9 GB/分钟**，需要充足的持续写入带宽和磁盘空间；名义帧率不是实测保证。

采集、压缩和浏览使用 default Pixi 环境及对应 overlay；相机发布者使用隔离 cameras 环境。
默认原始数据目录为 `/data/TianjiData/raw/`，压缩目录为 `/data/TianjiData/compressed/`；
请确保当前用户有写入权限且磁盘空间充足，也可通过 `--dataset` 指定其他数据盘，备份由操作者管理。

### 相机实时预览（不录制）

在桌面终端运行：

```bash
pixi run preview
```

先在另一终端 `pixi run cameras`。预览读取 `config/collect_real.json` 中启用的相机，
按 top、left_wrist、right_wrist 顺序显示，标注序列号及约 2 秒的帧率：
`RX` 为接收帧率，`UI` 为提交新画面的帧率，不是显示器物理刷新率，也不是 HDF5 写入率。
它是 ROS 订阅者，不打开 SDK pipeline，不连接机器人、不写文件；RGB8 1280×720 输入仅在预览缩小。
窗口只显示最新画面，绘图慢时允许 UI 降低；不编码 JPEG，Q50 由离线压缩另生成副本。
按 `Q`／`Esc`、关闭窗口或 Ctrl+C 只释放本次订阅者，不停止其他进程拥有的相机。
可与采集同时运行；不要另启第二套相同 serial 的 driver。需要桌面环境，支持 `--config <配置路径>`。
图像或 Metadata 缺失、配对失败或有效画面超过 2 秒不更新时会报错，不把冻结画面视为健康实时流。

### 离线批量压缩（固定 JPEG Q50）

采集并保存若干段后，定期手动运行：

```bash
bash bash/compress_data.sh
```

默认读取 `/data/TianjiData/raw`，输出到 `/data/TianjiData/compressed`；日期目录 `YYYYMMDD/` 对应输出 `YYYYMMDD_compressed/`，每个日期输出目录包含独立的 `dataset_config.json`，可单独浏览。
**压缩质量固定为 50；RGB 或已有 JPEG 源文件均不删除、不覆盖。** JPEG 输入先解码再编码为 Q50，会产生额外有损损失，程序启动时会提示源质量。保持文件名、所有关节值、时间戳、任务和成功标记；显式指定输出目录时保持原始相对路径，不追加日期目录后缀。
脚本跳过 `.partial.h5` 与符号链接；已完成输出通过源身份和 JPEG 格式校验后跳过，之后新增的 episode 会在下次运行时处理。
中断或失败不会发布半成品为完成文件；可重复运行继续，结束时显示 `converted/skipped/failed`，失败返回非零退出码。
不要手动改写已压缩文件的源文件；身份不匹配或目标配置冲突时脚本拒绝覆盖。

可显式指定原始目录和独立输出目录：

```bash
bash bash/compress_data.sh /data/tianji_raw /data/tianji_jpeg50
bash bash/visualize_data.sh /data/TianjiData/compressed
```

按日期处理 `/data/TianjiData/raw` 中的数据（配置从各自的日期目录读取，不同日期独立校验）：

```bash
bash bash/compress.sh --date 20260914
# 输出到 /data/TianjiData/compressed/20260914_compressed；不传 --date 时使用当天日期。
bash bash/view.sh --date 20260914
```

设置 `TIANJI_DATASET` 时，它指向原始数据根目录，压缩根目录为其同级 `compressed/`；采集命令单独传入的 `--dataset` 不会自动传给之后的压缩命令。
确认压缩命令成功（`failed=0`）且输出可读取后，可自行手动删除对应 `raw/YYYYMMDD/`（包括该日配置）；脚本不会自动删除原始数据。压缩后的 `compressed/YYYYMMDD_compressed/` 自带该日配置，不依赖 `raw/`，两个汇总根目录均不写共享配置。

源/目标不能相同或互相包含。建议在暂停录制时压缩，避免读盘、写盘和编码争抢采集资源。
压缩会额外占用空间，不会释放原始数据占用的空间；磁盘不足时先将原始数据备份到更大磁盘。

### 数据集可视化（只读）

从项目根目录启动本地网页查看器，不需要启动 PICO、Manus、机器人或相机：

```bash
bash bash/visualize_data.sh /data/TianjiData/raw
```

浏览器打开 `http://127.0.0.1:8765/`。也可传入整个数据集目录或单个 `.h5` 文件；
端口占用时加 `--port 8766`，使用终端打印的地址。终端 `Ctrl+C` 关闭查看器。
不传目录时默认查看原始 RGB 数据集；旧 JPEG 数据和 Q50 输出也可显式传目录查看。
相机面板标明 RGB/JPEG。原始 RGB 仅在网页传输时转为无损 PNG，不改动磁盘文件；JPEG 图片直接读取已有编码字节。

- 选择 episode，播放／暂停、调速、拖动时间轴，或跳到上一／下一条实际采样时间。
- 同屏查看相机 RGB、双臂 14 维和双手 40 维实测关节轨迹；曲线可勾选，读数支持 rad／度。
- 每条流按自身时间戳取“不晚于当前时间的最后一条记录”，显示样本时间与年龄；
  不插值、不显示未来帧，也不把保留的最后观测当作新测量。
  相机加载新帧期间保留上一张已解码且不晚于当前时刻的画面，解码完成后替换；
  时间标签对应实际显示的帧，加载较慢时年龄会增大，不每帧清空画面。
- 文件属性与统计显示任务、成功状态、各流样本数、实际平均频率和最大采样间隔。
- `.partial.h5` 始终标为未完成、非成功确认。可读取已关闭的部分数据；
  正在写入、损坏或不兼容的文件会明确报错，不绕过 HDF5 文件锁。

查看器只监听本机，包装器使用 default Pixi 环境，不修改文件、不连接硬件，
也不展示数据集中未记录的控制指令。关节名称从上级 `dataset_config.json` 读取。

## 人员档案与标定版本

从工作区根目录显式选择实际佩戴者。完整双侧档案使用以下入口；简化标定则用首页的 `run_pico.sh --user`：

```bash
pixi shell -e default
source bash/environment.sh
python -m tianji profile --list-users
PICO_PROFILE_DIR=$(python -m tianji profile --user YOUR_USER --component pico) &&
bash bash/start_tianji_pico_teleop.sh --calibration-dir "$PICO_PROFILE_DIR"

# 另一个终端，实际人员档案会映射到对应 Manus 标定。
bash bash/run_manus.sh --user YOUR_USER
```

人员选择不默认使用 syz，不支持 `--syz`。解析失败不要启动输入；不得省略 `--calibration-dir` 退回全局配置。
完整档案由 `src/tools/tianji_tools/teleop_profile/` 模块解析；人员选择与仿真／真机模式、硬件 IP/SN 分开。

历史档案格式示例 `profiles/syz/profile.yaml`（以实际 checkout 和实际佩戴者为准，不保证包含此人标定）：

```yaml
schema_version: 1
user_id: syz
pico:
  active_set: '20260908-01'
manus:
  user: syz
```

上述示例会从 `profiles/syz/pico/20260908-01/` 加载左右各 3 个文件：
`pico_{left,right}_{palm_tcp,wrist_pivot,arm_geometry}.yaml`。
迁移保留已有人员制品，不生成个人标定，也不改写原 `~/.config/pico_tracker/`。
Manus 唯一读取 `profiles/<user>/manus/` 下对应用户的左右 `.mcal`，不复制到源码包或安装目录。
`run_manus.sh --user NAME` 与 `--calibration-user NAME` 均直接选择同名人员，不需要 `profile.yaml`；
历史 `tianji profile --component manus` 映射仍解析到所选用户的上述目录。

按人启动 PICO 时，两侧 TCP、手腕和骨长依赖链都必须通过现有 artifact 校验。
缺失、损坏、身份不符、路径越界或哈希不匹配时，在停止旧会话前退出；
不回退到全局标定、其他版本、其他人员或 SMPL baseline。
选中的版本绝对路径显式传到 tmux 中的 M0 进程，不依赖 tmux 的历史环境变量。
Manus 与 PICO 独立校验：仅使用双手时，不要求 PICO 标定可用，反之亦然。

### 添加人员或更新标定

新人员可以直接在标定命令中指定人员 ID，无需手工复制文件或创建档案。先按附录启动
标定用 PICO 驱动并完成地面初始化，然后执行（以 `zjx` 为例）：

```bash
# 本终端先按“通用准备”进入 default Pixi shell 并 source bash/environment.sh。
bash bash/calibrate_pico_arm.sh left all --user zjx &&
bash bash/calibrate_pico_arm.sh right all --user zjx
```

- `--user NAME` 可以放在侧别／操作之前或之后；同一命令不能指定两个人员。
- 过程中的有效结果保存到 `profiles/zjx/pico/.draft/`，第二条命令继续使用该草稿。
- 新人员完成左侧后显示 `Calibration pending`，不创建可用的 `profile.yaml`；
  右侧完成、左右 6 个文件及依赖链全部有效后，显示 `Published profile`。
- 发布时自动生成 `profiles/zjx/pico/cal-<唯一ID>/`，并原子更新
  `profiles/zjx/profile.yaml` 的 `pico.active_set`，不覆盖旧版本或全局标定。
- 原始采集保存在 `profiles/zjx/recordings/`。该路径不会随草稿发布改变，
  保持 artifact 内部的 `source_recording` 路径有效。
- 失败或中断返回非零状态，保留草稿，不发布、不继续下一阶段。
  修正后可重跑相应单项，例如 `left wrist --user zjx`。
- 已有人员首次创建更新草稿时只从自己的有效活动版本复制，不读取全局文件。
  因此更新已有人员的一侧后，如果另一侧旧标定仍匹配，可直接发布一套完整新版本。
- 同一人员不能并发运行两个标定命令。

完成后只读检查与启动：

```bash
bash bash/calibrate_pico_arm.sh status --user zjx
PICO_PROFILE_DIR=$(python -m tianji profile --user zjx --component pico) &&
bash bash/start_tianji_pico_teleop.sh --calibration-dir "$PICO_PROFILE_DIR"
```

`status --user` 只检查已发布版本，不会创建草稿；首次仅完成一侧时会报告尚无已发布档案。
新档案默认将 `manus.user` 设为相同人员 ID，但不会生成 Manus 标定。
仍需准备 `profiles/zjx/manus/zjxLeftMetaglovePro.mcal` 和对应的 Right 文件后，
才能执行 `bash bash/run_manus.sh --user zjx`。添加标定文件无需重新构建。

不传 `--user` 的旧标定入口仍写入 `~/.config/pico_tracker/`；
多人适配必须显式指定人员，不能把旧全局结果当作新人员的草稿。

运行中的会话固定使用启动时选定的版本，不因修改 `active_set` 热切换。
直接调用底层 `bash/start_tianji_pico_teleop.sh` 而不传 `--calibration-dir` 仍可加载旧全局配置；
多人遥操必须显式解析本人的已发布版本，不能依赖该默认值。

---

## 一、仿真遥操作

本节只控制 MuJoCo，不连接真实机械臂或灵巧手。

### 1. 启动 PICO 双臂输入

若完整 PICO 链路已经正常运行，跳过此步。
如果仍有单独运行的 `bash/start_pico_driver.sh`，先在其终端按 `Ctrl+C` 停止，避免重复连接头显。

终端 1：

```bash
cd /path/to/tianji_teleop
# 完整双侧档案；先进入 default Pixi shell 并 source bash/environment.sh。
PICO_PROFILE_DIR=$(python -m tianji profile --user YOUR_USER --component pico) &&
bash bash/start_tianji_pico_teleop.sh --calibration-dir "$PICO_PROFILE_DIR"
```

该脚本启动驱动、修正骨架、人体骨架 Viewer 和 UDP 桥。
戴好设备、双脚站稳，按一次右手柄 **A 键**，等待日志出现：

```text
Canonical SMPL floor locked ...
```

查看后台窗口：

```bash
# 从工作区根目录操作，无需切换到旧 tracking 目录。
tmux attach -t pico_tianji_teleop
```

- `Ctrl-b`，松开后按 `0`：驱动窗口。
- `Ctrl-b`，松开后按 `1`：修正骨架窗口。
- `Ctrl-b`，松开后按 `2`：UDP 桥窗口。
- **`Ctrl-b`，松开后按 `d`：退出界面，后台继续运行。**

不要用 `Ctrl+C` 代替退出 tmux；它会停止窗口中的程序。
使用同一环境的 `tmux` 客户端与服务端。

### 2. 启动 Manus 灵巧手输入

终端 2：

```bash
cd /path/to/tianji_teleop

# 查看已登记的人员档案。
bash bash/run_manus.sh --list-users

# 选择实际佩戴者，档案会映射到已有的 Manus 标定用户。
bash bash/run_manus.sh --user YOUR_USER
```

该命令统一管理 Manus 采集器、ROS `/hand_input` 适配器及 Hand2 UDP 桥。
保持终端运行，不要另开第二个采集器。

#### 外骨骼手套输入（替代 Manus）

外骨骼源代码、设备／零位档案、手套与 Wuji 模型、官方重定向求解器位于 `src/teleop_inputs/exoskeleton/`，不依赖外部旧工程。终端 2 直接运行发送器，不需要额外桥接终端：

```bash
# 首次使用，或更新 C++ 内核后重新编译安装：
bash bash/install.sh --exoskeleton

# 只检查本地身份档案、零位、方向、FK/MANO 和模型，不连接设备：
bash bash/run_exoskeleton.sh --check-config

# 采集、重定向并直接向本机控制器发送 TJH2：
bash bash/run_exoskeleton.sh
# 已激活 default overlay 时也可：python -m tianji exoskeleton <相同参数>
```

默认启动双手；只启动左手用 `bash bash/run_exoskeleton.sh --hand left`，只启动右手用 `bash bash/run_exoskeleton.sh --hand right`。
入口默认允许采集发送和未验收方向调试，不需要追加 `--confirm-send` 或 `--commission-directions`。
当前迁入档案的方向尚未验收；默认调试许可不会修改验收标记，也不代表真机动作已验证。
`bash/run_exoskeleton.sh` 只管理发送器，不启动 PICO、仿真或真机。
缺少内部环境时会提示安装命令，不借用原工程环境。

后续设备与零位配置修改在 `src/teleop_inputs/exoskeleton/config/dataglove/devices/`，任务绑定在
同包的 `config/teleoperation/`。传给包装器的相对配置路径以 `src/teleop_inputs/exoskeleton/` 为基准；
也可用 `pixi run --manifest-path src/teleop_inputs/exoskeleton/pixi.toml --locked <任务>` 使用注册、标定等工具。
这是独立的仓库内源代码副本，原工程未删除，但两处代码和标定不会自动同步。
原有录制数据、虚拟环境、缓存和未使用的 STEP/USD 资源未迁入；部署所需的 URDF/MJCF/STL 已保留。
第三方模型与求解器许可证随资源保留；原手套 URDF 的 BSD 许可声明不完整，不能将整个目录视为统一 MIT 授权。

外骨骼每帧的数值内核位于 `src/teleop_inputs/exoskeleton/native/`：

- `encoder_kinematics.cpp`：四连杆几何求解和 21 通道机构换算，缓存机构参数，保留无解／退化检查。
- `morphology.cpp`：独立 MANO 手型的前向几何、残差／解析雅可比、初值选择、有界迭代、时序项和捏合二次拟合。固定大小工作数组；拟合期间释放 GIL，每个实例独立保护并原子提交历史状态。
- `module.cpp`：通过 pybind11 暴露给现有 Python 接口；不保留旧 Python 数值循环或缺少扩展时的回退路径。

这是数值内核迁移，不是整套程序 C++ 化：配置与标定、设备采集、软件零位和 URDF 映射调度仍在 Python，
MuJoCo FK 继续使用现有原生库，官方 Wuji 求解 worker 与 TJH2 发送路径保持不变。
`setup.py` 使用 C++17、`-O3`、`-ffp-contract=off`，不启用 fast-math。
运行 `bash bash/install.sh --exoskeleton` 会按锁文件安装依赖并重建本地 editable 包，避免仅修改 C++ 后误用旧扩展；
运行中的进程不会热更新，重新启动 `bash/run_exoskeleton.sh` 后使用新内核。

不要同时启动 `bash/run_manus.sh`。PICO 输入和 `bash/run_teleop.sh --sim/--real` 的原有流程不变。
旧版 JSON UDP 中转已移除；切换前停止旧发送器和旧桥接进程。外骨骼与 Manus 共用协议编码器，但外骨骼直发不依赖 ROS 或 Manus 标定。

发送器默认向 `127.0.0.1:16000` 发送 364 字节的 TJH2 v2 二进制包，含 CRC32、
递增序号、发布时间和每侧原始测量时间；每侧 20 个有限弧度角。
关节顺序以 `src/tools/tianji_tools/tianji/hand_protocol.py::JOINT_STEMS` 为准：
拇指、食指、中指、无名指、小指，每指 4 个关节。发送官方 Wuji 模型坐标下的 qpos 并重排关节顺序，不额外裁剪或拒绝有限的越界值；真机安全仍由执行端负责。

只有新求解结果触发发送，不再增加 100 Hz 中转重发。未更新侧可以随包携带缓存姿态，
但保留该侧原始帧读取完成时的 `monotonic_ns`，不能用另一侧更新或发布时间刷新其年龄。
FK／求解耗时计入帧年龄；发送器保留过期丢弃门禁，执行端继续独立检查每侧 150 ms 新鲜度。
冷启动求解超过该时限的帧也不能用于执行。故障侧清除缓存，不用零位替代；
没有新结果、空发送和退出均不补包，恢复需重启该侧采集。

这一路径要求发送器和控制器共享同机单调时钟，`--udp-host` 仅允许回环 IPv4 地址，
不能将目的地址改为另一台电脑。`--udp-port` 可显式覆盖默认端口，但必须匹配控制器配置。

使用 `bash bash/run_exoskeleton.sh --help` 查看参数。先仿真逐指验证，再走原有真机安全流程。
历史外骨骼迁入验证覆盖锁定环境、双手离线配置，以及合成零位帧经真实 FK／官方求解 worker／随机回环 UDP 直发，由原生协议解码和每侧新鲜度逻辑检查；覆盖旧侧过期、故障缓存清除与退出不补包。当时未连接实物手套、修改网络或启动真机，实际外骨骼＋PICO 带运动闭环仍需按安全流程验证。
历史原生内核迁移曾编译安装，并通过左右手各两帧合成编码器输入运行实际机构换算／FK／手型拟合／官方求解／TJH2 UDP 链路；当时未运行测试套件或性能基准，不宣称具体加速倍数，不作为本轮重跑结果。

### 3. 启动 DLS 参考显示

```bash
bash bash/run_teleop.sh --sim --user YOUR_USER --no-hand-teleop
```

该入口直接显示 DLS/Ruckig 参考，不做动力学积分，不连接真实硬件。
PICO 输入和原生核心通过 ROS 通信；没有有效输入时保持，不通过其他算法或脚本轨迹回退。
S/H/P 的操作见首页。通用物理显示引擎仍供 Mocap 回放使用，不是可选双臂 IK 后端。

### 4. 停止仿真

1. 关闭机器人 Viewer。
2. 在 Manus 终端按 `Ctrl+C`。
3. 停止 PICO 链路：

```bash
pixi run stop-pico
```

---

## 二、真机遥操作

> **先空载、低速测试。** 清空运动范围，确认硬件急停随手可按，停止其他控制同一设备的程序。
> 不要无人值守地使能，不要通过放宽限位或跳过反馈检查来解决启动失败。
> 软件 watchdog 不替代物理急停；SDK 阻塞、进程强制终止及固件行为都可能影响实际停机。

包含双臂的真机入口内部启动无界面控制器，并自动打开一个独立、只读的 MuJoCo 实测／目标窗口。
窗口必须成功打开后才连接设备；不要同时运行两个执行模式。双臂已改为 ROS，保留手部输入仍占用 16000。
PICO 人体骨架窗口可以保留。单手／双手独立执行不包含双臂时，保留原来的无窗口启动流程。

### 1. 核对设备配置和左右手 SN

配置文件：[`config/robot.json`](config/robot.json)。
当前示例配置为：

- 双臂 IP：`192.168.1.190`。
- 左手 SN：`WH2JA01260811019`。
- 右手 SN：`WH2KA01260814006`。
- `active_devices`：双臂、左手、右手；命令行可单独选择设备。

设备更换后，应先确认实际 SN，不能按扫描顺序或 IP 猜测左右手：
左右 SN 来自已保存的 `config/hand_devices.json` 固件身份查询结果，连接及使能前仍会再次校验。
两侧不能配置成同一个 SN；双手共用一份 SDK 初始化，但设备句柄、反馈和故障状态独立。

当前内部目标接口为 **ControllerJointTargets ROS 消息**，包含双臂 14 维 rad 参考及兼容旧手部路线的左右各 20 维参考。
它保留实际已应用输入时间、产生时间和有效标志；不是实测反馈，也不授予使能。旧 TJRC 字节协议只供历史离线工具。

```bash
cd /path/to/tianji_teleop

# 只扫描，不连接设备。
pixi run -e default bash -c 'source bash/environment.sh; exec python -m wuji_controller.read_hand_serials'

# 额外读取固件的左右手身份，随后断开，不使能电机。
pixi run -e default bash -c 'source bash/environment.sh; exec python -m wuji_controller.read_hand_serials --read-handedness'

# 可选：保存到新文件，不覆盖已有配置，也不自动改写控制配置。
pixi run -e default bash -c 'source bash/environment.sh; exec python -m wuji_controller.read_hand_serials --read-handedness --output config/hand_devices.new.json'
```

SDK 建立身份查询连接时可能进行时间同步；建议在没有真机控制会话运行时执行。

### 2. 只读设备预检

从工作区根目录执行；每个需要直接模块命令的终端先按通用准备进入 `pixi shell -e default` 并 `source bash/environment.sh`：

```bash
cd /path/to/tianji_teleop

# 左右手身份和各自 20 个关节反馈。
python -m tianji_controller.run_teleop \
  --devices hands --inspect

# 双臂 14 个关节反馈。
python -m tianji_controller.run_teleop \
  --devices arms --inspect
```

该模式不清错、不使能、不发送目标。检查反馈健康且持续更新，核对关节单位为 rad。
若身份错误、反馈缺失或超时，先解决问题，不进入使能步骤。
若只检查一侧，可将 `--devices hands` 改为 `--devices left_hand` 或 `--devices right_hand`。

### 3. 开启输入链路，先干跑

按仿真章节启动所需的 PICO 和 Manus 输入链路，但**不要启动机器人 Viewer**。
双臂使用 PICO；左右手使用 Manus。PICO 的 A 键复位、地面锁定和全部标定都应在真机使能前完成。

```bash
cd /path/to/tianji_teleop

# 仅双手干跑，不连接双臂硬件。
python -m tianji_controller.run_teleop \
  --devices hands --duration 10

# 或：双臂＋双手完整干跑。
python -m tianji_controller.run_teleop \
  --devices all --duration 10
```

不带 `--inspect` 或 `--confirm-real` 时，默认是 **DRY RUN**：不加载硬件 SDK、不连接设备、不发送硬件命令。
两条干跑命令不要同时运行。没有输入时 `flags=0` 表示未就绪；不要把它视为可以使能。

### 4. 首次使能：建议逐侧测试灵巧手

确认只读检查和输入链路正常，在交互式终端执行：

```bash
cd /path/to/tianji_teleop

python -m tianji_controller.run_teleop \
  --devices right_hand \
  --confirm-real
```

只有满足下列条件，程序才提示按 Enter 使能：

- 所选设备 SN 正确，固件左右身份与选定侧一致。
- 20 个关节状态与诊断完整，反馈持续更新。
- 输入新鲜，目标没有越过限位。
- 本节的单手／双手独立模式使用自动回零启动，不要求手套与真机人工对齐；包含双臂时使用下一节的分阶段流程。

确认现场安全后，在提示中输入：

```text
Enter（直接按回车使能；再次按回车停止并失能）
```
预检及回车后的使能复检都会在硬件反馈读取完成后，获取最新控制包并使用当前时间检查新鲜度；
真实输入过期或反馈异常仍拒绝使能。SDK 的 `Servo error code=[0,...]` 仅表示错误码为零，
不代表已使能：应先看到 `Press ENTER to enable`，确认现场安全后按回车，再检查
`REAL OUTPUT ARMED`。未通过时查看 `NOT ENABLED:` 或 `REAL EXECUTOR STOP:` 的具体原因。
执行器连接 Marvin 后关闭本地 SDK 常规打印，避免 `SERVO... value=0` 和全零伺服码刷屏。
这不会关闭机械臂端日志，也不跳过反馈／伺服码检查；检测到的故障仍通过执行器错误信息报告。

使能后，每只手的 20 个关节从实测姿态以不超过 1.0 rad/s 的 setpoint 速度回零。
零目标已发出且各关节反馈距零不超过 0.15 rad 后，锁存此刻的 Manus 重定向帧，
用 0.5 s 线性插值从零到该帧，随后恢复实时跟随。回零超时为 5 s。
半秒插值阶段的速度由目标距离/0.5 s 决定，不受实时跟随的 1.0 rad/s 上限约束；
因此 0.5 s 是命令轨迹时长，不保证机械关节恰好在该时刻到位。
全程保留反馈、限位、跟踪误差与输入 watchdog 检查；失败则终止会话。

当前 `config/robot.json` 中左右手增益为 **kp=6.0、kd=0.1**，使能时写入所有 20 个关节。
历史 SDK 示例曾采用 3.0/0.05；该历史数值不覆盖当前配置，也不是现场调参建议。
连接阶段仍输出设备当前增益用于核对；历史读回的 12/0.3 不作为当前控制增益。
此自动回零与半秒过渡尚未验证带运动实机闭环。不要强行掰动机械手；未使能不代表可安全背驱。

右手测试完成后，先按 Ctrl+C 并确认释放，再单独测试左手：

```bash
cd /path/to/tianji_teleop
python -m tianji_controller.run_teleop \
  --devices left_hand --confirm-real
```

两侧分别验证后，可用 `--devices hands --confirm-real` 只控制双手，不连接双臂硬件。
同一双手会话中任意一侧输入或反馈失效，都会终止整个已使能会话，不会悄悄继续另一只手。

### 5. 双臂＋双手联合真机遥操

先停止单手或双手测试会话，确认释放完成。保持 PICO、Manus 输入链路运行，再执行：

```bash
cd /path/to/tianji_teleop

bash bash/run_teleop.sh --real
```

通过已激活 default overlay 的 `python -m tianji real --devices all --confirm-real` 启动时，
外层包装器打印 `run log: <绝对路径>`，在
`logs/real/<时间戳>-<唯一标识>/` 保存本次运行，不覆盖上次记录：

| 文件 | 内容 |
| --- | --- |
| `console.log` | 终端 stdout/stderr，包括执行器、SDK 和子控制器输出；终端仍实时显示 |
| `launcher_exit.json` | 启动命令、起止时间、退出码、收到的信号及控制台日志是否完整 |
| `session.json` | 所选设备、真机配置快照、退出原因、结构化故障和清理错误 |
| `controller_configuration.yaml` | 本次实际使用的控制器配置，包含真机覆盖值和实测初始姿态 |
| `flight_recorder.jsonl` | 退出前最多 10 秒、最多 2000 个采样及故障现场；每行一个 JSON |

轨迹记录区分控制器输入参考 `reference`、最近成功发送给硬件的目标 `sent`、
实测 `feedback`，并带阶段、单调时钟、包序号、输入就绪标志和反馈时间。
机械臂跟踪超限还会记录具体关节（例如 `Joint3_R`）、命令角、实测角、误差、
阈值、反馈年龄和控制周期间隔；`joint_index` 从 0 开始，双臂顺序为左 7 轴、右 7 轴。
这些记录只帮助诊断，不改变速度、保护阈值、Enter 授权或停止逻辑。

高频样本只进有界内存缓冲，硬件停止／释放流程完成或报告失败后才写盘。
帮助、参数解析失败等早期退出可能只有前两个文件；控制器未启动时没有控制器配置快照。
断电、`SIGKILL` 或进程原生崩溃可能来不及保存内存轨迹，需结合已有控制台日志和
`launcher_exit.json` 判断；磁盘写入失败会明确报告，不能把不完整记录当作完整日志。
日志目录已忽略版本控制，不自动清除旧运行；需自行管理磁盘空间。
下次失败后可直接提供打印出的日志目录，无需再粘贴整段终端输出。

直接运行 `python -m tianji_controller.run_teleop` 不会默认启用外层日志包装；
需要上述 console／launcher 日志时，在已激活 default overlay 的终端运行
`python -m tianji real --devices all --confirm-real`。`bash bash/run_teleop.sh --real` 直接启动执行器，
不要仅凭入口名字假定存在外层日志；以终端实际打印的路径及生成文件为准。

`all` 指双臂＋左手＋右手，三路输入和反馈均需就绪；只测试双臂可使用
`python -m tianji real --devices arms --confirm-real`（需先激活 default overlay）。

若显示 `NOT ENABLED: left_hand: source input is not fresh/ready`，且控制器统计
`hand_datagrams=0`，表示没有收到 Manus 重定向 UDP 输入，不是 Hand2 硬件未连接。
在独立终端运行 `bash bash/run_manus.sh --user <实际佩戴者>` 并保持运行，先完成上节干跑检查。
启动时的 `Enter 1...` 是操作说明，不表示预检已通过；等待终端出现 `WAITING | 未使能 | Enter...`
后，才在启动执行器的终端按 Enter，不要在预检期间提前或连续按回车。
首次预检默认 30 秒超时，超时退出后需重新启动执行器。
非只读检查模式会先确认控制器可执行文件存在并独占绑定指令端口，再进行采集 prepared 检查（仅 `--data`）、显示窗口和硬件会话；
未编译控制器或端口被占用时，不连接设备。`--inspect` 不依赖控制器文件或指令端口。

包含双臂时采用以下流程，不再要求操作者先手动匹配每个关节：

| 当前阶段 | Enter 的作用 |
| --- | --- |
| WAITING（未使能） | 第一次确认：锁定当前目标，使能后从实测姿态慢速对齐 |
| ALIGNING（慢速对齐） | 中止，直接停止和失能，不提前开始遥操 |
| READY（已到位并保持） | 第二次确认：连续切换到实时遥操 |
| TELEOP（实时遥操） | 保持输入和机械臂静止后第三次确认：停止跟随，双臂平滑回指定 HOME |
| HOMING（回位） | 中止，直接停止和失能 |
| HOME_REACHED | 实测确认到位后自动失能、退出 |

终端按阶段显示状态及下一步操作，不重复刷关节角。未使能时始终显示 `WAITING`，
即使当前姿态已接近目标，也必须经过第一次 Enter 授权和实测稳定判定。
只有已使能且对齐完成才显示 `READY | 已对齐并保持 | Enter: 开始实时遥操`。

第一次 Enter 捕获的目标不会追逐后续输入变化；对齐完成后保持，等待第二次确认。
在包含双臂的分阶段模式下，Hand2 也从实测姿态直接慢速到达锁定目标，
不执行单独手部模式的回零／半秒插值。第二次 Enter 后，从上一条已发送指令连续限速跟随。
READY 必须同时满足指令已到目标、所有选中设备实测误差在各自 `alignment_rad` 内并持续稳定，
不是只凭模型或计时器判断；漂移会撤销 READY。
静止速度由连续实测位置与反馈时间戳估算，须不超过
`staged_motion.settle_speed_rad_s`（默认 `0.03 rad/s`），并在新反馈上持续
`settle_time_s`。重复反馈不能累计到位时间；反馈回退、过长间隔或运动会重置静止窗口。
第二次 Enter 也复查静止条件；独立回 HOME 入口使用相同判定。
回 HOME 前还要求双臂指令停止变化，不能把移动中的遥操指令直接接到零初速轨迹；
未满足此条件时请求被拒绝并走现有停止／失能流程。微小但持续变化的输入也可能阻止此切换，
此时应先安全退出遥操，释放设备后使用独立 `bash/run_home.sh`，不要放宽安全阈值强行切换。

只读 Viewer 清理时的非零退出、ERROR 或退出超时现在会计入会话 `cleanup_errors`，
并使入口返回失败状态；硬件停止/释放先于 Viewer 清理。历史修复报告记录 170 项软件回归，
当时尚未重新验收实际图形退出，GLFW 警告根因仍待处理；该记录不作为本轮测试数字或当前窗口验收结论。

正常回 HOME 时只移动双臂，双手保持最后指令姿态，不主动张开。
HOME 到位判定只针对双臂；双手仍须通过健康和新鲜度检查，
但不会要求受接触阻挡的手指重新满足启动对齐误差。
到位后所有选中设备失能；失能可能失去夹持力，必须先确认负载安全。
整个对齐、等待、遥操和回位阶段都保留输入新鲜度、epoch、反馈健康和限位检查。
机械臂保留跟踪误差退出；左右 Hand2 不再因运行时“反馈未跟上上一条指令”而退出，
被阻挡时也不会靠这项误差自动停止。启动对齐、单独手部模式的回零/到位超时检查仍保留；
手部 `tracking_error_rad` 仅继续用于该模式的启动 ramp 到位容差，不再作为运行时退出阈值。
回位期间也应保持 PICO／Manus 输入链路运行；断流、故障或关闭窗口时立即停止，不继续回位。

控制周期包含反馈读取、安全检查、发送和状态更新的耗时；默认 200 Hz 下，只等待本轮剩余的 5 ms 预算，
不再在工作完成后额外固定睡眠 5 ms。工作超时则开始新周期，不累计欠账、不突发补发；
原有限速、输入/反馈新鲜度及机械臂跟踪误差检查保持不变，实际频率仍受 SDK 和操作系统调度限制。

Manus 重定向结果由独立线程统一采样为双手 100 Hz；控制器以 10 ms 名义缓冲，
按发布时间戳线性插值为 200 Hz 目标，再交给真机执行器的限速和安全检查。
无插值区间时保持端点，不外推；重复目标保留各手原始输入时间，不能掩盖单手断流。
线性插值不替代速度限制，超过现有速度上限时实际指令仍会限速。
当前使用 TJH2 v2（364 字节）；协议升级后须构建控制器并重启 `bash/run_manus.sh` 与遥操/采集进程，
旧协议不能混用。`kp/kd`、电流上限和速度上限不因插值而改变。

所有退出路径先向全部设备发出停止请求，再逐个关闭连接和释放 SDK 资源，
避免某一路的关闭等待延迟其他设备收到停止请求；单路失败仍继续处理其余设备并报告。
这不使阻塞的 SDK 停止调用变成可抢占操作，物理急停仍必须随手可及。

#### 独立双臂回 HOME（不操作 Wuji Hand）

先停止正在运行的真机遥操／采集会话，等待设备释放，再从项目根目录执行：

```bash
bash bash/run_home.sh --dry-run  # 仅校验配置、显示目标，不连接硬件
bash bash/run_home.sh            # 授权使能／接管双臂，慢速回 HOME 后失能
```

该入口只连接 Marvin 双臂，不连接、使能、归零或失能任何 Wuji Hand，也不需要 PICO／Manus 输入。
双臂均未使能时先使能；双臂均已使能且健康时直接接管，先以实测姿态覆盖旧目标，再慢速回位，
不会先失能再重新使能。接管后本会话负责停止和失能；普通遥操／采集仍拒绝接管已使能设备。
单臂使能、另一臂未使能的混合状态仍拒绝操作，不自动清错或修复设备状态。
HOME 由 `config/robot.json` 的 `staged_motion.home_config` 指向
`src/teleop_outputs/tianji/tianji_description/config/home.yaml`，启动时加载左右臂目标；安装资源由 description 包统一提供。
速度取 `staged_motion.maximum_speed_rad_s` 与机械臂限速中的较小值（当前 `0.1 rad/s`），
加速度受 `staged_motion.maximum_acceleration_rad_s2` 限制（当前 `0.2 rad/s²`）。
先保持接管的实测姿态，收到新鲜且稳定的静止反馈后，再以共享进度的五次轨迹回位。
平滑起止会比原先独立匀速限幅耗时更长；若计算出的运动与整定时间无法满足原有 `60 s` 超时，
在开始轨迹前拒绝，不先移动到超时再停。实测到位并稳定后停止、失能并释放双臂；
Ctrl+C／SIGTERM 或故障则停止，不继续回位。
机械臂反馈新鲜度、健康、位置边界、跟踪误差及回位超时检查仍生效，不默认放宽限位执行越限恢复。
位置上下限检查对双臂及双手所有关节统一使用 `0.01 rad`（约 `0.573°`）余量，
目标角和实测反馈均适用；不改变模型限位、速度限制或跟踪误差阈值。
厂商 `FxRtCSDef.h` 定义 `101` 为切换到位置模式、`109` 为切换到 idle 的过渡状态。
仅在本程序发起使能后的 2 秒窗口内允许 `101`，期间仍检查反馈新鲜度、错误及位置边界，
并且必须实测双臂均到达 `1` 才能开始回位／遥操。失能必须实测到达 `0` 才算完成；
持续 `101/109` 表示切换未完成，不能当作健康运行或成功失能，也不自动清错重试。
与同配置遥操共用独占指令端口，端口被占用时在连接硬件前拒绝；不要与其他机器人控制程序同时运行。
需要外层终端日志时，在 default overlay 中显式执行 `python -m tianji home --confirm-real`，
日志写入 `logs/home/`；`bash/run_home.sh` 直接运行回位模块，不承诺外层 console 日志。
可用 `--config PATH` 指定另一份本项目格式的配置。
这是关节空间回位，不规划避障路径；执行前清空双臂运动路径，并确保物理急停可用。

#### 真机实测与目标窗口

- 实色模型来自硬件关节反馈，只做正运动学显示，不用动力学仿真冒充真实状态。
- 青色半透明模型是当前阶段目标：WAITING／TELEOP 为当前有效输入；
  ALIGNING／READY 为第一次 Enter 锁定的目标；HOMING 为 HOME 和保持的手指姿态。
- 缺失的设备反馈隐藏为 UNKNOWN；窗口快照超过 150 ms 时显示 STALE 并隐藏实测模型，
  不把历史姿态标成当前状态。目标仍作为已标记过期的参考保留。
- 手指的空间位置依赖机械臂姿态；缺少机械臂数据时，对应实测／目标手指也隐藏为 UNKNOWN，不借用旧臂姿态。
- 绘制运行在独立进程，最新状态通过有界、非阻塞通道传递，不占用 PICO／Manus 输入端口。
- 关闭窗口或窗口进程失败会触发停止和失能，不触发 HOME。

双臂控制模型初态仍由实测关节写入临时配置，原算法 YAML 不修改，
继续使用 `model_reference` 算法。慢速对齐／回位是关节目标限速，不是笛卡尔直线路径或自动避障；
第一次 Enter 和正常回位前均需确认整条运动路径无障碍，急停可用。

#### 指定 HOME

`config/robot.json` 的 `staged_motion.home_config` 指向共用 Home 文件，保存以下 HOME（各侧 Joint1→Joint7，单位 rad），
启动前会验证维数、有限值和关节限位，不会替换为全零或启动姿态：

```python
left  = ( 0.9599310886, -1.1344640138, -1.2217304764, -1.0471975512,  1.0471975512, 0.0, 0.0)
right = (-0.9599310886, -1.1344640138,  1.2217304764, -1.0471975512, -1.0471975512, 0.0, 0.0)
```

慢速对齐／回位使用共享进度五次轨迹：速度上限 `0.1 rad/s`（同时不超过各设备原有上限），
加速度上限 `0.2 rad/s²`；峰值同时受这两项约束，若无法在超时内完成则在运动前拒绝。
到位稳定时间为 `0.2 s`，每次对齐／回位超时为 `60 s`。这些值可在 `staged_motion` 中配置。

默认硬件边界限制：

| 参数 | 默认值 |
| --- | --- |
| 控制器速度比例（当前配置） | `1.0`；与下列硬件／执行器限速独立 |
| Marvin 速度／加速度比例 | `10% / 10%` |
| 双臂 setpoint 最大变化速度 | `0.5 rad/s` |
| 左／右手各自 setpoint 最大变化速度 | `1.0 rad/s` |
| 命令 watchdog 阈值 | `0.15 s` |
| 双臂／双手反馈 watchdog 阈值 | `0.30 s` |

这些是设备保护限制，不替换本项目的 IK 或重定向算法。

### 6. 停止与故障处理

- **正常回位退出（包含双臂）：**在 TELEOP 阶段按 Enter，等待 HOMING、HOME_REACHED 和设备释放。
- **直接停止：**Ctrl+C、关闭窗口、`bash bash/stop.sh` 或对齐／回位中按 Enter，均停止并失能，不追加 HOME 运动。
- **异常运动、碰撞风险或软件无响应：立即按硬件急停。** 不要只等待终端退出。
- 输入断流超过允许窗口、无效输入、反馈异常、越限、跟踪误差或控制器退出会终止已使能会话。
  不再提供旧后端断流宽限或自动重新使能；故障需显式退出、检查并重新授权。
- Hand2 非零诊断码由 SDK 静态目录按严重级别解析：`Warning` 不单独触发停机，
  会输出左右手、SN、NID、故障名及厂商处理建议，并保留在反馈 `detail` 中。
  `DeferredStop`、`ImmediateStop`、`Fatal`、未知码或解码失败仍锁存停机。
  警告不能绕过反馈过期、非有限值、关节失能和跟踪误差检查。
  `Enc1BitRate`（code=3）表示编码器数据质量警告；软件分级修复不代表硬件问题已解决。
- **使能期间不要按 PICO A 键、重新标定或重启输入驱动。** reset/resync 会撤销双臂运动授权，恢复需要重新启动并预检。
- 如果输出显示清理无法确认，先确认设备已实际停止，不要直接重启控制。
- 真机停止后，再按仿真章节的停止步骤关闭 Manus 和 PICO 输入链路。

### 后续实机验证边界

离线回归与只读预检不能证明实际运动安全。首次带运动测试仍需核实：

- 设备关节方向、零位及当前安装是否与模型一致。
- Hand2 使能瞬间的目标保持行为。
- 反馈序号、时间戳持续性及固件独立断流保护。
- SDK 停止／失能与物理急停的实际响应。

如果固件不在使能前提供反馈，程序会拒绝继续，不会先使能或用零值伪造反馈。

---

## 附录：PICO 标定命令

完整顺序为每侧 **TCP → 掌心到手腕 → 上臂／前臂骨长**。
标定只在真机执行器未使能时进行。

终端 1 单独启动驱动，并保持运行；不要与完整 PICO 启动脚本重复运行：

```bash
pixi run -e default bash bash/start_pico_driver.sh
```

驱动与标定器默认均使用 Jazzy／Fast DDS、`ROS_DOMAIN_ID=120`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`。
激活脚本清除旧 `ROS_LOCALHOST_ONLY`，不 source Humble 或旧 tracking overlay。
若显式覆盖，两个终端必须保持一致；旧进程不会因脚本更新而自动改变环境。
TCP 标定订阅 `/pico/pose/{left,right}_hand` 并同时需要 `/pico/pose/head`。
出现“未收到新鲜数据”时，先核对两个进程的 ROS 域和 localhost 设置，不要直接改话题名。

按一次 A 并完成地面初始化后，在终端 2 按顺序执行，每一步成功后再继续：

```bash
pixi shell -e default
source bash/environment.sh

bash bash/calibrate_pico_arm.sh left tcp --user zjx
bash bash/calibrate_pico_arm.sh left wrist --user zjx
bash bash/calibrate_pico_arm.sh left geometry --user zjx

bash bash/calibrate_pico_arm.sh right tcp --user zjx
bash bash/calibrate_pico_arm.sh right wrist --user zjx
bash bash/calibrate_pico_arm.sh right geometry --user zjx

bash bash/calibrate_pico_arm.sh status --user zjx
```

也可以使用以下顺序引导入口**替代**上面六条标定命令，不要重复执行两套：

```bash
bash bash/calibrate_pico_arm.sh left all --user zjx &&
bash bash/calibrate_pico_arm.sh right all --user zjx
```

完整 TCP 标定包含姿态。历史 `calibrate_pico_palm_orientation.sh` 只微调**全局标定**，
不会更新人员草稿或已发布版本；该旧脚本现归档，不作为当前操作命令。
当前人员标定需通过上述完整 TCP 流程更新并重新发布相依制品，不在运行中的遥操会话修改标定。

历史姿态微调的站姿要求：身体直立、头朝前，双臂向正前方水平伸直，掌心相对、手腕中立；按空格后保持约 2 秒静止。
仅微调姿态不改变 TCP 平移、手腕距离或骨长。重做完整 TCP 后，需要重做对应侧手腕和骨长；重做手腕后，需要重做对应侧骨长。

标定结束后先停止单独驱动，再切换到完整 PICO 链路。

## 进一步说明

- [PICO 标定与输入会话](docs/pico-simple-calibration.md)
- [双臂控制器与遥测说明](src/teleop_outputs/tianji/tianji_controller/native/README.md)
- [Wuji 重定向说明](src/teleop_outputs/wuji/wuji_retargeting/README.md)
- [真机配置](config/robot.json)
