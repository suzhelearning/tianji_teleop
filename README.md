# 天机遥操

工程保留 PICO＋VR 手柄、PICO2 裸手／手势识别、Manus 和外骨骼输入，以及仿真、真机和采集入口。
下面先介绍最新的 **新人员标定 → 骨架检查 → DLS 双臂仿真**，其他路线也可直接按各节启动。

| 使用路线 | 首页操作入口 | 范围 |
|---|---|---|
| PICO＋VR 手柄 | [人员标定](#2-新人员标定并显示骨架)、[DLS 仿真](#3-启动机械臂仿真) | 默认 DLS＋Ruckig，手部接收默认开启 |
| PICO2 裸手／手势识别 | [裸手遥操](#pico2-裸手与手势识别遥操) | 双臂＋双手仿真，不支持真机 |
| PICO＋Manus | [Manus 联合遥操](#picomanus-双臂双手遥操) | 手套输入与 PICO 分终端运行 |
| PICO＋外骨骼 | [外骨骼替代 Manus](#外骨骼替代-manus) | 与 Manus 二选一 |
| 真实机械臂／灵巧手 | [真机遥操](#真机遥操) | 独立预检和人工授权 |
| Mocap／Regrind | [回放与推理说明](README-mocap.md) | 独立可选依赖，真机需单独授权 |
| 数据采集 | [采集入口](#数据采集) | 观察录制与真机采集分开 |

所有命令在仓库根目录运行。下文 `NEW_USER` 换成实际人员名，`1.70` 换成实际身高（米）。
第 2～4 节的双臂流程使用 PICO 头显＋VR 手柄，不需要 Manus 或额外 Motion Tracker。

已安装并完成本人标定后的常用命令：

```bash
# 终端 A（需要双手时）：Manus 使用本人已有的手套标定
pixi run manus --calibration-user MANUS_USER --check
pixi run manus --calibration-user MANUS_USER

# 终端 B：PICO 人员选择 + DLS/Ruckig 双臂 + 默认双手接收
bash teleop.sh --sim --user NEW_USER
# 不使用手套：bash teleop.sh --sim --user NEW_USER --no-hand-teleop
```

首次使用不要跳过下方安装、PICO 标定和 Manus 准备步骤。PICO 人员名与 Manus 标定名
可不同，但必须对应同一实际佩戴者；仓库代码不保证包含个人标定或录制数据。

## 1. 首次安装与编译

支持 Linux x86_64，需要 Pixi、联网及图形桌面。首次克隆须获取 Git LFS 管理的 SDK 文件；
完整安装准备见[安装参考](README-reference.md#精简副本一键安装与编译)。

```bash
git lfs install
git clone git@github.com:suzhelearning/tianji_teleop.git
cd tianji_teleop
git lfs pull

# 系统依赖，由管理员安装一次
sudo apt-get update
sudo apt-get install build-essential libudev1 libusb-1.0-0 zlib1g adb tmux

bash install.sh --check
bash install.sh
```

`install.sh` 使用仓库内 Pixi 环境并编译控制器、tracking、Manus 和外骨骼组件，
另创建供旧路线／真机／采集使用的根 `.venv`；不会启动设备。
当前标定与 DLS/Ceres 仿真入口由 Pixi 选择解释器，无需手动激活环境。

已有环境、更新相关代码后重新编译：

```bash
pixi run build
pixi run -e tracking build-tracking
```

不要复制其他电脑的环境、构建产物或个人标定；安装后也不要随意移动工程目录。

## 2. 新人员标定并显示骨架

先退出旧遥操执行端，再停止旧 PICO 输入；真机保持未使能。
戴好头显，打开 **VR 手柄数据采集 APK**，连接 USB、允许调试，固定手柄握持方式。

```bash
pixi run -e tracking setup-pico --user NEW_USER --height-m 1.70
```

也可运行 `pixi run -e tracking setup-pico`，按提示输入人员名和身高。
向导自动启动原始 PICO 驱动，**不要同时另开 pico-driver 或 pico.sh**。

1. 确认接受简化模型：实测左 TCP，右 TCP 镜像推导，双侧骨长按身高估计。
2. 左 TCP 位置：掌心参考点保持不动，明显改变手柄朝向，按提示采集四次。
3. 左 TCP 姿态：双臂向前水平伸直、掌心相对，再按空格并保持姿态完成采样。
4. 检查展示结果；提示“回车确认使用并显示骨架，输入 q 取消”时，回车发布。

发布后自动打开 **MuJoCo 人体骨架窗口**，不启动机械臂。
检查掌心位置、转腕方向，以及前伸、侧展、屈肘时的骨架表现。
采集失败或取消发布不会自动加载旧人员；发布成功但窗口启动失败需按报错排查。

标定假设、手动分步入口和失败处理见[简化 PICO 标定](docs/pico-simple-calibration.md)。

## 3. 启动机械臂仿真

骨架检查后，另开终端，在同一工程目录执行：

```bash
bash teleop.sh --sim --user NEW_USER
# 等价入口：pixi run sim --user NEW_USER
```

默认 **Franka DLS＋Ruckig**，在线计算 IK 和轨迹平滑。
启动器校验该人员已发布的标定和输入新鲜度；匹配的输入会话可复用，
没有输入会话则启动，人员或版本冲突时拒绝，不静默换人。

此时分别有 **人体骨架窗口** 和 **机械臂仿真窗口**。
点击机械臂窗口使其获得键盘焦点：

| 按键 | 操作 |
|---|---|
| S | 输入有效且处于允许的静止状态时，平滑接入遥操 |
| H | 先制动，再平滑回 Home，完成后等待 |
| P / 空格 | 停止跟随并保持 |

回 Home 未结束时 S 可能被拒绝；等待 `HOME_REACHED` 后再按 S。
不要把骨架窗口当作机械臂按键窗口。

默认 DLS/Ceres direct 仿真已开启手部接收，可接入独立启动的 Manus 双手。
无需再写 `--hand-teleop`；仅双臂使用 `--no-hand-teleop`，不占用手部端口。
没有 Manus 数据时手指保持，不妨碍双臂接入；Manus 不会自动启动。
两种模式均不导出硬件指令，不等于动力学或真机验收。
故障处理与详细状态说明见[DLS/Ceres 交互仿真](control/docs/verification/ceres_interactive_sim.md)。

## 4. 结束与下次启动

先结束机械臂仿真。带 `--user` 启动时，**本次仿真新建的 PICO 会话会随退出自动关闭**；
如果复用了已有会话（例如标定向导启动的骨架），会保留它，结束时再手动停止：

```bash
pixi run -e tracking stop-pico
```

自动清理只匹配本次仿真的会话标记，不停止其他／后来重建的 PICO 会话，也不停止 Manus。
不带 `--user` 的启动方式不管理输入生命周期。手动停止优先使用上面的 Pixi 命令，
避免系统终端缺少 tmux 或 ROS 环境。Ctrl+C 和启动失败也会尝试清理本次新建的输入；
强制杀进程／断电无法保证自动清理，残留会话需检查后手动停止。
收到退出信号后给子进程 5 秒正常退出时间，超时后仅对本次子进程组执行 TERM／KILL；
强制结束可能导致录制不完整，不能把该次录制视为正常完成。

同一人员、标定与握持方式未变时，下次无需重新标定，直接运行：

```bash
bash teleop.sh --sim --user NEW_USER
```

换人、改身高或改变手柄固定方式后，先停止旧会话，再重新标定并检查骨架。
不要同时运行多套 PICO 输入、仿真和真机执行器。

## 身高与体型适配

简化标定按身高估计骨长；共享根映射再根据输入肩宽和臂展调整尺度，不固定为 1.62 m。
但模板骨长不是实际测量，不能保证不同臂长比例的人有完全相同的手感。

1.62 m 录制的 11 组合成测试中，等比例 1.45～1.95 m 的映射目标几乎一致；
单独改变肩宽或骨段比例会产生厘米级目标差异。全部组最终几何闭合，
**不代表真人换人、IK、碰撞或真机验收通过**。
详见[合成人体映射报告](control/docs/verification/synthetic_body_mapping_20260920.md)。

## PICO2 裸手与手势识别遥操

头显使用 **裸手跟踪 APK**，不是 VR 手柄 APK。先停止旧 PICO、Manus、外骨骼和执行器；
本路线不运行 `setup-pico`、`pico.sh` 或 Manus，不使用上面的手柄 TCP 标定。

```bash
# 首次／原生代码更新后构建
pixi install --locked --manifest-path pico2_hands/tools/wuji_hand_native/pixi.toml
bash pico2_hands/build_native.sh

# 连接头显并授权 USB 调试后
adb devices -l
adb forward tcp:10002 tcp:10002
bash pico2_sim.sh
```

默认显示双臂和双 Hand2，使用原版 V131 IK；只测试机械臂可加 `--disable-hands`。
S 接管；C 可选前伸 X/Z 标定；H 回 Home；Q／Ctrl+C 回 Home 并退出。
窗口／终端显示张手、握拳、捏合等观察标签，**手势识别不自动使能或停止**。
目前仅支持仿真，抖动和跟踪效果仍待验收，不能直接用于真机。
Python 依赖、录制参数和故障行为见[PICO2 裸手说明](pico2_hands/README.md)。

新增独立的**身高＋C 共享根映射＋Franka DLS/Ruckig**模式（仅仿真，现场待验收）：

```bash
pixi run --locked build
bash pico2_sim.sh --mapping-mode shared-root --height-m 1.62
```

替换实际身高；双臂前伸、双手间距约肩宽、掌心相对，面向前方按 C 保持约 1 秒，
成功后按 S。H 仅回双臂、手指保持；短时丢帧制动后可有界自动续接，
长时断流、重连、主动 P/H 仍需人工恢复；P／空格也可在 H 回程中取消回程并制动保持，
Q／Ctrl+C 退出回程不响应暂停，规则见下方新模式说明。
这里用头显和 C 建立虚拟人体根，用身高估计肩宽／臂长／腕掌距离，
复用共享根掌心仿射公式及 DLS 模型和在线 Ruckig；没有实测肩肘，不宣称全身骨架等价。
输入身份重连后会清除 C；重新 C、静止按 S 时会建立新的求解时间戳 epoch，允许设备重新计时。
旧 `bash pico2_sim.sh` 行为不变。详细边界见[新模式说明](pico2_hands/README.md)。

## PICO＋Manus 双臂＋双手遥操

### DLS＋Ruckig 双臂＋Manus（新接入）

先按第 2 节完成 PICO 标定。Manus 需要单独的左右 `.mcal` 文件，PICO 标定不会生成它们。
首次／更新手部原生代码后执行以下准备，Manus 使用仓库内独立 Pixi Python 3.11 环境，
ROS 使用同仓库 tracking SDK，不再要求根 `.venv`：

```bash
# SDK 若仍是 Git LFS 占位文件，先取回真实库
git lfs pull --include="manus/ManusSDK/lib/libManusSDK_Integrated.so"
pixi run prepare-manus
```

该步骤只安装／编译依赖，不连接手套。`pixi run build` 仅编译双臂控制器，不能替代它。
原有 `bash manus.sh` 的显式 Python／旧环境入口仍保留；推荐使用以下 Pixi 入口。
两个终端分别运行：

```bash
# 终端 A：只读列出具备双侧文件的 Manus 标定名
pixi run manus --list-calibration-users
# 可选：准备好本人标定后，只检查运行环境和模型，不启动设备
# pixi run manus --calibration-user MANUS_USER --check
# 将 MANUS_USER 换成实际佩戴者对应的手套标定名
pixi run manus --calibration-user MANUS_USER

# 终端 B：PICO 人员 + 默认 DLS/Ruckig 双臂 + 手部接收
bash teleop.sh --sim --user NEW_USER
# 等价：pixi run sim --user NEW_USER
# 仅双臂：bash teleop.sh --sim --user NEW_USER --no-hand-teleop
```

`--calibration-user` 显式选择 Manus 标定，不依赖旧 `profile.yaml`，不修改人员配置。
已有完整档案时，终端 A 也可用 `bash manus.sh --user NEW_USER` 保留原映射选择。
不得同时启动两套 Manus 或外骨骼；仿真不自动启动／停止 Manus，退出后在其终端 Ctrl+C。

点击机械臂窗口按 S 后双臂、双手跟随。P／空格停止跟随；H 仅让双臂回 Home，
手指保持。WAIT、制动、Home、HOLD、FAULT 均不跟随手套；重新 S 后只接受新样本。
单侧手部失鲜时该侧保持，恢复新鲜数据后在 TELEOP 内继续；手部失鲜不单独停止双臂。
Ruckig 用于双臂，手指复用现有插值、时效检查和模型限位，不宣称手指经过 Ruckig。

默认手部端口为本机 `16000`；改端口时，Manus 加 `--port PORT`，仿真加 `--hand-port PORT`，
并与 PICO 端口不同。本模式仍不支持动力学／真机，默认会话日志也不是完整 Manus 原始录制。
该接线已进行离线测试，真实 Manus＋PICO 联合仿真仍需现场验证。

### 旧 SPARK 双臂＋Manus

这一路 PICO 控制双臂，Manus 控制双 Hand2。需要正确的人员档案及独立的 Manus 标定；
`setup-pico` **不会生成 Manus 标定或旧完整 `profile.yaml`**。
先按[完整人员档案说明](README-reference.md#人员档案与标定版本)准备已发布的 PICO 配置，
核对档案的 `manus.user` 及对应的左右 `.mcal` 文件。

安装完成后，以下三个终端分别运行；`NEW_USER` 必须是实际已有完整档案的佩戴者：

```bash
# 终端 A：PICO 输入（与 setup-pico 启动的会话二选一，不重复启动）
bash pico.sh --user NEW_USER

# 终端 B：Manus 已开机、配对，保持此终端运行
bash manus.sh --list-users
bash manus.sh --user NEW_USER

# 终端 C：双臂＋双手仿真，必须显式选择旧 SPARK 路线
bash teleop.sh --sim --ik-backend spark
# 若需要动力学，改用：
# bash teleop.sh --sim --ik-backend spark --simulation-mode dynamics
```

此处是旧 SPARK 路线，不能省略 `--ik-backend spark`；新 DLS 路线默认接收手部输入，
无需再加 `--hand-teleop`。
先小幅检查双臂，再保持手腕稳定逐指检查左右对应和弯曲方向。
结束时先退出执行端，再在 Manus 终端 Ctrl+C，最后 `pixi run -e tracking stop-pico`。
完整接线、环境和诊断见[联合仿真参考](README-reference.md#一仿真遥操作)。

## 外骨骼替代 Manus

沿用上一节 PICO 和 SPARK 执行端，仅将终端 B 换成外骨骼输入：

```bash
# 已有主环境时补装／更新外骨骼原生组件
bash install.sh --exoskeleton
bash exo.sh --check-config
bash exo.sh
```

默认双手；单侧追加 `--hand left` 或 `--hand right`。
外骨骼与 Manus 不得同时发送手部输入。配置、零位及方向需核对，
离线配置通过不等于实机验收；详见[手部输入参考](README-reference.md#2-启动-manus-灵巧手输入)。

## 真机遥操

真机是独立路线，**不是将默认 DLS 仿真直接连到硬件**；默认仍使用原真机后端。
PICO2 裸手不支持真机。先退出仿真执行端，核对设备 IP、左右 SN、限位、急停及运动空间，
按[真机完整流程](README-reference.md#二真机遥操作)完成设备配置。

```bash
# 只读预检：不使能、不发送目标
.venv/bin/python real_robot/run_teleop.py --devices all --inspect

# 所需 PICO 和 Manus／外骨骼输入就绪后，先干跑（不连接硬件）
.venv/bin/python real_robot/run_teleop.py --devices all --duration 10

# 现场安全确认后才运行：双臂＋双 Hand2
bash teleop.sh --real
```

`all` 指双臂和左右两手，三路输入／反馈均需就绪。
默认联合流程在**启动终端**等待提示后依次按 Enter：慢速对齐 → 遥操 → 回 Home 后失能；
不要提前连续按键，也不要套用仿真窗口 S/H。
仅双臂可用 `.venv/bin/python real_robot/run_teleop.py --devices arms --confirm-real`，
仍需先对 `arms` 做只读预检和干跑。mapped-palm 的 C 标定与授权流程另见[专用说明](docs/mapped-palm-real-readiness.md)。

输入失鲜、身份／反馈异常时不要绕过门控；先停止并排查。
运动使能期间不要按 PICO A 键、重新标定或重启输入。
正常退出确认设备释放后再停输入；紧急危险使用现场急停。
`logs/real/` 保存会话日志。软件测试或仿真通过不代表当前设备的真机闭环已验收。

## 数据采集

观察录制和带真机执行的采集是不同入口，不互相授予运动权限：

- PICO2 裸手仿真录制：`bash pico2_sim.sh --record recordings/pico2_sim/NEW_SESSION.h5`，
  先创建父目录，每次使用新文件名；完整步骤见[裸手录制说明](pico2_hands/README.md)。
- 仅观察的数据集录制、相机和 R/S/D 操作见[观察采集](README-reference.md#观测数据集采集r--s--d)。
- 真机任务采集：`bash teleop.sh --data --task TASK`，沿用真机预检和人工授权，
  默认写入 `dataset/`，不是只读观察入口。

## 共用 Home 与 Mocap／Regrind

SPARK／mapped-palm 部署入口和真机受保护回位共用
[`control/mapped_palm/config/home.yaml`](control/mapped_palm/config/home.yaml)。
DLS／Ceres 仿真仍使用各自配置中的 Home，不因合并改变其速度或授权流程。

远端新增的 H5 回放、Motive 接入和 Regrind 推理入口保留，详见
[Mocap／Regrind 操作说明](README-mocap.md)。这些 Pixi task 仍调用独立配置的 `.venv`，
不是仅安装默认 Pixi 环境即可运行；依赖、模型与设备授权按该说明单独准备。

## 进阶说明与测试

以下是独立选择，不需要为上述双臂仿真全部启动：

| 需求 | 入口／说明 |
|---|---|
| Ceres LM＋Ruckig 仿真 | `pixi run sim --user NEW_USER --ik-backend ceres` |
| 旧 SPARK、双臂＋双手仿真 | [完整仿真参考](README-reference.md#一仿真遥操作)；显式选 `--ik-backend spark` |
| mapped-palm 后端 | [移植与启动说明](docs/mapped-palm-port.md) |
| PICO2 裸手仿真 | [独立入口](pico2_hands/README.md)，不支持真机 |
| 双侧实测标定、完整人员档案 | [人员档案参考](README-reference.md#人员档案与标定版本) |
| Manus／外骨骼输入 | [输入链路参考](README-reference.md#2-启动-manus-灵巧手输入)，一次只选一种 |
| 真机遥操 | [真机预检与授权流程](README-reference.md#二真机遥操作)，不沿用仿真放行结论 |
| 数据集／相机／录制 | [采集参考](README-reference.md#观测数据集采集r--s--d) |
| 控制器与原生测试 | [control/README](control/README.md) |
| 安装细节、目录与历史说明 | [完整参考页](README-reference.md) |

真机默认不随仿真 DLS 后端改变。`--user` 的上述自动启动方式仅适用于 DLS/Ceres
固定 PICO 端口 15000 的仿真，不能直接套用于 SPARK、mapped-palm 或真机。

软件回归命令（不等于现场验收）：

```bash
pixi run test-native
pixi run test-sim
pixi run -e tracking test-setup-pico
pixi run -e tracking test-pico-simple
```
