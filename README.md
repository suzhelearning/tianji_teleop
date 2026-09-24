# 天机遥操
## PICO＋Manus 双臂＋双手遥操

### 四终端启动（已完成本人标定）

各终端在本仓库根目录运行。`NAME` 必须对应实际佩戴者；切换前先停止执行器，再停止旧输入和相机。

```bash
# 终端 1：PICO 前台输入与骨架；保持终端打开
bash bash/run_pico.sh --user NAME

# 终端 2：Manus 双手 ROS 目标；保持终端打开
bash bash/run_manus.sh --user NAME

# 终端 3：三路相机、top 到 PICO 视频、RViz
bash bash/run_camera_views.sh

# 终端 4：仅在现场预检通过、前三路就绪后启动真机采集
bash bash/run_teleop.sh --data --task pick-and-place
```

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
# Manus 独立发布左右手各 20 维 ROS 目标；执行由仿真／安全执行器负责
bash bash/run_manus.sh --user MANUS_USER

# PICO 人员选择 + DLS/Ruckig 纯双臂仿真
bash bash/run_teleop.sh --sim --user NEW_USER --no-hand-teleop
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

bash bash/install.sh --check
bash bash/install.sh
```

`bash/install.sh` 按锁文件安装 default、control、arm-ros、cameras、manus、policy、spd 及独立外骨骼／Hand2
环境，分别构建 ROS 工作空间、原生控制器和 Manus；不会启动设备。
默认运行环境为 **ROS 2 Jazzy＋Python 3.12＋Fast DDS**。控制工具链和官方 RealSense
4.58.3 驱动保持隔离；新 Manus ROS 链在 default 中使用 `wuji-sdk==2026.8.31` 的
`RetargetSession`，只发布目标，不连接 Wuji。旧 manus/Pinocchio 环境不再用于该入口。
构建输出为 `build/<环境>`、`install/<环境>`、`log/<环境>`；公共原生程序在 `install/control/bin/`，
SPD 专用构建产物在 `install/spd/`，不借用 default 或外部 SPD 工作空间的 overlay。
不要 source 旧 tracking/Humble 或其他环境的 overlay。

已有环境、更新相关代码后重新编译：

```bash
pixi run build
```

只重建双臂 ROS 核心可用 `pixi run build-arm-ros`。它在独立 `arm-ros` 环境构建同一
DLS/Ruckig 控制循环，安装 `install/control/bin/tianji_arm_ros`；不混用 default 的 Eigen
或 Pinocchio ABI。正常 `pixi run build` 已包含此步骤。

不要复制其他电脑的环境、构建产物或个人标定；安装后也不要随意移动工程目录。

### PICO 头显应用（Git LFS）

安装包统一保存在 `apps/pico/`，由 Git LFS 管理，不随 PC 端构建自动安装到头显：

| 安装包 | 用途 |
|---|---|
| `pico_g1_teleop_wired_withcam.apk` | PICO 手柄遥操作、有线 PC 相机视频 |
| `pico_hand_tracking_adb.apk` | PICO 裸手跟踪，经 ADB 接入裸手仿真路线 |

克隆后先拉取实际 APK 文件，不能把 Git LFS 指针文件当作 APK 安装：

```bash
git lfs install --local
git lfs pull --include="apps/pico/*.apk"
```

安装前停止遥操作执行端、旧 PICO 输入和视频桥，保持机器人未使能；USB 连接头显并允许调试。
按实际路线选择一个安装包，`-r` 表示覆盖安装并保留兼容的应用数据：

```bash
adb devices -l
# 手柄 + 视频路线
adb install -r apps/pico/pico_g1_teleop_wired_withcam.apk
# 或：裸手路线
adb install -r apps/pico/pico_hand_tracking_adb.apk
```

多设备连接时使用 `adb -s SERIAL install -r ...`。签名不一致时不要直接卸载旧应用，
先核对安装包来源；安装完成后在头显中打开对应路线的应用。

后续更新用新版 APK 替换同名文件，再提交 APK 与必要说明；`.gitattributes` 的
`apps/pico/*.apk` 规则会将二进制内容存入 LFS。正常 `git push` 通过 LFS hook 上传对象，
远端必须支持 Git LFS 且有足够配额；不要只提交或复制指针而遗漏实际 LFS 对象。

手柄版追踪连接由头显主动发起：`ADB reverse tcp:9999 tcp:9999` → PC
`127.0.0.1:9999` 监听器。`run_pico.sh` 自动检查该映射；仅移除所选设备上精确匹配的
旧 `forward tcp:9999 tcp:9999`，其他设备或端口冲突直接拒绝，不改视频和裸手映射。
输入桥支持交付 APK 的逐关节、整包 float32 和半精度追踪帧，断线重连建立新 tracking epoch。

## 2. 新人员标定并显示骨架

先退出旧遥操执行端，再停止旧 PICO 输入；真机保持未使能。
戴好头显，打开 **VR 手柄数据采集 APK**，连接 USB、允许调试，固定手柄握持方式。

运行简化标定向导，显式指定实际人员名和身高：

```bash
pixi run setup-pico --user NEW_USER --height-m 1.70
```

向导自行启动原始 PICO 驱动，**不要同时另开 PICO 驱动或 `bash bash/run_pico.sh`**。
以下是向导内的步骤；需要双侧实测 TCP／腕轴／骨长时，改用[完整标定流程](README-reference.md#附录pico-标定命令)，不要混用两套流程。

1. 确认接受简化模型：实测左 TCP，右 TCP 镜像推导，双侧骨长按身高估计。
2. 左 TCP 位置：掌心参考点保持不动，明显改变手柄朝向，按提示采集四次。
3. 左 TCP 姿态：双臂向前水平伸直、掌心相对，再按空格并保持姿态完成采样。
4. 检查展示结果；提示“回车确认使用并显示骨架，输入 q 取消”时，回车发布。

发布后自动打开 **MuJoCo 人体骨架窗口**，不启动机械臂。
检查掌心位置、转腕方向，以及前伸、侧展、屈肘时的骨架表现。
采集失败或取消发布不会自动加载旧人员；发布成功但窗口启动失败需按报错排查。

标定假设、手动分步入口和失败处理见[简化 PICO 标定](docs/pico-simple-calibration.md)。

### 人员标定的唯一存放位置

PICO 与 Manus 人员标定统一放在工程根目录的 `profiles/<人员名>/`：

```text
profiles/zjx/
├── pico-simple/                 # PICO 标定版本与 active.json
└── manus/
    ├── zjxLeftMetaglovePro.mcal
    └── zjxRightMetaglovePro.mcal
```

新增 Manus 标定时，将本人实际生成的左右手文件放到 `profiles/NAME/manus/`，
文件名为 `NAMELeftMetaglovePro.mcal` 和 `NAMERightMetaglovePro.mcal`。不需要重新构建，
也不再复制到源码包或 `install/`。缺少一侧就不能启动，不会回退到其他人员或旧安装目录。

```bash
bash bash/run_manus.sh --list-calibration-users
bash bash/run_manus.sh --user zjx
# 等价人员选择：bash bash/run_manus.sh --calibration-user zjx
```

两种人员参数均直接选择对应目录，不需要额外的 `profile.yaml` 映射；两种列表参数
`--list-users` / `--list-calibration-users` 均列出完整的 Manus 双手标定。
修改标定后须先安全停止执行端，再重启 Manus 输入；运行中的进程不会热切换人员。
机器人硬件配置及外骨骼设备身份／零位配置不是人员标定，仍保留原设备配置目录。

## 3. 启动机械臂仿真

骨架检查后，另开终端，在同一工程目录执行：

```bash
bash bash/run_teleop.sh --sim --user NEW_USER
# 等价入口：pixi run sim --user NEW_USER
```

默认 **Franka DLS＋Ruckig**，在线计算 IK 和轨迹平滑。
启动器校验该人员已发布的标定和输入新鲜度；匹配的输入会话可复用，
没有输入会话则启动，人员或版本冲突时拒绝，不静默换人。

当前双臂生产通信为：

```text
PICO TCP/ADB → ROS 采集与校正 → pico_arm_input
  → /pico/arm_input (PicoArmInput)
  → tianji_arm_ros：原共享根映射 + Franka DLS + Ruckig
  → /tianji/controller/joint_targets (ControllerJointTargets)
  → Python 执行器的原安全门控 → SDK
```

普通仿真禁用最后的目标导出；真机／dry-run 执行入口才显式启用受限导出。
配置只读 `config/robot.json` 的 `pico_input_topic`、`joint_command_topic`；
默认 DLS 不再使用或接受 `pico_port`／`command_port`，不打开业务 UDP 15000／17000。
消息包含本机 boot/session、序号、epoch、源时钟及撤销代际，求解时间不能刷新已应用输入的年龄。
DDS 回调／发布在控制循环之外，latest-only 交接保留撤销与重置事件。原生失效后的输出
ready 撤销锁存至核心重启；执行器及 Home 使用跨域／跨话题的本机互斥锁，订阅不授予运动权限。
输入的配对、坐标／尺度映射和双臂算法未替换，也没有 ROS↔UDP 旁路进程。

更新后须先安全停止执行端，再停止并重启旧 PICO 输入；旧运行会话不会自动切换到新发布者。
历史 trace／benchmark 和显式历史对照工具仍可用 UDP，不是标准 DLS 的回退路径。

此时分别有 **人体骨架窗口** 和 **机械臂仿真窗口**。
点击机械臂窗口使其获得键盘焦点：

| 按键 | 操作 |
|---|---|
| S | 输入有效且处于允许的静止状态时，平滑接入遥操 |
| H | 先制动，再平滑回 Home，完成后等待 |
| P / 空格 | 停止跟随并保持 |

回 Home 未结束时 S 可能被拒绝；等待 `HOME_REACHED` 后再按 S。
不要把骨架窗口当作机械臂按键窗口。

默认 DLS direct 仿真接收 Manus ROS 双手目标；外骨骼须显式传 `--hand-source exoskeleton`。
仅双臂使用 `--no-hand-teleop`。Manus 不走旧 TJH2 UDP 接线。
没有有效手部输入时模型手指保持，不妨碍双臂仿真接入。仿真不导出硬件指令，不等于真机验收。
故障处理和详细状态说明见[原生 DLS 控制器](src/teleop_outputs/tianji/tianji_controller/native/README.md)。

## 4. 结束与下次启动

先结束机械臂仿真。带 `--user` 启动时，**本次仿真新建的 PICO 会话会随退出自动关闭**；
如果复用了已有会话（例如标定向导启动的骨架），会保留它，结束时再手动停止：

```bash
pixi run stop-pico
```

自动清理只匹配本次仿真的会话标记，不停止其他／后来重建的 PICO 会话，也不停止 Manus。
不带 `--user` 的启动方式不管理输入生命周期。手动停止优先使用上面的 Pixi 命令，
避免系统终端缺少 tmux 或 ROS 环境。Ctrl+C 和启动失败也会尝试清理本次新建的输入；
强制杀进程／断电无法保证自动清理，残留会话需检查后手动停止。
收到退出信号后给子进程 5 秒正常退出时间，超时后仅对本次子进程组执行 TERM／KILL；
强制结束可能导致录制不完整，不能把该次录制视为正常完成。

同一人员、标定与握持方式未变时，下次无需重新标定，直接运行：

```bash
bash bash/run_teleop.sh --sim --user NEW_USER
```

换人、改身高或改变手柄固定方式后，先停止旧会话，再重新标定并检查骨架。
不要同时运行多套 PICO 输入、仿真和真机执行器。

## 身高与体型适配

简化标定按身高估计骨长；共享根映射再根据输入肩宽和臂展调整尺度，不固定为 1.62 m。
但模板骨长不是实际测量，不能保证不同臂长比例的人有完全相同的手感。

2026-09-20 的历史实验使用 1.62 m 录制构造 11 组合成测试，等比例 1.45～1.95 m 的映射目标几乎一致；
单独改变肩宽或骨段比例会产生厘米级目标差异。全部组最终几何闭合，
**不代表真人换人、IK、碰撞或真机验收通过**。
详见[合成人体映射报告](src/teleop_outputs/tianji/tianji_controller/native/docs/verification/synthetic_body_mapping_20260920.md)；不是本次迁移重跑的验收结果。

## PICO2 裸手与手势识别遥操

头显使用 **裸手跟踪 APK**，不是 VR 手柄 APK。先停止旧 PICO、Manus、外骨骼和执行器；
本路线不运行 `setup-pico`、`bash/run_pico.sh` 或 Manus，不使用上面的手柄 TCP 标定。

```bash
# 首次安装独立 SPD 控制端环境；不会安装外部 SPD 仿真服务
pixi install --locked -e spd
# 首次／原生代码更新后构建（包括 DLS、Hand2 和 ROS 接口）
pixi run --locked -e spd build

# 连接头显并授权 USB 调试后；入口自动检查／补建默认 ADB 转发
bash bash/run_pico_hand_sim.sh --height-m 1.75
```
入口固定使用独立 `spd` 环境。本分区只向 `/home/current/syz/spd-syz` 的 SPD 仿真发送 ROS
目标；SPD 服务由该项目独立安装和启动，原生 Viewer 仅辅助观察，不是第二个执行目标。
默认 `127.0.0.1:10002` 的 ADB 转发自动复用或补建，无需手动执行 `adb forward`；
多设备须设置 `ANDROID_SERIAL`，冲突转发不会被覆盖。自测和自定义输入地址／端口不操作 ADB。
默认每 5 秒输出一条 `pico2_runtime_stats` 周期汇总，含输入／控制频率、P95/P99、
双臂／双手耗时和 BRAKING 原因计数；`--stats-interval-s 0` 关闭汇总，不逐帧刷屏。

裸手只保留 **身高模板＋共享根标定映射＋Franka DLS/Ruckig**；旧 V131、旧启动脚本和
`--mapping-mode` 已删除。`1.75` 必须替换为实际身高，不再有隐含人员或身高默认值。
默认向 SPD 仿真发布 `/spd/tianji_wuji2/v1/joint_command`
（`tianji_spd_interfaces/msg/JointCommand`，54 维 rad 目标），并由原生 Viewer 辅助显示。
`--headless` 只关闭显示、不停止 ROS 发布；`--disable-hands` 清除双手 ready 位。
此出口仅用于 SPD 仿真，不接入 Manus／外骨骼＋PICO 手柄的真机执行链。

普通启动：双臂前伸、双手间距约肩宽、掌心相对，面向前方按 **R** 保持约 1 秒，
标定成功后保持当前目标、显示 ready；再按 **S** 才开始持续跟随。运动中再次 R 会先制动，
再重新标定，成功后仍须 S。窗口和终端均支持 R/S；P／空格保持，H 双臂 Home；
**Q／Esc／关闭窗口／Ctrl+C** 直接退出，不发起 Home。C 不用于普通本地模式。
同连接失鲜时制动保持，恢复稳定有效输入后按原策略自动跟随；断连／重连必须重新 R 再 S。
SPD 的执行授权与采集独立管理，SPD 暂停、保存、回退均不暂停上游，也不请求上游 Home。
ROS 发布本身不是 SPD 授权，最后一次本地 publish 也不是接收端执行确认。

窗口／终端显示张手、握拳、捏合等观察标签，**手势识别不自动使能或停止**。
身高模板不是实测肩肘，不等于完整 VR 人体骨架；仅支持仿真，真实输入精度与实时性
仍需现场验收。离线可执行 `bash bash/run_pico_hand_sim.sh --height-m 1.75 --self-test`，
它使用真实 DLS/Hand2 worker，验证 R 后静止就绪、S 开始跟随及运动中重新标定后再次等待 S；不连接设备，ROS 强制隔离到测试域 121。
完整构建、录制和故障行为见[PICO 裸手说明](src/teleop_inputs/pico_hand/README.md)。

## top 相机画面传到 PICO

先运行已有的官方相机节点，再在另一个终端启动视频串流：

```bash
# 相机驱动已有实例时不要重复启动
bash bash/run_cameras.sh

# 另一个终端；头显通过 USB 连接并已授权调试
bash bash/run_pico_camera.sh
```

等价 Pixi 任务为 `pixi run pico-camera`。脚本从 `config/collect_real.json` 读取 top 角色，
只订阅 `/cameras/top/color/image_raw`，不另开 RealSense pipeline、不录制、不控制机器人。
默认最多 30 fps、4 Mbps；可用 `--bitrate 6M` 调整码率，`--timeout 30` 调整初始图像等待。

PICO 软件须启用支持 `OPEN_CAMERA` 的 PC 视频源，PC 地址填 **`127.0.0.1`**。
脚本自动检查／建立 `adb reverse tcp:13579 tcp:13579` 与
`adb forward tcp:12345 tcp:12345`，仅清理本次新建的视频映射，不改裸手输入的 `10002`。
已有 manager 管理视频映射时可加 `--no-adb`；多设备用 `ANDROID_SERIAL` 选择。
此脚本只支持有线回环视频，不按请求中的任意 IP 向外推送。
控制协议以自研 `PICO_2-PICO_G1_Teleop` 交付包的 `stream_to_pico.py` 为准：
4 字节大端**正文长度（不含长度头自身）**，随后依次为小端 int32 命令字节数、
UTF-8 命令、小端 int32 负载字节数和负载。`OPEN_CAMERA`／`CLOSE_CAMERA`
均使用这一帧格式，不需要 XRoboToolkit PC Service。

H.264 使用 PICO 的 4 字节大端长度头协议，按请求尺寸生成左右眼相同的 top 画面
（SBS，不是真双目深度）。源图像超过 250 ms 或视频发送阻塞会断开并报错，
不积压／回放旧编码帧；恢复输入后重新启动串流。top 未启用、未发布或画面格式不符时明确失败，
不会用黑图或冻结帧冒充相机。源码与编码器依赖随工作区安装，不依赖 Downloads 中的脚本。



终端 3 等待 `CAMERA_VIEWS_READY`；不要同时运行另一套 `run_cameras.sh` 或相机预览设备驱动。
终端 4 可替换为 `bash bash/run_teleop.sh --sim` 先验证仿真；纯仿真不要求启动相机。
仿真默认消费 Manus 双手目标，S 跟随、P／空格保持、H 双臂 Home；不发送真机指令。
采集模式首次 `r` 授权 Home 准备和张手，再次 `r` 冻结目标并缓慢对齐，录制确认后跟随；
`s` 保存、`d` 丢弃，均在制动、停稳及操作确认后回 Home 并张手。按键不带回车。
停止时先在终端 4 退出并确认机器人停止，再 Ctrl+C 停止终端 1–3。
四入口日志位于 `tmp/logs/{pico,manus,camera,teleop}/`；PICO 默认前台，不创建 tmux。

Manus 采集／适配／SDK Hand2 重定向不连接 Wuji 硬件；硬件写入只属于授权后的执行器。
已移除 rawviz 文本管道、私有重定向 worker 和 Manus TJH2 UDP 出口。

```bash
# 首次／原生代码更新后：只构建，不启动设备
bash bash/build_manus.sh
# NAME 必须替换为实际佩戴者的完整左右手标定名
bash bash/run_manus.sh --user NAME
```

`run_manus.sh` 只管理 Pixi/default 和 Jazzy overlay，然后执行：

```bash
# 已进入 Pixi 且 source bash/environment.sh 后
ros2 launch manus_bridge manus_hand2.launch.py user:=NAME
```

| 边界 | ROS 消息与话题 |
|---|---|
| Manus 采集 | `tianji_interfaces/msg/ManusGlove`，`/manus/raw/{left,right}` |
| 21 点骨架适配 | `tianji_interfaces/msg/HandLandmarks`，`/manus/landmarks/{left,right}` |
| 每手 20 关节目标 | `tianji_interfaces/msg/HandJointCommand`，`/wuji/{left,right}_hand/joint_commands` |

三段均为 BEST_EFFORT／KEEP_LAST 1／VOLATILE。每侧携带 `glove_id`、`side`、
`session_id`、`boot_id`、`sequence`、`source_monotonic_ns`、`revocation_generation` 和 `valid`；
时间在 SDK 原始骨架回调处取本机 CLOCK_MONOTONIC，并在适配／求解后原样保留，
不是手套内部采样时间。关节单位 rad，顺序为 thumb/index/middle/ring/pinky 各 S1～S4。
原始姿态使用右手 VUH XFromViewer、Z-up、世界坐标米；适配器保留该坐标，不额外镜像 Y。

无新源帧不重复刷新目标时间；单侧超时、无效骨架或发布者冲突会发布 `valid=false`，
无效数值使用 NaN，绝不当作补零命令。另一侧数据不会刷新失鲜侧。
启动／模型加载日志不代表有有效目标；需看到各侧 `published first fresh valid 20-joint commands`。

```bash
# 只读观察，不连接 Wuji 设备；需在同一 ROS 环境
ros2 topic echo /wuji/left_hand/joint_commands
ros2 topic echo /wuji/right_hand/joint_commands
```

调试可以只运行适配／求解节点：
`ros2 launch manus_bridge manus_hand2.launch.py user:=NAME start_acquisition:=false`。
该模式不启动 Manus SDK，仍校验所选人员档案，等待外部测试骨架；
离线验证应在 source 环境后设置 `ROS_DOMAIN_ID=121`，不要向生产域注入合成数据。
`--check`、`--host`、`--port` 和旧 `/hand_input`／UDP 接线已退役，不保留旧后端回退。
两只手仍须使用本人的 `.mcal`，不会复制参考仓库的标定、设备身份或自动使能设置。

仿真原生链与真机 Python 执行器分别接收 Manus ROS 目标；执行器保有唯一硬件写入权和安全门控。
逐条录制确认开始后才跟随；采集记录真机反馈，不把重定向目标当作实测状态。
构建、无硬件 DDS 和离线验证不代表真实手套、相机、头显或机器人现场验收。


## 外骨骼替代 Manus

使用同一 PICO 输入，将 Manus 换成外骨骼输入，并给仿真／执行端显式追加 `--hand-source exoskeleton`：

```bash
# 已有主环境时补装／更新外骨骼原生组件
bash bash/install.sh --exoskeleton
bash bash/run_exoskeleton.sh --check-config
bash bash/run_exoskeleton.sh
```

默认双手；单侧追加 `--hand left` 或 `--hand right`。
外骨骼与 Manus 不得同时发送手部输入。配置、零位及方向需核对，
离线配置通过不等于实机验收；详见[手部输入参考](README-reference.md#2-启动-manus-灵巧手输入)。

## 真机遥操

`--real` 和 `--data` 现与默认仿真统一为 **共享根掌心映射＋Franka DLS＋Ruckig**：
后端 `franka-dls`，算法 `pico_ee_franka_dls`，配置 `qp_ik_pico_shared_root_dls.yaml`，
`post_smoothing.mode=ruckig`。仿真和执行器只接受这个后端，其他双臂后端的实现、入口、
配置和构建依赖已删除。手部重定向仍独立，不套用双臂 DLS／Ruckig。

真机仍是独立的安全执行路线，**不是将仿真窗口直接连到硬件**。原生控制器只导出
本机 `ControllerJointTargets` ROS 参考，Python 执行器保留身份、反馈、限位、失鲜和分阶段人工授权。
原生受限输出开关不授予运动权限，普通仿真仍不导出硬件指令。
PICO2 裸手不支持真机。先退出仿真执行端，核对设备 IP、左右 SN、限位、急停及运动空间，
按[真机完整流程](README-reference.md#二真机遥操作)完成设备配置。

```bash
# 只读预检：不使能、不发送目标
pixi run -e default bash -c 'source bash/environment.sh; exec python -m tianji real --devices all --inspect'

# 所需 PICO 和 Manus／外骨骼输入就绪后，先干跑（不连接硬件）
pixi run -e default bash -c 'source bash/environment.sh; exec python -m tianji real --devices all --duration 10'

# 现场安全确认后才运行：双臂＋双 Hand2
bash bash/run_teleop.sh --real
```

`all` 指双臂和左右两手，三路输入／反馈均需就绪。
默认联合流程在**启动终端**等待提示后依次按 Enter：慢速对齐 → 遥操 → 回 Home 后失能；
不要提前连续按键，也不要套用仿真窗口 S/H。
仅双臂可用 `pixi run -e default bash -c 'source bash/environment.sh; exec python -m tianji real --devices arms --confirm-real'`，
仍需先对 `arms` 做只读预检和干跑。不再提供旧后端标定、断流宽限或恢复选项。

独立双臂回位使用 `bash bash/run_home.sh`，默认 Home 不变。
`bash bash/run_home.sh --L` 选择 `home-L.yaml`：左臂 `[90,-90,-90,-90,0,0,0]°`，
右臂 `[-90,-90,90,-90,0,0,0]°`（各侧 Joint1–Joint7），不控制手指。
先用 `bash bash/run_home.sh --L --dry-run` 无硬件校验；去掉 `--dry-run` 即进入真机回位流程，
必须先确认现场运动空间与急停。配置及限位检查不代表路径无碰撞或真机安全验收。
采集也可选同一姿态：`bash bash/run_teleop.sh --data --task pick-and-place -L`。
`-L`／`--L` 同时影响使能前的实测 Home 检查，以及每条采集结束后的回 Home；
必须先确认双臂已在 L Home，不能仅改变参数后从默认 Home 直接使能。
两个入口均接受 `-L` 和 `--L`，不传则保持默认 Home；选择只对本次运行生效，不改 `robot.json`。

输入失鲜、身份／反馈异常时不要绕过门控；先停止并排查。
运动使能期间不要按 PICO A 键、重新标定或重启输入。
正常退出确认设备释放后再停输入；紧急危险使用现场急停。
需要保存外层 console／退出日志时，在已激活 default overlay 的终端使用
`python -m tianji real --devices all --confirm-real`，日志在 `logs/real/`；
直接包装器的日志以实际输出为准。软件测试或仿真通过不代表当前设备的真机闭环已验收。

## 数据采集

观察录制和带真机执行的采集是不同入口，不互相授予运动权限：

- PICO 裸手仿真录制：`bash bash/run_pico_hand_sim.sh --height-m HEIGHT --record recordings/pico2_sim/NEW_SESSION.h5`，
  先创建父目录，每次使用新文件名；完整步骤见[裸手录制说明](src/teleop_inputs/pico_hand/README.md)。
- 独立观察采集器：`pixi run collect --task TASK`，只订阅 DDS，不连接机器人或打开相机；
  先用 `pixi run cameras` 启动官方相机节点。R/S/D 仍在真机执行器的交互终端操作，见[观察采集](README-reference.md#观测数据集采集r--s--d)。
- 真机任务采集：`bash bash/run_teleop.sh --data --task TASK`，复用 DLS／Ruckig 和安全门控，采用一次使能、多条采集循环；普通 `--real` 的三阶段 Enter 流程不变。
  先运行第三终端 `bash bash/run_camera_views.sh`；`--data` 要求已有且通过校验的相机监控，不再自行启动相机。
  它在设备连接前确认相机验证和 writer 准备完成（prepared），连接后再等待真实反馈 ready。
  只复用身份、配置和就绪状态匹配的独立会话，退出只停止本次创建的进程，不接管他人会话。
  默认原始数据为 `/data/TianjiData/raw/YYYYMMDD/`，不是 `dataset/`；
  `TIANJI_DATASET`／`--dataset` 可覆盖根目录，日期目录内独立保存配置。
  启动前双臂须已在 Home，执行器按实测反馈和配置容差确认；首次 `r` 授权使能并缓慢张手。`HOME_READY` 后再次 `r` 冻结目标并慢速对齐，采集确认开始后才跟随。
  `s` 截止并保存当前条，`d` 截止并丢弃当前条；受控制动、实测停稳和数据操作确认后回双臂 Home 并张手。条目间保持使能，不跟随 PICO／Manus。
  `q` 在 Home 结束整场；录制中按 `q` 则保存当前条、回 Home 后失能退出。其他过渡阶段拒绝录制键，不排队下一条。
  使能后 Enter、Ctrl+C 或关闭机器人窗口均立即安全停止，不自动 Home。故障路径不强行回程、不自动重新使能。
  USB 键盘式双踏板可映射为单次 `r`／`s`（不带 Enter、不连发），焦点必须保持在执行器终端；这不是全局脚踏设备监听。

`pixi run preview` 是相机话题订阅者，可与采集并行，不再自行打开 SDK pipeline。
离线压缩使用 `bash bash/compress.sh --date YYYYMMDD`，查看使用
`bash bash/view.sh --date YYYYMMDD`；默认压缩目录为 `/data/TianjiData/compressed/YYYYMMDD_compressed/`。
原始文件不删除、不覆盖，显式输出目录不自动追加后缀，详见参考页的[压缩说明](README-reference.md#离线批量压缩固定-jpeg-q50)。

## 共用 Home 与 Mocap／Regrind

真机受保护回位使用
[`src/teleop_outputs/tianji/tianji_description/config/home.yaml`](src/teleop_outputs/tianji/tianji_description/config/home.yaml)，
其 Home 向量与 DLS 配置一致；启动仍以只读实测姿态为初始参考，不会直接跳到 Home。
`controller_velocity_scale` 同时缩放执行器运行副本的 Ruckig 速度上限；
本次未改变设备限位、反馈保护或分阶段授权速度。模型名
`marvin_m6_wuji2_shared_root_ceres.xml` 是冻结资源名称，不代表选择 Ceres 算法。

远端新增的 H5 回放、Motive 接入和 Regrind 推理入口保留，详见
[Mocap／Regrind 操作说明](README-mocap.md)。`bash/run_mocap.sh` 将 infer/live/regrind-real/regrind-hand-sim
路由到同 Jazzy/Python ABI 的 policy 环境及 `install/policy` overlay，H5 回放使用 default。
policy 锁定 CPU PyTorch 2.10 与 Zenoh；GPU 运行库及模型权重仍须显式准备，CPU 验证不代表 GPU 或真机验收。

## 进阶说明与测试

以下是独立选择，不需要为上述双臂仿真全部启动：

| 需求 | 入口／说明 |
|---|---|
| PICO2 裸手仿真 | [独立入口](src/teleop_inputs/pico_hand/README.md)，不支持真机 |
| 双侧实测标定、完整人员档案 | [人员档案参考](README-reference.md#人员档案与标定版本) |
| Manus／外骨骼输入 | [输入链路参考](README-reference.md#2-启动-manus-灵巧手输入)，一次只选一种 |
| 真机遥操 | [真机预检与授权流程](README-reference.md#二真机遥操作)，不沿用仿真放行结论 |
录制控制已统一为两个 ROS Service，旧 `/tianji/collection/command` 已移除：

| 按键 | Service | 完成条件 |
|---|---|---|
| `r` | `/start_collect` (`tianji_interfaces/srv/StartCollect`) | writer 已进入 RECORDING；执行器再核对当前状态的会话／条目标识后放行跟随 |
| `s` | `/stop_collect` (`tianji_interfaces/srv/StopCollect`)，`save=true, abort=false` | 文件保存完成，且停止跟随已完成，再回双臂 Home |
| `d` | `/stop_collect`，`save=false, abort=false` | 本条文件丢弃完成，且停止跟随已完成，再回双臂 Home |

请求携带 `request_id`（规范 UUID）、`session_id` 和 `phase_revision`；
start 的可选 `task` 必须与采集器启动配置一致，空值沿用配置。start 的 `episode_id`
等于它的 `request_id`，stop 必须指定这个条目。响应 `success` 是完成结果，不是入队确认，
并回传请求／会话／条目标识、状态、`saved_path` 与错误说明。
同一进程内，同 UUID／同参数重试复用原结果或等待原操作；同 UUID 改参数拒绝，
旧条目的 stop 不能结束新条目。缓存不跨 collector 重启，执行器不自动重放未知结果。
`CollectionStatus` 增加 `episode_id`／`last_episode_id`，用于与服务结果交叉核对。
故障／退出内部使用 `abort=true, save=false` 保留 partial，不等同于操作者 `d` 的删除。
所有 Service 只操作录制；首次本地使能、运动门控、Home 和实际反馈采集仍由原模块负责。

| 数据集／相机／录制 | [采集参考](README-reference.md#观测数据集采集r--s--d) |
| 控制器与原生测试 | [原生控制器 README](src/teleop_outputs/tianji/tianji_controller/native/README.md) |
| 安装细节、目录与历史说明 | [完整参考页](README-reference.md) |

仿真、真机和原生 worker 统一为 DLS/Ruckig，没有旧算法回退。`--user` 的自动输入管理
使用配置中的 ROS 话题；未通过输入、资源和安全检查时拒绝启动。

软件回归命令（不等于现场验收）：

```bash
pixi run test-native
pixi run test-sim
pixi run test-pico       # PICO 手柄输入、标定和会话所有权
pixi run test-pico2      # 独立裸手路线
pixi run test-collection
pixi run test-ros        # 真实 DDS 回环，独立测试域 121；不是硬件验收
pixi run -e policy test-mocap
```

当前软件、DDS 和原生验证以[迁移验证状态](docs/migration-verification-status.md)为准；
不将历史分支的测试数字或动作记录当作当前真实输入、机器人或 GPU 验收。
