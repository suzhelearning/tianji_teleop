# PICO2 裸手仿真路线（V131／可选共享根 DLS，性能验收待完成）

已接通 TCP 输入、C++ V131／Hand2、仿真状态机与 MuJoCo 直接关节显示。
**用户已完成真实 PICO 输入仿真录制；抖动和跟踪误差仍存在，不代表性能验收通过；不支持真机。**
现有 `bash/run_teleop.sh`、VR、Manus、外骨骼和驱动不改动。
当前工作空间结构和软件验证范围见[迁移验收记录](../../../docs/migration-verification-status.md)。

## 完整仿真启动

先停止旧 VR、Manus、外骨骼、裸手观察及其他仿真会话。头显打开 **裸手跟踪 APK**，
不是 VR whole-body／手柄 APK。新入口不需要 Zenoh router，不运行 `bash/run_pico.sh`、
Manus 或外骨骼发送器，也不会自动配置 ADB。

```bash
# 工程根目录：首次或原生源码更新后构建
pixi install --locked --manifest-path src/teleop_inputs/pico_hand/tools/wuji_hand_native/pixi.toml
bash src/teleop_inputs/pico_hand/build_native.sh

# 用户现场连接头显（此次实现没有执行这些设备命令）
adb devices -l
adb forward tcp:10002 tcp:10002
adb forward --list

mkdir -p recordings/pico2_sim
PICO2_RUN_ID=$(date +%Y%m%d_%H%M%S_%N)
bash bash/run_pico2_sim.sh --record "recordings/pico2_sim/pico_${PICO2_RUN_ID}.h5"
```

只测机械臂追加 `--disable-hands`。默认显示机械臂和舞肌二代手的期望关节角，
不做 PD、动力学积分或重力下垂。`--headless` 关闭窗口；`--duration-s 30`
在到时后请求回 Home 并退出，并非 30 秒立即强制结束。

默认旧路线使用原版 V131 IK。抗抖实验入口及滤波/参数扫描代码已移除，
不再接受 `--arm-control-mode` 参数；历史原始录制保留。
X/Z 标定、J3 限位、Viewer 按键显示修复及手部 retarget 不变。

仿真 Python 由工作区 Pixi `default` 环境提供（`bash/run_pico2_sim.sh` 使用
`bash/pixi.bash` 选定的解释器）；该环境的 numpy／scipy／PyYAML／h5py／mujoco
版本见本包 `requirements-sim.txt`，不再为本路线创建独立 virtualenv。
V131 在工作区 Pixi `control` 环境构建；Hand2 优化器另用本包
`tools/wuji_hand_native` 的固定 Pinocchio 4 环境，不向默认 Python 注入其动态库或模块。

## 身高＋C＋共享根＋DLS/Ruckig 新模式

已将 upstream `90c575fe1dc6b02929a4f701a2f1263b8b347fd0` 适配到 protype-style 目录。
先按上文准备裸手 APK 和手部 worker，再构建工作区原生 DLS worker：

```bash
pixi run --locked build
bash bash/run_pico2_sim.sh --mapping-mode shared-root --height-m 1.62
```

身高单位米，范围 `[1.0, 2.4]`，必须替换为实际身高。只测双臂可加 `--disable-hands`；
`--record FILE.h5` 仍独占创建原 schema 录制，不覆盖旧文件。此模式不使用 VR 人员档案、
手柄 TCP 标定或 Manus，没有硬件输出权限；默认不加参数仍是原 V131 路线。

1. 面向前方，双臂水平前伸、双手间距约肩宽、掌心相对。
2. 空闲时按 **C** 并稳定约 1 秒；至少 30 个独立有效采样、覆盖至少 0.75 秒。
   成功后等新帧再按 **S**；未标定、标定中、失败、回程中或输入过期不能接管。
3. **H** 先制动再平滑回双臂 Home，手指保持；不走旧 V131 的 Python 五次回程。
4. **P／空格** 制动保持，可取消 H 的回程及回程前制动。
   **Q／Ctrl+C** 回双臂 Home 后退出；退出开始后不接受暂停，避免阻塞退出。

跟踪失效或帧龄超过 45 ms 时先制动，不继续推进旧目标。只有这种输入失效触发的保持
可有界自动续接：从最后有效输入起 **1 秒内**，同连接恢复至少 **100 ms、5 个独立有效帧**，
相邻帧间隔不超过 45 ms，且 C 和映射仍有效。恢复资格成立后还必须等原生 HOLD 静止及新帧，
再走与 S 相同的软启动；等待中再次失鲜则取消资格。重复／倒序帧不更新恢复计数。
超时、TCP 断开／重连、映射或 IK 拒绝，以及 P/空格/H/Q/C/R/S 均取消自动恢复。
主动暂停或 Home 不自动接管；长时断流需人工 S，连接身份变化还会清除 C。
重新 C 后，显式 S 只在静止时建立新求解 epoch，允许设备时钟重新计时；
同连接倒序输入仍拒绝，不在运动或制动中重置速度／加速度。

### 人体模板与掌心映射

肩宽 `0.1828 H`、上臂 `0.155882 H`、前臂 `0.152941 H`、腕掌 `0.037037 H`，
均为身高估计的对称模板，不是个人实测。输入从 **wrist** 出发，沿 wrist→middle proximal
（点 12）添加一次腕掌偏移；不用 palm 点二次补偿，关键点失效或向量退化时拒绝。
C 用头显水平朝向固定根轴，由前伸双掌及估计臂展建立虚拟肩中点，
同时保存腕姿态到机器人参考掌心的固定旋转。机器人参考由 FK 查询
（J2=-90°、其余为零），不实际执行该姿态。

虚拟根随头显平移、不随转头旋转；无法区分头部平移和躯干移动，转身或明显弯腰后需 H 再 C。
掌心目标复用共享根仿射公式 `p_B = o_B + R_BCt diag(s_reach, s_width, s_reach) p_Ct`，
读取同一 `shared_root_robot_geometry_ceres.yaml` 并校验模型指纹；位置缩放不施加到姿态上。
没有实测肩肘，不包含完整 VR M0 骨架重建、闭合投影、shape guidance 或人体骨架叠加。

### 原生控制与资源

`pico2_dls_worker` 直接链接主工程 `DualArmController`，复用 DLS、Pinocchio、掌心 TCP、
在线 Ruckig 和 `SimulationRecovery`；仅通过本地管道通信，不发布遥操 UDP。
worker 安装至 `install/control/bin/`，Python 经 `tianji_runtime` 查找可执行文件、
控制器 profile 和 `tianji_description` 模型，并在临时 runtime YAML 中解析绝对资源路径。

名义周期 200 Hz。PICO2 的 S 接近上限为 **1.4 rad/s、3 rad/s²、12 rad/s³**，
与正常限值取较小值，接近后按原 0.5 秒过程恢复正常限值；
VR 默认软启动 **0.35／0.5／2**、正常跟踪限值及 Home 参数不变。
显示启动时校验 worker 的 palm TCP，不混用旧 V131 Base 系 flange TCP。
手指仍走原 PICO2 retarget，不受双臂 Ruckig／Home 控制。

原 `--self-test` 仅用于 V131，不用于 shared-root。离线回归：

```bash
pixi run --locked bash -c 'source bash/environment.sh; PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/teleop_inputs/pico_hand/python/pico2_hands/tests -q'
```

合成输入验收不代表真实跟踪精度、握姿轴向或实时调度合格；实际人员和设备仍需现场验证。

## 默认 V131 按键和故障行为

窗口聚焦时或启动终端都可按键，状态显示在窗口及终端：

两种模式都在创建窗口前取消 MuJoCo 默认 **H→凸包显示**绑定，H 只触发 Home。
修改仅限当前进程的 UI 快捷键表，退出恢复；不改安装文件、模型和其他进程。
该实现检查固定 MuJoCo 3.10.0 ABI，不对其他版本静默套用。
C 的接触点、S/R 的阴影／反射仍在同步后恢复。以下按键表适用于默认 V131：

| 按键 | 行为 |
| --- | --- |
| S | 新鲜输入、空闲及标定门控通过后接管；重复 S 不停止 |
| C | 可选 X/Z 标定，双臂向前水平伸直、双腕稳定约 2 秒；不按 C 使用固定映射 |
| H | 停止跟随，平滑回配置 Home；到位后自动准备，再有新帧才能按 S |
| R | 保留手动重新准备，只允许空闲且已在 Home；之后等待新帧再按 S |
| Q / Ctrl-C | 回 Home 并退出，等待录制排空和 worker 清理 |

回程拒绝 S/C/R，不会自动重新接管。双手也会平滑回初始值，所以完整回程可能比
仅机械臂长。关节 Home 为左 `[55,-65,-70,-60,60,0,0]°`、右镜像。
普通张手、握拳、捏合只显示观察标签，不自动使能或停止。

C 现采用与 VR mapped-palm 相同的前伸参考姿态：J2＝−90°，其余关节＝0°。
仅通过当前 V131 worker 的 FK 查询该姿态的 IK TCP，不直接执行此姿态。
左右分别对映射后的 TCP 采样，计算固定 X/Z 偏移；Y、姿态及后续移动比例不变。
保留裸手相对当前头显的输入，不引入 VR 骨长配置或左右取最短处理。
偏移超过 1 m 拒绝；重复标定不累加，失败保留上次成功结果，首次失败需重试。
原 Z-only 数学保留在参考模块用于回归，但 `bash/run_pico2_sim.sh` 的 C 已改用 X/Z。

同连接内的短时头／腕识别失效会保持双臂双手上一姿态；恢复后沿原映射继续。
为避开 V131 的内部 50 ms 超时，超过 45 ms 的腕部输入先保持，不持续推进 IK。
TCP 重连／来源身份变化会退出跟随、回 Home、清除来源相关高度标定，需要重新按 S。
连续 IK 拒绝、原生进程异常或录制错误会退出并保留不完整录制，不能宣称成功。

### 录制与已验证范围

HDF5 使用独立 `pico2_sim_session_v1` schema，**不是 dexhand_deploy 原 session schema**：
保留每个接收到的 1982 字节原包、接收／源时间、连接代次与序号、54 关节 float64
目标、输入关联、按键及标定事件。文件独占创建，正常回 Home、worker 退出及录制
排空后才置 `complete=true`；错误保持 false。不修改旧录制读取器。

数值 IK、手部几何／优化器／滤波在 C++ worker 内；该新入口的管理调度、窗口和
独立 HDF5 写入仍为 Python，不宣称全 C++ 或实时合格。原始骨架叠加显示、旧 session
录制 schema 兼容和真机门控仍待后续移植；HUD 已显示状态与观察手势。
此入口隐藏共享模型中未绑定的静态目标标记，避免将其误认为当前 IK 目标。
左右 J3 在本入口的仿真实例内统一为 `[-3.1, 3.1] rad`，与 V131 一致；
同步更新关节范围、执行校验和 actuator 范围，启动时检查全部机械臂 IK 限位
均被显示执行范围覆盖。其他关节及旧路线／共享 XML 不变，不用于真机限位授权。

```bash
# 无设备自测；合成输入，绝不连接 TCP 或发送硬件命令
bash bash/run_pico2_sim.sh --self-test
```

早期接线验证：**312 项回归通过**；仅机械臂和完整双臂双手均完成 S→H→Home→S→Q；
假 TCP 分包接收＋HDF5 排空测试通过，`--real` 被拒绝。完整带录制自测 4,151 周期，
8,313 条全部处理、`complete=true`；数据保留于 `/tmp/pico2-sim-check-IvwQ9I/session.h5`。
显示模型的法兰 TCP 已与 V131 模型做启动校验；掌心控制点的 36.5 mm 偏移没有混用。
该早期阶段没有启动设备。后续用户已在 2026-09-16 完成两次真实输入仿真录制。
2026-09-17 提交前审查修正了按键操作后的新鲜度时钟及重新接管时的拒绝计数，
回归 **316 项通过**，原生 V131 CTest 3/3、手部调度器 1/1、优化器夹具通过，
16 帧手部 Python/C++ 对照最大差 0 rad。录制目录被忽略，不随代码提交。
完整双臂双手无窗口自测完成 4,128 周期，S→H→S→Q 正常退出；不代表实时合格。
暂存差异检查仅报告第三方原样迁入的 `mediapipe.py` 与左右 Hand2 URDF 中
11 处既有尾随空格；为保留来源 SHA256 未改写。排除第三方后的差异检查通过。
这不改变 IK 数值核心；此次未重新进行真实设备验收。

以下为**历史阶段记录**：其中“尚未接线”等描述仅代表当时状态，
当前入口行为以上方启动说明和 X/Z 标定说明为准。

在项目根目录、使用已安装 numpy/scipy/PyYAML 的 Python 环境执行。
头显需运行裸手跟踪 APK，而非 VR whole-body APK。现场需先自行停止旧输入会话。
以下为用户现场操作指令；本轮没有执行 ADB 或连接真实设备：

```bash
adb devices -l
adb forward tcp:10002 tcp:10002
python -m pico2_hands.observe --duration-s 10
```

默认每秒打印有效性、接收新鲜度和观察标签；不创建任何执行器，不配置 ADB，
没有自动切换 APK 或设备的逻辑。重复／倒序源时间不更新新鲜度，重连重新建立身份。
退出码 0 只表示结束时双臂／双手观察有效且线程退出；不证明持续流质量、标定、
手势准确率、retarget 或运动安全。Ctrl-C 返回 130，其余未通过返回 1。

`reference/` 保留来源 Python 数学与接收参考，`config/hand_tracking_target.yaml`
是来源参数快照，目前仅供映射回归，**不是目标项目可启动的 session 配置**。
runtime 摘取接收／转换类，不引入来源的 Zenoh、ROS、执行器或绝对路径依赖。
pose_mapping 移除了无关的 XR incremental 工厂分支，其余数学保持来源。

## 离线验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/teleop_inputs/pico_hand/tests -q
```

禁用自动插件是为避免宿主 ROS pytest 插件污染；本包测试不依赖 ROS。
覆盖解析、26→21、手腕相对头显、手势、mapper、分包／粘包、错误消息类型、
重复／倒序、重连与失效。官方 worker 内部坐标准备测试
已在第二阶段随官方 worker 迁入补齐；当时尚未接线的仿真手部输出现已接通，
真实输入效果仍待现场验收。

第一阶段结果（2026-09-16）：使用目标工程 `.pixi/envs/default/bin/python`，运行
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest pico2_hands/tests tests sim/tests real_robot/tests -q`
得到 **284 passed**；`python -m pico2_hands.observe --help` 和 `git diff --check` 通过。
首次未禁用插件时，宿主 ROS `launch_testing` 因缺少 `lark` 导致收集前失败；
没有为此修改系统 ROS 环境。当时尚未运行原生 IK 对照；第二阶段结果如下。
两个阶段均未进行真实输入仿真或真机验收。

## 第二阶段：原生算法与 worker（2026-09-16）

新增独立 V131 库和完整模型状态适配器、官方 Hand2 C++ worker／调度器、
几何与滤波、优化器、模型及固定版本的 Python 对照环境。旧 control 构建和
Manus bridge 未修改。`native/src/PORTING.md` 是来源历史说明，不是目标验收报告。

```bash
pixi install --locked --manifest-path src/teleop_inputs/pico_hand/tools/wuji_hand_native/pixi.toml
bash src/teleop_inputs/pico_hand/build_native.sh
```

仅构建、运行离线测试，不连接任何设备。依赖需先安装主工程 Pixi 环境。
构建使用单任务以限制内存占用。固定 Hand2 环境只安装在本模块目录，
不升级既有环境。原锁文件格式／CMake Boost 会产生兼容性警告，目前不影响构建。

本轮结果：V131 CTest **3/3**、Hand2 调度器 **1/1**，优化器右手夹具通过。
第二阶段相同 Python 回归命令得到 **289 passed**，包含新 worker 生命周期、
超时锁存、错误侧别拒绝与迁入文件指纹检查；构建脚本实际跑通，Shell 语法检查通过。
左右 Hand2 worker 与 Python 固定参考共 16 帧，最大关节差 **0 rad**，包含序号
间断后的重置。最初误给右手专用优化器夹具传左手模型触发断言；更正模型后通过。

V131 对来源现有 `v131_model_trace` 二进制进行移动、组合旋转两组各 600 帧对照：
接受状态一致，最大关节差 **0 rad**。这是二进制有限合成轨迹对照，不是全部源码、
真实输入或真机等价证明。来源二进制 SHA256：
`cd8bab2e555fc296fea671d642b22cde31cc6db2aa9551782f6c565ae67349b9`。
复跑时显式提供参考，仅用于离线审计，不构成运行依赖：

```bash
python src/teleop_inputs/pico_hand/scripts/compare_v131_trace.py --reference-binary /path/to/v131_model_trace
```

`NativeHandWorker` 管理单侧 C++ 子进程，验证握手、序号、时间戳、侧别、有限值，
限制通信等待；通信失败后锁存，必须关闭并由会话显式重建状态。它不具有发布权限。
V131 的 MuJoCo 模型改为构造时显式提供，避免全局环境变量干扰其他路线，数值路径保留。
迁入文件指纹见 `native_source_manifest.json`。

仍待：裸手 C 键现场绑定、输入所有权及状态机接线、仿真展示、统一录制、真机 mock 门控和
独立设备验收。不要把上述构建通过理解为 `teleop.sh` 已支持裸手运动。

## 可选 C 标定适配

`mapping.OptionalHeightMapping` 已接入来源高度采样器与模型水平参考：

- 未按 C：沿用固定映射，标定门控允许继续；不代表输入／使能等其他门控通过。
- 按 C：仅允许调用方确认空闲后采样；采样期间拒绝接管。
- 首次失败：没有有效旧结果，需重试；再次失败：保留旧结果。
- 成功只替换竖直分量，保留 X/Y、姿态、前移偏移和控制点补偿。

此适配器没有设备或发布权限，尚未绑定现场键盘及执行状态机。
它不是新的强制标定条件，也不是 mapped-palm 的 X/Z 对齐功能。

本阶段回归：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest pico2_hands/tests tests sim/tests real_robot/tests -q`
得到 **296 passed**，包含未按 C、首次失败、再次失败、只改竖直分量、重复采样、
错误模型与错误坐标系分支。初次测试把 URDF 舍入后的竖直轴当成精确配置轴，
导致约 0.37 微米投影残差；改为按来源实际采用的配置轴验证后通过，未修改映射数值。
`git diff --check` 通过。没有启动设备、仿真现场或真机；未提交／push。

## 原生 IK 通信与离线计算链路

`NativeIkWorker` 启动独立 `pico2_v131_worker`，固定二进制请求／响应，
不连接设备、不发布命令。接口提供 FK、数值 reset 和带时间戳的双臂 solve。
目标为各侧 Base 到 TCP，位置米、四元数 xyzw，关节弧度、左七轴后右七轴。
数值 reset 只清理算法状态，不代表机械臂 Home、静止或使能授权。

worker 同时校验双臂输入后才求解；首次 solve 必须先 reset，reset 递增 epoch。
拒绝序号／时钟倒退、错误 epoch、非法数值和零四元数。通信超时／错误响应锁存，
需由上层关闭并显式重建。非正常子进程退出或强制清理不会静默算作正常结束。
当前固定 200 Hz、0.02 rad 单步快速配置仍为 **simulation-only**，没有接入真机入口。

```bash
bash src/teleop_inputs/pico_hand/build_native.sh
python src/teleop_inputs/pico_hand/scripts/smoke_pipeline.py
```

离线烟测构造可达腕部位姿和手骨架，无 C、使用固定映射，完成 60 帧双臂 IK 和双手
C++ retarget；左右臂接受均为 60/60。输入为合成数据，不是现场录制。
额外 600 帧测试对照通信路径与直接 C++ 调用，接受状态一致、关节差在 1e-8 rad 内。
现场状态机、键盘、录制和实际仿真执行仍未接线，不能用本烟测代替设备验收。

本阶段回归同上运行命令：**302 passed**；补充 epoch 上界和四元数范数溢出保护
后重新编译，IK 专项 **6 passed**，60 帧计算烟测再通过，`git diff --check` 通过。
旧入口、驱动和默认配置未修改；无设备操作、无提交／push。
