# PICO2 裸手仿真路线（原版 V131，性能验收待完成）

已接通 TCP 输入、C++ V131／Hand2、仿真状态机与 MuJoCo 直接关节显示。
**用户已完成真实 PICO 输入仿真录制；抖动和跟踪误差仍存在，不代表性能验收通过；不支持真机。**
现有 `teleop.sh`、VR、Manus、外骨骼和驱动不改动。
[实施方案](../docs/pico2-hands-design-and-plan.md)列出了剩余接线工作。

## 完整仿真启动

先停止旧 VR、Manus、外骨骼、裸手观察及其他仿真会话。头显打开 **裸手跟踪 APK**，
不是 VR whole-body／手柄 APK。新入口不需要 Zenoh router，不运行 `pico.sh`、
Manus 或外骨骼发送器，也不会自动配置 ADB。

```bash
# 工程根目录：首次或原生源码更新后构建
pixi install --locked --manifest-path pico2_hands/tools/wuji_hand_native/pixi.toml
bash pico2_hands/build_native.sh

# 用户现场连接头显（此次实现没有执行这些设备命令）
adb devices -l
adb forward tcp:10002 tcp:10002
adb forward --list

mkdir -p recordings/pico2_sim
PICO2_RUN_ID=$(date +%Y%m%d_%H%M%S_%N)
bash pico2_sim.sh --record "recordings/pico2_sim/pico_${PICO2_RUN_ID}.h5"
```

只测机械臂追加 `--disable-hands`。默认显示机械臂和舞肌二代手的期望关节角，
不做 PD、动力学积分或重力下垂。`--headless` 关闭窗口；`--duration-s 30`
在到时后请求回 Home 并退出，并非 30 秒立即强制结束。

当前仅使用原版 V131 IK。抗抖实验入口及滤波/参数扫描代码已移除，
不再接受 `--arm-control-mode` 参数；历史原始录制保留。
X/Z 标定、J3 限位、Viewer 按键显示修复及手部 retarget 不变。

仿真 Python 优先使用 `pico2_hands/.venv`，否则使用主工程 `.pixi/envs/default`；
可通过 `PICO2_PYTHON` 指定。缺少仿真 Python 依赖时，可单独安装，不升级旧路线：

```bash
python3.11 -m venv pico2_hands/.venv
pico2_hands/.venv/bin/python -m pip install -r pico2_hands/requirements-sim.txt
```

原生构建仍需主工程 Pixi CMake／MuJoCo／Pinocchio／qpOASES 环境。

### 按键和故障行为

窗口聚焦时或启动终端都可按键，状态显示在窗口及终端：

本入口每次显示同步后恢复 C/H 对应的接触点／凸包为关闭、S/R 对应的阴影／
反射为开启，避免 MuJoCo 默认快捷键导致底座外形或画面持续变化。
这是公开 passive Viewer 接口下的显示恢复，不是底层键盘事件拦截；
窗口现场效果仍需验收，极短的显示切换不作为模型或基座运动。

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
原 Z-only 数学保留在参考模块用于回归，但 `pico2_sim.sh` 的 C 已改用 X/Z。

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
bash pico2_sim.sh --self-test
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
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest pico2_hands/tests -q
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
pixi install --locked --manifest-path pico2_hands/tools/wuji_hand_native/pixi.toml
bash pico2_hands/build_native.sh
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
python pico2_hands/scripts/compare_v131_trace.py --reference-binary /path/to/v131_model_trace
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
bash pico2_hands/build_native.sh
python pico2_hands/scripts/smoke_pipeline.py
```

离线烟测构造可达腕部位姿和手骨架，无 C、使用固定映射，完成 60 帧双臂 IK 和双手
C++ retarget；左右臂接受均为 60/60。输入为合成数据，不是现场录制。
额外 600 帧测试对照通信路径与直接 C++ 调用，接受状态一致、关节差在 1e-8 rad 内。
现场状态机、键盘、录制和实际仿真执行仍未接线，不能用本烟测代替设备验收。

本阶段回归同上运行命令：**302 passed**；补充 epoch 上界和四元数范数溢出保护
后重新编译，IK 专项 **6 passed**，60 帧计算烟测再通过，`git diff --check` 通过。
旧入口、驱动和默认配置未修改；无设备操作、无提交／push。
