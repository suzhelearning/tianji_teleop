# TJ Arm PICO SPARK Headroom Feedforward Velocity QP

这是采集工作区内的天玑双臂 C++ 控制模块。推荐算法和 Viewer 默认值均为：

```text
spark_upper_qpoases_headroom_feedforward_velocity_qp
```

该路径以 SPARK 两阶段 qpOASES IK 保持人形手臂构型，以 Headroom 余量感知前馈提高跟踪响应，
并通过 Velocity QP、连续性代价和 settled-hold 抑制关节限位附近及静止阶段的可见抖动。
默认 Viewer 是 `model_reference` 参考运动显示，不是执行器驱动的动力学积分。
工作区根目录提供 `bash teleop.sh --sim`（双臂＋双 Hand2 动力学）、
`--real`（真机显式使能）和 `--data --task TASK`（真机采集）三个互斥入口；
参考运动显示使用 `.venv/bin/tianji view`，详见[工作区操作说明](../README.md)。
`--sim` 复用本控制器生成目标，在独立模拟状态上通过有界执行器和 `mj_step` 推进物理时间；
它不修改本控制器的 IK 算法，也不把模拟反馈伪装成本 Viewer 的 `actual`。

## 构建与测试

要求 Linux x86-64 和 [Pixi](https://pixi.sh/)。以下命令在工作区根目录执行：

```bash
pixi install
pixi run configure
pixi run build
pixi run test-native
```

完整测试包含配置、SPARK、Headroom、Velocity QP、PICO UDP、Wuji Hand 2 UDP、TJVR 录制以及 Viewer 集成测试。

构建依赖为 CMake 3.24+、C++20 编译器、Eigen 3.4、固定 MuJoCo 3.10.0（保留已验证的 Viewer API）、
Pinocchio 3.8、OSQP 1.x、qpOASES 3.2、yaml-cpp 和 GLFW；测试另需
GoogleTest 和 Python。Ruckig Community 0.19.4 的原始 C++ 源码与 MIT
许可证保存在 `third_party/ruckig/`，来源为
[上游 v0.19.4](https://github.com/pantor/ruckig/tree/v0.19.4)。
本地构建仅启用离线轨迹生成器，不下载仓库，不启用云端客户端。
也可由工作区根 CMake 使用 `add_subdirectory(control)` 集成；配置和模型
仍位于本目录，运行时从 `control/` 启动或显式传入其路径。

双臂控制循环通过同一个 `buildArmInput` 构造约束和姿态任务，保留左右臂
独立状态及原有验证、制动回退和原子提交顺序。Pinocchio 采样仅调用
同时更新关节位姿的 `computeJointJacobians(model, data, q)`，避免每次
IK 迭代重复执行完整的正运动学遍历；TCP／臂部坐标系和 Jacobian 约定不变。
TJVR／TJH2／TJRC 共用编译期生成的 IEEE CRC-32 表：每字节一次查表替代
八轮逐位计算，无堆分配和运行期初始化；多项式、初值、最终异或与报文布局保持不变。

## PICO 实时遥操

PICO 驱动、标定与骨架桥接源码位于同一工作区的 `tracking/`：

```bash
cd ../tracking
pixi shell
./scripts/start_tianji_pico_teleop.sh
```

该脚本启动 PICO 驱动、corrected M0 骨架、骨架 Viewer 和 TJVR v4 UDP bridge。

天机侧在工作区的 `control/` 目录无参数启动当前主算法：

```bash
OMP_WAIT_POLICY=ACTIVE OMP_PROC_BIND=close OMP_PLACES=cores \
  ./build/tianji_qp_ik_viewer
```

### 默认值

| 项目 | 默认值 |
|---|---|
| 算法 | `spark_upper_qpoases_headroom_feedforward_velocity_qp` |
| 配置 | `config/qp_ik_pico_teleop.yaml` |
| 模型 | `models/marvin_m6_wuji2.xml` |
| 控制层 | `velocity`，200 Hz |
| 控制状态源 | `model_reference` |
| PICO UDP | `127.0.0.1:15000` |
| 骨架 Overlay | 启用 |

没有 PICO 数据时控制器进入 stale/hold，不回退到脚本轨迹。若端口 15000 已被占用，请先关闭其他 Viewer 或显式指定另一端口。

## PICO + Manus + Wuji Hand 2

本工作区的 `../manus/` 是天机专用采集端，不再使用 mocap 的 Zenoh、Viser
或 ROS 2 Jazzy 环境。它使用 Manus Integrated SDK 采集原始节点，按 SDK
节点 ID 和手指语义映射为 21 点，再通过 ROS 2 `/hand_input` 驱动官方
Hand2 重定向器，以 `TJH2` UDP 控制 `models/marvin_m6_wuji2.xml` 中的灵巧手。
PICO 继续通过原有 TJVR v4/QP 流程控制双臂；两条输入链路互不改写对方关节。

### 环境与构建

需要根 `.venv` 和匹配 Python ABI 的 ROS 2 Humble SDK；安装方法见根 README。
手部重定向源码和模型资产位于 `../retargeting/`，由工作区统一安装，
不需要克隆或可编辑安装外部重定向仓库。

从工作区根目录构建采集器，并查看可用的手套标定用户名：

```bash
bash manus/build.sh
.venv/bin/python -m manus.start_hand_teleop --list-users
```

采集器为 `manus/build/manus_raw`，运行时 SDK 来自 `manus/ManusSDK/`；
标定文件来自 `manus/calibration/<用户名>LeftMetaglovePro.mcal` 和对应的 Right 文件。
构建使用 Linux x86-64 系统 `g++`、libudev、libusb 和 zlib。
如 SDK 无法访问 USB 接收器，可由管理员检查并安装 `90-manus.rules`；
启动器不会自动更改系统 USB 权限。

### 启动联合仿真

保持 PICO 链路运行。终端 1 在工作区根目录启动手部链路；
下面的 `syz` 是已有标定用户的示例，必须换成实际佩戴者对应的用户名：

```bash
bash manus.sh --user syz
```

该命令统一管理采集器、ROS 输入适配器和 Hand2 UDP 桥；
默认 ROS domain 为 120，输出到 `127.0.0.1:16000`。
按 `Ctrl+C` 会停止本命令启动的全部手部进程，不会停止 PICO 驱动。
可用 `--ros-domain-id`、`--host`、`--port` 和 `--topic` 显式配置。

终端 2 先关闭占用同一接收端口的旧机器人 Viewer，再从本控制工程目录启动：

```bash
./build/tianji_qp_ik_viewer \
  --model models/marvin_m6_wuji2.xml \
  --pico-teleop --hand-teleop \
  --pico-port 15000 --hand-port 16000
```

如只验证灵巧手，可加 `--no-pico-teleop`；没有图形会话时可加
`--headless --duration 10` 检查接收统计。整个入口仅输出仿真协议，不控制真实机械手。

### 数据契约与状态

- SDK 原始世界坐标为右手系，单位为米；适配器保留 XYZ，不作轴反射，避免颠倒掌心屈曲方向。
- `/hand_input` 为 `Float32MultiArray`：每条消息仅含一只手的 63 个 XYZ 浮点数，
  `layout.dim[0].label` 为 `left` 或 `right`，size/stride 均为 63。
  桥仍支持旧约定的无标签 63 数输入（默认右手）和无标签 126 数输入（右手在前）。
- SDK 帧按源 monotonic 时间检查过期和乱序；录制回放需重基准化源时间戳。
  ROS 消息本身不携带采集时间，因此每手源时间使用桥接回调接收时、求解前的 monotonic 时间。
  TJH2 v2（364 字节）分别携带双手采样发布时间和左右手源时间；旧 v1 包被拒绝。
- 使用 `retarget_manus_wuji_hand_2_left/right.yaml` 和官方 Hand2 模型，按关节名映射输出。
  输入先减去第 0 点手腕位置，再用第 0/5/9 点估计手腕 MANO 基准；
  桥从 Hand2 中立模型推导固定旋转，将该基准转换到优化模型坐标轴。
  不使用 Manus 手腕四元数，也不沿用手一代的 Euler 修正角或个体骨长缩放。
- 重定向仍逐输入帧求解；独立发布线程每 10 ms 采样各手最新结果，以 100 Hz 输出一个双手包。
  未出现过的侧保持无效；无新输入或求解失败时重复最后结果，但不刷新该手源时间。
  发布线程错过周期后不突发补发，也不倒填时间戳。
- 控制器每周期按当前 monotonic 时间减 10 ms 对目标作线性插值，真机配置下为 200 Hz。
  缺少相邻样本时保持可用端点，不外推；暂停/重置清空历史，恢复后要求新的源输入。
  每手源时间与接收时间独立检查 150 ms 新鲜度，持续重复旧目标不能维持真机 READY。
  Viewer 对无数据或超时的手保持最后合法状态；首次无数据保持模型初始状态。
- 更新后需要重新构建控制器并同时重启 Manus 桥与遥操进程，不能混用 TJH2 v1/v2。
- SDK 连接接收器成功不代表手套已输出骨架；还需手套开机、正确配对并持续发送数据。

## 真机 Hand2 序列号识别

工作区提供 `real_robot/read_hand_serials.py`，使用从 `dexhand_deploy`
固定复制的 Wuji C SDK；只需系统 Python 3，不依赖 ROS 或重定向环境。
以下命令从 `tianji_teleop` 工作区根目录执行：

```bash
# 只扫描 USB/网络，列出 Hand2 的 SN、型号和地址；不连接设备。
python3 real_robot/read_hand_serials.py

# 额外建立身份查询连接，读取固件报告的 left/right 后断开。
python3 real_robot/read_hand_serials.py --read-handedness

# 以 JSON 保存到新文件；不会覆盖已有配置。
python3 real_robot/read_hand_serials.py --read-handedness --output hand_devices.json
```

SDK 发现结果本身没有左右字段；默认显示 `unknown`，不能按扫描顺序或 IP
猜测左右。身份查询失败时仍保留 SN/地址并显示错误，不自动填充控制配置。
`--json` 可输出机器可读结果，`--timeout-ms` 设置单设备身份查询超时。
脚本不调用电机使能、清错、标定、运动或失能接口；身份查询连接时 SDK
可能进行时间同步，建议在没有真机控制会话运行时执行。
未发现设备时检查设备供电、USB/网络连接和主机网段。
该脚本不是遥操控制入口，不表示双臂或灵巧手真机运动接口已启用。

## 双臂／双侧 Hand2 真机执行边界

`../real_robot/` 接收本工程控制线程提交的最终关节参考，复用当前 SPARK /
Headroom / Velocity QP 与 Manus Hand2 重定向；不导入 `dexhand_deploy` 的
IK、policy、Home 轨迹或 Zenoh coordinator。只提取厂商 SDK 和设备 I/O。
支持 `arms`（双臂）、`left_hand`、`right_hand`、`hands`（双手）及 `all`（双臂＋双手）。
代码已做离线验证，尚未进行真机使能／运动测试；真机命令须在只读预检通过后由操作者执行。

### 默认干跑：不加载 SDK、不连接设备

以下命令均从 `tianji_teleop` 工作区根目录执行，使用已安装的手部 Python 环境：

```bash
# 手部执行链离线检查，不需要任何硬件开机。
python real_robot/run_teleop.py --devices hands --duration 5

# 双臂和双手的完整离线检查。
python real_robot/run_teleop.py --devices all --duration 5
```

执行器内部启动本工程无界面控制器，并通过只绑定 loopback 的 TJRC 数据包接收
左臂 7、右臂 7、左手 20、右手 20 个 rad 目标。无输入时 `flags=0` 是正常的未就绪状态，
不是已获准运动。`--duration 0`（默认）持续运行到 Ctrl+C。
不要同时启动占用相同 PICO/手部接收端口的旧机器人 Viewer。
单手或仅双手模式不会连接 Marvin，也不占用正常的 PICO 双臂输入端口。
当前协议为 TJRC v2 / 468 字节，旧 v1 / 308 字节被拒绝；必须同时更新控制器和执行器。

### 设备开机后：先只读检查

核对 `real_robot/config.json` 的左右 SN 和双臂 IP；左右身份来自已保存的
`hand_devices.json` 固件查询结果，不能按 IP 或发现顺序猜测，也不能让两侧共用一个 SN。

```bash
python real_robot/run_teleop.py --devices hands --inspect
# 需要检查双臂时：
python real_robot/run_teleop.py --devices arms --inspect
```

该模式只建立身份／反馈会话，不清错、不使能、不发目标。
左右手必须分别由设备报告为 Left/Right，关节状态和诊断须完整且序号、时间戳持续推进。
固件如果不在使能前提供反馈，程序会明确拒绝，而不是用零位或先使能来绕过。

### 后续真机操作入口

先让对应的 PICO／Manus 输入链正常运行；确认工作区安全、硬件急停可立即触达，
并停止其他控制该设备的程序。以下为双手入口；首次仍建议分别用 left_hand/right_hand 测试：

```bash
python real_robot/run_teleop.py --devices hands --confirm-real
```

双臂与双手联合入口为同一命令改用 `--devices all`。
`--confirm-real` 还要求交互终端、输入新鲜、反馈健康。
仅单手／双手模式保持 Enter 使能、再次 Enter 停止的流程，手指从实测姿态回零，
再用 0.5 s 插值到锁存的 Manus 帧；增益为 kp=3.0、kd=0.05。
包含双臂时自动打开独立的实测／目标双模型窗口：第一次 Enter 锁定目标并慢速对齐，
实际到位后只显示 READY；第二次 Enter 才开始实时跟随；第三次 Enter 让双臂慢速回
`real_robot/config.json` 中指定的 HOME，再失能。此模式的双手直接慢速对齐，不经过回零。
回 HOME 时双手保持最后指令，不主动张开；失能后仍可能失去夹持力，必须先确保负载安全。
Ctrl+C、关闭窗口、输入／反馈故障，或对齐／回位中按 Enter，均直接停止，不追加 HOME。
窗口绘制不运行在控制循环中，过期实测数据标记 STALE，不用模拟姿态代替反馈。
左右手采用厂商设备的 MIT 参数接口，最终目标仍由本工程求解。
两只手共享 SDK 生命周期，但设备句柄、身份、回调、反馈时效和故障状态独立。

双臂模型参考初态由实测关节写入临时配置，原 YAML 不修改；仍使用 `model_reference`
算法，真实反馈用于初态同步、设备健康和跟踪误差检查，而不是悄悄换成另一套 IK。
真机配置将控制器速度比例约束为 0.1，并在 SDK 边界限制 setpoint 变化速度：
双臂实时跟随默认 0.5 rad/s，每只手实时跟随为 1.0 rad/s；
包含双臂时对齐／回位统一额外限制为 0.1 rad/s。仅手部模式的半秒启动插值仍是原有例外。
Marvin 只在 SDK 边界 rad→degree；Hand2 使用 finger-major 的 20 个 rad 关节值。

输入／反馈超时、严重设备故障／失能、跟踪误差、限位违规、控制器退出、PICO reset/resync
都会终止已使能会话；锁存故障不能被后续正常帧覆盖。没有自动故障恢复或清错。
Hand2 诊断由 SDK 目录分级：`Warning` 记录并在反馈 detail 中显示，
包含左右手、SN、NID、故障名和处理建议；停止类级别、未知码、解码失败仍停机。
警告不跳过任何状态、时效、限位或跟踪检查；编码器警告持续出现仍需检查硬件。
Ctrl+C 会尝试停止并释放本命令拥有的设备会话，随后停止内部控制器。
如果 SDK 无法确认清理，会明确要求检查硬件急停，不把异常当作成功。
PICO 目标新鲜度使用 YAML 的 `cartesian_servo.target_timeout_seconds`（当前遥操配置为 0.5 s）；
启用 `--pico-teleop` 不再将其强制覆盖成 50 ms。配置变化需重启控制器才能生效。
PICO bridge 发送时间到真机指令输出的年龄上限为 300 ms；它独立于
`target_timeout_seconds`，不会随该 YAML 配置自动改变。未来时间戳仍被拒绝。
真机指令包、设备反馈及每手源输入仍保留各自的 150 ms 新鲜度检查。
软件 watchdog 是判断阈值，停机延迟还受 SDK 阻塞和操作系统影响；
它不替代物理急停及设备自身保护。Hand2 使能瞬间的目标保持行为、反馈时钟和
固件独立断流保护必须在后续受控带电测试中核实，不能由离线通过推断。

## 主算法

```text
PICO TJVR v4
→ SPARK 两阶段 qpOASES IK
→ 关节速度与笛卡尔前馈
→ Headroom 余量衰减
→ Velocity QP
→ 输出连续性与 settled-hold
→ MuJoCo model_reference
```

SPARK 使用 corrected 肩、肘、腕和掌心数据，按机器人骨长求解连续的双臂关节构型。前馈路径从 SPARK 关节参考和掌心运动提取速度信息，减少纯反馈路径的相位滞后。

Headroom 根据关节位置、速度、加速度和 jerk 可用余量调节前馈强度：余量充足时保留
跟踪响应；接近硬约束时快速衰减，避免前馈持续把关节推向限位。QP 最终仍强制执行
关节动态边界、上臂外侧安全边界和笛卡尔任务约束。

输出连续性项抑制 `qdot` 和 jerk 方向突变。手部已静止或约束余量过低时，
stationary-reference hold 与 settled-hold 停止追逐不可实现的骨架抖动。恢复运动后，
前馈按配置的恢复时间平滑重新进入。

当前关键配置位于 `config/qp_ik_pico_teleop.yaml` 的：

```text
spark_feedforward_velocity_qp
spark_headroom_feedforward_velocity_qp
spark_upper_qpoases
hierarchical_qp
joint_limits
upper_arm_outward
```

## 完整显式启动

需要固定所有参数并保存遥测时：

```bash
mkdir -p benchmark_results/pico_live/traces
OMP_WAIT_POLICY=ACTIVE OMP_PROC_BIND=close OMP_PLACES=cores \
  ./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_wuji2.xml \
  --pico-teleop \
  --pico-skeleton-overlay \
  --pico-bind 127.0.0.1 \
  --pico-port 15000 \
  --control-level velocity \
  --algorithm spark_upper_qpoases_headroom_feedforward_velocity_qp \
  --model-state-only \
  --pico-record benchmark_results/pico_live/traces/output_continuity_retest.tjvr \
  --telemetry benchmark_results/pico_live/headroom_live.csv \
  --joint-telemetry benchmark_results/pico_live/headroom_live_joints.csv
```

遥测包含末端误差、QP slack、任务缩放、关节状态、有效动态边界、Headroom、前馈状态、
控制周期分位数和 PICO 输入统计。CSV 写线程与 200 Hz 控制线程隔离。TJVR 录制器不会
覆盖已有文件；重复测试时请保存旧轨迹或为 `--pico-record` 指定新文件名。

## MuJoCo TJVR 可视化回放

TJVR 实测轨迹是本地测试资产，不随仓库发布。可以通过上一节的 `--pico-record` 自行
录制，也可以由测试人员放到约定位置。两个终端均从工作区的 `control/` 目录启动。

本节回放身份固定为：

- 回放算法：`spark_upper_qpoases_headroom_feedforward_velocity_qp`
- 回放控制层：`velocity`
- 回放模型：`models/marvin_m6_qp_pico_fast.xml`
- 回放状态源：`model_reference`

终端 1：

```bash
mkdir -p benchmark_results/pico_live
OMP_WAIT_POLICY=ACTIVE OMP_PROC_BIND=close OMP_PLACES=cores \
  ./build/tianji_qp_ik_viewer \
  --duration 120 \
  --telemetry benchmark_results/pico_live/pico_headroom_main_replay.csv \
  --joint-telemetry benchmark_results/pico_live/pico_headroom_main_replay_joints.csv
```

看到 `pico_udp_bind=127.0.0.1:15000` 后，在终端 2 运行：

```bash
TRACE_FILE=benchmark_results/pico_live/traces/output_continuity_retest.tjvr
test -f "$TRACE_FILE" || { echo "missing TJVR trace: $TRACE_FILE" >&2; exit 1; }
python3 scripts/replay_pico_udp_trace.py \
  --input "$TRACE_FILE" \
  --host 127.0.0.1 \
  --port 15000 \
  --lead 0.5
```

当前本地基准轨迹包含 9442 帧，时长 106.887 秒；使用其他录制文件时以回放工具的实际
统计为准。该基准成功时回放工具输出 `replayed_frames=9442`。两份回放 CSV 位于
`benchmark_results/pico_live/`，便于与实测结果统一分析。

回放工具使用本地接收时间间隔发送原始数据包，不修改序号、源时间、
bridge 时间或 CRC。它只用于模拟与分析，不将历史 bridge 时间伪装为
新鲜的实时输入；真机命令的新鲜度保护保持生效。

## 常用操作

| 输入 | 功能 |
|---|---|
| `P` | 启用或关闭 PICO 输入 |
| `G` | 切换 PICO 臂角参考模式 |
| `F2` | 显示或隐藏七轴关节曲线 |
| `F3` | 切换 `q`、`dq`、`ddq`、`jerk` |
| `F4` | 切换并锁定左/右臂曲线 |
| `F5` | 曲线恢复跟随当前选择臂 |
| `Space` | 暂停或恢复 |
| `F1` / `Esc` | 显示帮助 / 退出 |

## 验证默认路径

```bash
./build/tianji_qp_ik_viewer --headless --duration 1
```

启动摘要应包含：

```text
viewer_config=config/qp_ik_pico_teleop.yaml
control_state_source=model_reference
pico_udp_bind=127.0.0.1:15000
algorithm=spark_upper_qpoases_headroom_feedforward_velocity_qp
control_level=velocity
```

## 安全与范围

- 当前 Viewer 只驱动 MuJoCo，不包含真实机械臂 SDK、力矩控制或硬件急停。
- 当前速度、加速度、jerk 和制动参数是运动学验证值，不是实机安全认证参数。
- 工程不包含完整碰撞/自碰撞、双臂 14 自由度耦合优化或执行器动力学。
- 单臂参考无效或 QP 不安全时只冻结故障臂；健康臂继续运行。
- 非 PICO 运行需显式使用 `--no-pico-teleop --no-pico-skeleton-overlay` 并指定对应
  `--config`、`--model`、`--control-level` 和 `--algorithm`。

## 共享根映射的身高／体型适配

共享根映射按人体肩宽和总臂展估计尺度，再生成机械臂目标；映射与后续 IK 是不同层。
2026-09-20 基于 1.62 m 的 4471 帧录制完成 11 组合成人体实验：等比例身高
1.45～1.95 m 的掌心目标与基线差值低于 `1e-12 m`；独立改变肩宽或骨段比例时
目标不再相同。所有组最终几何闭合，不代表关节限位、碰撞、IK 或真人验收通过。

完整结果及 Pixi 复现命令见[合成人体映射实验](docs/verification/synthetic_body_mapping_20260920.md)，
换人步骤见[简化标定](../docs/pico-simple-calibration.md#换人身高与臂长适配)。
实验仅改变录制中 M0 输出后的骨架点，不覆盖上游按身高模板重建／截断和传感误差；
生成的 `.synthetic.tjvr` 仅用于离线映射审计，不得送入在线执行器。
此结论不外推到其他映射模式，也不修改现有控制器默认配置。

## 历史算法

当前共享根 SPARK `qp_ik_pico_shared_root_reachable.yaml` 已与 Ceres 统一源模型限位、
初始姿态和硬运动限值；IK 与运动参考均在 Viewer 控制循环在线计算，见
[限位与在线接线说明](docs/verification/shared_root_ceres_f615b8c.md)。其他历史配置不随之批量修改。

另有默认关闭的[共享根 Ceres LM＋Ruckig 实验后端](docs/verification/shared_root_ceres_f615b8c.md)，
标准 `pixi run build` 已启用 `TIANJI_ENABLE_CERES=ON`，产物统一在 `control/build`。
直接使用 CMake 的旧构建仍可关闭该选项。当前仅允许模型状态仿真、禁止关节导出；
已同步源 `f615b8c` 的 Pinocchio 路径及 Ruckig 修复，见[最新移植验证](docs/verification/shared_root_ceres_f615b8c.md)；
左臂残差明显改善但仍未通过跟踪验收。原生 Viewer 与真机默认保持不变；
上层 `teleop.sh --sim` 已按用户选择默认 Franka DLS＋Ruckig。

交互仿真可选 `bash teleop.sh --sim --ik-backend ceres`（在仓库根目录运行）：
S 接入、H 平滑回 Home 后等待、P/Space 停止，详见
[DLS/Ceres 交互仿真与验收清单](docs/verification/ceres_interactive_sim.md)。仅双臂 direct 仿真，无硬件指令输出。
阶段报告与历史消融集中在[历史归档](docs/archive/2026-09-shared-root/README.md)，不作为当前运行指令。

Hierarchical QP、null-space DLS、Cartesian OTG velocity/acceleration、旧 SPARK A/B
模式、数学公式、基准指令和完整旧键位说明已迁移至
[历史算法与回归入口](docs/legacy_algorithms.md)。这些路径继续保留用于回归，但不是
当前 `main` 的推荐默认算法。
