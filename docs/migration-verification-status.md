# 迁移验收状态与未验证范围

基线为本地 `main@90c575fe1dc6b02929a4f701a2f1263b8b347fd0`。当前实现已按 [protype-style 结构方案](protype-style-data-collection-structure.md) 切换到 `bash/`、`config/`、`src/` 和分环境安装目录；不是旧根目录入口的兼容层。以下为 2026-09-21 的实际软件验收，硬件边界单列。数值证据见 [JSON 记录](migration-verification-results.json)。

> 范围更新：现已按三条主输入路线移除旧 PICO 专用录制、外接 IMU 与 Odin 辅助包；正式 schema-v1 collector 和 PICO2 仿真录制保留。下方第 1～5 节是删除前的迁移验收快照，历史构建数量和测试数量不反写为本次结果；删除后的验证另记于文末。

## PICO 裸手 → SPD 独立采集单话题验收（2026-09-23）

按用户确认的“SPD 独立采集”职责收敛：两项目之间只保留
`/spd/tianji_wuji2/v1/joint_command`。删除上游远程控制服务／状态、下游控制客户端和
对应开关，不保留控制 socket 或 RPC 租约。`JointCommand.msg` 内容及指纹未改变，
两端生成绑定清理后分别重建，避免退役接口残留。此节仅覆盖该独立链，不替代其他链路验收。

- 上游实际 `pixi run --locked -e spd test-pico2`：**113 passed**。
- SPD 独立三键采集、随机任务生命周期、接入过渡回归：分别 **4、1、10 项通过**。
- 隔离 domain **173**：合成 TCP 输入经真实 DLS／Hand2 和上游正式入口，直接 DDS
  进入 SPD 正式无窗口入口。真实 PTY 操作上游 R→S，SPD R 开段、R 检查点、S 暂停、
  D 回退并自动恢复、S→R→R 保存；SPD 暂停时，上游统计仍为 TELEOP。
- DDS 图检查：目标话题 **1 个发布端、1 个订阅端**，无 `/spd/pico/` 控制服务或话题。
- 保存 **6374 帧** `hardware_free` 场景轨迹，独立 `validate_episode` 与 `replay_episode`
  均通过：`complete=true`、`success=true`、恢复标注有效，机器人 qpos/qvel 最大恢复误差 **0**。
- 两个正式入口均通过本地 Q 正常退出（退出码 0）。测试源、临时脚本和临时数据随后清理。

没有连接真实头显或机器人，没有验收桌面虚影、任务物体交互、GPU 渲染吞吐或真实输入端到端延迟。

## 当前双臂 ROS 与唯一 DLS 后端验收

本节覆盖后文历史快照中的双臂后端、端口、构建和测试数量；历史结果不代表当前仍提供旧入口。

生产链为 `pico_arm_input → /pico/arm_input → tianji_arm_ros →
/tianji/controller/joint_targets → Python 执行器安全门控`。业务 UDP 15000／17000
不再用于生产双臂链；外骨骼输入及其手部 UDP 路线未改动。
ROS 输入／输出适配复用原生 DLS/Ruckig 控制循环，不增加第二套求解实现。
输入身份、epoch、撤销代际和实际应用输入的单调时间参与失鲜与撤销判定；
DDS 发布和回调位于控制循环之外。订阅、发布或采集服务都不授予运动权限。

Ceres LM、SPARK、mapped-palm、其他双臂求解分支及其专属入口、配置、构建依赖、
测试与对照工具已删除；历史由 Git 保存。公共代码使用 DLS／SharedRoot／Ruckig
命名，配置只使用 `shared_root`／`shared_root_shape`，不保留旧键别名。
冻结模型文件名中的 `ceres` 是资产名称，不是可选后端，模型内容未因本轮删除而替换。

| 实际检查 | 结果 |
|---|---|
| control 原生构建与完整 CTest | **34/34 通过**，覆盖 DLS、Ruckig、共享根、协议和实际 Viewer 集成 |
| 隔离 arm-ros 构建 | 通过；使用与 control 相同数值库版本，独立 Jazzy 消息与运行时 |
| default／policy 最终定向构建 | 各 **5 包通过** |
| 执行器／仿真／接口回归 | 分别 **240／22／33 passed** |
| policy Mocap 回归 | **85 passed** |
| 合成 typed ROS → 实际原生 DLS/Ruckig → typed ROS，domain 121 | **391** 个有效目标；初始姿态保持、撤销、epoch 改变、输入停止后的抑制通过 |
| ROS 目标与同次原生已提交参考比较 | 14 维最大差 **4.994e-12 rad**；此项不是异步跨运行轨迹逐位一致性证明 |
| 最终实际 Python 执行器 dry-run，domain 121 | **530** 帧；目标限位和输入新鲜度通过，未加载 SDK、未连接设备、未发送电机命令 |
| 最终实际仿真启动 | `DLS_SIM: WAITING`，model-reference，无目标导出，运行 2 秒后退出 0 |

原生测试同时修复了根 YAML 重复键未拒绝的问题。共享根形态学检查针对未经可达域
投影的原始偏好，保持原有数值容差，不再错误地要求投影后的目标等于未投影长度。

本轮没有做物理输入、真机反馈、设备使能或运动验收。Manus 新 ROS 手部目标仍未
接入执行器，不能宣称完整 Manus 真机采集已可用。用户正在运行的 SPD worker 及其
`install/spd` 部署未停止或覆盖；该部署不能算作本轮重新安装验收的结果。

## 采集双服务与三键门控（2026-09-22）

录制控制已从旧 `RecordingCommand` 切换为 `/start_collect`（StartCollect）和
`/stop_collect`（StopCollect），没有旧服务别名。请求按规范 UUID、执行会话、
phase_revision 与条目 ID 关联；start 的条目 ID 等于该请求 UUID。
同进程重复请求返回原完成结果或等待同一在途操作，UUID 改参数拒绝，
拒绝过的同一请求不会在授权条件改变后悄悄变成成功。

- r 等待实际开始的成功响应及匹配的当前 RECORDING 状态，再释放原执行器的跟随保持。
- s/d 先停止跟随，再请求保存／丢弃；只有本条完成响应、匹配终态和受控制动完成，
  才允许双臂 Home。迟到的上一条 stop 或状态不能作用于下一条。
- stop 的内部 abort 标志保留 partial，不等同操作者 d。慢写盘期间服务异步等待，
  不阻塞 DDS 图像、实测反馈、状态或执行器控制循环。
- 结果缓存仅在当前 collector 进程内有效；采集器重启后不自动重放未知结果。
  首次本地授权、硬件安全门控和 schema-v1 实测数据契约没有放宽。

| 检查 | 结果 |
|---|---|
| default 定向构建 | 新消息、执行器和采集器 3 包通过 |
| policy 构建 | 补齐已有缺失依赖后，6 包通过 |
| 执行器回归 | **241 passed，1 skipped**，含错条目结果不放行、未完成 stop 不 Home |
| 采集器非 DDS 回归 | **81 passed** |
| 完整真实 DDS 回归（domain 121） | **32 passed**，含重复 start/stop、过期条目 stop、拒绝结果重放、慢写盘等待和 partial 保留 |
| 独立冒烟 | 合成 ROS 相机／实测反馈源＋真实采集器，经两个真实服务保存 HDF5、丢弃下一条；重复请求不新建条目，旧服务不存在 |
| 实际执行器后台客户端 | 无硬件 `ExecutorObserver` 对真实 collector 发起 start/save，Future 仅在 RECORDING／IDLE 完成结果时返回，条目身份匹配且 HDF5 实际存在 |

原 DDS 回归有两处将现行有效的 `640,480,30` 当作错误模式；已改为从当前图像契约
构造必定不匹配的宽度，保留真实的模式拒绝／恢复检查，没有修改相机生产配置。
本轮没有连接相机、机器人或手套，也没有使能／运动。Manus 的新 ROS 目标尚未接入
执行器，不能据此宣称完整 Manus 真机采集已经可用。隔离验证进程与临时数据已清理。

## Manus Hand2 ROS 目标发布（2026-09-22）

本轮用户明确限定“先执行到把双手命令发送出来”：已将 Manus 生产输入切换为
`manus_data_publisher → manus_adapter → manus_hand2_retarget`，统一
`ros2 launch manus_bridge manus_hand2.launch.py user:=NAME`，Shell 入口保持
`bash bash/run_manus.sh --user NAME`。参考 `wuji_teleop-main` 的 SDK
`RetargetSession`，锁定 `wuji-sdk==2026.8.31`，没有旧 Pinocchio 算法回退。

- ROS 输出为 `/wuji/left_hand/joint_commands` 和 `/wuji/right_hand/joint_commands`，
  `tianji_interfaces/msg/HandJointCommand`，每侧 20 维 rad，具名固件顺序。
- 采集、21 点和目标消息保留每侧 glove/session/boot/sequence/源单调时间和 valid；
  SDK 回调时间不是手套内部采样时间，SDK 之前的传输延迟未被测量。
- 移除 rawviz 文本管道、私有重定向 worker 和 Manus TJH2 出口；
  外骨骼／裸手、原生双臂链、执行器授权及录制系统未修改。
- 原 ROS-free 重定向库继续用于其他离线工具，其构建入口迁到
  `src/teleop_outputs/wuji/wuji_retargeting/build_runtime.py`，不是新 Manus 依赖。

| 实际检查 | 结果 |
|---|---|
| default／policy 定向构建 | 各构建 `tianji_interfaces`、`manus_bridge` 成功 |
| `pixi run test-manus` | **32 passed** |
| `pixi run test-interfaces` | **33 passed** |
| 保留的离线重定向工具 | 独立 wheel 构建／安装成功；`pixi run test-retargeting` **18 passed**，移除仅覆盖已退役 Manus 文本管道的一项旧集成测试 |
| `pixi run check-env` | Jazzy／Python 3.12.14／Fast DDS／domain 120／LOCALHOST，通过 |
| 实际 ament 入口 | 仅 `manus_data_publisher`、`manus_adapter`、`manus_hand2_retarget`；launch 的 user 必填 |
| 合成骨架 → 真实 SDK → DDS，隔离域 121 | 左右各接收 **227** 个有效目标；检查 20 关节命名、有限值、源身份；左侧停发后失效、右侧继续，左侧恢复后重新有效 |
| SDK 回调夹具 → 实际原生采集节点 → 真实 SDK 求解 → DDS，隔离域 121 | 12 秒观测左侧 **867**、右侧 **1167** 个有效目标；夹具故意间歇断左侧，左侧 **3** 次失效，右侧 **0** 次。夹具仅为临时测试库，不属于生产包 |
| 实际 Manus 设备尝试 | SDK 检测到 Dongle；本次隔离域 12 秒观测无左右手原始骨架或关节命令，未取得真实手套端到端输出证据 |
| 真机运动／采集联动 | **未执行**；没有启动机器人执行器，没有连接／使能 Wuji 硬件，没有录制机器人数据 |

本次证明了命令发布软件链路，不宣称真实佩戴效果已验收。真实采集仍需现场确认双手
连接和有效骨架。新消息尚未接入 `run_teleop.sh --data`，不得按旧四终端 Manus 组合
直接做真机采集；后续由现有执行器在逐条录制确认开始后放行手部目标，并记录实测反馈。
本轮启动的测试／采集进程已停止，临时合成输入与 SDK 夹具已清理。

## 1. 环境、安装与原生资源

- 从未激活 Pixi 的 shell 执行 `bash bash/install.sh` 成功：安装锁定环境、重建外骨骼扩展、构建原生目标与 default/policy overlay、构建 Manus 和 PICO2 worker。没有启动设备。
- default、policy 均通过 `check-env`：Python 3.12.14、ROS 2 Jazzy、Fast DDS、domain 120／LOCALHOST，无外部 ROS overlay 混入。
- default 和 policy 的 colcon 各构建 18 个可用包；Odin 三包因缺少外部 mTLS 凭据明确跳过，未伪造凭据。
- `control` 原生程序安装到 `install/control`，ELF RPATH 指向该 checkout 的 `.pixi/envs/control/lib`，不借 default 的 Pinocchio／MuJoCo ABI。模型与共用 Home 由 `tianji_description` 安装；资源查找支持 colcon merged／isolated 两种布局，不向上猜工作区。
- `policy` 是独立的同版本 Jazzy/Python 环境，锁定 CPU PyTorch 2.10 与 Zenoh；不把它描述为已安装 CUDA 或已验证真实策略权重。
- ROS `run/launch/node/topic/service/param` 和 ABI 匹配的 tmux 已显式声明。PICO recorder 的安装后 `ros2 run`、`ros2 launch`、CLI/ROS 参数配置及无 TTY 启动均有行为验证。

## 2. 行为回归

| 实际执行入口 | 结果 |
|---|---|
| `pixi run test-native` | core **106/106**，mapped-palm **1/1** |
| `pixi run test-interfaces -q` | **32 passed** |
| `pixi run test-controller -q -rs` | **256 passed，1 skipped** |
| `pixi run test-collection -q` | **81 passed** |
| `pixi run test-sim -q` | **42 passed** |
| `pixi run test-pico -q` | **334 passed**，包含 PICO recorder 安装入口与 checkout 限定停止 |
| `pixi run test-pico2 -q` | **86 passed** |
| `pixi run test-manus -q` | **24 passed** |
| `pixi run test-retargeting -q` | **19 passed**，在 Manus 原生环境运行 |
| `pixi run test-mocap -q` | **85 passed**，在 policy 环境运行；原归档的五个回放／策略／安全测试文件已迁回活跃包 |
| `pixi run test-ros -q` | **29 passed**，真实 DDS 进程，domain 121 |
| 外骨骼独立 manifest 的 `pixi run test` | **323 passed**，含离线 integration |
| PICO2 `build_native.sh` | V131 **3/3**，手部 scheduler **1/1**，optimizer fixture 通过；16 帧 Python/C++ 对照最大误差 **0 rad** |

唯一 skipped 是 `test_mapped_palm_port.py` 的“explicit offline source checkout”外部对照项，不是主模型启动或本地 mapped-palm 回归。此前的 107 个 core 测试中，一个只回显配置文件路径、没有独立行为断言的测试已删除；其余默认算法、模型状态来源、UDP 绑定与实际运动行为回归保留，没有把旧路径断言改成新路径来凑通过。

DDS 回归包括：身份／phase_revision／新鲜度授权、相机先于反馈准备、重复帧和单侧旧手 token、不健康／回退／换 publisher／停流／执行器离开 TELEOP、队列溢出 partial、保存后 discard 不删除既有文件、即时状态转换、相机 profile／serial／配置 digest 认证、重新认证后的显式新录制、session 限定 abort，以及采集失败时独立软件 MotionGate 继续运行。

## 3. 实际程序 smoke

### 输入、推理与仿真

- 六个 Mocap 入口的 `--help` 全通过：`h5-sim`、`h5-real`、`infer`、`live`、`regrind-real`、`regrind-hand-sim`；另外完成九个操作入口帮助及 Manus `--check`、外骨骼 `--check-config`，共 17 个成功入口检查。
- default 与 policy 均运行了真实隔离 HandRetargeter 子进程和原始 H5 预计算，得到有限的 `(2, 20)` 结果。父进程没有导入 Pinocchio、nlopt 或 rclpy；数值仍由 Manus 环境中的原官方求解器计算，不是 Python 回退或模拟求解。
- 安装后的 VR Franka DLS、Ceres 均打开真实 MuJoCo 窗口，使用专属回环端口的合成 TJVR 输入，走通 **WAITING → S/TELEOP → P/HOLD → H/HOME_REACHED**，正常退出。截图确认机器人和轨迹面板可见；未连接头显。
- 本轮默认 PICO2 双臂＋双手 `--self-test --record` 走通 **S→H→S→Q**，4074 个周期，正常 Home/退出。共享根模式在 `90c575f` 接入轮已完成实际窗口 **C→S→H→P→Q**、双臂＋双手录制完成；本轮 86 个 PICO2 回归再次覆盖实际 worker、失鲜恢复、重连和录制。
- 所有上述仿真结果均不代表 200 Hz 实时性能合格或真实跟踪精度验收；程序仍报告 `real_time_qualified=false`。

### 独立相机／collector 复用

通过真实 ExecutorObserver／CollectionSupervisor 与独立 DDS fixture 进程运行，确认：匹配的相机／collector 被复用而不变成执行器拥有的子进程；本会话录制可 abort/drain；退出后外部进程存活；不匹配 task 被拒绝且不终止外部进程。测试未实例化机器人 SDK。

## 4. 三路原始 RGB 持续吞吐

无真实相机，用实际 CameraMonitor、驱动参数／DeviceInfo fixture、CollectorNode、DDS 服务客户端及独立预览进程完成 **60.008 秒**合成录制。配置为每路 **1280×720×30 RGB8**，双臂／双手反馈 120 Hz；使用仓库同机 Fast DDS SHM 配置，不声称 zero-copy。

| 流 | 接收 Hz | HDF5 写入 Hz | 写入数量 | 源序号缺口 | 最大接收间隔 ms | 最大存储间隔 ms |
|---|---:|---:|---:|---:|---:|---:|
| top | 30.012 | 30.000 | 1801 | 0 | 53.991 | 39.479 |
| left_wrist | 29.996 | 30.000 | 1801 | 0 | 84.748 | 41.178 |
| right_wrist | 29.996 | 30.004 | 1800 | 0 | 82.116 | 44.419 |
| arms | — | 119.984 | 7200 | — | — | 16.416 |
| hands | — | 119.984 | 7200 | — | — | 16.271 |

独立测量订阅者与 writer 的启停边界不同，数量不强求相等。三路 monitor 均实际读回 `1280,720,30`／`RGB8`；配对源缺口和未配对丢弃均为 0。预览请求 30 Hz，实际 UI **29.99 Hz**，各路 RX **29.96 Hz**；最大 UI 间隔 **107.331 ms**。这些间隔如实保留，不用平均帧率掩盖调度抖动。预览窗口已截图确认；合成内容主要为黑底和编码源序号，不是真实相机画面。

## 5. DDS 录制 → 压缩 → 真实网页

- 成功文件通过 `validate_episode(require_success=True)`，各路独立时间戳，双臂 14 维、双手 40 维，原始 RGB 尺寸不变。
- 通过实际压缩 CLI 生成 JPEG Q50。显式 destination 保留 `YYYYMMDD`；默认 destination 使用 `YYYYMMDD_compressed`，各自有独立配置。
- 原始文件 SHA256 压缩前后相同：`08693db8374b225ef16084f4f7e299ea304de30c3bc7bb6a487ef92450b54c40`。
- 从普通 wheel 安装目录（非 symlink install）启动浏览服务，Chromium 实际打开压缩后的 episode，看到任务／成功标记、三路帧数、14/40 关节图，播放再暂停到 1.005 秒，三张图片均解码为 1280×720。HTTP POST 返回 **405**，保持只读。

## 6. 仍需外部前提／现场验收

以下不属于“已通过的软件验收”，也不因迁移完成而获得动作授权：

- **真实输入**：PICO adb／M0 实流、真实佩戴者标定、Manus 连接、外骨骼 I/O／零位／方向和实际 TJH2 输入联合运行。
- **真实相机**：三台设备的身份、实际目标模式、曝光策略、真实画面、持续采集及断连恢复。合成 profile 认证不能代替物理设备的模式支持验证。
- **真机闭环**：只读身份检查、现场人工授权、实际反馈与失鲜保护、Home／停止／限幅、机械臂和灵巧手动作、安全空间与方向验收。
- **真实策略**：操作者授权的模型权重、参考录制、Motive 流及 GPU/CUDA 环境；CPU fixture 和帮助成功不等于这些已可用。
- **外部源码对照**：上述唯一 skipped 的 mapped-palm 对照需要显式提供外部 checkout。

本轮没有使用真实设备、没有运行 adb 连接命令、没有执行真机 enable/Home，也没有推送远端。

## 7. 主输入路线精简后的验证

按当前需求删除 `pico_recorder`、`imu_ros2`、`pico_odin`、两个 Odin ROS 驱动及其 SDK，
同时删除 PICO 中仅供外接脚部 IMU 使用的融合节点、启动配置、辅助状态类及专属测试。
四个主输入包、PICO 人员标定／会话所有权、PICO2 仿真录制和独立 schema-v1 collector 保留。
既有人体标定与数据没有删除；生成对称档案时不再复制已废弃的 Odin 附件。

- default／policy 的构建均成功，各 **16 个 ROS 包**，不再存在 Odin 凭据检查或跳过分支。
- 从两个环境的实际 ament index 确认被删除包不可发现，PICO、Manus、PICO2 和正式采集器仍可发现。
- `test-pico`：**322 passed**；保留的 PICO C++ 帧协议、地面对齐、epoch store：**3/3**。
- `test-sim`：**42 passed**；`test-pico2`：**86 passed**。
- `test-collection`：**81 passed**；真实 DDS `test-ros`：**29 passed**。
- `check-env` 和 `rviz2 --help` 通过；PCL／VTK 及只供已删除组件使用的 ROS 依赖已从 default／policy 锁文件与环境中移除。
- policy 环境检查通过，`test-mocap`：**85 passed**，确认推理／回放未依赖已移除的组件。
- 本轮只做构建和无硬件验证，没有启动真实输入设备或机器人。

## 8. 目录归类调整（teleop_inputs / teleop_outputs）

按“人侧输入、机器人侧输出”重新归类物理目录，**不改 ROS 包名、Python 模块名、可执行文件名、话题和线上协议**：

```text
src/teleop_inputs/    pico_controller/  manus/  exoskeleton/  pico_hand/
src/teleop_outputs/   tianji/{tianji_cmd_pub,tianji_controller,tianji_description}
                      wuji/{wuji_controller,wuji_retargeting}
```

未创建 `auxiliary/`，也没有恢复已删除的辅助包。

实际执行与验证：

| 项目 | 结果 |
|---|---|
| 裸 shell `bash bash/install.sh` | 通过；锁定环境、外骨骼与裸手独立环境在新前缀下重建，control／default／Manus／裸手原生目标全部编译 |
| default／policy 的 colcon | 各 **16 个 ROS 包** |
| `pixi run test-native` | core **106/106**，mapped-palm **1/1** |
| `test-controller`／`test-interfaces`／`test-sim`／`test-collection` | **256 passed（1 skipped）／32／42／81** |
| `test-pico`／`test-pico2`／`test-manus` | **322／86／24** |
| `test-ros` | **29 passed**，真实 DDS，domain 121 |
| `test-mocap`／`test-retargeting` | **85／19** |
| 外骨骼独立 manifest | **323 passed**（新前缀下重建扩展后） |
| 裸手 `bash bash/run_pico2_sim.sh --self-test --record` | 4104 周期，S→H→S→Q 正常 Home 退出 |
| 安装后 `bash bash/run_teleop.sh --sim --duration 3` | 实际 DLS 窗口启动，`control_state_source=model_reference`、`pico_udp_bind=127.0.0.1:15000` |
| `run_manus.sh --calibration-user … --check`／`run_exoskeleton.sh --check-config` | 均离线通过，未启动设备 |
| CLI／路由模块导入 | `tianji`、`tianji mocap`、`simulation.run_sim`、`tianji_controller.run_teleop`、`data_collector.node`、`tianji_cameras.monitor` 全部可用 |

模型、标定、设备备份和算法数值未改：**750 个模型／标定／备份文件**逐字节校验一致；`config/robot.json` 仅两项资源路径变化。

归类过程同时修复一处**既有指纹不一致**：删除辅助包时移除了 `pico_symmetric_geometry.py` 中的 Odin 附件拷贝，但该文件登记在共享根输入契约的 `source_files` 中。现按实际内容更新该指纹，并依序重链契约 SHA、两份几何 artifact 的 `tjvr_input_contract_sha256` 及原生允许列表与固定指纹测试；除注册指纹外，几何／运动学字段与算法数值不变。

上述均为无硬件验收；真实输入、相机、机器人动作与 GPU／模型现场条件仍见第 6 节。

## 9. 保留离线工具的默认启动收尾

本轮只修复资源定位与离线工具生命周期，不改目录、模型、控制算法或设备配置。
可执行文件统一使用 `native_executable()`，控制器 YAML 使用 `controller_profile()`，
模型和网格使用 `package_share("tianji_description", ...)`；profile 内引用通过
`controller_resource()` 解析。临时复制的 profile 保留原配置所指向的 Home、URDF
和几何 artifact；显式相对路径仍按调用者工作目录解释，不再强制子进程切回源码目录。
没有添加旧 build 路径回退或兼容软链接。

实际安装 `default`／`control` 锁定环境并执行 `pixi run build`，control 原生目标及
default 的 **16 个 ROS 包**构建成功。以下工具均从仓库外
`/tmp/tianji-offline-az02bc5a` 启动，使用安装 overlay；没有覆盖 `--binary`／`--viewer`，
也没有用 `--help` 代替执行：

| 工具／流程 | 实际结果 |
|---|---|
| `record_shared_root_actions.py record --source-kind synthetic` | 回环 UDP 持续录制 50 秒，**5000 帧**；`complete=true`、`motion_authorized=false`、`actions_confirmed=false` |
| `validate_shared_root_contract.py` | 默认安装 profile、契约及几何指纹一致 |
| `report_shared_root_actions.py` | 5000 帧录制全部读取；几何有效 5000 帧，未伪造动作标注或现场验收 |
| `report_shared_root_layers.py` | 500 帧合成轨迹生成分层报告，1998 个双侧诊断样本 |
| `audit_shared_root_trace.py`／`audit_synthetic_body_mapping.py` | 独立合成标定与 500 帧骨长一致轨迹，完成几何审计和体型扰动分析 |
| `view_shared_root_mapping.py` | `--check` 通过；实际 MuJoCo 窗口播放至第 500 帧，截图确认模型和映射覆盖层，Q 正常退出 |
| `run_shared_root_three_way.py` → `report_shared_root_three_way.py` | SPARK／Ceres／DLS 各回放完整 5000 帧录制并生成 CSV 和报告；5～47 秒报告窗口有 4200 个共同源帧 |
| `view_shared_root_comparison.py` | 实际 SPARK／Ceres 双窗口、同一回放时钟，各发送 5000 帧，`complete=true`；截图确认模型与曲线可见 |
| `run_cartesian_frf_benchmark.py --smoke --jobs 1` | 默认 binary／config／model／URDF 完成两组 case，自动分析并生成结果 |
| `run_pico_trace_algorithm_benchmark.py` → `analyze_pico_trace_algorithm_benchmark.py` | SPARK velocity-QP 完成 500 帧回放；分析得到 500 个共同源帧、469 个 clean 源帧 |

窗口使用临时 Xvfb 显示，不依赖真实桌面或硬件。实际 Q 退出暴露了 MuJoCo passive
viewer 的异步销毁竞争；mapping 工具现在在关闭 viewer 后等待本次启动的线程结束，
再次运行实际窗口并按 Q，退出码为 0。

12 个相关 Python 回归文件：**151 passed，9 skipped**。9 项跳过均需要未随仓库提供的
既有人员录制，不是默认资源查找失败；另将几何审计测试原来依赖私人 `profiles/syz`
文件的 fixture 改为确定性的合成标定，6 项行为测试全部执行。

边界：三路输入端到端仿真、真实相机与真机闭环不计入本轮验收。合成微动轨迹只证明
离线工具可运行，不能作为算法排名或动作安全结论。外部 Ceres 历史源码对照工具
`prepare_shared_root_ceres_trial.py` 的本地默认资源已迁移，相关单元回归通过；完整外部
对照仍要求显式 `--source-root` 提供
`config/qp_ik_pico_ee_franka_ceres_lm_ruckig_mujoco.yaml` 及配套源码。当前 checkout
和文档记录的外部目录均无该配置，本轮不宣称完成这项外部对照。

复现默认路径启动的方式（`TRACE` 必须是已有离线 TJVT/TJVR 文件）：

```bash
pixi shell --manifest-path /absolute/path/to/tianji_teleop-ros2/pixi.toml
source "$TIANJI_WORKSPACE/bash/environment.sh"
TOOLS="$TIANJI_WORKSPACE/src/teleop_outputs/tianji/tianji_controller/native/scripts"
cd "$(mktemp -d)"
python "$TOOLS/run_cartesian_frf_benchmark.py" --smoke --jobs 1 --output-root frf
python "$TOOLS/report_shared_root_actions.py" "$TRACE"
python "$TOOLS/view_shared_root_mapping.py" "$TRACE" --check
python "$TOOLS/run_pico_trace_algorithm_benchmark.py" --trace "$TRACE" \
  --output pico --algorithms spark_upper_qpoases_velocity_qp
python "$TOOLS/analyze_pico_trace_algorithm_benchmark.py" \
  --manifest pico/manifest.json --output pico-report
```

本次临时数据、CSV、报告、日志和截图保留在上述 `/tmp` 目录；临时启动器与显示进程已清理。
持久数值记录见 `migration-verification-results.json` 的 `offline_tools_cutover`。

## 10. 三路合成输入到实际仿真的端到端验收

本轮完成的是**合成设备输入 → 生产处理链 → 实际 MuJoCo 仿真**，不是实际佩戴者或
机器人验收。未改目录、模型、算法参数或真实标定；未打开相机、机器人 SDK 或 Manus
采集 SDK。原始日志、录制、截图及数值检查保留在 `/tmp/tianji-routes-hjc5phk_`。

### 实际发现并修复的启动／退出问题

- `start_tianji_mujoco_teleop.launch.py` 的 Node 仍指向 `pico_bridge`，导致找不到
  `tianji_mujoco_teleop_bridge`。改为其真实包 `tianji_cmd_pub`，重装后实际收到合法
  656 字节 TJVR v4；既有原子 UDP 行为回归改为走正式 `ros2 launch`。
- `bash/install.sh` 原来先安装 default/policy overlay、后构建 Manus `rawviz`，
  首次安装会漏掉 collector。现在先完成 `build_manus.sh`，再构建两套 overlay。
  实际构建 collector 并重装 default 包后，`run_manus.sh --calibration-user syz --check`
  通过；这里的 `syz` 仅用于离线资源检查，不是指定现场佩戴者。
- PICO2 实际窗口完成 Home 和录制后仍曾异常退出。按照离线 mapping viewer 的现有
  生命周期模式，关闭 passive viewer 后等待本次启动的渲染线程结束，再释放资源；
  V131 和 shared-root 实际窗口再次运行均退出 0，录制完整。

### 注入边界与实际运行链

| 路线 | 真正执行的生产链 | 明确排除的硬件边界 |
|---|---|---|
| 共用 PICO 手柄 | 合成 APK TCP 分片 → `pico_bridge_node` → ground normalizer、双侧 TCP 校正、M0 filter → 正式 ROS launch → TJVR → 默认 `run_teleop.sh --sim` | 头显、ADB 转发、真实人员标定 |
| Manus | 合成 `HAND/NODE/POSE` 文本 → 生产 parser → `/hand_input` → ROS bridge → 隔离 manus worker、真实 Hand2 重定向 → TJH2 → 仿真 | SDK 采集、手套配对、实际佩戴 |
| 外骨骼 | 回环 HTTP 元数据及 `encoder-v1` TCP 分片 → 真实身份／协议校验、零位、四连杆、FK、native MANO、官方 worker → TJH2 → 仿真 | USB 网卡发现和网络配置；临时副本仅将网络端点改为回环，方向验收标记不变 |
| PICO 裸手 | 合成 1982 字节 TCP 帧 → 生产 receiver/parser → V131 或 shared-root DLS/Ruckig → 两个真实 native Hand2 worker → 仿真及 HDF5 | 头显、真实手部追踪 |

手套链额外使用临时 UDP 观察中继 `16001 → 16000` 保存原始包；逐字节转发，不修改
时间戳、关节值或新鲜度。不是跳过采集／重定向直接生成 TJH2。各路线串行运行。

清理检查发现不属于本轮的旧工作区 PICO 进程 `1467197`
（`tianji_teleop/tracking/install/pico_bridge/...`），未擅自停止。两条手柄组合路线
随后在**独立 DDS domain 121**重新完成运行和故障场景，避免共享域干扰；默认生产域
120 的配置未修改。

| 最终证据 | 结果 |
|---|---|
| 独立域 Manus 组合路线 | 仿真实际接受 **5905 个 TJH2 包**，坏包 0；双手输出关节变化约 0.805／0.802 rad，双臂参考变化约 0.661 rad |
| 独立域外骨骼组合路线 | 仿真实际接受 **755 个 TJH2 包**，坏包 0；双手输出关节变化约 0.725／0.770 rad，双臂参考变化约 0.545 rad |
| 外骨骼关闭并重启 | 单侧故障不会自动恢复；干净停止执行端和发送端后重启，两侧有效，额外接受 629 包并完成 Home |
| V131 故障场景录制 | 7236 个原始 TCP 帧、23116 个 54 维仿真采样；失鲜／旧帧重放／无效侧的稳定保持区间变化均为 **0 rad** |
| V131 退出修复后复跑 | 1315 个原始帧、2397 个采样，双臂双手有响应，Home 误差 0，录制完整，退出 0；另一次有时限窗口复跑亦退出 0 |
| 裸手 shared-root | 3846 个原始帧、12555 个采样；Home 双臂误差约 `1.1e-16 rad`，检查的 H 回程手指保持区间变化 **0 rad** |
| 现有回归 | `pixi run test-pico2 -q`：**86 passed**；正式 ROS launch 原子 UDP 回归：**1 passed** |
| 默认离线入口检查 | Manus `--check`、外骨骼 `--check-config` 均通过，不打开设备 |

动作与故障边界：

- 两条手柄组合路线走通 WAITING → S/TELEOP → 失鲜/HOLD → 新输入仍 HOLD → S；
  PICO TCP 断开重连不自动接管，H 后可用 P 取消回程，最终正常 HOME_REACHED/退出。
- Manus 单侧停止或重放旧文本不会刷新该侧源时间；另一侧继续更新，新鲜输入恢复。
- 外骨骼短停流后可恢复；TCP 单侧故障清除该侧有效位和时间戳，另一侧继续，
  故障侧要求重启，不使用旧值或退出零值补包。
- V131 同一连接失鲜保持后可随新鲜输入继续；TCP 换代则返回 Home，等待再次 S。
  它不采用 shared-root 的 P/空格取消 Home 约定。
- shared-root C 前拒绝 S；长断流保持并要求 S；短断流经过稳定窗口和制动后软恢复；
  TCP 重连使 C 失效，必须重新 C 后 S；H 只回双臂且手指保持，P 可以取消 H。

记录显示真实渲染与计算存在迟到周期，`real_time_qualified=false`；以上关节幅度只用来
确认响应，不是动作精度、优化质量或 200 Hz 实时性能的合格结论。

### 真实输入阶段仍阻塞

只读 `lsusb` 枚举到 Manus Sensor Dongle `3325:0049`，未枚举到 PICO USB 设备；
`ip -j link show` 仅有 lo、enp3s0、wlx08beac41f6ab，没有外骨骼所需的
`02:33:80:00:00:01` 网卡。工作区 `profiles/` 不存在，实际佩戴者及其完整 PICO／Manus
档案尚未指定；Manus dongle 存在不等于手套已配对、佩戴或已提供有效骨架。

因此三路**真实输入 → 仿真**仍待设备与人员前提，不能标成完成。继续现场输入前还须
由操作者处理上述旧 PICO 进程；本轮没有改动它。真实相机与经现场授权的真机闭环
保持后续阶段。所有本轮拥有的进程已停止，TCP 10002/19999 和 UDP 15000/16000/16001
均已实际重新绑定验证释放；临时生成器和显示运行器不进入仓库。

## 11. zjx 真实输入切换尝试（后续现场状态）

用户确认 PICO 使用有线 ADB、Manus 接收器已连接，并确认复用 `zjx` 标定。
此处记录第 10 节之后的状态变化，不将之前的 USB 枚举结果当作当前连接状态：

- 起初 `adb devices -l` 显示 PICO A9210 为已授权的 `device`，序列号
  `PA9210MGK9250095G`；已有 `9999 → 9999`、`10002 → 10002` 转发。
- 将旧工作区 `zjx/pico-simple` 的活动版本 `cal-cea5ec573c7d42dda2edcddfb0cc3d4b`
  复制到当前 `profiles/zjx/pico-simple/`：7 个标定文件 SHA256 全部一致，另复制
  `active.json`，旧文件不改。身高为 1.75 m，手腕距离仍为身高模板估计，
  `hardware_acceptance_complete=false` 未改。
- `run_manus.sh --calibration-user zjx --check` 通过。实际运行 Manus SDK 后识别到
  MetaglovePro 接收器；尚未确认手套骨架流，5 秒观察窗口内没有 TJH2 输出。
- 初次新入口因旧 tmux 会话的 checkout 身份不匹配而拒绝接管。用户随后明确授权
  切换；仅在 `@tianji_checkout` 仍等于旧 `tianji_teleop/tracking` 时停止该
  `pico_tianji_teleop` 会话，确认旧输入进程退出，未删 ADB 转发或原标定。
- 重试时 PICO 从 ADB 设备列表消失，普通 shell 与 Pixi 都报告无设备；
  `run_teleop.sh --sim --user zjx --duration 30` 在 ADB 前置检查处退出，
  **没有启动机器人仿真，也没有完成真实输入验收**。未把错误归因于错误的标定或
  修改安全检查来绕过它。

本次拥有的 Manus 采集及临时显示进程均已停止；没有连接机器人或相机。后续需要
恢复 PICO 的 ADB `device` 状态并运行相应采集应用，同时确认 Manus 手套开机、
配对且输出骨架。日志已去除 SDK license key 行，保存在
`/tmp/tianji-live-zjx-uzicb6r_`；本轮允许保留的人员档案已在新工作区就位。

## 12. real／data 统一为共享根 DLS＋Ruckig

`--real` 和 `--data` 共用执行器现只接受 `franka-dls`，默认配置为
`qp_ik_pico_shared_root_dls.yaml`，算法 `pico_ee_franka_dls`，
`post_smoothing.mode=ruckig`。默认模型切到与冻结几何一致的
`marvin_m6_wuji2_shared_root_ceres.xml`；资源名称不是 Ceres 后端选择。
运行副本开启共享根、绝对化资源引用、以只读反馈初始化双臂，并将
`controller_velocity_scale` 作用于实际 Ruckig 速度上限，不仅修改旧 QP 的字段。

这不是简单撤掉仿真出口限制：原生新增显式 `--franka-dls-executor`，只允许
DLS／Ruckig、model-reference、velocity、headless＋continuous 与回环输入／输出。
其他共享根后端、普通仿真和跳过 PICO 跳变检查的模式仍不能借此导出。
输出使用双臂已提交的 Ruckig 参考、双手独立时效标志；求解后复查 mapping、
源时效、epoch／reset，停止时撤销所有 ready。原生程序没有硬件使能权限。

Python 的 SDK 身份／反馈／限位、慢速对齐和分阶段 Enter 授权保持；
`--data` 的相机／writer prepared-before-connect、输入 ready-before-enable、
会话所有权与录制无运动授权保持。旧 real 的 mapped-palm 参数、私有事件／
C 标定集成及宽限停流分支已移除，不提供旧后端回退。历史 native／offline 对照保留。

本轮实际执行：

| 验证 | 结果 |
|---|---|
| `pixi run build` | control 两套原生目标及 default 16 个 ROS 包成功构建／安装 |
| `pixi run test-native` | core **106/106**，历史 mapped-palm 模型 **1/1** |
| 执行器、仿真、采集 Python 回归 | **343 passed，1 skipped**；跳过项需要显式外部 mapped-palm 源码对照 |
| `pixi run test-ros -q -rs` | **29 passed**，真实 DDS、合成相机／反馈；非真实设备采集 |
| `pixi run test-pico2 -q` | **86 passed**，确认共享 DLS 核心未破坏裸手链路 |
| 共用配置加载器的 Mocap 调用回归 | **37 passed，4 skipped**；default 环境无 torch 的 policy 场景未运行 |
| 正式 real／data 包装器传旧后端 | 均在 argparse 拒绝，退出 2，未启动设备／采集 |

实际安装后的执行器从仓库外 `/tmp/tianji-dls-executor-u3jwe54d` 运行，
使用临时端口、合成 TJVR／TJH2 和无 SDK 的 dry-run，不是只运行 `--help`：

- 正常样本 `flags=7`；PICO 失鲜变为 `6`；同 epoch 的新鲜输入恢复为 `7`。
- 仅右手失鲜变为 `5`，左手与双臂保持 ready；右手新鲜输入恢复为 `7`。
- epoch 改变后保持 `flags=6`，不因新输入自动重新授予双臂 ready。
- 标准倍率 1 的已提交参考速度峰值约 **4 rad/s**；倍率 0.05 的峰值约
  **0.2 rad/s**，均按实际 TJRC 序列和控制周期计算，未只检查配置回显。
- 仅双手独立运行时没有任何 PICO 帧，正常 `flags=6`、右手失鲜 `flags=4`，
  全程没有双臂 ready。
- 三次 dry-run 的 hardware factory 调用数均为 0，flight recorder 没有设备
  feedback 或 motor send；未连接机器人、相机或实际输入设备。

还修复了实际二次构建暴露的依赖问题：旧 FetchContent 的 `Ceres_BINARY_DIR` cache
会使已安装的 Ceres 2.1 package 跳过导入 target。依赖发现前清除这个过期身份后，
正常重复构建通过；没有用关闭历史依赖或兼容路径绕过错误。
安全回归同时捕获并清除 Home 转换中遗留的旧 dropout 状态引用，原有 Home／静止／
反馈故障测试重新全部通过。

原始 session、运行时 YAML、flight recorder 和分阶段数值保留在上述 `/tmp` 目录；
临时 smoke 启动器已清理。**本节是代码接入与离线验收，不是 DLS 真机闭环、
真实相机持续采集或现场动作安全验收。**

## 13. 裸手移除 V131，入口改名为 run_pico_hand_sim

裸手现在只保留共享根掌心映射＋Franka DLS＋Ruckig。新入口为
`bash bash/run_pico_hand_sim.sh --height-m HEIGHT`，身高必须显式给出；
旧脚本和 `--mapping-mode` 已移除，没有别名或回退。`pico2_hands` Python／ROS 包名及
`pico2_sim_session_v1` 录制 schema 保持不变。

删除 V131 原生库、worker／probe／model trace、专属模型和头文件，以及旧 Python
IK worker、旧映射／可选标定、五次 Home 和对应专属测试。`DlsWorker`、共享根 owner
不再继承旧后端。Hand2 数学、模型与独立 ABI 保留，手部构建只负责 Hand2，
双臂使用主工作区安装的 `pico2_dls_worker`。已清理本次会话先前生成的
`native/build/pico2-v131` 和 13 个旧 Python 字节码缓存；人员标定及历史录制未删除。

`--self-test` 已迁到真实 DLS／Hand2：先确认 C 前 S 被拒绝，再执行 C/S/H/S/Q，
不绕过校准，不从测试模块借运行时输入 fixture。保留只读观察器和
`pico2_hands.scripts.smoke_pipeline`，后者复用同一生产自测，不再维护第二套求解链。

| 本轮实际验证 | 结果 |
|---|---|
| `pixi run build` | 主原生目标及 default **16 个 ROS 包**构建／安装成功 |
| 裸手 `build_native.sh` | Hand2 scheduler **1/1**、优化器夹具通过；16 帧官方手部对照最大差 **0 rad** |
| `pixi run test-pico2 -q -rs` | **63 passed**，无跳过；旧路线专属测试已随实现删除 |
| 新脚本、仓库外运行、`--self-test --record` | **1575 周期／1575 原始帧**，Home 最大误差约 `3.3e-16 rad`，录制完整、退出 0 |
| 真实 TCP 分片源＋实际 MuJoCo 窗口 | **16851 周期／12927 原始帧**，截图确认 DLS 及模型；短失鲜恢复、长失鲜人工 S、P 取消 Home、重连重新 C、最终 Home／退出通过 |
| GUI H 回程的手指保持 | 检查 25 和 669 个采样，变化均 **0 rad** |
| 干净 wheel 构建、独立目标目录安装 | 包内没有旧 V131／旧 Python 求解器；从实际 wheel 包运行 DLS/Hand2 自测 **1592 周期**，录制完整、退出 0 |
| 保留的只读 `observe` | 真实回环 TCP 观察 **97 帧**，双臂／双手观察有效，正常关闭；不创建执行器 |
| CLI 拒绝边界 | 缺少身高、旧模式参数退出 2；未尝试连接设备 |
| 保留代码来源指纹 | **5 个接收参考＋28 个 native/vendor 记录**全部匹配 |

指纹核对发现一项已有遗漏：Hand2 CMake 早已从原 `Eigen3 3.4` 请求改为使用独立
Pinocchio 4 环境的 Eigen CONFIG，但 manifest 仍写 unchanged。已与旧工作区原文件
及其 SHA256 对照，只补记这项既有构建适配的目标指纹；没有修改手部数值源码。

运行记录、HDF5、截图、wheel 和校验结果保留在
`/tmp/tianji-pico-hand-dls-q5ea_ryx`。临时源、显示进程和验收脚本已清理。
前面历史章节中的 V131 数字和当时脚本名仍是历史证据，不表示旧实现继续存在；
当前操作只按新入口执行。本轮未连接头显、机器人或相机，仍不宣称实时性能或现场
跟踪质量验收通过。

## 14. 裸手默认向 SPD 发布 ROS 关节命令

`bash bash/run_pico_hand_sim.sh --height-m HEIGHT` 默认发布
`/spd/tianji_wuji2/v1/joint_command`，类型为 `tianji_spd_interfaces/msg/JointCommand`。
独立发布线程只取最新 54 维控制目标，最多 60 Hz；UTC 时间保留生产周期起点，
与原生 Viewer 共用 `core.tick()` 的输出，但不从显示状态读取命令。
未标定失效、C／连接／epoch 会话边界、逐手有效性与失鲜、P/H/Q 制动／保持／回程
均有显式 readiness；不沿用本地显示的固定 flags=7。

本次没有修改 Manus／外骨骼＋PICO 手柄的真机输入、执行器或授权，不新增硬件出口。
SPD 消息包以固定指纹快照在本工作区生成 Python 3.12 绑定；运行不依赖 SPD 源码路径。

| 本轮实际验证 | 结果 |
|---|---|
| `pixi run build --packages-select tianji_spd_interfaces pico2_hands` | 原生目标及两个选定 ROS 包构建／安装通过 |
| `pixi run test-pico2 -q -rs` | **77 passed**，无跳过；含 readiness、会话、失鲜、非阻塞发布和异常清理 |
| domain 121 真实 Fast DDS＋完整 DLS/Hand2 自测及录制 | 最终复跑收到 **481 帧、2 个会话**，ready 为 0／7；收到目标与录制的 54 维命令最大差 **0 rad**，最后 Home 目标已收到；发布发生在现有完整模型校验之后 |
| SPD 当前 `snapshot_from_ros` 验证器 | 对以上收到帧检查类型字段、配置、名字顺序、有限性、会话内序号、UTC 新鲜度，全部通过；这不是 SPD 物理执行验收 |
| domain 121 真实 DDS 生产停止更新 | 收到一次新序号、新 UTC 的 mask0 失效通知；没有重放旧健康命令，新生产输出的序号继续递增，SPD 协议验证通过 |

实际 QoS 为 BEST_EFFORT／KEEP_LAST1／VOLATILE。合成源及验证脚本未连接头显、机器人
或相机；未启动 SPD 动力学／场景／录制。双方模型限位、初始双手目标、零位／方向、
SPD 本地启用与物理到位仍须按接收端最终配置验收；ROS 接收成功不替代这些检查。
自测强制隔离到 domain 121，普通域 120 未注入合成目标。临时验证脚本与 HDF5 已清理。

### 裸手默认 ADB 启动预检

默认本机 10002 输入在启动 ROS／Viewer 前自动检查设备和转发，复用匹配规则或以
`--no-rebind` 补建；冲突与未授权拒绝启动，多设备要求显式 `ANDROID_SERIAL`。
不改动其他输入路线，自测／自定义 host 或 port 不操作 ADB。

- 真实头显上验证现有 `tcp:10002` 转发复用。
- 在独占临时端口 **39373** 验证真实 ADB 缺失补建与重复复用；结束只删除该临时转发，
  原 `tcp:10002` 未改动，不连接头显数据流或发送关节命令。
- ADB 所有权与现有自定义 TCP 入口回归：**6 passed**。
- 禁止任何 ADB 预检调用的真实 DLS／Hand2＋domain 121 自测：**1604 周期**，
  正常 Home 退出，证明 `--self-test` 旁路 ADB。

## 15. top RGB 经 H.264 传入 PICO

新增 `bash/run_pico_camera.sh`／`pixi run pico-camera`，从唯一采集配置读取 top，
只订阅现有官方驱动的 RGB8 1280×720 图像；不重复打开 RealSense，不启动机器人或记录图像。
复用给定 PC 推流协议：13579 上的混合端序 OPEN/CLOSE_CAMERA 控制帧，
12345 上的 4 字节大端长度＋H.264 AnnexB access unit，左右眼同源 SBS。
软件编码使用已有 GPL FFmpeg 8.1.2/libx264，现已显式锁定依赖；没有新增 PyAV 环境。
ADB 只管理视频 reverse/forward，保留裸手 10002 和其他既有映射。

| 本轮验证 | 结果 |
|---|---|
| `pixi lock` 与 camera 包构建 | 锁文件已满足 FFmpeg 显式约束；`tianji_cameras` 构建／安装通过 |
| 相机协议／ADB 所有权回归 | **12 passed**，包含分片／连续消息、超时失步、非法请求、冲突与回滚、空 reverse 列表 |
| 真实 ROS（domain 121）→ 正式 camera 入口 → FFmpeg → PICO 协议接收端 → FFmpeg 解码 | 30 fps 收集解码 **18 帧**，1 fps 收集解码 **3 帧**；每帧左右眼像素平均差 **0**，合成颜色随时间变化，非冻结重复帧 |
| OPEN/CLOSE 与源失鲜 | 同一控制连接重复开关两个流；停止 RGB 后关闭视频并报错，没有回放旧帧 |
| 无输入及不读取视频的接收端 | 分别在初始源等待／新鲜度边界失败，编码线程退出；FFmpeg 由拥有者终止并回收 |
| 真实 PICO USB 上的视频映射 | reverse 13579、forward 12345 建立及清理通过；前后既有 forward/reverse 表一致，10002 未改 |
| 裸手回归 | **81 passed**，无跳过 |

实际运行检查修正了私有 ROS Context 必须使用对应 Executor，以及真实 `adb reverse --list`
空行输出的解析问题；修正后重新完成上述验证。临时测试脚本与内存／临时日志已清理。

**现场边界：**本轮设备枚举显示配置中的 top／左右腕相机均未连接，所以没有验证实际 top 画面。
APK 元数据能看到 OPEN_CAMERA／PicoH264Decoder 相关标识，但未在头显中验收画面显示。
需要连接相机、启动其官方 ROS 节点并在 PICO 启用 PC 视频源后，另行确认实际画面、延迟与视场。

## 16. 备份四终端配套恢复（2026-09-24）

按用户指定备份 `tianji_teleop-ros2.6S2HXDkc/tianji_teleop-ros2` 恢复相关源码，
不覆盖整个分支、不复制旧 build/install/.pixi。入口为 PICO 前台、Manus、
`run_camera_views.sh`（相机＋PICO 视频＋RViz）、`run_teleop.sh --data --task TASK`。
完整恢复 Manus 仿真 ROS 输入、TeleopReference 原生接口配套、手部撤销代际、
相机图发现及采集键时刻截止；双臂仍为共享根 DLS／Ruckig。
`profiles/kj/manus/` 两份标定与备份 SHA256 一致；未加载到真实手套验证。

| 本轮验证 | 结果 |
|---|---|
| `pixi run --locked -e default build` | control、arm-ros 和 default 15 个 ROS 包构建通过 |
| 输入／仿真／相机／接口／执行门控／落盘定向 Python 测试 | **318 passed**；含真实 headless 仿真与合成 FFmpeg 编解码 |
| domain 121 采集 DDS 定向测试 | **3 passed**；发现未收敛后恢复、请求重放与截止时间、非法截止不夺取终结权 |
| 原生定向 CTest | **5 passed**；DLS、Ruckig、episode reference、配置与手部协议 |
| 四入口 CLI smoke | PICO help、Manus 人员列表（含 kj）、camera_views help、teleop help 均退出 0 |
| 独立真实原生仿真 smoke | domain 121、2 秒、headless；Manus 双手 ROS ready，DLS_SIM WAITING，关节导出 disabled，退出 0 |
| 中文 HUD 合成帧 | 实际渲染并查看中文提示；原始输入不变，下半幅像素不变 |

当前验证进程没有 DISPLAY/WAYLAND_DISPLAY，未启动实际 RViz 窗口、相机、ADB、
Manus 手套或机器人。上述结果不代表现场图像、跟随、延迟、碰撞或运动安全验收。

### 终端 4 按备份直接覆盖

用户报告真机启动 `arms: source input is not fresh/ready` 且没有目标虚影后，
明确要求停止局部修改、直接复制备份。已覆盖并逐字节校验终端 4 的执行器、
监视窗口、原生控制、仿真、ROS 接口、采集器及构建入口共 194 个源码／资源文件。
机器人配置、现场进程和原始失败日志未改；没有复制备份 build/install/.pixi。
重新构建通过（default 15 包），目标显示与 ROS 接收定向测试 71 项通过。
关键运行源码在此次覆盖前已与备份相同；未重跑真机，不能据复制和测试宣称该次 ready／虚影故障已消失。
