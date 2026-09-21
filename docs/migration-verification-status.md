# 迁移验收状态与未验证范围

基线为本地 `main@90c575fe1dc6b02929a4f701a2f1263b8b347fd0`。当前实现已按 [protype-style 结构方案](protype-style-data-collection-structure.md) 切换到 `bash/`、`config/`、`src/` 和分环境安装目录；不是旧根目录入口的兼容层。以下为 2026-09-21 的实际软件验收，硬件边界单列。数值证据见 [JSON 记录](migration-verification-results.json)。

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
- **Odin**：缺少 `src/teleop_inputs/odin/odin-sdk2/sdk/utils/http/certs/certs.h`，`odin_ros_driver`、`odin_ros_driver_rev1`、`pico_odin` 未构建；凭据必须现场提供，不入库。
- **外部源码对照**：上述唯一 skipped 的 mapped-palm 对照需要显式提供外部 checkout。

本轮没有使用真实设备、没有运行 adb 连接命令、没有执行真机 enable/Home，也没有推送远端。
