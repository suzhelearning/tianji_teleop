# pico_project

[English](README.md) | [简体中文](README.zh-CN.md)

PICO 4U 数据采集与脚部姿态融合流水线。本 ROS 2 Humble 工作空间使用根目录 `.venv`，
包含 PICO 数据流、双 IMU900 输入、鱼眼相机、Odin 里程计、数据录制和 MuJoCo 可视化。

## 快速启动：PICO 掌心约束骨架

快速脚本默认使用 `ROS_DOMAIN_ID=120`、`ROS_LOCALHOST_ONLY=1`，并关闭
ROS 2 CLI daemon。标定和 PICO 单独验证使用根目录 `.venv` 与 ROS SDK；所有启动脚本
会自动加载同一环境。

终端 1 启动 PICO 驱动：

```bash
cd /path/to/tianji_teleop/tracking
source scripts/environment.sh
./scripts/start_pico_driver.sh
```

新手套或新操作人员在终端 2 打开统一标定菜单。可以任选左右侧和单项标定，也可以
按 `TCP → 掌心到手腕 → 上臂/前臂骨长` 的顺序完成整侧适配：

```bash
cd /path/to/tianji_teleop/tracking
source scripts/environment.sh
./scripts/calibrate_pico_arm.sh
```

只查看当前标定状态，不修改任何文件：

```bash
./scripts/calibrate_pico_arm.sh status
```

完成标定后根据用途选择运行方式。若只验证 PICO 掌心约束骨架，在终端 2 运行：

```bash
./scripts/start_pico_m0.sh --viewer
```

若要进行天机 PICO 遥操，请先确认左右侧均已按
`TCP → 掌心到手腕 → 上臂/前臂骨长` 完成标定，然后直接运行：

```bash
./scripts/start_tianji_pico_teleop.sh
```

该一键脚本会先清理历史遥操实例，再自动启动 PICO driver、M0 修正骨架及其
MuJoCo Viewer、ROS 2 → TJVR bridge；无需再分别运行前述 driver 和 M0 命令。
结束遥操时运行：

```bash
./scripts/stop_tianji_pico_teleop.sh
```

两个运行入口都会自动验证并加载左右侧有效 artifact，不会从 `recordings/` 中猜测或
选择候选文件。

MuJoCo 主骨架来自 `/pico/smpl_palm_corrected_ik`。它与
`/pico/smpl_palm_corrected` 的 24 个关节位置完全一致，只调整肩、肘局部坐标轴以
匹配 IK 约定。运行后检查：

```bash
ros2 topic hz /pico/smpl_palm_corrected
ros2 topic hz /pico/smpl_palm_corrected_ik
ros2 topic echo /pico/smpl_palm_corrected/status --once
```

### 一键启动天机 Spark 遥操的 PICO 侧

左右臂已有有效标定文件后，一条命令即可启动 PICO 驱动、M0 修正上肢骨架、
PICO 骨架 MuJoCo Viewer 和 ROS 2 → TJVR bridge：

```bash
cd /path/to/tianji_teleop/tracking
./scripts/start_tianji_pico_teleop.sh
```

脚本管理名为 `pico_tianji_teleop` 的 tmux session，包含 `driver`、`m0` 和
`bridge` 三个窗口。按 `Ctrl-b n` 切换到下一个窗口，按 `Ctrl-b 0/1/2` 直接选择，
按 `Ctrl-b d` 退出界面但保持后台运行。每次正常启动都会先清理旧工作目录和所有
ROS domain 中残留的 driver、M0 与天机 bridge，再创建唯一的新 session。管理命令为：

```bash
./scripts/start_tianji_pico_teleop.sh --detach
./scripts/start_tianji_pico_teleop.sh --status
./scripts/stop_tianji_pico_teleop.sh
```

bridge 只接受时间戳严格匹配的 corrected 骨架与状态帧。默认
`robot_arm_segments` 模式采用天机机械臂骨段长度，以 `0.95` 的臂展比例重建目标。
每个有效源帧生成一个 656 字节的原子双臂 TJVR v4 UDP 包，包含掌心目标、完整修正
上肢位置与旋转，以及带有效标志的左右臂冗余方向；不进行重采样，也不使用 clutch。

可在任意tracking ROS 终端检查输入和 bridge 诊断：

```bash
ros2 topic hz /pico/smpl_palm_corrected_ik
ros2 topic echo /pico/smpl_palm_corrected/status --once
ros2 topic echo /pico/tianji_mujoco_teleop/status --once
```

启动脚本默认发送到 `127.0.0.1:15000`。该 bridge 只服务天机 MuJoCo 联动验证，
不会向真实机械臂发送命令。

`m0` 窗口默认打开 PICO 骨架 MuJoCo Viewer，显示
`/pico/smpl_palm_corrected_ik` 并叠加原始 SMPL。该脚本仍不会启动接收端
DLS/Spark 机械臂 MuJoCo Viewer；需要检查机械臂侧效果时再独立运行：

```bash
cd ../control
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_pico_teleop.yaml \
  --model models/marvin_m6_wuji2.xml \
  --algorithm spark_upper_qpoases \
  --pico-teleop \
  --pico-bind 127.0.0.1 \
  --pico-port 15000
```

`--algorithm spark_upper_qpoases` 选择 Spark 风格骨架缩放与两阶段 qpOASES 位置
IK；`--pico-teleop` 启用实时 UDP 输入。如果 overlay 仍显示 `pico_enabled=0`，按一次
`P`。

若需要与其他机器通信，请在每个终端运行脚本前设置 `ROS_LOCALHOST_ONLY=0`，并保证
所有机器的 `ROS_DOMAIN_ID=120` 一致。

## 软件包（位于 `src/`）

| 软件包 | 语言 | 用途 |
|---------|------|------|
| `pico_bridge` | C++ (rclcpp) | PICO TCP 桥接、IMU900 脚部融合和 MuJoCo 可视化。 |
| `imu_ros2` | C++ (rclcpp) | 支持双脚配置的 IM900/IM948 串口驱动。 |
| `fisheye_camera` | C++ (rclcpp) | 将 V4L2 USB 鱼眼相机数据发布为 ROS 话题。 |
| `data_collector` | Python (rclpy) | 将订阅的 ROS 话题保存为 HDF5 和各数据流 MP4。 |
| `odin_ros_driver` | C++ (rclcpp) | 支持重定位地图的 Odin1 驱动。 |
| `odin_ros_driver_rev1` | C++ (rclcpp) | 提供 SLAM 点云和里程计的 Odin Lite / SDK2 驱动。 |
| `pico_odin` | C++ (rclcpp) | 选择 Odin 驱动，并标定、修正 Odin Lite 骨盆位姿。 |

## 前置条件

- 根目录 `.venv` 与 ROS 2 Humble SDK，见[安装说明](docs/install-guide.md)
- Linux x86_64
- `PATH` 中可用的 `adb`
- `PATH` 中可用的 `tmux`（用于一键启动天机 PICO 遥操链路）
- 运行 `pico_wholebody_stream.apk` 的 PICO 4U
- 两个安装在鞋上的 IMU900（脚部融合可选依赖）

## 构建

```bash
# From the monorepo root after installing its .venv and ROS SDK:
bash tracking/scripts/build.sh --all
cd tracking
source scripts/environment.sh
```

也可以在 `tracking/` 下重新构建与测试：

```bash
bash scripts/build.sh --all
colcon test --base-paths src --event-handlers console_direct+
```

包括 ARM64/RK3588、ADB 和冒烟测试在内的完整安装说明，参见
[`docs/install-guide.md`](docs/install-guide.md)。

## 底层命令与可选服务

上面的快速脚本是 PICO 手臂默认流程。只有单独启动可选服务或调试 bridge 时，才使用
以下底层命令：

```bash
# 终端 1 — PICO bridge
source scripts/environment.sh
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py

# 终端 2 — 鱼眼相机（可选）
source scripts/environment.sh
ros2 launch fisheye_camera fisheye_camera.launch.py

# 终端 3 — 数据采集程序
source scripts/environment.sh
ros2 run data_collector data_collector_node \
  --config src/data_collector/config/collect_config.yaml
# 按 's' 开始，按 'd' 停止并保存，按 'q' 退出
```

## 配合 Odin 里程计运行

Odin 启动辅助程序会将两种实现发布到录制话题 `/raw/odom/odin` 和
`/raw/odom/odin_highfreq`。启用本地位姿时，Odin1 还会提供
`/raw/odom/odin_local`。

```bash
# Odin1，可选加载重定位地图
ros2 launch pico_odin odin_select.launch.py \
  odin_impl:=original \
  odin_map:=/path/to/map_merged.bin

# Odin Lite / SDK2
ros2 launch pico_odin odin_select.launch.py odin_impl:=lite
```

`pico_odin` 的 Odin Lite 路径默认使用纯里程计配置。骨盆标定和运行时修正只
依赖 Odin 里程计，因此不会启动原始/SLAM 点云、相机和 Odin IMU 数据流，从而
避免不必要的 UDP 带宽与内存开销。确实需要点云时，请显式使用 Odin 驱动自身
的点云配置。

Lite 驱动进程使用 Fast DDS UDPv4 传输，避免进程异常终止后损坏的共享内存段。
如果其他 ROS 2 CLI/节点出现 `ParticipantEntitiesInfo`、`Fast CDR` 或 `Bad alloc`，
可仅清理 Fast DDS 僵尸共享内存段，然后重新启动受影响的命令：

```bash
fastdds shm clean
```

使用类似 `catkin_exoskeleton_ws` 的 tmux 启动方式：

```bash
./scripts/start_pico_odin_collect.sh --odin-impl original --odin-map /path/to/map_merged.bin
./scripts/start_pico_odin_collect.sh --odin-lite
tmux attach -t pico_odin_collect
```

`collect_config.yaml` 将 `/raw/odom/odin` 和 `/raw/odom/odin_highfreq` 视为
必需数据集。`/raw/odom/odin_local` 为可选数据集，因为仅 Odin1 在启用地图重定位或
本地位姿时发布该话题。

### Odin Lite 到骨盆的一次性安装标定

安装或移动 Odin Lite 后，需要标定刚性变换 `T_pelvis_odin`。标定结果默认保存到
`~/.config/pico_tracker/odin_pelvis_extrinsics.yaml`，后续运行会重复使用。分别在三个
终端启动原始驱动，并确保所有终端使用相同的 ROS domain：

```bash
# 所有终端（将 120 替换为实际使用的 ROS 2 domain）
export ROS_DOMAIN_ID=120
export ROS_LOCALHOST_ONLY=0

# 终端 1 — 原始 Odin Lite（标定期间关闭运行时修正）
ros2 launch pico_odin odin_select.launch.py \
  odin_impl:=lite \
  enable_pelvis_runtime:=false

# 终端 2 — PICO bridge
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py

# 终端 3 — 独立标定程序
ros2 run pico_odin odin_pelvis_calibrator
```

标定程序提示 ready 后，按 PICO 手臂控制器的 `A` 键启动（电脑键盘小写 `a` 仍可作为
备用触发）。先站直静止 2 秒，前后俯仰约 25--35 度，再左右转向约 30--60 度，最后回到直立状态并至少静止 3 秒。只有通过质量检查的结果才会
原子保存；标定失败不会覆盖上一次有效文件。两段静止期间，PICO 骨盆与 Odin 必须
同时保持静止。标定文件会保存空间安装变换、动作相关性、质量指标和输入话题；旧版
schema 1 和 2 文件仍可读取并在内存中升级，其中旧的时间偏移字段会被明确忽略。

对于安装在骨盆后方的 Odin Lite，重新标定时建议增加物理约束，拒绝明显不合理的安装矩阵：

```bash
ros2 run pico_odin odin_pelvis_calibrator --ros-args \
  -p mount_abs_z_max_m:=0.25 \
  -p max_condition_number:=30.0
```

只有在实际测量确认安装偏移更大时才应放宽这些参数。启用运行时修正前先检查结果：

```bash
cat ~/.config/pico_tracker/odin_pelvis_extrinsics.yaml
```

正常使用时，默认的 `auto` 会在选择 `odin_impl:=lite` 时自动启动骨盆修正：

```bash
ros2 launch pico_odin odin_select.launch.py odin_impl:=lite
```

PICO 与 Odin 同步静止采集 30 帧后，会发布以下修正结果：

- `/calibrated/odom/pelvis`
- `/calibrated/odom/pelvis_highfreq`
- `/pico/smpl_odin`
- 启用脚部融合时的 `/pico/smpl_fused_odin`

运行时初始骨盆位置和高度来自 PICO，后续叠加 Odin 的位姿变化；24 个关节会作为
整体刚性变换，所有原始话题保持不变。PICO 与 Odin 的设备时间戳可以属于完全不同的
时间基准：C++ 节点使用本机单调接收时间配对，在高频 Odin 数据间插值，并根据本次
运行的动作估计传输接收延迟。该时间状态每次启动都会重建，不会从安装标定文件读取。
PICO world reset、任一路 Odin 数据异常、时间戳回退、坐标帧变化、接收断流或位姿
原点跳变都会清除本次运行对齐并暂停修正输出，重新静止对齐后自动恢复；这些操作不会
删除一次性安装标定。可用 `pelvis_extrinsics_file:=/path/to/file.yaml` 覆盖默认文件。运行时输入、
输出话题以及对齐和重启阈值都可作为 launch 参数配置，可运行
`ros2 launch pico_odin odin_select.launch.py --show-args` 查看。

启动运行时修正后，保持站直静止并按一次 PICO `A` 键，建立本次运行的对齐。进行原地
旋转测试时，观察 `/calibrated/odom/pelvis`，骨盆的 X/Y 位置应基本保持不变而只有航向角
变化。如果出现明显圆周漂移，说明安装矩阵不可靠，应重新标定后再使用修正骨架。

当前工程支持以下两种运行配置：

| 配置 | 主要输出 |
|---|---|
| PICO + Odin Lite | `/pico/smpl_odin` |
| PICO + Odin Lite + 双 IMU900 | `/pico/smpl_fused_odin` |

使用双 IMU900 时，必须先使用 `/pico/smpl` 完成 Odin 安装矩阵标定，再启动双 IMU900
融合和 Odin 运行时。运行时订阅 `/pico/smpl_fused`，最终发布
`/pico/smpl_fused_odin`。两个串口不能同时被其他 IMU 节点占用。

## PICO 全身数据

全身协议使用帧类型 `0x20` 至 `0x37` 发送 24 个 `BodyTrackerRole` 关节。
`pico_bridge` 将时间戳相同的关节组合成未经几何修改、以头部为原点的
`/pico/smpl_raw`。按下 PICO 右手柄 A 键后，地面归零节点等待 30 帧稳定的双脚
数据，锁定初始脚底高度，再以 `pico_ground` 坐标发布 `/pico/smpl`
（`geometry_msgs/msg/PoseArray`）。标准话题中地面为 `Z=0`、脚底触地，站立时
骨盆和头部高度为正。按 A 前或地面尚未稳定锁定时，`/pico/smpl` 会保持静默。

采集程序将数组保存为 `/states/smpl/pose`，形状为 `(frames, 24, 7)`，布局为
`[x, y, z, qx, qy, qz, qw]`。标准关节名称保存为 HDF5 组属性。

## PICO + 双 IMU900 脚部融合

融合节点保留 PICO 的非脚部关节位置。每只脚使用 IMU900 姿态，并根据 PICO 脚踝
位置和标定得到的中立偏移重建脚部位置。结果发布到 `/pico/smpl_fused`，作为
标准输入的地面坐标 `/pico/smpl` 不会被脚部融合修改。

默认硬件分配为左脚 `/dev/ttyUSB0`、右脚 `/dev/ttyUSB1`。从仓库根目录先启动
PICO bridge，再在另一个终端启动两个 IMU 驱动和融合节点：

```bash
# 终端 1 — PICO bridge
source scripts/environment.sh
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py

# 终端 2 — 双 IMU900 驱动和融合节点
source scripts/environment.sh
ros2 launch pico_bridge start_pico_foot_fusion.launch.py \
  left_port:=/dev/ttyUSB0 \
  right_port:=/dev/ttyUSB1
```

检查两个 IMU 和 PICO 骨架数据。body tracking 启动后应立即看到
`/pico/smpl_raw`；需要按 A 并保持站立静止后才会看到 `/pico/smpl`：

```bash
ros2 topic echo /imu/left_feet/ready --once
ros2 topic echo /imu/right_feet/ready --once
ros2 topic hz /imu/left_feet
ros2 topic hz /imu/right_feet
ros2 topic hz /pico/smpl_raw
ros2 topic hz /pico/smpl
ros2 topic hz /pico/smpl_fused
```

保持站立、双脚平放并静止，然后按下 PICO 右手柄 A 键。自动事务会依次复位左右
IMU900 的 Z 轴，等待 1 秒，丢弃旧样本，再采集 60 帧新的中立姿态样本。

节点输出 `PICO foot IMU calibration complete` 日志后会恢复
`/pico/smpl_fused`。硬件复位或 IMU 重连后需要重新标定。

下面的手动备用方式会采集相同的软件标定样本，但**不会**发送 IMU900 硬件 Z 轴
复位命令：

```bash
ros2 service call /pico_foot_imu_fusion/calibrate std_srvs/srv/Trigger "{}"
```

### PICO + Odin Lite + 双 IMU900

确认 Odin 安装矩阵有效后，保持 PICO bridge 运行，在另外两个终端分别执行：

```bash
# 终端 1 — 双 IMU900 融合
ros2 launch pico_bridge start_pico_foot_fusion.launch.py \
  left_port:=/dev/ttyUSB0 \
  right_port:=/dev/ttyUSB1 \
  imu_reset_settle_sec:=1.0 \
  calibration_samples:=60

# 终端 2 — Odin Lite 运行时修正
ros2 launch pico_odin odin_select.launch.py \
  odin_impl:=lite \
  enable_pelvis_runtime:=true
```

保持站直静止并按一次 PICO `A` 键。该操作会完成 PICO 地面对齐、双脚 IMU 复位与软件
标定，以及本次运行的 Odin 对齐。先确认最终话题发布：

```bash
ros2 topic hz /pico/smpl_fused_odin
```

可视化最终融合骨架，并叠加只经过 Odin 修正的骨架：

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_fused_odin \
  --show-raw \
  --raw-topic /pico/smpl_odin \
  --scale 1.0 \
  --rate 60 \
  --timeout 2.0
```

仓库内置的 IM900/IM948 驱动使用两个独立的 115200 波特率串口，以约 110 Hz 发布，
并提供 ready 状态话题和自动重连。经过实机验证的默认脚部安装四元数为
`[0, 0, 0, 1]`。

安装、标定和故障排查详情参见
[`docs/PICO_FOOT_IMU_FUSION.md`](docs/PICO_FOOT_IMU_FUSION.md)。

## MuJoCo 几何骨架可视化

可选的 MuJoCo 可视化程序会绘制 24 个关节球、23 根胶囊骨骼和带方向的脚板。
仅显示经过地面归零的 PICO 标准骨架：

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0
```

在同一坐标系中比较 IMU 修正后的脚部姿态和 PICO 原始脚部姿态：

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_fused \
  --show-raw \
  --raw-topic /pico/smpl \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0

ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_fused \
  --show-raw \
  --raw-topic /pico/smpl_raw \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0

```

如果还需要在骨架旁同步显示 PICO 头显以及左右手柄的原始位姿，加入
`--show-controllers`。这是独立于 `--show-raw` 的显式 overlay；显示原始骨架不会再自动
添加三个额外末端点。
如果两套骨架使用 `--raw-offset 0 0 0` 完全重合，原始骨架可能被前景覆盖，建议比较时使用一个小的
显示偏移：

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_palm_corrected \
  --show-raw --raw-topic /pico/smpl_raw \
  --palm-topic /pico/palm_left \
  --right-palm-topic /pico/palm_right \
  --show-arm-axes --show-controllers \
  --raw-offset 0.30 0 0
```

`--show-arm-axes` 只作用于主（TCP/修正后）骨架。原始 PICO overlay 默认不绘制上肢局部
XYZ 轴，只有确实需要查看原始诊断时才显式增加 `--show-raw-arm-axes`。

三个位姿来源分别是 `/pico/pose/head`、`/pico/pose/left_hand` 和
`/pico/pose/right_hand`；可以通过 `--head-topic`、`--left-controller-topic` 和
`--right-controller-topic` 覆盖默认话题。它们与 SMPL/掌心消息使用相同的 PICO 世界坐标转换，
只做显示，不参与掌心校正或骨架求解。

如果要专门用卷尺验证 PICO 双手末端定位，可使用上半身模式。该模式从骨盆开始绘制脊柱、
头部以及左右肘部—腕部—手部链路，隐藏双腿，保留地面和世界坐标轴，并在窗口中显示
`pico_ground` 坐标系下的左右手末端坐标（单位：米）：

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl \
  --hands-only \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0
```

上半身模式会自动只在左右肩、肘、腕和手关键点显示局部 XYZ 坐标轴，颜色分别为
红色 X、绿色 Y、蓝色 Z；脊柱、骨盆和头部不添加局部坐标轴，固定世界坐标轴仍保留。
在完整骨架模式下，可通过 `--show-arm-axes` 开启同样的上肢坐标轴而不隐藏双腿。

评估 Odin 修正后的手部位姿时，可将 `--topic` 替换为 `/pico/smpl_odin` 或
`/pico/smpl_fused_odin`。

### 掌心 TCP 标定与掌心约束骨架

#### 统一交互式标定入口（推荐）

在 `tianji_teleop` 工作区内为不同人员适配时，推荐显式指定人员：

```bash
bash scripts/calibrate_pico_arm.sh left all --user zjx &&
bash scripts/calibrate_pico_arm.sh right all --user zjx
```

有效候选自动保存到工作区 `profiles/zjx/pico/.draft/`，左右完整依赖链通过后自动发布新版本
并更新 `profiles/zjx/profile.yaml`。失败保留草稿和原活动版本；原始采集固定保存在
`profiles/zjx/recordings/`，不会因发布而改变路径。`status --user zjx` 只读检查已发布版本。
`--user` 可用于下文的单项或交互式入口；不传时保留下述旧全局目录行为。
完整人员档案说明见[工作区 README](../README.md#人员档案与标定版本)。


先启动 PICO bridge，再在当前仓库进入 root .venv / ROS SDK environment。脚本默认使用 domain 120；只在本机
通信时保持 `ROS_LOCALHOST_ONLY=1`。无参数会进入菜单：可以任意选择左/右侧的 TCP、掌心到
手腕或上臂/前臂骨长标定，也可以选择新操作人员的严格顺序 `TCP → 掌心到手腕 → 骨长`：

```bash
cd /path/to/pico_project
source scripts/environment.sh
export ROS_DOMAIN_ID=120 EXO_REQUESTED_ROS_DOMAIN_ID=120 ROS_LOCALHOST_ONLY=1
./scripts/calibrate_pico_arm.sh
```

也可以直接执行单项或完整顺序（依赖不满足时脚本会 fail-closed，不会覆盖旧 artifact）：

```bash
./scripts/calibrate_pico_arm.sh left tcp
./scripts/calibrate_pico_arm.sh left wrist
./scripts/calibrate_pico_arm.sh left geometry
./scripts/calibrate_pico_arm.sh right all
./scripts/calibrate_pico_arm.sh status
```

左右两侧共用算法实现，但 artifact 和骨长必须分别采集、分别保存，不能镜像。`wrist` 会
临时启动同侧 TCP publisher；`geometry` 会调用现有的几何标定器，并且只把通过 Gate 的
`pico_*_arm_geometry_quick_v3` 候选原子激活到所选目录（未指定人员时为 `~/.config/pico_tracker/`）。标定器的交互动作
和退出方式仍以各阶段提示为准；`all` 会在每一步成功后才进入下一步。

左右 TCP 标定只在首次安装或手柄—手套固定关系变化后执行。标定左手手套掌心位置时，
让掌心中心保持接触同一个固定点，每次只改变手柄姿态，然后在终端按键盘空格 4 次；
这 4 次采样只求解手柄到掌心的平移。位置检查通过后，双臂向正前方水平伸直、左右掌心
相对，摆好后再按第 5 次空格启动所选侧姿态采集，并保持当前姿势约 1～2 秒。系统使用
至少 120 个时间配对样本进行 SO(3) 平均；掌心俯仰/横滚对齐重力水平，yaw 跟随头显
朝向。两个阶段都通过后才会保存完整 TCP 标定文件：

```bash
ros2 run pico_bridge pico_palm_tcp_calibrator
```

右手使用同样的流程：

```bash
ros2 run pico_bridge pico_palm_tcp_calibrator --side right
```

标定结果文件如下：

```text
左手：~/.config/pico_tracker/pico_left_palm_tcp.yaml
右手：~/.config/pico_tracker/pico_right_palm_tcp.yaml
```

如果 PICO 重新开机后某一侧掌心朝向存在明显固定偏差，不需要重做 TCP 平移、腕部或
骨长标定。进入本项目的 `source scripts/environment.sh`，只重标所选侧 TCP 姿态：

```bash
./scripts/calibrate_pico_palm_orientation.sh left
# 或
./scripts/calibrate_pico_palm_orientation.sh right
```

操作时自然站立、头部正视前方，双臂向正前方水平伸直、左右掌心相对；摆好后按空格并
保持约 2 秒静止。运行中的同侧 TCP publisher 也提供
`/pico/palm_orientation/<side>/calibrate`（`std_srvs/Trigger`），供之后外骨骼联合标定
直接调用。成功后无需重启 publisher 即刻生效，只更新所选 TCP 文件的
`quaternion_xyzw`；TCP 平移、掌心到手腕距离以及上臂/前臂骨长继续有效。失败时磁盘和
运行时状态均保持不变。

日常运行时不需要保持两个交互式 calibrator 运行。PICO bridge 已发布
`/pico/smpl_raw` 和左右原始手柄位姿后，只需一条 launch 指令即可同时启动：

- 左掌心只读 TCP publisher；
- 右掌心只读 TCP publisher；
- 掌心约束骨架 filter。

### 左右臂个体骨长快速标定

若默认 PICO SMPL 上臂/前臂比例导致掌心约束骨架变形，应对左右两侧分别采集。
保持 PICO 驱动在线，并在 root .venv / ROS SDK environment 中直接选择对应侧的骨长标定：

```bash
./scripts/calibrate_pico_arm.sh left geometry
./scripts/calibrate_pico_arm.sh right geometry
```

左右共用算法，但必须分别采集、分别验证、分别保存，禁止镜像或复用另一侧测量骨长。

预检通过后自动开始，不需要再操作键盘。终端依次提示：左臂自然下垂、两次左臂
伸直前举并静止、上臂下垂贴身且肘部弯至约 90 度并静止、再次伸直验证、恢复
自然下垂。每次应在倒计时结束前摆好；程序自动选择每段最长的静止后缀。
自然下垂、直臂前举和约 90 度屈肘三种姿态会消去未知肩点：前举腕点减去屈肘
腕点辨识上臂，屈肘腕点减去下垂腕点辨识前臂。三次前举中自动选择位置最一致
的两次并记录被丢弃的离群阶段；PICO 原始肩/肘/腕的约 90 度误差只作为诊断，
不提供骨长真值，也不会否决已通过独立掌心几何 Gate 的结果。输出目录为
`recordings/pico_left_arm_geometry_<时间>/`，其中包含：

```text
capture.npz
stage_ranges.yaml
pico_left_arm_geometry_candidate.yaml
gate_report.json
```

候选 artifact 为 `pico_left_arm_geometry_quick_v3`。`capture.npz` 同时保存原始
掌心位置、掌心四元数、PICO 原始腕点和沿掌心局部 +X 重建的腕点，可在失败后
完整离线复算；旧的 v1/v2 候选不会被运行时加载。

只有 `gate_report.json` 中 `valid=true` 且 candidate 中
`candidate_status=accepted` 的结果才会原子激活到 `~/.config/pico_tracker/`。
失败数据保留在 `recordings/` 供复查，不会覆盖当前有效配置。检查当前状态并启动：

```bash
./scripts/calibrate_pico_arm.sh status
./scripts/start_pico_m0.sh --viewer --record --duration 120
```

双侧状态都应为 `geometry_source=quick_arm_artifact`，并分别携带正数
`geometry_revision`。某侧尚未标定时允许该侧为 `raw_smpl_baseline`、revision 0，
但此时不能宣称双侧 M0 已收口。

运行器会打印 NPZ 路径。使用同一干净环境生成 fail-closed Gate 报告：

```bash
ros2 run pico_bridge pico_m0_comparison_report report \
  /绝对路径/pico_m0_capture.npz \
  --output /绝对路径/pico_m0_gate.json
```

```bash
ros2 launch pico_bridge start_pico_palm_skeleton_filter.launch.py \
  left_tcp_artifact:="$HOME/.config/pico_tracker/pico_left_palm_tcp.yaml" \
  right_tcp_artifact:="$HOME/.config/pico_tracker/pico_right_palm_tcp.yaml" \
  left_wrist_pivot_artifact:="$HOME/.config/pico_tracker/pico_left_wrist_pivot.yaml" \
  right_wrist_pivot_artifact:="$HOME/.config/pico_tracker/pico_right_wrist_pivot.yaml" \
  require_wrist_pivot_artifact:=true
```

该 launch 同时发布：

- `/pico/smpl_palm_corrected`：保持原有 PICO 关节坐标语义；
- `/pico/smpl_palm_corrected_ik`：所有关节位置完全相同；左肩局部 X 右乘
  `+pi/2`、右肩局部 X 右乘 `-pi/2`。左右肘部局部 X 严格沿肘到腕的前臂
  方向，局部 Y 为肩—肘—腕平面的肘屈伸轴，局部 Z 补成右手坐标系。左右肘
  局部 +Y 均选择朝人体左侧的符号；手臂接近伸直、屈伸平面不可辨识时，使用
  人体左方向在前臂法平面上的投影。当人体左方向无法可靠判定可观测屈伸轴的
  正负时，沿用上一有效帧的肘部 Y 符号以保持时间连续。退化 IK 帧只会被跳过，
  并通过 `ik_frame_valid`/`ik_frame_failure_reason` 报告，绝不会中断原修正骨架流。

检查左右掌心和两种修正骨架：

```bash
ros2 topic hz /pico/palm_left
ros2 topic hz /pico/palm_right
ros2 topic hz /pico/smpl_palm_corrected
ros2 topic hz /pico/smpl_palm_corrected_ik
ros2 topic echo /pico/smpl_palm_corrected/status --once
```

`./scripts/start_pico_m0.sh --viewer` 默认显示 IK-frame topic，并保留原始 SMPL
叠加对比；它只改变肩部和肘部红绿蓝坐标轴，不改变骨架几何位置或掌心目标。

运行时 publisher 以只读方式加载 artifact，将标定得到的 `T_controller_palm` 对原始手柄
位姿严格应用一次；它不会写回标定文件，也不需要按键。若确实要手动运行掌心 publisher
或 calibrator，必须给 launch 增加 `start_palm_publishers:=false`，避免同一掌心 topic 出现
重复发布者。

viewer 仍可使用保存的 `--left-tcp-artifact`/`--right-tcp-artifact` 和原始手柄话题回算掌心。
配置
`--left-wrist-pivot-artifact`/`--right-wrist-pivot-artifact` 后，viewer 始终从同一帧 TCP
掌心沿掌心局部 `-X` 回算腕部。兼容旧 artifact 时仅使用 `wrist_to_palm_m` 的模长，
不会使用其旧 XYZ 方向；腕部姿态与掌心严格一致。即使 `/pico/wrist_left`、`/pico/wrist_right`
同时发布，也不会被不同时间基准或标定版本覆盖。没有 pivot artifact 时才使用实时 wrist
topic，再回退到修正 PoseArray 的 LEFT_WRIST/RIGHT_WRIST。窗口状态栏会显示
`wrist_pivot`、`topic` 或 `primary_posearray` 数据来源，便于定位掌心球未显示的问题。

完整数据流、状态字段和故障排查见
[`docs/PICO_PALM_SKELETON_FILTER.md`](docs/PICO_PALM_SKELETON_FILTER.md)。

融合骨架使用蓝色关节、灰色骨骼和橙色脚板。未融合的 PICO 标准骨架更细，显示为
半透明绿色。两路数据使用相同的 `pico_ground` 坐标；叠加数据缺失或超时不会阻塞
融合骨架。`/pico/smpl_raw` 是独立的 APK/头部原点诊断流，未应用地面偏移时不能
直接与上述骨架叠加。

可视化程序使用 `BEST_EFFORT` QoS 订阅。主数据流超时后会以灰色保留显示，并在
终端输出限频警告。MuJoCo 由根目录 `.venv` 安装。

## 无 PICO 实机测试

```bash
python3 src/pico_bridge/scripts/mock_pico_server.py --fps 30 --duration 60
```

## 工作空间布局

参见 [`docs/workspace-layout.md`](docs/workspace-layout.md)。

## 设计与历史

- [`docs/PICO_Streaming_Guide.md`](docs/PICO_Streaming_Guide.md) — PICO 协议参考。
- [`docs/superpowers/specs/`](docs/superpowers/specs/) — 设计文档。
- [`docs/superpowers/plans/`](docs/superpowers/plans/) — 实施计划。
- [`reference/receiver.py`](reference/receiver.py) — 原始 Python 接收程序。
