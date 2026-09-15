# Odin ROS Driver

**中文**：基于 Odin SDK 的 ROS 驱动程序，支持 ROS1 和 ROS2 多版本。

**English**: ROS driver built on top of the Odin SDK, supporting both ROS1 and ROS2.

## 功能特性 / Features

- **多平台支持 / Multi-platform**: ROS1 Noetic、ROS2 Humble/Iron/Jazzy
- **热插拔检测 / Hot-plug**: 自动发现和连接设备，支持设备断开重连 / Auto-discovery and reconnection of devices
- **心跳监控 / Heartbeat**: 实时监测设备连接状态 / Real-time device connection monitoring
- **多数据流 / Multi-stream**: 点云、图像、IMU、里程计同步输出 / Synchronized point cloud, image, IMU and odometry output
- **后处理 / Post-processing**: 图像去畸变、点云着色、立体校正、点云转深度图 / Image undistortion, cloud colorization, stereo rectification, point-cloud → depth

## 功能模块 / Nodes

| 节点 / Node | 说明 / Description |
| --- | --- |
| `odin_ros_driver_node` | 主驱动节点：连接设备、发布原始数据、提供标定服务 / Main driver: device connection, raw data publishing, calibration service |
| `post_process_node` | 后处理节点：图像去畸变、点云着色、立体校正 / Post-processing: undistortion, cloud colorization, stereo rectification |
| `pointcloud_reprojector_node` | 点云重投影节点：稀疏点云重投影到 `camera0` 输出深度图 + 叠加图 / Reprojects sparse point cloud onto `camera0` to produce a depth + overlay image |
| `pointcloud_to_depth_node` | 点云转深度图节点（多设备）：投影 + 膨胀 + Sobel 边缘抑制 + 最近邻上采样 / Multi-device point-cloud → depth: projection + dilation + Sobel edge suppression + NN upsample |

## 系统要求 / System Requirements

| Ubuntu 版本 / Version | ROS 版本 / ROS Version |
| --- | --- |
| 20.04 | ROS1 Noetic, ROS2 Foxy |
| 22.04 | ROS2 Humble, ROS2 Iron |
| 24.04 | ROS2 Jazzy |

## 依赖安装 / Dependency Installation

### 编译工具（必须）/ Build tools (required)

```bash
sudo apt install build-essential cmake git
```

### ROS1 (Noetic)

```bash
# ROS1 基础 / ROS1 base
sudo apt install ros-noetic-desktop-full

# ROS1 功能包依赖 / ROS1 package dependencies
sudo apt install ros-noetic-ddynamic-reconfigure
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
sudo apt install ros-noetic-pcl-ros ros-noetic-stereo-msgs

# 系统库依赖 / System library dependencies
sudo apt install libyaml-cpp-dev libopencv-dev libeigen3-dev libpcl-dev libssl-dev
```

### ROS2 (Humble/Iron/Jazzy)

```bash
# ROS2 基础（选择对应版本）/ ROS2 base (pick one for your Ubuntu version)
sudo apt install ros-humble-desktop  # Ubuntu 22.04
# 或 / or
sudo apt install ros-iron-desktop    # Ubuntu 22.04
# 或 / or
sudo apt install ros-jazzy-desktop   # Ubuntu 24.04

# ROS2 功能包依赖（将 humble 替换为对应版本：humble/iron/jazzy）
# ROS2 package dependencies (replace 'humble' with your distro: humble/iron/jazzy)
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
sudo apt install ros-humble-pcl-ros ros-humble-stereo-msgs

# 系统库依赖 / System library dependencies
sudo apt install libyaml-cpp-dev libopencv-dev libeigen3-dev libpcl-dev libssl-dev
```



## 编译 / Build

**中文**：SDK 会在编译时自动构建，无需单独安装。

**English**: The SDK is built automatically as part of the package build — no separate install step is required.

### ROS1

```bash
source /opt/ros/noetic/setup.bash
cd your_ros_workspace
./src/odin_ros_driver/script/build_ros1.sh
```

### ROS2

```bash
source /opt/ros/humble/setup.bash
cd your_ros_workspace
./src/odin_ros_driver/script/build_ros2.sh
```

## 运行 / Run

### ROS1

```bash
source devel/setup.bash
# 默认配置（自动发现设备，使用设备默认分辨率）
# Defaults (auto-discover device, use device-default resolution)
roslaunch odin_ros_driver_rev1 driver.launch

# 指定图像分辨率和帧率 / Specify image resolution and frame rate
roslaunch odin_ros_driver_rev1 driver.launch image_width:=640 image_height:=544 image_fps:=30

# 不启动 RViz / Do not launch RViz
roslaunch odin_ros_driver_rev1 driver.launch start_rviz:=false
```

### ROS2

```bash
source install/setup.bash
# 默认配置（自动发现设备，使用设备默认分辨率）
# Defaults (auto-discover device, use device-default resolution)
ros2 launch odin_ros_driver_rev1 driver.launch.py

# 指定图像分辨率和帧率 / Specify image resolution and frame rate
ros2 launch odin_ros_driver_rev1 driver.launch.py image_width:=640 image_height:=544 image_fps:=30

# 不启动 RViz / Do not launch RViz
ros2 launch odin_ros_driver_rev1 driver.launch.py start_rviz:=false
```

### Launch 参数说明 / Launch Parameters

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
| --- | --- | --- |
| `start_rviz` | `true` | 是否启动 RViz / Whether to start RViz |
| `image_width` | `0` | 图像宽度，0 = 使用设备默认 / Image width, 0 = device default |
| `image_height` | `0` | 图像高度，0 = 使用设备默认 / Image height, 0 = device default |
| `image_fps` | `0` | 图像帧率，0 = 使用设备默认 / Image frame rate, 0 = device default |
| `image_format` | `mjpeg` | 图像格式 / Image format: mjpeg/yuyv/nv12/nv21/rgb24 |

## 发布话题 / Published Topics

### odin_ros_driver_node（主驱动节点 / main driver node）

| 话题 / Topic | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `odin/cloud_raw` | `sensor_msgs/PointCloud2` | 原始点云（256×192）/ Raw point cloud (256×192) |
| `odin/cloud_slam` | `sensor_msgs/PointCloud2` | SLAM 彩色点云 / SLAM colored point cloud |
| `odin/image/compressed` | `sensor_msgs/CompressedImage` | 左相机 JPEG 压缩图像 / Left camera JPEG compressed image |
| `odin/image_raw` | `sensor_msgs/Image` | 左相机解码后 RGB 图像 / Left camera decoded RGB image |
| `odin/image2/compressed` | `sensor_msgs/CompressedImage` | 右相机 JPEG 压缩图像 / Right camera JPEG compressed image |
| `odin/image2_raw` | `sensor_msgs/Image` | 右相机解码后 RGB 图像 / Right camera decoded RGB image |
| `odin/imu` | `sensor_msgs/Imu` | IMU 数据（加速度、角速度）/ IMU data (acceleration, angular velocity) |
| `odin/odometry` | `nav_msgs/Odometry` | 里程计数据 / Odometry data |
| `odin/gray_image` | `sensor_msgs/Image` | 灰度图像（mono8）/ Gray image (mono8) |

### post_process_node（后处理节点 / post-processing node）

| 话题 / Topic | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `odin/image_undistort` | `sensor_msgs/Image` | 左相机去畸变图像 / Left camera undistorted image |
| `odin/image2_undistort` | `sensor_msgs/Image` | 右相机去畸变图像 / Right camera undistorted image |
| `odin/cloud_color` | `sensor_msgs/PointCloud2` | 着色点云（点云+图像融合）/ Colored point cloud (cloud + image fusion) |
| `odin/left_rect` | `sensor_msgs/Image` | 立体校正后左图像 / Stereo-rectified left image |
| `odin/right_rect` | `sensor_msgs/Image` | 立体校正后右图像 / Stereo-rectified right image |
| `odin/left_camera_info` | `sensor_msgs/CameraInfo` | 左相机校正后内参 / Rectified left camera intrinsics |
| `odin/right_camera_info` | `sensor_msgs/CameraInfo` | 右相机校正后内参 / Rectified right camera intrinsics |
| `odin/depth_pointcloud` | `sensor_msgs/PointCloud2` | 视差转点云输出 / Disparity → point cloud output |

### pointcloud_reprojector_node（点云重投影节点 / point-cloud reprojector）

**中文**：把稀疏 dToF 点云重投影到 `camera0` 图像平面，输出稠密的深度图和叠加可视化图，用于标定校验与 3DGS 深度监督。支持两种模式：`raw`（`cloud/raw` + 静态外参）和 `slam`（`cloud/slam` + 实时 odom，世界坐标 → 相机坐标）。源码位于 `src/pointcloud_reprojector/`。

**English**: Reprojects the sparse dToF point cloud onto `camera0` to produce a dense depth image plus a visualization overlay. Useful for calibration verification and 3DGS depth supervision. Two modes: `raw` (`cloud/raw` + static extrinsic) and `slam` (`cloud/slam` + live odometry, world → camera). Source under `src/pointcloud_reprojector/`.

| 话题 / Topic | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `{prefix}reproject/depth` | `sensor_msgs/Image` (32FC1) | 重投影深度图（米） / reprojected depth map (meters) |
| `{prefix}reproject/overlay` | `sensor_msgs/Image` (bgr8) | 点云叠加在相机图上 / cloud overlaid on the camera image |

主要参数 / Key parameters: `mode` (`raw`/`slam`), `topic_prefix`, `min_depth`, `max_depth`, `point_size`, `cloud_topic_suffix`, `image_topic_suffix`, `odom_topic_suffix`。

### pointcloud_to_depth_node（点云转深度图节点 / point-cloud → depth）

**中文**：基于纯 C 风格的投影核心 (`pointcloud_to_depth_node.hpp`)，把同步好的 `cloud/raw` 与 `camera0/undistort` 投影到 `1/scale` 分辨率的低分画布上，再经膨胀、Sobel 边缘抑制、最近邻上采样得到与相机同分辨率的稠密深度图。**支持多设备**：每个 SN 一个 `DeviceWorker`，通过 `device_online` / `device_offline` 动态生灭，`resolution_change` 按 SN 分发。源码位于 `src/pointcloud_to_depth/`，详见同目录下 `README.md`。

**English**: A library-free point-cloud → depth projection node. Synchronizes `cloud/raw` with `camera0/undistort`, projects onto a `1/scale` low-resolution canvas, then dilates / Sobel-edge-suppresses / nearest-neighbor-upsamples to a full-resolution dense depth map. **Multi-device aware**: one `DeviceWorker` per SN, spawned / torn down on `device_online` / `device_offline`; `resolution_change` is dispatched by SN. The plain-types projection core (`pointcloud_to_depth_node.hpp`) is PCL/Eigen/ROS-free so it can be lifted into the on-device SDK. Source under `src/pointcloud_to_depth/`; see the in-folder `README.md` for the algorithm details.

| 话题 / Topic | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `{prefix}pointcloud2depth/depth` | `sensor_msgs/Image` (32FC1) | 稠密深度图，米，0 表示无效像素 / dense depth (meters), 0 = invalid |
| `{prefix}pointcloud2depth/depth_color` | `sensor_msgs/Image` (bgr8) | JET 配色可视化（无效像素为黑）/ JET-colored depth (black where invalid) |
| `{prefix}pointcloud2depth/depth_overlay` | `sensor_msgs/Image` (bgr8) | JET 叠加在相机图上 / JET overlay on the RGB image |

订阅 / Subscribes: `{prefix}cloud/raw`, `{prefix}camera0/undistort`, `/{topic_prefix}/driver/device_online`, `/{topic_prefix}/driver/device_offline`, `/{topic_prefix}/driver/resolution_change`。
依赖服务 / Service: `{prefix}get_calibration`（首次上线时拉取相机内外参 / fetched once per device on first online event）。

主要参数 / Key parameters:

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
| --- | --- | --- |
| `scale` | `7.0` | 投影画布下采样比 / projection canvas downscale factor |
| `z_min` / `z_max` | `0.1` / `50.0` | 有效深度范围（米）/ valid depth range (meters) |
| `dilate_radius` | `1` | 投影写入时的 3×3 膨胀 / dilation radius when writing projected pixels |
| `use_z_buffer` | `false` | 是否启用 Z-buffer 防止远点覆盖近点 / enable z-buffer to prevent far-over-near overwrite |
| `edge_grad_threshold` | `0.75` | Sobel 边缘抑制阈值（米/像素）/ Sobel edge suppression threshold (m/pixel) |
| `vis_depth_min` / `vis_depth_max` | `0.1` / `5.0` | JET 配色固定归一化范围；`<0` 时改为逐帧自动归一化 / fixed JET normalization range; set min `<0` for per-frame auto-norm |
| `sync_queue_size` / `sync_slop_sec` | `10` / `0.1` | message_filters 时间同步参数 / time-sync queue size & slop |

## 服务 / Services

| 服务 / Service | 类型 / Type | 说明 / Description |
| --- | --- | --- |
| `odin/get_calibration` | `GetCalibration` | 获取相机标定 YAML 内容 / Fetch the camera calibration YAML content |

## 常用参数 / Common Parameters

| 参数 / Parameter | 默认值 / Default | 说明 / Description |
| --- | --- | --- |
| `operating_mode` | `normal` | 工作模式 / Operating mode: `normal` / `standby` |

> **中文**：设备发现、序列号、网卡选择、UDP 端口等都由 SDK 内部自动处理，无需也无法在 launch 中配置。节点会枚举所有发现到的设备并为每台单独建立 topic 前缀（多设备热插拔）。
>
> **English**: Device discovery, serial number, NIC selection and UDP ports are all handled internally by the SDK — they are neither required nor configurable from the launch file. The node enumerates every discovered device and assigns a per-device topic prefix (multi-device hot-plug).

## 配置文件 / Configuration Files

| 文件 / File | 说明 / Description |
| --- | --- |
| `config/camera_calib.yaml` | 相机标定文件（从设备自动获取）/ Camera calibration file (auto-fetched from device) |
| `launch/driver.launch` | ROS1 启动配置 / ROS1 launch configuration |
| `launch/driver.launch.py` | ROS2 启动配置 / ROS2 launch configuration |
| `rviz/odin.rviz` | RViz 可视化配置 / RViz visualization configuration |

## 启动说明 / Startup Notes

1. `device_ip` 留空时自动广播发现设备 / When `device_ip` is empty, devices are auto-discovered via broadcast.
2. 连接后自动设置为 `normal` 模式开始数据推送 / Upon connection the device is set to `normal` mode and starts streaming data.
3. 默认启动 RViz，可通过 `start_rviz:=false` 禁用 / RViz starts by default; disable with `start_rviz:=false`.
4. Ctrl+C 退出时自动将设备设为 `standby` 模式 / On Ctrl+C exit the device is set back to `standby` mode.

## 目录结构 / Directory Layout

```
ros_driver/
├── config/                         # 配置文件 / Configuration files
│   └── control_command.yaml        # 数据流开关配置 / Per-stream enable flags
├── include/                        # 公共头文件 / Common headers
│   ├── camera/                     # 内参类型 / Intrinsic types
│   ├── service_types.h
│   └── utility/                    # 工具类（ros_compat, yaml loader 等）/ Utilities
├── launch/                         # 启动文件 / Launch files
├── module/sdk_api/                 # Odin SDK（自动编译）/ Odin SDK (auto-built)
├── rviz/                           # RViz 配置 / RViz configs
├── script/                         # 编译脚本 / Build scripts
├── src/                            # 源代码 / Source code
│   ├── odin_ros_driver_node.cpp    # 主驱动节点（ROS1/ROS2 统一）/ Main driver (ROS1/ROS2 unified)
│   ├── post_process_node.cpp       # 后处理节点 / Post-processing node
│   ├── camera_model/               # 相机投影模型 / Camera projection models
│   ├── cloud_render/               # 点云着色 / Cloud colorization
│   ├── color_undistort/            # 图像去畸变 / Image undistortion
│   ├── decoder/                    # JPEG 解码 / JPEG decoding
│   ├── disparity_to_pointcloud/    # 视差→点云 / Disparity → point cloud
│   ├── pointcloud_reprojector/     # 点云重投影节点 / Point-cloud reprojector node
│   ├── pointcloud_to_depth/        # 点云转深度图节点（多设备）/ Point-cloud → depth (multi-device)
│   └── stereo_rectifier/           # 立体校正 / Stereo rectification
└── srv/                            # 自定义服务 / Custom services
```

## 数据流配置 / Stream Configuration

**中文**：通过 `config/control_command.yaml` 可配置启用/禁用各数据流。

**English**: Enable/disable individual streams via `config/control_command.yaml`.

```yaml
register_keys:
  raw_point: true      # 原始点云 / Raw point cloud
  slam_point: true     # SLAM 点云 / SLAM point cloud
  image0: true         # 左相机图像 / Left camera image
  image1: true         # 右相机图像 / Right camera image
  imu: true            # IMU 数据 / IMU data
  odom: true           # 里程计数据 / Odometry data
```



## Q&A

### 1. 多台电脑话题数据干扰 / Topic collisions across multiple machines

**问题 / Problem**：多台电脑通过 WiFi/局域网连接，启动 ROS Driver 时话题相同导致数据干扰。

When multiple machines run the driver on the same WiFi/LAN, identical topic names cause cross-machine interference.

**ROS2 解决方法 / ROS2 solution**：
```bash
# 查看当前 DOMAIN_ID（空白或 0 为共享模式）
# Check current DOMAIN_ID (empty or 0 means shared)
echo $ROS_DOMAIN_ID

# 设置独立的 DOMAIN_ID（0-100，0 为共享）
# Set an isolated DOMAIN_ID (0-100; 0 = shared)
export ROS_DOMAIN_ID=1
```

**ROS1 解决方法 / ROS1 solution**：
```bash
# 方法一（推荐）/ Option 1 (recommended)
export ROS_LOCALHOST_ONLY=1

# 方法二：使用不同端口 / Option 2: use a different master port
roscore -p 11312
export ROS_MASTER_URI=http://localhost:11312
```

> 以上命令只对当前终端生效。永久设置请添加到 `~/.bashrc` 或 `/etc/profile`。
> The commands above only affect the current shell. Append them to `~/.bashrc` or `/etc/profile` to make them permanent.

### 2. 设备连接失败 / Device connection failure

**问题 / Problem**：无法发现或连接设备。 / Cannot discover or connect to the device.

**解决方法 / Solution**：
1. 确保设备与电脑在同一网段 / Make sure the device and host are on the same subnet.
2. 检查防火墙是否阻止 UDP 广播（端口 60000-60010）/ Check whether the firewall blocks UDP broadcast (ports 60000-60010).
3. 尝试指定设备 IP / Try specifying the device IP explicitly: `roslaunch odin_ros_driver_rev1 driver.launch device_ip:=192.168.1.251`

### 3. 图像分辨率不生效（ROS1）/ Image resolution not applied (ROS1)

**问题 / Problem**：ROS1 下设置 `image_width`/`image_height` 参数无效。 / The `image_width`/`image_height` parameters have no effect on ROS1.

**解决方法 / Solution**：升级到 v0.9.0 或更高版本，已修复此问题。 / Upgrade to v0.9.0 or later — the issue has been fixed.

### 4. rosbag 录制丢包 / rosbag Recording Packet Loss

**问题 / Problem**：使用 `rosbag record` 或 `ros2 bag record` 录制数据时，高频话题（如 IMU 400Hz）出现丢包或时间戳间隔异常。

When using `rosbag record` or `ros2 bag record`, high-frequency topics (e.g., IMU at 400Hz) experience packet loss or abnormal timestamp intervals.

#### ROS2 解决方法 / ROS2 Solution

ROS2 默认使用 DDS 协议，QoS 策略为 `BEST_EFFORT`，高频数据容易丢包。需要配置 QoS 覆盖文件：

ROS2 uses DDS protocol with default `BEST_EFFORT` QoS, which can cause packet loss for high-frequency data. You need to configure a QoS override file:

```bash
# 使用提供的 QoS 配置文件录制（配置文件位于 config/rosbag2_qos.yaml）
# Use the provided QoS configuration file for recording (located at config/rosbag2_qos.yaml)
ros2 bag record -a --qos-profile-overrides-path <path_to_ros_driver>/config/rosbag2_qos.yaml

# 示例 / Example:
ros2 bag record -a --qos-profile-overrides-path ~/catkin_ws/src/odin_ros_driver/config/rosbag2_qos.yaml
```

**QoS 配置说明 / QoS Configuration**：
- `reliability: reliable` - 确保数据可靠传输 / Ensures reliable data transmission
- `depth: 4000` - 队列深度，建议为频率的 10 倍 / Queue depth, recommended 10x the frequency

#### ROS1 解决方法 / ROS1 Solution

ROS1 使用 TCP 协议（TCPROS），天然保证数据可靠传输，**无需额外配置**。

ROS1 uses TCP protocol (TCPROS), which inherently guarantees reliable data transmission. **No additional configuration is needed.**

```bash
# ROS1 直接录制即可
# ROS1 can record directly
rosbag record -a

# 如果磁盘 IO 较慢，可增加缓冲区
# If disk IO is slow, increase buffer size
rosbag record -a --buffsize=256
```

#### 为什么 ROS1 和 ROS2 操作不同？ / Why are ROS1 and ROS2 different?

| 特性 / Feature | ROS1 | ROS2 |
| --- | --- | --- |
| 传输协议 / Protocol | TCP (TCPROS) | DDS (UDP-based) |
| 默认可靠性 / Default Reliability | 可靠 / Reliable | 尽力而为 / Best Effort |
| 丢包可能性 / Packet Loss | 低 / Low | 高频数据可能丢包 / Possible for high-freq data |
| 配置方式 / Configuration | 无需配置 / No config needed | 需要 QoS YAML 文件 / Requires QoS YAML |

**原理说明 / Technical Explanation**：
- **ROS1**：基于 TCP，协议层保证数据按序到达，不会丢包
- **ROS2**：基于 DDS/UDP，默认 `BEST_EFFORT` 策略不保证送达，需要显式配置 `RELIABLE` 策略

- **ROS1**: Based on TCP, the protocol layer guarantees in-order delivery without packet loss
- **ROS2**: Based on DDS/UDP, default `BEST_EFFORT` policy does not guarantee delivery, explicit `RELIABLE` configuration is required

## 版本历史 / Version History

详见 [CHANGELOG.md](CHANGELOG.md) / See [CHANGELOG.md](CHANGELOG.md) for details.
