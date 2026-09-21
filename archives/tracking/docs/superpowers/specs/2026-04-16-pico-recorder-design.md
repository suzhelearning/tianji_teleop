# PICO Recorder 设计文档

**日期：** 2026-04-16
**目标：** 在本目录 `/home/eai/project/pico_project/pico_reciver_data/` 下，构建一个自包含的 ROS2 ament_cmake 包 `pico_recorder`，用来采集 PICO 4U 头显通过 USB/adb 推送过来的多路数据（双目相机、头+双手 6DOF、双 BLE 灵巧手），落盘成 HDF5。完全不修改 `/home/eai/project/exoskeleton_v2/` 中的任何代码。

---

## 1. 背景与目标

### 1.1 数据源
`PicoStreamingServer`（Unity 端，安装在 PICO 4U APK 里）通过 TCP 9999 推送以下数据，协议详见 `PICO_Streaming_Guide.md`：

```
帧格式：[0xAB magic:1B][type:1B][ts_ms:8B LE int64][payload_len:4B LE uint32][payload]
```

| Type | 含义 | Payload |
|------|------|---------|
| `0x01` | 左相机 | JPEG 字节 |
| `0x02` | 右相机 | JPEG 字节 |
| `0x03` | 左手柄 6DOF | 7×float32 = pos.xyz + rot.xyzw（28B） |
| `0x04` | 右手柄 6DOF | 同上 |
| `0x05` | 头显 6DOF | 同上 |
| `0x10` | 左 BLE（灵巧手/夹爪） | `[esp32_ts:4B LE u32][sensor_data:N B]` |
| `0x11` | 右 BLE | 同上 |

PC 端通过 `adb forward tcp:9999 tcp:9999` 把 PICO 的 9999 端口映射到本机。

### 1.2 设计目标
- **自包含**：所有代码、配置、构建脚本都在本目录，不依赖外部项目。
- **数据完整**：PICO 发送什么就保存什么，`ts_ms` / `esp32_ts` 原值落盘，BLE payload 原始字节一字不删。
- **架构解耦**：Bridge（TCP→ROS topic）和 Recorder（订阅→HDF5）两个节点分开，便于 rviz/rqt/rosbag 在线查看或离线重放。
- **实现语言一致**：两个节点都用 C++（rclcpp），和 `inference_cpp/` 现有节点风格对齐，避免 Python GIL/GC 抖动对高吞吐图像流的影响。

### 1.3 非目标
- 不采集外骨骼 Odin 编码器关节数据。
- 不做 TF 广播、不和机器人坐标系对齐（离线分析自己处理）。
- 不做实时可视化（直接用 `rqt_image_view` 订阅 topic 即可）。
- 当前 PICO 协议未传输手柄按键信号（仅 6DOF 位姿），所以本期 **不** 实现按键触发采集；但**预留 ROS service 接口**，未来 PICO 端协议扩展出按键帧后，外部小节点一行调用 service 即可触发 start/stop，无需修改本包。

---

## 2. 整体架构

```
PICO 4U ──adb forward──► TCP :9999
                           │
                           ▼
            ┌───────────────────────────┐
            │ pico_bridge_node (C++)     │
            │  TCP recv thread           │
            │  → 7 个 ROS Publisher       │
            └───────────────────────────┘
                           │
                           ▼
            ┌───────────────────────────┐
            │ pico_recorder_node (C++)   │
            │  7 个 Subscriber           │
            │  → DataCollector (本地精简) │
            │  → HDF5 (30Hz 栅格)         │
            │  键盘 a/s/q + ROS service   │
            └───────────────────────────┘
```

两个节点都运行在 PC 上，通过 DDS 本机回环通讯，零额外系统调用。

---

## 3. 组件详细设计

### 3.1 pico_bridge_node（新建）

**职责**：单线程 TCP 客户端，拆帧后按类型分派到 7 个 publisher。

**关键实现**：
- `rclcpp::Node` 派生类 `PicoBridgeNode`。
- 构造时：
  - 声明参数 `host` (默认 `127.0.0.1`)、`port` (默认 `9999`)。
  - 创建 7 个 publisher，QoS 按下表。
  - 启动 1 个 `std::thread` 跑 `recv_loop()`。
- `recv_loop()`：
  - BSD socket TCP 连接 + `TCP_NODELAY=1`。
  - 循环 `recv_exact(14)` → 校验 magic `0xAB` → `recv_exact(payload_len)` → switch by type → publish。
  - 连接异常时自动重连（2 秒间隔，复刻 `receiver.py` 行为）。
- 发布消息时：
  - `header.stamp.sec = ts_ms / 1000`
  - `header.stamp.nanosec = (ts_ms % 1000) * 1'000'000`
  - `header.frame_id = "pico"`
  - CompressedImage：`format = "jpeg"`，`data` 直接移动构造 JPEG 字节（零拷贝）。
  - PoseStamped：解 7×float32 到 `pose.position.{x,y,z}` / `pose.orientation.{x,y,z,w}`。
  - BleFrame：`header.stamp` 填 `ts_ms`；`esp32_ts` 字段从 payload 前 4B 取；`data` 字段填 payload 的剩余部分（不含 `esp32_ts`）。注：payload 里的 `esp32_ts` 在 bridge 里就抽出来走消息字段，而 HDF5 保存时 recorder 再把 `esp32_ts + data` 重新拼回完整字节存入 `hand_*/data`，保证 HDF5 里"原始字节一字不删"的承诺。

**QoS 规划**：

| Topic | 消息类型 | QoS profile |
|-------|----------|-------------|
| `/pico/cam_left/compressed` | `sensor_msgs/CompressedImage` | `rclcpp::SensorDataQoS()` (best_effort, keep_last=5) |
| `/pico/cam_right/compressed` | `sensor_msgs/CompressedImage` | 同上 |
| `/pico/pose/head` | `geometry_msgs/PoseStamped` | 默认 reliable (keep_last=10) |
| `/pico/pose/left_hand` | `geometry_msgs/PoseStamped` | 同上 |
| `/pico/pose/right_hand` | `geometry_msgs/PoseStamped` | 同上 |
| `/pico/ble/left` | `pico_recorder/msg/BleFrame` | 默认 reliable |
| `/pico/ble/right` | `pico_recorder/msg/BleFrame` | 默认 reliable |

**代码规模**：约 350 行。

### 3.2 data_collector（精简本地复刻）

**来源**：从 `/home/eai/project/exoskeleton_v2/hyper_leg_ros2/src/inference_cpp/include/inference_cpp/data_collector.hpp` 和 `src/data_collector.cpp` 复刻后精简。

**删除（相对原版）**：
- `default_datasets()` 整个函数。
- `register_default_datasets()` 成员函数和构造函数里的调用。
- `save_hdf5()` 里的 `/states/lidar`、`/states/lower_limb`、`/states/upper_limb`、`/states/torso` 四个写分组块，以及它们依赖的 `extract_columns(...)` 对应调用。

**扩展字段**：
```cpp
struct FrameData {
    Eigen::VectorXd data;
    double msg_timestamp;      // ROS header.stamp → seconds
    double recv_timestamp;     // PC wall clock on arrival
    int64_t ts_ms = 0;         // 新增：PICO 帧头 ts_ms 原值
};

struct ImageFrameData {
    std::vector<uint8_t> jpeg_data;  // 对 BLE 来说是整个 payload
    double msg_timestamp;
    double recv_timestamp;
    int64_t ts_ms = 0;         // 新增：PICO 帧头 ts_ms
    uint32_t esp32_ts = 0;     // 新增：BLE payload 前 4B，相机流恒为 0
};
```

**接口调整**：
```cpp
void add_data(const std::string& name,
              const Eigen::VectorXd& data,
              double msg_timestamp,
              int64_t ts_ms);

void add_image_data(const std::string& name,
                    const std::vector<uint8_t>& bytes,
                    double msg_timestamp,
                    int64_t ts_ms,
                    uint32_t esp32_ts = 0);
```

**新的 `save_hdf5()` 布局**：见第 4 节。

**命名空间**：改为 `pico_recorder`（避免和原版 `inference_cpp` 冲突，即使两边一起 build 也安全）。

**代码规模**：精简后约 450 行（原版 730 行）。

### 3.3 pico_recorder_node（新建）

**职责**：订阅 7 个 topic，按 30Hz 栅格把最新值喂给 DataCollector，键盘 + service 触发 start/stop。

**结构模板**：直接复刻 `inference_cpp/src/human_keypoint_recorder_node.cpp` 的骨架（键盘线程 + termios + `a/s/q` 处理 + `std_srvs/Trigger` service），把 ZMQ 订阅换成 ROS 订阅。

**关键实现**：
- 构造时：
  - 声明参数：`data_dir` (默认 `/tmp/pico`)、`collection_frequency` (默认 30.0)、`max_age` (默认 0.2)、`task_type` (默认 `"pico_collection"`)。
  - 实例化 `DataCollector`。
  - 注册数据集：
    ```cpp
    dc_->register_dataset("pose_head",       7, "Head 6DOF pos+quat",        true, -1.0);
    dc_->register_dataset("pose_left_hand",  7, "Left hand 6DOF pos+quat",   true, -1.0);
    dc_->register_dataset("pose_right_hand", 7, "Right hand 6DOF pos+quat",  true, -1.0);
    dc_->register_image_dataset("cam_left",   "Left camera JPEG stream",       true, -1.0);
    dc_->register_image_dataset("cam_right",  "Right camera JPEG stream",      true, -1.0);
    dc_->register_image_dataset("hand_left",  "Left BLE raw payload [4B esp32_ts | sensor]",  false, -1.0);
    dc_->register_image_dataset("hand_right", "Right BLE raw payload", false, -1.0);
    ```
    （BLE 设为非必需 `required=false`，避免未连接时阻塞采集。）
  - 创建 7 个 subscription，回调里调用 `dc_->add_*(...)`，时间戳从 `header.stamp` 反推 `ts_ms`；对 `UInt8MultiArray` 额外读前 4B 填 `esp32_ts`。
  - 创建 30Hz `rclcpp::TimerBase`，`timer_cb` 里调 `dc_->collect_frame()` 或 `dc_->update_stats()`。
  - 启动键盘线程（复刻 human_keypoint_recorder）。
  - 注册 services `/pico/start_collect`、`/pico/stop_collect`（`std_srvs/Trigger`）。

**CompressedImage 回调**：
```cpp
int64_t ts_ms = static_cast<int64_t>(msg->header.stamp.sec) * 1000
              + msg->header.stamp.nanosec / 1'000'000;
double msg_ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
dc_->add_image_data("cam_left", msg->data, msg_ts, ts_ms, 0);
```

**BleFrame 回调（自定义消息）**：
```cpp
int64_t ts_ms = static_cast<int64_t>(msg->header.stamp.sec) * 1000
              + msg->header.stamp.nanosec / 1'000'000;
double msg_ts = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

// 把 esp32_ts 和 data 重新拼成完整原始 payload，再喂给 DataCollector
std::vector<uint8_t> raw_payload;
raw_payload.reserve(4 + msg->data.size());
raw_payload.push_back(static_cast<uint8_t>(msg->esp32_ts & 0xFF));
raw_payload.push_back(static_cast<uint8_t>((msg->esp32_ts >> 8) & 0xFF));
raw_payload.push_back(static_cast<uint8_t>((msg->esp32_ts >> 16) & 0xFF));
raw_payload.push_back(static_cast<uint8_t>((msg->esp32_ts >> 24) & 0xFF));
raw_payload.insert(raw_payload.end(), msg->data.begin(), msg->data.end());

dc_->add_image_data("hand_left", raw_payload, msg_ts, ts_ms, msg->esp32_ts);
```

**`BleFrame.msg` 定义**：
```
std_msgs/Header header
uint32 esp32_ts
uint8[] data
```
需要在 `CMakeLists.txt` 里开 `rosidl_generate_interfaces(...)` 并在 `package.xml` 加 `rosidl_default_generators`/`rosidl_default_runtime` 依赖。成本：2-3 行 CMakeLists + 1 个 .msg 文件 + 2 行 package.xml 依赖。其他 topic（CompressedImage/PoseStamped）仍用 ROS 标准类型。

**代码规模**：约 250 行。

### 3.4 采集触发接口（稳定 API）

`pico_recorder_node` 暴露两层等价的触发方式，**任一方式都能完整完成 start → collect → stop+save 的流程**。这些是稳定接口，后续扩展（手柄按键、脚踏、外部脚本）只需调用，不用改本节点代码。

**层 1：键盘（交互 / 默认）**

| 键 | 行为 | 备注 |
|---|------|------|
| `a` | Start collection | 清空 buffer，开始收帧 |
| `s` | Stop + save HDF5 | 写 `pico_<unix_ts>.hdf5` 到 `data_dir` |
| `q` | Quit 节点 | 不保存 |

实现上复刻 `human_keypoint_recorder_node.cpp` 的 termios 非阻塞键盘线程。

**层 2：ROS service（编程 / 远程）**

| Service | 类型 | 等价键 |
|---------|------|--------|
| `/pico/start_collect` | `std_srvs/srv/Trigger` | `a` |
| `/pico/stop_collect`  | `std_srvs/srv/Trigger` | `s` |

两者共用同一把 `std::mutex` + 同一个 start/stop 实现函数，键盘和 service 触发互斥、状态一致。

**未来手柄按键接入方式（本期不实现）**

一旦 PICO 协议扩展出按键帧（方案 A/B），只需要写一个 **10~20 行**的小节点 `pico_button_trigger_node`，订阅按键 topic，在按键按下瞬间调用 `/pico/start_collect` / `/pico/stop_collect`。它不进 `pico_recorder` 包，可以放在本仓库或任何地方，和本包完全解耦。

### 3.5 launch 文件

`launch/start_pico_recorder.launch.py` 同时启动 bridge 和 recorder：

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    host_arg = DeclareLaunchArgument("host", default_value="127.0.0.1")
    port_arg = DeclareLaunchArgument("port", default_value="9999")
    data_dir_arg = DeclareLaunchArgument("data_dir", default_value="/tmp/pico")
    freq_arg = DeclareLaunchArgument("collection_frequency", default_value="30.0")

    bridge = Node(package="pico_recorder", executable="pico_bridge_node",
                  name="pico_bridge", output="screen",
                  parameters=[{"host": LaunchConfiguration("host"),
                               "port": LaunchConfiguration("port")}])
    recorder = Node(package="pico_recorder", executable="pico_recorder_node",
                    name="pico_recorder", output="screen", emulate_tty=True,
                    parameters=[{"data_dir": LaunchConfiguration("data_dir"),
                                 "collection_frequency": LaunchConfiguration("collection_frequency")}])
    return LaunchDescription([host_arg, port_arg, data_dir_arg, freq_arg, bridge, recorder])
```

---

## 4. HDF5 输出结构

```
pico_<unix_ts>.hdf5
├── @attrs:
│     task_type              str       (e.g. "pico_collection")
│     collection_frequency   float64   (Hz, 目标)
│     actual_frequency       float64   (Hz, 实测)
│     duration               float64   (s)
│     num_frames             int32
│     dropped_frames         int32
│     collection_start_time  float64   (unix sec, PC wall clock)
│
├── /states/
│   ├── pose_head/
│   │   ├── pos          (N, 3)  float64    pos.xyz
│   │   ├── rot_xyzw     (N, 4)  float64    quat
│   │   ├── ts_ms        (N,)    int64      PICO 帧头原值
│   │   └── recv_stamp   (N,)    float64    PC wall clock on arrival
│   ├── pose_left_hand/  同上
│   └── pose_right_hand/ 同上
│
├── /observations/images/
│   ├── cam_left/
│   │   ├── jpeg         (N,)    vlen uint8  原始 JPEG 字节
│   │   ├── ts_ms        (N,)    int64
│   │   └── recv_stamp   (N,)    float64
│   │   @attrs: format="jpeg", description=...
│   ├── cam_right/       同上
│   ├── hand_left/
│   │   ├── data         (N,)    vlen uint8  完整 payload (含 4B esp32_ts 前缀)
│   │   ├── ts_ms        (N,)    int64       PICO 帧头
│   │   ├── esp32_ts     (N,)    uint32      从 payload 前 4B 冗余抽取
│   │   └── recv_stamp   (N,)    float64
│   └── hand_right/      同上
│
└── /timestamp/
    └── collect_time     (N,)    float64    30Hz 栅格 tick 的墙钟秒
```

**每一帧的 N 对齐**：DataCollector 的 30Hz 栅格模式 → 所有 dataset 共享同一个 `N = 总采集帧数`；每个 tick 对每个 dataset 取"最新 buffer 值"，自然对齐。

**存储估算**（5 分钟录制，30Hz）：
- N = 9000
- 位姿：3 × (9000 × 7 × 8B) ≈ 1.5 MB
- 相机：2 × 9000 × ~25KB ≈ 450 MB
- BLE：2 × 9000 × ~30B ≈ 550 KB
- 时间戳：< 1 MB
- **总计 ~450 MB**（JPEG 已压缩，HDF5 gzip 再压基本无压缩收益）

---

## 5. 时间戳语义一览

| 字段 | 来源 | 类型 | 单位 | 语义 |
|------|------|------|------|------|
| `ts_ms` | PICO 帧头 | int64 | ms | 相对 PICO app 启动，**所有类型共享同一时钟**，帧间对齐的金标准 |
| `esp32_ts` | BLE payload 前 4B | uint32 | ms | 相对 ESP32 启动，左右手两个 ESP32 各自独立时钟 |
| `recv_stamp` | Recorder 回调 | float64 | s (unix) | PC 侧墙钟，诊断 bridge→recorder 延迟 |
| `collect_time` | Timer tick | float64 | s (unix) | DataCollector 30Hz 栅格时刻 |

---

## 6. 文件布局

```
pico_reciver_data/                       ← ament_cmake 包根目录
├── CMakeLists.txt                       ← 新
├── package.xml                          ← 新
├── PICO_Streaming_Guide.md              ← 保留
├── receiver.py                          ← 保留（协议参考）
├── README.md                            ← 新，使用说明
├── msg/
│   └── BleFrame.msg                     ← 新，自定义 BLE 消息
├── include/pico_recorder/
│   └── data_collector.hpp               ← 从 inference_cpp 复刻+精简
├── src/
│   ├── data_collector.cpp               ← 从 inference_cpp 复刻+精简
│   ├── pico_bridge_node.cpp             ← 新
│   └── pico_recorder_node.cpp           ← 新
├── launch/
│   └── start_pico_recorder.launch.py    ← 新
└── docs/superpowers/specs/
    └── 2026-04-16-pico-recorder-design.md  ← 本文档
```

---

## 7. 构建与运行

### 7.1 构建（方式 2：父级 mini workspace）
```bash
cd /home/eai/project/pico_project
mkdir -p ws/src
ln -s ../../pico_reciver_data ws/src/pico_recorder
cd ws
colcon build --packages-select pico_recorder
source install/setup.bash
```

### 7.2 运行
```bash
# 终端 1：adb 端口转发
adb forward tcp:9999 tcp:9999

# 终端 2：启动 bridge + recorder
source /home/eai/project/pico_project/ws/install/setup.bash
ros2 launch pico_recorder start_pico_recorder.launch.py data_dir:=/tmp/pico

# 在 recorder 终端：按 a 开始，按 s 停止并保存，按 q 退出
# 或通过 service 触发：
ros2 service call /pico/start_collect std_srvs/srv/Trigger
ros2 service call /pico/stop_collect  std_srvs/srv/Trigger
```

### 7.3 在线查看（调试用）
```bash
ros2 topic echo /pico/pose/head
ros2 run rqt_image_view rqt_image_view /pico/cam_left/compressed
```

---

## 8. 测试策略

### 8.1 单元测试
- DataCollector：注册 → 喂假数据 → collect_frame → stop → 检查 HDF5 字段存在、形状正确、时间戳递增。
- PICO 帧解析：构造 mock TCP 字节流（固定 magic + 各类型 payload），验证解析正确性。

### 8.2 集成测试
- 不带真机：用 Python 小脚本模拟 `PicoStreamingServer`，监听 9999 端口按协议推送假帧。跑完整 launch，验证 HDF5 生成、7 个 dataset 全部齐全、时间戳一致。
- 带真机：连上 PICO 4U，`a` → 走两分钟 → `s`，用 `h5py` 读出 HDF5，抽几帧 `cv2.imdecode` 看图像正常，画位姿轨迹看起来合理。

---

## 9. 风险与缓解

| 风险 | 影响 | 缓解 |
|------|------|------|
| 位姿原生 72Hz 被降采到 30Hz | 快速动作细节丢失 | 30Hz 对大多数任务足够；若需高频可将 `collection_frequency` 参数调为 60 或 90 |
| 相机帧积压 | 延迟飙升 | SensorDataQoS `best_effort + keep_last=5`，自动丢旧帧 |
| BLE 断连 | 整帧 stale | `hand_*` 注册为 `required=false`，不阻塞采集；HDF5 中缺失帧该 dataset 为空 |
| adb forward 断开 | Bridge recv 失败 | Bridge while 循环自动重连 |
| `UInt8MultiArray` 无 header | 拿不到 ts_ms | 改用自定义 `BleFrame.msg`（已定）|
| 30Hz 栅格对不齐导致"错配一帧" | 不同流可能差 1/30 秒 | 额外存了原始 `ts_ms` 供离线重采样；30Hz 误差上限 16.67 ms，对人类动作可接受 |

---

## 10. 未来可能的扩展（非本期）

- 支持 rosbag2 录制（把 topic 直接录为 .mcap，绕过 HDF5）。
- Bridge 加反向指令（ROS service → TCP → PICO），控制 PICO 端重置坐标系。
- 加个 `pico_replayer_node` 从 HDF5 反向发布 topic，便于离线重放。
- 相机帧独立异步存盘（不走 30Hz 栅格，保留 PICO 原生 30fps），位姿仍走栅格。

---

## 11. 术语

- **PICO 4U**：VR 头显硬件。
- **adb forward**：Android Debug Bridge 端口转发，把 Android 设备的 TCP 端口映射到 PC 本机。
- **6DOF**：6 Degrees of Freedom，3 维平移 + 3 维旋转（这里用四元数 4 维表达），共 7 个 float。
- **ESP32**：BLE 灵巧手/UMI 夹爪上的微控制器。
- **vlen uint8**：HDF5 的 variable-length uint8 数组类型（`H5Tvlen_create(H5T_NATIVE_UINT8)`），每帧可以是不同长度的字节数组。
- **DataCollector**：`inference_cpp` 中的核心采集工具类，本文档指其本目录精简复刻版。
