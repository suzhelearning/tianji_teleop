# 双脚 IMU900 原版驱动移植设计

## 目标

将参考工程
`/home/zj/sdk_test/catkin_exoskeleton_ws/src/imu_ros2`
中的 IM900/IM948 ROS 2 驱动移植到当前工程，替换当前误用的旧式下肢
`LL/RL` tagged-ASCII 驱动。当前工程最终应能通过两个独立串口读取左右脚
IMU900，并向 PICO 脚部融合节点稳定提供标准 `sensor_msgs/msg/Imu` 数据。

## 已确认的硬件事实

- 参考工程原有 `/imu/left_feet`、`/imu/right_feet` 来自下肢控制板聚合数据，
  使用 921600 波特率和 `LL/RL` tagged-ASCII 协议；它们不是独立 IMU900。
- 当前左右脚各安装一个独立 IMU900，两者都相对参考工程旧脚部传感器绕
  `+Z` 轴旋转 `+90°`。
- 参考工程真正的 IMU900 驱动是 `imu_ros2`，实现 IM900/IM948 二进制协议，
  每个传感器使用一个串口，默认波特率为 115200。

## 架构选择

### 原版驱动保持独立

把参考工程的整个 `imu_ros2` 包复制为当前工程的 `src/imu_ros2`。协议解析器、
单设备节点、多设备节点、头文件、原始 launch 文件和测试均保持参考行为一致；
仅增加通道 ready 状态和串口自动重连，以满足脚部融合的运行时安全要求。不得在
该包内加入 PICO 专用话题名、脚部安装角或融合算法。

通过协议测试和逐项 diff 校验确认当前工程的 `src/imu_ros2` 与参考目录在协议、
消息和设备命令行为上一致；ready 和重连代码属于本工程为双脚融合增加的运行时补丁。

### PICO 双脚配置位于集成层

PICO 专用的双脚 launch/config 放在 `pico_bridge` 中，由它启动
`imu_ros2/imu_multi_node`：

| 参数 | 左脚 | 右脚 |
|---|---|---|
| 默认串口 | `/dev/ttyUSB0` | `/dev/ttyUSB1` |
| channel name | `im900/left_foot` | `im900/right_foot` |
| frame id | `left_foot_imu_link` | `right_foot_imu_link` |
| 输出重映射 | `/imu/left_feet` | `/imu/right_feet` |

当前实机已确认左脚 IMU900 为 `/dev/ttyUSB0`、右脚 IMU900 为
`/dev/ttyUSB1`。两个串口仍作为 launch 参数开放，设备枚举发生变化时不需要
修改源码。

共享驱动参数严格采用参考工程 IMU900 配置：

- `baudrate=115200`
- `report_hz=110`
- `report_tag=46`
- `target_address=255`
- `enable_compass=true`
- `force_positive_w=true`
- `use_quaternion_continuity=true`
- `clear_ins_position=true`
- `clear_world_axes=false`
- `restore_world_axes=false`
- `use_device_timestamp=true`
- `coalesce_frames_per_poll=true`

### 删除错误驱动和耦合

删除当前 `src/pico_imu900_driver`。该包实际上是旧下肢控制板串口驱动，不能继续
以 IMU900 名义保留。融合节点删除对其 `DriverStatus.msg` 和
`/driver/lower_foot_imu/health` 的编译及运行时依赖。

融合节点的就绪条件改为：左右 IMU 四元数有效、两侧都已收到数据、消息接收时间
未超过配置的最大时延。任何一侧丢失或超时都停止发布修复结果并输出节流警告；
恢复收到双侧新数据后自动继续，无需专用驱动消息。

## 坐标与标定

参考工程旧脚部传感器到脚掌坐标的映射为 `Rz(-90°)`。当前 IMU900 又相对旧
传感器实测表现为循环轴映射 `x→y、y→z、z→x`，当前候选 IMU900→脚掌轴变换为
四元数 `[0.5,0.5,0.5,0.5]`，需要通过 pitch、roll、yaw 实机动作继续确认。

融合仍使用标定时刻的 IMU 四元数基线计算相对旋转。固定安装偏角在正确的相对
四元数计算中会抵消。左右安装四元数保留为可配置参数，默认使用上述循环轴候选值；
不得把参考旧传感器的 `Rz(-90°)` 再无条件施加到新 IMU900 数据上。

标定时必须同时满足：PICO 恰好包含 24 个点、全部位置为有限值、全部姿态四元数
有效、左右 IMU 数据新鲜且左右 IMU 姿态有效，
并在静止窗口内收集足够样本。标定完成后只替换左右 FOOT 姿态和脚尖位置，PICO
ANKLE 位置与姿态保持原始数据，同时继续发布脚踝相对姿态诊断话题。

## 故障处理

- 某一串口无法打开：`imu_multi_node` 报告对应端口错误；另一通道可以启动，但
  融合节点保持等待，不能生成单脚或伪造数据。
- 两个串口都无法打开：原版驱动按现有行为退出。
- IMU 四元数为无效哨兵、零范数或包含非有限值：立即清空该侧输入缓存，不用于
  标定或融合；后续有效数据可恢复输入，既有标定基线保留。
- 数据中断：基线保留，恢复后继续；如果设备重启导致其世界参考系变化，用户应
  重新调用融合标定服务。
- 左右设备接反：通过单脚动作测试识别，并交换 launch 的左右端口参数。

## 验证

### 静态验证

1. `diff -qr` 确认 `src/imu_ros2` 与参考包完全相同。
2. 构建全部 ROS 2 包。
3. 运行 `imu_ros2` 原有 IM948 协议测试及当前融合数学测试。
4. 测试双脚 launch 的端口、通道名、frame id、参数和话题重映射。
5. 确认源码中不再出现旧驱动的 `LL/RL`、921600 配置或 `DriverStatus` 依赖。

### 真机验证

1. 确认 `/dev/ttyUSB0` 对应左脚 IMU900、`/dev/ttyUSB1` 对应右脚 IMU900。
2. 检查 `/imu/left_feet`、`/imu/right_feet` 类型均为
   `sensor_msgs/msg/Imu`，频率接近 110 Hz，四元数范数接近 1。
3. PICO 和两个 IMU900 静止站立后执行融合标定。
4. 左右脚分别执行脚尖上翘、下压、内翻、外翻，确认只影响对应脚且方向正确。
5. 原地转身检查脚掌相对骨盆姿态没有 pitch/roll 串轴。
6. 拔掉任一 IMU，确认融合停止发布；重新连接并重启驱动后重新标定可恢复。

## 非目标

- 不修改参考工程目录中的任何文件。
- 不修改 `imu_ros2` 原版协议或加入 PICO 专用逻辑。
- 不兼容旧下肢控制板的 `LL/RL` tagged-ASCII 脚部传感器。
- 本次不自动生成 udev 规则；如果 USB 枚举顺序变化，通过 launch 参数覆盖左右端口。
