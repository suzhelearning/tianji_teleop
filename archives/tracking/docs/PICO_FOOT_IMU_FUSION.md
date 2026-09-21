# PICO + IMU900 脚部姿态融合

`pico_foot_imu_fusion` 用 PICO `/pico/smpl` 的 24 个关键点位置和骨盆姿态，
再用左右脚 IMU900 的姿态替换 PICO 的脚尖姿态，输出 `/pico/smpl_fused`。
作为标准输入的地面坐标 `/pico/smpl` 不会被融合节点修改；APK 原始头部原点
数据另行保存在 `/pico/smpl_raw`。

## 启动

本仓库的 `src/imu_ros2` 内置 IM900/IM948 驱动，
并增加 ready 状态和串口自动重连。左右脚 IMU900 各使用一个独立串口，不能使用旧下肢控制板的
921600 波特率 `LL/RL` 配置。

先构建并启动 PICO bridge：

```bash
bash scripts/build.sh
source scripts/environment.sh
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py
```

在另一个终端启动双 IMU900 驱动和融合节点：

```bash
cd /path/to/tianji_teleop/tracking
source scripts/environment.sh
ros2 launch pico_bridge start_pico_foot_fusion.launch.py \
  left_port:=/dev/ttyUSB0 \
  right_port:=/dev/ttyUSB1
```

默认驱动配置与参考工程一致：115200 波特率、110 Hz、`report_tag=46`、
`target_address=255`、启用磁力计和设备时间戳。检查两路数据：

```bash
ros2 topic info /imu/left_feet
ros2 topic info /imu/right_feet
ros2 topic hz /imu/left_feet
ros2 topic hz /imu/right_feet
ros2 topic echo /imu/left_feet --field orientation --once
ros2 topic echo /imu/right_feet --field orientation --once
ros2 topic echo /imu/left_feet/ready --once
ros2 topic echo /imu/right_feet/ready --once
```

如果实际设备名不同，只需覆盖 `left_port`、`right_port`。如果抬左脚却只有右侧
话题变化，交换这两个启动参数。

需要单独运行融合节点时：

```bash
source install/setup.bash
ros2 run pico_bridge pico_foot_imu_fusion \
  --ros-args \
  -p left_imu_topic:=/imu/left_feet \
  -p right_imu_topic:=/imu/right_feet \
  -p calibration_samples:=60 \
  -p imu_reset_settle_sec:=1.0
```

## 安装方向

当前两只脚的 IMU900 已通过实机逐轴动作验证：单位安装四元数能正确对应
roll、pitch、yaw 的轴和方向，因此默认使用 `[0,0,0,1]`。

如果以后改变物理安装方向，可在单独运行融合节点时传入左右安装四元数
（顺序 `x,y,z,w`）：

```bash
ros2 run pico_bridge pico_foot_imu_fusion --ros-args \
  -p left_mount_quaternion:="[0,0,0,1]" \
  -p right_mount_quaternion:="[0,0,0,1]"
```

## 标定

正常运行时，按 PICO 右手柄 A 键会发布 `/pico/world_reset`。融合节点随后并行调用
`/im900/left_foot/zero_z_axis` 和 `/im900/right_foot/zero_z_axis`。收到两路成功响应后，
节点按参考工程躯干 IMU 的流程等待 AHRS 稳定；脚部默认等待 1 秒。等待结束时丢弃期间的
IMU 缓存，再等待两路新数据并自动采集默认 60 帧中立姿态。看到日志
`PICO foot IMU calibration complete` 后才会恢复 `/pico/smpl_fused`。按键时请保持站立、双脚平放。

可观察事件和服务：

```bash
ros2 topic echo /pico/world_reset --once
ros2 service list | grep zero_z_axis
```

如果 IMU 服务不可用，自动事务会失败且不会保留旧基线；修复串口后再次按 A。也可以
使用下面的手动标定（不会向 IMU900 发送 Z 轴复位）：

保持站立、双脚平放，执行：

```bash
ros2 service call /pico_foot_imu_fusion/calibrate std_srvs/srv/Trigger "{}"
```

节点采集默认 60 个同步样本，建立骨盆和左右脚的中立姿态，以及 PICO 脚踝到脚尖
的位置偏移。完成后检查：

```bash
ros2 topic info /pico/smpl_fused
ros2 topic echo /pico/smpl_fused --once
ros2 topic echo /pico/ankle_relative --once
```

可视化融合结果：

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_fused --scale 1.0 --rate 60 --timeout 1.0
```

要在同一坐标系中对比 IMU 修正前后结果，启用原始骨架叠加：

```bash
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_fused \
  --show-raw \
  --raw-topic /pico/smpl \
  --scale 1.0 \
  --rate 60 \
  --timeout 1.0
```

主骨架仍使用蓝色关节、灰色骨骼和橙色脚板；PICO 未融合标准骨架使用较细的半透明绿色
关节、骨骼和脚板。两者不做平移，使用相同缩放和 `--yaw`，因此脚板姿态差异可直接
比较。原始话题无数据或超时不会阻塞 `/pico/smpl_fused` 的显示；不传 `--show-raw`
时行为与以前一致。

清除标定并重新开始：

```bash
ros2 service call /pico_foot_imu_fusion/reset std_srvs/srv/Trigger "{}"
```

## 注意事项

- IMU900 话题必须提供 `sensor_msgs/msg/Imu.orientation`，且左右脚消息时间间隔不能超过 `max_imu_age_sec`（默认 80 ms）。
- `/pico/smpl_fused` 只替换左右 FOOT 姿态/位置，保留 PICO 脚踝；`/pico/ankle_relative` 给出左右脚相对小腿的踝关节旋转。
- 任一 IMU 未 ready、姿态无效或超过 `max_imu_age_sec` 未更新时，融合暂停发布，而不是伪造另一只脚的数据；串口恢复后必须重新标定。
- IMU 串口初始打开失败或运行中断开时，驱动每秒自动重连并重新发送启动配置；ready 恢复前融合不会输出。
- MuJoCo 中橙色脚板使用融合四元数，因此脚部 pitch、roll、yaw 都可见。
- MuJoCo 对比模式中的半透明绿色脚板来自未融合的 `/pico/smpl`，橙色脚板来自融合结果。
- 如果脚尖仍与脚踝重合，先检查 `/pico/smpl_raw` 中索引 7/10 和 8/11 的位置是否有非零偏移，再调整安装方向和标定姿态。
