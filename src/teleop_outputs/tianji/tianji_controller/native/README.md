# Tianji 双臂原生控制核心

本目录只保留一条控制链：**共享根掌心映射 → Franka DLS → Ruckig**。
唯一算法标识为 `pico_ee_franka_dls`，唯一控制 profile 为
`config/qp_ik_pico_shared_root_dls.yaml`。共享根映射和形状参数分别使用
`shared_root`、`shared_root_shape`，不提供运行时算法切换。

Pinocchio 提供双臂运动学，Franka DLS 求关节目标，Ruckig 按关节速度、加速度和
jerk 约束生成参考。MuJoCo 用于模型状态和显示；模型参考不等于机器人实测状态。
无新鲜输入时保持／制动，不用脚本轨迹替代现场输入。

## 构建

以下命令从工作区根目录执行，依赖由 Pixi 的独立环境提供：

```bash
# 原生模型 Viewer、输入录制器和 DLS worker。
pixi run -e control bash bash/build_native.sh

# 生产 ROS 边界；脚本自行进入独立 arm-ros 环境。
bash bash/build_arm_ros.sh
```

核心构建目录为 `build/control/core`，ROS 构建目录为 `build/arm-ros/core`，
安装前缀为 `install/control`。不要混用 default 环境与 control／arm-ros 的数值库 ABI。
核心依赖为 C++20、CMake、Eigen、MuJoCo、Pinocchio、yaml-cpp、GLFW 和 Ruckig；
ROS 入口另依赖 rclcpp 与 `tianji_interfaces`。
Ruckig Community 源码和 MIT 许可证位于 `third_party/ruckig/`，
只使用本地在线轨迹生成，不需要云端服务。

## 生产 ROS 入口

`tianji_arm_ros` 接收 `tianji_interfaces/msg/PicoArmInput`，输出
`tianji_interfaces/msg/ControllerJointTargets`。工作区生产话题由根
`config/robot.json` 指定，通常为 `/pico/arm_input` 和
`/tianji/controller/joint_targets`；此入口不使用双臂业务 UDP 15000／17000。

下面是**不导出关节目标**的无窗口运行方式：

```bash
pixi run -e arm-ros install/control/bin/tianji_arm_ros \
  --config src/teleop_outputs/tianji/tianji_controller/native/config/qp_ik_pico_shared_root_dls.yaml \
  --pico-topic /pico/arm_input \
  --joint-target-topic '' \
  --headless --continuous
```

ROS domain 与发现范围应和上游发布端一致。生产仿真禁用目标导出；真机／dry-run
由 Python 执行器启动本入口，并显式同时传入 `--franka-dls-executor` 和非空
`--joint-target-topic`。该开关只允许受限 DLS/Ruckig 目标导出，**不是电机使能授权**。
设备连接、会话授权、硬件 SDK、输入／反馈时效检查仍归 Python 执行器所有。
不要将直接启动原生程序当作真机启动流程。

## 原生 Viewer 与独立显示

`tianji_qp_ik_viewer` 保留本地 TJVR 输入与模型参考显示，用于脱离 ROS 的输入观察。
从工作区根目录启动：

```bash
pixi run -e control install/control/bin/tianji_qp_ik_viewer \
  --config src/teleop_outputs/tianji/tianji_controller/native/config/qp_ik_pico_shared_root_dls.yaml \
  --pico-teleop --pico-bind 127.0.0.1 --pico-port 15000
```

默认不导出关节命令。录制器与 Viewer 不能同时绑定相同接收端口。
`--headless --duration 10` 可用于有界无窗口会话；`--continuous` 持续运行。
`--telemetry FILE` 和 `--joint-telemetry FILE` 保存控制与关节诊断。
非 ROS 入口的受限目标导出同样要求 `--franka-dls-executor`，并限定 loopback。

`--external-display` 保留为 Python 仿真／执行器的私有 stdin 显示入口，
只显示调用方传入的关节状态，不运行控制循环、输入接收、录制或目标导出。
`--continuous-follow` 仅可配合该显示模式使用；不要用它代替生产 ROS 控制入口。

## 输入录制、回放与 artifact 工具

以下脚本位于 `src/teleop_outputs/tianji/tianji_controller/native/scripts/`，
在已安装工作区 Python 包的环境运行；它们不连接硬件 SDK，不授予运动权限。

| 工具 | 用途 |
|---|---|
| `record_shared_root_actions.py` | 调用 `tianji_record_action_trace`，录制 50 秒原始 TJVR 输入、提示事件与配置快照 |
| `replay_pico_udp_trace.py` | 按录制接收时间间隔发送 TJVT 内的原始 TJVR 报文 |
| `audit_shared_root_trace.py` | 只读检查简化标定与 TJVR 有效骨长、坐标及报文一致性 |
| `validate_shared_root_contract.py` | 只读检查 DLS profile、输入来源哈希与冻结机器人几何 |
| `repin_shared_root.py` | artifact 维护工具：更新输入契约引用哈希与原生几何允许哈希；`--check` 不写文件 |

例如，以下命令均从工作区根目录执行：

```bash
native=src/teleop_outputs/tianji/tianji_controller/native

python "$native/scripts/record_shared_root_actions.py" plan
python "$native/scripts/record_shared_root_actions.py" record \
  --output recordings/shared_root/session-new \
  --participant PARTICIPANT --calibration-dir /path/to/pico-simple/revision

python "$native/scripts/replay_pico_udp_trace.py" \
  --input recordings/shared_root/session-new/input.tjvr --port 15000

python "$native/scripts/validate_shared_root_contract.py"
```

录制目录必须不存在；现场录制必须指定参与者和标定目录。
动作提示只是待确认区间，不能证明佩戴者实际完成动作；只有观察者核对后才可运行
`confirm-prompts --session DIR --reviewer NAME`，并且确认不构成运动授权。
回放不重写源时间戳或 CRC，不能冒充新鲜现场输入驱动真机。

## 冻结几何与边界

共享根输入来源、坐标系和连续性所有权见
[输入契约](docs/shared_root_input_contract.md)。
`config/shared_root_robot_geometry_dls.yaml` 固定模型路径、标识与 SHA256；
模型文件名 `marvin_m6_ceres_source.urdf` 和
`marvin_m6_wuji2_shared_root_ceres.xml` 是保留的冻结来源名称，不是运行时后端选项。
来源几何数据见 [冻结模型几何证据](docs/verification/shared_root_dls_geometry.md)。

离线 artifact 一致性、模型显示和录制回放都不能替代真实输入、动力学或真机验收。
