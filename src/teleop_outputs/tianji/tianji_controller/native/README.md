# Tianji SPD 原生 DLS 求解与显示

本目录只服务一条链路：**PICO 裸手 → Python 共享根映射 → 原生 Franka DLS + Ruckig →
SPD 关节命令**。原生进程不再拥有 PICO 网络输入、真机输出或 ROS 边界：

| 目标 | 角色 |
|---|---|
| `pico2_dls_worker` | 私有 stdin/stdout 协议的双臂 DLS/Ruckig 求解器，只做模型参考 |
| `tianji_qp_ik_viewer` | `--external-display` 专用辅助窗口，只显示父进程传入的 54 维关节 |

唯一算法标识为 `pico_ee_franka_dls`，唯一 profile 为
`config/qp_ik_pico_shared_root_dls.yaml`。Pinocchio 提供双臂运动学，Franka DLS 求
关节目标，Ruckig 按速度、加速度和 jerk 约束生成参考；MuJoCo 提供模型状态与显示。
模型参考不等于机器人实测状态。无新鲜输入时保持／制动，不用脚本轨迹替代现场输入。

## 构建

```bash
# 工作区根目录：编译并安装两个 SPD 原生命令到 install/spd/bin。
bash bash/build_spd.sh
```

构建使用独立 `control` 环境（CMake 3.24、Eigen、MuJoCo、Pinocchio、yaml-cpp、
GLFW、Ruckig）；`bash/build_spd.sh` 以 `-DBUILD_TESTING=OFF` 配置，只构建并安装
`pico2_dls_worker` 与 `tianji_qp_ik_viewer`（安装组件 `spd`）。Ruckig Community
源码与 MIT 许可证位于 `third_party/ruckig/`，只使用本地在线轨迹生成。

需要单元测试时用 `control` 环境自行配置 `-DBUILD_TESTING=ON`；测试覆盖被两个
目标实际链接的库代码，不再包含已删除的 ROS、UDP 录制与交互标记路线。

## `pico2_dls_worker`：私有管道求解器

启动参数：`<profile.yaml> <model.xml> [--continuous-follow]`。

- 启动后先输出一行握手：
  `{"schema_version":1,"kind":"pico2_dls_ready","simulation_only":true}`。
- 请求固定 272 字节（magic `P2IQ`，版本 1），响应固定 432 字节（magic `P2IR`）；
  单进程单所有者，序列号与 epoch 必须单调，时钟必须非负且 `received <= now`。
- 操作码：`1` 复位（epoch+1）、`2` 求解（唯一产生运动参考的路径）、`3` 前向运动学、
  `4` 软启动跟随、`5`/`6` 有界停止、`7` 恢复状态推进、`8` 连续跟随恢复。
  `--continuous-follow` 只影响 `8` 的可用性。
- 求解前必须先复位；求解种子必须等于本进程参考，超出关节限位、非有限数、四元数
  退化、时钟回退与越权操作都会拒绝并进入保持。
- 响应包含每侧接受标志、参考关节、TCP 位姿、跟踪误差与恢复阶段名。

进程不监听任何 socket，不打开设备，不导出关节命令；退出码非零表示求解器故障，
调用方必须显式重建 worker。

## `tianji_qp_ik_viewer`：显示专用窗口

```bash
install/spd/bin/tianji_qp_ik_viewer --external-display \
  --model <marvin_m6_wuji2_shared_root_ceres.xml> --config <profile.yaml>
```

父进程（`pico2_hands`）通过私有 stdin 管道写帧：
`<timestamp_ns> <54 关节弧度> <状态文本>\n`。时间戳必须单调；帧必须整行且
54 维有限；状态文本中的 `|` 显示为换行。省略 `--model` 时使用 profile 冻结的
共享根模型。

输出行：`DISPLAY_READY`（窗口/管道就绪）、`DISPLAY_KEY <GLFW 键码>`
（控制键与退出转发给父状态机）、`display_complete frames=<N>`（结束）。
按键：`C` 标定（`--continuous-follow` 下为 `R`）、`S` 跟随、`P`/`Space` 保持、
`H` Home、`Q`/`Escape`/关窗退出、`F1` 帮助、`F2` 曲线、`F3` 指标、`F4` 锁手臂、
`F5` 跟随选中、`L` 选择左臂、`R` 选择右臂（仅非连续跟随），鼠标旋转／平移／缩放。

该窗口没有控制循环、没有第二个求解器、没有网络输入、没有录制与关节导出；
`--headless` 保留同一帧契约但不开窗口，便于无显示服务器的自检。

## 冻结几何与 artifact

共享根输入来源、坐标系和连续性所有权见
[输入契约](docs/shared_root_input_contract.md)。`config/shared_root_robot_geometry_dls.yaml`
固定模型路径、标识与 SHA256；`config/shared_root_tjvr_input_contract.yaml` 记录
输入语义。模型文件名 `marvin_m6_ceres_source.urdf` 和
`marvin_m6_wuji2_shared_root_ceres.xml` 是保留的冻结来源名称，不是运行时后端选项。
来源几何数据见 [冻结模型几何证据](docs/verification/shared_root_dls_geometry.md)。

两个 artifact 内容寻址：改动其内容会同时改变 profile 引用、几何文件的指向和
`src/shared_root_options.cpp` 中的冻结哈希。使用
`scripts/repin_shared_root.py` 重新固定几何哈希，并用
`scripts/validate_shared_root_contract.py` 只读核对 profile、artifact 与模型指纹；
两者都不授予运动权限。
