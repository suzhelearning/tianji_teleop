# 两种遥操路径

工作目录：`~/syz/tianji_teleop`。PICO 控制 Tianji 双臂；外骨骼或 Manus 控制 Wuji 双手。两种路径共用 `teleop.sh`，只替换手部输入来源。

下列命令分别在独立终端运行。`--user zj` 仅为示例，必须替换为实际佩戴者已标定的档案名。

外骨骼代码、标定配置、模型和官方求解器已纳入 `exoskeleton/`，不依赖外部 `~/syz/data_glove_wuji_teleop`。已有主项目环境时，先执行 `bash install.sh --exoskeleton` 安装仓库内独立的 Python 3.12 环境；主项目仍使用 Python 3.11。原外部目录保留，但两处配置不会自动同步，后续使用和标定以本项目 `exoskeleton/config/` 为准。

每帧四连杆／21 通道机构换算和独立 MANO 手型拟合使用 `exoskeleton/native/` 中的 C++17 扩展，不再运行旧 Python 数值循环。配置、设备管理和官方 Wuji worker 保持现有接口。首次安装或修改 C++ 后执行 `bash install.sh --exoskeleton`，需要 `build-essential`；安装器会重建本地扩展，重启发送器后生效，无 Python 回退路径。

## 外骨骼 + PICO → Tianji + Wuji

```text
PICO → UDP :15000 → 双臂控制
外骨骼 → exo.sh（采集、重定向、TJH2 直发）→ UDP :16000 → 双手控制
双臂／双手控制 → teleop.sh 选择仿真或真机
```

```bash
# 终端 1：PICO
cd ~/syz/tianji_teleop
bash pico.sh --user zj

# 终端 2：外骨骼采集、重定向与直发
cd ~/syz/tianji_teleop
bash exo.sh

# 终端 3：仿真
cd ~/syz/tianji_teleop
bash teleop.sh --sim
```

- 默认启动双手；仅左手用 `bash exo.sh --hand left`，仅右手用 `bash exo.sh --hand right`。
- 入口默认允许采集发送和未验收方向调试，不需要追加 `--confirm-send` 或 `--commission-directions`；这不会修改方向验收标记，也不代表真机方向与动作已验证。
- 此路径不运行 `manus.sh`。外骨骼使用其自身设备身份、零位和方向档案。
- `bash exo.sh` 与 `.venv/bin/tianji exoskeleton` 是同一发送入口；只启动手套发送器，不自动启动 PICO 或执行器。
- 默认直接向 `127.0.0.1:16000` 发送控制器的 TJH2 v2 数据，不再需要独立桥接。源时间使用同机单调时钟，仅允许回环 IPv4 目的地址；切换前先停止旧发送器和旧桥接进程。
- 每侧保留原始帧读取完成时的本机时间，FK／求解耗时计入年龄；另一侧更新不会刷新旧姿态的年龄。没有新结果不发包，失败侧清除缓存，退出不补零。
- 离线检查：`bash exo.sh --check-config`；只检查左手用 `bash exo.sh --hand left --check-config`，均不连接设备或发送数据。
- 设备与零位档案位于 `exoskeleton/config/dataglove/devices/`，任务绑定位于 `exoskeleton/config/teleoperation/`。传给 `exo.sh` 的相对配置路径以 `exoskeleton/` 为基准。

## Manus + PICO → Tianji + Wuji

```text
PICO → UDP :15000 → 双臂控制
Manus → manus.sh（采集、重定向和桥接）→ UDP :16000 → 双手控制
双臂／双手控制 → teleop.sh 选择仿真或真机
```

```bash
# 终端 1：PICO
cd ~/syz/tianji_teleop
bash pico.sh --user zj

# 终端 2：Manus，使用同一实际佩戴者的档案
cd ~/syz/tianji_teleop
bash manus.sh --user zj

# 终端 3：仿真
cd ~/syz/tianji_teleop
bash teleop.sh --sim
```

此路径不运行 `exo.sh`；`manus.sh` 已管理 Manus 采集、手部适配和 Hand2 UDP 桥。

## 共用执行模式与切换

在 `~/syz/tianji_teleop` 中选择一种模式：

| 模式 | 命令 |
|---|---|
| 有窗口仿真 | `bash teleop.sh --sim` |
| 无窗口仿真 | `bash teleop.sh --sim --headless` |
| 真机执行 | `bash teleop.sh --real` |

切真机前先停止仿真，完成只读设备预检：

```bash
.venv/bin/python real_robot/run_teleop.py --devices all --inspect
```

确认设备身份、标定、方向和安全空间后，才启动真机模式并按交互提示逐步授权；只读预检通过不等于动作安全已验证。

**同一时刻只运行一套 PICO、一种手部输入和一个执行模式。**外骨骼与 Manus 不可同时向手部控制端发送，仿真与真机也不可同时占用相同输入端口。切换路径时先停止执行端，再停止旧手部输入，切换并检查后重新启动执行端。
