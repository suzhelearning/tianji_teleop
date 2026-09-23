# PICO / Quest 裸手 → SPD

本分支只提供 **PICO 或 Quest 裸手输入到 SPD 仿真的控制端**。不包含 PICO 手柄、Manus、外骨骼、真机执行、相机采集或策略推理路线。

```text
PICO / Quest 裸手跟踪 APK → USB / ADB TCP :10002 → pico2_hands
  ├─ 手腕／掌心 → 共享根映射 → Franka DLS + Ruckig → 双臂 14 关节
  └─ 手部骨架 → Hand2 重定向                      → 双手 40 关节
       ↓
  ROS 2 JointCommand（54 维 rad）→ 独立 SPD 仿真
       └─ 私有管道 → 本地原生 Viewer（仅辅助显示）
```

SPD 仿真本体由独立项目 `/home/current/syz/spd-syz` 安装和启动。本仓库不启动、不管理该进程，也不连接机器人硬件。本地 Viewer 不运行第二套控制器，不代表 SPD 已接收或执行目标。

## 安装与构建

支持 Linux x86_64（glibc ≥ 2.34），需要 Pixi；实际头显输入另需 ADB 和 USB 调试权限，显示窗口需要图形桌面。

```bash
# 首次获取头显安装包；APK 使用 Git LFS 管理
git lfs install --local
git lfs pull --include="apps/pico/*.apk,apps/quest/*.apk"

# 检查安装前提，不连接设备
bash bash/install.sh --check
# 安装锁定环境并构建整个 SPD 控制端
bash bash/install.sh
```

也可分步执行：

```bash
pixi install --locked -e spd
pixi run --locked -e spd build
```

环境按实际 ABI 隔离：

- `spd`：Python 3.12、ROS 2 Jazzy、Fast DDS、裸手运行时与 ROS 消息绑定。
- `control`：原生 DLS/Ruckig worker 与辅助 Viewer 的编译和运行库。
- `src/teleop_inputs/pico_hand/tools/wuji_hand_native/`：Hand2 的独立锁定环境。

构建入口会准备后两者；构建产物只写入 `build/spd`、`install/spd`、`log/spd`。不要混用其他工作空间的 ROS overlay。

## 启动与操作

头显安装并打开裸手跟踪 APK（不是 VR 手柄 APK）。安装前停止旧输入和执行会话：

```bash
adb devices -l
adb install -r apps/pico/pico_hand_tracking_adb.apk

# 1.75 是示例，必须改成实际佩戴者身高（米）
bash bash/run_pico_hand_sim.sh --height-m 1.75
```

入口自动进入 `spd` 环境并检查／补建默认 `tcp:10002 → tcp:10002` 的 ADB 转发；已有冲突不会被覆盖。多设备时设置 `ANDROID_SERIAL`。不使用 VR 人员档案或手柄 TCP 标定。

双臂水平前伸、双手约肩宽、掌心相对，面向前方按 **R** 稳定保持约一秒；标定成功后按 **S** 开始跟随。

| 按键 | 上游控制端行为 |
|---|---|
| R | 标定／重新标定；运动中先制动，标定成功后仍等待 S |
| S | 标定有效且输入新鲜时开始／恢复跟随 |
| P / 空格 | 制动并保持 |
| H | 双臂回 Home，不自动恢复跟随 |
| Q / Esc / 关闭窗口 / Ctrl+C | 退出，不自动回 Home |

- `--headless` 只关闭显示，**仍发布 ROS 目标**。
- `--disable-hands` 关闭手指重定向并清除双手 ready 位，仍要求有效双腕输入。
- 同连接失鲜时制动；满足稳定输入条件后，可以恢复之前已用 S 启动的跟随。断连／重连须重新 R、再 S。
- 手势分类仅作观察标签，不承担使能、暂停或急停。
- SPD 的执行授权、暂停、保存和回退由 SPD 本地管理，不远程改变上游状态。ROS 发布不是执行授权或接收确认。

## Quest 3S 接入

Quest 交付文件从 `/home/current/syz/spd` 的 `spd-syz` 分支提交
`50a7ed954c871ee401c5d5e9693c1e0c0b34d9aa` 迁入，原仓库不作修改：

- `apps/quest/quest3s_hand_tracking.apk`：头显安装包。
- `tools/hand_tracking/quest_hand_tracking_receiver.py`：原始 TCP 诊断、JSONL 记录与可视化。
- `tools/hand_tracking/requirements-visualize.txt`：可选诊断 GUI 依赖；遥操不需要这些依赖。
- `bash/run_quest_hand_sim.sh`：复用**本仓库**的控制链，不依赖相邻 `tianji_teleop-ros2`。

Quest 开启开发者模式、授权 USB 调试并打开手部跟踪：

```bash
adb devices -l
adb install -r apps/quest/quest3s_hand_tracking.apk
# 在头显中打开应用并授予跟踪权限，然后运行：
bash bash/run_quest_hand_sim.sh --height-m 1.75
# 等价 Pixi 任务：
# pixi run --locked -e spd spd-quest-teleop --height-m 1.75
```

Quest 与 PICO 使用同一 v1 wire 协议：TCP 10002，小端 `<BBqI>` 帧头，
magic `0xAB`、type `0x40`、payload 1968 字节，双手各 26 个 OpenXR 关节。
坐标为 FLU（前、左、上），四元数为 `xyzw`；不额外翻轴或交换左右手。
共享模块、日志及录制 schema 仍使用 `pico2_hands` / PICO 名称。
R/S/P/H 操作和 SPD 授权边界与上文相同。

可先运行只读诊断，**结束诊断后**再启动遥操：

```bash
pixi run --locked -e spd spd-quest-receive --print
# 多设备诊断可加 --adb-serial SERIAL；遥操入口用 ANDROID_SERIAL。
# 记录解码数据可加 --save-jsonl /tmp/quest-frames.jsonl。
```

独立诊断默认执行 `adb forward`；若已有映射由其他程序管理，加
`--no-adb-forward`。不要同时运行诊断与遥操，也不要同时启动 PICO 与 Quest 发布器。
诊断 GUI 可在独立 Python 环境安装
`pip install -r tools/hand_tracking/requirements-visualize.txt`，再运行
`python tools/hand_tracking/quest_hand_tracking_receiver.py --visualize`；
它不是控制或 SPD 仿真窗口。

APK SHA-256：`4206849228aa6be0c8c16c3ee48240afb142f053820dc970bcd47991687ac22c`。
协议兼容不代表真实 Quest 跟踪精度、腕部方向或 USB 稳定性已验收。

## ROS 契约

| 项目 | 值 |
|---|---|
| 话题 | `/spd/tianji_wuji2/v1/joint_command` |
| 类型 | `tianji_spd_interfaces/msg/JointCommand` |
| 顺序 | 左臂 7、右臂 7、左手 20、右手 20 |
| 数值 | 绝对关节角目标，rad |
| QoS | BEST_EFFORT / KEEP_LAST(1) / VOLATILE |
| 环境 | Jazzy、Fast DDS、默认 domain 120、仅本机 |
| 频率 | 名义 200 Hz 控制，最多 60 Hz 发布最新生产快照 |

双方独立构建消息绑定，不互相 source overlay。`ready_mask`、会话代际和源时间参与接收门控；关节名一致不代表零位、方向、限位或动力学已验收。

## 离线检查

```bash
# 合成输入 + 真实 DLS/Hand2 worker + ROS 发布；不连接头显
bash bash/run_pico_hand_sim.sh --height-m 1.75 --self-test
# Quest 入口复用同一自测：
bash bash/run_quest_hand_sim.sh --height-m 1.75 --self-test

# 已有 SPD 行为测试，ROS 隔离到测试域
pixi run --locked -e spd test-pico2
```

自测强制使用 ROS domain 121，检查 R 后静止就绪、S 后双臂双手运动、运动中重新标定及再次显式启动；不代表真实头显质量、SPD 接收执行或现场实时性已验收。

本次分支精简已完成原生目标与 6 个 ROS 包构建。Quest 入口的合成自测经真实
DLS/Hand2 与独立 DDS 订阅器收到 250 条命令，覆盖 ready 撤销、R/S 和运动中重新标定；
另验证了 Quest 分片 TCP 解码、右手失跟踪有效位及 APK 完整性。
原生 Viewer 仅验证了无窗口私有管道；未连接真实头显、未运行外部 SPD 物理执行，
也未验收图形窗口或实时性能。

## 代码位置

- `src/teleop_inputs/pico_hand/`：裸手输入、标定映射、Hand2、SPD 发布及本地诊断录制。
- `src/interfaces/`：SPD 消息定义与资源定位辅助包。
- `src/teleop_outputs/tianji/`：SPD 所需 DLS/Ruckig 原生核心、显示器与模型资源。
- `src/simulation/`：裸手控制端消费的模型和关节布局辅助。
- `bash/`：SPD 安装、构建、启动与测试入口。
- `tools/hand_tracking/`、`apps/quest/`：迁入的 Quest 独立诊断工具与 APK。

详细参数、录制格式和失效规则见 [SPD 裸手控制端说明](src/teleop_inputs/pico_hand/README.md)。
