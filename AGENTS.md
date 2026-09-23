# SPD 专用分支

本分支只维护 PICO / Quest 裸手 → 外部 SPD 仿真的控制端。保留裸手 TCP/ADB 输入、共享根映射、Franka DLS/Ruckig、Hand2、54 维 ROS 目标发布及辅助原生 Viewer；不恢复 PICO 手柄、Manus、外骨骼、真机、相机采集或推理入口。

## 构建与运行

所有命令从仓库根目录执行：

```bash
bash bash/install.sh --check
bash bash/install.sh
# 更新源码后的构建
pixi run --locked -e spd build
# 1.75 必须替换为实际佩戴者身高
bash bash/run_pico_hand_sim.sh --height-m 1.75
# Quest 使用相同控制链，二选一启动：
bash bash/run_quest_hand_sim.sh --height-m 1.75
```

`spd` 是 Python 3.12 / ROS 2 Jazzy 运行环境；`control` 是原生 DLS/Viewer 的独立 ABI 环境，仍是 SPD 必需依赖。Hand2 使用包内 `tools/wuji_hand_native/` 的独立环境。构建入口负责准备依赖，产物位于 `build/spd`、`install/spd`、`log/spd`；不借用其他工作空间的 overlay。

## 控制边界

PICO 使用 `apps/pico/pico_hand_tracking_adb.apk`，Quest 使用 `apps/quest/quest3s_hand_tracking.apk`，输入默认 TCP 10002。两者 v1 FLU/OpenXR wire 格式相同，无需额外轴变换或 VR 人员档案；R 标定，S 开始，P/空格保持，H 双臂 Home，退出不自动 Home。`--headless` 不关闭 ROS 发布。Quest 独立诊断位于 `tools/hand_tracking/`，不要与遥操同时占用输入。

唯一跨项目出口为 `/spd/tianji_wuji2/v1/joint_command`，类型 `tianji_spd_interfaces/msg/JointCommand`，左臂7、右臂7、左手20、右手20，单位 rad，默认 domain 120、本机通信。SPD 仿真本体在独立 checkout `/home/current/syz/spd-syz`；本仓库不管理其进程，也不连接真机。

上游和 SPD 独立授权、暂停。发布不是授权、接收确认或执行确认；本地 Viewer 仅显示同一份目标，不生成命令。不要增加跨项目控制 RPC、第二套 IK 或旧 UDP 回退。

## 验证

优先运行不连接设备的真实 worker 自测：

```bash
bash bash/run_pico_hand_sim.sh --height-m 1.75 --self-test
```

它强制 domain 121，使用真实 DLS/Hand2 和合成输入。已有行为测试入口：`pixi run --locked -e spd test-pico2`。按改动选定向验证，不把合成结果宣称为真实头显或 SPD 动力学验收。

不要批量删除本地录制、日志、未跟踪文件或已安装环境。源码精简不意味着授权删除个人数据。
