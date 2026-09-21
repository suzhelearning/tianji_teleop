# mapped-palm 独立后端移植

本入口移植 `dexhand_deploy` 的 C++ `pico_ee_mapped_corrected_palm_velocity_qp`，
在本工程中用 `--ik-backend mapped-palm` 显式选择；`--sim`、`--real` 和 `--data` 默认均为共享根 Franka DLS＋Ruckig。
源码基线为 `a6ff1bd9a32e0a2b3dac2c131abf0cf0c2ea00b4`；数学核心、模型/TCP、
原始参数与来源关系见 `control/mapped_palm/migration_manifest.json`。

## 范围

链路：本工程原有 PICO driver/M0/bridge → TJVR → 新 C++ mapped-palm →
本工程 TJRC → 原有仿真或执行器。Manus/Hand2 保留本工程的输入、重定向、
侧别时间戳协议和驱动。没有引入源工程整套 Zenoh、HDF5 或现场调度框架。

- 位置跟踪松弛权重 90000，姿态权重 30000，原生 QP/OTG 逻辑保留。
- `bandwidth.yaml` 保留来源参考配置；现场 `deployment.yaml` 仅替换启动 Home：
  左 `[55,-65,-70,-60,60,0,0]°`，右 `[-55,-65,70,-60,-60,0,0]°`。
- 采用随库模型的 `hand_tcp_frame_L/R`，不借用 legacy SPARK 模型/TCP。
- 真机入口只新增后端选择；仍经过原有实际反馈初始化、降速、限位、
  多次 Enter 确认和退出处理。**离线通过不代表真机验收。**
- 输入失鲜撤销机械臂 ready；已接管后 tracking epoch/reset 改变会锁住 ready，
  不静默跨 epoch 恢复。新仿真入口可用下面的 P/R/S 受控恢复；显式 mapped-palm 真机默认仅对 TELEOP
  的纯 PICO 断流提供从最后有效输入起 300 ms 的保持/恢复窗口，其他故障仍锁存。
  详见 [真机断流策略](mapped-palm-real-readiness.md)；可用 `--mapped-palm-dropout-policy stop` 禁用该例外。
- 上述短时保持仅属于 mapped-palm；默认 DLS 真机在源失鲜、epoch 改变或映射／IK 拒绝时停止，不启用仿真自动恢复。

## 构建

在本工程根目录运行：

```bash
bash scripts/build_mapped_palm.sh
```

使用本工程 Pixi 原生依赖和 `control/third_party/ruckig`，不依赖其他工程的
绝对路径或构建产物。构建目录为 `control/build-mapped-palm/`。
Python 仿真需安装 MuJoCo 和 YAML；本轮验证版本为 MuJoCo 3.10.0：

```bash
pixi run python -m pip install 'mujoco==3.10.0' 'PyYAML>=6,<7'
export TIANJI_PYTHON="$PWD/.pixi/envs/default/bin/python"
```

本次合并在当前目录创建了 Pixi 默认环境，未复制来源虚拟环境或修改 `.venv`。
使用上述 `TIANJI_PYTHON` 显式选择已配置环境；PICO ROS runtime、厂商 SDK 和
外骨骼独立环境仍按原 README 配置，本次未重新部署或连接这些硬件依赖。

## PICO＋VR 手柄仿真（无需 Manus）

先戴好正确的 VR 手柄路线 APK，并选择**实际已发布的人员标定**，不要复制其他人的标定。
终端一：

```bash
export TIANJI_PYTHON="$PWD/.pixi/envs/default/bin/python"
bash scripts/start_mapped_palm_pico.sh --user YOUR_USER
```

将 `YOUR_USER` 替换为本工程实际人员名称。脚本解析该人员的 PICO 配置，
调用原有托管 PICO runtime，显式设置 bridge 世界 X 偏移 **+0.20 m**。
已有托管会话时拒绝启动，避免误用 SPARK 的旧参数；不要同时运行两条输入路线。
原始 PICO 启动脚本不带新选项时仍为 +0.10 m。

终端二，原版直接映射：

```bash
export TIANJI_PYTHON="$PWD/.pixi/envs/default/bin/python"
bash scripts/run_mapped_palm_sim.sh
```

不带标定选项时，按目标工程原有方式在有效输入到达后接收命令，无需按 S。
没有 Manus 数据时，手部保持目标，不要求连接手套。

### 对照源工程的直接关节显示

仿真默认使用 `direct`（SPARK 和 mapped-palm 均适用）：直接显示期望关节角，
不经过 PD 或动力学积分，没有重力下垂。可显式指定 `--simulation-mode dynamics`
运行原有动力学测试；真机入口不受此默认值影响。
同一套输入、IK、配置和 C 标定条件下，使用：

```bash
bash scripts/run_mapped_palm_sim.sh \
  --simulation-mode direct \
  --mapped-palm-xz-calibration
```

直接模式将通过原有新鲜度、ready、限位检查的关节目标写入 qpos，再调用 `mj_forward`。
不调用 `mj_step`，所以没有 PD 追踪滞后、重力下垂或碰撞阻挡运动。
这是源工程同类的**运动学显示语义**，不是完整源 Viewer/调度框架迁移，也不是物理仿真。
恢复模块使用显示关节变化率判断静止，不把活动中的显示模型一律当作零速度。

与下面的动力学模式分别运行，不能同时争用 PICO/手部输入端口：

```bash
bash scripts/run_mapped_palm_sim.sh \
  --simulation-mode dynamics \
  --mapped-palm-xz-calibration
```

P/R/S 与 C 操作不变；IK 权重、限速、输入映射及真机驱动未改变。
direct 日志中的 `clock_kind=display_only` 表示时间仅为显示调度时间，
兼容字段 `physics_time_s` 不代表做过动力学积分。
直接显示跟随顺畅不等于真机也能同样跟随；真机入口不支持该参数。

本轮验证：仿真回归 11/11；direct 无输入 headless 两秒正常退出，收到 322 帧，
序号无缺口。相同 Home、相同 0.1 rad 关节阶跃的离线 A/B 中，direct 第一步即等于
关节目标且保持；dynamics 则存在伺服过渡和重力静差。此测试未使用真实输入，
不代替现场运动对照，也不证明实时 QP 预算没有影响。

需要前伸 X/Z 标定时改为：

```bash
bash scripts/run_mapped_palm_sim.sh --mapped-palm-xz-calibration
```

点击 MuJoCo 窗口取得键盘焦点，双臂向前水平伸直，按 **C** 并稳定保持两秒；
终端显示 `calibration SUCCESS` 后按 **S**。
机器人参考姿态为左右臂均 **J2=-90°、其他轴为 0°** 的模型 TCP。
各侧仅校准 X/Z 平移，不改 Y、姿态和骨长，不使用双臂最小 X 替代。
标定失败不允许首次接管；C/S 不会直接向硬件驱动发送命令。

关闭窗口或 Ctrl-C 直接退出本仿真，不自动回位；Q 不承担回 Home 退出功能。

### 仿真 H 回 Home（mapped-palm 专用）

**H → 停止跟随 → 静止 → 平滑回 Home → 到位且静止 → 自动 R → 用户 S 接管**。
Home 来自本次启动 controller 配置中的初始关节姿态，不是 C 标定的水平参考姿态。
保留已有 X/Z 标定偏移。回位使用五次插值，计划速度不超过 0.35 rad/s、
计划加速度不超过 0.5 rad/s²；到位要求双臂误差不超过 0.01 rad，
速度不超过 0.03 rad/s 并持续 0.3 秒，再提交实测 q/qdot 重新准备。

- 回位/重新准备过程中拒绝 C/S/R，重复 H 不重启轨迹；P 可中止并保持。
- 收到原生 R 成功确认后显示等待 S；绝不自动接管。S 仍检查新输入与来源身份。
- 回位期间屏蔽跟随帧，手部保持。恢复前也屏蔽旧缓存命令。
- 回位 60 秒未完成或确认超时则保持；输入不足导致 R 拒绝时，恢复输入后手动 R→S。
- dynamics 可能因重力或接触无法满足实测到位条件，不会伪造到位。
- 仅仿真调度新增该流程，不修改真机多次 Enter 流程，也不是完整复制源工程状态机。

### 仿真原地保持与恢复（新入口）

点击 MuJoCo 窗口后操作：

- **P**：停止跟随并保持最后目标，等待模型静止；不是回 Home。
- **R**：父进程确认仿真关节速度连续 0.3 秒不超过 0.03 rad/s，再通过独占 stdin
  管道提交实际 q/qdot。原生控制器检查反馈时效、关节限位及新鲜 PICO 输入，
  重置 IK 内部状态，但仍不允许输出。等待终端显示 `R accepted at measured rest`。
- **S**：必须有 R 后的新输入，且 epoch/流代次不变，才重新接管。
- tracking reset 自动保持后也使用 R→S；直接 S 不会清除锁存。
- 仿真中同一 tracking epoch 下的流重新同步（例如快速转腕触发姿态跳变门控），
  在接收端确认后由 IK 重建参考并继续跟随，不再要求 R→S。实际 epoch 改变或
  按键暂停/恢复事件仍走锁存保护。比较的是 IK 已应用 epoch，不是最新收到的 epoch。
  显式 P/H 保持不会被流重新同步解除；真机及未启用 simulation-recovery 的路径不变。
- X/Z 模式保留偏移。需要重新 C 时先 P→静止→R，再 C 标定成功后 S。
  追踪重定位可能改变物理空间关系，应重新检查映射，必要时重做 C。

仿真恢复开关只由 `sim/run_sim.py` 为 mapped-palm 子进程提供；SPARK 和真机入口
不启用，不使用虚构反馈授权物理机器人。P/追踪锁存期间联合手部 ready 也关闭。
短时输入失鲜、无 tracking reset 时仍沿用原有新鲜度策略。

触发保持时原生适配器输出 `MAPPED_RESET_DIAG`：epoch、流代次、按键动作
（0=无、1=暂停、2=恢复）、事件及锚点帧号、接受帧间隔、左右掌心跳变米数和弧度。
跳变量相对于接收端上次接受的掌心，并非相邻原始采样。事件元数据保留到后续帧，
避免 latest-only 交换丢掉诊断；按键单独触发时该元数据可能属于更早的事件，需核对帧号。
这是进程内诊断字段，不改变 TJVR/TJRC 线协议或门控阈值；旧进程需重启加载。
`hold_required=0` 表示同 epoch 重同步未锁存，`hold_required=1` 表示需要手动恢复。

停止 PICO 托管输入使用原脚本：

```bash
bash tracking/scripts/start_tianji_pico_teleop.sh --stop
```

MuJoCo 目标标记显示 C++ 实际 IK 目标（独立只读 MPT1 UDP），失鲜时隐藏。
不是静态模型标记，也不是机器人命令通道；尚未迁入完整人体骨架连线显示。

## 与源工程效果的边界

1. 原生 worker 同一组 100 帧合成 TJVR 输入的 q/qdot/qddot 和目标位姿，
   已与来源二进制逐帧对照到小数点后 9 位。但这不替代长录制回放及真实输入验收。
2. 本工程保留动力学 MuJoCo 仿真，不是源工程直接设置模型关节位置的显示方式，
   重力、伺服和碰撞可能导致可视化末端误差，不能都归因于 IK。
3. 人员标定保留目标工程原文件。源机器上已验证的**对称骨长属于外部配置**，
   不能从 Git 推断已迁入；本轮没有覆盖人员配置。现已接入自动生成对称快照，见下节。
   两边必须使用经确认的等价 TCP/腕心/骨长标定才能比较现场映射效果。
4. 真机接线可选择新后端，但尚未真实输入仿真验收、尚未真机运动验收。
   真机仍使用本工程降速与安全流程，不保证与仿真运行速度相同。
5. 录制保留目标工程现有方案，未增加源工程的 HDF5/Zenoh session 等价入口。

## 自动生成对称骨长配置

标定入口仍为本工程原有入口，例如（真实标定时才执行）：

```bash
"$TIANJI_PYTHON" -m tianji calibrate left geometry --user YOUR_USER
"$TIANJI_PYTHON" -m tianji calibrate right geometry --user YOUR_USER
```

默认 `--geometry-policy symmetric_max`。每侧骨长候选通过验证并激活后，
当左右两套 TCP/腕心/骨长齐全且校验通过时，自动创建：

```text
人员标定版本/
  pico_left_arm_geometry.yaml        原始测量，不改写
  pico_right_arm_geometry.yaml       原始测量，不改写
  symmetric_profiles/sym-唯一版本/   六份原始文件副本＋对称策略及指纹
  runtime_symmetric.json             指向快照的相对路径
```

上臂和前臂分别取左右最大值，仅在 M0 运行时替换两个长度；肩部锚点、TCP、腕心、
测量质量证据不变。这是明确的运行策略，不表示重新测得该长度，也不保证较大值必然正确。
人员草稿发布时目录会改名，所以本工程使用相对路径 JSON，而非源工程的符号链接。

缺少另一侧文件时保留本次原始结果，提示等待双侧齐全，不生成快照。
双侧校验或生成失败时返回非零（标定入口为 3），不更新旧快照；原始结果保留在草稿。
后续 TCP、腕心或骨长变化后，选择旧快照会因指纹不匹配被拒绝，不静默使用过期数据。
只需原始独立骨长时，标定命令追加 `--geometry-policy original`。

**自动生成不等于自动启用。** 后端选择不自动改变人员骨长策略，需显式选择新快照：

```bash
bash scripts/start_mapped_palm_pico.sh --user YOUR_USER --symmetric-geometry
```

已有完整人员配置也可离线生成，不必重新测量（命令会新增该目录中的快照和指针）：

```bash
PICO_PROFILE_DIR=$("$TIANJI_PYTHON" -m tianji profile --user YOUR_USER --component pico)
"$TIANJI_PYTHON" scripts/pico_symmetric_profile.py --source "$PICO_PROFILE_DIR"
```

运行时会打印 `Explicit symmetric_max runtime geometry`。没有新选项时，启动仍选择
原始配置；不会覆盖系统 `~/.config/pico_tracker`。本轮没有为任何现有人员实际生成快照。

首次更新后，需按本工程原流程重建 PICO ROS 包，以安装新增运行时校验模块：

```bash
bash tracking/scripts/build.sh --all
```

## 离线验证

后续全段真实录制测试见 [真实 TJVR 对照报告](mapped-palm-real-trace-review.md)：
24,033 周期核心数值一致。该报告记录了修复前 epoch 改变后锁存 ready=false 的现象；
现已新增上面的仿真 P/R/S 恢复及 H 回 Home 流程；不等于真机验收通过。

```bash
pixi run ctest --test-dir control/build-mapped-palm --output-on-failure
pixi run python -m unittest discover -s control/mapped_palm/tests
pixi run python -m unittest discover -s sim/tests
pixi run python -m unittest discover -s real_robot/tests
TIANJI_PYTHON="$PWD/.pixi/envs/default/bin/python" \
  bash scripts/run_mapped_palm_sim.sh --headless --duration 2 \
  --pico-port 25431 --hand-port 25432
```

对照测试需显式设置 `MAPPED_PALM_REFERENCE_ROOT` 指向来源 checkout（仅离线测试读取），
未设置时该项跳过。以上测试使用合成回环数据和模拟硬件，不启动 SDK/ADB/真实机器人。

本轮结果：原生模型 CTest 1/1，原有仿真测试 6/6，原有真机层离线测试 152/152；
新后端测试 7/7（启用来源对照），覆盖参考一致性、TJRC 失鲜/epoch 门控、C/S 标定和目标观察协议。
头部/控制器现场输入、Manus 实物及真机性能仍需分别验收。

对称配置迁移额外验证：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pixi run python -m pytest -q \
  tests/test_symmetric_profile.py tests/test_teleop_profile.py \
  tracking/src/pico_bridge/test/test_pico_calibration_artifact.py \
  tracking/src/pico_bridge/test/test_pico_calibration_menu.py \
  tracking/src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q \
  tracking/src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py
```

本轮分别 89/89、42/42 通过。节点测试使用本机 ROS Humble 对应的系统 Python 3.10；
Pixi Python 3.11 无法加载其 rclpy 扩展。测试关闭无关插件自动加载；目标 Pixi 环境补装
项目 test extra 所需的 pytest 8.4.2。未执行 ROS 包重建或真实设备标定/遥操。
