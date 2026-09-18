# 天机遥操数据采集

PICO2 裸手已有独立[仿真入口 `pico2_sim.sh`](pico2_hands/README.md)，
已通过合成输入／假 TCP 测试，用户已进行真实 PICO 输入仿真并录制；
跟踪抖动仍是已知限制，不能视为性能验收完成；不支持真机。
现有 VR、Manus、外骨骼入口及默认算法不变。
详见[实施方案](docs/pico2-hands-design-and-plan.md)。

2026-09-15 已合入 mapped-palm 路线及退出清理修复，保留本工程原生外骨骼输入。
合并范围、当前环境和验证限制见 [合并记录](docs/merge-mapped-palm-2026-09-15.md)。

以 **人员标定 → PICO／Manus 输入 → C++ 重定向与控制 → 安全执行 → 数据落盘** 为主链。
标定、Hand2 retargeting、双臂控制、模型和必要的原生依赖源码均位于本仓库，不需要克隆其他业务仓库或初始化子模块。
硬件 SDK、ROS、MuJoCo、Pinocchio 等通用库仍须安装；厂商二进制和私钥不由重构替代。

> **三种模式必须区分，且不能同时占用相同输入端口。**
> - `bash teleop.sh --sim`：默认直接显示双臂＋双 Hand2 的期望关节角，不经过 PD、动力学积分或重力下垂；添加 `--simulation-mode dynamics` 可切换为动力学仿真。
> - `bash teleop.sh --real`：双臂＋双 Hand2 真机执行与实测／目标双模型窗口；三次 Enter 分别授权慢速对齐、实时遥操、回 HOME 后失能。直接运行 `real_robot/run_teleop.py` 不带使能参数仍为 dry-run。
> - `bash teleop.sh --data --task TASK`：在相同真机安全门控下采集数据，默认写入根 `dataset/`；用 `--dataset PATH` 指定目录。
> - 当前真机支持双臂与左右两只 Hand2；`all` 表示双臂＋双手，`hands` 表示仅双手。
> - 已完成离线测试及设备身份读取；双手执行路径需重新做只读预检，尚未验证带运动的实机闭环。

## 共用 Home 姿态

部署入口的双臂 Home 统一读取 [`control/mapped_palm/config/home.yaml`](control/mapped_palm/config/home.yaml)；
按每侧 Joint1～Joint7 排列，文件存储弧度，注释标明对应角度：

- 左臂：`[55, -65, -70, -60, 60, 0, 0]°`
- 右臂：`[-55, -65, 70, -60, -60, 0, 0]°`

SPARK／mapped-palm 默认遥操模型、Mocap 回放模型初始化和真机受保护回位共用此文件。
控制器 YAML 的 `controller.home_config`、真机配置的 `staged_motion.home_config`
均相对各自配置文件解析。修改 Home 后重启入口；不需要修改 URDF 关节零位。
显式模型起始关节与真机实测接管仍优先使用完整双臂关节向量，不能把 Home 冒充实测姿态；
离线算法基准配置保持各自实验起点。
`h5-real`／`regrind-real` 从编码器实测姿态初始化并自动定位，不要求先到 Home，也不自动回 Home。

只读检查：`bash home.sh --dry-run`。**不带 `--dry-run` 的 `bash home.sh` 会授权双臂真机回位**，
不是配置查看命令。

## Mocap 轨迹回放与 RL policy runtime

`mocap_policy_runtime/` 从 dexhand_deploy 移入数据读取、轨迹回放、Regrind 推理及
Motive 接入，运行时不导入或依赖原工程目录。采集仍属于 `mocap` 工程；
模型和录制数据通过显式路径读取，不复制进代码仓库，也不修改原始文件。
新增入口为 `.venv/bin/tianji mocap`，不改变现有 PICO／Manus／外骨骼入口。

源入口已在本项目提供对应的 Pixi task：

| 源工程入口 | 本项目入口 | 行为 |
| --- | --- | --- |
| `h5_sim` | `pixi run h5_sim -- --h5 FILE` | 实时 Motive Home 对齐、Enter 保压、H5 仿真回放 |
| `h5_real` | `pixi run h5_real -- --h5 FILE --confirm-real` | 自动定位；Enter 使能、按住接近 frame0、新按 Enter 回放 |
| `rl_infer` | `pixi run rl_infer --model MODEL --h5 REFERENCE` | 离线推理测速 |
| `rl_live_infer` | `pixi run rl_live_infer --model MODEL --reference REFERENCE` | 实时 Motive 影子推理，不发送控制 |
| `regrind_real` | `pixi run regrind_real --model MODEL --reference REFERENCE --confirm-real` | 同一 Enter 操作流程，使用实测反馈策略推理 |
| `regrind_hand_sim` | `pixi run regrind_hand_sim --reference REFERENCE` | 参考手／锤子回放，不运行 policy 或物理闭环 |

对应 Python 子命令为 `h5-sim`、`h5-real`、`infer`、`live`、
`regrind-real`、`regrind-hand-sim`，例如 `.venv/bin/tianji mocap h5-sim --h5 FILE`。
Pixi task 使用本项目 `.venv`，不启动源工程的 coordinator/executor，也不访问源工程源码。

### 安装与构建

在本项目完成基础安装后，安装可选依赖和本地 TCP worker：

```bash
# CPU 环境可先执行此行；需要 GPU 时改用下述 CUDA wheel。
.venv/bin/python -m pip install torch==2.10.0 --index-url https://download.pytorch.org/whl/cpu
pixi run .venv/bin/python -m pip install -e '.[mocap-policy]'
pixi run configure
pixi run cmake --build control/build --target mocap_tcp_worker --parallel 4
.venv/bin/tianji mocap --help
```

CUDA 可用时可安装官方 `https://download.pytorch.org/whl/cu128` 的同版本 PyTorch，
并使用 `--device cuda`；显式 CUDA 不可用时会报错，不静默降级。
`--device auto` 在 CUDA 可用时选择 GPU，否则使用 CPU。
推理计时包含动作同步拷回 CPU；不保证小网络用 GPU 更快。

### 输入格式与回放

| 格式 | 读取内容 | 回放语义 |
| --- | --- | --- |
| `acquisition` | mocap-acquisition v4/v5 手腕、关键点、可选 Wuji 关节及物体 | 相邻有效帧插值；无效侧不伪造 tracking |
| `regrind` | `regrind_retargeting_root_*`、`regrind_retargeting_joints`、`object_*` | 原始参考四元数 wxyz，输出转为 xyzw；默认 50 Hz |
| `session` | dexhand session 1.0/1.1/1.2 的 target 或 joint/command | 按录制时间零阶保持；joint 不重新求 IK |
| `data_collection` | 本工程 schema-v1 的 arms/hands 实测关节 | 必须 `--mode joint`，不是已录制 command；不读取图像 |

```bash
# 只读检查，自动识别格式；未知 schema 不猜测。
.venv/bin/tianji mocap inspect /absolute/path/to/take.h5

# 通用离线工具：采集轨迹 → 原生 IK / 手部 retarget → MuJoCo。
# 此入口没有实时 Motive Home 对齐，不是下面的 h5_sim。
.venv/bin/tianji mocap replay /absolute/path/to/take.h5

# Regrind 参考默认只显示手与锤子；不加载 policy，不做动力学。
.venv/bin/tianji mocap replay /absolute/path/to/reference.h5

# 显式把参考轨迹送入机器人 IK；仍不是 policy 闭环。
.venv/bin/tianji mocap run --sim --trajectory /absolute/path/to/reference.h5

# 旧 session 目标的 TCP 约定必须显式指定：原 flange 或 hand 腕心。
.venv/bin/tianji mocap replay /absolute/path/to/session.h5 --mode target --legacy-tcp flange
.venv/bin/tianji mocap replay /absolute/path/to/session.h5 --mode joint

# 本工程的数据集需要祖先目录中的 dataset_config.json 校验关节顺序。
.venv/bin/tianji mocap replay /absolute/path/to/episode.h5 --mode joint
```

`replay` 默认打开窗口；Space 暂停/继续，关闭窗口退出。自动化使用
`--headless --no-realtime`；`--duration SECONDS` 限制墙钟运行时间，
`--speed` 控制回放速度，`--loop` 循环，`--snapshot PATH.png` 输出末帧图像。
无显示服务器时可设置 `MUJOCO_GL=egl` 使用离屏渲染。
`--reference-only` 只显示采集手/物体；`--robot` 明确选择机器人回放。
机器人默认 `direct` 直接显示关节状态，`--simulation-mode dynamics` 复用已有
MuJoCo 执行器；两者都不执行 RL，也不宣称完成了手与锤子的物理闭环。

### H5 仿真交互回放：原始采集与 Regrind IK

先在 `mocap` 工程启动 router 与 Motive 发布，再在本项目执行：

```bash
RAW=/home/current/Documents/regrind_20260829/20260826/20260826_163712_837567_take001.h5
IK=/home/current/Documents/regrind_20260829/ik_data/trajectories/hammer__20260826_163712_837567_take001.h5
OBJECT_MESH=/home/current/Documents/objects/hammer_m.obj
ENDPOINT=tcp/127.0.0.1:7447

# 原始采集：腕部跟随，手部关键点预计算重定向为 Wuji 关节角。
.venv/bin/tianji mocap h5-sim --h5 "$RAW" --object-mesh "$OBJECT_MESH" --endpoint "$ENDPOINT" --speed 1

# 停止上一个仿真后再运行：已经过 Regrind IK 求解的腕部／手部轨迹。
.venv/bin/tianji mocap h5-sim --h5 "$IK" --object-mesh "$OBJECT_MESH" --endpoint "$ENDPOINT" --speed 1
```

仅使用右腕 `right_wrist`；H5 回放不要求 `hammer` 存在。按名称解析刚体，
不硬编码 ID；通过 `--wrist-name` 选择其他名称。当前已校准安装外参用于
rigid→marker→mount→wrist；`--replay-config` 可覆盖回放参数及安装外参。
仿真和真机窗口均显示橙色独立 `DATA target` 右手，以及绿色物体 `TARGET`、
紫色物体 `REAL/LIVE` 和各自 RGB 位姿坐标轴。仿真实体颜色机器人是仿真状态，不是真机反馈；
紫色物体来自实时 Motive，不是模拟物理结果。期望手／物体在接近和等待装载时保持 frame0，
运行时跟随实际采用的数据帧；松键暂停只冻结数据参考，实时物体仍继续更新。

`--object-mesh` 指定当前 hammer 的米制 OBJ，两个物体姿态复用同一网格，保留原始原点、朝向、
尺寸，并补偿 MuJoCo 编译时的网格居中／主轴变换。当前文件约为 117×205×23 mm。
它只加载到窗口／仿真显示模型，零碰撞、零质量，不改变机械臂 IK 或仿真动力学。
不传此参数时仍显示物体位姿标记／坐标轴；显式路径不存在或网格无效则报错，不静默忽略。
其他命名物体可显示数据目标，当前实时物体通道为 `hammer`。
实时 hammer 丢失／未跟踪／过期会隐藏紫色物体并标注 `UNAVAILABLE`／`UNTRACKED`／`STALE`，
不会拿绿色目标充当实测。H5 回放不因仅 hammer 缺失而拒绝运行；RL 仍要求有效实时 hammer。


1. 确保被 `right_wrist` 跟踪的实体机器人处于配置 Home，且安装外参有效；等待有效 Motive，松开 Enter 后按 `s` 锁定完整世界位姿关系。
2. 按住物理 Enter 接近 frame0，松开立即保持，不积累暂停期间的运动。
3. 到位并稳定后，完全松开 Enter，再按 `r` 装载后续轨迹。
4. 再按住 Enter 推进；`s` 取消并回 Home，回位后可重新 `s` 开始；
   `q` 回 Home 后退出，正常播完自动回 Home。

默认 `direct` 显示目标关节；`--simulation-mode dynamics` 启用本项目动力学。
`--headless` 仅关闭窗口，仍要求交互 TTY 与可读取物理 Enter 的键盘环境，
不提供自动使能／自动开始。无效或过期 Motive 输入使仿真进入 fault 并保持，
不能继续消费旧轨迹目标。动力学不能满足起点或回位容差时不会假装到位。

`returning`（正常播完、`s` 取消或 `q` 退出后的仿真回 Home）默认峰值关节速度为 **2 rad/s**，
用 `--return-speed 1` 等参数可单独调整，参数必须是正有限数。
仍使用五次平滑插值，右臂和右手共用回位进度；运动时长按最大关节行程计算，最短为 `0.5 s`，
结束后还需满足原有实测到位与稳定条件。实际墙钟耗时可能因循环速率不足而增加。
`--speed` 只控制数据回放倍率，不改变回位速度；`--return-speed` 只用于 `h5-sim`，
不修改真机 `real_robot/config.json` 中的 `0.1 rad/s` Home／frame0 接近上限。
已运行的仿真需退出并重新启动才能采用新默认值。

`h5-sim` 与 `h5-real` 自动识别以下 H5；数据语义相同，操作流程不同：仿真沿用上面的 `s/r`，
真机使用下文的自动定位与 Enter 流程，不需要 `s/r/i` 或先回 Home。

| 输入 | 机械臂目标 | 手部目标 | `hand_mode` |
| --- | --- | --- | --- |
| 原始 acquisition，无 `wuji2_joints` | Manus wrist 转为 Wuji wrist，再求 TCP／机械臂 IK | 21 点手型经现有官方 Hand2 求解器预计算为 20 关节角 | `retargeted` |
| acquisition，已有 `hands/right/wuji2_joints` | 同上 | 直接读取关节角，不重复重定向 | `canonical` |
| Regrind IK／reference | `regrind_retargeting_root_*` 已是 Wuji wrist，不再套 Manus 转换 | 直接读取 `regrind_retargeting_joints` | `regrind` |

原始采集截取右腕首末有效帧，跨无效腕部间隙插值；重定向只在启动时按有效帧顺序求解，
不在 200 Hz 控制循环重复运行。求解失败会报告源帧并拒绝启动，不以零手型或旧解冒充成功。
两种执行端均检查整条准备好的手部轨迹是否有限且满足关节限位，真机在设备连接前完成检查。
Regrind 默认 50 Hz，`--speed` 仅控制轨迹时间推进。原始输入保留真实采集骨架；
IK 输入没有原始关键点时不伪造骨架，改为显示独立期望 Wuji 手型及物体目标／实测位姿。
输入 H5 只读，不回写重定向结果。以上是轨迹回放，不运行 RL policy。

骨架和控制共用完整 Home 位姿标定：`T_robot_mocap = T_robot_home_wrist × inverse(T_mocap_home_wrist)`。
实时腕部先经过 rigid→marker→mount→wrist 安装外参，不能把原始刚体朝向直接当作腕部朝向。
按 `s` 前预览按实体机器人处于 Home 的假设定位，按 `s` 后固定变换，直到回位重置。
只有显式提供已标定的 `motive_to_robot_quaternion_xyzw` 时才固定世界旋转、仅由 Home 解平移；
默认不再假设机器人与 Motive 世界轴重合。
原生 IK 异常会透传命令、序号及原始错误；若进程直接退出，则报告可获取的退出码／信号。
窗口显示 `Mocap O` 和红／绿／蓝 XYZ 正轴（各长 0.2 m）；这是实际动捕世界原点
映射到仿真后的坐标，不是腕部原点或屏幕角标。原点在视野外时需要缩放／平移视角。

### Regrind 推理与只读实时检查

```bash
MODEL=/home/current/Documents/regrind_20260829/policy/model_5750.pt
REFERENCE=/home/current/Documents/regrind_20260829/ik_data/trajectories/hammer__20260826_163712_837567_take001.h5

# 原始参考第零帧测速，不连接 Motive 或机器人。
.venv/bin/tianji mocap infer --policy regrind --model "$MODEL" \
  --h5 "$REFERENCE" --device cpu --iterations 1000

# 需要 mocap 项目的 router 与 right_wrist / hammer 实时刚体流。
# 只订阅，输出 JSON；手部 observation 明确采用上一 policy target 假设。
.venv/bin/tianji mocap live --policy regrind --model "$MODEL" \
  --reference "$REFERENCE" --endpoint tcp/127.0.0.1:7447 --device auto

# 不加载模型：锤子起始放置检查，或只读参考/实时对齐窗口。
.venv/bin/tianji mocap live --reference "$REFERENCE" --preflight-only
.venv/bin/tianji mocap live --reference "$REFERENCE" --viewer
```

策略固定 50 Hz，`--reference-speed` 在 `(0,1]` 内改变每 tick 的参考游标增量，
不改变策略频率；推理使用整数参考帧，不是回放显示时的插值。
网络观测只订阅 `mocap/rigid_body_names` 与 `mocap/hands/frame`。
viewer 中绿色锤子为参考、橙色为实时，实时手腕仅画坐标轴，不冒充手部实测关节。
锤子 mesh 若存在，从 reference 包的 `../objects/hammer/visual.obj` 读取；
缺少 mesh 时显示物体坐标轴，不伪造物体形状。

Motive 末端刚体使用 `right_wrist`／`left_wrist` 名称，消费端解析实时名称到 ID 的映射；
必须对应机器人自身右／左臂，不能根据面对机器人时的观察者左右命名，也不固定 ID 数字。
Regrind 仍只使用右腕；右侧 GL `0 -4 4` mm、GO `-1 -12 0` deg
已同步到 live 默认外参和 `mocap_policy_runtime/configs/regrind.yaml`，
保留原 marker→mount→wrist 链。左侧 GL `1 -5 3` mm、GO `3 18 -2` deg
保存在 `../mocap/config/object_offsets.yaml`，不自动套用右侧安装外参。
两侧均为机器人末端定位来源，不加入 mocap 人体采集物体列表。
原始流不预先应用 GL/GO；接口与组合顺序见
[`../mocap/docs/RIGID_SUBSCRIBE.md`](../mocap/docs/RIGID_SUBSCRIBE.md)。

### 受保护的真机接入

**下面的入口能够使能真实机器人，不能与任何其他执行器同时运行。**
先完成目标项目设备预检、Home/外参/关节顺序检查并准备物理急停。
迁移后的机械臂使用目标项目现有 `DualArmController` 的 200 Hz 原生 IK；
不携带旧 Zenoh coordinator/executor，不声称与旧机械臂命令逐帧相同。

```bash
RAW=/home/current/Documents/regrind_20260829/20260826/20260826_163712_837567_take001.h5
IK=/home/current/Documents/regrind_20260829/ik_data/trajectories/hammer__20260826_163712_837567_take001.h5
MODEL=/home/current/Documents/regrind_20260829/policy/model_5750.pt
OBJECT_MESH=/home/current/Documents/objects/hammer_m.obj
ENDPOINT=tcp/127.0.0.1:7447

# 先做只读设备预检。下面三种执行命令只能择一运行。
.venv/bin/python real_robot/run_teleop.py --devices all --inspect

# 1. 原始数据：腕部跟随＋手部重定向。
.venv/bin/tianji mocap h5-real --h5 "$RAW" --object-mesh "$OBJECT_MESH" --endpoint "$ENDPOINT" --speed 1 --confirm-real

# 2. Regrind IK 数据：已求解的腕部／手部轨迹直接回放。
.venv/bin/tianji mocap h5-real --h5 "$IK" --object-mesh "$OBJECT_MESH" --endpoint "$ENDPOINT" --speed 1 --confirm-real

# 3. RL policy：实测反馈推理，不是 IK 轨迹逐帧回放；还需要实时 hammer。
.venv/bin/tianji mocap regrind-real --model "$MODEL" --reference "$IK" \
  --object-mesh "$OBJECT_MESH" --endpoint "$ENDPOINT" --device auto --confirm-real
```

缺少 `--confirm-real` 或非 TTY 时不会连接设备。带 `--confirm-real` 启动后自动进入只读设备连接，
不再要求输入 `ENABLE REAL`；`READ-ONLY CONNECT arms` 是连接日志，不是输入提示。
设备连接不会自动使能或运行，后续仍须 Enter 运动授权。
真机默认打开只读监视窗口，区分机器人状态和数据／物体参考：

- 实体颜色机器人：SDK 返回的实测关节状态。
- 青色半透明机器人（`CONTROL target`）：当前控制阶段的目标，可能仍在限速接近数据目标。
- 橙色独立右手（`DATA target`）：数据的期望腕部位姿和 Wuji joints，不依赖机械臂 IK 是否已到位。
- 绿色物体（`TARGET`）：当前数据参考帧的物体位姿；RL 取同一参考索引，不把实测物体当目标。
- 紫色物体（`REAL/LIVE`）：最新有效 Motive 物体位姿；独立于轨迹暂停，按源时间检查新鲜度。

收到新鲜编码器和 `right_wrist` 后自动建立 `T_robot_mocap = T_robot_wrist × inverse(T_mocap_wrist)`，
无需 Home 假设或手动 `s`。使能前再次用当前实测状态核对／建立坐标关系，运动中不跟随指令重拟合。
窗口把整个实测／控制机器人变换到动捕世界，数据期望手与目标／实测物体也在同一动捕世界显示；
Motive 原点位于显示坐标的零点。此变换仅用于显示，控制仍使用原有机器人坐标和安全边界。
尚未收到有效输入时不伪造定位。接近 frame0、READY 等待期间，数据参考保持 `t=0`；
RL 同样先显示参考起点，运行后才显示策略输出。窗口标注执行阶段、数据帧号和时间。
过期快照隐藏实测机器人和实测物体，并明确标记保留的控制／数据目标为 STALE；
即使快照仍新鲜，物体源时间过期也会单独隐藏实测物体。
关闭窗口或反馈／输入失效会停止执行，不强行 Home。

窗口子进程持有渲染线程，退出时等待 GL 清理完成，避免 daemon 渲染线程与解释器退出竞争。
此处隔离使用 MuJoCo 的内部 passive launcher（公开 `launch_passive` 不提供可等待的线程）；
不支持该接口的版本会明确拒绝启动，不降级为无监视窗口的真机执行。

### 真机操作：只使用 Enter 与 q

原始 H5、Regrind IK H5、RL policy 都使用同一流程；**在启动命令的终端操作 Enter**：

1. 启动后只读连接设备，等待实测静止和 Motive 定位稳定；`REAL PLANNING` 在不运动的情况下求解 frame0，收敛后锁定关节目标并显示。
2. 看到 `REAL ENABLE`，**按一次 Enter，仅使能**。这次按键即使一直按住也不会直接开始接近。
3. 使能完成后**先松开，再按住 Enter**，机械臂和手部沿固定关节目标平滑靠近。松开后进入 `REAL BRAKING` 限加速度减速，**仍会有少量运动**；停下后显示 `REAL HOLD`。重新按住时先确认实测静止，再从保持指令重新规划，不跳到编码器位置或补跑旧轨迹。
4. 腕部位姿和手指实测到位并稳定后显示 `REAL READY`。**先松开，再新按一次 Enter** 开始运行。
   RL 在此仍检查锤子起始放置；不满足时提示偏差，调整后重新按 Enter。
5. 运行阶段改为 **Enter 暂停／继续**，不必一直按住；松键本身不暂停。暂停时数据时间、参考和控制历史冻结，
   恢复不补跑暂停期间的轨迹。只有新的物理按键边沿生效，键盘自动重复不会触发阶段跳转。

`q`、Ctrl+C、窗口关闭、故障或正常播放完成均停止并失能，**不自动回 Home**。
失能可能失去支撑，应确保机械臂和抓取物安全。`s/r/i` 不再控制这些真机入口。
启动时已按住 Enter 必须先释放；缺少可用物理键态后端则拒绝启动，不用终端重复字符猜测松键。
仍保留设备身份、关节限位、限速、实测跟踪、输入新鲜度和策略模型白名单等检查。
只执行右臂／右手，左臂保持使能时的实测姿态；不强制右手先归零。

接近期间不再每拍追逐新的 IK 解。用独立实测 FK 与实时 Motive 检查锁定定位关系，
允许输入时间差范围内的正常腕部运动；持续不一致会撤销 READY，进入 `REAL RELOCALIZING`，
先减速保持，再等待实测静止、定位稳定并重新求解。新目标锁定后仍须**重新松开并按住 Enter**，
不会自动追逐新目标。重定位／求解受超时保护；输入失鲜、故障与退出仍直接走原安全停止流程，不等待正常减速。

Home 与 frame0 接近共用关节空间五次轨迹 `s(u)=10u³−15u⁴+6u⁵`：
双臂 14 关节共用进度，每只手单独共用进度，不要求手与臂同一时刻到达。
每组中需要运动的关节同步完成指令轨迹；实物是否到位仍由反馈整定确认。
时长同时满足速度和加速度峰值，不使用简单的“行程／速度”作为五次轨迹时长。
`staged_motion.maximum_speed_rad_s` 当前为 `0.1`，新增 `maximum_acceleration_rad_s2` 当前为 `0.2`；
该加速度是待实机验收的指令限制，不是厂商认证的实际制动性能。
正常暂停保持指令速度连续，但切换减速时加速度可以突变，不承诺 jerk 限制。
在当前限制下，正常暂停的指令曲线最多需要约 `0.5 s`、额外约 `0.025 rad` 停下；
物理运动还受跟随误差与驱动器影响，不能把此数值当作安全停距。
慢速轨迹按实际单调时间采样，超出执行 watchdog 间隔则停止；
实时遥操、H5／policy 运行阶段保留原有逐关节限速与时间基准。
这不是碰撞规划或末端直线运动，必须人工确认整条接近路径无障碍。

RL policy 在使能后受保护地切换右臂 SDK joint impedance（state 3），用于接近／推理阶段；
切换后重新读取实测 hold，重建限速种子与时钟，不把慢速 SDK 调用期间的时间计入运动。
SDK 的模式、K/D、工具参数回显均受检查；
不支持或回显不符会拒绝，不静默退化为普通 position 回放。
源兼容参数显式列在 `configs/regrind.yaml`：右手 MIT kp=3、kd=0.05，
电流上限取 profile 与目标硬件配置的较小值；右臂 K/D 和工具参数同样显式配置。
启动会打印所选参数，不改写 `real_robot/config.json`，设备身份及更严格安全边界保留。
这些参数仍须现场确认，代码与模拟反馈验证不等于实机动作验收。

SHA256 白名单和 Motive/臂/手新鲜度、时间偏差、起始放置和手跟踪误差门禁位于
`mocap_policy_runtime/configs/regrind.yaml`；不可为绕过预检随意修改。
真机 TCP 在各侧 Base 坐标系中保留原工作空间软区、线/角速度与加速度整形，
按 200 Hz IK tick 执行；Regrind 上限仍为 0.09 m/s、0.3875 rad/s、
0.875 m/s²、2.25 rad/s²，采集回放参数在 `configs/replay.yaml`。
暂停时同时冻结已下发关节命令并重置 IK/限幅器历史，恢复不积累隐藏运动。
当前选定 checkpoint 是 `regrind_20260829/policy/model_5750.pt`（SHA256 前缀 `c2373a187e82`），
已替换原 `model_6000.pt` 白名单；其他模型仍可离线/影子推理，但不会自动获得真机权限。
权重选择及离线推理通过不代表实机运动已验收，其他安全门禁保持不变。
legacy session 的 target 回放保持 simulation-only；真机 joint 回放还要求录制的
`source_type=joint_replay`。全部无效的侧不启动，运行中有效侧失踪则停止。

模块边界：`data/` 处理格式，`replay/` 处理时钟和可视化，
`policies/regrind/` 独占 123 维 observation / 26 维 residual action 语义，
`integration/` 连接本工程 IK、仿真和安全执行层。后续 policy 放在独立子包，
不必采用 Regrind 的观测或动作维度。

迁移验证：实际 take003 在 0°／25° yaw 下共 242 个采样点与源 H5 实现对照，
位置最大差约 5.8e-17 m，姿态最大差约 2.7e-7 rad（本项目小角度四元数插值近似）。
实际 `model_5750.pt` 在四个参考帧的 123 维观测、26 维动作及残差转换对照通过，
CPU 动作输出最大差为 0。另已运行实际 NativeIK＋MuJoCo 的
Home/s/Enter/r/暂停恢复/回位/退出场景、原点坐标轴离屏渲染、
完整参考手／锤子回放、合成 Motive 影子推理，以及模拟设备反馈下
H5／Regrind 状态机与故障清理回归。**未连接真实设备，未做带运动真机验收，
未验证现场键盘／Motive 联调，也未验证当前环境的 CUDA 推理。**

## PICO＋VR 手柄：mapped-palm IK＋对称骨架启动

**仅运行 `bash teleop.sh --sim` 或 `bash teleop.sh --real`，仍默认使用 SPARK，
不会自动改用 mapped-palm，也不会替 PICO 输入端选择对称骨长。** 两项需分别选择：

| 选择项 | 入口与参数 | 不选择时 |
| --- | --- | --- |
| 左右对称骨长 | `start_mapped_palm_pico.sh --symmetric-geometry` | 使用人员原始骨长 |
| 新 mapped-palm IK | `teleop.sh --ik-backend mapped-palm` | 使用 SPARK |
| 现场末端 X/Z 对齐 | `teleop.sh --mapped-palm-xz-calibration` | 不要求 C，使用原映射 |

骨长标定默认 `symmetric_max`：左右完整 TCP/腕心/骨长均通过校验后，自动生成对称快照，
上臂、前臂分别取左右较大值；不改写原始测量。**自动生成不等于自动启用。**
C 标定则是人体末端与机器人末端的 X/Z 对齐，不改变骨长，也不修改 Y 或姿态。

### 1. 环境和构建

在本工程根目录操作，各终端均设置已配置的 Python 环境，例如：

```bash
export TIANJI_PYTHON="$PWD/.pixi/envs/default/bin/python"
```

首次安装先按下文安装流程准备依赖；代码更新后构建新后端及 PICO ROS 包：

```bash
bash scripts/build_mapped_palm.sh
bash tracking/scripts/build.sh --all
```

### 2. 终端一：启动对称骨架输入

使用 VR 手柄路线的头显 APK。`YOUR_USER` 必须替换成实际佩戴者的已发布档案名。
切换前先退出旧执行器及旧 PICO 输入会话，不能重复启动或同时占用端口。

```bash
bash pico.sh --list-users
bash scripts/start_mapped_palm_pico.sh --user YOUR_USER --symmetric-geometry
```

此入口选择对称快照，bridge 的 PICO 世界 X 偏移为 +0.20 m。
确认日志出现 `Explicit symmetric_max runtime geometry`；未生成或已过期的快照会被拒绝，
不静默退回原配置。已有完整标定但没有快照时，可先离线生成（会新增快照与指针）：

```bash
PICO_PROFILE_DIR=$("$TIANJI_PYTHON" -m tianji profile --user YOUR_USER --component pico)
"$TIANJI_PYTHON" scripts/pico_symmetric_profile.py --source "$PICO_PROFILE_DIR"
```

若需 A 键地面初始化，应在真机未使能前完成；执行中不要重置 PICO 坐标系。
保持 PICO 输入运行，再从下面两种执行模式中选择一种。

### 3A. 终端二：仿真（不连接真机）

```bash
bash teleop.sh --sim \
  --ik-backend mapped-palm \
  --mapped-palm-xz-calibration \
  --simulation-mode direct
```

在 **MuJoCo 窗口**按 C，双臂向前水平伸直并稳定两秒；标定成功后按 S 接管。
H 停止跟随并回配置 Home，到位静止且重新准备完成后可按 S 恢复；保留手动 R。
回位期间拒绝 S，不自动接管。direct 显示期望关节角，不经过 PD/动力学；
没有 Manus 输入时可先只测试双臂，仿真手部保持目标。

### 3B. 终端二：真机仅双臂（无需 Manus）

先停止仿真执行器，核对设备 IP、左右臂身份、人员标定并做只读预检：

```bash
"$TIANJI_PYTHON" -m tianji real --devices arms --inspect
```

预检失败不要继续。确认活动范围安全、负载妥善处理、硬件急停可用后单独执行：

```bash
bash teleop.sh --real \
  --devices arms \
  --ik-backend mapped-palm \
  --mapped-palm-xz-calibration \
  --mapped-palm-resync-policy stop
```

在 **启动终端**按 C（不用回车），水平前伸稳定两秒，确认标定成功；
第一次 Enter 锁定标定并慢速对齐，等 READY 后第二次 Enter 才开始遥操。
C 本身不使能、不驱动真机。TELEOP 中第三次 Enter 慢速回 Home，到位后失能退出；
对齐/回位期间 Enter 或 Ctrl+C 直接停止并失能，不追加回位。失能可能失去支撑。
真机这条入口使用 C/Enter，不能照搬仿真的 S/H/R；`--devices arms` 不启用真实灵巧手。

本目录尚需现场验收，不能把合并前来源目录的成功测试当作当前目录的验收结果。
更多边界见 [真机说明](docs/mapped-palm-real-readiness.md)，骨长策略与异常处理见
[对称骨长说明](docs/mapped-palm-port.md#自动生成对称骨长配置)。

## 目录

| 目录 | 用途 |
| --- | --- |
| `tracking/` | PICO 驱动、人体标定、修正骨架和双臂目标 UDP 桥 |
| `control/` | 本项目双臂控制算法、MuJoCo 模型与 Viewer |
| `manus/` | Manus 原始骨架采集、21 点语义适配及手部链路启动器 |
| `retargeting/` | Hand2 重定向；C++17 分析目标函数／梯度，Python 优化器编排 |
| `real_robot/` | 真机 SDK、只读设备识别、执行器与安全配置 |
| `sim/` | 仿真专用执行器、54 关节 MuJoCo 动力学及物理行为测试 |
| `profiles/` | 按人员保存的档案与完整 PICO 标定版本，不包含机器人硬件配置 |
| `data_collection/` | 有界采样、HDF5 段生命周期、离线压缩与可视化 |
| `tianji/` | 统一 CLI 和可移植数据路径策略 |
| `scripts/` | 根环境选择；不再依赖各子工程的虚拟环境 |
| `control/third_party/ruckig/` | 固定版本离线轨迹生成源码及许可证 |

### C++ 与 Python 边界

- PICO／IMU 桥、Manus 原始采集、双臂 IK／QP／轨迹限制和 UDP 控制协议在 C++ 中运行。
- Hand2 分析目标函数和梯度位于 `retargeting/wuji_retargeting/native.cpp`，由根安装编译；
  每次优化迭代借用连续 float64 数组，三维临时量在栈上，缺少扩展时不静默退回 Python。
- Pinocchio 已有原生 FK／Jacobian 保留；双臂迭代去掉重复 FK，左右臂输入构造合并但安全提交和回退顺序不变。
- TJRC／TJVR／TJH2 共用 constexpr CRC-32 表，协议布局、校验范围和多项式不变。
- Python 保留标定交互、配置、SDK 编排、安全状态机和数据段生命周期；NumPy、h5py／HDF5、OpenCV 本身调用原生库。
  不为了换语言重写这些边界，也不改变 schema-v1、人工授权或过期输入拒绝策略。

本次离线微基准：左右 Hand2 实际模型共 240 组状态（含 alpha 端点、thumb mask、正则和耦合项），
新旧目标函数及梯度一致；含 FK 的目标函数约 **255 μs → 90–91 μs，2.8×**。
CRC 对 0–656 字节随机数据及标准校验向量一致，360／464／652 字节包约 **4.3×**。
这是局部计算测量，不代表真机采集帧率或完整闭环延迟提高同样倍数。

## 通用准备

所有命令从仓库根目录开始。Python 采用 **editable 安装**，资源、SDK、标定和控制器仍从完整 checkout 读取，不是可脱离源码目录分发的独立 wheel。

### 精简副本：一键安装与编译

本仓库保留精简副本的源码、模型、厂商 SDK 和现有标定，不包含原仓库历史、旧工程、
Python／Pixi 环境、构建产物、日志、录制数据及 Odin 私钥。
Manus SDK 动态库使用 Git LFS 管理；克隆时请先安装 Git LFS，再获取真实库文件：

```bash
git lfs install
git clone git@github.com:suzhelearning/tianji_teleop.git
cd tianji_teleop
git lfs pull
```

如果使用的是已经包含真实 SDK 动态库的精简压缩包，则无需执行 Git LFS 命令。

支持 Linux x86_64，首次安装需要联网。先按
[Pixi 官方安装说明](https://pixi.sh/latest/installation/) 安装 Pixi。
Ubuntu／Debian 系统依赖由管理员安装一次：

```bash
sudo apt-get update
sudo apt-get install build-essential libudev1 libusb-1.0-0 zlib1g adb tmux
```

然后在本目录执行：

```bash
bash install.sh --check  # 只检查基础前置条件，不下载、不编译
bash install.sh          # 安装依赖并编译全部基础遥操组件
```

脚本使用根 `pixi.lock` 安装默认和 tracking 环境，创建根 Python 3.11 `.venv`，安装 Python
依赖和原生扩展，编译双臂控制器、Manus 采集器及默认 tracking ROS 包；另外使用
`exoskeleton/pixi.lock` 安装仓库内独立的 Python 3.12 外骨骼环境并编译其 C++ 数值扩展。
统一使用副本内的 tracking ROS 环境，不复用原电脑的环境路径。
Pixi 依赖受锁文件约束；Python pip 依赖仍按 `pyproject.toml` 的版本范围解析。
失败时立即停止，修复报错后可重新运行。脚本不会自动调用 sudo，不启动或使能硬件。

请为依赖、下载缓存和构建产物预留充足磁盘空间，不能只按源码包大小预留。
安装后不要移动或删除本目录，因为 Python 使用 editable 安装。
USB 权限、设备网络地址及个人标定仍须按后续文档配置；已有标定不适用于任意新操作者。
默认不构建需要另行提供私钥的可选 Odin 组件。

已有主项目环境，仅补装外骨骼输入时执行 `bash install.sh --exoskeleton`。
该分支安装 `exoskeleton/.pixi/envs/default` 并重建本项目外骨骼扩展，不重装主环境、不编译控制器／Manus／ROS，也不启动硬件。编译需要 `/usr/bin/gcc`、`/usr/bin/g++`（Ubuntu/Debian 的 `build-essential`）。

交付验证：复制文件已进行 SHA-256 对比；安装脚本基础预检及 Manus 原生编译通过。
受交付机器磁盘空间限制，未在本副本中执行完整依赖安装及双臂／tracking 全量构建。

### 手动分步安装

```bash
# 原生编译依赖；不获取其他 Git 仓库。
pixi install
pixi run python -m venv .venv
.venv/bin/python -m pip install -e '.[collection,retargeting,test]'
pixi run configure
pixi run build
pixi run build-manus
```

PICO、Manus 与相机发布器需要与 Python ABI 匹配的 ROS 2。已有系统 ROS 时设置 `ROS_SETUP` 为其 `setup.bash`；
否则安装根清单的 tracking 工具链。两个工具链都固定 Python 3.11，复用上面创建的 `.venv`，不要重新创建：

```bash
pixi install -e tracking
.venv/bin/python -m pip install -e '.[collection,retargeting,tracking,test]'
export ROS_SETUP="$PWD/.pixi/envs/tracking/setup.bash"
pixi run -e tracking bash tracking/scripts/build.sh
```

系统还需 `adb`、`tmux`、USB／网络权限。Manus SDK 动态库通过本仓库 LFS 提供；Odin 的可选私钥独立配置。
主项目运行器使用 `.venv/bin/python`，可显式设置 `TIANJI_PYTHON`；外骨骼入口 `exo.sh` 使用仓库内 `exoskeleton/` 的锁定 Pixi Python 3.12 环境。两者均不搜索外部旧工程或自动回退到旧环境。
Tracking 启动器同时加载所选 ROS SDK 的 `lib/` 共享库目录，普通 `bash pico.sh --user NAME` 即可加载 GLFW／MuJoCo，无需先进入 `pixi shell`。

```bash
.venv/bin/tianji --help
.venv/bin/tianji profile --list-users
# PICO、Manus 分别启动，用户必须与实际佩戴者一致。
bash pico.sh --user syz
bash manus.sh --user syz
# 在独立交互终端选择以下一种模式，不要同时运行。
bash teleop.sh --sim
bash teleop.sh --real
bash teleop.sh --data --task pick_hammer --dataset /data/tianji
# 离线工具无需机器人或输入设备。
.venv/bin/tianji compress /data/tianji /data/tianji_jpeg50
.venv/bin/tianji visualize /data/tianji_jpeg50
```

`real`／`home` CLI 默认不使能，须明确传入 `--confirm-real`；便捷 shell 入口沿用既有显式授权语义。
`collect` 入口请求真机采集，但仍受 TTY、设备、相机预检和人工 Enter 门控，不能无人值守使能。

- 首次克隆后运行 `git lfs install` 和 `git lfs pull`，获取由 Git LFS 管理的 Manus SDK 动态库。
- 可选 Odin Lite SDK 的 `tracking/src/odin/odin-sdk2/sdk/utils/http/certs/certs.h` 含 mTLS 私钥，不纳入版本控制；
  `tracking/scripts/build.sh --all` 构建该 SDK 时需要通过受控渠道配置此文件。PICO／Manus 基础遥操不使用它。

真机运行前置条件（本次离线验证不代表已连接硬件）：

- 已完成 PICO、双臂控制工程和 Manus 采集器的安装、构建。
- PICO 已通过 USB 连接并授权，头显动作流应用在前台，追踪设备正常。
- 双臂遥操前，左右侧 **TCP → 手腕 → 骨长** 标定均有效；首次标定命令见文末。
- 手部遥操前，Manus 手套已开机、配对，且选择实际佩戴者对应的标定用户。
- 图形仿真需在本机桌面终端运行。

默认通道：

| 通道 | 默认设置 |
| --- | --- |
| ROS 2 | Domain `120` |
| PICO 双臂输入 | UDP `127.0.0.1:15000` |
| Hand2 手部输入 | UDP `127.0.0.1:16000` |
| 控制器 → 真机执行器 | UDP `127.0.0.1:17000`，仅 loopback |

**同一时刻只保留一套 PICO 驱动、一套 Manus 采集链路和一个机器人控制器。**
仿真 Viewer 与真机执行器内部的控制器不能同时占用相同输入端口。

## RealSense RGB 相机

`bash realsense.sh` 只读取相机，固定请求 **1280×720、30 FPS、RGB8**；
不启用深度流，不发布彩深外参或相机到机器人基座的 TF，也不连接机械臂／灵巧手。
相机或连接方式不支持该模式时由 SDK 报错，不自动降分辨率。

启动器默认使用根 `.venv/bin/python`；该环境需要
`rclpy`、`sensor_msgs`、`numpy`、`cv2` 和 `pyrealsense2`，并预先加载上述 ROS 环境。
也可通过 `REALSENSE_PYTHON` 指定显式配置的兼容解释器：

```bash
bash realsense.sh

# 不打开预览窗口，仅发布 ROS 图像和内参。
bash realsense.sh --no-preview
```

默认 ROS Domain 为 `120`。发布 `/realsense/color/image_raw`（`rgb8`）和
`/realsense/color/camera_info`，QoS 为 Best Effort、队列深度 1。
内参对应当前分辨率，时间戳来自已同步主机时钟的相机采集时间；重复帧不重复发布。
D405 的 `realsense_inverse_brown_conrady` 畸变系数不能直接作为 OpenCV `plumb_bob` 使用。
多相机时使用 `--serial SN`；预览中按 `Q`、`Esc`、关闭窗口或 `Ctrl+C` 退出。
此入口只预览／发布，不会自动保存完整 RGB 数据集。

## 观测数据集采集（R / S / D）

采集契约见 [schema-v1.md](schema-v1.md)。只保存实际双臂关节位置（14 维）、
实际双手关节位置（40 维）和 RGB；**不保存 actions 或控制目标**。
状态以 120 Hz 为读取目标，每路 RGB 配置为 1280×720@30 FPS；采集直接保存未压缩 uint8 RGB 像素，不进行 JPEG 编码。
各流使用独立、以 episode 起点为零的主机单调时间戳；实际采样频率以文件时间戳为准。
训练时的时间偏移和 state/action 配对由训练端自行完成。

当前 `collection_config.json` 的设备对应：

| 视角 | 序列号 |
| --- | --- |
| top | `409122274035` |
| left_wrist | `409122274621` |
| right_wrist | `409122273692` |

先启动所需的 PICO 和 Manus 输入，停止独立的 `realsense.sh` 或其他占用这三台相机的程序。
本入口沿用真机设备连接和安全门槛，不会绕过使能授权：

```bash
bash collection.sh --task pick_hammer

# 可选：保存到指定的数据集目录。
bash collection.sh --task pick_hammer --dataset /data/pick_hammer
```

默认输出到根目录 `dataset/`（可用 `TIANJI_DATASET` 或 `--dataset` 覆盖），仍在 `logs/real/` 保留运行日志。按原来的 Enter 流程完成对齐并进入
TELEOP；**进入 TELEOP 不自动录制**。在启动命令的终端直接按单键，不要追加 Enter：

| 按键 | 数据操作 | 对机器人模式的影响 |
| --- | --- | --- |
| `r` | 开始新的一段，等待 `DATASET RECORDING` 提示 | 保持 TELEOP |
| `s` | 结束当前段，后台保存并校验；操作者确认成功，`success=true` | 保持 TELEOP |
| `d` | 丢弃当前录制段；不会删除以前已保存的数据 | 保持 TELEOP |

看到 `DATASET SAVED` / `DATASET DISCARDED` 后可再次按 `r`；保存/准备尚未完成时不会叠加另一段。
原有 Enter（进入 HOME）、Ctrl+C 和关闭机器人窗口的停止行为不变。
离开 TELEOP 或采集故障时，未按 `s` 的当前段保留 `.partial.h5`，不会自动当作成功样本。
采集故障只结束数据段并报告；机器人自身的反馈、限位、输入新鲜度等保护仍独立生效。
相机或采样线程已终止时不能直接恢复录制，先排除故障并重启采集会话。
录制中即使采样线程仍存活，关节反馈/发布超过 300 ms 或图像发布超过 2 s 未更新也会判为采集故障；
结束当前段时再次检查并冻结该段状态。按 `s` 截止后的相机故障不否决已结束的正常录制，
但截止前的采集故障及该段 HDF5 写入/校验错误仍会阻止成功发布。
写入者关闭超时会进入 `FAILED`，不允许再次按 `r`，退出时重试清理；
只有确认关闭后才释放文件管理权。初始化超时或中断后，阻塞的文件操作一旦返回，
后台写入者会关闭并清理未开始录制的文件；无法强制中断尚未返回的文件系统调用。

目录和文件名沿用 mocap 的 `YYYYMMDD/YYYYMMDD_HHMMSS_ffffff_takeNNN.h5`，
例如 `20260913/20260913_143025_123456_take001.h5`。使用本地时间、六位微秒和进程内递增的 take 序号；
时间在文件准备时生成（第一段在硬件连接前预留），丢弃或失败不会回收序号。
完整文件包含 `observations/arms`、`observations/hands` 和
`images/top`、`images/left_wrist`、`images/right_wrist`，各相机包含 `timestamp_ns` 和 `rgb[N,720,1280,3]`。单个后台 HDF5 写入者处理有界队列和校验，
控制线程不等待写盘；保存失败保持 `.partial.h5`，不覆盖旧文件。
`dataset_config.json` 仍只写在数据集根目录；关节/相机/编码契约不匹配时拒绝向该数据集追加。
已有 JPEG 数据集保留原有契约和数据，不覆盖配置、不补造相机图片。
原始 RGB 采集请使用新默认目录或另一个空目录，不向旧 JPEG 数据集追加。
三路 30 Hz 原始图像约 **249 MB/s、14.9 GB/分钟**，需要充足的持续写入带宽和磁盘空间；名义帧率不是实测保证。

采集、压缩和浏览使用统一根 `.venv`，安装方法见通用准备，不再维护独立采集 requirements 或子工程环境。
`COLLECTION_PYTHON` 仍可显式指定兼容解释器。默认 `dataset/` 和 `dataset_jpeg50/` 已忽略；
原始图像数据建议通过 `--dataset` 放在容量充足的数据盘，备份由操作者管理。

### 相机实时预览（不录制）

在桌面终端运行：

```bash
bash visualize_oneline.sh
```

读取 `collection_config.json` 中启用的相机，按 top、left_wrist、right_wrist 顺序在一个窗口中并排显示，
标注序列号和最近约 2 秒的帧率：`RX` 为后台接收帧率，`UI` 为提交到窗口的新画面帧率（不是显示器的物理刷新率）。
采集 RGB8 1280×720@30，预览缩小显示；不写数据文件、不启动 ROS、不连接机器人。
每台相机使用独立阻塞取帧线程，窗口只取最新画面，不积压旧帧；窗口变慢时允许 `UI` 降低，不让绘图拖慢取帧。
实时预览不编码 JPEG；采集保存原始 RGB，离线压缩脚本另生成 JPEG 质量 50 的副本。
窗口内按 `Q` / `Esc`、关闭窗口或终端 `Ctrl+C` 退出并释放相机。
**不能与 `collection.sh`、`realsense.sh` 或其他占用相机的程序同时运行；开始正式采集前先退出预览。**
需要桌面显示环境和 `.venv` 依赖；支持 `COLLECTION_PYTHON` 和 `--config <配置路径>`。
缺少相机、打开失败或画面连续超过 2 秒不更新时会报错退出，不把冻结画面当作正常实时流。

### 离线批量压缩（固定 JPEG Q50）

采集并保存若干段后，定期手动运行：

```bash
bash compress_data.sh
```

默认读取 `dataset`，输出到 `dataset_jpeg50`。
**压缩质量固定为 50；RGB 或已有 JPEG 源文件均不删除、不覆盖。** JPEG 输入先解码再编码为 Q50，会产生额外有损损失，程序启动时会提示源质量。保持 episode 相对路径、所有关节值、时间戳、任务和成功标记。
脚本跳过 `.partial.h5` 与符号链接；已完成输出通过源身份和 JPEG 格式校验后跳过，之后新增的 episode 会在下次运行时处理。
中断或失败不会发布半成品为完成文件；可重复运行继续，结束时显示 `converted/skipped/failed`，失败返回非零退出码。
不要手动改写已压缩文件的源文件；身份不匹配或目标配置冲突时脚本拒绝覆盖。

可显式指定原始目录和独立输出目录：

```bash
bash compress_data.sh /data/tianji_raw /data/tianji_jpeg50
bash visualize_data.sh dataset_jpeg50
```

按日期处理 `$HOME/Documents/TianjiData` 中的数据：

```bash
bash compress.sh --date 20260914
# 输出到 $HOME/Documents/TianjiData/20260914_compressed；不传 --date 时使用当天日期。
```

源/目标不能相同或互相包含。建议在暂停录制时压缩，避免读盘、写盘和编码争抢采集资源。
压缩会额外占用空间，不会释放原始数据占用的空间；磁盘不足时先将原始数据备份到更大磁盘。

### 数据集可视化（只读）

从项目根目录启动本地网页查看器，不需要启动 PICO、Manus、机器人或相机：

```bash
bash visualize_data.sh dataset
```

浏览器打开 `http://127.0.0.1:8765/`。也可传入整个数据集目录或单个 `.h5` 文件；
端口占用时加 `--port 8766`，使用终端打印的地址。终端 `Ctrl+C` 关闭查看器。
不传目录时默认查看原始 RGB 数据集；旧 JPEG 数据和 Q50 输出也可显式传目录查看。
相机面板标明 RGB/JPEG。原始 RGB 仅在网页传输时转为无损 PNG，不改动磁盘文件；JPEG 图片直接读取已有编码字节。

- 选择 episode，播放／暂停、调速、拖动时间轴，或跳到上一／下一条实际采样时间。
- 同屏查看相机 RGB、双臂 14 维和双手 40 维实测关节轨迹；曲线可勾选，读数支持 rad／度。
- 每条流按自身时间戳取“不晚于当前时间的最后一条记录”，显示样本时间与年龄；
  不插值、不显示未来帧，也不把保留的最后观测当作新测量。
  相机加载新帧期间保留上一张已解码且不晚于当前时刻的画面，解码完成后替换；
  时间标签对应实际显示的帧，加载较慢时年龄会增大，不每帧清空画面。
- 文件属性与统计显示任务、成功状态、各流样本数、实际平均频率和最大采样间隔。
- `.partial.h5` 始终标为未完成、非成功确认。可读取已关闭的部分数据；
  正在写入、损坏或不兼容的文件会明确报错，不绕过 HDF5 文件锁。

查看器只监听本机，使用现有 `.venv` 环境，不修改文件、不连接硬件，
也不展示数据集中未记录的控制指令。关节名称从上级 `dataset_config.json` 读取。

## 人员档案与标定版本

从工作区根目录显式选择实际佩戴者：

```bash
bash pico.sh --list-users
bash pico.sh --user syz

# 另一个终端使用同一人员档案启动手部输入。
bash manus.sh --user syz
```

两个入口都要求 `--user NAME`，不默认选择 syz，不支持 `--syz`。
统一由 `teleop_profile.py` 解析档案；人员选择与 `--view/--sim/--real` 模式、硬件 IP/SN 分开。

当前档案 `profiles/syz/profile.yaml`：

```yaml
schema_version: 1
user_id: syz
pico:
  active_set: '20260908-01'
manus:
  user: syz
```

PICO 从 `profiles/syz/pico/20260908-01/` 加载左右各 3 个文件：
`pico_{left,right}_{palm_tcp,wrist_pivot,arm_geometry}.yaml`。
本版本原样保存已确认属于 syz 的当前标定；原 `~/.config/pico_tracker/` 文件保留且未改写。
Manus 复用 `manus/calibration/` 下对应用户的左右 `.mcal`，不复制第二套；
`manus.user` 可以与人员 ID 不同。

按人启动 PICO 时，两侧 TCP、手腕和骨长依赖链都必须通过现有 artifact 校验。
缺失、损坏、身份不符、路径越界或哈希不匹配时，在停止旧会话前退出；
不回退到全局标定、其他版本、其他人员或 SMPL baseline。
选中的版本绝对路径显式传到 tmux 中的 M0 进程，不依赖 tmux 的历史环境变量。
Manus 与 PICO 独立校验：仅使用双手时，不要求 PICO 标定可用，反之亦然。

### 添加人员或更新标定

新人员可以直接在标定命令中指定人员 ID，无需手工复制文件或创建档案。先按附录启动
标定用 PICO 驱动并完成地面初始化，然后执行（以 `zjx` 为例）：

```bash
cd tracking

bash scripts/calibrate_pico_arm.sh left all --user zjx &&
bash scripts/calibrate_pico_arm.sh right all --user zjx
```

- `--user NAME` 可以放在侧别／操作之前或之后；同一命令不能指定两个人员。
- 过程中的有效结果保存到 `profiles/zjx/pico/.draft/`，第二条命令继续使用该草稿。
- 新人员完成左侧后显示 `Calibration pending`，不创建可用的 `profile.yaml`；
  右侧完成、左右 6 个文件及依赖链全部有效后，显示 `Published profile`。
- 发布时自动生成 `profiles/zjx/pico/cal-<唯一ID>/`，并原子更新
  `profiles/zjx/profile.yaml` 的 `pico.active_set`，不覆盖旧版本或全局标定。
- 原始采集保存在 `profiles/zjx/recordings/`。该路径不会随草稿发布改变，
  保持 artifact 内部的 `source_recording` 路径有效。
- 失败或中断返回非零状态，保留草稿，不发布、不继续下一阶段。
  修正后可重跑相应单项，例如 `left wrist --user zjx`。
- 已有人员首次创建更新草稿时只从自己的有效活动版本复制，不读取全局文件。
  因此更新已有人员的一侧后，如果另一侧旧标定仍匹配，可直接发布一套完整新版本。
- 同一人员不能并发运行两个标定命令。

完成后只读检查与启动：

```bash
bash scripts/calibrate_pico_arm.sh status --user zjx

cd /path/to/tianji_teleop
bash pico.sh --user zjx
```

`status --user` 只检查已发布版本，不会创建草稿；首次仅完成一侧时会报告尚无已发布档案。
新档案默认将 `manus.user` 设为相同人员 ID，但不会生成 Manus 标定。
仍需准备 `manus/calibration/zjxLeftMetaglovePro.mcal` 和对应的 Right 文件后，
才能执行 `bash manus.sh --user zjx`。已有人员的 Manus 映射在发布时保持不变。

不传 `--user` 的旧标定入口仍写入 `~/.config/pico_tracker/`；
多人适配必须显式指定人员，不能把旧全局结果当作新人员的草稿。

运行中的会话固定使用启动时选定的版本，不因修改 `active_set` 热切换。
直接调用 PICO 子工程的旧脚本且不传 `--calibration-dir` 仍保留旧全局加载行为；
多人遥操应使用这里的根目录人员入口。

---

## 一、仿真遥操作

本节只控制 MuJoCo，不连接真实机械臂或灵巧手。

### 1. 启动 PICO 双臂输入

若完整 PICO 链路已经正常运行，跳过此步。
如果仍有单独运行的 `start_pico_driver.sh`，先在其终端按 `Ctrl+C` 停止，避免重复连接头显。

终端 1：

```bash
cd /path/to/tianji_teleop
bash pico.sh --user syz
```

该脚本启动驱动、修正骨架、人体骨架 Viewer 和 UDP 桥。
戴好设备、双脚站稳，按一次右手柄 **A 键**，等待日志出现：

```text
Canonical SMPL floor locked ...
```

查看后台窗口：

```bash
cd tracking
tmux attach -t pico_tianji_teleop
```

- `Ctrl-b`，松开后按 `0`：驱动窗口。
- `Ctrl-b`，松开后按 `1`：修正骨架窗口。
- `Ctrl-b`，松开后按 `2`：UDP 桥窗口。
- **`Ctrl-b`，松开后按 `d`：退出界面，后台继续运行。**

不要用 `Ctrl+C` 代替退出 tmux；它会停止窗口中的程序。
使用同一环境的 `tmux` 客户端与服务端。

### 2. 启动 Manus 灵巧手输入

终端 2：

```bash
cd /path/to/tianji_teleop

# 查看已登记的人员档案。
bash manus.sh --list-users

# 选择实际佩戴者，档案会映射到已有的 Manus 标定用户。
bash manus.sh --user syz
```

该命令统一管理 Manus 采集器、ROS `/hand_input` 适配器及 Hand2 UDP 桥。
保持终端运行，不要另开第二个采集器。

#### 外骨骼手套输入（替代 Manus）

外骨骼源代码、设备／零位档案、手套与 Wuji 模型、官方重定向求解器已纳入本项目 `exoskeleton/`，不再依赖外部 `~/syz/data_glove_wuji_teleop`。终端 2 直接运行发送器；不需要额外桥接终端：

```bash
# 首次使用，或更新 C++ 内核后重新编译安装：
bash install.sh --exoskeleton

# 只检查本地身份档案、零位、方向、FK/MANO 和模型，不连接设备：
bash exo.sh --check-config

# 采集、重定向并直接向本机控制器发送 TJH2：
bash exo.sh
# 等价入口：.venv/bin/tianji exoskeleton <相同参数>
```

默认启动双手；只启动左手用 `bash exo.sh --hand left`，只启动右手用 `bash exo.sh --hand right`。
入口默认允许采集发送和未验收方向调试，不需要追加 `--confirm-send` 或 `--commission-directions`。
当前迁入档案的方向尚未验收；默认调试许可不会修改验收标记，也不代表真机动作已验证。
`exo.sh` 只管理发送器，不启动 PICO、仿真或真机。
缺少内部环境时会提示安装命令，不借用原工程环境。

后续设备与零位配置修改在 `exoskeleton/config/dataglove/devices/`，任务绑定在
`exoskeleton/config/teleoperation/`。传给 `exo.sh` 的相对配置路径以 `exoskeleton/` 为基准；
也可通过 `pixi run --manifest-path exoskeleton/pixi.toml --locked <任务>` 使用迁入的注册、标定等工具。
这是独立的仓库内源代码副本，原工程未删除，但两处代码和标定不会自动同步。
原有录制数据、虚拟环境、缓存和未使用的 STEP/USD 资源未迁入；部署所需的 URDF/MJCF/STL 已保留。
第三方模型与求解器许可证随资源保留；原手套 URDF 的 BSD 许可声明不完整，不能将整个目录视为统一 MIT 授权。

外骨骼每帧的数值内核已迁入 `exoskeleton/native/`：

- `encoder_kinematics.cpp`：四连杆几何求解和 21 通道机构换算，缓存机构参数，保留无解／退化检查。
- `morphology.cpp`：独立 MANO 手型的前向几何、残差／解析雅可比、初值选择、有界迭代、时序项和捏合二次拟合。固定大小工作数组；拟合期间释放 GIL，每个实例独立保护并原子提交历史状态。
- `module.cpp`：通过 pybind11 暴露给现有 Python 接口；不保留旧 Python 数值循环或缺少扩展时的回退路径。

这是数值内核迁移，不是整套程序 C++ 化：配置与标定、设备采集、软件零位和 URDF 映射调度仍在 Python，
MuJoCo FK 继续使用现有原生库，官方 Wuji 求解 worker 与 TJH2 发送路径保持不变。
`setup.py` 使用 C++17、`-O3`、`-ffp-contract=off`，不启用 fast-math。
运行 `bash install.sh --exoskeleton` 会按锁文件安装依赖并重建本地 editable 包，避免仅修改 C++ 后误用旧扩展；
运行中的进程不会热更新，重新启动 `exo.sh` 后使用新内核。

不要同时启动 `manus.sh`。`pico.sh`、`teleop.sh --sim/--real` 保持不变。
旧版 JSON UDP 中转已移除；切换前停止旧发送器和旧桥接进程。外骨骼与 Manus 共用协议编码器，但外骨骼直发不依赖 ROS 或 Manus 标定。

发送器默认向 `127.0.0.1:16000` 发送 364 字节的 TJH2 v2 二进制包，含 CRC32、
递增序号、发布时间和每侧原始测量时间；每侧 20 个有限弧度角。
关节顺序以 `tianji/hand_protocol.py::JOINT_STEMS` 为准：
拇指、食指、中指、无名指、小指，每指 4 个关节。发送官方 Wuji 模型坐标下的 qpos 并重排关节顺序，不额外裁剪或拒绝有限的越界值；真机安全仍由执行端负责。

只有新求解结果触发发送，不再增加 100 Hz 中转重发。未更新侧可以随包携带缓存姿态，
但保留该侧原始帧读取完成时的 `monotonic_ns`，不能用另一侧更新或发布时间刷新其年龄。
FK／求解耗时计入帧年龄；发送器保留过期丢弃门禁，执行端继续独立检查每侧 150 ms 新鲜度。
冷启动求解超过该时限的帧也不能用于执行。故障侧清除缓存，不用零位替代；
没有新结果、空发送和退出均不补包，恢复需重启该侧采集。

这一路径要求发送器和控制器共享同机单调时钟，`--udp-host` 仅允许回环 IPv4 地址，
不能将目的地址改为另一台电脑。`--udp-port` 可显式覆盖默认端口，但必须匹配控制器配置。

使用 `bash exo.sh --help` 查看参数。先仿真逐指验证，再走原有真机安全流程。
已验证仓库内锁定环境、双手离线配置，以及合成零位帧经真实 FK／官方求解 worker／随机回环 UDP 直发，由原生控制器协议解码和每侧新鲜度逻辑检查；覆盖旧侧过期、故障缓存清除与退出不补包。此次验证没有连接实物手套、修改网络或启动真机；实际外骨骼＋PICO 带运动闭环仍需按安全流程验证。
原生内核迁移已编译安装，并通过左右手各两帧合成编码器输入运行实际机构换算／FK／手型拟合／官方求解／TJH2 UDP 链路；未运行测试套件或性能基准，不宣称具体加速倍数。

### 3. 选择参考显示或动力学仿真

先关闭旧机器人 Viewer，再在终端 3 执行：

```bash
cd /path/to/tianji_teleop

# 参考运动可视化：检查 IK / 重定向结果，不模拟受力后的运动。
.venv/bin/tianji view

# 或：默认 direct，直接显示期望关节角，不经过 PD 或动力学积分。
bash teleop.sh --sim

# 显式选择动力学积分，仍然不连接任何真机硬件。
bash teleop.sh --sim --simulation-mode dynamics
```

检查顺序：先缓慢、小幅活动双臂，再保持手腕稳定，逐个活动手指。
确认左右对应，握拳时手指朝掌心弯曲。

两者都使用上文的 PICO 和手部输入（Manus 或外骨骼二选一）。`--sim` 内部启动同一套无界面参考控制器，
通过独占的动态 loopback UDP 端口接收 TJRC v2 目标，不占用真机输出端口 `17000`。
不能再额外启动独立机器人 Viewer。没有输入时保持初始目标；某一路输入失效时保持
该路最后目标。仅 dynamics 模式继续物理积分，受力后仍可能运动；direct 模式没有重力下垂。
终端区分 `waiting`、`ready` 和 `stale-hold`。

动力学路径保留原模型惯性、重力、关节及力矩限制，增加 54 个有界位置／阻尼执行器，
以 `1 ms` 步长和 `implicitfast` 积分。只在初始化设置关节姿态，后续目标仅写入执行器。
控制器仍产生 `model_reference`，没有把模拟反馈送回 IK；反馈闭环在模拟关节伺服层。
仿真不做重力补偿，因此存在重力下垂和目标误差，这与 `--view` 的参考姿态显示不同。
可用 MuJoCo 鼠标交互调整视角；窗口默认隐藏侧栏以完整显示双臂和双手。

**物理仿真不等于经过真机辨识的数字孪生。** 增益位于 `sim/physics.py`，仅为仿真调参，
不是硬件标定值。碰撞采用原模型网格的凸包，定点排除肩部安装壳、复合腕关节及复合指根
装配内部的重叠；其余接触保持启用。原始 XML 不修改。不能据此认定真机抓取或碰撞安全。

无窗口验证使用同一动力学路径：

```bash
bash teleop.sh --sim --simulation-mode dynamics --headless --duration 10
```

结束时输出 `SIM_SUMMARY`，包括实际物理时间、各路目标误差和输入状态。
`--duration` 对有窗口与无窗口动力学均有效。关闭动力学窗口或按 `Ctrl+C` 会清理内部控制器。
统一停止整套遥操使用根目录的 `./stop.sh`：依次停止真机执行器、动力学执行器、
本项目机器人 Viewer、Manus 和受管理的 PICO 会话；不停止其他工程的同名进程。

### 4. 停止仿真

1. 关闭机器人 Viewer。
2. 在 Manus 终端按 `Ctrl+C`。
3. 停止 PICO 链路：

```bash
cd tracking
./scripts/stop_tianji_pico_teleop.sh
```

---

## 二、真机遥操作

> **先空载、低速测试。** 清空运动范围，确认硬件急停随手可按，停止其他控制同一设备的程序。
> 不要无人值守地使能，不要通过放宽限位或跳过反馈检查来解决启动失败。
> 软件 watchdog 不替代物理急停；SDK 阻塞、进程强制终止及固件行为都可能影响实际停机。

包含双臂的真机入口内部启动无界面控制器，并自动打开一个独立、只读的 MuJoCo 实测／目标窗口。
窗口必须成功打开后才连接设备；不需要也不能另开占用相同 `15000/16000` 输入端口的机器人 Viewer。
PICO 人体骨架窗口可以保留。单手／双手独立执行不包含双臂时，保留原来的无窗口启动流程。

### 1. 核对设备配置和左右手 SN

配置文件：[`real_robot/config.json`](real_robot/config.json)。
当前示例配置为：

- 双臂 IP：`192.168.1.190`。
- 左手 SN：`WH2JA01260811019`。
- 右手 SN：`WH2KA01260814006`。
- `active_devices`：双臂、左手、右手；命令行可单独选择设备。

设备更换后，应先确认实际 SN，不能按扫描顺序或 IP 猜测左右手：
左右 SN 来自已保存的 `hand_devices.json` 固件身份查询结果，连接及使能前仍会再次校验。
两侧不能配置成同一个 SN；双手共用一份 SDK 初始化，但设备句柄、反馈和故障状态独立。

本次内部目标协议升级为 **TJRC v2 / 468 字节**，包含 14 个机械臂关节和左右各 20 个手关节。
旧 v1 / 308 字节会被拒绝；升级后停止旧执行器和旧机器人 Viewer，再使用新编译的控制器与执行器。

```bash
cd /path/to/tianji_teleop

# 只扫描，不连接设备。
python3 real_robot/read_hand_serials.py

# 额外读取固件的左右手身份，随后断开，不使能电机。
python3 real_robot/read_hand_serials.py --read-handedness

# 可选：保存到新文件，不覆盖已有配置，也不自动改写控制配置。
python3 real_robot/read_hand_serials.py --read-handedness --output hand_devices.json
```

SDK 建立身份查询连接时可能进行时间同步；建议在没有真机控制会话运行时执行。

### 2. 只读设备预检

从工作区根目录执行，使用已安装的手部 Python 环境：

```bash
cd /path/to/tianji_teleop

# 左右手身份和各自 20 个关节反馈。
.venv/bin/python real_robot/run_teleop.py \
  --devices hands --inspect

# 双臂 14 个关节反馈。
.venv/bin/python real_robot/run_teleop.py \
  --devices arms --inspect
```

该模式不清错、不使能、不发送目标。检查反馈健康且持续更新，核对关节单位为 rad。
若身份错误、反馈缺失或超时，先解决问题，不进入使能步骤。
若只检查一侧，可将 `--devices hands` 改为 `--devices left_hand` 或 `--devices right_hand`。

### 3. 开启输入链路，先干跑

按仿真章节启动所需的 PICO 和 Manus 输入链路，但**不要启动机器人 Viewer**。
双臂使用 PICO；左右手使用 Manus。PICO 的 A 键复位、地面锁定和全部标定都应在真机使能前完成。

```bash
cd /path/to/tianji_teleop

# 仅双手干跑，不连接双臂硬件。
.venv/bin/python real_robot/run_teleop.py \
  --devices hands --duration 10

# 或：双臂＋双手完整干跑。
.venv/bin/python real_robot/run_teleop.py \
  --devices all --duration 10
```

不带 `--inspect` 或 `--confirm-real` 时，默认是 **DRY RUN**：不加载硬件 SDK、不连接设备、不发送硬件命令。
两条干跑命令不要同时运行。没有输入时 `flags=0` 表示未就绪；不要把它视为可以使能。

### 4. 首次使能：建议逐侧测试灵巧手

确认只读检查和输入链路正常，在交互式终端执行：

```bash
cd /path/to/tianji_teleop

.venv/bin/python real_robot/run_teleop.py \
  --devices right_hand \
  --confirm-real
```

只有满足下列条件，程序才提示按 Enter 使能：

- 所选设备 SN 正确，固件左右身份与选定侧一致。
- 20 个关节状态与诊断完整，反馈持续更新。
- 输入新鲜，目标没有越过限位。
- 本节的单手／双手独立模式使用自动回零启动，不要求手套与真机人工对齐；包含双臂时使用下一节的分阶段流程。

确认现场安全后，在提示中输入：

```text
Enter（直接按回车使能；再次按回车停止并失能）
```
预检及回车后的使能复检都会在硬件反馈读取完成后，获取最新控制包并使用当前时间检查新鲜度；
真实输入过期或反馈异常仍拒绝使能。SDK 的 `Servo error code=[0,...]` 仅表示错误码为零，
不代表已使能：应先看到 `Press ENTER to enable`，确认现场安全后按回车，再检查
`REAL OUTPUT ARMED`。未通过时查看 `NOT ENABLED:` 或 `REAL EXECUTOR STOP:` 的具体原因。
执行器连接 Marvin 后关闭本地 SDK 常规打印，避免 `SERVO... value=0` 和全零伺服码刷屏。
这不会关闭机械臂端日志，也不跳过反馈／伺服码检查；检测到的故障仍通过执行器错误信息报告。

使能后，每只手的 20 个关节从实测姿态以不超过 1.0 rad/s 的 setpoint 速度回零。
零目标已发出且各关节反馈距零不超过 0.15 rad 后，锁存此刻的 Manus 重定向帧，
用 0.5 s 线性插值从零到该帧，随后恢复实时跟随。回零超时为 5 s。
半秒插值阶段的速度由目标距离/0.5 s 决定，不受实时跟随的 1.0 rad/s 上限约束；
因此 0.5 s 是命令轨迹时长，不保证机械关节恰好在该时刻到位。
全程保留反馈、限位、跟踪误差与输入 watchdog 检查；失败则终止会话。

当前左右手均显式采用 SDK 示例增益 **kp=3.0、kd=0.05**，使能时写入所有 20 个关节。
连接阶段仍输出设备当前增益用于核对；此次读回的 12/0.3 不作为本次控制增益。
此自动回零与半秒过渡尚未验证带运动实机闭环。不要强行掰动机械手；未使能不代表可安全背驱。

右手测试完成后，先按 Ctrl+C 并确认释放，再单独测试左手：

```bash
cd /path/to/tianji_teleop
.venv/bin/python real_robot/run_teleop.py \
  --devices left_hand --confirm-real
```

两侧分别验证后，可用 `--devices hands --confirm-real` 只控制双手，不连接双臂硬件。
同一双手会话中任意一侧输入或反馈失效，都会终止整个已使能会话，不会悄悄继续另一只手。

### 5. 双臂＋双手联合真机遥操

先停止单手或双手测试会话，确认释放完成。保持 PICO、Manus 输入链路运行，再执行：

```bash
cd /path/to/tianji_teleop

bash teleop.sh --real
```

每次 `--real` 自动打印 `run log: <绝对路径>`，在
`logs/real/<时间戳>-<唯一标识>/` 保存本次运行，不覆盖上次记录：

| 文件 | 内容 |
| --- | --- |
| `console.log` | 终端 stdout/stderr，包括执行器、SDK 和子控制器输出；终端仍实时显示 |
| `launcher_exit.json` | 启动命令、起止时间、退出码、收到的信号及控制台日志是否完整 |
| `session.json` | 所选设备、真机配置快照、退出原因、结构化故障和清理错误 |
| `controller_configuration.yaml` | 本次实际使用的控制器配置，包含真机覆盖值和实测初始姿态 |
| `flight_recorder.jsonl` | 退出前最多 10 秒、最多 2000 个采样及故障现场；每行一个 JSON |

轨迹记录区分控制器输入参考 `reference`、最近成功发送给硬件的目标 `sent`、
实测 `feedback`，并带阶段、单调时钟、包序号、输入就绪标志和反馈时间。
机械臂跟踪超限还会记录具体关节（例如 `Joint3_R`）、命令角、实测角、误差、
阈值、反馈年龄和控制周期间隔；`joint_index` 从 0 开始，双臂顺序为左 7 轴、右 7 轴。
这些记录只帮助诊断，不改变速度、保护阈值、Enter 授权或停止逻辑。

高频样本只进有界内存缓冲，硬件停止／释放流程完成或报告失败后才写盘。
帮助、参数解析失败等早期退出可能只有前两个文件；控制器未启动时没有控制器配置快照。
断电、`SIGKILL` 或进程原生崩溃可能来不及保存内存轨迹，需结合已有控制台日志和
`launcher_exit.json` 判断；磁盘写入失败会明确报告，不能把不完整记录当作完整日志。
日志目录已忽略版本控制，不自动清除旧运行；需自行管理磁盘空间。
下次失败后可直接提供打印出的日志目录，无需再粘贴整段终端输出。

直接运行 `real_robot/run_teleop.py` 不会默认启用这套会话日志；
日常联合真机操作使用上述 `bash teleop.sh --real` 入口。

`all` 指双臂＋左手＋右手，三路输入和反馈均需就绪；只测试双臂可使用
`.venv/bin/python real_robot/run_teleop.py --devices arms --confirm-real`。

若显示 `NOT ENABLED: left_hand: source input is not fresh/ready`，且控制器统计
`hand_datagrams=0`，表示没有收到 Manus 重定向 UDP 输入，不是 Hand2 硬件未连接。
在独立终端运行 `bash manus.sh --user <实际佩戴者>` 并保持运行，先完成上节干跑检查。
启动时的 `Enter 1...` 是操作说明，不表示预检已通过；等待终端出现 `WAITING | 未使能 | Enter...`
后，才在启动执行器的终端按 Enter，不要在预检期间提前或连续按回车。
首次预检默认 30 秒超时，超时退出后需重新启动执行器。
非只读检查模式会先确认控制器可执行文件存在并独占绑定指令端口，再打开相机、显示窗口和硬件会话；
未编译控制器或端口被占用时，不连接设备。`--inspect` 不依赖控制器文件或指令端口。

包含双臂时采用以下流程，不再要求操作者先手动匹配每个关节：

| 当前阶段 | Enter 的作用 |
| --- | --- |
| WAITING（未使能） | 第一次确认：锁定当前目标，使能后从实测姿态慢速对齐 |
| ALIGNING（慢速对齐） | 中止，直接停止和失能，不提前开始遥操 |
| READY（已到位并保持） | 第二次确认：连续切换到实时遥操 |
| TELEOP（实时遥操） | 保持输入和机械臂静止后第三次确认：停止跟随，双臂平滑回指定 HOME |
| HOMING（回位） | 中止，直接停止和失能 |
| HOME_REACHED | 实测确认到位后自动失能、退出 |

终端按阶段显示状态及下一步操作，不重复刷关节角。未使能时始终显示 `WAITING`，
即使当前姿态已接近目标，也必须经过第一次 Enter 授权和实测稳定判定。
只有已使能且对齐完成才显示 `READY | 已对齐并保持 | Enter: 开始实时遥操`。

第一次 Enter 捕获的目标不会追逐后续输入变化；对齐完成后保持，等待第二次确认。
在包含双臂的分阶段模式下，Hand2 也从实测姿态直接慢速到达锁定目标，
不执行单独手部模式的回零／半秒插值。第二次 Enter 后，从上一条已发送指令连续限速跟随。
READY 必须同时满足指令已到目标、所有选中设备实测误差在各自 `alignment_rad` 内并持续稳定，
不是只凭模型或计时器判断；漂移会撤销 READY。
静止速度由连续实测位置与反馈时间戳估算，须不超过
`staged_motion.settle_speed_rad_s`（默认 `0.03 rad/s`），并在新反馈上持续
`settle_time_s`。重复反馈不能累计到位时间；反馈回退、过长间隔或运动会重置静止窗口。
第二次 Enter 也复查静止条件；独立回 HOME 入口使用相同判定。
回 HOME 前还要求双臂指令停止变化，不能把移动中的遥操指令直接接到零初速轨迹；
未满足此条件时请求被拒绝并走现有停止／失能流程。微小但持续变化的输入也可能阻止此切换，
此时应先安全退出遥操，释放设备后使用独立 `home.sh`，不要放宽安全阈值强行切换。
mapped-palm 可显式选择 `--mapped-palm-resync-policy bounded`，通过私有事件通道
执行同 epoch 重同步的短时保持及限速衔接；默认仍为 `stop`。
这是待实机验收的选项，不是取消失鲜/限位/反馈保护；详见
[真机执行层说明](docs/mapped-palm-real-readiness.md)。
仅新 mapped-palm 真机路线可选 `--mapped-palm-xz-calibration`：在启动终端按 **C**
（无需回车）水平前伸标定 X/Z；成功后第一次 Enter 锁定标定并慢速对齐，
到位静止后第二次 Enter 遥操。C 不使能/不驱动机械臂，执行期间禁止 C，epoch
变化后标定失效。此选项不自动开启 bounded；未启用时仍使用固定映射，SPARK 不变。
mapped-palm 双臂真机默认启用 `--mapped-palm-dropout-policy hold-300ms`：
仅 TELEOP 中的 PICO 短时断流可保持所有选中设备的最后下发目标，
在最后有效输入时间起 300 ms 内恢复同 epoch／同流代次的新有效帧后，按原执行限速继续。
不补发积压目标，重复帧不续期；超过窗口、无效包、求解失败、坐标系重置仍停机。
命令与硬件反馈看门狗、双手输入时效及关节保护不变。显式传 `--mapped-palm-dropout-policy stop`
可恢复原失鲜即停行为；它与上面的重同步策略是两个独立选项，新增恢复行为仍待实机验收。

2026-09-15 已完成一次 PICO＋VR 手柄、mapped-palm、C 标定、`stop` 策略的真实双臂
流程测试：标定 → 对齐 → 遥操 → 回 Home → 释放连接。该结果不覆盖 bounded、
Manus/Hand2 联合真机或长期稳定性；完整命令和证据见
[mapped-palm 真机说明](docs/mapped-palm-real-readiness.md)。

只读 Viewer 清理时的非零退出、ERROR 或退出超时现在会计入会话 `cleanup_errors`，
并使入口返回失败状态；硬件停止/释放先于 Viewer 清理。本修复经 170 项软件回归，
尚未重新验收实际图形退出；此前观察到的 GLFW 警告根因仍待处理。

正常回 HOME 时只移动双臂，双手保持最后指令姿态，不主动张开。
HOME 到位判定只针对双臂；双手仍须通过健康和新鲜度检查，
但不会要求受接触阻挡的手指重新满足启动对齐误差。
到位后所有选中设备失能；失能可能失去夹持力，必须先确认负载安全。
整个对齐、等待、遥操和回位阶段都保留输入新鲜度、epoch、反馈健康和限位检查。
机械臂保留跟踪误差退出；左右 Hand2 不再因运行时“反馈未跟上上一条指令”而退出，
被阻挡时也不会靠这项误差自动停止。启动对齐、单独手部模式的回零/到位超时检查仍保留；
手部 `tracking_error_rad` 仅继续用于该模式的启动 ramp 到位容差，不再作为运行时退出阈值。
回位期间也应保持 PICO／Manus 输入链路运行；断流、故障或关闭窗口时立即停止，不继续回位。

控制周期包含反馈读取、安全检查、发送和状态更新的耗时；默认 200 Hz 下，只等待本轮剩余的 5 ms 预算，
不再在工作完成后额外固定睡眠 5 ms。工作超时则开始新周期，不累计欠账、不突发补发；
原有限速、输入/反馈新鲜度及机械臂跟踪误差检查保持不变，实际频率仍受 SDK 和操作系统调度限制。

Manus 重定向结果由独立线程统一采样为双手 100 Hz；控制器以 10 ms 名义缓冲，
按发布时间戳线性插值为 200 Hz 目标，再交给真机执行器的限速和安全检查。
无插值区间时保持端点，不外推；重复目标保留各手原始输入时间，不能掩盖单手断流。
线性插值不替代速度限制，超过现有速度上限时实际指令仍会限速。
这次升级使用 TJH2 v2（364 字节）；需构建新控制器并重启 `manus.sh` 与遥操/采集进程，
旧协议不能混用。`kp/kd`、电流上限和速度上限不因插值而改变。

所有退出路径先向全部设备发出停止请求，再逐个关闭连接和释放 SDK 资源，
避免某一路的关闭等待延迟其他设备收到停止请求；单路失败仍继续处理其余设备并报告。
这不使阻塞的 SDK 停止调用变成可抢占操作，物理急停仍必须随手可及。

#### 独立双臂回 HOME（不操作 Wuji Hand）

先停止正在运行的真机遥操／采集会话，等待设备释放，再从项目根目录执行：

```bash
bash home.sh --dry-run  # 仅校验配置、显示目标，不连接硬件
bash home.sh            # 授权使能／接管双臂，慢速回 HOME 后失能
```

该入口只连接 Marvin 双臂，不连接、使能、归零或失能任何 Wuji Hand，也不需要 PICO／Manus 输入。
双臂均未使能时先使能；双臂均已使能且健康时直接接管，先以实测姿态覆盖旧目标，再慢速回位，
不会先失能再重新使能。接管后本会话负责停止和失能；普通遥操／采集仍拒绝接管已使能设备。
单臂使能、另一臂未使能的混合状态仍拒绝操作，不自动清错或修复设备状态。
HOME 由 `real_robot/config.json` 的 `staged_motion.home_config` 指向
`control/mapped_palm/config/home.yaml`，启动时加载左右臂目标。
速度取 `staged_motion.maximum_speed_rad_s` 与机械臂限速中的较小值（当前 `0.1 rad/s`），
加速度受 `staged_motion.maximum_acceleration_rad_s2` 限制（当前 `0.2 rad/s²`）。
先保持接管的实测姿态，收到新鲜且稳定的静止反馈后，再以共享进度的五次轨迹回位。
平滑起止会比原先独立匀速限幅耗时更长；若计算出的运动与整定时间无法满足原有 `60 s` 超时，
在开始轨迹前拒绝，不先移动到超时再停。实测到位并稳定后停止、失能并释放双臂；
Ctrl+C／SIGTERM 或故障则停止，不继续回位。
机械臂反馈新鲜度、健康、位置边界、跟踪误差及回位超时检查仍生效，不默认放宽限位执行越限恢复。
位置上下限检查对双臂及双手所有关节统一使用 `0.01 rad`（约 `0.573°`）余量，
目标角和实测反馈均适用；不改变模型限位、速度限制或跟踪误差阈值。
厂商 `FxRtCSDef.h` 定义 `101` 为切换到位置模式、`109` 为切换到 idle 的过渡状态。
仅在本程序发起使能后的 2 秒窗口内允许 `101`，期间仍检查反馈新鲜度、错误及位置边界，
并且必须实测双臂均到达 `1` 才能开始回位／遥操。失能必须实测到达 `0` 才算完成；
持续 `101/109` 表示切换未完成，不能当作健康运行或成功失能，也不自动清错重试。
与同配置遥操共用独占指令端口，端口被占用时在连接硬件前拒绝；不要与其他机器人控制程序同时运行。
终端日志保存在 `logs/home/`；可用 `--config PATH` 指定另一份本项目格式的配置。
这是关节空间回位，不规划避障路径；执行前清空双臂运动路径，并确保物理急停可用。

#### 真机实测与目标窗口

- 实色模型来自硬件关节反馈，只做正运动学显示，不用动力学仿真冒充真实状态。
- 青色半透明模型是当前阶段目标：WAITING／TELEOP 为当前有效输入；
  ALIGNING／READY 为第一次 Enter 锁定的目标；HOMING 为 HOME 和保持的手指姿态。
- 缺失的设备反馈隐藏为 UNKNOWN；窗口快照超过 150 ms 时显示 STALE 并隐藏实测模型，
  不把历史姿态标成当前状态。目标仍作为已标记过期的参考保留。
- 手指的空间位置依赖机械臂姿态；缺少机械臂数据时，对应实测／目标手指也隐藏为 UNKNOWN，不借用旧臂姿态。
- 绘制运行在独立进程，最新状态通过有界、非阻塞通道传递，不占用 PICO／Manus 输入端口。
- 关闭窗口或窗口进程失败会触发停止和失能，不触发 HOME。

双臂控制模型初态仍由实测关节写入临时配置，原算法 YAML 不修改，
继续使用 `model_reference` 算法。慢速对齐／回位是关节目标限速，不是笛卡尔直线路径或自动避障；
第一次 Enter 和正常回位前均需确认整条运动路径无障碍，急停可用。

#### 指定 HOME

`real_robot/config.json` 的 `staged_motion` 保存以下 HOME（各侧 Joint1→Joint7，单位 rad），
启动前会验证维数、有限值和关节限位，不会替换为全零或启动姿态：

```python
left  = ( 0.9599310886, -1.1344640138, -1.2217304764, -1.0471975512,  1.0471975512, 0.0, 0.0)
right = (-0.9599310886, -1.1344640138,  1.2217304764, -1.0471975512, -1.0471975512, 0.0, 0.0)
```

慢速对齐／回位使用共享进度五次轨迹：速度上限 `0.1 rad/s`（同时不超过各设备原有上限），
加速度上限 `0.2 rad/s²`；峰值同时受这两项约束，若无法在超时内完成则在运动前拒绝。
到位稳定时间为 `0.2 s`，每次对齐／回位超时为 `60 s`。这些值可在 `staged_motion` 中配置。

默认硬件边界限制：

| 参数 | 默认值 |
| --- | --- |
| 控制器速度比例 | `0.1` |
| Marvin 速度／加速度比例 | `10% / 10%` |
| 双臂 setpoint 最大变化速度 | `0.5 rad/s` |
| 左／右手各自 setpoint 最大变化速度 | `1.0 rad/s` |
| 命令 watchdog 阈值 | `0.15 s` |
| 双臂／双手反馈 watchdog 阈值 | `0.30 s` |

这些是设备保护限制，不替换本项目的 IK 或重定向算法。

### 6. 停止与故障处理

- **正常回位退出（包含双臂）：**在 TELEOP 阶段按 Enter，等待 HOMING、HOME_REACHED 和设备释放。
- **直接停止：**Ctrl+C、关闭窗口、`./stop.sh` 或对齐／回位中按 Enter，均停止并失能，不追加 HOME 运动。
- **异常运动、碰撞风险或软件无响应：立即按硬件急停。** 不要只等待终端退出。
- 输入断流超过允许窗口、无效输入、反馈异常、越限、跟踪误差或控制器退出会终止已使能会话。
  唯一默认断流恢复例外为 mapped-palm TELEOP 的 PICO `hold-300ms`；其余故障不自动清错恢复。
- Hand2 非零诊断码由 SDK 静态目录按严重级别解析：`Warning` 不单独触发停机，
  会输出左右手、SN、NID、故障名及厂商处理建议，并保留在反馈 `detail` 中。
  `DeferredStop`、`ImmediateStop`、`Fatal`、未知码或解码失败仍锁存停机。
  警告不能绕过反馈过期、非有限值、关节失能和跟踪误差检查。
  `Enc1BitRate`（code=3）表示编码器数据质量警告；软件分级修复不代表硬件问题已解决。
- **使能期间不要按 PICO A 键、重新标定或重启输入驱动。** reset/resync 会撤销双臂运动授权，恢复需要重新启动并预检。
- 如果输出显示清理无法确认，先确认设备已实际停止，不要直接重启控制。
- 真机停止后，再按仿真章节的停止步骤关闭 Manus 和 PICO 输入链路。

### 后续实机验证边界

离线回归与只读预检不能证明实际运动安全。首次带运动测试仍需核实：

- 设备关节方向、零位及当前安装是否与模型一致。
- Hand2 使能瞬间的目标保持行为。
- 反馈序号、时间戳持续性及固件独立断流保护。
- SDK 停止／失能与物理急停的实际响应。

如果固件不在使能前提供反馈，程序会拒绝继续，不会先使能或用零值伪造反馈。

---

## 附录：PICO 标定命令

完整顺序为每侧 **TCP → 掌心到手腕 → 上臂／前臂骨长**。
标定只在真机执行器未使能时进行。

终端 1 单独启动驱动，并保持运行；不要与完整 PICO 启动脚本重复运行：

```bash
cd tracking
bash scripts/start_pico_driver.sh
```

驱动与标定器默认均使用 `ROS_DOMAIN_ID=120`、`ROS_LOCALHOST_ONLY=1`。
驱动在加载 ROS SDK 前设置这些默认值，避免 SDK 将驱动改为非 localhost 模式。
若显式覆盖，两个终端必须保持一致；旧进程不会因脚本更新而自动改变环境。
TCP 标定订阅 `/pico/pose/{left,right}_hand` 并同时需要 `/pico/pose/head`。
出现“未收到新鲜数据”时，先核对两个进程的 ROS 域和 localhost 设置，不要直接改话题名。

按一次 A 并完成地面初始化后，在终端 2 按顺序执行，每一步成功后再继续：

```bash
cd tracking

bash scripts/calibrate_pico_arm.sh left tcp --user zjx
bash scripts/calibrate_pico_arm.sh left wrist --user zjx
bash scripts/calibrate_pico_arm.sh left geometry --user zjx

bash scripts/calibrate_pico_arm.sh right tcp --user zjx
bash scripts/calibrate_pico_arm.sh right wrist --user zjx
bash scripts/calibrate_pico_arm.sh right geometry --user zjx

bash scripts/calibrate_pico_arm.sh status --user zjx
```

也可以使用以下顺序引导入口**替代**上面六条标定命令，不要重复执行两套：

```bash
bash scripts/calibrate_pico_arm.sh left all --user zjx &&
bash scripts/calibrate_pico_arm.sh right all --user zjx
```

完整 TCP 标定包含姿态。以下旧入口仅微调**全局标定**的掌心朝向，不会更新上述人员草稿或已发布版本：

```bash
bash scripts/calibrate_pico_palm_orientation.sh left
bash scripts/calibrate_pico_palm_orientation.sh right
```

姿态微调：身体直立、头朝前，双臂向正前方水平伸直，掌心相对、手腕中立；按空格后保持约 2 秒静止。
仅微调姿态不改变 TCP 平移、手腕距离或骨长。重做完整 TCP 后，需要重做对应侧手腕和骨长；重做手腕后，需要重做对应侧骨长。

标定结束后先停止单独驱动，再切换到完整 PICO 链路。

## 进一步说明

- [mapped-palm 独立后端：构建、PICO＋VR 仿真及迁移边界](docs/mapped-palm-port.md)
- [PICO 工程中文说明](tracking/README.zh-CN.md)
- [双臂控制器与遥测说明](control/README.md)
- [Wuji 重定向说明](retargeting/README.md)
- [真机配置](real_robot/config.json)
