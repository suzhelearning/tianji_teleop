# 共用 Home 与 Mocap／Regrind

[返回遥操首页](README.md)。本页保留 Mocap／Regrind 的独立工作流，不替代 PICO 标定和 DLS 仿真流程。
当前工作区基于 `main@90c575f` 完成 protype 式目录迁移；下面使用锁定 Pixi 环境和对应安装 overlay，不从旧环境导入模块。安装、离线或合成验证不等于现场运动验收。

## 共用 Home 姿态

部署入口的双臂 Home 统一读取 [`src/teleop_outputs/tianji/tianji_description/config/home.yaml`](src/teleop_outputs/tianji/tianji_description/config/home.yaml)，运行时通过 `tianji_description` 的安装 share 资源定位；
按每侧 Joint1～Joint7 排列，文件存储弧度，注释标明对应角度：

- 左臂：`[55, -65, -70, -60, 60, 0, 0]°`
- 右臂：`[-55, -65, 70, -60, -60, 0, 0]°`

SPARK／mapped-palm 默认遥操模型、Mocap 回放模型初始化和真机受保护回位共用此文件。
控制器 YAML 的 `controller.home_config`、真机配置的 `staged_motion.home_config`
均相对各自配置文件解析。修改 Home 后重启入口；不需要修改 URDF 关节零位。
显式模型起始关节与真机实测接管仍优先使用完整双臂关节向量，不能把 Home 冒充实测姿态；
离线算法基准配置、DLS／Ceres 仍保持各自的 Home／实验起点，不受此共用部署 Home 替换。
`h5-real`／`regrind-real` 从编码器实测姿态初始化并自动定位，不要求先到 Home，也不自动回 Home。

只读检查：`bash bash/run_home.sh --dry-run`。**不带 `--dry-run` 的 `bash bash/run_home.sh` 会授权双臂真机回位**，
不是配置查看命令。

## Mocap 轨迹回放与 RL policy runtime

`src/inference/mocap_policy_runtime/mocap_policy_runtime/` 从 dexhand_deploy 移入数据读取、轨迹回放、Regrind 推理及
Motive 接入，运行时不导入或依赖原工程目录。采集仍属于 `mocap` 工程；
模型和录制数据通过显式路径读取，不复制进代码仓库，也不修改原始文件。
统一入口为已激活 workspace overlay 中的 `tianji mocap`（分发到 `bash/run_mocap.sh`），也可直接使用下面的 shell 入口，不改变现有 PICO／Manus／外骨骼入口。需要显式使用统一 CLI 时可运行 `pixi run bash -c 'source bash/environment.sh; tianji mocap --help'`。

源入口已在本项目提供对应的 Pixi task：

| 源工程入口 | 本项目入口 | 环境 | 行为 |
| --- | --- | --- | --- |
| `h5_sim` | `pixi run h5_sim --h5 FILE` | `default` | 实时 Motive Home 对齐、Enter 保压、H5 仿真回放 |
| `h5_real` | `pixi run h5_real --h5 FILE --confirm-real` | `default` | 自动定位；Enter 使能、按住接近 frame0、新按 Enter 回放 |
| `rl_infer` | `pixi run rl_infer --model MODEL --h5 REFERENCE` | `policy` | 离线推理测速 |
| `rl_live_infer` | `pixi run rl_live_infer --model MODEL --reference REFERENCE` | `policy` | 实时 Motive 影子推理，不发送控制 |
| `regrind_real` | `pixi run regrind_real --model MODEL --reference REFERENCE --confirm-real` | `policy` | 同一 Enter 操作流程，使用实测反馈策略推理 |
| `regrind_hand_sim` | `pixi run regrind_hand_sim --reference REFERENCE` | `policy` | 参考手／锤子回放，不运行 policy 或物理闭环 |

对应 Python 子命令为 `h5-sim`、`h5-real`、`infer`、`live`、
`regrind-real`、`regrind-hand-sim`，例如 `bash bash/run_mocap.sh h5-sim --h5 FILE`。
wrapper 将 `infer`／`live`／`regrind-real`／`regrind-hand-sim` 路由到 `policy`，其余命令路由到 `default`；
统一 CLI 也经过此 wrapper。每个环境只加载自己的 `install/<环境>` overlay，不注入其他环境的 Python 包，
不启动源工程的 coordinator/executor，也不访问源工程源码。

### 安装与构建

完整安装入口在裸 shell 中执行，安装所有锁定环境、构建 `default`／`policy` overlay、control 原生目标、Manus 和独立外骨骼／PICO2 扩展；不会连接或使能硬件：

```bash
bash bash/install.sh

# 已安装环境后，源码修改对应的重建入口。
pixi run build
pixi run -e policy build
bash bash/run_mocap.sh --help

# Mocap 回归任务在 policy 环境中运行；不是现场验收。
pixi run -e policy test-mocap
```

`default` 与 `policy` 使用相同 Jazzy／Python 3.12 ABI，但依赖和 overlay 分开安装。
`policy` 的可选推理依赖已纳入锁文件：CPU PyTorch `2.10.0` 与 `eclipse-zenoh`；
完整安装包含它们，不再要求另建虚拟环境或从根 `pyproject.toml` 安装 extras。模型权重、参考 H5、
物体网格、Motive router／发布端及现场键盘权限仍需显式准备，不由安装器下载或生成。

原生源位于 `src/inference/mocap_policy_runtime/mocap_policy_runtime/native/tcp_worker.cpp`；
控制工程从 `src/teleop_outputs/tianji/tianji_controller/native/` 构建，将 `mocap_tcp_worker` 安装到
`install/control/bin/`。Python 通过 `tianji_runtime.native_executable()` 定位，worker 在
`control` 环境运行，以原有 stdin/stdout 协议通信，不把 control 库导入 Python 运行环境。
机器人模型从 `tianji_description` 的安装 share 读取，源码在
`src/teleop_outputs/tianji/tianji_description/models/` 及其模型资源闭包；不要指向旧构建树里的二进制或孤立复制 XML。

原始关键点的手部重定向复用 `manus` 环境中已安装的官方 HandRetargeter；
完整安装已准备该闭包，单独重建用 `bash bash/build_manus.sh`。default／policy
通过 `manus` 的隔离 Python 子进程进行有界 stdin/stdout 请求，不在调用端加载
Pinocchio／nlopt，也不让手部 worker 运行 ROS 或发送 UDP。H5 在启动阶段按时间顺序
预计算并关闭 worker 后才进入执行／设备访问；canonical joints、joint-only 与 Regrind
输入不启动该重定向 worker。依赖或求解失败明确报错，不用零手型或 Python 数值替代。

默认 CPU lock 不是 GPU 配置。CUDA 需要另行评审并显式配置兼容的官方 wheel／锁文件、
驱动和模型；不能把 CPU 成功称为 CUDA 验收，也不要临时 pip 覆盖已锁定环境。
例如官方同版本 `https://download.pytorch.org/whl/cu128` 仍需针对部署环境验证。
显式 `--device cuda` 不可用时会报错，不静默降级；`--device auto` 在 CUDA 可用时选择 GPU，否则使用 CPU。
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
bash bash/run_mocap.sh inspect /absolute/path/to/take.h5

# 通用离线工具：采集轨迹 → 原生 IK / 手部 retarget → MuJoCo。
# 此入口没有实时 Motive Home 对齐，不是下面的 h5_sim。
bash bash/run_mocap.sh replay /absolute/path/to/take.h5

# Regrind 参考默认只显示手与锤子；不加载 policy，不做动力学。
bash bash/run_mocap.sh replay /absolute/path/to/reference.h5

# 显式把参考轨迹送入机器人 IK；仍不是 policy 闭环。
bash bash/run_mocap.sh run --sim --trajectory /absolute/path/to/reference.h5

# 旧 session 目标的 TCP 约定必须显式指定：原 flange 或 hand 腕心。
bash bash/run_mocap.sh replay /absolute/path/to/session.h5 --mode target --legacy-tcp flange
bash bash/run_mocap.sh replay /absolute/path/to/session.h5 --mode joint

# 本工程的数据集需要祖先目录中的 dataset_config.json 校验关节顺序。
bash bash/run_mocap.sh replay /absolute/path/to/episode.h5 --mode joint
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
bash bash/run_mocap.sh h5-sim --h5 "$RAW" --object-mesh "$OBJECT_MESH" --endpoint "$ENDPOINT" --speed 1

# 停止上一个仿真后再运行：已经过 Regrind IK 求解的腕部／手部轨迹。
bash bash/run_mocap.sh h5-sim --h5 "$IK" --object-mesh "$OBJECT_MESH" --endpoint "$ENDPOINT" --speed 1
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
不修改真机 `config/robot.json` 中的 `0.1 rad/s` Home／frame0 接近上限。
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
bash bash/run_mocap.sh infer --policy regrind --model "$MODEL" \
  --h5 "$REFERENCE" --device cpu --iterations 1000

# 需要 mocap 项目的 router 与 right_wrist / hammer 实时刚体流。
# 只订阅，输出 JSON；手部 observation 明确采用上一 policy target 假设。
bash bash/run_mocap.sh live --policy regrind --model "$MODEL" \
  --reference "$REFERENCE" --endpoint tcp/127.0.0.1:7447 --device auto

# 不加载模型：锤子起始放置检查，或只读参考/实时对齐窗口。
bash bash/run_mocap.sh live --reference "$REFERENCE" --preflight-only
bash bash/run_mocap.sh live --reference "$REFERENCE" --viewer
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
已同步到 live 默认外参和 `src/inference/mocap_policy_runtime/mocap_policy_runtime/configs/regrind.yaml`，
保留原 marker→mount→wrist 链。左侧 GL `1 -5 3` mm、GO `3 18 -2` deg
保存在 `../mocap/config/object_offsets.yaml`，不自动套用右侧安装外参。
两侧均为机器人末端定位来源，不加入 mocap 人体采集物体列表。
原始流不预先应用 GL/GO；接口与组合顺序见
外部 `mocap` 工程的 `docs/RIGID_SUBSCRIBE.md`（不随本仓库提供）。

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
pixi run bash -c 'source bash/environment.sh; python -m tianji_controller.run_teleop --devices all --inspect'

# 1. 原始数据：腕部跟随＋手部重定向。
bash bash/run_mocap.sh h5-real --h5 "$RAW" --object-mesh "$OBJECT_MESH" --endpoint "$ENDPOINT" --speed 1 --confirm-real

# 2. Regrind IK 数据：已求解的腕部／手部轨迹直接回放。
bash bash/run_mocap.sh h5-real --h5 "$IK" --object-mesh "$OBJECT_MESH" --endpoint "$ENDPOINT" --speed 1 --confirm-real

# 3. RL policy：实测反馈推理，不是 IK 轨迹逐帧回放；还需要实时 hammer。
bash bash/run_mocap.sh regrind-real --model "$MODEL" --reference "$IK" \
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
源兼容参数显式列在包内 `configs/regrind.yaml`：右手 MIT kp=3、kd=0.05，
电流上限取 profile 与目标硬件配置的较小值；右臂 K/D 和工具参数同样显式配置。
启动会打印所选参数，不改写 `config/robot.json`，设备身份及更严格安全边界保留。
这些参数仍须现场确认，代码与模拟反馈验证不等于实机动作验收。

SHA256 白名单和 Motive/臂/手新鲜度、时间偏差、起始放置和手跟踪误差门禁位于
`src/inference/mocap_policy_runtime/mocap_policy_runtime/configs/regrind.yaml`；不可为绕过预检随意修改。
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

历史功能验证记录（保留原数据，不作为本次 Jazzy／目录迁移的重跑结论）：实际 take003 在 0°／25° yaw 下共 242 个采样点与源 H5 实现对照，
位置最大差约 5.8e-17 m，姿态最大差约 2.7e-7 rad（本项目小角度四元数插值近似）。
实际 `model_5750.pt` 在四个参考帧的 123 维观测、26 维动作及残差转换对照通过，
CPU 动作输出最大差为 0。另已运行实际 NativeIK＋MuJoCo 的
Home/s/Enter/r/暂停恢复/回位/退出场景、原点坐标轴离屏渲染、
完整参考手／锤子回放、合成 Motive 影子推理，以及模拟设备反馈下
H5／Regrind 状态机与故障清理回归。**未连接真实设备，未做带运动真机验收，
未验证现场键盘／Motive 联调，也未验证当前环境的 CUDA 推理。**

本次迁移的逐项结果与尚未完成的 DDS／推理回归见
[迁移验收状态](docs/migration-verification-status.md)。裸 shell 完整安装和安装后的 VR DLS／Ceres
合成输入窗口流程已通过，但它们不能替代本页的真实 Motive、物理键盘、机器人、
设备身份／方向／外参、模型权重及 CUDA 联调；不得据此宣称六条路线全部现场通过。
