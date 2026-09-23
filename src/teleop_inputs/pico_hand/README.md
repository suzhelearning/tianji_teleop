# PICO 裸手 → SPD：共享根 DLS／Ruckig 与 ROS 关节命令

裸手仿真只保留这一条双臂主线：`pico_ee_franka_dls`，配置
`qp_ik_pico_shared_root_dls.yaml`，`post_smoothing.mode=ruckig`。
旧 V131 求解器、模型、构建目标和旧映射分支已移除，不提供回退或兼容入口。

入口为 **`bash bash/run_pico_hand_sim.sh --height-m HEIGHT`**，脚本自行进入独立的
**`spd`** Pixi 环境，不读取 default／policy overlay。
身高必须显式给出，单位米、范围 `[1.0, 2.4]`；不猜默认人员或身高。
不再接受 `--mapping-mode` 或旧 `--arm-control-mode`。
Python／ROS 包名仍为 `pico2_hands`，录制 schema 不改名。

**本入口默认向 SPD 仿真发布 ROS 关节命令；不连接机器人，不发布真机 TJRC／遥操 UDP。**
手腕／掌心位姿经共享根映射和 DLS/Ruckig 生成双臂目标，手骨架经独立 Hand2 重定向
生成双手目标。原生 Viewer 只辅助观察同一份目标，不是命令来源。
Manus／外骨骼＋PICO 手柄的真机入口、授权和输出完全独立，本功能不修改它们。
手势分类只是观察标签，不会自动使能、暂停或代替急停；合成验证不代表现场跟踪质量合格。

## 构建与环境

本入口固定使用根 `pixi.toml` 的独立 **`spd`** 环境（`no-default-feature`）：Python 3.12、
Jazzy ROS 构建工具、numpy／scipy／h5py／pyyaml／MuJoCo，以及编译原生目标所需的
CMake／Ninja／编译器。default 与 policy 的 `pixi run build` 已跳过 `pico2_hands` 和
`tianji_spd_interfaces`，两条链路不共用运行时。SPD 仿真本身不在这里安装：
它属于独立 checkout `/home/current/syz/spd-syz`，关节目标经 ROS 发往该项目。

从工作区根目录执行：

```bash
# 首次安装或锁文件更新后
pixi install --locked -e spd

# 首次／原生代码更新后：编译 DLS worker、Hand2 与 ROS 接口
pixi run --locked -e spd build
```

`build` 调用 `bash bash/build_spd.sh`，依次完成三件事：

1. 复用既有 `control` 工具链编译双臂 DLS worker 和辅助 `tianji_qp_ik_viewer`，
   连同控制器 profile 安装到 `install/spd`（可执行文件在 `install/spd/bin/`），
   由 `native_executable()`／`controller_profile()` 在本环境前缀内定位，
   不回退到 `install/control`；
2. 安装包内独立的 Hand2 环境（`tools/wuji_hand_native/pixi.toml`，Pinocchio 4／
   NLopt／Python 3.12），构建 Hand2 worker／scheduler／optimizer 并安装到
   `install/spd`（`bin/`／`lib/`）；不需要另外手动执行 `build_native.sh`；
3. 只构建这条路线需要的 ROS 包：`tianji_interfaces`、`tianji_spd_interfaces`、
   `pico2_hands`、`tianji_description`、`tianji_controller`、`simulation`。

构建产物只写入 `build/spd`、`install/spd` 和 `log/spd`，不写入 default／policy 的
overlay，也不从它们借用 site-packages。模型和配置经 `tianji_description`／
`controller_profile()` 解析；Hand2 独立环境的 site-packages 与动态库不注入 spd；
不依赖外部源码 checkout，也不回退到旧程序。
消息包 `tianji_spd_interfaces` 由本环境构建生成，不能靠源码路径代替生成消息包。
`src/interfaces/tianji_spd_interfaces/source_manifest.json`
记录 SPD 消息源码的固定指纹；这是同一 schema 的只读快照，更新须与 SPD 同步，
不得在两边独立改 `.msg`。两端各自构建对应 Python ABI 的绑定，不混用安装 overlay。

## 真实裸手输入启动

先停止占用同一输入的旧会话。头显运行**裸手跟踪 APK**，不是 VR whole-body／
手柄 APK。这条路线不启动 `run_pico.sh`、Manus 或外骨骼，不读取 VR 人员档案或
手柄 TCP 标定。USB 连接头显并授权调试后，入口自动检查默认裸手 ADB 转发。

```bash
# 1.75 仅为示例，必须替换为实际佩戴者身高
bash bash/run_pico_hand_sim.sh --height-m 1.75
```

入口按实际 `CONDA_PREFIX` 判断环境：不在 `<工作区>/.pixi/envs/spd` 时，清除继承的
`PYTHONPATH`／ROS／overlay 变量后用 `pixi run --locked -e spd` 重新执行自身。
不需要先手动 `pixi shell -e spd`；在 default／policy 环境里直接执行会被切换到 spd，
且不借用原环境的 overlay。改过 Python／原生代码后须先重跑 `pixi run --locked -e spd build`，
运行中的进程不会热加载新代码。

默认输入为 `127.0.0.1:10002`，可用 `--host`／`--port` 指定。
普通启动使用 `127.0.0.1:10002`（或 `localhost:10002`）时，先检查 `adb devices`
和 `adb forward --list`：正确的同设备同端口规则直接复用，缺失时用 `--no-rebind`
补建 `tcp:10002 → tcp:10002`。未授权／离线、多设备不明确或已有冲突映射会在
启动 Viewer／ROS 发布器之前报错；多设备时可用 `ANDROID_SERIAL` 显式指定头显。
转发保留到退出之后。自测和自定义主机／端口不操作 ADB，接收器不自动重写冲突规则。
默认显示双臂和双 Hand2；窗口复用主线原生 `tianji_qp_ik_viewer` 的模型渲染、
相机和关节曲线，不再使用 Python passive viewer。显示子进程仅以
`--external-display` 从私有 stdin 管道接收 54 维模型参考与状态，不运行控制器、
不接收网络输入、不导出关节命令。`--disable-hands` 只关闭手指重定向，仍需要有效双腕输入。
`--headless` 只关闭本地窗口，ROS 发布继续，键盘操作可在交互终端完成。
`--duration-s 30` 到时退出，不发起 Home。
同一端口受输入锁保护，不能重复启动两个仿真会话。

### 与 SPD 独立操作

普通启动无需远端控制参数。在上游窗口或交互终端先 **R** 标定、再 **S** 开始实时目标生成；
SPD 只订阅 ROS `JointCommand`，执行与采集按键由 SPD 自己管理。SPD 的暂停、保存与检查点
回退不会暂停上游，也不请求上游 Home。暂停 SPD 时仍可移动双手，恢复时使用最新实时目标。
SPD 侧的启动与完整状态表见 `/home/current/syz/spd-syz` 的 README。

两项目间只保留 54 维 `JointCommand` 目标话题：没有跨项目控制 RPC、控制状态 Topic、
心跳租约或控制 socket。SPD 的本地操作不改变上游状态，失鲜与会话变化由目标接收门控处理。

## SPD ROS 命令契约

不需要额外发布开关，普通启动默认发布。接收端是独立 checkout
`/home/current/syz/spd-syz` 里的 SPD 仿真：它只订阅关节目标，不运行第二套 PICO／IK
或重定向；不要同时启动 SPD 自己的其他目标发布器。发布器的同机／同域锁只排除
本入口的重复实例，不能替代对其他 DDS 发布者的部署检查。

两侧环境完全独立：本路线安装／构建 `spd`，SPD 侧用其自身 Pixi 环境和消息绑定，
关节目标通过默认 domain 120 的 ROS 话题发布（自测隔离到 121）；两端不互相 `source` overlay，也不共享
site-packages、构建目录或接口源码。本仓库不启动、不管理 SPD 进程，也不复制它的
安装／仿真命令；SPD 侧的启动、按键与采集流程见该项目自己的 README。
原生 Viewer 是本入口管理的辅助显示子进程，从私有管道接收模型参考，不生成命令，
也不代表 SPD 已接收或已执行任何目标。

| 项目 | 固定契约 |
|---|---|
| 话题 | `/spd/tianji_wuji2/v1/joint_command` |
| 类型 | `tianji_spd_interfaces/msg/JointCommand` |
| schema／配置名 | `1`／`tianji_wuji2_v1` |
| 顺序 | 左臂 7、右臂 7、左手 20、右手 20；每帧携带 54 个关节名 |
| 数值 | DLS／Hand2 控制输出边界的绝对关节目标，rad；不是 qpos 测量或绘图差分 |
| QoS | BEST_EFFORT、KEEP_LAST(1)、VOLATILE |
| 环境 | Jazzy、Fast DDS、默认 domain 120、LOCALHOST，清除静态发现 peer |
| 发布节拍 | 最多 60 Hz、只发最新生产快照；DDS 不阻塞 200 Hz 控制循环 |
| 时间 | 控制周期起点的 UTC 时间；保留求解耗时，发送时不刷新旧目标时间 |
| 接收端 | 独立 checkout `/home/current/syz/spd-syz` 的 SPD 仿真，只订阅本话题的关节目标 |
| 本地显示 | 辅助原生 Viewer 子进程，只收私有管道模型参考；不是命令来源，也不证明已接收 |

`ready_mask` 为双臂 `1`、右手 `2`、左手 `4`，不是本地显示的固定 `flags=7`：
未标定时为 0；标定后已初始化的当前命令可成为 SPD 的启用候选，但发布不自动授权 SPD。
跟随时按有效目标与源时间独立检查手部位；`--disable-hands` 不置手部位。
P／H 的受控制动、Home 和已知手指保持仍发布有效命令；Q 直接退出，不请求 Home。
真正失效的手部位清除；未主动保持的手部缓存不会被另一侧或新输入帧续命。

进程启动、接受新的标定请求、输入身份／连接变化或 DLS epoch 变化建立新 UUID 会话。
新会话先发送 `ready_mask=0` 的失效边界，之后序号严格递增。发布线程不重放同一目标；
生产端停止更新超过 100 ms 后只发一次新的失效快照，不能用新时间戳续接旧健康命令。
UTC／monotonic 偏移变化超过 5 ms 时当前会话保持失效，需要新的显式会话恢复。

SPD 仅由本地操作授权，按自身平滑接入和失鲜规则推进物理；上游发布不授予 SPD 运动权限。
双方仍须统一关节名称、零位、方向与限位，发布器不把超范围目标偷偷裁剪。
同名关节和消息校验通过不代表 SPD 的动力学／初始对齐已验收。
本地退出不请求 Home。最后一次本地 publish 不是
接收确认，本地 `home=true` 也不证明 SPD 的物理关节已经到 Home。

## 本窗口 R 标定、S 开始持续跟随

所有操作在上游窗口或交互终端完成，SPD 不远程启动或暂停上游。

1. 面向前方，双臂水平前伸，双手间距约肩宽，掌心相对。
2. 点击 Viewer 窗口按 **R**（交互终端也可），稳定保持约 1 秒。
   要求至少 30 个独立有效采样、覆盖至少 0.75 秒。
3. 标定成功后保持当前目标并显示 ready；按 **S** 才进入 `teleop` 持续跟随。
   S 需要标定完成后的新鲜有效输入帧，不能用完成标定的缓存帧启动。

| 按键 | 行为 |
|---|---|
| R | 标定／重新标定；运动中先受控制动到 HOLD，再采样，成功后等待 S |
| S | 有效标定及新鲜输入就绪后开始／恢复持续跟随 |
| P／空格 | 上游受控制动并保持，等待 S；不是 SPD 的暂停键 |
| H | 上游双臂 Home，不自动恢复跟随 |
| Q／Esc／关闭窗口 | 退出本地控制端，不发起 Home |
| Ctrl+C | 在终端退出，清理 worker 和录制队列，不发起 Home |

窗口和交互终端均接受上述按键；C 不用于普通本地模式，R 不再是选择右臂的显示快捷键。
重新标定立即撤销旧标定的跟随资格，失败后不会继续使用旧映射，成功也不会自动恢复运动。

原生显示控制：F1 显示帮助，F2 显示／隐藏关节曲线，F3 切换位置／速度／加速度／jerk，
F4 切换并锁定曲线的左／右臂，F5 恢复跟随当前观察臂；
L 仍可选择左臂，选择右臂使用 F4。鼠标沿用原生相机操作，不能编辑关节或目标位姿。
曲线由父进程的单调时间戳与模型参考计算，不是真机反馈。
裸手窗口按最近 5 秒的模型曲线自动缩放每个关节的纵轴，留出上下边距，
不再由完整关节限位决定显示范围；位置曲线的最小纵轴跨度为 0.1 rad，避免放大数值噪声。
角度与导数的数值、单位不变。红色限位线可能位于当前视野之外，窗口会提示，
不能用“没看到红线”判断不存在限位，也不能按曲线高度直接比较不同关节的运动幅度。
模型参考以名义 200 Hz 非阻塞发送，显示繁忙时丢弃显示帧，不等待渲染，
也不改变 DLS／Ruckig、标定或录制的状态机语义。

### 失鲜与重连

输入超过 45 ms 或跟踪无效时先制动，不继续推进旧目标。同一连接、标定仍有效时，
不论丢帧是否超过 1 秒，恢复至少 100 ms、5 个独立有效帧，相邻帧间隔不超过 45 ms，
并完成原生 HOLD 静止后，按既有策略自动软启动跟随；此恢复只适用于之前已用 S 启动的会话。
重复／倒序／缓存帧不能刷新资格；首次及重新标定完成均等待显式 S。

普通本地连续模式在速度／加速度满足原有静止阈值后，再确认 50 ms 即进入 HOLD；
其他原生调用者默认仍为 300 ms。恢复仍需要上述新鲜度与稳定帧门控。
同一标定／连接、距最后原生接受的输入不超过 1 秒（含制动和稳定等待）的恢复，
保留已进入的 Ruckig 软启动／正常跟随进度，不重新从慢速接近起步。首次标定、
重新 R、重连／数值 epoch 重置、较长断流或原生拒绝保留进度时，仍走完整软启动。
短时恢复不会刷新最后有效输入时间，也不跳过静止检查、关节限位或速度／加速度／jerk 约束。

TCP 断开或输入身份改变会撤销标定，制动保持而不是自动 Home；恢复连接后必须重新 R 再 S。
标定失败、原生 FAULT 或通信失败不会绕过门控自动接管。不在运动或制动中把参考
速度／加速度清零。

### 处理链与周期统计

每个已接受的输入帧复用有效性、手部观察、掌心几何与映射目标；目标缓存同时绑定标定解。
重复／倒序输入不能刷新源时间，失鲜判断仍逐周期检查原始接收时间。断开、换源、重新标定
或标定失败会使相应缓存失效，不用旧几何冒充新输入。

左右手各自的原生 worker 保持独立：先向两侧有效输入都提交请求，再收取两侧结果。
每侧最多一个在途请求，响应沿用提交时建立的绝对超时，不积压旧帧；所有已请求结果校验
通过后才一起提交手部目标和源时间。一侧失败不提交半帧，也不自动重启 worker。

默认每 **5 秒**输出一行 `kind=pico2_runtime_stats` JSON；正常退出再输出最后一个非空窗口
（`final=true`）。沿用已有按键／状态变化日志，不新增逐帧统计输出。
可用 `--stats-interval-s 2` 改为每 2 秒，`--stats-interval-s 0` 关闭汇总。

| 字段 | 含义 |
|---|---|
| `control_hz` | 窗口内实际完成的控制周期频率，不是配置值 |
| `input_hz`／`accepted_input_hz` | 接收回调中的唯一帧频率／控制线程实际接受的帧频率；最新帧槽可能跳过中间帧 |
| `timing_ms.cycle`／`work` | 周期起点间隔／周期工作耗时，后者不含主动 sleep |
| `timing_ms.input_interval`／`receive_age` | 同连接帧的本机接收间隔／完成输出时距原始本机接收的时间，**不是头显端到端传输延迟** |
| `timing_ms.input_processing` | 控制端接收帧处理、缓存与手势计算，不含 TCP 读取和协议解码 |
| `timing_ms.dls`／`hands` | 双臂求解及 IPC／双手并行请求和统一收取的总耗时；不是左右手时间之和 |
| `timing_ms.model`／`output` | 本地模型检查与更新／发布入队、状态和录制处理、Viewer 提交；不含异步绘制及 DDS 传输完成时间 |
| `work_over_5ms` | 工作耗时超过 5 ms 的周期数；周期调度抖动另看 `cycle` |
| `braking_entries` | 按 `stale_input`、`recalibration`、`disconnect` 等原因统计进入 BRAKING 的次数，不按停留帧重复计数 |

每项耗时包含样本数、平均值、P50/P95/P99 和最大值；没有执行的阶段为 `null`。
统计使用固定大小直方图，不保留逐帧样本。分位数为桶上界近似值：20 ms 内分辨率
0.1 ms，20–100 ms 为 1 ms，100–1000 ms 为 10 ms，更长尾部报告实际最大值。
汇总生成和输出只在窗口结束发生；统计不修改控制周期、滤波、失鲜门槛或运动权限。

## 映射与数值边界

肩宽 `0.1828 H`、上臂 `0.155882 H`、前臂 `0.152941 H`、腕掌 `0.037037 H` 是
对称身高模板，**不是个人实测**。从 wrist 沿 wrist→middle proximal（点 12）
增加一次腕掌偏移，不对已偏移的 palm 点重复补偿。

R 用头显水平朝向固定根轴，以前伸双掌和估计臂展建立虚拟肩中点，同时记录掌心
姿态修正。参考姿态只通过 FK 查询，不直接执行。虚拟根随头显平移、不随转头旋转；
转身或明显弯腰后应恢复标定姿态，按 R 重新标定。

位置复用共享根仿射映射，读取冻结的 `shared_root_robot_geometry_ceres.yaml` 并校验
模型指纹；文件名含 `ceres` 不代表选择 Ceres 求解器。它不包含 VR M0 的实测肩肘、
完整骨架重建或 shape guidance，不能把身高模板称作完整人体追踪。

`DlsWorker` 通过有界本地管道调用本路线构建的原生 DLS worker（Pinocchio、Ruckig 和
`SimulationRecovery`）。目标是世界坐标掌心 TCP；启动时与显示模型实际 FK 对照。
双臂沿用主线模型限位，不再施加旧 J3 范围覆盖。Hand2 的几何、优化器和滤波不变。
通信错误、异常退出和时钟／序号错误仍失败锁存，不以旧值或零值补包。

仿真为 direct joint-state 显示，不进行 PD、动力学积分或重力下垂。名义周期 200 Hz，
持续跟随的软启动接近上限仍为 1.4 rad/s、3 rad/s²、12 rad/s³，与正常限值取较小值；
这些不是实时性能或真机授权结论。

## 录制与只读观察

```bash
mkdir -p recordings/pico2_sim
RUN_ID=$(date +%Y%m%d_%H%M%S_%N)
bash bash/run_pico_hand_sim.sh --height-m 1.75 \
  --record "recordings/pico2_sim/pico_${RUN_ID}.h5"
```

文件必须不存在。继续使用 `pico2_sim_session_v1`：保存 1982 字节原始帧、源／接收
时间、连接代次与序号、54 维关节目标、输入关联及按键／标定事件。
正常退出、worker 清理和队列排空后才标记 `complete=true`；出错保留不完整记录。
旧人员标定和历史录制不删除、不改写。

只观察输入、不创建执行器时，在 `spd` 环境并激活同一 overlay 运行：

```bash
pixi run --locked -e spd bash -c \
  'source bash/environment.sh; exec python -m pico2_hands.observe --duration-s 10'
```

它不配置 ADB，不代表标定、手势准确率或运动安全已通过；退出状态不能替代持续流验收。

## 离线自测

```bash
# 不连接 TCP 或设备：真实 DLS/Hand2、R 后等待 S、运动中重新 R 后再次 S；隔离域 121
bash bash/run_pico_hand_sim.sh --height-m 1.75 --self-test \
  --record /tmp/pico_hand_dls_self_test.h5

pixi run --locked -e spd test-pico2 -q
```

保留的 `python -m pico2_hands.scripts.smoke_pipeline --height-m 1.75`（同样在 `spd`
环境与其 overlay 中运行）复用同一自测，不实现第二套算法。
自测走真实 R 收集与原生求解，检查首次标定和运动中重新标定后均保持静止就绪，
等待期间输入持续变化，显式 S 后检查双臂／双手响应；最后直接退出而不请求 Home。
自测仍走真实 ROS 发布链，但强制使用同机 domain 121，不能把合成目标发往默认 SPD domain 120。
这里只验收发布与控制输出；SPD 物理执行、模型一致性和真实裸手输入需分别验收。
`test-pico2` 里的 Hand2 用例会真正启动 `install/spd` 中的原生 worker（未构建时跳过）；
离线对照脚本 `pico2_hands.scripts.compare_hand_worker` 保留，需要比对官方参考时手动运行。

## 历史证据说明

旧 V131 实现已经删除，以下仅保留以前记录的数字，不是当前可运行路线或当前测试数量：

| 历史阶段 | 当时记录 |
|---|---|
| 初始接收参考接入（2026-09-16） | 284 passed |
| 旧可选高度标定适配 | 296 passed |
| 旧原生 IK 通信阶段 | 302 passed，另有 IK 专项 6 passed |
| 旧完整仿真接线 | 312 passed；4151 周期、8313 条录制完整处理 |
| 旧原生移植阶段 | 289 passed；V131 CTest 3/3、Hand2 scheduler 1/1；16 帧手部对照最大差 0 rad |
| 后续回归（2026-09-17） | 316 passed；另一次自测 4128 周期 |

旧 V131 参考版本为 `3cfa5108b12d21232ce13a1f0d84831ad525d294`，当时有限轨迹对照
的参考二进制 SHA256 为 `cd8bab2e555fc296fea671d642b22cde31cc6db2aa9551782f6c565ae67349b9`；
这些来源信息不再构成运行或构建依赖。保留代码的来源及指纹见 `source_manifest.json`
和 `native_source_manifest.json`。当前迁移验收统一见
[迁移验证状态](../../../docs/migration-verification-status.md)。
