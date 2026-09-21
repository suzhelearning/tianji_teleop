---
title: 参考 protype 的遥操作数据采集仓库结构
status: implemented-with-open-field-acceptance
type: decision
updated: 2026-09-21
---

# 参考 protype 的遥操作数据采集仓库结构

## 1. 定位与范围

`tianji_teleop` 的核心定位是 **Tianji 双臂 + Wuji2 双手的遥操作数据采集仓库**。学习 `protype` 的模块组织和实验操作流程，让相机、输入设备、指令转换、控制器、采集器、推理接入都具有清晰的位置与启动入口，而不是进一步抽象成通用机器人框架。

本方案的目录与 ROS 边界已在当前工作区实施。下文记录实际布局、采用的环境隔离与必须保持的验收条件，而不是待创建模块清单。迁移以现有实现为基础，不借目录整理改变控制算法、数据契约或硬件安全边界；**实现落地不等于所有回归或现场验收通过**，逐项证据见[迁移验收状态](migration-verification-status.md)。

本文件仅保存在 `tianji_teleop` 项目内，不同步到个人知识库。

### 1.1 本次核对基线

- 当前迁移以 `main@90c575f` 为基线，整合原 shared-root DLS／标定／Pixi、Mocap／Regrind 与 PICO2 新模式；最终迁移提交以分支实际提交为准，本文不预填 hash。
- 早期草案核对过 `6adfb7ab4af93e066e391d1d8acc97037c88873a` 及当时未提交的数据日期目录改动。该历史边界保留：`/data/TianjiData`、按日期配置与压缩规则不能误记为 `6adfb7a` 自带行为。
- 当前入口在 `bash/`，部署配置在 `config/`，业务包在唯一 `src/` 树，SDK／第三方依赖按其资源闭包归属管理；旧根目录入口不是运行兼容层。
- 已完成裸 shell 完整安装及安装后的 VR DLS／Ceres 合成输入窗口 `S → TELEOP → P/HOLD → H/Home` 验证。最终 DDS／推理回归仍需按验收记录逐项核对；真实输入、相机、机器人、Odin 凭据和 GPU／模型现场条件不由这些结果覆盖。

### 1.2 当前路线不能压成一条流水线

| 路线 | 当前入口与实现 | 迁移必须保留的区别 |
|---|---|---|
| PICO 手柄 DLS 仿真 | `bash bash/run_teleop.sh --sim --user NAME` → `simulation.run_sim` → `simulation.ceres_session` | 默认 `franka-dls`＋Ruckig、direct 交互窗口；默认接收双手，不输出硬件指令 |
| Ceres 仿真 | 同一入口加 `--ik-backend ceres` | 独立求解配置，沿用 DLS/Ceres 会话管理；不是默认真机后端 |
| 旧 SPARK／mapped-palm | 显式 `--ik-backend spark` 或 `mapped-palm` | 保留既有 TJRC 执行路径；无窗口／动力学应显式选择支持它的后端，不能直接给默认 DLS 加 `--headless` |
| 真机与任务采集 | `bash bash/run_teleop.sh --real`／`--data --task TASK` → `tianji_controller.run_teleop` | 独立预检、人工授权及原真机后端；不能把仿真 S/H/P 当作真机授权 |
| PICO2 裸手 | `bash bash/run_pico2_sim.sh`、`src/teleop_inputs/pico2_hands/` | 默认 legacy V131；显式 `--mapping-mode shared-root --height-m HEIGHT` 使用身高模板＋C 标定＋DLS/Ruckig，先 C 后 S；仍仅仿真，不支持真机 |
| Mocap／Regrind | `tianji mocap` → `bash/run_mocap.sh`、`src/inference/mocap_policy_runtime/` | H5 回放、Motive 输入、策略推理和受保护执行有各自入口与可选依赖 |

Manus 与外骨骼是二选一的独立手部发送端，不由 DLS 仿真自动启动或停止。默认双臂输入端口 `15000`、双手 `16000`；协议兼容不代表任意路线组合已完成现场验收。

## 2. 从 protype 学什么

优先借鉴五个方面：

1. **实验流程驱动入口**：操作者按设备检查、相机、输入、遥操、录制的顺序启动，不必了解全部内部模块。
2. **controller / cmd_pub / description 分工**：硬件执行、输入到目标的转换、模型资源分别管理。
3. **配置驱动部署**：设备身份、相机角色、录制路径不散落在脚本中。
4. **相机可独立诊断**：能单独检查序列号、profile、实际取帧频率和预览，不必使能机器人。
5. **数据采集是一等模块**：采样、episode 管理、写盘、验证、压缩和可视化有明确归属。

不照搬具体的 YAM/ARX CAN 驱动、六轴加夹爪的数据维度、增益、回零逻辑或主从助力实现。Tianji 每侧七轴、Wuji2 每侧二十关节，硬件和控制语义不同。

## 3. 已实施的目录与安装布局

```text
tianji_teleop/
├── README.md / README-reference.md / README-mocap.md
├── pixi.toml / pixi.lock / pyproject.toml
├── bash/                                # 安装、构建、环境激活和日常入口
│   ├── install.sh / build.sh / build_native.sh / build_manus.sh
│   ├── environment.sh / check_env.py / test_python.sh / test_native.sh
│   ├── run_cameras.sh / preview_cameras.sh / inspect_cameras.sh
│   ├── run_pico.sh / run_setup_pico.sh / run_stop_pico.sh
│   ├── run_manus.sh / run_exoskeleton.sh / run_pico2_sim.sh
│   ├── run_teleop.sh / run_sim.sh / run_home.sh / stop.sh
│   ├── run_data_collector.sh / run_mocap.sh
│   └── compress.sh / compress_data.sh / view.sh / visualize_data.sh
├── config/
│   ├── robot.json / hand_devices.json
│   └── collect_real.json / cameras.rviz  # collect_real 是唯一相机角色／serial 来源
├── src/
│   ├── tianji/
│   │   ├── tianji_controller/
│   │   │   ├── native/                  # 原控制 CMake、apps、include、mapped_palm
│   │   │   └── tianji_controller/       # Python SDK 所有者、执行器、反馈、保护
│   │   ├── tianji_cmd_pub/              # PICO 位姿 → TJVR 目标，保留原生几何协议
│   │   └── tianji_description/
│   │       ├── models/ / marvin_m6_ccs/  # 模型及网格相对资源闭包
│   │       ├── mapped_palm/assets/
│   │       ├── config/                  # 部署 Home、deployment、bandwidth
│   │       └── tianji_description/      # Home／模型读取 helper
│   ├── wuji/
│   │   ├── wuji_controller/             # Hand2 SDK、反馈与命令契约
│   │   └── wuji_retargeting/            # Python retargeting 包和 _native 扩展
│   ├── teleop_inputs/
│   │   ├── pico_bridge/ / pico_recorder/ # recorder 保留原 PICO HDF5＋MP4 格式
│   │   ├── imu_ros2/ / pico_odin/ / odin_ros_driver/ / odin/
│   │   ├── manus_bridge/ / exoskeleton_bridge/
│   │   └── pico2_hands/                 # legacy V131 与显式 shared-root 仿真
│   ├── cameras/
│   │   ├── tianji_cameras/              # 官方驱动 launch、预检、monitor、订阅预览
│   │   └── fisheye_camera/              # 可选鱼眼，不是默认 RGB 来源
│   ├── data_collector/
│   │   ├── data_collector/
│   │   │   ├── node.py / runtime.py     # 独立 DDS 观测，不持有硬件或 pipeline
│   │   │   ├── session.py / dataset.py  # 生命周期、有界写盘、schema 校验
│   │   │   └── compress.py / visualize.py / web/
│   │   ├── launch/
│   │   └── tests/
│   ├── inference/mocap_policy_runtime/
│   │   ├── mocap_policy_runtime/
│   │   │   ├── data/ / replay/ / policies/regrind/
│   │   │   ├── integration/ / configs/
│   │   │   └── native/tcp_worker.cpp
│   │   └── tests/
│   ├── simulation/simulation/           # direct/dynamics、ceres、PICO owner 会话
│   ├── interfaces/
│   │   ├── tianji_interfaces/           # msg/srv 与独立 tianji_runtime helper
│   │   └── realsense2_camera_msgs/      # 匹配官方驱动的 metadata 接口
│   └── tools/tianji_tools/              # tianji CLI、profile 和组合启动管理
├── profiles/                            # 实际人员档案，不补造个人标定
├── vendor/                              # 厂商 SDK／第三方依赖及许可证
├── docs/
├── build/{default,policy,control}/       # control 下 core 与 mapped-palm 分开
├── install/{default,policy,control}/     # native executable 在 control/bin
├── log/{default,policy}/                # colcon 日志
├── logs/                                # 运行日志，不是训练数据
└── recordings/                          # 输入／仿真／审计，不等同 schema-v1
```

此树强调实际职责，不枚举每个包的清单与测试文件。colcon 只扫描 `--base-paths src`，
不扫描 `vendor/`、旧构建树或环境目录。`src/` 与 `src/tianji/` 仅作目录分组，
不是 Python 包；`tianji` CLI 安装在 `src/tools/tianji_tools/` 对应包内。
原生 control 的 core 与 mapped-palm 保持独立 CMake 工程，不为目录名拆坏算法闭包。

训练数据在仓库外 `/data/TianjiData/raw/` 和 `/data/TianjiData/compressed/`。
Pixi 与独立输入工具环境是安装产物，不搬入 `vendor/`，也不复用旧虚拟环境的扩展。
模型从 `tianji_description` 的安装 share 读取，原生程序通过
`tianji_runtime.native_executable()` 在 `install/control/bin/` 定位；
checkout 的 config／profiles／vendor 由 `TIANJI_WORKSPACE` 显式定位，不从 cwd 猜根目录。

## 4. 主数据流与模块职责

```text
PICO / Manus / 外骨骼 / Pico2 手势
                  │
                  ▼
        teleop_inputs：设备输入适配
                  │
         人员标定与输入新鲜度
                  │
       ┌──────────┴──────────┐
       ▼                     ▼
tianji_cmd_pub         wuji_retargeting
双臂目标转换           双手目标转换
       │                     │
       └──────────┬──────────┘
                  ▼
       控制核心与统一执行会话
       人工授权、限位、限速、反馈检查
                  │
          ┌───────┴────────┐
          ▼                ▼
        真机执行         仿真执行
          │                │
          └──────观测──────┘
                  │
相机观测 ─────────┤
                  ▼
      data_collector：采样与 episode
                  │
         后台写盘、关闭与校验
                  │
                  ▼
        HDF5 → 压缩 / 回放 / 查看
```

这是职责图，不是所有路线共用一个执行器或 writer 的声明。DLS/Ceres 交互 viewer、旧 TJRC 仿真／真机、PICO2 和 Mocap 仍有不同入口；迁移复用已有能力，但不替换算法或合并安全状态机。schema-v1 collector 只接收 `mode=real` 的有效反馈，仿真、dry-run 和审计 trace 不冒充真实关节数据。

### 4.1 控制器、指令转换和模型分开

| 关注点 | 归属 |
|---|---|
| PICO 数据解码与输入发布 | `teleop_inputs/pico_bridge` |
| 人体/VR 坐标到机器人目标映射 | `tianji_cmd_pub` |
| 臂 IK、轨迹与限制 | `tianji_controller` |
| 机器人 SDK、设备反馈、人工授权与协调停止 | 控制器的执行会话 |
| 手型到 Wuji2 关节目标 | `wuji_retargeting` |
| Wuji2 连接、反馈、命令与保护 | `wuji_controller` |
| URDF、MuJoCo 模型、网格资源 | `tianji_description` |

目录拆分不等于进程拆分。双臂和双手仍可在同一执行会话中协调；不能因为拆出了 `wuji_controller` 就重复建立 SDK 会话，也不能破坏现有的协调停止和设备释放顺序。

shared-root 的人体尺度、坐标与目标构建具有指令转换职责，但与 DLS/Ceres 求解、Ruckig 和运动限制紧耦合的原生闭包仍位于 `src/tianji/tianji_controller/native/`。`tianji_cmd_pub` 接收拆出的 PICO 目标转换；不增加每帧 Python 往返或网络跳转。原 `control/apps/` 的交互 viewer、录制及审计工具仍随原生工程构建安装，不把整个控制库当成单一硬件驱动目录。

### 4.2 采集器保持三层边界

当前 `src/data_collector/data_collector/` 保持三层：

- `runtime.py`：订阅 DDS 真实反馈和相机新帧，校验 boot/session/source token、单调时间、新鲜度与缓存。
- `session.py`：由原 `integration.py` 迁移，管理开始、冻结、保存／丢弃／abort 及 episode 文件归属。
- `dataset.py`：有界队列、后台 HDF5 写入、关闭、校验和最终文件发布。

预览、压缩、可视化不混进机器人控制循环。开始/停止录制不改变机器人 TELEOP 状态；记录故障与机器人自身安全机制的关系沿用现有定义。

collector 是独立 ROS 进程，不持有 SDK 或相机 pipeline。真实设备仍只由执行器连接一次，其 feedback sampler 复用同一锁定 SDK 会话，独立线程发布 DDS。`--data` 在设备连接前检查相机与 writer prepared，连接后再等待真实反馈 inputs_ready；相机／collector 已运行时只在精确配置、身份与 ready 验证通过后复用。退出只停止本次拥有的进程，不杀复用会话。

### 4.3 相机独立诊断，但不能重复占用

`tianji_cameras` 独立完成：

```text
枚举设备 → 校验角色与序列号 → 查询支持 profile
        → 检查实际启动 profile → 取帧 → 频率统计 → 预览
```

- 同一 serial 由会话锁保证只有一个官方 `realsense2_camera` 4.58.3 pipeline 所有者；驱动运行在隔离的 `cameras` 环境。
- collector、monitor、preview 都是 DDS 消费者，可以同时订阅；没有另一套正式 SDK 取流路径。
- `pyrealsense2` 2.58.3 仅用于设备枚举与支持 profile 预检，不打开第二条 pipeline。
- 启动使用 `rgb_camera.color_profile/color_format`，D405 使用 `depth_module.color_profile/color_format`；目标固定 `1280,720,30`／`RGB8`，monitor 读取实际参数并校验 Image＋Metadata 后才 ready。
- Image／Metadata 按 frame_id＋stamp 配对，各 topic 分别固定 publisher 身份；源帧号、时间回退、参数改变、换 publisher 或断流撤销健康，不补帧、不自动重启继续拼段。
- 区分请求 FPS、实际 profile FPS、DDS 有效接收率、源序号间隙、HDF5 记录率和 UI 更新率。不能静默降分辨率或改曝光伪造达标。

### 4.4 已采用的 ROS 观测与操作契约

默认 ROS 2 Jazzy／Fast DDS，仅支持同机共享单调时钟：domain `120`、discovery `LOCALHOST`、
`use_sim_time=false`；回归使用独立 domain `121`。TJVR／TJH2／TJRC、MRE1／MPT1、
native stdin/stdout worker 协议保持，不改为 DDS，也不增加第二条硬件命令通道。

- `/tianji/feedback/{arms,left_hand,right_hand}` 的 `DeviceFeedback` 原子携带 boot/session、
  sequence、源单调时间、位置与健康；BEST_EFFORT／VOLATILE／KEEP_LAST(1)。
- `/tianji/executor/state` 的 `ExecutorState` 携带 mode、phase、phase_revision 和 fault；
  RELIABLE／VOLATILE／KEEP_LAST(8)，不是使能授权。
- `/tianji/collection/status` 的 `CollectionStatus` 区分 prepared、inputs_ready、状态、文件路径与错误；
  RELIABLE／TRANSIENT_LOCAL／KEEP_LAST(1)。
- `/tianji/collection/command` 只接受 start/save/discard/abort。start/save/discard 必须匹配新鲜、
  健康 real TELEOP 的 session／phase_revision；abort 只减少录制状态。响应是入队确认，
  最终文件与错误看 status，不自动重试未知结果。
- `/tianji/collection/check_ready` 与 `/tianji/cameras/check_ready` 是只读就绪检查；
  服务不能增加硬件权限。执行器 TTY 的 R/S/D 经后台有界队列转交，不阻塞控制循环。

两只手都具有新 source token 且各自新鲜时才合成 40 维；不能用一侧更新刷新另一侧。
反馈失鲜阈值 0.3 s、图像 2 s；录制前缓存不补写为零时刻。writer 队列满、失鲜、
Home／fault／离开 TELEOP 或执行器心跳中断保留 partial，不因 collector 退出额外停止独立真机 TELEOP。

### 4.5 推理与回放复用执行边界

`src/inference/mocap_policy_runtime/` 容纳现有 `mocap_policy_runtime` 包，不删掉模型运行逻辑。桥接、回放和具体 policy 子模块仍保持内部边界；六个入口的路由与现场边界见 [README-mocap](../README-mocap.md)。

需保留 `data/`、`replay/`、`policies/regrind/`、`integration/`、`native/tcp_worker.cpp` 和 `configs/` 的归属。Mocap 参考数据、策略输入和 schema-v1 观测数据不是同一契约；H5 扩展名相同也不能互相替代。具体数据适配以现有读取器为准，不在目录整理中自动生成 actions 或训练副本。

任何新动作来源都必须接入现有授权、输入新鲜度、限位、限速和反馈检查，不另建一条直接调用机器人 SDK 的旁路。

## 5. 配置分层与唯一来源

| 配置类别 | 位置 | 内容 |
|---|---|---|
| 实验台部署配置 | 根 `config/` | 设备身份、IP、相机角色与序列号、采集组合 |
| 模块参数 | 各模块 `config/` | 控制增益、Home、输入映射、算法选择 |
| 人员档案 | `profiles/` | 操作者骨长、手部标定、映射版本 |
| 数据集契约 | 每个日期数据目录内的 `dataset_config.json` | schema、关节顺序、图像尺寸、编码；同日不可变，不同日期独立校验 |

每个字段只有一个权威来源。相机角色、序列号与禁用槽位只来自 `config/collect_real.json`，不另建 `cameras.json` 双写。SPARK／mapped-palm 部署与真机受保护回位共用 `src/tianji/tianji_description/config/home.yaml`，通过 description 包的 Home helper 和安装资源读取；**DLS／Ceres 仍使用各自配置的 Home**。不得以“统一 Home”为由覆盖不同后端的姿态、速度或授权语义。根配置和模块配置不得重复维护同一个安全阈值。

人员变化不等于机器人配置变化；修改现场序列号也不允许覆盖既有数据集的尺寸、编码或关节顺序契约。

### 5.1 环境也是部署边界

| 环境／任务 | 当前职责与边界 |
|---|---|
| `default`，Python 3.12／Jazzy／Fast DDS | 执行器、采集、仿真、PICO、Manus ROS 桥；MuJoCo 3.10.0，overlay 为 `install/default` |
| `control`，独立原生工具链、无 ROS | C++／Pinocchio／OSQP／qpOASES／Ceres／Ruckig；core 与 mapped-palm 分开构建，程序安装到 `install/control/bin` |
| `cameras`，独立 Jazzy 驱动环境 | 官方 RealSense ROS 4.58.3；隔离其库 ABI，其他进程只通过 DDS 消费，不导入它的 site-packages |
| `manus`，Python 3.12／Pinocchio 3.8、无 ROS | 手重定向与原生扩展；由 default 桥接输入，通过既有进程协议与 TJH2 输出隔离，不 source ROS |
| `policy`，同 default 的 Python 3.12／Jazzy ABI | 单独安装 CPU torch 2.10.0 与 Zenoh，使用自己的 `install/policy`；GPU wheel、驱动和模型权重需显式准备，CPU 不代表 GPU 验收 |
| 外骨骼独立 Python 3.12 manifest | `src/teleop_inputs/exoskeleton_bridge/` 的官方 worker、C++17 FK／拟合扩展；无 Python 数值回退 |
| PICO2 原生工具独立 manifest | `src/teleop_inputs/pico2_hands/tools/wuji_hand_native/pixi.toml`；独立构建入口，共享根 worker 另由 control 安装 |

`bash/environment.sh` 统一校验 Jazzy 和当前环境，加载对应 overlay，清理旧 ROS 前缀污染。
`bash/install.sh` 安装全部锁定环境、外骨骼和 PICO2，构建 default／policy／control 与 Manus；
`pixi run build` 和 `pixi run -e policy build` 分别重建自己的 overlay。缺 Odin mTLS 凭据时相关
Odin 包明确跳过，不能把其他包安装成功说成 Odin 可用。
`bash/run_mocap.sh` 将 infer/live/regrind-real/regrind-hand-sim 分发到 policy，其余到 default；
统一 CLI 不意味着统一解释器，shell 保留 `exec`、退出码、终端及信号所有权。

### 5.2 当前工作区的数据目录

```text
/data/TianjiData/
├── raw/
│   └── YYYYMMDD/
│       ├── dataset_config.json
│       └── <episode>.h5
└── compressed/
    └── YYYYMMDD_compressed/
        ├── dataset_config.json
        └── <episode>.h5
```

- `EpisodeWriter._open_episode()` 按预留 episode 的日期目录校验配置；同日契约冲突拒绝追加，不覆盖已有配置。两个汇总根目录不写共享配置。
- `TIANJI_DATASET` 改变原始数据根目录，压缩默认根目录为其同级 `compressed/`。采集的单次 `--dataset` 不会自动传给下一条压缩命令。
- `compress.py` 支持单个含配置的数据集和日期汇总目录；默认日期输出加 `_compressed`，显式传入 destination 时保留日期名称与内部相对路径，不自动追加后缀。
- JPEG Q50 压缩不删除原始数据，不改关节、时间戳、任务或成功标记；旧 JPEG 输入会二次有损编码。每个日期输出自带配置，可独立查看。
- 上述行为来自原工作区数据目录改动，并随迁移保留；不是早期 `6adfb7a` 提交的默认行为。不要用旧快照覆盖这些日期与路径语义。

## 6. 迁移来源与当前归属

本表左列仅用于追溯旧代码，不是当前命令或导入路径。

| 迁移来源 | 当前归属与保留边界 |
|---|---|
| `tianji/`、`teleop_profile.py` | `src/tools/tianji_tools/`，保留统一命令面和人员操作 |
| `real_robot/` | `src/tianji/tianji_controller/tianji_controller/`；Wuji 专属适配移到 `src/wuji/wuji_controller/`，共用 Feedback 契约在 `tianji_runtime`，不重复连接 SDK |
| `control/` 算法／apps | `src/tianji/tianji_controller/native/`；保留 DLS、Ceres、SPARK、mapped-palm 差异及 shared-root 原生闭包，非测试 executable 安装到 control |
| PICO 中 Tianji 目标转换与协议 | `src/tianji/tianji_cmd_pub/`；保留 TJVR 字节协议和端口 |
| 控制模型、mapped-palm assets、Home helper | `src/tianji/tianji_description/`；保留网格树、安装规则与后端覆盖 |
| `retargeting/` | `src/wuji/wuji_retargeting/`；保留 Python 模块 retargeting／_native，按使用环境重建 |
| PICO／Manus／外骨骼输入 | `src/teleop_inputs/` 对应包；SDK 与自有代码按依赖闭包隔离 |
| PICO 标定、人员和 owner 管理脚本 | `pico_bridge` 包与 `bash/`；保留发布、取消、人员／版本冲突和 token 限定释放，不补造个人标定 |
| 原 PICO `data_collector` | `src/teleop_inputs/pico_recorder/`，ROS／Python 包和节点完整改名；保留 `/pico/record_flag` 与 HDF5＋MP4，避免与 schema-v1 collector 冲突 |
| `pico2_hands/` | `src/teleop_inputs/pico2_hands/`；legacy 默认及显式 shared-root 新模式都仅仿真 |
| `data_collection/` | `src/data_collector/`；integration→session，writer／压缩／web 随包；订阅预览归 `tianji_cameras` |
| `mocap_policy_runtime/` | `src/inference/mocap_policy_runtime/`；保留 data／replay／policies／integration／native／configs |
| `sim/` | `src/simulation/`；保留 ceres_session／pico_owned_session，不把退出清理改成全局 stop |
| 根日常 shell 与部署 JSON | `bash/` 与 `config/`，无旧入口 shim 或第二份相机配置 |

包发现、资源与安装路径已切到唯一 src／install 布局。保留旧来源说明不意味着保留旧模块别名；
审计 trace、历史报告与训练数据仍分开管理。

## 7. 启动与操作体验

`tianji` CLI 是统一业务入口，`bash/` 提供与 protype 类似的直观操作脚本：

```text
CLI / shell：解析参数、选择配置、启动会话
        ↓
业务模块：输入、目标转换、执行、采集
        ↓
设备适配：管理 SDK / 相机 / 网络资源
```

推荐操作者流程：

1. 检查设备身份、配置、磁盘空间与端口占用。
2. 启动官方相机节点后同时预览／检查，确认 serial、profile、Image＋Metadata 和健康；预览只是订阅者，无需为录制释放另一套 pipeline。
3. 选择一种路线：DLS/Ceres 可由 `--user` 校验并启动／复用 PICO；旧路线手动启动 PICO。需要手部时另启 Manus 或外骨骼，二选一；PICO2 使用自己的入口。
4. 仿真使用对应窗口的接入操作；真机／任务采集先做只读预检，再完成现场人工授权，不复用仿真放行结论。
5. 在拥有采集生命周期的会话内启用录制；进入 TELEOP 不自动录制，按既有 R/S/D 流程管理 episode，不再启动第二个执行器。
6. 停止录制、确认任务结果、验证文件并进行离线压缩/查看。

`bash/run_data_collector.sh` 启动独立 DDS collector，不连接机器人、不开相机；`--data` 则管理本次需要的相机／collector 并保留执行器交互 TTY。已有独立节点必须通过精确配置／身份／ready 检查才复用，冲突拒绝，不偷偷接管。

进程创建不等于系统就绪。输入新鲜度、相机首帧、实际 profile 和设备反馈才是就绪条件；固定 `sleep` 不能代替验证。

### 当前入口示例

以下是不同任务的入口索引，不是一组应同时执行的命令；`NAME`、`TASK` 和日期均需替换。

| 任务 | 当前命令 |
|---|---|
| 新人员手柄标定 | `pixi run setup-pico --user NAME --height-m 1.70`（身高替换为实测值） |
| DLS direct 仿真 | `bash bash/run_teleop.sh --sim --user NAME` |
| Ceres direct 仿真 | `bash bash/run_teleop.sh --sim --user NAME --ik-backend ceres` |
| 旧 SPARK 无窗口仿真 | `bash bash/run_teleop.sh --sim --ik-backend spark --headless` |
| PICO2 shared-root 仿真 | `bash bash/run_pico2_sim.sh --mapping-mode shared-root --height-m HEIGHT`（先 C 后 S） |
| Manus 环境／模型离线检查 | `pixi run manus --calibration-user NAME --check` |
| 外骨骼配置离线检查 | `bash bash/run_exoskeleton.sh --check-config` |
| 真机只读预检 | `pixi run bash -c 'source bash/environment.sh; python -m tianji_controller.run_teleop --devices all --inspect'` |
| 真机任务采集（预检及现场授权后） | `bash bash/run_teleop.sh --data --task TASK` |
| 单日离线压缩／查看 | `bash bash/compress.sh --date YYYYMMDD`／`bash bash/view.sh --date YYYYMMDD` |

DLS/Ceres 的 `--user` 仅支持 PICO 端口 `15000`；本次新建的输入会话随仿真退出释放，复用的既有会话保留。`src/simulation/simulation/pico_owned_session.py` 使用本次 token 限定清理范围；Ctrl+C、启动失败和超时也不能改成停止所有 PICO。需要显式停止已有输入时使用 `pixi run stop-pico`，先停止执行端。

## 8. 必须保留的现有约束

### 数据契约

当前 schema-v1 只记录实际双臂/双手关节状态、RGB、各流时间戳和必要元数据：

- 双臂实际关节为 14 维，双手实际关节为 40 维。
- 当前 RGB 契约为 1280×720、RGB8，每路目标 30 FPS。
- 各流独立时间戳、独立样本数量，不要求每次组成同一长度的同步帧。
- 不把控制目标当实测反馈，不把重复缓存作为新观测。
- 不在目录迁移中新增 actions、深度、目标关节或其他 schema 字段。
- 不悄悄改成 640×480、JPEG 采集或重复补帧以追求名义频率。
- 保留 `.partial.h5` → 关闭与校验 → 最终文件的发布流程。
- 按日期的不可变配置、压缩输出独立可读及显式 destination 规则保持；它们属于当前工作区契约，不在目录迁移中退回根共享配置。
- schema-v1 仅约束观测数据集；DLS 动作 trace、PICO2 仿真录制与 Mocap 参考 H5 保留各自格式，不强制套用该 schema。

若未来要记录目标动作、深度或改采集编码，应作为独立的数据契约变更讨论。

### 安全与运行边界

- 保留人工使能授权、设备身份检查、新鲜度检查、限位、限速和反馈保护。
- 真机、仿真和回放复用已有适用的 IK／手重定向实现；不得为“统一”删除现有后端差异，也不得把 DLS/Ceres direct 仿真描述成已接入真机。
- 采集器只订阅真实反馈；SDK 读写与反馈采样复用执行器的硬件会话，不再连接一遍机器人。
- 保留必要的环境隔离，例如已有的外骨骼 Python/SDK 工具链，不为了单一根环境而强行合并 ABI。
- 离线测试和仿真通过不等同于实际运动或持续采集验收。
- 简化 PICO 标定不生成 Manus `.mcal` 或旧完整人员档案；仓库未附带的个人配置必须由实际佩戴者准备，不恢复或伪造已移除的档案。
- DLS/Ceres 的 S/H/P、真机终端 Enter 和 PICO2 操作保持各自语义；不让手势标签自动授权运动。

## 9. 六阶段切换与验收条件

下表保留原六阶段范围；“工作已迁移”不代表对应的现场验收条件已全部满足。

| 阶段 | 已实施的工作范围 | 验收要求 |
|---|---|---|
| 一：入口与配置 | `bash/`、`config/`、锁定 Pixi 环境与独立 overlay | 原参数、任务、退出码、信号、人工授权与 Home 差异保留；完整安装不接触硬件 |
| 二：采集模块 | `src/data_collector/` 独立 DDS collector | schema-v1、日期配置、路径覆盖、显式 destination、partial 与离线读取不变；真实 DDS 服务／失鲜／故障逐项验证 |
| 三：相机模块 | 官方 4.58.3 节点、统一配置、monitor 与订阅预览 | 单一 pipeline；profile 不匹配失败；真实单台／三台持续采集报告频率与间隔，合成 DDS 不能替代设备验收 |
| 四：输入模块 | PICO、pico_recorder、Manus、外骨骼、PICO2 两种仿真模式 | 标定发布／取消、人员版本冲突、owner 创建／复用／释放、侧别与失鲜不变；真实佩戴／方向需现场验收 |
| 五：控制与模型 | controller／cmd_pub／description，control 安装资源 | 后端、Home、资源定位与原生构建无隐式切换；设备会话、授权、协调停止与释放不回归 |
| 六：推理与仿真 | simulation 与 inference 包、六个 Mocap 路由及 worker | policy/default 隔离、native worker 和 CPU 可选依赖边界；模型／GPU／Motive／真机显式验收，不混淆录制格式 |

每个阶段都按完整切换处理：同步更新 imports、资源路径、安装规则、launch、CLI、测试和文档，再移除被替代的旧路径。不要长期保留两套入口或重复实现。

验证顺序为无硬件导入/构建、协议与数据契约检查、仿真、相机只读实测，最后才是经过现场授权的真机操作。未授权时不启动或使能机器人。

按受影响模块使用 `pixi run test-native`／`test-sim`／`test-collection`／`test-controller`／`test-interfaces`／`test-pico`／`test-pico2`／`test-ros`，以及 `pixi run -e policy test-mocap`。`test-pico` 包含 pico_recorder；PICO 简化标定／owner 子集仍有 `test-pico-simple`／`test-sim-user`。`test-ros` 使用 domain 121 的真实 DDS，不是服务 mock 回显。入口还须实际执行帮助／路径解析和相应窗口 smoke，不能只检查文件存在；本次数字与未完成项统一记录在[迁移验收状态](migration-verification-status.md)，历史功能报告不替代本次部署验收。

## 10. 与已有文档的关系

本文记录“学习 protype、以数据采集为中心”的已实施组织选择及原验收范围。功能、安全与数据契约仍以对应实现和契约为准；逐项验证状态单独维护，不能把架构落地读成所有设备均已验证。

- [项目说明与当前操作流程](../README.md)
- [详细操作与采集参考](../README-reference.md)
- [Mocap／Regrind 独立工作流](../README-mocap.md)
- [简化 PICO 标定与会话](pico-simple-calibration.md)
- [DLS／Ceres 交互仿真边界](../src/tianji/tianji_controller/native/docs/verification/ceres_interactive_sim.md)
- [本次基线的历史合并验证记录](merge-main-2026-09-20.md)
- [迁移验收状态与未验证范围](migration-verification-status.md)
- [当前数据采集契约 schema-v1](../schema-v1.md)
- [此前的未来重构讨论](../未来重构计划.md)
- [mapped-palm 真机边界](mapped-palm-real-readiness.md)
- [Pico2 裸手设计与计划](pico2-hands-design-and-plan.md)

最终目标：打开目录就能找到相机、输入、指令转换、控制器、采集器和推理桥；打开 `bash/` 就能按实验流程操作。所有组织调整都服务于稳定地产生可信的遥操作数据。
