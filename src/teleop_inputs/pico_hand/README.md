# PICO 裸手 → SPD：共享根 DLS／Ruckig 与 ROS 关节命令

裸手仿真只保留这一条双臂主线：`pico_ee_franka_dls`，配置
`qp_ik_pico_shared_root_dls.yaml`，`post_smoothing.mode=ruckig`。
旧 V131 求解器、模型、构建目标和旧映射分支已移除，不提供回退或兼容入口。

入口为 **`bash bash/run_pico_hand_sim.sh --height-m HEIGHT`**。
身高必须显式给出，单位米、范围 `[1.0, 2.4]`；不猜默认人员或身高。
不再接受 `--mapping-mode` 或旧 `--arm-control-mode`。
Python／ROS 包名仍为 `pico2_hands`，录制 schema 不改名。

**本入口默认向 SPD 仿真发布 ROS 关节命令；不连接机器人，不发布真机 TJRC／遥操 UDP。**
手腕／掌心位姿经共享根映射和 DLS/Ruckig 生成双臂目标，手骨架经独立 Hand2 重定向
生成双手目标。原生 Viewer 只辅助观察同一份目标，不是命令来源。
Manus／外骨骼＋PICO 手柄的真机入口、授权和输出完全独立，本功能不修改它们。
手势分类只是观察标签，不会自动使能、暂停或代替急停；合成验证不代表现场跟踪质量合格。

## 构建与环境

从工作区根目录执行：

```bash
# 主控制环境提供唯一双臂 DLS/Ruckig worker
pixi run build

# 手部优化器保持独立 Pinocchio 4 / NLopt 环境
pixi install --locked --manifest-path src/teleop_inputs/pico_hand/tools/wuji_hand_native/pixi.toml
bash src/teleop_inputs/pico_hand/build_native.sh
```

`build_native.sh` 仅构建 Hand2 worker／scheduler／optimizer，并运行手部原生检查，
不再构建独立双臂求解器。`pico2_dls_worker` 和显示用的 `tianji_qp_ik_viewer`
由主工作区构建、安装到 `install/control/bin/`，通过 `native_executable()` 定位。

仿真 Python 使用工作区 Pixi default 环境与安装 overlay；模型和配置使用
`tianji_description`／`controller_profile()`。Hand2 独立环境的 site-packages
和动态库不注入 default；不依赖外部源码 checkout，也不回退到旧程序。
消息包 `tianji_spd_interfaces` 也由 `pixi run build` 在 default overlay 中生成，
不能靠源码路径代替生成消息包。`src/interfaces/tianji_spd_interfaces/source_manifest.json`
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
`--duration-s 30` 到时请求回 Home 后退出，不是 30 秒立即强制结束。
同一端口受输入锁保护，不能重复启动两个仿真会话。

## SPD ROS 命令契约

不需要额外发布开关，普通启动默认发布。SPD 只订阅关节目标，不运行第二套 PICO／IK
或重定向；不要同时启动 SPD 自己的其他目标发布器。发布器的同机／同域锁只排除
本入口的重复实例，不能替代对其他 DDS 发布者的部署检查。

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

`ready_mask` 为双臂 `1`、右手 `2`、左手 `4`，不是本地显示的固定 `flags=7`：
未 C 标定时为 0；成功 C 后已初始化的当前命令可成为 SPD 的启用候选，但不自动授权。
跟随时按有效目标与源时间独立检查手部位；`--disable-hands` 不置手部位。
P／H／Q 的受控制动、Home 和已知手指保持仍发布有效命令，不把正常保持伪装成断流。
真正失效的手部位清除；未主动保持的手部缓存不会被另一侧或新输入帧续命。

进程启动、成功接受新的 C 请求、输入身份／连接变化或 DLS epoch 变化建立新 UUID 会话。
新会话先发送 `ready_mask=0` 的失效边界，之后序号严格递增。发布线程不重放同一目标；
生产端停止更新超过 100 ms 后只发一次新的失效快照，不能用新时间戳续接旧健康命令。
UTC／monotonic 偏移变化超过 5 ms 时当前会话保持失效，需要新的显式会话恢复。

SPD 仍须本地显式启用并检查接入差值；应先统一双方初始目标、零位、方向和限位，
再在 C 后、S 前对齐并授权。发布器不修改模型范围，也不把超范围目标偷偷裁剪。
同名关节和消息校验通过不代表 SPD 的动力学／初始对齐已验收。
Q 会等待最后一份 Home 目标在本地调用 DDS publish 后关闭，但 BEST_EFFORT 不是接收确认，
本地 `home=true` 也不证明 SPD 的物理关节已经到 Home；接收端须检查自己的实际状态。

## C 标定与操作

1. 面向前方，双臂水平前伸，双手间距约肩宽，掌心相对。
2. 空闲时按 **C**，稳定保持约 1 秒。要求至少 30 个独立有效采样、覆盖至少 0.75 秒。
3. 标定成功且有新鲜帧后按 **S**。未标定、标定中、失败、回程中或输入过期均不能接管。

| 按键 | 行为 |
|---|---|
| C | 空闲时建立本次共享根与掌心姿态标定 |
| S | 标定、新鲜度及静止门控通过后软启动跟随 |
| P／空格 | 制动保持；也可取消 H 回程 |
| H | 先制动再回双臂 Home，手指保持 |
| Q／Ctrl+C／Esc／关闭窗口 | 回双臂 Home 后退出，排空录制并清理 worker |

退出回程不接受暂停，避免 Q 被留在半途。已取消旧路线的 R 重新准备操作。
点击原生窗口后使用上述按键；H 只请求父进程回 Home，不切换凸包显示。
Q 保持窗口直到回程结束；Esc／关闭窗口可先关闭显示，但父进程仍完成 Home，
不可把窗口消失当作回程完成。终端操作仍可用。

原生显示控制：F2 显示／隐藏关节曲线，F3 切换位置／速度／加速度／jerk，
F4 切换并锁定曲线的左／右臂，F5 恢复跟随 L／R 选择的臂；
L／R 只选择观察臂，不接管控制。鼠标沿用原生相机操作，不能编辑关节或目标位姿。
曲线由父进程的单调时间戳与模型参考计算，不是真机反馈。
裸手窗口按最近 5 秒的模型曲线自动缩放每个关节的纵轴，留出上下边距，
不再由完整关节限位决定显示范围；位置曲线的最小纵轴跨度为 0.1 rad，避免放大数值噪声。
角度与导数的数值、单位不变。红色限位线可能位于当前视野之外，窗口会提示，
不能用“没看到红线”判断不存在限位，也不能按曲线高度直接比较不同关节的运动幅度。
模型参考以名义 200 Hz 非阻塞发送，显示繁忙时丢弃显示帧，不等待渲染，
也不改变 DLS／Ruckig、C 标定或录制的状态机语义。

### 失鲜与重连

输入超过 45 ms 或跟踪无效时先制动，不继续推进旧目标。短时丢帧仅在以下条件下
允许有界自动续接：距最后有效输入不超过 1 秒，同一连接恢复至少 100 ms、5 个独立
有效帧，相邻帧间隔不超过 45 ms，且 C／映射仍有效。还必须等原生 HOLD 静止和新帧，
再走与 S 相同的软启动；重复／倒序帧不能刷新资格。

长时断流需要人工 S。TCP 断开立即取消恢复资格并使 C 失效，即使快速重连也不能
直接接管；静止后重新 C，再有新帧才允许 S。映射／IK 拒绝、主动暂停、Home 和退出
不会自动接管。不在运动或制动中把参考速度／加速度清零。

## 映射与数值边界

肩宽 `0.1828 H`、上臂 `0.155882 H`、前臂 `0.152941 H`、腕掌 `0.037037 H` 是
对称身高模板，**不是个人实测**。从 wrist 沿 wrist→middle proximal（点 12）
增加一次腕掌偏移，不对已偏移的 palm 点重复补偿。

C 用头显水平朝向固定根轴，以前伸双掌和估计臂展建立虚拟肩中点，同时记录掌心
姿态修正。参考姿态只通过 FK 查询，不直接执行。虚拟根随头显平移、不随转头旋转；
转身或明显弯腰后应 H 回位再 C。

位置复用共享根仿射映射，读取冻结的 `shared_root_robot_geometry_ceres.yaml` 并校验
模型指纹；文件名含 `ceres` 不代表选择 Ceres 求解器。它不包含 VR M0 的实测肩肘、
完整骨架重建或 shape guidance，不能把身高模板称作完整人体追踪。

`DlsWorker` 通过有界本地管道调用主控制器的 DLS、Pinocchio、Ruckig 和
`SimulationRecovery`。目标是世界坐标掌心 TCP；启动时与显示模型实际 FK 对照。
双臂沿用主线模型限位，不再施加旧 J3 范围覆盖。Hand2 的几何、优化器和滤波不变。
通信错误、异常退出和时钟／序号错误仍失败锁存，不以旧值或零值补包。

仿真为 direct joint-state 显示，不进行 PD、动力学积分或重力下垂。名义周期 200 Hz，
裸手 S 接近上限仍为 1.4 rad/s、3 rad/s²、12 rad/s³，与正常限值取较小值；
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
正常 Home、worker 退出和队列排空后才标记 `complete=true`；出错保留不完整记录。
旧人员标定和历史录制不删除、不改写。

只观察输入、不创建执行器时，在已激活工作区 overlay 的环境运行：

```bash
python -m pico2_hands.observe --duration-s 10
```

它不配置 ADB，不代表标定、手势准确率或运动安全已通过；退出状态不能替代持续流验收。

## 离线自测

```bash
# 不连接 TCP 或设备：真实 DLS/Hand2、C 标定、S/H/S/Q；ROS 强制隔离到 domain 121
bash bash/run_pico_hand_sim.sh --height-m 1.75 --self-test \
  --record /tmp/pico_hand_dls_self_test.h5

pixi run test-pico2 -q
```

保留的 `python -m pico2_hands.scripts.smoke_pipeline --height-m 1.75` 复用同一自测，
不实现第二套算法。自测先确认 C 前的 S 被拒绝，然后走真实 C 收集与原生求解，
检查双臂／双手响应、H 手指保持及最终 Home；不会跳过标定门禁。
自测仍走真实 ROS 发布链，但强制使用同机 domain 121，不能把合成目标发往默认 SPD domain 120。
这里只验收发布与控制输出；SPD 物理执行、模型一致性和真实裸手输入需分别验收。
手部构建还检查 scheduler、优化器 ABI 和 16 帧官方参考对照。

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
