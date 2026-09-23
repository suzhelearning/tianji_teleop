# 简化 PICO 标定

## 当前入口：新用户从零采集并独立发布

已接通左侧掌心 TCP → 身高派生腕掌距离及臂长 → 派生右侧 TCP → 确认发布 → 独立生效指针。
`setup-pico` 向导回车确认发布并显示骨架；单独 `calibrate-pico` 入口仍输入 `publish`。
以下是当前软件入口说明；新 Pixi 启动链路仍待真实设备完整验收。
候选阶段的未完成项及早期验证已移入[历史归档](../control/docs/archive/2026-09-shared-root/pico-simple-calibration-history.md)，不代表当前状态。

新模式明确采用 `symmetric_local_y` 模型假设：双手柄镜像握持，两局部坐标系 Y 为左右轴。
上臂=`height*0.155882`，前臂=`height*0.152941`，腕掌距离=`height*0.037037`，双侧共用，
右 TCP 为 `t_R=M*t_L; R_R=M*R_L*M`，`M=diag(1,-1,1)`。
不拟合旧右侧、不要求右侧独立采样、不生成臂长实测的虚假质量字段。
该模型约定仍需首次现场检查轴向，不能用软件通过代替硬件验收。

### 换人：身高与臂长适配

身高参数控制上游人体骨长模板；下游共享根映射另根据输入骨架的肩宽、总臂展
估计映射尺度。这两层不能混为“测得了真实臂长”：同身高的人仍可能有不同肩宽、
上臂／前臂比例和握持偏移。简化模式左右共用模板，不能表达双侧独立实测差异。

2026-09-20 的[合成人体映射实验](../control/docs/verification/synthetic_body_mapping_20260920.md)
使用 1.62 m、4471 帧录制，在 M0 输出后构造其他体型。等比例 1.45～1.95 m
映射目标几乎不变；只改骨段比例或肩宽则产生厘米级差异，全部 11 组最终几何闭合。
这是下游映射的离线结果，不验证上游模板截断、真实 PICO 估计、IK 或真机安全，
也不保证该身高区间内所有真人和动作都表现一致。

新人员操作顺序：

1. 先停止旧执行端和旧 PICO 输入，不在遥操中切换人员。
2. 用 `pixi run -e tracking setup-pico --user NEW_USER --height-m 1.70` 重新采集；
   人员名和身高替换为实际值，不复制其他人的 TCP。
3. 确认发布后检查骨架：自然姿态、前伸、侧展、屈肘和转腕时掌心位置与方向是否合理。
   如果伸臂时掌心明显偏离或骨架过早伸直，不以身高模板“已通过”代替检查。
4. 骨架确认后另开终端 `bash teleop.sh --sim --user NEW_USER`，先检查机械臂窗口，
   再按 S 接入；仿真检查不等于真机放行。

明显体型不匹配时保留录制，进一步评估双侧实测标定路径；不要手工修改派生 YAML
或绕过配置完整性检查。

### 前置条件

- 完成 README 的项目安装；以下 Pixi 任务统一使用 tracking 环境的 Python 和 ROS SDK，
  不依赖 `.venv`，不混用系统 ROS 的 Python ABI。直接 Bash 入口仍需显式配置兼容环境。
- 戴好头显，运行手柄路线 APK，连接 USB 并授权 ADB；左右手柄握持应镜像一致。
- 退出已有执行器和 PICO 输入会话，天机真机保持未使能，不启动 Manus。
- 更新代码后执行 `pixi run -e tracking build-tracking`，安装新的派生 artifact 模块。

### 推荐：单终端向导，完成后自动显示骨架

先停止已有遥操执行端及 PICO 输入，戴好头显、打开 APK 并完成 USB 调试授权。
首次或更新后先执行 `pixi run -e tracking build-tracking`。在工程根目录运行：

```bash
pixi run -e tracking setup-pico
# 或预填人员名和身高（示例必须替换）
pixi run -e tracking setup-pico --user NEW_USER --height-m 1.70
```

向导询问人员名/身高及镜像假设，检查图形桌面、USB 和已有输入，再后台启动原始 driver，
等待左右手柄及原始骨架的时间戳持续更新后，进入既有左侧 TCP 交互采样。
展示结果后提示“回车确认使用并显示骨架，输入 q 取消”：回车发布本次结果，q 取消；
其他输入重新提示，EOF/Ctrl+C 中止而不发布。右 TCP 和骨长的估计语义不变，不自动确认采样质量。
此交互仅用于 `setup-pico`；手动 `calibrate-pico` 仍须输入 `publish`，不会自动打开骨架。
随后停止本次原始 driver，用本次确切标定版本启动 M0 MuJoCo 骨架窗口与 TJVR 输入，
不启动机械臂仿真、不接入遥操。检查骨架后另开终端运行 `pixi run sim`，按 S 接入。

取消、采集失败或超时会停止本次原始 driver，不打开旧标定；输出留在本次采集目录，
driver 日志在 `logs/pico-setup/setup-*/driver.log`。发布成功但骨架启动失败不会撤销标定，
按错误提示查看保留的 tmux 窗口，排查后可用 `pico-simple --user` 重试。
成功后输入会话继续运行，结束时先退出机械臂仿真，再使用下文 `--stop` 命令。

下面保留手动双终端操作，**选择一种流程，不要与向导同时启动 driver**。

### 手动终端一：仅启动原始 PICO 输入

```bash
cd ~/current_robotics/PICO_Hand_Tracking/tianji_teleop
adb devices -l
pixi run -e tracking pico-driver
```

此入口自动设置 ADB TCP 9999，不启动 M0、IK 或机器人执行器。

### 终端二：为新用户标定

```bash
cd ~/current_robotics/PICO_Hand_Tracking/tianji_teleop
pixi run -e tracking calibrate-pico \
  --user NEW_USER \
  --height-m 1.70 \
  --accept-symmetric-model
```

替换人员名与实际身高（米）；1.70 只是示例。
`--accept-symmetric-model` 接受上述模型/握持假设，不授予真机运动权限。

1. 按既有 TCP 提示进行左手位置采样，再采左侧姿态。姿态参考动作仍可要求双臂前伸，
   但只拟合左侧，不采右侧参数。
2. 不再采集腕部旋转。按身高计算腕掌距离；162 cm 对应 0.05999994 m。
3. 工具生成双侧派生模型并显示摘要。TCP 仍以掌心为原点；M0 沿掌局部 +X 反推腕心，
   即 `p_wrist = p_palm - R_palm * [distance, 0, 0]`，不更改 TCP 平移。
4. 输入 `publish` 发布到新模式的独立指针；输入其他内容取消发布并保留采集文件。

任何采集失败均不更新指针。每次命令创建新版本，从 TCP 开始，不自动复用失败的部分采集。
新用户目录只在命令运行时创建，本次代码实施没有创建任何真实人员版本。

新 manifest 为版本 2，只将左侧掌心 TCP 列为实测来源；双侧腕掌距离标记为身高估计，
不生成虚假的实测 RMS。已有版本 1（左腕实测）仍可加载，原始双侧标定入口不变。
已有版本不自动迁移。zhoujie `cal-91c71441c51b47db9366390c96c96199` 经用户明确要求，
已覆盖为版本 2，双侧腕掌距离 0.05999994 m；TCP 和上、前臂保持不变。
原始完整配置备份在同级 `pre-height-wrist-backup-O6H2m8AI/`，没有删除实测证据。

### 切换到新输入模型

先在终端一 Ctrl-C 退出原始 driver，避免重复采集，再执行：

```bash
bash bash/run_pico.sh --user NEW_USER
```

此入口只启动 driver/M0/ROS 输入适配，PICO 世界 X 偏移保持 +0.20 m。
不启动仿真或真机执行器。确认双侧骨架长度、掌心位置及转腕轴向后，再按原有仿真流程测试。
共享根实验功能不会因此启用。

另开终端执行 `pixi run build`、`pixi run sim`，启动默认 DLS＋Ruckig 机械臂仿真；
它读取 `/pico/arm_input` 的原子 ROS 输入，不自行选择或重写人员标定。按 S 接入前先检查骨架。
tmux 各输入窗口显式继承本次选择的 Python/ROS SDK，避免已存在的 tmux 服务残留旧环境。
启动会拒绝已有会话或残留 PICO 输入进程，不再按进程名自动杀进程或重启旧会话。
新会话最多等待 30 秒：输入窗口必须存活，且收到唯一、同机、递增而新鲜的 ROS 输入，
tracking epoch／撤销代际一致，才报告启动成功。
这只证明输入发布就绪，不代表机械臂已经执行或获得运动授权。
超时、窗口退出或重复状态发布者会返回非零状态，保留窗口供查看日志，不自动清理。

停止该入口管理的会话：

```bash
bash bash/run_stop_pico.sh
```

停止命令只接受带有当前工程所有权标记的会话；旧版本未标记或其他工程的会话会被拒绝，
需先人工确认归属并停止，不能通过补写标记自动接管。
旧 `cleanup_tianji_pico_processes.py` 现仅做只读冲突检查，不再发送任何终止信号。

### 保存位置与旧流程隔离

```text
profiles/NEW_USER/pico-simple/cal-<id>/       # 左侧实测＋派生文件＋模型 manifest
profiles/NEW_USER/pico-simple/active.json    # 独立的新模式指针
profiles/NEW_USER/recordings/simple-cal-<id>/ # 采集记录
```

`profile.yaml`、原 `pico/` 和 `symmetric_max` 快照均不修改。
当前入口 `bash bash/run_pico.sh --user NAME` 显式选择该简化标定指针，不自动改写人员数据。
只读查询使用：

```bash
pixi run python src/teleop_inputs/pico_controller/scripts/calibrate_pico_simple.py --user NEW_USER --resolve
```

运行时对新 schema 验证完整 bundle、左侧源哈希、模板长度和镜像推导结果；不接受混合版本、
改写右侧派生参数、缺失 manifest 或跨目录 TCP/wrist。旧 measured schema 的门限不变。
改身高或重新标定时重新运行新入口，不手工编辑派生 YAML，也不对派生右 TCP 做旧姿态服务更新。
缺少 `active.json` 时拒绝启动，不按最新目录或历史人员名猜测配置，也不回退到全局标定。
重新完成标定并明确确认发布后才建立指针（向导回车；手动入口输入 `publish`）；
不会自动发布历史版本。

### 本次完整入口的软件验证

单终端向导验证（2026-09-20）：`pixi run --locked -e tracking test-setup-pico`
42 项通过（0.25 秒），不依赖真实人员目录；覆盖成功、取消、不发布、超时、采集中断、
清理失败阻止切换、精确版本传递及骨架启动失败返回。`setup-pico --help`、Shell 语法和
差异格式检查通过。未运行真实驱动、ROS 节点或 MuJoCo GUI，设备流程仍待实际佩戴者验证。
已删除人员配置后的全量标定测试仍存在旧个人样例依赖，本轮未宣称该组合回归通过。

启动安全修复验证（2026-09-20，后续轮次）：

- `pixi run --locked -e tracking test-pico-simple`：129 项通过（2.08 秒），新增覆盖
  冲突检查不发信号、会话归属、就绪计数/时间戳、死窗口、启动失败返回码及 Python 模式隔离。
- `TIANJI_REAL_PYTHON=/nonexistent/review-python pixi run --locked sim --help` 成功，
  确认仿真不再被真机解释器覆盖；真机和数据采集仍保留原覆盖语义。
- 本轮未运行设备、真实 tmux 会话或就绪探测 ROS 节点；真实输入就绪仍待现场验收。
  未改 IK/C++ 控制逻辑，未重跑原生完整回归；Shell 语法和差异格式检查通过。

Pixi 接线补齐验证（2026-09-20）：

- `pixi run --locked -e tracking build-tracking --parallel-workers 2`：4 个包构建成功。
- `pixi run --locked -e tracking test-pico-simple`：105 项通过，包含真实 ROS 模块导入的
  纯加载测试、取消发布后的缺失指针拒绝、身高模板与镜像校验及 tmux 环境转义。
- 两个新用户入口的 `--help`、修改脚本的 `bash -n` 和 `git diff --check` 通过。
- 未连接设备、启动 ROS 节点或重启遥操；未改人员档案、自动发布历史版本或进行真机验收。
  下方较早验证记录不是本轮重新执行的结果。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  tests/test_pico_simple_lifecycle.py tests/test_pico_simple_calibration.py \
  tests/test_symmetric_profile.py tests/test_teleop_profile.py \
  tracking/src/pico_bridge/test/test_pico_calibration_artifact.py \
  tracking/src/pico_bridge/test/test_pico_palm_tcp_runtime.py \
  pico2_hands/tests control/tests/test_shared_root_contract.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  tests/test_pico_simple_ros_loaders.py \
  tracking/src/pico_bridge/test/test_pico_arm_geometry_calibrator.py \
  tracking/src/pico_bridge/test/test_pico_palm_wrist_calibrator.py \
  tracking/src/pico_bridge/test/test_pico_palm_orientation_service.py -q
```

本轮分别为 163 项通过（6.12 秒）和 22 项通过（1.02 秒）。后者使用本机与 ROS Humble
匹配的 Python 3.10；最初用项目 Python 3.11 执行 ROS 相关测试因 ABI 不匹配在收集阶段失败，
没有将该次运行算作通过。纯加载测试没有 `rclpy.init`、节点创建或硬件连接。
Shell 语法、`git diff --check` 通过。新用户生命周期以假采集器验证，仅写 pytest 临时目录；
已有 `profiles/`、`pico2_hands/` 和真机驱动零差异。未实际运行 ROS 构建/安装或设备采集。
软件覆盖源文件篡改、左右版本混用、TCP 在腕心采样期间改变、取消发布、采集失败、并发锁
以及路径别名拒绝；这不替代真实设备的握持/轴向验证。
