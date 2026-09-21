# Franka DLS / Ceres＋Ruckig：交互仿真与当前验收清单

## 退出与 Manus 预检边界修复

仿真监督进程收到 SIGINT／SIGTERM／SIGHUP 后最多等待 5 秒正常退出；随后仅向
该次 `start_new_session` 子进程组发送 TERM，等待 3 秒，再 KILL 并最多等待 2 秒。
子进程退出无法确认时不继续自动清理 PICO，明确提示人工检查。
清理辅助进程也使用独立进程组与 20 秒超时，并有同样的终止兜底。
强制退出可能丢失尚未排空的录制，不标记为正常录制完成。

Manus 启动与 `--check` 均检查 Python／采集器执行权限、采集器及 SDK 的 ELF 头、
`ldd` 动态依赖和实际解析的 SDK 路径。缺少依赖、LFS 占位文件、错误 SDK 路径、
依赖检查超时均拒绝；检查不执行采集器 main、不初始化 SDK 设备或 ROS 节点。

## 2026-09-20：可选 Manus 双手接线

`bash teleop.sh --sim --user NEW_USER --hand-teleop` 保留双臂 DLS＋Ruckig，
同时接收现有 TJH2 v2 手部流。Ceres 可显式加 `--ik-backend ceres`。
后续按用户要求，DLS/Ceres 启动器默认开启手部接收，`--hand-teleop` 可省略；
仅双臂显式用 `--no-hand-teleop`，不会绑定手部端口。不自动启动 Manus，缺手部输入时保持。
Manus 独立运行：`pixi run manus --calibration-user MANUS_USER`，必须使用实际佩戴者
已有的双侧标定，可先 `pixi run manus --list-calibration-users` 只读查询。
旧 `bash manus.sh --user NEW_USER` 人员档案映射保留，没有静默回退或自动发布。

- 原生手部接收仅绑定本机；手部和 PICO 端口不可相同；硬件指令导出仍禁止。
- S 接入 TELEOP 后双手才跟随。H 仅回双臂 Home，双手保持；P、制动、Home、故障期间均保持。
- 所有已接受的 S/H/P 状态变化清理手部历史；不回放接管前的缓存样本。
- 保留 TJH2 序号、CRC、源时间、新鲜度、插值和模型限位。左右失鲜独立保持；
  新鲜手部输入在 TELEOP 内恢复，不额外请求 S，也不因单侧手失鲜单独停止双臂。
- Ruckig 仍只用于双臂；没有新增手指 Ruckig、碰撞检查或手部完整原始录制。
- 仿真启动器不创建 Manus 进程，不抢占或切换发布者；须确保手部只有一个发送器。

新增离线覆盖：启动选项／端口校验、旧档案和显式 Manus 标定选择、所有恢复阶段手部门控；
原生 DLS/Ceres Viewer 实际接收合成 UDP 数据但在 WAITING 不移动手指。
现场 S/H/P 操作与真实 Manus＋PICO 联合仿真尚未验证，不等于真机验收。
初次接线时 Manus 仍依赖缺失的根 `.venv`；后续已改为独立 `manus` Pixi 环境，
使用本仓库 tracking ROS SDK。`pixi run prepare-manus` 安装本地原生扩展并编译采集器，
不启动设备；SDK 为 LFS 占位文件时明确拒绝构建。启动仍必须具备实际佩戴者双侧 `.mcal`。
`pixi run manus --calibration-user NAME --check` 只检查依赖和模型，不启动设备／ROS 节点／UDP。

Pixi 修复轮次验证：`pixi install -e manus` 成功并更新锁文件，
`pixi run --locked -e manus build-manus-retarget` 完成本地 editable 安装与原生扩展编译。
在 Manus Python＋tracking ROS 环境内导入 rclpy、std_msgs、Pinocchio 3.8.0、nlopt 成功，
左右 HandRetargeter 均可离线初始化。禁用外部 pytest 插件后，手部内核／桥接／Manus 输入
相关测试 35/35 通过，启动器测试 6/6 通过，Shell 语法及差异检查通过。
本轮没有重跑 C++ 控制器全套测试；下方 106/106 属于前一接线轮次。
采集器构建仍因 SDK 为 Git LFS 占位文件被明确拒绝；`yp` 双侧标定缺失，启动在设备操作前拒绝。
没有将现有 `yq` 或其他人员的标定替代 `yp`，没有连接设备或运行现场遥操。

本轮验证：`pixi run --locked build` 成功；`pixi run --locked test-native` 106/106 通过
（164.90 s）；`pixi run --locked test-sim` 34/34 通过；
`pixi run --locked -e tracking test-sim-user` 62/62 通过。
`bash -n manus.sh`、`git diff --check` 及只读 Manus 标定列表通过。
测试使用隔离的合成输入／假 SDK 启动树，没有连接 Manus、PICO 或真实执行器。

## 2026-09-20：H 回 Home 误入 FAULT 修复

现场录制 `dls_interactive_1fbva9ez` 在第 7779 周期进入 FAULT。离线取前一帧，
右臂加速度约 40.876 rad/s²，符合 DLS 对应 90 rad/s² 限值，却超过未使用的
SPARK 中间参考 15.708 rad/s² 限值。原 H 流程在启动恢复后重置该参考，导致误拒绝。
现为共享根 DLS/Ceres 增加独立映射会话重置，保留映射历史/确认清理，但不重置
未使用的 SPARK 运动参考；DLS/Ceres 制动与 Home 仍执行自身状态和轨迹限位检查。
旧 SPARK 重置路径保留。H 接受/拒绝及映射重置失败现在有终端提示。

本次 `pixi run --locked build` 成功；相关 CTest 9/9 通过（44.31 s），覆盖
共享根、SPARK guidance/reference、DLS/Ceres controller/viewer、Ruckig 和恢复状态机。
新增原生回归覆盖双后端的跨限值回 Home、到位不自动遥操、超限及非有限状态拒绝。
未跑完整 CTest，未进行真实 PICO 输入现场复测或真机验收。

当前流程：先按[标定与身高输入说明](../../../docs/pico-simple-calibration.md)启动唯一 PICO 输入，
再运行本页机械臂仿真。模型/限位和移植差异见[后端说明](shared_root_ceres_f615b8c.md)，
映射契约见[映射设计](../spark_shared_root_retarget.md)。

2026-09-20：同一交互入口已支持 `bash teleop.sh --sim --ik-backend franka-dls`。
该选项加载 `qp_ik_pico_shared_root_dls.yaml`，使用独立 Franka DLS 参数与 Ruckig
限制；S/H/P 状态机和实时 Ruckig 曲线相同。随后按用户要求将 `teleop.sh --sim`
默认后端切换为 Franka DLS＋Ruckig；Ceres、SPARK 保留显式选择，真机默认不变。
本轮核对源工程 `f615b8c2931601957315d1d3ea7f8aad8bb369a6`：两个 DLS 核心
实现去除隔离命名差异后逐字一致；当前 iterative_dls、pico_ee_franka_dls 配置字段
与源 `qp_ik_pico_ee_franka_dls_ruckig_mujoco.yaml` 对应值一致。
保留本工程双臂事务提交、失联停止、模型状态与禁止关节导出的安全语义，
不宣称两工程完整应用行为等价。候选 FK/Jacobian 仍使用 Pinocchio。
本轮构建成功；相关原生回归 11/11（17.53 s），Python 仿真回归 18/18（4.065 s）；
未重新运行全部跨工程数据集 A/B，未进行真机验收。

推荐入口：`pixi run build`，然后 `pixi run sim`（DLS）或
`pixi run sim --ik-backend ceres`。Pixi 任务使用本环境 Python 和标准 `control/build`。
仅在 DLS/Ceres 模型仿真中，可显式追加 `--sim-allow-pico-jumps` 跳过接收端的
位置/姿态跳变拒绝，例如 `bash teleop.sh --sim --user NAME --sim-allow-pico-jumps`。
默认仍检查跳变；该选项不修改配置文件，不允许关节导出，不适用于真机或 SPARK。
CRC/数据合法性、乱序/epoch 检查、输入超时、映射门控和 Ruckig/关节限位保持有效。
异常目标仍可能进入 IK；平滑器限制运动变化，但不保证目标正确。启动时会打印警告。
已发布简化标定时可运行 `pixi run sim --user NAME`，或 `bash teleop.sh --sim --user NAME`。
该选项先用 tracking 环境解析人员标定，再启动输入或核对已有会话的工程、版本和内容指纹，
通过就绪检查后才打开机械臂窗口。仅支持 DLS/Ceres、direct、有窗口和端口 15000。
就绪检查同时要求当前 M0 窗格对应的骨架渲染心跳持续推进；骨架窗口退出时，
M0 会话会报告失败并退出，不再仅凭估计器存活报告成功。升级前启动的旧输入会话
需要显式停止再启动，才能提供该心跳。
缺标记的旧会话或不同人员/版本一律拒绝，需显式 `pixi run stop-pico` 后重试。
带 `--user` 时，本次新建的 PICO 会话随机械臂退出自动清理；复用的已有输入保留，
结束后用同一停止命令。不带 `--user` 仍只接收已有输入，不管理其生命周期。
自动清理以创建时写入的唯一仿真 token 为条件，不按会话名称盲目停止；Manus 始终保留。
正常退出、Ctrl+C／SIGTERM 和启动失败均执行该清理；SIGKILL／断电无法保证。
本轮软件验证：仿真测试 40/40、人员启动测试 66/66 通过，包含独立 tmux 测试服务器
上的未标记会话／替换 token 保留和匹配 token 清理。未操作现场 PICO 或 Manus，
未改 C++ 控制器，未重跑原生回归或进行真实输入验收。
已有 `.venv` 或显式 `TIANJI_PYTHON` 的入口 `bash teleop.sh --sim` 继续可用。
不传 `--ik-backend` 时仿真使用 Franka DLS＋Ruckig；旧 SPARK 仿真请显式选择
`--ik-backend spark`。默认 DLS 为 direct 窗口，现已默认开启手部接收，不支持 headless/dynamics。

本入口只支持 direct 模型参考仿真，不是动力学验证或真机验收。
在原生 Viewer 内运行在线 DLS/Ceres IK 和 Ruckig，不导出 TJRC；启动器默认接收 Manus 双手，
可显式 `--no-hand-teleop` 禁用，见本页新增接线说明。
PICO 输入和骨架窗口由既有 PICO 启动入口单独管理；此入口不会重复启动设备。

先点击机械臂窗口，再使用键盘：

S 接入默认先低速接近实时目标：关节速度不超过 0.35 rad/s、加速度
0.5 rad/s²、jerk 2 rad/s³（原限值更低时仍取更低值）。每侧末端位置误差
小于 3 cm、姿态误差小于 0.1 rad，且关节目标差小于 0.08 rad，持续 0.2 秒后，
用 **0.5 秒**平滑恢复正常遥操限值。双臂独立判断接近，但仍双侧事务提交。
不重置运动状态、不跳关节位置；大幅持续移动或无法接近的目标会延长低速阶段。
H/P 和失联保护仍有效；该过渡只在交互模型仿真接受 S 时启用，不改普通后端默认行为。

- 启动：WAITING，保持配置中的初始 Home，不自动跟随输入。
- S：输入新鲜且处于静止的 WAITING/HOLD/HOME_REACHED 时接入；重置映射会话，等待新输入。
- H：先按当前 Ruckig 限制制动，再低速平滑回启动时的 Home。
  Home 段上限不超过 0.5 rad/s、0.8 rad/s²、3 rad/s³；原配置更低时取更低值。
  制动段沿用遥操限制，S 低速接入参数保持不变。
- P / Space：制动并保持，也可以中止回 Home。
- 回到 Home 后仍等待 S，不自动恢复。输入超时、流重置或有效目标求解失败也会停止接入。
- 既有 PICO 按钮暂停仍有效；按钮恢复仅恢复输入会话，不能替代 S 授权。
- FAULT：保持并检查日志，不能用 S 绕过；修复原因后重启会话。
- Esc 或终端 Ctrl+C：退出并关闭录制。

Home（度）：左 `[55,-65,-70,-60,60,0,0]`，右 `[-55,-65,70,-60,-60,0,0]`。
关节空间回 Home 不包含碰撞规划，不能直接迁移到真机。

每次启动独占创建 `recordings/shared_root/dls_interactive_*` 或 `ceres_interactive_*`，保存 runtime.yaml、
input.tjvr 和遥测。仅会话副本启用共享根，正式配置仍默认关闭。
窗口标题和标准输出显示 WAITING / TELEOP / BRAKING / HOMING / HOME_REACHED / HOLD / FAULT。
正常等待、制动、Home 和保持的 `hold_reason=none`，但 `accepted=false`，不代表跟踪授权。
恢复故障为 `solver_failure`。主遥测新增 `simulation_phase`：-1 为旧入口，
0 WAITING、1 TELEOP、2 BRAKING、3 HOMING、4 HOME_REACHED、5 HOLD、6 FAULT。
DLS 输出前缀为 `DLS_SIM`，Ceres 为 `CERES_SIM`。

实时关节图：F2 显示面板，F3 切换位置/速度/加速度/jerk，F4 切换左右臂。
DLS/Ceres 青色曲线使用提交后的 Ruckig 参考速度和内部加速度；`avg jerk` 为相邻
控制周期内部加速度之差除以固定控制周期，不是瞬时解析 jerk，也不是速度二次差分。
蓝色模型曲线的加速度/jerk 仍由模型速度差分得到，不是独立的真机测量。
正常制动、Home 和静止保持均保留有效曲线；故障或未提交的轨迹不作为有效导数绘制。
红线显示该阶段限值，Home 阶段使用低速限制。CSV 新增左右 `ruckig_output` 标记；
S 接入及恢复正常限值的过渡期间，红线与 CSV 使用每侧已接受 Ruckig 样本的
实际生效限值，而非正常遥操配置中的固定上限。
标记为 1 时，`reference_qddot/reference_jerk` 使用上述 Ruckig 口径，旧记录不变。

验证入口：

```bash
pixi run build
pixi run test-native
pixi run test-sim
```

## 当前验收边界

- 已有：原生软件回归、录制回放、七类动作时间标注，以及使用者反馈的真实输入模型参考仿真。
  各次版本、数据和结果分别记录，不合并成一次“全流程验收”。
- 仍需：代表性人员/动作覆盖与启用门限评审；新 Pixi 启动就绪探测的现场验证；
  S/H/P、失联及恢复的完整现场检查。换人后需重新选择/发布标定并检查镜像轴与骨长。
- 未证明：执行器动力学、碰撞/自碰撞安全、硬实时周期、实际硬件反馈跟踪或真机验收。
- 共享根配置文件仍默认关闭；交互仿真只启用独占会话副本，禁止关节导出和灵巧手接管。
  `accepted`、几何有效和小 IK 残差均不等于末端准确跟踪或运动授权。
- 本次文档整理不重跑测试。下方数字为对应历史轮次的记录，不是本次验证结果。

## 历次软件验证

新增单元测试覆盖接入新鲜度、Home 双臂轨迹连续性和速度/加速度/jerk 边界、
禁止自动恢复、中止 Home 和单侧无效状态的故障锁定。
这些软件测试不替代佩戴者在真实输入下的 S/H/P 全流程确认。

此前交互入口验证（2026-09-20）：构建成功；CTest 106/106，通过用时 118.58 秒；
Python 仿真回归 17/17，通过用时 4.223 秒；`git diff --check` 通过。
真实输入启动检查会话 `ceres_interactive_oggv7iil`：4 秒接收 439 包，
785 条遥测均未接入；所有双臂参考/模型关节位置变化为 0，录制 finalized。
持续会话 `ceres_interactive_38zoh2c1` 已观察到窗口 S 接入，双臂均有 accepted 周期。
现场 H/P 全流程尚未确认；未进行动力学或真机验收。正式配置未开启，无提交或 push。

### Pixi 接线与诊断修复验证（2026-09-20）

- `pixi run --locked build` 成功：标准 `control/build` 启用 Ceres，保留 DLS 和 SPARK。
  仿真及对照工具不再依赖另建的 `build-ceres`；Python 使用 Pixi 环境，
  PyYAML 和匹配原生库版本的 MuJoCo Python 绑定已纳入锁文件。
- `pixi run --locked test-native -j 2`：106/106 通过，用时 119.66 秒。
  DLS/Ceres Viewer 测试检查 WAITING 的 `hold_reason=none`、
  `accepted=false`、`simulation_phase=0`，DLS 同时检查日志前缀。
- `pixi run --locked test-sim`：23/23 通过，用时 3.868 秒；
  覆盖标准构建路径、Pixi 任务依赖及缺少构建产物时的错误提示。
- 仿真三种后端及两个对照脚本的 `--help` 检查通过；`git diff --check` 通过。
- 本轮仅软件与无窗口回归，未启动或重启现场遥操，未进行真实输入、动力学或真机验收；
  未删除历史文档，未提交或 push。
