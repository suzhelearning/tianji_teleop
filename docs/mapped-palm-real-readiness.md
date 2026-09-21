# mapped-palm 真机执行层补齐

本文现场证据来自合并前的 `tianji_teleop_full/tianji_teleop`，所述日志保留在该来源目录，
未复制到当前工程。当前目录的软件验证见[迁移验收记录](migration-verification-status.md)，
来源现场测试不等于本目录已经完成现场验收。

本文记录实现与验收范围，不构成后续真实运动授权；SDK、驱动和 IK 核心保持原实现。

## 已接入：到位且静止

多次 Enter 的 ALIGNING/READY/HOMING 及独立回 Home 共用反馈静止判定。
以新鲜实测位置和原有反馈时间戳差估算速度，默认上限 0.03 rad/s；
位置到位、速度达标并持续配置的 settle_time_s 才完成转换。
重复帧不累计静止时长；回退/长间隔/非有限数/超速重置窗口。
估计不是硬件速度传感器，不代表物理停止已经验收。

## 已接入、待实机验收：PICO 短时断流保持（2026-09-16）

mapped-palm 双臂入口默认 `--mapped-palm-dropout-policy hold-300ms`；
显式选择 `stop` 可恢复失鲜即停。此选项与 `--mapped-palm-resync-policy stop|bounded`
独立，不放宽姿态突变、坐标系重置、IK 失败或硬件反馈保护。

- 仅已使能的 TELEOP 可保持；WAITING、ALIGNING、READY、HOMING 不接受该例外。
- 原生控制器在控制周期发现有效源年龄达到 50 ms 时暂停求解推进；
  执行端保持所有选中设备最后下发角度，不继续追逐旧 IK 目标。
- 300 ms 截止从最后成功应用输入的 `min(receive_monotonic_ns, bridge_send_monotonic_ns)`
  起算，不从发现断流时起算。重复输入和重复保持事件不能续期；迟到的新帧也不能撤销已超时故障。
- 窗口内恢复同 epoch、同流代次的最新有效输入后，按既有最大速度衔接；
  不补发积压目标、不赋予新的使能权限。恢复姿态仍须通过原有跳变门控。
- 无效包、输入身份变化、按键事件、IK 未接受、控制通道异常均不是可恢复断流。
  双手输入时效、硬件反馈时效/健康/使能状态、限位、跟踪误差和标定锁保持原约束。
- 保持期间 Enter 请求回 Home 会停止会话，不在失鲜状态下追加回位；Ctrl+C 仍直接停止。

私有 MRE1 version=3 在既有 80 字节头和 56 字节标定区后增加
`int64 input_valid_ns`（偏移 136，总头长 144）；未启用标定时标定区为零。
事件 9 表示纯断流保持，双臂 ready 保持为假，由执行端严格校验后允许保持。
默认策略因此要求上下游同时更新并重启；外部 TJVR/TJRC 格式不变。

本机离线验证：原生套件 24 项通过、1 项跳过，执行端套件 183 项通过，
仿真套件 15 项通过。隔离端口、合成 PICO 输入和反馈的端到端检查确认约 120 ms
断流保持原命令、最新目标恢复仍受执行限速约束、源年龄超过 300 ms 后停机。
另修复周期内跨越 50 ms 时误归为 IK 失败的时间快照不一致问题。
这些结果不代表联合真机断流恢复已完成现场验收。

## 已接入、待实机验收：真机有界重同步（显式选项）

参数 `--ik-backend mapped-palm --mapped-palm-resync-policy bounded` 选择新策略。
默认 `--mapped-palm-resync-policy stop` 保留原先停机策略；SPARK 不接受 bounded。
该选项不是实机运行授权；勿在未经预检时直接启动运动。

父进程创建独占 Unix SOCK_SEQPACKET socketpair，并通过 pass_fds 传给子进程。
MRE1 信封携带随机会话标识、输出周期/序号、时间、epoch、流代次和原始 TJRC v2。
一条消息原子交付事件与命令，防止分离 UDP 通道错配；既有 UDP/TJRC v2 格式不变。
执行器严格校验长度、身份、序号连续性、周期、时间和命令一致性；关闭/堵塞/缺帧均失败停止。
这是本机持有文件描述符的信任边界，不是网络身份认证。

事件：1 正常，2 姿态/位置突变待确认，3 同 epoch 重同步确认，4 输入无效/失鲜，
5 IK/输出未接受，6 epoch 重置，7 按键事件，8 重同步超时；version=3 另有事件 9 纯断流保持。
接收端沿用原突变门控；事件 2 时暂停原生求解推进，不宣称 ready。
仅 TELEOP 可在同 epoch 的事件 2 下保持执行层最后下发角度（包括所选手部），
最长 100 ms；仍检查命令与反馈时效、限位、跟踪误差和其他设备 ready。
新输入确认或返回原连续流后，执行层从最后命令按原最大速度衔接，不瞬跳。
ALIGNING/READY/HOMING 不接受这项临时例外。其他故障仍停止、失能、结束会话。

该策略与仿真自动继续路径隔离。没有 SDK/驱动改写，没有扩大速度或限位。
静止判定及保持状态均不替代物理制动/碰撞安全验证。

## 已接入、已完成一次双臂现场流程测试：mapped-palm 的 C 标定

显式选择 `--ik-backend mapped-palm --mapped-palm-xz-calibration`。
该真机选项需要 arms 和 `--confirm-real`，不接受 SPARK/inspect/仅手模式。
它与 `--mapped-palm-resync-policy` 独立；不传 bounded 仍使用 stop。

操作：在**启动终端**按 C（不用回车），双臂向前水平伸直并稳定两秒。
C 只采集输入、计算 X/Z 偏移；参考为模型左右臂 J2=-90°、其他轴为零的 TCP。
不改 Y/姿态，不使能设备。标定未成功，Enter 不获得任何运动权限。
成功后第一次 Enter 请求锁定当前标定版本；收到新周期的锁定确认后重新检查
输入、限位和真实反馈，再进入原有慢速对齐；到位静止后第二次 Enter 遥操。
启用过程中再次 Enter 中止，Ctrl+C 仍为停止。锁定后 C 拒绝，不修改标定。

私有事件通道的 version=2/3 帧携带标定版本、epoch、状态与四个 X/Z 偏移；
版本/epoch 不匹配或状态失效时拒绝使能。epoch 变化使标定失效，执行中按原故障流程停止。
标定失败可在未锁定且未使能时重新 C；没有自动复用上一会话结果。
监视窗口显示标定阶段，但真机按键由启动终端接收，不是仿真窗口的 C/S。
启用此功能仍保留原来的 SDK、反馈读取及多次 Enter 行为；现场验证范围见下文。

### 真机仅双臂入口

先保持正确的 PICO＋VR 手柄输入链路运行，退出占用输入端口的仿真/其他执行器。
在工程根目录的桌面终端执行；所选 Python 环境需已配置本工程及硬件 SDK 依赖：

```bash
export TIANJI_PYTHON="$PWD/.pixi/envs/default/bin/python"

# 只读预检：失败时不要继续使能。
"$TIANJI_PYTHON" -m tianji real --devices arms --inspect
```

确认预检通过、现场活动空间安全且硬件急停可用后，单独执行：

```bash
bash teleop.sh --real \
  --devices arms \
  --ik-backend mapped-palm \
  --mapped-palm-xz-calibration \
  --mapped-palm-resync-policy stop
```

`--devices arms` 覆盖脚本默认的 all，不启用真实灵巧手，不要求 Manus 输入。
启动终端 C → 标定成功 → 第一次 Enter 慢速对齐 → READY → 第二次 Enter 遥操。
TELEOP 时第三次 Enter 慢速回 Home，到位后失能退出；对齐/回位中 Enter 或 Ctrl+C
直接停止、失能，不追加回位。失能可能失去支撑，应提前处理负载。

### 2026-09-15 现场证据与限制

用户现场确认本次遥操可用。会话 `logs/real/20260915-115501-72109-491698/console.log`
记录了 C 标定成功、锁定、ALIGNING、READY、TELEOP、HOMING、HOME_REACHED、
Robot released，入口退出码为 0。仅覆盖 PICO＋VR 手柄、mapped-palm、C 标定、
`stop` 策略和真实双臂；不代表 bounded、Manus/Hand2 联合真机或长期稳定性验收。
现场退出时出现 GLFW 未初始化警告；不能仅凭该警告认定崩溃，也尚未修复其根因。

## Viewer 退出清理异常报告（2026-09-15）

父进程现在检查只读 Viewer 的退出结果：非零退出码（含信号退出）、子进程 ERROR、
正常退出超时而需强制终止、kill 后仍无法确认退出，均报告清理失败。
退出未确认时保留进程句柄以便重试；状态管道仍释放。正常关闭可重复调用，
启动失败时保留原始错误并附加清理错误，不用清理错误覆盖启动原因。

真机执行器先停止/释放硬件，再关闭 Viewer；Viewer 清理异常写入会话
`cleanup_errors`，入口返回非零状态，不再把异常清理误报为成功。
这不改变硬件驱动、IK、C/Enter 状态机，也不等于修复 GLFW 警告。

该修复后运行 `.pixi/envs/default/bin/python -m unittest discover -s real_robot/tests -q`：
170 项通过；包括 Viewer 非零退出、模拟 -11、强制终止、退出未确认及入口日志传播。
`git diff --check` 通过。测试使用 stub/mock，不加载真实驱动；修复后的实际图形窗口
退出尚未重新现场验收。新增事件/标定字段在诊断录制中的补齐不在本次修复范围内。

## 尚未接入：跨会话标定文件传递

需显式保存/加载标定文件并校验模型/映射配置摘要、来源身份和 epoch；
不可自动沿用上一场仿真的 X/Z 偏移。当前真机入口没有该文件加载能力；
上面的现场 C 标定仅在本会话内生效。

## 验收边界

离线 mock、录制回归和真实输入仿真均不替代真机验收。真机应在另行明确授权后，
按只读检查、低速对齐、回位、慢速遥操、快速转腕逐项验证。
