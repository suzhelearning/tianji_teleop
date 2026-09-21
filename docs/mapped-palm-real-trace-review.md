# mapped-palm 真实录制离线对照

这是合并前的历史对照记录，JSON 中路径与指纹属于当时的来源工作区；
迁移没有重新运行该完整录制。当前软件验证见[迁移验收记录](migration-verification-status.md)。

本轮仅测试代码和离线回放工具变更，未修改 IK、生产状态机或设备驱动，未启动硬件。
原始结果见 [JSON](mapped-palm-real-trace-result.json)。这不是完整调度器或真机验收报告。

## 数据及方法

- 数据：`pico_eggbeat_bandwidth_20260904_session01.tjvr`，10,682 帧，119.910905873 秒。
- 来源工程 commit：`a6ff1bd9a32e0a2b3dac2c131abf0cf0c2ea00b4`。
- 两个 worker 统一使用目标 `deployment.yaml`，即位置权重 90000 和当前部署 Home。
  模型/URDF 各自来自两边仓库，二进制及配置指纹见 JSON。
- 第一层：保留原始接收顺序、数据、序号、epoch，通过来源参考输入门控，在 200 Hz
  固定离线时序下分别计算两边 worker；尾部追加 250 ms 检查失鲜。
- 第二层：按录制接收节奏发送到目标实际 `mapped_palm_tjrc_controller`，独占临时
  localhost 输入端口和 TJRC 观察端口。没有接上执行器、MuJoCo 或 SDK。
- XR 源时钟与 PC bridge 时钟分别重定位到回放时钟，保留各自增量。
  文件缺少原始绝对接收时钟，不能恢复绝对网络延迟；本轮不是延迟等价验收。

## 第一层：核心数值一致

共 24,033 个周期，左右 q、qdot、qddot、目标位置及目标四元数的最大绝对差值均为 **0**。
输入 live、采用序号、epoch、reset 标记和左右 accepted 标记也一致。

参考输入门控处理全部 10,682 帧：接受 10,596，拒绝 86，latest-only 覆盖 1,109；
无格式错误。23,982 个控制周期 input_live，左右 accepted 各 24,033。
accepted 是内核输出状态，不代表每个周期获得了执行授权。
内核 reset 标记出现 38 次，包含流重同步语义，不能说成发生了 38 次物理 tracking epoch 变化。

**结论：这段真实数据没有发现 IK 核心迁移数值误差。**
离线 deterministic 模式放宽墙钟求解预算；不据此证明生产实时预算内的求解性能。

## 第二层：发现状态恢复缺口

目标适配器正常退出（退出码 0），收到 23,426 帧 TJRC，其中 1,541 帧机械臂 ready。

| TJRC 序号 | epoch | 机械臂 ready |
|---|---:|---|
| 1 | 0 | false |
| 12 | 194 | false |
| 13 | 194 | true |
| 1554 | 195 | false，并保持到结束 |

与 `control/mapped_palm/src/joint_command.cpp` 的 `JointCommandArmReadiness::update`
实现一致：在曾经 ready 后遇到 reset，将 `inhibited_` 锁存；当前适配器没有解除该锁存的
H/R 生命周期。该问题应通过受控恢复流程解决，不能简单删掉 reset/新鲜度保护。

发送端最大回放调度滞后 3.341473 ms，仅描述本次测试工具，不代表控制器端到端延迟。
本层未与源调度器进行同相位输出逐帧比较，也未取得每个生产 QP 周期的诊断状态。

**结论：实际接线能收到输入并输出，但不能认定整段遥操连续性等价通过。**

## 回放工具问题及排除

首次 UDP 回放错误地共用 XR 和 PC 两个时钟的平移量；原始两时钟相差约 1840 秒，
导致 bridge 被判为未来数据、ready 全为 false。这个结果作废，不作为生产 bug 证据。
仅修正离线工具并增加独立时钟回归后，重新完整运行得到本文结果；未放宽生产保护。

## 复现

```bash
pixi run python scripts/compare_mapped_palm_trace.py \
  --source-root /实际路径/dexhand_deploy \
  --trace /实际路径/pico_eggbeat_bandwidth_20260904_session01.tjvr \
  --report /新的路径/result.json \
  --udp
```

report 独占创建，不覆盖已有结果。来源路径只用于本离线工具，不是生产依赖。
时钟转换回归：`pixi run python -m unittest discover -s control/mapped_palm/tests -p test_trace_compare.py`，3/3 通过。

下一步：补齐受控 Home/重新准备接管流程，再对照源调度器状态转换。
C 标定、Manus、MuJoCo 动力学、实际设备输入和真机运动均未由本测试覆盖。

## 后续修复验证：仿真受控恢复

已在 mapped-palm **仿真入口**接入独占 stdin 恢复协议：P 保持、R 接受静止模型反馈并重置、
新的同 epoch/流代次输入到达后 S 接管。不删除锁存规则，真机入口不启用此协议。
R 反馈需新鲜（50 ms 内）、有限、关节位置在模型限位内、关节速度不超过 0.03 rad/s；
仿真父进程另外要求连续静止 0.3 s。新执行阶段会重新初始化 IK，不沿用冻结前的速度/加速度。

本次重放同一录制前 15 秒，共 1,341 帧；用**合成静止反馈**检验恢复协议，不冒充实物反馈：

- 收到 TJRC 2,934 帧，ready 2,750 帧。
- 序号 14：epoch 194，ready=true。
- 序号 1546：epoch 195，ready=false。
- 显式 R、等待新输入、S 后，序号 1667：epoch 195，ready=true。
- 数据结束后序号 2885：ready=false，正常退出（0）。

回归：新增/移植测试 11/11（包含来源 worker 对照）；仿真测试 9/9；
原真机控制层离线测试 152/152；原生模型 CTest 1/1。
无真实输入 headless 仿真正常结束，收到 326 帧、序号无缺口。
完整 H/Home 轨迹、真机恢复授权、实际 Viewer 按键及真实设备仍需分别验收。
