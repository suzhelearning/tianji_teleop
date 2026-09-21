# PICO 连续跳变受控重同步设计

## 目标

修复 PICO 遥操作流在一次有效快速移动超过单帧跳变阈值后，持续与最后接受帧比较并长时间拒绝新帧的问题。保留对单帧毛刺的拒绝能力，同时让稳定的新位姿簇在三个连续帧后安全恢复控制。

本次只修改 PICO 输入流状态机及其重同步信号传递，不调整速度 QP、加速度 QP、OTG 参数、关节限值或碰撞约束。

## 已确认根因

`PicoTeleopStreamGate` 只保存最后一个被接受帧的双臂位姿。发生位置或姿态跳变后，该帧被拒绝，接受基准不更新；后续即使都稳定处于同一个新位置，仍会与旧基准比较并继续拒绝。

录制证据与该行为一致：

- 速度 QP 最长连续 stale 为 0.65 s，期间累计 57 次跳变拒绝。
- 加速度 QP 最长连续 stale 为 1.84 s，期间累计 162 次跳变拒绝。
- 两段事件均没有 malformed、CRC failure 或 reordered 数据报。

仅在接收门中接受新帧还不够。Viewer 必须知道这次接受代表输入不连续，从当前机器人位姿重置目标历史和笛卡尔 OTG；否则旧目标速度估计可能把新位置解释为超大速度脉冲。

## 方案选择

采用“稳定新簇重同步”：

1. 第一个超过 `0.15 m` 或 `0.60 rad` 的新帧继续拒绝，并建立候选簇。
2. 后续帧必须满足：
   - tracking epoch 不变；
   - sequence 严格递增；
   - 相对候选簇上一帧的左右臂位置变化均不超过 `0.15 m`；
   - 相对候选簇上一帧的左右臂姿态变化均不超过 `0.60 rad`。
3. 连续三个候选帧满足上述条件后，第三帧作为新基准被接受，并标记 `stream_discontinuity=true`。
4. 若候选帧之间再次发生超阈值变化，当前帧成为新候选簇的第一帧，计数重新开始。
5. 若输入回到旧接受基准阈值内，则清除候选簇并按普通连续帧接受，不触发重同步。
6. 新 tracking epoch 继续立即接受，并沿用现有 epoch reset 语义。

未采用的方案：

- stale 50 ms 后无条件接受下一帧：恢复快，但可能接受持续错误数据。
- 根据估计速度动态放宽门限：参数和状态更多，当前录制证据不足以安全标定。

## 状态与接口

### PICO 流门

`PicoTeleopStreamGate` 增加：

- 最后观察到的同 epoch sequence，用于让被拒绝帧也参与严格顺序检查；
- 候选簇上一帧的左右臂位姿；
- 候选簇连续计数；
- 固定确认帧数 `3`。

`PicoStreamDecision` 墌加 `stream_discontinuity`。普通连续接受为 `false`，稳定新簇的第三帧为 `true`。新 epoch 仍由 `epoch_changed` 表达。

`PicoTeleopFrame` 增加只在本机接收链路中设置的 `stream_discontinuity` 标志；它不进入 UDP 协议编码，不改变 V1/V2 包大小。

由于接收器与控制线程之间使用 latest-only 交换，单个事件帧可能在读取前被下一普通帧覆盖。因此本地帧还携带单调递增的 `resynchronization_generation`：重同步时递增，后续普通帧继续携带当前值。Session 仅在 generation 大于最后已应用值时重置一次，从而保证事件不会因 supersede 丢失，也不会让每个后续帧重复重置。

### UDP 接收器

接收器在流门返回接受时，将 `decision.stream_discontinuity` 写入发布帧。前两个候选帧仍计入 `jump_rejections`，第三帧计入 `accepted`。

增加 `resynchronizations` 统计计数，以便测试和后续 CSV 验证区分“新 epoch”与“同 epoch 稳定新簇重同步”。

### Viewer 会话与 OTG

`PicoTeleopSession::classify` 在以下任一条件成立时返回现有的 `kResetEpochAndApply` 动作：

- 首个有效帧；
- tracking epoch 改变；
- 同 epoch 帧带有 `stream_discontinuity=true`。

因此 Viewer 复用现有安全路径：

1. 从当前双臂机器人位姿创建新的 `TargetManager`；
2. 清空旧目标 twist 历史；
3. 将左右笛卡尔 OTG 重置到当前末端位姿；
4. 重置臂角方向管理器；
5. 通过现有目标步长限制和 OTG 平滑追踪新目标。

重同步不会直接跳变 MuJoCo 关节位置。

## 异常与安全边界

- 零 epoch、epoch 回退、重复或乱序 sequence 仍按原规则拒绝。
- 候选簇不会跨 epoch 保留。
- `reset()` 清除接受基准、最后观察 sequence 和候选簇。
- 单个或两个离群帧永远不会触发重同步。
- 三个帧必须彼此连续；三个任意跳变帧不能累计触发接受。
- 以约 90 Hz PICO 输入计算，正常重同步约需 22–33 ms。

## 测试要求

先添加失败测试，再实施代码：

1. 流门拒绝前两个稳定新簇帧，并在第三帧接受且标记 discontinuity。
2. 候选簇内部再次跳变会重置三帧计数。
3. 单帧跳变后回到旧基准，普通接受且不重同步。
4. 被拒绝候选中的重复/倒序 sequence 仍被识别为乱序。
5. 新 epoch 和显式 `reset()` 会清空候选簇。
6. UDP 接收器只发布第三个稳定候选帧，并增加重同步统计。
7. Session 对同 epoch discontinuity 返回 `kResetEpochAndApply`。
8. 原有 V1/V2 解码、接收、Viewer 集成和全量测试保持通过。

## 验收标准

- 自动测试证明同 epoch 稳定新簇最多拒绝两个连续帧，不再永久锚定旧位姿。
- 单帧和双帧毛刺仍不能进入控制链路。
- Viewer 对重同步帧走现有 OTG reset 路径。
- 新增统计能在下一次 PICO CSV 中确认重同步次数。
- 不改变 QP、OTG、SDK 限值或碰撞相关配置。
