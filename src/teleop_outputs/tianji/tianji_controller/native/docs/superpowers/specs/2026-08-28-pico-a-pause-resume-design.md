# PICO A 键暂停与继续接管设计

## 目标

在保持当前 `TJ_arm_control` 主控制流程、PICO 末端映射、SPARK IK 和关节空间 Ruckig 接管逻辑不变的前提下，实现 PICO 右手柄 A 键每次按下只触发一次暂停或继续：

- 第一次按 A：从当前机械臂状态平滑接管 PICO；
- 遥操中再次按 A：原地暂停并保持当前关节目标；
- 暂停后再次按 A：重新建立 PICO 参考并继续接管；
- 长按 A 或持续收到按钮高电平时只触发一次，不得逐帧切换。

## 非目标

- 不运行或复制 `tianji_teleop` 的整套 IK、控制器或第二套 PICO SDK 会话。
- 不改变 `TJ_arm_control` 当前默认机械臂 IK 算法、`hand_tcp_frame_L/R` 定义或 MuJoCo 手模型。
- 不让 A 键触发回安全初始位；本设计的暂停是原地保持。
- A 键暂停覆盖联合遥操：机械臂和 Manus/Wuji 灵巧手都保持暂停瞬间的状态。
- 键盘 `P` 继续作为无 PICO 按钮时的人工备用开关。

## 参考实现与当前差异

`tianji_teleop` 在每个输入周期直接调用 `xrobotoolkit_sdk.get_A_button()`，由 `_RisingEdge` 检测物理按键的 `false -> true`。当前 `PICO_tracker` 的 `/pico/record_flag` 则是每按一次 A 就在 `0/1` 间翻转的状态，因此 TJVR 接收端必须检测任意状态变化，不能只检测上升沿。

因此只移植以下语义，不移植 SDK 读取方式：

```text
PICO 原始按钮状态
  -> ROS Bool 状态
  -> TJVR v4 flags bit8
  -> TJ_arm_control 解码为 user_button_pressed
  -> 接受帧上的状态变化检测
  -> pause/resume/接管状态机
```

## 方案

### 启动状态

保持现有 `--pico-teleop` 启动行为：Viewer 启动后 PICO 会话默认处于 enabled。Viewer 首次收到的 A 状态只用于初始化；之后每次 A 使状态发生变化时切换暂停或继续。若先通过键盘 `P` 关闭 PICO，则下一次 A 状态变化执行首次接管。

### PICO Tracker 桥

在 `PICO_tracker/src/pico_bridge/src/tianji_mujoco_teleop_bridge_node.cpp` 中，桥端保存的按钮状态必须与 ROS 消息状态一致：

```cpp
record_button_state_ = message->data;
```

不能使用“每收到一次消息就翻转”的逻辑。TJVR 编码器继续将该状态写入 v4 的 bit8。

`TianjiTeleopBridgeCore` 还需要处理 PICO TCP 重连造成的源时间戳回退。新源时间戳比当前高水位回退超过 `1 s` 时，应清空旧的 skeleton/status 配对缓存和已完成时间戳集合，再接受新流；不超过该阈值的小范围乱序仍按现有缓存规则处理。这样 PICO 重连后不会长期停留在 `duplicate_source_stamp`/`status_cache_miss` 状态。

### TJVR 协议接收

在 `TJ_arm_control` 的 PICO 协议头中增加：

```cpp
inline constexpr std::uint32_t kPicoTeleopUserButtonPressedFlag = 1U << 8U;
```

`PicoTeleopFrame` 增加 `user_button_pressed` 字段。V4 解码器将已知标志扩展为 bit0～bit8，保留对 bit9 及以上未知标志的拒绝；解码成功后把 bit8 转换为按钮电平。V1～V3 的协议兼容性保持不变。

### Viewer 状态机

按钮事件只对已通过协议解码、流门控和新鲜度检查的 PICO 帧生效。Viewer 保存上一帧上报状态，并只在状态发生变化时执行一次转换：

```text
PICO disabled --A state change--> enabled + fresh takeover
PICO enabled  --A state change--> paused + hold current joint target
PICO paused   --A state change--> enabled + fresh takeover
```

暂停动作必须：

- 取消正在进行的 joint-space Ruckig takeover；
- 将当前已发布的关节参考固定为暂停瞬间的目标；
- 停止消费后续 PICO 位姿作为机械臂目标；
- 停止消费后续 Manus 手部关节帧，保持当前灵巧手关节位置；
- 不将机械臂回零、不同步到 MuJoCo 实际反馈、不产生位姿跳变。

继续接管时必须：

- 清除旧的 PICO 已应用帧、序号和 epoch 状态；
- 将继续接管前的 PICO 帧作为新的相对运动基准；
- 用当前机械臂关节参考作为 Ruckig 起点；
- 对新的目标重新执行现有的关节空间平滑接管。

键盘 `P` 复用同一套 enable/disable 状态转换，但不修改 PICO A 的上报状态记录。

### 安全与异常处理

- PICO 数据过期或流门控拒绝时，暂停状态保持暂停；不得因为恢复数据自动重新接管。
- PICO 重连或 tracking epoch 变化只重建数据流基准，不自动切换 pause 状态。
- 重连后的第一帧 A 状态只用于初始化，不把重连本身当作新的按键事件；下一次状态变化才允许切换。
- TJVR bit8 只在 V4 中有效；V1～V3 继续使用原有已知标志校验，出现 bit8 时按原有未知标志规则拒绝，不改变旧协议行为。

## 测试与验收

### 单元测试

1. TJVR V4 bit8 能被正确解码为 `user_button_pressed=true`。
2. V4 bit9 仍被判定为未知标志并拒绝。
3. 桥端 Bool `true -> false` 产生相同电平的 TJVR 标志，不发生额外翻转。
4. PICO 源时间戳回退后，桥端清空旧缓存并恢复配对发送。
5. A 长按只产生一次状态转换。
6. `enabled -> paused -> enabled` 的转换保持当前关节目标，并在继续时启动新的 Ruckig 接管。
7. A 暂停期间 Manus 手关节不再更新，继续接管后手关节更新恢复。
8. A 事件、P 键和 PICO 流断开不会互相误触发。

### 集成测试

使用合成 TJVR 序列验证：

```text
button=0,0 -> 无切换
button=1   -> 接管
button=1,1 -> 不重复切换
button=0   -> 仅更新边沿基准
button=1   -> 暂停或继续，取决于当前状态
```

同时验证暂停和继续过程中关节参考连续、控制失败计数为零，并保持现有 PICO、SPARK、Wuji2 手和 Direct 模式测试通过。

## 验收标准

1. 新鲜启动保持 enabled 时，PICO 右手柄 A 第一次按下后 Viewer 原地暂停；再次按 A 后以当前位置为新基准继续控制。
2. 通过键盘 `P` 或其他方式关闭 PICO 后，PICO 右手柄 A 第一次按下执行平滑接管。
3. 暂停或重新接管过程中机械臂不回安全位、不突然跳动，并执行现有关节空间平滑过渡。
4. PICO 重连后不因源时间戳回退而永久停止发送 TJVR 数据。
5. A 键长按、快速连按和普通 PICO 丢帧不会造成重复切换或失控运动。
6. 现有测试和 `pixi run build` 全部通过。
