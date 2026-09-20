# 50 秒共享根动作采集

新增独立的输入观察工具，不启动 Viewer、PICO 驱动、Manus 或机器人执行器，不发布运动命令。
UDP 接收、TJVR 校验、原包写入和阶段时间戳均在 C++ 中执行；Python 只管理会话、
配置快照、提示显示和离线产物。旧录制器、控制器和默认启用配置保持原行为。

## 编译和预览

在仓库根目录执行：

```bash
.pixi/envs/default/bin/cmake --build control/build --target tianji_record_action_trace -j 2
.pixi/envs/default/bin/python control/scripts/record_shared_root_actions.py plan
```

`plan` 不绑定端口、不访问设备、不创建录制目录。固定流程：

| 录制时间 | 提示 |
|---|---|
| 0～5 s | 自然站立，双手放松静止 |
| 5～11 s | 双手自然前伸，再收回 |
| 11～17 s | 双手靠近，再分开，不碰撞手柄 |
| 17～23 s | 胸前交叉，再展开 |
| 23～29 s | 左高右低，再交换 |
| 29～35 s | 左手静止、右手移动；32 s 提示交换 |
| 35～41 s | 双手共同左右、上下移动，尽量保持间距 |
| 41～47 s | 缓慢接近伸直，再收回，不锁肘 |
| 47～50 s | 回舒适位置，静止 |

先收到有效 v4 数据，再倒计时 5 秒；倒计时内持续读取并丢弃准备数据。
倒计时后收到的第一帧是录制时间零点。第一帧缺失时继续等待，超时报错，
不会把命令启动时间、源时钟或倒计时开始时间当成录制起点。
提示输出到运行该命令的终端，附终端响铃字符；没有内置语音合成。
佩戴头显时应由观察者读出提示，或确保能看到主机终端；聊天消息不适合精确报时。

## 真实采集

先确认佩戴者、标定档案和已有输入会话。只运行一条 PICO 输入路线，端口 15000
应由本工具独占；不启动仿真或真机执行器。工具不会自动停止占用端口的其他进程。
上轮 zhoujie 录制采用简化标定输入和 +0.20 m 桥接 X 偏移，不能仅凭用户名
改用普通 `pico.sh` 并宣称来源相同。启动设备仍遵循本轮授权范围。

```bash
.pixi/envs/default/bin/python control/scripts/record_shared_root_actions.py record \
  --participant zhoujie \
  --calibration-dir /实际使用的标定目录 \
  --output recordings/shared_root/本次新目录
```

`--output` 必须不存在。可设置 `--port`（默认 15000，仅绑定回环地址）、
`--countdown`（默认 5 秒）、`--wait-seconds`（默认 30 秒）。
`--source-kind synthetic` 仅用于软件测试，始终保留在结果和确认文件中，
不能冒充现场动作采集。

## 产物与确认

- `input.tjvr`：成功关闭、校验后独占发布的 TJVT v1 容器，原始 TJVR v4 包不改写。
- `input.tjvr.partial`：恢复用原始文件；成功后与 `input.tjvr` 是同一 inode 的硬链接，
  不是两份独立副本，请勿修改任何一份。失败时保留已有字节，不发布成功录制。
- `events.jsonl`：准备倒计时、第一帧单调时钟、阶段提示、32 秒换手提示及结束记录。
  阶段同时记录计划时间和实际发出时刻；后者相对第一帧本机接收时间。
- `capture-start.json` / `capture-result.json`：配置及源码指纹、参与者、来源类型、
  成功/失败状态、帧数、录制/事件/候选标注指纹。`snapshot/` 保存来源文件快照。
- `action-candidates.json`：由实际提示时刻产生的候选区间，**不是正式标注文件**。
- `actions-unconfirmed.json`：兼容动作统计工具的空标注，所有动作仍未覆盖。
- `native.stderr.log`：原生进程错误；坏包另保留为 `rejected.datagram`。

录制成功只表示本次 50 秒采集流程成功关闭，不保证跟踪持续有效、没有丢包或动作
已执行。录制末帧之后的尾部不可观测时间单列为 `unobserved_tail_ns`，候选区间
裁剪到末帧；失联区间不补帧。中断、端口冲突、坏包、写入失败或采集中配置变化
均保留失败状态，不生成成功标记。突然断电/SIGKILL 可能只有 start/partial，
缺少成功 result 就不能视作完整。

提示时间不能证明实际动作开始时间。采集后由佩戴者/观察者检查，若全部动作确实
与提示区间一致，才显式执行：

```bash
.pixi/envs/default/bin/python control/scripts/record_shared_root_actions.py confirm-prompts \
  --session recordings/shared_root/本次新目录 --reviewer 实际确认者
```

该命令校验文件指纹并独占创建 `actions-confirmed.json`，不会覆盖已确认文件。
若动作迟做、漏做或中断，**不要使用整段确认**；另建 JSON，只填写可信区间，格式见
[按动作统计说明](shared_root_action_coverage.md)。静止及未知区间保持未标注。

```bash
.pixi/envs/default/bin/python control/scripts/report_shared_root_actions.py \
  recordings/shared_root/本次新目录/input.tjvr \
  --annotations recordings/shared_root/本次新目录/actions-confirmed.json
```

工具不冻结门限、不自动验收、不修改 `spark_shared_root.enabled:false`。
本机源码/标定快照也不证明远端发布者使用了这些配置；现场还需保留启动日志和
实际输入链路证据，并做原始数据契约检查。

## 软件验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_record_shared_root_actions.py \
  control/tests/test_shared_root_actions.py -q
```

包含一次实际等待 50 秒的回环合成输入采集，检查原包完整保留、接收时间零点、
提示时间偏差、倒计时、换手提示、候选/确认隔离；以及坏包、超时、中断、独占创建、
端口冲突、区间裁剪和确认时指纹校验。不连接真实设备，不构成现场或真机验收。

2026-09-18 本轮结果：上述命令 **38 passed，53.71 s**，包括新采集测试 14 项和
既有动作统计测试 24 项。原生采集目标编译成功；随后执行：

```bash
.pixi/envs/default/bin/ctest --test-dir control/build --output-on-failure \
  -R 'test_pico_(trace_recorder|teleop_protocol|udp_receiver)|test_shared_root_(input|pipeline|continuity)' -j 1
git diff --check
```

结果为 **6/6 passed，0.65 s**，差异空白检查通过。没有重跑完整 98 项，
不把上轮完整回归算成本轮结果。本轮此处记录的软件测试没有连接设备；
真实动作采集、动作确认、启用门限及真机验收仍独立待完成。
