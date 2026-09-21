# PICO Headroom 全局 Viewer 默认设计

## 目标

将 `main` 分支的 Viewer 主路径统一为 PICO 遥操下的 SPARK + Headroom
Feedforward Velocity QP，使无参数启动与当前实测效果较好的运行命令一致，同时保留
显式切换历史配置和 A/B 算法的能力。

## 默认行为

`tianji_qp_ik_viewer` 未收到相应 CLI 覆盖参数时采用以下默认值：

- 配置：`config/qp_ik_pico_teleop.yaml`
- 模型：`models/marvin_m6_qp_pico_fast.xml`
- PICO UDP：启用，绑定 `127.0.0.1:15000`
- PICO 骨架 Overlay：启用
- 控制层：`velocity`
- IK 算法：`spark_upper_qpoases_headroom_feedforward_velocity_qp`
- 控制状态源：`model_reference`

`config/qp_ik_pico_teleop.yaml` 自身也声明 `velocity` 和 Headroom Feedforward
Velocity QP，确保显式加载该配置但未传 `--control-level` 或 `--algorithm` 时得到相同
行为。Viewer 默认选项与配置文件保持一致，不依赖隐式覆盖纠正配置。

## CLI 优先级与兼容性

现有 CLI 参数继续拥有最高优先级：

- `--config` 可加载历史 acceleration、hierarchical 或其他配置。
- `--model` 可选择其他 MuJoCo 模型。
- `--control-level`、`--algorithm` 和 `--actual-feedback-control` 可覆盖新默认值。
- 已注册的历史算法和基准测试入口保持不变。

本次不删除算法、不改变算法内部参数，也不改动原始 benchmark 数据。

由于 SPARK guidance 要求 PICO 输入和 velocity 控制，无参数启动必须默认启用 PICO
UDP。没有收到 PICO 数据时，控制器按现有 stale/hold 机制等待；不会回退到脚本轨迹。
默认启动会占用 UDP 端口 15000。

## 文档

README 的 PICO 遥操章节以简化的无参数启动作为主入口，并明确列出默认配置、模型、
UDP、控制层、算法和状态源。保留一条显式完整命令，作为复现实验和排查默认值的基准。
原有非 PICO 和离线基准命令继续显式指定配置，因此含义不变。

README 同时提供 `output_continuity_retest.tjvr` 的 MuJoCo 可视化回放流程。该流程用
两个终端分别运行当前 main 默认 Viewer 和 TJVR UDP replay 工具，Viewer 运行 120 秒，
回放工具按原始时间戳向 `127.0.0.1:15000` 发送 9442 帧、106.887 秒的基准轨迹。
说明中必须明确实际算法为
`spark_upper_qpoases_headroom_feedforward_velocity_qp`，控制层为 `velocity`，状态源为
`model_reference`，模型为 `models/marvin_m6_qp_pico_fast.xml`。回放遥测写入 `/tmp`，
不污染仓库。配置与安全章节中旧的 acceleration 默认说明同步修正；历史 profile 仍可
通过显式参数运行。

## 验证

更新默认 Profile 测试，使其验证：

- Viewer 报告 PICO 配置；
- 控制层为 `velocity`；
- 算法为 `spark_upper_qpoases_headroom_feedforward_velocity_qp`；
- 状态源为 `model_reference`；
- 默认 PICO UDP receiver 能启动并在无输入时安全 hold/退出。

更新 PICO 配置测试中原有 acceleration/hierarchical 默认预期。最后运行完整 CTest
套件；需要本机 UDP 权限的测试在允许创建 loopback UDP socket 的环境中执行。验收标准
为 80 项测试全部通过，且无参数 Viewer 的启动摘要与上述默认行为一致。
