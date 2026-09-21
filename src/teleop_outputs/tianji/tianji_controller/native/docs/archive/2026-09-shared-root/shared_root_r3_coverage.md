# R3 覆盖率补充与 Phase A 验收缺口

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

日期：2026-09-18。HEAD：`b0713576724499d229e12155d7db1daed096e9fe`。
接续 [R3 接线报告](shared_root_r3_wiring.md)。本轮只改离线审计和测试，
不改映射结果、控制器、QP、驱动、录制系统或线程。没有运行设备或 Viewer。

## 本轮新增

- 原生 `--mapping-only` 输出 `mapping_coverage`，统计唯一源帧覆盖率、
  累计无效时长、最大连续无效时长及 raw/filtered 分侧闭合原因。
- 时间统计仅使用录制接收单调时间；连续无新帧超过 profile freshness 后的
  区间计入无效。源时钟不参与时长相减。
- 重复 epoch/sequence 在离线读取阶段拒绝，不累计覆盖率或刷新 freshness。
  这是离线审计策略，不修改现场协议解码或接收行为。
- 未标注动作明确为 `unannotated`；动作验收为 `unavailable`，不自动猜标签。
- 单元测试覆盖新鲜输入、无帧过期、连续失效、首帧无效、相同接收时间、
  时间倒退和多次查询；集成测试校验原录制结果及重复帧拒绝。

## 统计口径

本工具目前要求完整可解码、源帧有序唯一、morphology 可评估的录制；协议损坏、
倒序或不支持的 morphology 重建情形会报错，**不跳过坏包后宣称高覆盖率**。
因此以下是本条合法录制内的条件覆盖率，不是任意现场输入的可用率。
`closure_unevaluated` 单列尚未进入闭合计算的候选，不能归为几何不可达。

- 分母：4388 个完整评估的唯一源帧，不丢弃几何失败帧。
- 不使用 latest-only，superseded=0；与 200 Hz reference 回放分母不同。
- 某帧 raw 或 filtered 任一侧不可达，整对不接受；分侧/分层统计允许重复归因，
  不能把各层失败数相加当作失败帧数。
- 时间窗为第一条至最后一条接收时间，约 49.9867 s；不推测 EOF 后的停流时长。
- 无效帧从收到它时开始计时；合法帧有效期为 freshness=0.1 s，超过后计失效。
- 这里是 standalone mapping，不是 continuity 恢复等待时长、QP 故障率或 actual。

## zhoujie 录制结果

输入：`recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr`。
SHA256：`8d92f338056a40f73602e6d029c807d807dea29cc6e378b15946d85d3354ab62`。

| 指标 | 结果 |
|---|---:|
| 双侧 raw+filtered 均有效 | 4376 / 4388（99.7265%） |
| 任一层任一侧几何不可达 | 12 / 4388（0.273473%） |
| 闭合未评估 | 0 |
| 累计映射无效 | 0.136743 s |
| 最长连续映射无效 | 0.062382 s |
| raw 左 / 右不可达 | 12 / 0 帧 |
| filtered 左 / 右不可达 | 9 / 0 帧 |
| 分支未定 / 其他闭合失败 | 0 / 0 |

固定双掌间距门仍关闭；失败由两连杆几何边界判定，不是 1.6 m 臂展门控。
raw 关系、固定骨长与唯一掌末点仍通过原来的数值断言。

## 本轮验证

```bash
cmake --build control/build --parallel 4
ctest --test-dir control/build -R shared_root --output-on-failure
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py \
  control/tests/test_shared_root_contract.py \
  control/tests/test_shared_root_palm_priority_profile.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  pico2_hands/tests tests sim/tests real_robot/tests -q
control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root.yaml \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr --mapping-only
```

外围回归：346 passed、1 skipped；跳过 `test_pico_simple_ros_loaders.py`，当前
Python 环境不能导入 ROS `rclpy`（ABI 环境要求）。未改系统或安装环境来绕过。
原生 shared-root 回归 **10/10 组通过**（20.23 s）；审计/契约 Python 回归
**52 项通过**（89.78 s，无跳过），包含逐源帧及 200 Hz 参考循环，未放宽原断言。
`git diff --check` 通过；PICO2 目录及入口、real_robot 和原 MuJoCo XML 相对 HEAD
零差异。CMake 配置仍有既有 Boost policy / hpp-fcl 名称警告，本次新代码编译
无新增警告。没有将此前测试结果当成本轮复测。

## 尚不能宣布最终启用

阶段 1R～6 的实现及离线验证证据已分别落在输入、几何、接线与本报告中；
最终启用门仍缺**有动作标签的代表性录制**和评审冻结的覆盖率/连续失效门限。
自然前伸、靠近、交叉、一高一低、单手工作、共同运动、伸直边界目前均不能
凭这条无标签录制单独验收。不能把总体 99.73% 当成每类动作均通过。

不自动采集新数据、不猜动作区间、不替用户冻结门限；`enabled:false` 保留。
actual、动力学、真实输入仿真和真机未验收；本轮没有提交、push 或进入 Phase B。
