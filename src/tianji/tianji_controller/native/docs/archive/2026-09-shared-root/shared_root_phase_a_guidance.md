# Shared-root Phase A：guidance 离线接线进度

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

日期：2026-09-17。本文是阶段 6 进行中的验证记录，**不是 Phase A 完成报告或设备验收**。
前序契约、模型和模块记录见 [阶段 1 与后续进度](shared_root_phase_a_stage1.md)。

## 本轮实现

- `DualArmSparkGuidance` 增加可选 shared-root 实例；默认无实例，直接走旧分支。
  disabled 测试使用不存在的 artifact 路径，验证旧分支不会读取新配置或调用 adapter。
- 显式 `updateSharedRootFrame(frame, now)` 与 `stepSharedRoot(..., authorized, model_valid)`；
  模型 q/qdot/qddot 必须有限，调用方必须显式确认有效性。旧 step 不替新模式假定授权。
- 启动重验 artifact，实际使用的 URDF、模型 TCP、限位和 FK 几何与冻结模型交叉检查；
  仅接受 headroom/feedforward、velocity 控制配置。builder 使用实际 guidance 的 SPARK 参数。
- shared-root 独立 blend；控制 palm 不被 shape proxy 覆盖，header 左右掌位姿不参与该链。
- 恢复起点、TCP/shape FK、首次 IK seed 来自同一 `left_model/right_model` 快照；
  重置 IK、OTG、reference、feedforward、twist、headroom、最后有效 seed 及静止保持历史。
- intent 使用映射后的同语义 pose；尺度升级/重建帧的零 twist 不充当 stationary/moving 证据。
- RECOVERING 暂时抑制普通 settled-hold 和 stationary-reference-hold，不绕过 stale/授权门控。
  普通 settled-hold 释放请求也重新进入 shared recovery，不调用 legacy blend。
- HOLD 不伪装新 source/freshness；复用现有 feedforward stale/stop 路径。
- 恢复 alpha=1 后还需本周期双侧 IK 与 feedforward target 接受，最后等待外层
  `confirmSharedRootReference(cycle, bilateral_accepted)`；确认带控制周期标识，
  拒绝旧周期、重复确认和外层拒绝。该接口必须在单一控制线程中同步调用。

没有修改线协议、接收器、录制系统、线程架构、真机驱动、PICO2 裸手或其 V131。
所有新模块由既有单一控制线程拥有；启动校验不进入热路径。

## 本轮测试

`test_shared_root_guidance` 的 10 项用例覆盖：

1. 可达目标与当前周期外层接受确认；
2. 丢帧期间换到新姿态、恢复后静止，不需再晃手；
3. 真实 `DualArmController` velocity-QP 的同周期双侧接受（离线，无动力学积分）；
4. 模型/reference 与旧 seed 不同时，恢复起点与首次 seed 一致；
5. 显式授权、模型有效性、非有限 qdot 与旧 API 误用；
6. disabled 与 legacy 的目标、q_ik、状态数值对照；
7. 错误姿态模式或 TCP 模型拒绝；
8. 普通 settled-hold 释放后使用 shared recovery；
9. 同一合成 FK trace 的三组 IK 冒烟对照；
10. 候选处理链独立计时。

本轮先构建后执行：

```bash
pixi run cmake --build control/build --parallel 2 --target \
  test_shared_root_guidance test_shared_root_options test_shared_root_pipeline \
  test_shared_root_target_builder test_shared_root_continuity test_shared_root_input \
  test_shared_root_morphology test_shared_root_geometry test_config \
  test_spark_guidance test_spark_upper_retarget test_pinocchio_arm_kinematics \
  test_spark_feedforward_reference test_spark_constraint_headroom \
  test_controller test_pico_teleop_session tianji_qp_ik_viewer
pixi run ctest --test-dir control/build --output-on-failure \
  -R '^(test_shared_root_.*|test_config|test_spark_guidance|test_spark_upper_retarget|test_pinocchio_arm_kinematics|test_spark_feedforward_reference|test_spark_constraint_headroom|test_controller|test_pico_teleop_session)$'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_contract.py pico2_hands/tests -q
.pixi/envs/default/bin/python control/scripts/validate_shared_root_contract.py \
  control/config/qp_ik_pico_shared_root.yaml
git diff --check
```

结果：**16/16 CTest 程序、64/64 Python 测试通过**，artifact 校验通过。
Viewer 仅编译兼容性验证，未运行。隔离路径与协议/receiver/recorder 文件零差异。
新测试初版发生过 Eigen 表达式类型编译错误，修复后重建并运行以上验证；
不存在用旧可执行文件代替新代码结果的情况。

## 合成样本与计时：仅用于冒烟检查

已知可达 FK 轨迹 160 帧，排除前 30 帧统一初始化段，每组统计 130 对双臂样本。
三个组的输入轨迹与起始模型相同，排除失败样本数均为 0；测试将 IK wall-clock
预算显式放大到 0.2 秒排除 CI 调度干扰，**没有改发布 profile 的 4.5 ms 预算**。

| 组 | palm 位置 p90 | palm 姿态 p90 | B 系双掌向量误差 p90 | 相对旋转误差 p90 |
|---|---:|---:|---:|---:|
| legacy / 旧权重 | 10.6702 mm | 0.0381913 rad | 11.5548 mm | 0.0417081 rad |
| shared / 旧权重 | 9.19407 mm | 0.00245252 rad | 5.01772 mm | 0.00169304 rad |
| shared / 测试权重 | 8.80130 mm | 0.00235956 rad | 4.80346 mm | 0.00164216 rad |

测试权重只在该用例内将 Stage 2 elbow/wrist 绝对位置权重从 5 降到 0.25；
其他权重不变。实验 profile 未据此调参或冻结权重。

这里衡量 **q_ik FK 相对送入 IK 的目标**，不是 reference 或 actual 跟踪误差。
IK-only 对照不伪造外层接受，因此 shared 组保持 RECOVERING(alpha=1)、普通 hold 被抑制；
**这不是完整闭环 A/B，不能据此宣称新算法更好。** 真实 velocity-QP 的接受由独立用例验证。
真实反馈缺失，actual 明确为 `unavailable`。

独立候选链计时：2,048 个重复姿态、唯一源帧样本，去除前 32 个预热样本，
本轮 verbose 测试输出 p99 为 **1.162 μs**（2,016 样本）。包含 adapter、尺度、
builder/filter 与 continuity.observe，不含 IK、模型 FK、控制器或 I/O。
单机合成小样本满足候选链 50 μs 初始目标，但不构成实时资格或真实 trace 性能验收。

## 仍未完成，继续保持默认关闭

上述是 guidance 库接口的阶段记录。后续已增加 `run_qp_ik_viewer.cpp` 显式分支，
详见下节；没有启动应用或设备进行验收。实验 profile 仍保持 `enabled=false`。

仍需：

- 应用分支实际执行验证（本轮仅编译与组件组合测试，没有运行 Viewer）；
- 已证实契约匹配的真实 TJVR 录制 A/B（目前仅合成 FK 冒烟，不是现场录制）；
- anisotropic/isotropic 尺度消融、完整实验权重选择与冻结；
- 更全面的 shape 冲突、限位、奇异、预算耗尽等样本及分层原因统计；
- 实际配置预算下的全链离线计时与最终阶段 6 验证报告。

不得把上述库测试通过当作现场 shared-root 路线已经可以启用。
未进行设备仿真、真机操作、Phase B、提交或 push。

## 后续推进：应用分支与配置消费者边界

2026-09-17，本次修改补齐 Viewer 的静态接线，未执行 Viewer：

- `loadConfig` 默认仍为 legacy 消费者，开启 shared-root 的配置会明确拒绝。
  Viewer 显式声明 `ConfigConsumer::kSharedRootAware`，开启时才校验完整 artifact；
  disabled 不读取新 artifact。未改其他消费者的默认调用。
- 新配置只支持 PICO、headroom/feedforward、velocity；禁止 override 成其他算法或加速度模式。
  shared-root 会话内的 IK/控制级别切换请求不执行，必须结束会话后选择其他 profile。
- **实验模式拒绝非零 joint-command-port**，不向外部真机执行端发送关节命令。
- 已有 PICO session 先做身份/新鲜度分类，再交给新 adapter；合格候选才 commitApplied。
  不使用 legacy 的 joint-space takeover，恢复交给 shared continuity。
- 每周期显式传入 controller referenceState、授权状态和本机 monotonic 时间。
  这些是旧入口原本传入 guidance 的模型/reference 状态，不冒充真实反馈。
- 不在每个 stale tick 清空共享根连续性状态；由其本机时钟执行短 hold/长 invalid。
- 当 mapping invalid/stale/disabled，缓存的 Cartesian reference 同步标 stale、twist 置零。
  旧参考曾经 fresh 不能使当前无效目标重新获得 freshness。
- 外层控制器完成本周期求解后，双侧接受、源新鲜和目标有效均满足时才确认恢复。
- Pause/rearm 转换立即重置本模式历史，包含同一控制周期内排队的 pause/resume；
  ResetNominal/Home 后关闭本模式接管，需用户再次启用，不能由后续新输入自动接管。

新增配置用例：aware 开启完整配置成功、legacy 消费者拒绝、非法窗口拒绝、
disabled 下不存在的 artifact 不被读取。
新增 guidance+现有 PicoTeleopSession 组合用例：首次接收、epoch 初始化、暂停和重新启用，
证明候选形成与 commitApplied 不会构成永远重置的恢复死循环。

按本文构建/回归命令重新验证：**16/16 CTest 程序通过**（guidance 现有 11 项用例），
**64/64 Python 测试通过**；Viewer 编译成功、artifact 校验通过、`git diff --check` 通过。
`pico2_hands/`、`pico2_sim.sh`、`tracking/`、`real_robot/`、线协议、UDP receiver、
trace recorder 均零差异。没有运行 Viewer，包括 headless Viewer。

配置临时门已由“一律禁止”改为“只有明确支持该模式的消费者可解析”；这不是验收放行。
真实录制 A/B、尺度消融、完整异常样本及实际预算下的性能验收仍待完成，
不能将编译和组件测试通过描述为生产应用实际运行已验证。

## 2026-09-17：尺度几何消融与历史录制资格审计

本轮仅新增 target-builder 离线测试，不改变实现、实验权重、默认开关或输入配置。
`OfflineScaleAblationMeasuresRawDistortionNotTracking` 将同一组 360 帧合成输入分别送入
两个 builder；仅在测试内设置 `reach=1.2, lateral=0.8` 与 `reach=lateral=1.2`。
没有增加运行时尺度切换，也没有绕过有效性门（排除 0 帧）。

| raw 映射指标 | 各向异性 | 统一尺度 |
|---|---:|---:|
| 输入 XY 圆映射后的 X/Y 轴长比 | 1.5 | 1.0 |
| 双掌相对向量方向最大偏差 | 0.136079 rad | 2.69374e-16 rad |

同时逐帧检查 raw 双掌关系等式、两组 shape 不被尺度再次拉伸，以及掌姿态不被再次乘 basis。
这只是方案 8.2 的**几何部分**；未测这组轨迹的 IK 跟踪、可达性、限位/饱和和臂形误差，
不能据此选择最终尺度策略，更不能将统一尺度无角度畸变等同于机器人跟踪更好。

### 真实 trace 的具体阻塞证据

只读核对用户此前指定的历史文件：
`/home/zj/pico参考数据/pico_eggbeat_bandwidth_20260904_session01/pico_eggbeat_bandwidth_20260904_session01.tjvr`。
SHA-256 为 `85e06eb32a5b0a8783bb74e7d663ad4f1de75a9f4db053bedcccd853ed55b9a3`，
与 `bandwidth_recovery/hf_floor_sweep/manifest.json` 一致。
该 manifest 包含回放模型/工具哈希和回放结果，但没有录制时的 M0/bridge 源码版本及骨长配置。
`bandwidth_recovery/deterministic_frf_floor0/manifest.json` 仅记载 case 数量、耗时与任务数，
不能补足输入来源证据。

按现有 `pico_trace_recorder.cpp` 的 TJVT v1 容器以及 `pico_teleop_protocol.cpp` 的 v4
布局只读解包：16 字节文件头，每条为 8 字节相对接收时间 + 656 字节报文；
points 在报文偏移 204 处，为 8×3 个 little-endian double。
以点 0→1→2→3 和 4→5→6→7 的欧氏距离统计骨段，不使用报文 header 掌位姿推算。

- 10,682 帧，文件大小与头部条数匹配，全部 TJVR v4，flags=255；
- 全部报文 CRC32 校验一致，points 全部 finite；epoch 为 194、195；
- 双肩中点相对 `[0,0,1.121] m` 的最大偏差为 `2.3755e-16 m`；
- 原点和结构通过不等于完整源语义通过，尤其不能证明 basis/掌心来源。

| 米制骨段长度中位数 | 左侧 | 右侧 | 左右绝对差中位数 |
|---|---:|---:|---:|
| shoulder→elbow | 0.270496837 | 0.276315014 | 0.005818177 |
| elbow→wrist | 0.176162966 | 0.247517187 | 0.071354220 |
| wrist→palm proxy | 0.029941927 | 0.071908760 | 0.041966833 |

上臂/前臂左右差的 p99 同样约为 0.005818177/0.071354220 m，不是少量离群帧。
这份录制不能作为用户指定的 **M0 对称有效骨架** 正式验收 trace。
这里不把 wrist→palm 外参差单独当作人体骨长异常；仅上臂/前臂的持续不对称已足以指出缺口。
未重写录制、未离线强行对称后冒充实测、未启动回放 UDP 或 Viewer。

正式 trace A/B 需要另行提供：满足本次 bridge/M0 输入契约的对称骨架 TJVR，以及对应的
bridge/M0 版本、实际生效骨长配置和输入来源记录。未取得这些证据前，不猜测来源、不冻结
实验参数，保持 `enabled=false`。既有合成 FK 对照不能替代这一验收项。

本轮验证命令沿用上文：重建 `test_shared_root_target_builder` 后执行相同 16 项 CTest，
**16/16 通过，16.62 秒**；target-builder 内含 10 个用例。
Python 回归 **64/64 通过，4.91 秒**；artifact 校验、`git diff --check` 通过。
`pico2_hands/`、`pico2_sim.sh`、`tracking/`、`real_robot/`、线协议、receiver、recorder
相对 HEAD 均零差异。未操作设备、未提交和 push。
阶段 6 仍未验收完成：正式 trace A/B 受上述数据契约阻塞；其余未完成项仍以上文清单为准。

## 2026-09-17：使用 zhoujie 新配置继续 Phase A

本轮不是切回 mapped-palm，而是继续 shared-root 实施与离线验证。
选定 `profiles/zhoujie/pico-simple/cal-91c71441c51b47db9366390c96c96199`，
不修改其 active 选择、不覆盖其他人员档案、不自动启动设备。

- 身高 1.62 m；双侧上臂 0.25252884000000003 m、前臂 0.24776442 m、
  腕掌距离 0.11933279483594278 m。
- manifest SHA256：`0a2c8b1868045366026d0bb8278809547f9fb30912667708e77570c14a8f3dac`。
- input contract SHA256：`54db28f43139a90d94eb9039129c52c2b631c7c9f57e925bf82817ec7a4a7328`。
- robot geometry artifact SHA256：`46bd753104f8a3efa56926b04495c46d67adc8460ee2a94ce85743541cfc97d6`。
- 实验 profile 保持 `enabled=false`，哈希仍为
  `97675c6028e7006c6a4e95541fccec7acf0291652522c95b90d80c2c63ed645f`。

新增两项 C++ pipeline 回归使用该人员尺寸快照及实际实验配置：
共同尺度估计/双掌关系/固定高度仅处理一次/静止 intent；
丢帧期间移手后静止恢复、拒绝旧序号接受确认、恢复完成后进入 TRACKING。
肩宽 0.4 m 和动作是明确合成输入，不是 zhoujie 实测；未模拟驱动、未验证镜像物理轴，
也没有将配置或合成帧当作现场 trace。同步修正 native options 测试遗留的旧契约哈希。

实际验证：

```bash
pixi run cmake --build control/build --target test_shared_root_pipeline test_shared_root_options -j 2
pixi run ctest --test-dir control/build --output-on-failure \
  -R '^(test_shared_root_.*|test_config|test_spark_guidance|test_spark_upper_retarget|test_pinocchio_arm_kinematics|test_spark_feedforward_reference|test_spark_constraint_headroom|test_controller|test_pico_teleop_session)$'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_audit.py control/tests/test_shared_root_contract.py pico2_hands/tests -q
.pixi/envs/default/bin/python control/scripts/audit_shared_root_trace.py \
  --calibration-dir profiles/zhoujie/pico-simple/cal-91c71441c51b47db9366390c96c96199
.pixi/envs/default/bin/python control/scripts/validate_shared_root_contract.py control/config/qp_ik_pico_shared_root.yaml
git diff --check
```

结果：16/16 CTest 通过（15.04 s），72 项 Python 测试通过（4.67 s）；
配置完整性与 artifact 校验通过。`pico2_hands/`、`pico2_sim.sh`、`real_robot/`
相对 HEAD 零差异。tracking 的已有差异属于另行授权的简化标定接入，不能再称整个 tracking 零差异。

阶段 6 仍未全部完成：新的 zhoujie 真实 TJVR 与来源证据、同 trace 三路 IK A/B、
完整负例和运行预算验收仍待补齐。actual/dynamics/设备/真机未验证。
本轮未运行 Viewer、未连接设备，未提交或 push。

### 后续更正：用户指定腕掌距离改用身高比例

以上 0.11933279483594278 m 的尺寸快照是历史结果，不再是当前 zhoujie 配置。
按用户明确要求，原目录已升级 manifest v2，双侧腕掌距离为
`1.62 * 0.037037 = 0.05999994 m`，来源为模型估计；掌心 TCP 与双侧上、前臂未改变。
原始完整目录备份到 `profiles/zhoujie/pico-simple/pre-height-wrist-backup-O6H2m8AI/`。
当前 manifest SHA256 为 `5fd97dee1c3342c6e7c2d488f2cccba8ea75135208d198ea104d4a5627e0df71`。
input contract SHA256 为 `68304d5b76b063996bdf386b28d103af1d998f7f9a801acb5a55960aaa2fccda`；
geometry artifact SHA256 为 `be93c26db544ad40c96275f49478015256d916657457a0ab31b14c736a56ccd8`。
二者重新固定版本只是同步输入来源证据，机器人几何数值未改变。

新生命周期只调用 left tcp，不再调用 left wrist；旧 manifest v1 和旧双侧标定保持兼容。
默认 Python 的简化标定、契约审计及 PICO2 回归 102 项通过（5.32 s）；
ROS Python 纯 M0 加载器 v1/v2 两项通过（0.30 s），未创建节点。
重建并运行 shared_root_options/pipeline/guidance 三项 CTest，3/3 通过（11.34 s）。
目录完整性校验、TCP 备份逐字比较、git diff --check 通过。

补充预算负例明确了旧 solver 的语义：预算耗尽仍可能 accepted 最佳受限候选，
不等于数值收敛；不改变 legacy solver。外部拒绝参考时仍不得确认 shared-root 恢复。
新的 zhoujie 尺寸合成 guidance 探针采用相同 100 个统计帧，配置预算 4.5 ms 与
宽裕预算 200 ms 均为 accepted=100、排除=0、预算耗尽=0，掌位置 p90 均为
0.0814357 m；guidance wall p99 分别为 58.535/56.559 us。
这说明该合成样本的残差不是放宽墙钟预算可以消除的；尚不能判断为全局不可达或
臂形冲突，记为 CauseUndetermined。该探针通过表示统计/有限性/限位检查通过，
不是跟踪误差验收通过；它没有真实动作、闭环 actual 或动力学。
shared-root 保持默认关闭，Phase A 阶段 6 尚未验收，不据此调整现有 IK。

### 2026-09-17：zhoujie 掌目标与臂形任务消融

继续阶段 6，不运行设备。新增
`SharedRootGuidance.ZhoujieStaticPalmOnlyVersusShapeMultiSeedDiagnostic`：
采用当前 zhoujie 三段长度构造静态合成输入，由 pipeline 生成同一个掌目标，
使用模型 TCP 的 Pinocchio FK 检查结果。每侧每组使用 5 个不同限位区间初值，
Stage 1/2 各最多 100 次迭代、收敛增量 1e-10、每次 200 ms 预算。
这些是离线诊断覆盖值，不写入任何现场 profile，也不修改配置验证器。

| 对照 | 左掌最佳候选位置误差 | 右掌最佳候选位置误差 | 左/右满足 10 mm 且 0.1 rad 的初值数 |
|---|---:|---:|---:|
| 原完整臂形权重 | 81.3843 mm | 79.7323 mm | 0/0 |
| 仅降低 Stage 2 肘/腕位置权重至 0.25 | 80.9166 mm | 79.1498 mm | 0/0 |
| 去除两阶段骨段方向任务及 Stage 2 肘/腕位置任务 | 0.000258 mm | 0.000264 mm | 3/5 |
| Stage 2 骨段方向权重 0.05、肘/腕位置 0.25 | 10.2687 mm | 8.80465 mm | 0/2 |

最佳候选按 `position/0.01 + orientation/0.1` 选择，同一行的位置和姿态来自同一个解，
不分别挑最小值拼接。40 次求解均返回 accepted，预算耗尽为 0；均验证 finite 和带 margin 限位。
纯掌位姿组新增非空达标解及最佳位置 <1 mm、姿态 <0.01 rad 的断言。

结论限于该静态样本：掌目标可达；完整臂形任务参与时存在显著折中残差，
仅放宽墙钟预算或降低肘/腕绝对位置权重不足以消除。
不能据此证明 full-shape 全局不可行，仍不输出全局 ShapeConflict 判决。
历史诊断第 4 组左掌未达到 10 mm。2026-09-18 用户确认该精度仅作统计，不作为
验收阻塞条件，不继续针对单样本调参到“刚好通过”，也不因此增加现场迭代次数。
未更改运行权重、默认开关、solver、驱动或线程架构。

后续仍需多姿态/奇异/限位集合、尺度消融和符合新 zhoujie 来源契约的真实 trace 三路 A/B。
这次纯掌位姿组只用于诊断，不是新增现场 palm-only 模式；没有放弃方案要求的臂形软任务。

本轮重新链接相关测试后，既有同组 16 项 CTest 全通过（17.05 s）；
契约、trace 审计及 PICO2 共 72 项 Python 回归通过（4.70 s）。
测试通过不覆盖尚未达标的跟踪指标，未启动 Viewer 或设备，未提交/push。
