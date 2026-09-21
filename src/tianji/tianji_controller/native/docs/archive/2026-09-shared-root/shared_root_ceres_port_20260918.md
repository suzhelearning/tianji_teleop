# Ceres LM + Ruckig 独立后端移植

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

历史记录：本文数值对应 2026-09-18 的旧关节限位/jerk。2026-09-19 经用户确认后的配置及最新对照见 [源限位对齐报告](shared_root_ceres_limits_20260919.md)，不以当前配置复现本文旧数值。

2026-09-18。已从外部试验推进到当前工程的原生控制器、共享根接线及 Viewer 配置入口。**软件接入不等于跟踪验收通过：完整共享根回放仍有左臂大残差，暂不能替换默认路线。**

## 交付与边界

- 算法名 `pico_ee_franka_ceres_lm`；配置 [qp_ik_pico_shared_root_ceres.yaml](../../../config/qp_ik_pico_shared_root_ceres.yaml)。原有默认配置不变，新配置的 `spark_shared_root.enabled` 仍为 `false`。
- 路径为共享根输入校验、映射/投影/恢复 → Ceres 末端位姿 IK → Ruckig → 模型关节参考。该分支不运行 SPARK 形状 IK、headroom governor 或二次速度 QP。
- 源工程版本 `6f241497b2cba4c48b1b4de57eab4298dfc4cfef`。`src/pico_ee_franka_ceres_lm.cpp` 与源文件 SHA256 完全一致：`58600f8938249ca1dd8c5e0a705aafb67be9ed0ceb7499fb3457ce587dd27074`。
- 核心保留世界坐标位姿残差/雅可比、LM 搜索、限位盒、时间预算、nullspace/home 和本次最佳候选语义。每臂以先前接受的 IK goal 作下一帧 seed，不以滞后的 Ruckig 输出反灌 seed。
- 仅移植 Ruckig 模式，不提供 fourth-order 或无平滑旁路。源 `JointTrajectoryLimiter7` 独立改名为 `CeresTrajectoryLimiter7`，包括源版本的离散位置/速度投影；旧平滑器完全不变。当前和源工程均使用 Ruckig 0.19.4。
- 使用当前完整54自由度模型、冻结 TCP、米/弧度及200 Hz 时序；不再依赖临时14自由度 XML。专用 `armKinematicsOnlyAt` 只执行 FK 和雅可比所需的 MuJoCo 步骤，不执行接触/动力学求解。64组左右臂姿态对比中，TCP、几何点和雅可比与原 `armKinematicsAt` 一致（阈值1e-12），不修改实际模型状态。旧接口仍调用完整 `mj_forward`。
- 运行时没有对外部源工程的路径依赖。Ceres 为编译时 opt-in：默认 OFF，不查找、下载或链接 Ceres；ON 时优先找 Ceres >=2.1，否则下载固定 SHA256 的2.1.0源码。此次 ON 构建使用后者。

## 安全适配，不宣称整个源控制器逐位等价

- 首版仅允许 `model_state_only=true`、velocity 控制层的共享根仿真；Viewer 禁止关节命令导出。未改现场 coordinator、设备驱动、手部发布者和真机执行入口。
- 保留共享根身份/时间/新鲜度、模型指纹、TCP、限位及授权校验。禁用配置不能静默退回旧映射；缺少 Ceres 构建时明确报错。
- 源求解器提供最佳努力候选，因此 `accepted` 表示候选及平滑输出可接受，**不是位置残差达标**。没有给这条路线添加未经评审的自动放行门限。
- 单侧求解/平滑失败时双臂均不提交候选，也不提交新 seed 或平滑历史。共享根本周期外层确认仍由双臂控制结果驱动；epoch/恢复重置会清掉旧 seed。
- 失效输入不继续追赶上一 IK goal，而以当前参考为目标执行受限停止，且不报告新目标接受。正常恢复从当前参考重新初始化。这是为当前共享根状态机增加的安全适配，区别于原外部试验入口。
- 当前加速度60/90 rad/s²、jerk1000/1500 rad/s³，不自动采用源配置更激进的3000/4500 jerk。Ruckig配置不能超出当前运动包络；不支持的 outward 硬约束显式拒绝，而不是忽略。
- 双臂事务更新目前复制平滑器状态，会有堆分配；本轮不宣称硬实时或真机适用。

## 本工程完整录制回放

使用 `zhoujie_actions_20260918_041020/input.tjvr` 的全部4427帧，接入实际共享根恢复/断流状态机，200 Hz推进并追加1秒失效输入停止检查。不再只取上轮外部试验的最长连续有效段。

正式输出：`/tmp/tianji_ceres_port_final_trace.log`；指标和文件指纹保存在[JSON记录](shared_root_ceres_port_20260918.json)。没有真实输入、socket或机器人执行器；模型参考不称作实测末端。

| 5～47秒、有效且接受的样本 | 左臂 | 右臂 |
|---|---:|---:|
| 样本数 | 8395 | 8395 |
| Ceres位置残差 P90 | 204.72 mm | 0.382 mm |
| 平滑参考位置跟踪误差 P90 | 204.72 mm | 54.07 mm |
| 平滑参考姿态误差 P90 | 0.0672 rad | 0.0707 rad |
| IK残差超过10 mm的比例 | 23.73% | 0% |

全程10200周期，映射就绪9983周期、相应控制候选接受9983周期；4次恢复完成（日志历史字段名为 `acknowledgements`，实际计数是恢复状态转换）。末尾失效停止后左右关节速度范数均小于4e-17 rad/s。

左臂最大求解残差约352.58 mm。**不能用“零映射就绪周期拒绝”掩盖这一跟踪问题。** 右臂求解精度较好而仍有平滑滞后；左臂除平滑外还存在 IK 求解残差。原因尚未由单独对照确证，下一步应对齐源/本地的同目标、同初值和完整 seed 历史，检查初始化恢复路径及局部解分支，不先改映射或扩大运动限制。

上轮外部试验跳过了前段无效区间，并使用其 Viewer 的目标初始化/回放逻辑；本次包含当前共享根恢复路径，不能直接把两组数字当作同条件算法比较。调试中还运行过完整 `mj_forward` 求值版本，其误差较大；该日志及早期快速 FK 试跑保留在 `/tmp/tianji_ceres_port_trace.log`、`/tmp/tianji_ceres_port_fast_trace.log`，不替代正式结果。

## 构建与复现

在 `tianji_teleop` 根目录，已有本工程 Pixi 依赖环境时：

```bash
env PATH="$PWD/.pixi/envs/default/bin:$PATH" cmake -S control -B control/build-ceres \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PWD/.pixi/envs/default" \
  -DTIANJI_ENABLE_CERES=ON -DBUILD_TESTING=ON
cmake --build control/build-ceres -j 4
control/build-ceres/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root_ceres.yaml \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr --ceres-reference-loop
ctest --test-dir control/build-ceres --output-on-failure -j 1
```

离线 probe 在内存中启用映射，不改配置，也不获得运动权限。Viewer 接入已通过启用副本的无输入启动测试，但本轮没有启动真实 PICO 输入会话或完整带输入窗口回放。若后续要用 Viewer，应建立显式启用的实验配置副本，保持 artifact 路径有效，并使用本地 `build-ceres/tianji_qp_ik_viewer`，而不是上轮外部二进制。

本轮测试覆盖源求解器的7项单测、8项本地控制/运动学测试、共享根确认及授权、Viewer默认关闭/禁止导出/禁止反馈模式/无输入零速度、无Ceres构建拒绝启用。第一轮完整测试暴露无输入测试对 `pico_enabled` 的错误假设，已改为检查有效输入、控制接受和速度；没有为通过测试而放宽运行门控。

最终构建及回归均为本轮执行结果：

- `cmake --build control/build-ceres -j 4` 成功，`ctest --test-dir control/build-ceres --output-on-failure -j 1`：101/101通过，141.79秒。
- `cmake --build control/build -j 2` 成功（`TIANJI_ENABLE_CERES=OFF`），`ctest --test-dir control/build --output-on-failure -j 1`：99/99通过，136.88秒。
- `git diff --check` 通过；原 reachable 配置和冻结 XML 的SHA256与移植前一致。
- 最终回归日志：`/tmp/tianji_ceres_port_on_ctest_final.log`、`/tmp/tianji_ceres_port_off_ctest_final.log`。旧SPARK、协议、状态机和无窗口合成输入回归包含在上述测试中；不代表真实输入或真机验收。

未提交、未 push、未改原始录制，未运行真机。跟踪验收尚未通过，不将软件测试结果替代真实输入仿真或硬件验收。
