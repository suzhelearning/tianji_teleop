# DLS/Ceres 后端、源模型限位与 Ceres f615b8c 移植记录

## 当前后端与限位

默认交互仿真、真机和采集均使用共享根 Franka DLS＋Ruckig；
SPARK／mapped-palm 为显式可选后端，Ceres 与 PICO2 裸手入口仍仅仿真。
真机共用参考生成算法，由独立安全执行器保留实测初态、限速、反馈和 Enter／对齐／Home 门控；
源失鲜、epoch 改变、映射／IK 拒绝停止，不启用仿真自动恢复，尚未获真机运动验收。
同算法不保证模拟与硬件实测轨迹相同。入口见[交互仿真与导出边界](ceres_interactive_sim.md)
和[真机流程](../../../README-reference.md#二真机遥操作)，
DLS 核心来源与参数核对见[三后端对照](shared_root_three_way_20260919.md)。

- DLS/Ceres：共享根校验、映射/投影/恢复 → 末端 IK → Ruckig → 模型参考；
  不额外运行 SPARK 形状 IK、headroom 或第二层速度 QP。
- SPARK reachable 配置：新输入触发姿态 IK，参考处理与速度 QP 在控制循环运行；
  不额外串联 DLS/Ceres 的 Ruckig。复用同一序号的 IK 是缓存，不是离线预计算轨迹。
- 三个当前对照配置使用仓库内源限位 URDF/XML 与 161.5 mm TCP；不批量改变历史 SPARK 配置。
  DLS/Ceres 的候选 FK/Jacobian 使用 Pinocchio，模型状态/TCP 使用 MuJoCo，启动时校验一致性。
- 左右 J2 `[-1.76, 2.0944]`、J3 `[-3.1, 3.1]`、J4 `[-2.5307, 2.5307]` rad，
  其余关节范围以冻结模型为准；角度安全余量 0.05 rad。
  模型速度上限 4 rad/s；加速度 J1–J3 为 60、J4–J7 为 90 rad/s²；
  jerk 分别为 3000、4500 rad/s³。SPARK reachable 开启硬 jerk 约束。
- Home：左 `[55,-65,-70,-60,60,0,0]°`，右 `[-55,-65,70,-60,-60,0,0]°`。
  SPARK 从此姿态启动，不因此增加 Ceres 式 Home 吸引项。交互回 Home 使用额外低速包络。
- 最佳努力近似解的 `accepted` 不代表残差达标。失效输入受限停止，单臂失败不提交
  另一臂的新候选或 seed/平滑历史；不为“与原版一致”绕过门控，也不宣称硬实时。

源 TCP 默认 95 mm，与本工程 161.5 mm 不同；移植源码一致不等于完整应用语义一致。
旧 MuJoCo 求解路径与旧限位数字仅见[归档](../archive/2026-09-shared-root/README.md)，
不能用于描述当前 Pinocchio 后端。

## 历史移植记录（2026-09-19）

以下结果保留对应轮次的版本、指纹和比较口径，不是当前配置重新测试结果。

源工程 `TJ_arm_control_pico_ee_ik`，提交 `f615b8c2931601957315d1d3ea7f8aad8bb369a6`。本报告替代此前关于当前 Ceres 运行后端的描述；旧报告保留为历史数据，不混用指标。

## 本轮实际接入

- 新增独立 `CeresPinocchioArmKinematics`，移植源工程固定大小工作区和 `sampleTcp()` 路径；没有改旧 SPARK 的 Pinocchio 实现。Ceres 的候选 FK/Jacobian 和参考 TCP 都使用 Pinocchio，实际模型状态/TCP 仍来自 MuJoCo。
- 源 URDF 按字节复制为 `models/marvin_m6_ceres_source.urdf`，SHA256 `f72dcd970dd9d539c55c5897e63ebd18d5e61b6895ef460fe0d73c8179cec006`。配置使用仓库相对路径，不依赖源工程的本机目录。空路径、未评审指纹、限位不一致或 FK/Jacobian 校验失败均拒绝启动，不静默 fallback。
- Ceres 本体源码未改，与本轮源文件 SHA256 相同：`58600f8938249ca1dd8c5e0a705aafb67be9ed0ceb7499fb3457ce587dd27074`。初始姿态、Ceres 参数和 Ruckig 限制对齐源配置。
- 源默认 Link7→TCP 为 95 mm；当前共享根仍使用已冻结的 161.5 mm 手掌 TCP。新增显式 TCP 构造参数，并在启动时与选定 MuJoCo 模型校验，不把两种控制点混用。冻结 Ceres 几何 artifact 同步引用新 URDF；其他路线的模型与 artifact 不改。
- Ceres 专用 Ruckig 限制器同步新版“到位浮点残差规范化”修复，除类型/头文件重命名外与源实现一致。只有全臂位置、速度、加速度都落在按量纲缩放的 1e-12 舍入范围内才规范化输入导数；不跳变位置、不抹去真实运动，jerk 仍对原内部加速度检查。
- 分别记录 IK、Ruckig 和两者流水线耗时，以及 Ruckig 调用、Pinocchio 后端标记。原生离线 audit 输出这些字段；Viewer CSV 将字段追加在末尾，已有列序不变。
- Viewer 的共享根接线改为使用冻结 artifact 指定的 URDF，修复新 artifact 配合旧硬编码 URDF 时的启动不一致；未放松指纹验证。

当次尚未新增 Franka DLS 后端；后续已独立接入，不能将该历史状态解释为当前不支持 DLS。

## 明确保留的安全差异

当前工程仍在失效输入时以当前关节参考执行受限停止，恢复时清理求解历史；双臂成功才一并提交结果。源控制器对 stale 目标的内部保持语义及逐臂提交不同。这些差异属于当前共享根输入新鲜度和事务门控，不以“原版一致”为由删除。

因此不宣称所有故障/恢复周期与原版逐位等价。`accepted` 也不等于 IK 残差达标。新配置仍默认关闭、模型参考模式限定和禁止关节导出均保留，未授权真机运动。

## 验证与对照

最终遥测改动完成、两种构建均成功后顺序执行：

- `ctest --test-dir control/build-ceres --output-on-failure -j 1`：**102/102 通过，138.03 秒**，Ceres ON。
- `ctest --test-dir control/build --output-on-failure -j 1`：**100/100 通过，132.32 秒**，Ceres OFF。
- `git diff --check`：通过。旧 reachable 配置、旧几何 artifact、旧共享根模型、输入契约指纹与本轮开始时一致。

日志：`/tmp/ceres_f615b8c_ctest_on_final.log`、`/tmp/ceres_f615b8c_ctest_off_final.log`。上述为软件、合成输入/无窗口回归，不包含真实输入现场会话或真机验收。

新增/扩展验证包括：源 Ruckig 七项回归（含真实运动不被清零、到位残差后可重新启动、jerk 检查）；Pinocchio 每臂 100 个姿态的 TCP/Jacobian 一致性；缺失/错误 URDF 无 fallback；分段耗时字段；Viewer 默认关闭、禁止导出、无输入零运动。

同条件源工程对照使用源默认模型、URDF、初始姿态及原配 1.5 ms 预算，双方均使用 Pinocchio，保留全部 10200 周期的失效/恢复事件、各自连续维护 seed/Ruckig 状态，没有中途注入另一组状态。原 161.5 mm 手掌目标通过固定变换表达在源 95 mm TCP；变换不改原版模型或求解器。

该对照范围是末端目标进入控制器后的完整求解/平滑/状态更新链，不冒称两工程不同的 VR 上游映射、Viewer 交互、真实输入或真机验收都已完成。此前手动清空原版 Pinocchio 配置的 MuJoCo fallback 实验不能作为原版默认链的性能结论。

### 本轮离线数值（2026-09-19）

对照录制 SHA256：`20340067256a3a78205c78a0d5ba6fc01f818d14da5e1e57f116391451df2f2b`。冻结的源 TCP 目标序列 SHA256：`4ee989eb1b923fa4ff09e41276b43ff0c0ec18e4d82d9a816076869ae6f61002`。每轮 10200 周期、每周期双臂各一行；统计窗为 5～47 秒有效且接受的样本，每臂 8395 条。P90 使用 nearest-rank（NumPy `inverted_cdf`）。跟踪误差是步进前的模型关节参考 FK 相对目标，不是硬件实测误差。

| 同源 95 mm TCP 对照 | 左 IK P90 / mm | 右 IK P90 / mm | 左跟踪 P90 / mm | 右跟踪 P90 / mm |
|---|---:|---:|---:|---:|
| 原版第 1 轮 | 11.089 | 0.380 | 28.108 | 33.489 |
| 移植第 1 轮 | 10.795 | 0.380 | 28.108 | 33.489 |
| 原版第 2 轮 | 10.980 | 0.380 | 28.108 | 33.489 |
| 移植第 2 轮 | 10.795 | 0.380 | 28.108 | 33.489 |

四次运行均无有效输入求解拒绝。原版每次在失效输入上仍有 434 个单臂内部 accepted，当前为 0；该计数不代表原版获得硬件运动授权。两轮中都有偶发耗时超过 1.5 ms 的情况，时间预算和调度噪声仍会影响结果，不把单轮微小差异当成优化收益。

**参考轨迹差异：**0.645 秒之前有效输入的 IK goal 最大差为 `3.9968e-15 rad`，但 Ruckig 后的关节参考在 0.125 秒已首次出现超过 `1e-10 rad` 的差异，该前缀最大差为 `0.0174244 rad`。首次超过 `1e-6 rad` 的 IK goal 分歧发生在 0.645 秒失效事件；全程关节参考最大差为 `0.244514 rad`。因此，不能把全部轨迹差异归因于 stale 门控，也不能据此声称两边全流程逐位等价。

追加限制器隔离实验：使用同一份当前 `CeresTrajectoryLimiter7`、相同初始状态/配置和 reset 事件，分别输入双方保存的逐周期 IK goal，回放前 129 周期/臂。两路结果与各自原始全流程的关节参考最大差均为 **0**；两路之间左/右最大差为 `0.0142812 / 0.0174244 rad`。这将首段参考差异收敛到“极小 goal 浮点差异进入 Ruckig 后产生不同轨迹”，无需引入门控或不同限制器实现来解释。限制器源代码在类型/头文件重命名后完全一致。该实验只解释首段，不将其外推为全部恢复过程和残差的唯一原因；尚未进一步定位具体 Ruckig 数值分支，也未通过任意舍入目标来掩盖差异。

当前正式 **161.5 mm 手掌 TCP** 配置另跑完整 `--ceres-reference-loop`：20400 个单臂周期均标记 `pinocchio=1`，有效映射拒绝 0，输出的 smoother 失败/无效/拒绝 0，停止后双臂速度均为 0。左/右 IK P90 为 **13.060 / 0.382 mm**，模型参考跟踪 P90 为 **30.545 / 36.057 mm**。这说明新版已接入且该录制可离线运行，不说明左臂残差已解决，更不说明实机验收通过。配置 SHA256：`ea684f97df687368f12a366b7fb09bdd27013c4a55611a99059f05ce8a0822cf`。

本机临时审计文件：`/tmp/ceres_f615b8c_parity.FgpGMb/`（独立编译/运行脚本、对照 harness、四份逐周期结果及 `results.json`），`/tmp/ceres_f615b8c_native_trace.log`（当前配置完整输出）。这些是临时产物，清理后需重新生成，不是仓库持久测试资产；本节保存本轮结论及输入指纹。

## 复现当前共享根路线

```bash
pixi run build
pixi run test-native
pixi run control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root_ceres.yaml \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr --ceres-reference-loop
```

若创建临时启用配置副本，除两个 artifact 路径外，还应将 `controller.pico_ee_dls_kinematics_urdf_path` 正确解析到仓库文件。文件未启用不等于获得设备运动授权。未提交、未 push、未改原始录制。
