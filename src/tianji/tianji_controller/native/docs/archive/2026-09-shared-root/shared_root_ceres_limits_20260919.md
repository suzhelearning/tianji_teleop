# Ceres 源模型关节限位与 Ruckig 对齐

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

历史版本说明：本文数据对应尚未接入 Pinocchio 的 Ceres 版本。后续源 `f615b8c` 更新及当前运行路径见 [新移植报告](../../verification/shared_root_ceres_f615b8c.md)。

2026-09-19。用户确认源工程默认模型符合机械臂，并要求参考其 Ruckig 限位。本轮仅更新独立 Ceres 模型参考路线，不启用真机，也不替换旧 SPARK 默认路线。

## 范围与来源

源模型为 `TJ_arm_control_pico_ee_ik/models/marvin_m6_qp_pico_fast.xml`，SHA256 `17cd6d29d23d5c58c7f078020c9f7ebe06d7863aa8b6585ec08109e96a9a1841`。
源配置为 `config/qp_ik_pico_ee_franka_ceres_lm_ruckig_mujoco.yaml`，SHA256 `d759268a688ac19faa7a3fc3743369a8f6aae69c8ce22431dca3703d4f414937`。
运行时不依赖该外部目录。

- 新增 [Ceres 专用 XML](../../../models/marvin_m6_wuji2_shared_root_ceres.xml)，与原共享根 XML 相比只有双臂 J2/J3/J4 共六个 `range` 属性变化。14 个关节角度及模型速度限制逐项对照源模型一致；手部、惯性、力矩、碰撞与连杆/TCP 定义未改。
- [Ceres 配置](../../../config/qp_ik_pico_shared_root_ceres.yaml) 的 `joint_limits` 和 Ceres `post_smoothing` 两个完整块与源配置一致。速度均为 4 rad/s，加速度 J1–J3 为 60、J4–J7 为 90 rad/s²；jerk 分别为 3000、4500 rad/s³；角度安全余量仍为 0.05 rad。`joint_acceleration_limits` 的 jerk 同步，旧后端配置不变。
- 新增 [冻结几何 artifact](../../../config/shared_root_robot_geometry_ceres.yaml)，哈希仅允许 Ceres 算法使用，仍校验模型文件指纹。旧 artifact 和旧 XML 均未改。
- 保留 0.1615 m 手掌 TCP 和共享根映射。URDF 仍仅作为独立 FK 几何参考，其历史关节限位未改；此分支 IK/Ruckig 的角度限位来自新 MuJoCo XML，不来自 URDF。不宣称整个模型与源工程逐位等价。
- `spark_shared_root.enabled: false` 不变；仍只允许模型状态参考、禁止关节导出。未启动设备、真实输入会话或真机运动。

| 角度范围（rad） | 原共享根模型 | Ceres 新模型 |
|---|---|---|
| J2，左右 | [-2.0944, 2.0944] | [-1.76, 2.0944] |
| J3，左 | [-3.1067, 0] | [-3.1, 3.1] |
| J3，右 | [0, 3.1067] | [-3.1, 3.1] |
| J4，左右 | [-2.5307, 0] | [-2.5307, 2.5307] |

J1/J5/J6/J7 不变。J2 下界实际上收紧，而 J3/J4 不再被旧模型限制在原先的单侧范围。

## 本轮同录制前后对照

使用全部 4427 帧 `recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr`，200 Hz 原生共享根状态机 + Ceres + Ruckig，追加 1 秒输入失效停止。下表统计 5～47 秒有效且接受的样本，每臂 8395 个；是模型关节参考 FK 与送入 IK 的目标之差，不是真实机器人实测误差，也不是原始映射正确性的独立证明。

| 指标 | 更新前 | 更新后 |
|---|---:|---:|
| 左臂 IK 位置残差 P90 | 204.72 mm | 12.43 mm |
| 左臂 IK 位置残差最大值 | 352.58 mm | 25.92 mm |
| 左臂 IK 残差 >10 mm 比例 | 23.73% | 11.30% |
| 左臂平滑参考位置误差 P90 | 204.72 mm | 30.54 mm |
| 右臂 IK 位置残差 P90 | 0.382 mm | 0.382 mm |
| 右臂平滑参考位置误差 P90 | 54.07 mm | 36.04 mm |

更新后左右姿态跟踪误差 P90 为 0.04381/0.04768 rad。两次均为 10200 周期、9983 个映射就绪/接受周期、0 个映射就绪拒绝周期、4 次恢复完成。更新后末尾双臂参考速度范数均小于 5e-16 rad/s。失效期日志仍存在极小速度下的 Ruckig failed 状态，不把末尾近零速度解读为所有失效周期均成功平滑。

结果明显改善，但左臂仍有约 26 mm 求解残差，平滑参考也有瞬态滞后，**跟踪验收尚未通过**。本轮同时对齐角度范围和 jerk，不将综合改善全部归因于单个参数。保留后续按剩余高残差帧检查局部分支/约束活跃集的工作，不在本轮擅自增加多 seed 或改变映射。

本轮前后日志摘要、样本数和 SHA256 保存在 [JSON 结果](shared_root_ceres_limits_20260919.json)。完整临时日志位于 `/tmp/ceres_limits_before_20260919.log` 和 `/tmp/ceres_limits_after_final_20260919.log`，可能随系统临时目录清理而消失。源求解器代码未改，墙钟 1.5 ms 预算下的重复运行可能有微小差异。

## 几何与回归

`test_shared_root_geometry` 同时保留旧模型测试，并对新关节范围每臂采样 100 个确定性姿态。新模型最大跨模型 TCP 位置误差：左 1.264e-6 m、右 1.148e-6 m；旋转误差：左 2.520e-6 rad、右 2.505e-6 rad，均小于原 1e-5 门限。腕点闭合变化小于 3.43e-16 m，Link5/6/7 原点偏差为 0。

新增源限位/运动包络断言和 Ceres-only artifact 门控测试。首次针对性回归发现旧 reset 测试误把下一周期 Ruckig 运动参考当作重置时 IK seed；改为直接比较重置前参考与重置后 IK goal（容差 1e-8），未放宽运行保护。

复现命令（工程根目录）：

```bash
env PATH="$PWD/.pixi/envs/default/bin:$PATH" cmake --build control/build-ceres -j 4
ctest --test-dir control/build-ceres --output-on-failure -j 1
control/build-ceres/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root_ceres.yaml \
  recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr --ceres-reference-loop
env PATH="$PWD/.pixi/envs/default/bin:$PATH" cmake --build control/build -j 4
ctest --test-dir control/build --output-on-failure -j 1
```

本轮两套构建成功，完整 CTest：Ceres ON 101/101 通过（138.33 秒），Ceres OFF 99/99 通过（130.41 秒）。日志分别为 `/tmp/ceres_limits_on_ctest_20260919.log`、`/tmp/ceres_limits_off_ctest_20260919.log`。测试包含旧 SPARK、协议/状态机及无窗口合成输入，未使用现场输入。`git diff --check` 和本报告相对链接检查通过；旧 reachable 配置、旧几何 artifact 和旧 XML 指纹与本轮前一致。

软件测试、离线录制参考对照不替代真实输入仿真或真机验收。未提交、未 push，未修改原始录制。
