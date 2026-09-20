# 共享根三后端在线 MuJoCo 对照（2026-09-19）

本页保留该次对照与 DLS 来源核对，不据此判断当前默认选择或全场景性能优劣。
当前默认 Franka DLS＋Ruckig，启动与验收边界见[交互仿真](ceres_interactive_sim.md)。

## 范围与复现

三组均在当前工程 Viewer 中，按录制时间回放原始 UDP 输入，在线运行映射、IK、
运动参考生成和模型状态提交。不是预先计算关节轨迹后播放。

- SPARK：`qp_ik_pico_shared_root_reachable.yaml`，保留当前默认 SPARK／速度 QP 参数。
- Ceres LM＋Ruckig：`qp_ik_pico_shared_root_ceres.yaml`，无新备用初值或搜索调参。
- Franka DLS＋Ruckig：新增 `qp_ik_pico_shared_root_dls.yaml`，独立后端，不替换旧通用 DLS。

同一共享根映射、源模型 URDF/XML、161.5 mm 掌心 TCP、200 Hz、关节位置／速度／
加速度／jerk 限值及用户 Home：
左 [55, -65, -70, -60, 60, 0, 0]°，右 [-55, -65, 70, -60, -60, 0, 0]°。
正式配置仍保持共享根默认关闭，仅运行副本启用。串行、回环、无窗口、禁止关节导出。

**边界：这是 MuJoCo model-reference 模式，不是带执行器动力学的实际反馈跟踪，更不是真机验收。**
表中的跟踪误差是控制器命令目标相对当前模型参考的误差，不是原始 VR 输入误差。
SPARK 会改变中间命令并触发保持，因此另报“源帧身份相同且三组命令位姿相同”的子集。
该子集不是完整动作覆盖，也不能单独证明某个求解器普遍优越。

原始输出保存在工程内：
`recordings/shared_root/benchmarks/three_way_20260919_2352/`。
其中 `manifest.json` 保存完整启动命令、输入／配置／二进制 SHA256、结束状态；
各数据集目录保存三组运行配置、日志、主遥测与关节遥测。

复现（输出目录必须不存在）：

```bash
pixi run build
pixi run python control/scripts/run_shared_root_three_way.py \
  --output recordings/shared_root/benchmarks/three_way_NEW \
  --trace recordings/shared_root/zhoujie_actions_20260918_041020/input.tjvr \
  --trace recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr
pixi run python control/scripts/report_shared_root_three_way.py \
  recordings/shared_root/benchmarks/three_way_NEW
```

## DLS 移植核对

参考工程版本 `f615b8c2931601957315d1d3ea7f8aad8bb369a6`。

- 源 `src/iterative_pose_dls.cpp` SHA256：
  `5b29519bf8c1bfd4125be410184193380b2fd87e2a76af8a25863d7abaae30fa`。
- 源 `src/pico_ee_franka_dls.cpp` SHA256：
  `c506ab34d1f7c825d043610d8c2215e531fce85517e939bb844492679ef5d166`。
- 两份实现仅作隔离类／输入／辅助函数命名和 include 调整，规范化后逐字一致；
  未改旧 `iterative_pose_dls.cpp`。
- 参数取自源 `qp_ik_pico_ee_franka_dls_ruckig_mujoco.yaml`：
  20 次迭代，固定 Home 零空间参考，nominal gain 0.5，上一最佳原始 IK 解作为下一次初值；
  保留最佳可行近似解，不强求每周期达到零残差。
- 禁用内部旧四阶规划器和直接速度截断；原始 IK goal 交给在线 Ruckig。
- 候选 FK/Jacobian 使用 Pinocchio；复用已校验源 URDF、显式 TCP 与源版 Ruckig limiter。
  实际／显示状态来自 MuJoCo。
- 为保持当前工程安全语义，双臂提交是事务式的；输入过期时朝当前参考有界停止、丢弃旧 IK 初值。
  不照搬原工程继续追逐旧目标等行为。仅允许模型状态模式，禁止关节导出。
- 本轮不是重新运行原工程完整应用的跨工程 A/B；不据此宣称全部会话／故障语义与原工程相同。

## 历史软件验证（当时使用 build-ceres；当前统一 control/build）

- `cmake --build control/build-ceres -j 2` 成功。
- `ctest --test-dir control/build-ceres --output-on-failure -j 1`：105/105 通过，147.06 s。
- 之后补齐可选肘平面约束的 full Pinocchio sample 和源速度包络接线，重建后：
  `ctest --test-dir control/build-ceres --output-on-failure -R '(franka_dls|ceres)' -j 1`：
  7/7 通过，13.97 s。
- 新增源 DLS 14 项单元用例、DLS 控制器 13 项用例和 Viewer 安全入口测试。
  控制器测试含 Pinocchio／MuJoCo FK/Jacobian 一致性、初值复位、过期停止、双臂原子提交，
  以及 100 个周期逐一比较独立 DLS→Ruckig 与 Viewer 所用控制器的 goal/q/qdot/qddot。
- 本轮未重建 Ceres-OFF 配置；未启动真实输入、设备 SDK 或真机；未提交或 push。
