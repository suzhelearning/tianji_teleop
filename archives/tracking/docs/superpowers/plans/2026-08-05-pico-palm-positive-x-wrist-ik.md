# PICO 掌心局部 +X 腕目标与固定骨长 IK 实施计划

**目标：** 修正掌心约束骨架的数据链，使 TCP 掌心观测先按掌心局部 `+X` 的标量腕掌距离反算腕目标，再以原始 PICO 骨架提供的肩位置、肘部分支和固定骨长重建上肢。

## 固定契约

- TCP 先且仅先应用一次：`T_G_H = T_G_C · T_C_H`。
- 腕与掌心姿态一致：`R_G_W = R_G_H`。
- 掌心位于腕部局部 `+X`：`p_G_H = p_G_W + R_G_H [d_WH, 0, 0]^T`。
- 因此腕目标为：`p_G_W* = p_G_H - R_G_H [d_WH, 0, 0]^T`。
- 现有 `wrist_to_palm_m` 三维 artifact 只作兼容输入；运行时取其欧氏范数作为 `d_WH`，不沿旧 artifact 的 XYZ 方向求腕点。
- 原始骨架提供肩位置、上臂/前臂长度、当前肘部 branch reference 和其他 20 个不修改的 pose。
- 目标腕可达时，修正骨架掌心必须与 TCP 掌心一致。
- 目标腕不可达时，将腕目标投影到固定骨长的可达边界，保持两段骨长；修正骨架掌心由投影腕点沿局部 `+X` 重建，同时保留 TCP 掌心作为独立观测并报告位置残差。
- 左右臂独立；任何一侧失败不能冻结另一侧或躯干、头部、下肢。

## Task 1：核心几何（TDD）

**Files**

- `src/pico_bridge/scripts/pico_palm_skeleton_filter_core.py`
- `src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py`

**测试先行**

- 90° 掌心旋转下，腕点必须沿旋转后的局部 `-X`，不能沿旧 artifact 的 Y/Z 分量。
- 可达目标严格满足固定上臂、前臂长度和零腕掌残差。
- 不可达目标保持固定骨长、设置 `reach_clamped`，并产生非零 TCP endpoint residual。
- 腕与修正掌心姿态均严格等于 TCP 掌心姿态。
- 原始肘点/上一帧肘点决定交圆 branch，非上肢 pose 不变。

**实现**

- 将 `wrist_to_palm_offset_m` 改为 `wrist_to_palm_distance_m`。
- 删除不可达时拉伸骨长的 `solve_endpoint_preserving_elbow` 路径。
- 始终先投影腕点到 `[|Lu-Lf|+epsilon, ratio*(Lu+Lf)]`，再调用三角形解析解。
- 修正 HAND 位置由最终腕点和 `[d_WH,0,0]` 重建；计算其与 TCP 掌心的残差。

**退出条件**

- focused core pytest 全部通过。

## Task 2：Artifact 与 ROS 节点（TDD）

**Files**

- `src/pico_bridge/scripts/pico_palm_skeleton_filter_node.py`
- `src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py`

**实现**

- 优先支持未来字段 `wrist_to_palm_distance_m`。
- 兼容现有 `wrist_to_palm_m` mapping/list，取 `norm(vector)`，状态标为 `loaded_legacy_vector_norm`。
- 节点状态明确发布 `wrist_palm_axis=palm_local_positive_x`、距离、clamp 和 TCP endpoint residual。
- `require_wrist_pivot_artifact=true` 时 artifact 缺失或非法仍 fail closed。

**退出条件**

- loader、状态和节点测试全部通过。

## Task 3：MuJoCo 可视化一致性（TDD）

**Files**

- `src/pico_bridge/scripts/smpl_mujoco_visualizer.py`
- `src/pico_bridge/test/test_smpl_mujoco_geometry.py`

**实现**

- 可视化 loader 使用与运行节点完全相同的标量距离兼容规则。
- 紫色腕点严格由橙色 TCP 掌心沿局部 `-X` 推导。
- 修正骨架掌心显示 IK 最终结果；TCP 掌心保持独立橙色目标，clamp 时二者允许出现可解释残差。
- 原始骨架坐标轴、controller 和其他 overlay 只由显式选项开启。

**退出条件**

- visualizer geometry tests 全部通过，默认显示不会产生含义重复的末端点。

## Task 4：文档与完整验证

- 更新设计文档、README 和运行说明中的局部 `+X` 语义。
- 运行 focused pytest、`py_compile`、`colcon build/test`、`colcon test-result --all` 和 `git diff --check`。
- 不修改、不删除 `MUJOCO_LOG.TXT` 或任何录制数据。
