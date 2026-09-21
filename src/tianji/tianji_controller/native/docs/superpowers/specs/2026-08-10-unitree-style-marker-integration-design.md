# Unitree 风格 RViz 6D Marker 交互整合设计

日期：2026-08-10
状态：设计已确认，待编写实施计划

## 1. 目标与范围

当前工程已经具备 MuJoCo 6D marker、双臂 QP-IK、异步控制线程和安全检查。下一步目标是把 `unitree_ik` 中已经验证过的 RViz 风格交互语义整合到当前 viewer，使用户可以用接近 RViz gizmo 的方式直接拖动末端目标：

- 鼠标拖动可见的 XYZ 平移箭头，沿对应坐标轴移动末端目标；
- 鼠标拖动 XYZ 旋转环，绕对应坐标轴旋转末端目标；
- 空白区域的拖动仍然只操作 MuJoCo 相机，不产生 IK 目标命令；
- 鼠标按下时选择目标和 gizmo，按住期间捕获当前 handle，松开后结束拖动；
- 保留当前世界坐标系/局部坐标系切换、左右臂目标和手动目标预览等能力；
- 所有目标修改继续通过现有 `ViewerCommand::kSetManualTarget` 进入控制线程，不直接改写关节状态或绕过 SafetyGuard。

本设计只覆盖 viewer 的目标选择、handle 选择、鼠标事件路由和屏幕空间拾取整合。QP 建模、求解器、控制线程、SPSC command queue 和安全策略不在本次设计的修改范围内。

## 2. 现有实现与参考实现

### 2.1 当前工程

当前 `feature/mujoco-cpp-qp-ik-v1` worktree 已有以下基础设施：

- `InteractiveMarker6D`：保存 6D pose、世界/局部坐标系、平移和旋转 handle、drag 状态，并提供平面、轴线、旋转环约束；
- `mujoco_marker_adapter`：把 MuJoCo 相机转换为世界空间射线，并把 marker 几何体追加到 scene，包含 3 个箭头、144 段旋转环和中心球；
- `run_qp_ik_viewer`：处理鼠标和键盘事件，把手动目标通过非阻塞命令队列发送给 QP 控制线程，并用 `ManualTargetPreview` 跟踪尚未确认的目标；
- 现有单元测试、headless viewer 测试和安全检查，已经覆盖当前解析射线拖动的数值稳定性。

### 2.2 `unitree_ik` 参考语义

参考工程的交互约定是：空白处拖动相机，点击目标附近先确定目标，命中箭头或旋转环后捕获 handle；拖动过程中即使鼠标离开 handle 的初始拾取范围，也继续编辑同一个 handle。其 gizmo picker 使用屏幕空间的投影线段/环段距离，而不是要求鼠标射线必须穿过细小的三维几何体。

参考工程的局部坐标变换也符合 RViz 直觉：平移沿目标当前局部轴，旋转在目标当前局部轴上累加。当前工程已有更完整的解析射线约束，因此只吸收参考工程的交互时序和屏幕空间拾取语义，不整体复制参考工程的 Pinocchio、task configuration 或异步 IK 架构。

## 3. 已确认的设计决策

### 3.1 保留当前架构，增加交互适配层

当前 `InteractiveMarker6D` 继续作为 pose 和拖动约束的单一实现，MuJoCo adapter 负责可视化和屏幕空间几何查询，viewer 负责事件路由。新增逻辑不得把目标 pose 直接写入 joint state，也不得在渲染线程内运行 QP 求解。

### 3.2 屏幕空间拾取作为可见 handle 的权威入口

鼠标按下时，优先把当前帧中的可见箭头和旋转环投影到 viewport，选择与鼠标最近且在像素容差内的 handle。这样用户点击的是屏幕上看到的 handle，而不是一个必须精确穿过薄几何体的三维射线体积。

现有解析射线逻辑保留两项职责：

1. 在已经选定 handle 后，计算真实拖动的轴线、平面或旋转环约束；
2. 屏幕投影不可用、相机退化或几何接近奇异状态时作为 fallback，并保证无效输入不会产生 NaN。

屏幕拾取只解决“选中哪个 handle”，不替代现有拖动约束数学。

### 3.3 目标选择与 handle 选择分两步

鼠标按下按以下顺序处理：

1. 先尝试命中目标中心/目标可点击区域；命中时切换当前 active arm；
2. 若没有命中目标，则保留当前 active arm，继续在该目标的 marker 上进行 handle 拾取；
3. 若命中可见箭头或旋转环，启动并捕获该 handle；
4. 若没有命中 handle，则把这次操作交给相机交互；
5. 如果既没有可用目标也没有可用 handle，则忽略 marker 编辑，不伪造 IK 命令。

这样既支持“先点目标再拖 gizmo”的参考行为，也避免必须点击中心小球才能拖动当前已选目标的操作负担。

### 3.4 活动 handle 在释放前保持捕获

成功启动拖动后，记录 active arm、坐标系、handle 类型、按下位置和拖动初始 pose。后续鼠标移动只更新这个 handle，不重新拾取，也不因为鼠标离开箭头/旋转环的投影范围而切换到相机。鼠标释放、窗口焦点丢失、模式切换或目标切换时结束/取消捕获。

### 3.5 目标更新继续走 command/preview/ACK 链路

每个有效拖动更新都构造 `ViewerCommand::kSetManualTarget`，由现有 SPSC queue 交给 QP 控制线程。viewer 端的 `ManualTargetPreview` 使用 command id 记录最新未确认 pose；控制线程确认后再清理对应 preview。队列已满时立即丢弃该次更新并计数，不能阻塞渲染或鼠标事件线程。

## 4. 交互行为契约

| 操作 | 命中条件 | 行为 | 是否产生 IK 目标命令 |
| --- | --- | --- | --- |
| 空白处按下并拖动 | 未命中目标或 handle | 进入现有 MuJoCo 相机拖动 | 否 |
| 目标区域按下后拖动空白 | 命中目标，未命中 handle | 目标成为 active arm；若未启动 handle，保留相机/选择语义，不发送 pose | 否 |
| 平移箭头按下并拖动 | 屏幕空间命中箭头 | 捕获轴向平移 handle，沿该轴更新 pose | 是，更新有效时 |
| 旋转环按下并拖动 | 屏幕空间命中旋转环 | 捕获轴向旋转 handle，绕该轴更新 pose | 是，更新有效时 |
| 已捕获 handle 后移出投影范围 | 已有 active drag | 继续使用原 handle 计算，不重新拾取 | 是，更新有效时 |
| 鼠标释放 | 任意 active drag | 提交最后一个有效状态并结束捕获 | 不额外生成空命令 |
| 窗口失焦/模式切换/目标切换 | 已有 active drag | 清除 drag state；保留最后一个已排队的有效目标 | 否 |

坐标系选择沿用当前 viewer 的世界/局部模式。局部模式下，箭头和旋转环按 marker 当前 orientation 定义；世界模式下按固定世界 XYZ 定义。目标的平移和旋转仍由 `InteractiveMarker6D` 的现有局部/世界变换完成。

## 5. 组件边界与接口方向

### 5.1 `apps/run_qp_ik_viewer.cpp`

负责把 GLFW/viewport 鼠标事件转换成 marker interaction event，并维护 active target/active drag 的生命周期：

- mouse down：目标候选解析、屏幕空间 handle 拾取、启动 drag 或转交相机；
- mouse move：若 marker drag active，调用当前 marker 的 drag update；否则沿用相机逻辑；
- mouse up、focus loss、模式切换：结束或取消 drag；
- 将有效 pose 包装成现有 command，并更新 preview/统计信息；
- 保留当前键盘控制和 headless 路径。

viewer 不拥有投影公式，也不重复实现轴/环最近点算法。

### 5.2 `include/tianji_qp_ik/mujoco_marker_adapter.hpp` / `src/mujoco_marker_adapter.cpp`

新增或扩展纯几何辅助接口，用于在当前 MuJoCo scene/camera 下完成屏幕空间查询：

- 世界点投影到 viewport 像素坐标；
- 对箭头轴线和旋转环段计算鼠标到投影几何的最近距离；
- 返回最近 handle、距离和必要的深度/可见性信息；
- 对 viewport 尺寸、相机前方方向、近裁剪面和数值退化做有限性检查。

picker 的输出只描述候选 handle，不修改 marker pose，也不向控制线程发送命令。箭头和旋转环的投影几何应复用 adapter 生成可视化 geometry 时使用的坐标和坐标系，避免显示位置与可点击位置分叉。

### 5.3 `InteractiveMarker6D`

原则上不改动其已有 pose、frame、constraint 和 analytic drag 实现。若 viewer/adaptor 需要更清晰的公开 helper，只添加不改变语义的薄接口，例如读取当前 handle 的世界基向量、环中心、初始 pose 或判断 drag result 是否有限。所有新增接口必须保持可单元测试。

### 5.4 控制线程、队列和安全模块

本次不修改 QP 构造、OSQP/qpOASES 后端、控制线程调度、SPSC queue 协议、SafetyGuard 或命令 ACK 语义。marker integration 只能调用既有边界。

## 6. 屏幕空间拾取算法

### 6.1 投影坐标

使用当前 MuJoCo viewport 和 scene camera，把 marker 的世界空间几何投影为像素坐标。投影失败、点在相机后方或输入包含非有限值时，该候选不参与比较。像素坐标转换必须明确 viewport 原点和鼠标坐标方向，避免再次引入相机水平轴镜像问题。

### 6.2 平移箭头

对每个可见坐标轴构造“轴起点到箭头头部”的投影线段。计算鼠标点到线段的最短像素距离，在距离小于平移 handle 容差的候选中选择最近者。若多个候选距离相同，优先深度更接近相机者，再以固定轴顺序作为稳定 tie-breaker。

### 6.3 旋转环

每个旋转轴以 marker center 为中心，把环离散成与显示一致的 48 段或等价的固定采样段，逐段投影并计算鼠标到屏幕线段的最短距离。只在环投影有效、长度足够且距离处于旋转 handle 容差内时返回候选。环段数量和半径应与 adapter 的渲染设置保持一致；不要求使用三维射线精确穿透环面。

### 6.4 候选优先级和相机遮挡

候选排序先按像素距离，再按深度和稳定的 handle 顺序。对于投影退化或离屏几何，不降低空白相机操作的可靠性。marker geometry 本身是交互 overlay；若当前 MuJoCo scene 无法提供可靠深度，则不引入依赖深度缓冲的复杂遮挡规则，而使用有限性和投影可见性检查。

## 7. 拖动、异常与并发处理

### 7.1 有效拖动

屏幕空间 picker 选出 handle 后，调用现有 `InteractiveMarker6D` analytic drag：

- 平移：把鼠标射线与轴线/约束平面求交，得到沿选定轴的增量；
- 旋转：把鼠标射线投影到旋转环所在平面，使用相对初始向量的有符号角度；
- 当前解析实现中的 screen-space fallback 继续生效，用于射线与约束几何平行等情况；
- 得到的 pose 经过有限性和合理约束检查后才形成 command。

拖动计算以按下时的初始 pose 为基准，避免连续累计浮点误差，也保证鼠标在捕获期间移动时结果稳定。

### 7.2 无效几何

以下情况只忽略本次 update，不发送命令，也不写入无效 pose：

- 鼠标射线无效；
- 轴线/平面/环面求交分母接近零；
- 旋转环投影退化或初始向量长度过小；
- 投影/反投影出现 NaN 或无穷值；
- candidate、active handle 或 viewport 信息不完整。

无效拖动不能污染上一次有效的 preview 和 command id。

### 7.3 队列满与控制线程延迟

发送 command 使用现有非阻塞 push。push 失败时递增已有 command drop 统计或为 marker 增加明确统计项，但不等待消费者、不分配不可控的增长队列、不改变 QP 控制周期。preview 只记录成功入队的 command。

### 7.4 状态取消

以下事件清除 active drag：鼠标释放、窗口失焦、切换世界/局部模式、切换 active arm、退出 marker edit 模式和 viewer 关闭。取消只清除本地捕获状态，不撤销已经成功入队的最后一个目标；控制线程仍按其已有 ACK 和安全策略处理。

## 8. 测试与验收

### 8.1 纯几何单元测试

在 `tests/test_mujoco_marker_adapter.cpp`（必要时配套 `InteractiveMarker6D` 测试）增加：

1. 屏幕投影下鼠标靠近箭头时选择对应轴；
2. 同时靠近多个候选时选择最近者，并验证 tie-breaker 稳定；
3. 鼠标远离所有 handle 时返回 no-hit；
4. 旋转环离散段可以被选中，且三个旋转轴可区分；
5. 已有 active handle 时，鼠标移出初始拾取范围仍不改变 handle；
6. 世界/局部模式下投影方向和返回轴一致；
7. 解析射线平行、相机退化、旋转向量退化时结果有限且不产生 NaN；
8. 当前相机水平轴方向修正的回归样例继续通过。

### 8.2 viewer/系统测试

- 空白鼠标拖动只改变相机，不产生 `kSetManualTarget`；
- 命中箭头/旋转环后产生命令，command id 和 `ManualTargetPreview` 一致；
- queue 满时 viewer 不阻塞，drop 统计正确；
- control thread 能 ACK 手动目标，preview 正确收敛；
- 现有 `ctest --test-dir build --output-on-failure` 全部通过；
- 现有 headless QP-IK viewer 运行完成，`control_failures=0`，不因 marker 交互改动而改变控制线程的 deadline/command 统计语义。

### 8.3 人工 GUI 验收

在有显示环境的情况下，至少验证：

- 平移箭头拖动时末端目标只沿所选轴移动；
- 旋转环拖动时末端目标绕所选轴旋转；
- 屏幕左右方向拖动不会出现镜像反向；
- 空白区域拖动仍可 orbit/pan/zoom；
- 拖动中移出 handle 后仍持续编辑，松开后相机不会接管；
- 左右臂切换、世界/局部模式切换以及窗口失焦不会留下卡住的 drag state。

## 9. 文件变更边界

预期变更文件：

- `apps/run_qp_ik_viewer.cpp`：鼠标事件、目标路由和 active drag 生命周期；
- `include/tianji_qp_ik/mujoco_marker_adapter.hpp`：屏幕投影/picker 的公开接口；
- `src/mujoco_marker_adapter.cpp`：投影和最近 handle 实现；
- `tests/test_mujoco_marker_adapter.cpp`：picker 和交互回归测试；
- `README.md`：补充 RViz 风格 marker 操作说明。

可能的小范围辅助变更：

- `InteractiveMarker6D` 的头文件/实现，仅在现有 analytic drag 需要公开无语义变化的几何 helper 时修改。

明确不修改：

- QP 求解器、QP 约束和权重；
- OSQP/qpOASES backend 及 benchmark；
- 控制线程和 SPSC command queue 协议；
- SafetyGuard、命令 ACK 和执行器接口；
- `unitree_ik` 工程本身。

## 10. 风险与取舍

- 屏幕空间 picker 的像素容差过小会让箭头难以点击，过大会造成相邻 handle 误选；容差必须通过纯几何测试和 GUI 验收校准。
- 投影结果与渲染结果若各自维护坐标或尺寸，会出现“看得到但点不中”；因此 adapter 的 picker 必须复用渲染几何的轴向、长度、半径和坐标系。
- 解析射线拖动和屏幕空间选择是两种不同坐标约束；设计明确让前者负责拖动、后者负责选中，避免为追求统一而牺牲平行/退化情况下的稳定性。
- marker 交互仍通过已有控制链路，因此渲染线程不会获得即时关节响应的保证；验收关注 command 入队、preview 和控制线程 ACK 的正确关系，而不是绕过异步架构追求同步写关节。

本设计完成后，下一步再将其拆成可独立验证的小步实施计划；在计划获确认前不开始修改实现代码。
