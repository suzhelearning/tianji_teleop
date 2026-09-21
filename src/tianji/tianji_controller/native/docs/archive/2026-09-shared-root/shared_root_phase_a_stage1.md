# Shared-root Phase A：阶段 1 审计（未通过）

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

## 基线与结论

- 审计日期：2026-09-17。
- 仓库 HEAD：`b0713576724499d229e12155d7db1daed096e9fe`；审计开始时工作区干净。
- 依据：外部《天机双臂共享肩坐标系几何映射与 SPARK 融合：V2.1 实施方案》，尤其 3.1、3.2 与阶段 1 门槛。
- 状态：**输入来源契约不满足方案，阶段 1 未通过，阶段 2～6 未开始。**
- 按用户“无法从当前协议或模型得到明确证据时停止”要求，不猜测缺失字段，不生成占位 artifact 或声称可用的实验 profile。

这不是已证实的旧遥操功能缺陷，而是新方案对现有输入的语义假设不成立。

## SharedRootInput 来源审计

路径以下均相对仓库根目录。

| 方案字段 | 当前可证实来源 | 契约判定 |
|---|---|---|
| 左/右 shoulder、elbow、wrist、hand | `control/include/tianji_qp_ik/pico_teleop_protocol.hpp`，索引分别为 0/1/2/3、4/5/6/7 | 索引明确；坐标系不是方案 H |
| `p_control_H` | M0 的重建掌心经 bridge 变换后进入 skeleton hand | 有候选控制点，但不能直接声明为 PICO 世界 H 下的 solver TCP |
| `p_shape_H` | 当前 hand 已被 M0 改写；协议没有独立原始解剖学 hand 字段 | **不能满足方案的解剖学点契约** |
| `R_hand_raw_H` | skeleton hand rotation 来源为 M0 的 `palm_quat`，再经 bridge 旋转 | 不是未经修正的原始 hand 旋转，也不是 H 中表达 |
| `R_palm_H` | 现有 mapped-palm 对 skeleton hand rotation 后乘一次左右 basis | basis 实现有依据；不能沿用错误的 H/raw 名义冻结新接口 |
| 有效性 | `upper_limb_skeleton.valid`、`rotations_valid` | 现有有效性不能证明解剖学 hand 存在或证明 TCP 点语义 |
| `sequence`、`source_timestamp_ns`、`receive_monotonic_ns`、`tracking_epoch` | `PicoTeleopFrame` 同名字段 | 字段存在；尚未完成全链路时间行为测试 |
| `resynchronization_generation`、`stream_discontinuity` | receiver-local 字段，协议头文件有明确注释 | 可复用，不需要扩展线协议 |

### 阻塞 1：进入消费者的骨架已经换过参考系

`tracking/src/pico_bridge/src/tianji_teleop_geometry.cpp:204` 起：

1. 以双肩中点为原点、肩连线为 Y、肩中点到 spine2 的投影为 Z，建立 `pico_shoulder_frame`。
2. 构造 `robot_midpoint_frame`。
3. `pico_to_robot_rigid = robot_midpoint_frame * pico_shoulder_frame.inverse()`。
4. 所有 8 个上肢点和旋转均经过该变换后装入 `upper_limb_skeleton`。

因此，方案的“输入 H 为 PICO corrected 世界且 +Z 是固定世界竖直”不能直接用于此输入。
该 bridge 使用躯干方向构造 Z，不是只消除世界 yaw。协议不提供逆变换所需的原始肩 frame/spine2。
不能把 bridge 输出重命名为 H 就宣称恢复了原世界竖直，也不能无证据再 canonicalize 一次。

### 阻塞 2：独立解剖学 hand 没有保留

`tracking/src/pico_bridge/scripts/pico_palm_skeleton_filter_core.py:389` 起明确写入：

```text
output_positions[hand] = reconstructed_palm
output_orientations[hand] = palm_quat
```

`pico_palm_skeleton_filter_node.py:109` 的 IK-frame 适配仅修改 shoulder/elbow 的姿态，不恢复 hand 位置。
随后 bridge 发送修正骨架；TJVR 上肢字段只有这些点及旋转。

因此不能把相同 hand 数值分别标成“重建掌心”和“解剖学 hand”，以形式上的双字段冒充语义分离。
也不能假定一个未经证明的固定偏移能找回已经覆盖的原始点。

## 机器人几何：已知事实与未完成项

- `control/src/spark_guidance.cpp:17` 从 MuJoCo 模型读取左右 `tcpRelativeToLink7`，并用于构造 Pinocchio 运动学；不能拿 Pinocchio 的默认 TCP 常量代替实际接线。
- `control/models/marvin_m6_wuji2.xml` 的 `tcp_L/tcp_R` 位于各自 Link7 的 `[0,-0.1315,0]`，不同于 flange 的 `[0,-0.095,0]`。
- XML 中左右 Link1 body 的位置分别为 `[0,0.2115,1.121]`、`[0,-0.2115,1.121]`。这些是静态 XML 值，**不是已完成的跨模型 FK 验证结果**。
- XML SHA256：`dcb3040c7cb5b5d1d897c56f9e5834df0c682d179919baa6a1fef60ce782b4ae`。
- 基线 `control/config/qp_ik_pico_teleop.yaml` SHA256：`aaa0f71eb3fdbf5f832a340b3328a5916b65555095551af694f7071d849afead`。

因输入契约已触发停止条件，未执行 Home/零位/两组非零姿态的 MuJoCo/Pinocchio 对照，未冻结 `R_BC`、几何 artifact 或完整实验配置。不能把上面的 XML 摘录当作阶段 1 通过。

## 解除阻塞所需的设计决定（尚未实施）

保持线协议和驱动不变时，需要先批准对方案输入契约的修订：

1. 明确使用 bridge 已映射骨架，接受其躯干参考系语义；重新定义该输入下的不变性要求，不能承诺还原原始世界竖直。
2. 明确允许重建掌心作为 shape 代理点，而非声称存在独立解剖学 hand；同步调整形态长度、质量门控和接口语义。

若必须保留现方案的原世界系与独立解剖学 hand 要求，则需要额外输入/上游信息；这超出本轮不改线协议及既有接线的边界。

## 本轮验证与范围

- 已执行：HEAD/工作区检查、源代码逐段追溯、模型与配置 SHA256 记录。
- 未执行：构建、算法/回归测试、跨模型 FK、trace 对照、设备仿真和真机测试；无通过声明。
- 本轮仅新增本审计文档，运行代码、默认行为及 `pico2_hands/**`、`pico2_sim.sh` 均未修改。
- 未提交、merge 或 push；未进入 Phase B。

## V2.1-R2 / 阶段 1R 续审（保留上文历史结论）

用户随后批准使用 bridge frame 与 reconstructed-palm proxy，并明确要求保留
M0 对称骨架。旧的两项输入假设阻塞已通过契约修订解除；“未经人为缩放”不再被
解释为禁止 `symmetric_max`，而是禁止未声明的额外缩放。最新依据为下载目录的
V2.1(5)，本轮同步修订了其阶段 1R 第 6 条。

新增工作区交付物：

- `control/config/shared_root_tjvr_input_contract.yaml`：源码指纹、bridge 根偏移、
  轴、左右索引、M0 有效骨长、固定 skeleton 掌姿态来源与一次 basis。
- `control/config/shared_root_robot_geometry.yaml`：实际模型指纹、零位参考几何、
  shoulder/wrist/TCP frame；Link5 局部向量明确仅为 reference shape。
- `control/config/qp_ik_pico_shared_root.yaml`：完整旧 SPARK 配置加默认关闭的实验组；
  候选滤波/门控值尚非现场验证调参。
- `control/scripts/validate_shared_root_contract.py`：只读离线哈希/旋转/配置校验。
- `control/docs/shared_root_input_contract.md`：来源及范围说明。

实际执行的验证：

```bash
pixi run cmake -S control -B control/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$PWD/.pixi/envs/default/bin/python"
pixi run cmake --build control/build --parallel 2 --target \
  test_pinocchio_arm_kinematics test_spark_upper_retarget test_spark_guidance test_config
pixi run ctest --test-dir control/build --output-on-failure \
  -R '^(test_pinocchio_arm_kinematics|test_spark_upper_retarget|test_spark_guidance|test_config)$'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest pico2_hands/tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest control/tests/test_shared_root_contract.py -q
.pixi/envs/default/bin/python control/scripts/validate_shared_root_contract.py control/config/qp_ik_pico_shared_root.yaml
```

结果：基线 4 个 C++ 测试程序通过；PICO2 Python 测试 58 项通过；契约测试 6 项通过。
新增 `test_shared_root_geometry` 验证跨模型 shoulder/wrist/TCP 及固定肩中心，通过。
Python 环境没有 pinocchio 模块，使用现有 C++ Pinocchio 链路完成对照，没有安装或替换依赖。
上述 Python 选择用于绕过本机缺失的 `.venv/bin/python`，未修改原 Pixi task。

阶段 2 adapter 先增加测试，确认缺失实现时构建失败，再新增纯 C++ 实现。
其测试覆盖只扣一次偏移、左右 basis、点差不变、不按当前肩点重定位、双侧无效拒绝、
metadata 透传及不使用报文 side pose fallback。尚未完成阶段 3～6；不得作为现场入口使用。

本轮没有设备输入、Viewer、真机、线程/录制改造或 Git 提交操作。

### 阶段 2 与阶段 3 的后续进度

- 阶段 2：`shared_root_input.hpp/.cpp` 已实现，3 项 adapter 用例通过。
- 阶段 3：新增 `shared_root_morphology.hpp/.cpp`，固定最大 128 样本窗口、
  median/MAD、provisional/confident、单帧离群拒绝、持续异常失效、身份与唯一帧检查。
  3 项初始用例通过，覆盖错误首帧纠正、重复帧、epoch、持续异常及有效骨架尺寸缩放。
- `MorphologyState::kInvalid` 不自行 reacquire；后续连续性管理应显式重建候选窗口。
  当前没有 shadow-window 或 Phase B 实现。
- bridge-frame 滤波、target builder、intent、连续性管理、guidance 和同 trace A/B
  尚未完成；这些纯算法模块的通过不能替代现场接线与阶段 6 验收。
- 临时配置门禁止 `enabled=true`，避免未接线时静默使用 legacy。默认配置文件未修改。

契约、配置、模型测试只验证本仓库离线一致性；filter/gate 的候选参数尚未接受
真实 trace 量化验收。运行时全量配置接线及 artifact 加载仍需在后续阶段完成。

### 后续推进：滤波 / target builder / intent / continuity 核心

新增 `shared_root_target_builder.hpp/.cpp`：

- raw 与 filtered 分层；中心、关系、尺度指数滤波，姿态 slerp，单位骨段方向插值；
- 独立 `UpperSparkSkeletonScaler` 历史，共同 R_BCt 用于姿态和骨段，形态方向不做各向异性缩放；
- 主 palm 与 hand proxy 分开，shared-root blend 不调用 legacy 的 palm=hand 复制；
- intent 首帧/升级帧零导数且证据无效，普通尺度更新不改变锁定 intent scale；
- 候选状态以有界值拷贝计算，仅双侧所有 gate 通过后提交，失败不污染 filter/scaler 历史；
- 短中断可重建滤波/导数但保留 intent scale；身份变化清除锁定；
- 9 项测试覆盖 raw 关系、proxy、滤波、重复/无效帧、尺度升级、0.8/1/1.2 尺寸、
  rotation jump 的同参考系 position fallback、epoch 和中断恢复。

新增 `shared_root_continuity.hpp/.cpp`：

- 五状态、唯一帧/源时间/接收时间门控、短 hold / 长 invalid、同步 blend；
- hold 保留原目标时间，不刷新 freshness；授权撤销清空候选；
- alpha=1 后仍等待匹配 sequence/epoch/generation 的外层接受确认；
- 6 项测试覆盖重复帧、最终确认、授权与 epoch、显式 discontinuity、
  丢帧期间移手后静止，以及 hold 不续命。

先添加测试并观察缺失实现导致构建失败，再实现；两组测试均通过。
复建后的相关 CTest 共 9 个程序通过（其中包含既有 config、SPARK 与跨模型测试）。

**当前仍未完成阶段 6，不是验收交付。** 尚需完整配置/模型哈希的运行时加载、
guidance 的 reset/intent/静止保持接线、外层控制参考接受确认、完整故障序列及
同 trace A/B、权重与尺度消融、性能量化。未启动任何设备或 Viewer。
当前 `enabled=true` 继续 fail fast，禁止将纯模块测试通过解释为现场路线已经可用。

### 2026-09-17 后续推进：原生 artifact 校验与候选链组合

新增 `shared_root_options.hpp/.cpp`、`shared_root_sha256.cpp`，用于启动时离线加载：

- 原生 SHA-256 流式校验，不启动 Python 或外部进程；
- 固定 input/geometry artifact 版本、交叉引用、协议/M0/bridge 源文件哈希和模型哈希；
- 配置必填、未知键、重复键、非有限值、正范围及固定容量检查；
- XML 与 URDF 的独立 FK 对照，XML 固定肩根、段长、参考姿态的 wrist→shape proxy 检查；
- 六项选项测试覆盖已冻结证据、SHA 已知向量、异地 profile 显式路径、非法配置、
  artifact 篡改和重复键。

本轮测试发现并修复加载器自身的几何口径错误：artifact 定义的是 XML 模型几何，
最初实现却将 URDF 的舍入后肩位置作为固定机器人根，导致 `robot root mismatch`。
现采用 XML 几何构造 artifact 对应的固定映射，URDF 只按已冻结跨模型容差独立对照。
没有放宽容差、修改模型或覆盖骨长配置。

新增 `shared_root_pipeline.hpp/.cpp`，组合现有 adapter → morphology → builder → continuity：

- 同一控制线程持有全部新状态，无内部线程、时钟、设备 I/O 或执行授权；
- 唯一源帧检查位于滤波/尺度更新之前；陈旧包不推进尺度，重复包不累计恢复帧；
- 长 gap 或无效映射重建 builder 历史，避免旧 source dt 造成永久拒绝；
- 持续尺度异常后从空窗口重新收集，达到 confident 才成为恢复候选；没有 shadow window；
- 目标 gate 失败不提交候选尺度更新；短异常沿用 intent scale、重建导数；
- 撤销授权清空候选/滤波/尺度，同时保留唯一源帧边界，旧包不能直接重启；
- epoch/generation 变化重建上下文，正常接收会话重建通过显式 resetSession；
- 最终恢复仍依赖外层接受确认，pipeline 不授予运动权限。

七项组合测试覆盖：完整链丢帧后移手并静止恢复、撤销授权、持续尺度变化重建、
重复/旧包、epoch、没有中间控制 tick 的长 gap，以及目标 gate 失败的事务边界。

本轮实际验证命令：

```bash
pixi run cmake --build control/build --parallel 2 --target \
  test_shared_root_options test_shared_root_pipeline \
  test_shared_root_target_builder test_shared_root_continuity \
  test_shared_root_input test_shared_root_morphology test_shared_root_geometry \
  test_config test_spark_guidance test_spark_upper_retarget test_pinocchio_arm_kinematics
pixi run ctest --test-dir control/build --output-on-failure \
  -R '^(test_shared_root_.*|test_config|test_spark_guidance|test_spark_upper_retarget|test_pinocchio_arm_kinematics)$'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_contract.py pico2_hands/tests -q
.pixi/envs/default/bin/python control/scripts/validate_shared_root_contract.py \
  control/config/qp_ik_pico_shared_root.yaml
git diff --check
git diff --name-only -- pico2_hands pico2_sim.sh tracking real_robot
```

结果：CTest **11/11** 测试程序通过，Python **64/64** 通过，artifact 校验通过；
diff 空白检查通过，所列隔离路径零差异。没有启动 Viewer、设备输入或真机；
没有提交或 push。测试临时文件由测试在独占临时目录中创建并清理。

**本节不是阶段 6 完成报告。** 原生 loader/pipeline 目前只在离线测试中调用，
尚未接入 `DualArmSparkGuidance` 的实际 step、控制历史 reset、静止保持门控，
也未连接外层控制参考接受回执。完整 profile 中原有 SPARK 参数尚需在 guidance
构造 pipeline 时显式传入，不能用独立模块默认值冒充已完成运行配置接线。
同 trace A/B、尺度/权重消融、实际 IK/velocity-QP 联合恢复及分位耗时验证仍未完成。
`enabled=true` 的临时禁止门继续保留；旧分支不调用新模块。

### guidance 后续进度入口

本节之前的“guidance 尚未接线”是当时的进度快照。
2026-09-17 后续已增加 guidance 库接口与离线 velocity-QP 接受确认测试，
但生产应用入口仍未启用，阶段 6 尚未完成。
具体实现、16 个 CTest/64 项 Python 结果、合成样本限制和剩余项见
[guidance 离线接线报告](shared_root_phase_a_guidance.md)。
