# 简化 PICO 标定：候选阶段历史记录

> 历史归档，不代表当前实现状态。当前操作见[简化标定](../../../../docs/pico-simple-calibration.md)。

## 候选阶段历史记录（不代表当前现场入口状态）

目标流程：输入身高 → 左侧 TCP 位置/姿态 → 左侧腕心 → 推导右侧 TCP 与双侧骨长 →
双侧检查 → 发布。旧双侧实测流程、人员默认指针和 M0 加载逻辑保持不变。

## 当前已实现与未实现

`scripts/pico_simple_calibration.py` 是纯离线候选生成器，不是标定采集或现场启动入口。
它不连接 ROS/ADB、不启动设备、不修改人员 profile，也不能直接交给 M0 加载。

- 身高以米输入，候选范围 1.0～2.4 m；单位错误、NaN、无穷与布尔值被拒绝。
- 模板取自已核对的 `noitom_male170_opensim_spine_v1`：上臂系数 0.155882、前臂系数
  0.152941，左右采用相同数值。系数固定在本项目，无 davinci 绝对路径运行依赖。
- 左侧 TCP、腕心先经过原来的实测 artifact 校验，不放宽旧门限。
- 腕掌距离取现有标量，或旧向量的范数；双侧均沿掌局部 +X 使用，不复制旧向量方向。
- 左侧 TCP 外参为 `(R_L,t_L)`，右侧候选为
  `t_R=M_controller*t_L`、`R_R=M_controller*R_L*M_palm`。
  两个矩阵需显式提供，或显式选择 `symmetric_local_y` 模型；不静默默认使用局部 Y 轴。
- 结果采用新类型 `pico_simple_candidate_v1`，`candidate_status=review_required`，
  `runtime_eligible=false`，保留源文件 SHA-256、模板比例和镜像依据文字。
  不复制右侧实测 quality、采样数或 geometry gate_results。
- 输出文件独占创建，已有文件（包括原始输入）不可覆盖。

**尚未完成：**现场单侧采集编排、新配置的 runtime 校验/加载、双侧检查和人员配置发布。
当前阶段不能宣称简化现场标定已可用。

## 阻塞证据与下一步

`tracking/src/pico_bridge/src/pico_bridge_node.cpp::publish_pose` 将 APK 报文中的位置及四元数
直接写入 `/pico/pose/left_hand` 与 `/pico/pose/right_hand`；没有声明左右手柄局部轴镜像契约。
TCP runtime 合成的是 `T_G_controller * T_controller_palm`。人体矢状面已知并不等于
`M_controller` 在各局部坐标系中已知。

人员文件里现有的人工 Y 反号平移只能说明一次参数调整，不能证明旋转镜像规则。
测试里的对角镜像矩阵是合成数学场景，不是 PICO 硬件轴标定结果。

用户已明确新模型不拟合 zj 的既有右侧姿态。新增 `symmetric_local_y` 显式模型约定：
两局部系均将 Y 视为左右轴，`M_controller=M_palm=diag(1,-1,1)`。它不是从 zj 的姿态
反推出来的硬件事实，也不加固定旋转补偿来强行复现 zj。仅生成候选，不自动放行现场。

继续接入现场前，需要工程级验证：APK/SDK 的左右手柄局部轴定义，或双手镜像握持时
的成对姿态观测核验。它用于固定设备型号的镜像契约，不要求每位用户重复标定右手。
核实前不生成默认硬件镜像配置，不允许通过补写“verified”字段绕过验证。

旧 wrist v1 文件可能没有绑定 TCP translation fingerprint；候选明确记录这一限制，
不把“两个文件单独通过旧校验”宣称为它们来自同一次有效标定链。
新现场流程须捕获标定顺序及上游指纹，TCP 变化时使下游腕心候选失效。

## 离线用法

选择统一对称模型（身高 1.70 仅为示例，不是 zj 的已知身高）：

```bash
.pixi/envs/default/bin/python scripts/pico_simple_calibration.py \
  --left-tcp /absolute/path/pico_left_palm_tcp.yaml \
  --left-wrist /absolute/path/pico_left_wrist_pivot.yaml \
  --height-m 1.70 \
  --mirror-model symmetric_local_y \
  --output /absolute/path/new-candidate.json
```

也可显式提供其他已核对的局部轴契约，与 `--mirror-model` 互斥：

```bash
.pixi/envs/default/bin/python scripts/pico_simple_calibration.py \
  --left-tcp /absolute/path/pico_left_palm_tcp.yaml \
  --left-wrist /absolute/path/pico_left_wrist_pivot.yaml \
  --height-m 1.70 \
  --mirror-contract /absolute/path/mirror-contract.json \
  --output /absolute/path/new-candidate.json
```

镜像契约 JSON 必须且仅包含 `schema_version`（整数 1）、`controller_mirror`（3×3）、
`palm_mirror`（3×3）与非空 `evidence`。两个选项都未选或同时选择时拒绝。
模型约定和文件依据均不自动获得运行资格；输出仍是待审查候选。

## 本轮离线验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  tests/test_pico_simple_calibration.py tests/test_symmetric_profile.py \
  tests/test_teleop_profile.py pico2_hands/tests control/tests/test_shared_root_contract.py -q
```

124 项通过。包含非平凡旋转镜像、镜像两次恢复、世界位姿合成一致性、不同 controller/palm
镜像平面、非法反射/单位拒绝、源文件保留和输出不可覆盖。
旧双侧标定、runtime、驱动及裸手代码没有改动；没有修改任何已存在的人员标定文件。

另外执行 `tracking/src/pico_bridge/test/` 中的 `test_pico_calibration_artifact.py`、
`test_pico_palm_tcp_runtime.py`、`test_pico_palm_orientation_core.py`、
`test_pico_arm_geometry_core.py`：45 项通过。
这是软件离线回归，不是镜像坐标契约或现场标定验收。

新增显式模型选择后再次运行上述组合回归：126 项通过（5.63 秒），`git diff --check` 通过。
新增测试使用临时目录，仅包含
左侧两个文件，验证右侧生成不依赖任何右侧配置、三维 TCP 长度保持、旋转正确镜像，
且未选模型/同时指定两个来源时拒绝。
