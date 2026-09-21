# Marvin M6-S-CCS-696 V4.0 模型资产

来源为用户提供的本地归档：

`Marvin M6-S-CCS-696-V4.0_Base_and_Stand_Asm urdf.rar`

- 导入日期：2026-07-28
- 原始归档 SHA-256：
  `08656857ca6025e103c4d15e55a2b5d147f4127c4e59f800408cfd3113598b50`
- 内容：完整底座、支架、左右 7 自由度机械臂、左右 TCP，共 20 个
  STL 网格和一个 URDF。

网格文件保持原样。URDF 仅做了两项部署规范化：

1. 文件名改为 `marvin_m6_s_ccs_696_v4.urdf`；
2. 网格 URI 改为本 ROS 2 包内的
   `package://pico_body_tianji/assets/marvin_m6_ccs/meshes/`。

资产的授权和分发范围以原始提供方约定为准。

## MuJoCo QP-IK 使用说明

本目录是原始模型资产源，不由控制算法修改。C++ QP-IK 工程使用：

- `models/marvin_m6_s_ccs_696_v4_local.urdf`：仅将 ROS `package://` URI 改为项目内相对路径；
- `models/marvin_m6_qp_test.xml`：MuJoCo 编译后的运行模型，并增加 `tcp_L`、`tcp_R` 与两个可交互目标。

模型转换不改变 14 个旋转关节的 origin、axis、limit 或左右 TCP 固定变换。

运行模型还把原 URDF 的关节速度上限写入 MuJoCo 的 `joint user[0]`，供 QP 硬速度边界读取；`target_L` 和 `target_R` 是仅用于仿真交互的 mocap 目标，不属于原机械结构。可在项目根目录执行 `pixi run inspect` 验证 14 个关节地址、范围与两个 TCP 映射。

当前 V1 不修改或声称辨识原始资产的质量、惯量、摩擦或执行器参数。模型中的动力学数据不能直接作为真实机械臂 PD 调参依据。
