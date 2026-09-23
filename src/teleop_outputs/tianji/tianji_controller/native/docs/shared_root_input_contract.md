# Shared-root TJVR 输入契约

本契约规定共享根掌心映射的输入来源、坐标系和几何证据，不修改发布端或线协议。
配置证据见 [输入 artifact](../config/shared_root_tjvr_input_contract.yaml) 和
[机器人 artifact](../config/shared_root_robot_geometry_dls.yaml)。

## 来源与单一所有权

| 输出 | 来源 / 处理 |
|---|---|
| 左肩肘腕掌 | `PicoTeleopFrame.upper_limb_skeleton` 点索引 0、1、2、3 |
| 右肩肘腕掌 | 同字段索引 4、5、6、7 |
| `*_root_Ct` | adapter 从以上绝对点扣除一次 `[0,0,1.121] m`；后续模块不再扣除 |
| control / shape proxy | 同一 M0 重建掌心样本，明确是代理，不是两份观测 |
| 掌旋转 | skeleton hand rotation 后乘一次左右 EE basis；不读取 `frame.left/right` |
| 肩肘腕旋转 | skeleton 对应旋转，已在 bridge frame 中；不乘掌 basis |
| 序列/源时间/接收时间/epoch/generation/discontinuity | 同名 `PicoTeleopFrame` 字段原样透传 |

`bridge_robot_midpoint_frame` 是本地契约名称，不是 TJVR 携带的 frame ID。
bridge 利用双肩及 spine2 构造轴，再平移到高度 1.121 m；此契约不声称其 Z 轴
就是原世界竖直。控制端不重建世界 frame，不重新取当前肩中点作为平移。

`correct_side()` 同时生成重建掌心位置及 `palm_quat`；IK-frame 适配只调整
肩/肘姿态。bridge 将位置与旋转作相同刚体变换，其 skeleton rotation 不应用 EE basis。
报文的 `left/right` 是另一条目标构造支路，可能包含 reach scale / X offset，不能
当成 skeleton 掌姿态的备用字段。

bridge 的状态配对要求同源时间戳、合法 epoch、`stream_valid=true` 和 `ik_frame_valid=true`。
左右侧 `corrected` 标志不再作为拒绝条件；raw/hold 回退若仍满足其余检查，可以通过此门。
因此不能把 bridge 的有效输出解释为双侧校正均成功。这也不是对任意外部发布者的身份认证；
artifact 哈希不能证明现场正在运行的远端程序版本。

## 对称骨架

依据用户明确要求，保留既有 `symmetric_max`。M0 可将左右已标定的上臂、前臂
长度分别替换为共同最大值，bridge 对 skeleton 仅做刚体变换，不再缩放骨段。
估计器读取这些**米制有效骨长**，不恢复原始不对称观测，不重复对称化。
这不是宣称修正后的长度是未经处理的真实人体测量。

## 机器人几何

共同 B 使用现有 MuJoCo world / Pinocchio universe 对齐后的模型坐标。
固定根方向来自模型基座轴和肩中心关系，不来自随 J1 转动的 Link1 局部姿态。
零位、配置初始姿态和两组非零关节姿态由 `test_shared_root_geometry` 对照。

控制器将 MuJoCo 的 Link7→tcp 固定变换传给 Pinocchio；求解 TCP 是 `tcp_L/R`，
不是 flange TCP。冻结共享根模型的腕中心→TCP 距离为 0.1615 m。
Link5 局部向量随腕关节变化；
artifact 中局部向量只是 reference q 的 shape 参考，不得当作固定安装外参。

## 当前接线状态

唯一控制链为共享根掌心映射 → Franka DLS → Ruckig。
配置使用 `shared_root` 和 `shared_root_shape`，
唯一 profile 为 `config/qp_ik_pico_shared_root_dls.yaml`。
纯 C++ adapter、尺度估计、目标构造/滤波、连续性管理和 guidance
为 DLS 提供目标；不提供会话内算法选择。

生产入口 `tianji_arm_ros` 使用 ROS 输入/输出。关节目标导出要求执行器显式
启用 `--franka-dls-executor`；硬件授权与 SDK 属于 Python 执行器。
离线 validator 仅验证工作区 artifact 与配置一致性，不授予运动权限。
入口和传输边界见 [原生控制器说明](../README.md)；
冻结模型来源见 [几何证据](verification/shared_root_dls_geometry.md)。

连续性模块的 `accept(sequence, epoch, generation)` 只能由最终控制参考接受路径调用，
不能仅因为生成目标、IK 成功或 guidance 返回 `accepted` 就调用。
guidance 消费恢复期间普通 stationary hold 的抑制和历史重置信号；
安全保持、stale、Home、失能和故障仍然优先。

## 简化对称配置

除既有 measured/symmetric_max 来源外，输入契约也允许显式声明的
`left_measured_symmetric_local_y_height_template`：左侧实测 TCP，右侧采用声明的镜像模型，
双侧臂长由身高模板生成。简化 bundle v2 的腕掌距离也由身高估计，v1 才使用左腕实测；
它是有效模型几何，不是双侧独立实测。
控制端仍只消费 M0/bridge 输出，不再镜像 TCP、不再生成一次骨长，也不读取个人文件热更新。

启动时必须验证参与者的 `pico-simple/active.json` 指针。
历史录制与文件存在不等于当前配置已发布；原生合成输入不能证明
右侧物理轴、SDK 输出或换人后的现场跟踪效果。
