# 当前遥操作与采集结构

## 1. 唯一双臂算法

本分支只保留 **共享根掌心映射 → Franka DLS IK → Ruckig**。算法标识为
`pico_ee_franka_dls`，profile 为 `qp_ik_pico_shared_root_dls.yaml`。
其他双臂后端、专属入口、构建依赖、配置、测试和历史对照说明已删除；历史由 Git 保存。

共享根配置使用 `shared_root`，形态参数使用 `shared_root_shape`，没有旧键兼容回退。
DLS 共用的运动学和 Ruckig 限幅已使用中性／DLS 命名。当前冻结 XML/URDF 文件名中的
`ceres` 只表示资产来源，不包含或启用其他求解器；这些资产仍被 DLS 使用，数值不能随意改动。

## 2. 双臂通信

```text
PICO APK → ADB/TCP :9999 → pico_bridge
  → ROS 原始位姿／骨架 → 人员标定与掌心／骨架校正
  → pico_arm_input（原配对和几何计算）
  → /pico/arm_input : PicoArmInput
  → tianji_arm_ros（原 DLS/Ruckig 控制循环）
  → /tianji/controller/joint_targets : ControllerJointTargets
  → Python 执行器 → 原安全门控 → 设备 SDK
```

生产路径不再经过业务 UDP 15000/17000，也不额外增加 ROS↔UDP 转发进程。
DDS 自身传输由 Fast DDS 管理；这不等于禁止 DDS 内部使用 UDP。
旧字节协议只用于仍有实际用途的 DLS 输入审计／录制工具，不是生产回退。

`PicoArmInput` 将匹配的左右目标、臂方向、八点上肢几何／旋转和状态原子发送。
源 PICO 时间与本机发布单调时间分别保留，不能把两个时钟混用。
boot/session/sequence/epoch 和持续的撤销代际让重置、重连和无效输入不能被 latest-only 覆盖。

`ControllerJointTargets` 是提交后的双臂 14 维参考，不是原始 IK 目标或实测反馈。
它携带产生时间、实际已应用输入时间及 ready 标志；保留原执行分支需要的手部兼容字段。
新 Manus ROS 目标尚未接入执行器，不能把这些兼容字段当成已经完成 Manus 接入。

高频目标用 BEST_EFFORT／KEEP_LAST1／VOLATILE。ROS 接收／发布不在控制循环执行，
不积压旧目标；未完成 DDS 节点发现时丢弃未认证样本，只有唯一、已确认的发布者才能更新缓存。
源失鲜、身份／epoch 改变、数值非法和 ready 撤销均不能伪装成有效目标。

## 3. 进程与环境

- `default`：Jazzy、Python 3.12、Fast DDS；输入适配、执行器、采集器及 Manus SDK 重定向。
- `control`：无 ROS 的 DLS/Ruckig 原生工具链和 stdin worker。
- `arm-ros`：与 control 一致的数值 ABI，加 ROS C++ 边界；不借用 default 的 Eigen/Pinocchio。
- `cameras`：官方 RealSense 驱动，和其他进程只通过 DDS 通信。
- `policy`：独立 overlay 的推理／回放环境；GPU、权重和真实 Motive 需现场准备。
- `spd`：裸手 SPD 仿真专用环境，保持独立构建和输出边界。

默认 domain 120、LOCALHOST、同机 CLOCK_MONOTONIC；验证使用隔离域 121。
构建由 `pixi run build` 管理，单独双臂 ROS 构建可用 `pixi run build-arm-ros`。
消息在每个相应 ABI 环境编译，不注入其他环境的 site-packages。

## 4. 输入与模型职责

- PICO 输入负责人员标定、TCP／腕点、骨长和坐标转换，不负责机器人 IK。
- Manus 为独立 ROS 链：原始手套骨架 → 21 点 → SDK Hand2 RetargetSession → 每侧 20 维目标。
  当前只发布 `/wuji/{left,right}_hand/joint_commands`，不连接或使能 Wuji。
- 外骨骼输入保持现状，不属于本轮迁移范围。
- 裸手和 Mocap 的 DLS worker 复用同一保留核心；通用物理显示／回放引擎不是另一个 IK 后端。
- 模型、网格和 Home 放在 `tianji_description`，通过安装资源读取。标定放在实际人员 profiles 中。

配置只有一份来源：`config/robot.json` 定义设备、安全边界、模型和 ROS 话题；
`config/collect_real.json` 定义相机角色、序列号与采样率。源文件路径和指纹必须一致，
不能复制参数到多个启动脚本绕过校验。

## 5. 执行与录制权限

执行器是唯一硬件写入者。ROS Topic 或 Service 均不授予使能权限。
首次本地授权、实测反馈、限位、失鲜、受限接管和故障停止全部保留。
执行器与 Home 通过本机互斥租约排他；换 ROS domain/topic 不能绕过。
普通仿真只显示参考，不导出硬件目标。

采集器只订阅真实反馈和 RGB，不打开设备／相机，不用参考目标冒充状态。
连续数据分别由 `/tianji/feedback/{arms,left_hand,right_hand}`、`/tianji/executor/state`、
`/cameras/<role>/color/image_raw` 和 `/tianji/collection/status` 传递。

| 按键 | 服务 | 执行边界 |
|---|---|---|
| r | `/start_collect` | 实际录制开始且当前状态匹配本条之后才放行跟随 |
| s | `/stop_collect`，save=true | 保存本条，停止跟随完成后回双臂 Home |
| d | `/stop_collect`，save=false | 丢弃本条，停止跟随完成后回双臂 Home |

请求绑定 UUID、执行会话和条目 ID；同进程重复请求复用原结果，改参复用 ID 拒绝。
服务 success 是实际完成结果，不是入队确认。未知结果不自动重放，缓存不跨采集器重启。
内部 abort 保留 partial，不等同操作者 d。Home 不录入任务条目；录制服务不直接控制 Home。

相机独立启动一次；采集、预览和视频桥订阅同一图像。身份、实际 profile、发布者和新鲜度
持续认证，异常不能用旧帧重复发布掩盖。显示／录制成功不能代替现场机器人动作验收。

## 6. 验证与维护

先做纯配置／接口检查，再做隔离 ROS 合成输入与真实原生核心验证；真机仍需独立现场授权。
数据集 schema-v1 的图像及实测 14+40 维关节约定不因算法清理而改变。
当前验证证据与未验证范围见 [migration-verification-status.md](migration-verification-status.md)。
