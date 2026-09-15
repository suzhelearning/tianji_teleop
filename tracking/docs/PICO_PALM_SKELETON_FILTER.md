# PICO 掌心约束骨架快速验证

这个实验节点把已有 TCP 标定后的掌心位姿作为左右手末端，参考 PICO
24 点骨架重建肩、肘、腕和手。它不接外骨骼、Odin、ESEKF、QP，也不用于
active control。

## 坐标约定

当前 /pico/palm_left、/pico/palm_right 的 frame_id 是 pico。
/pico/smpl_raw 也是 pico，因此快速节点使用这两个原始输入。不要直接把
/pico/smpl（pico_ground，已经做过地面 Z 对齐）和当前掌心 topic 混合。

输出 /pico/smpl_palm_corrected 仍是 pico frame。它保留躯干、头部和下肢，
只改变左右肩/肘/腕/手索引 16,18,20,22 和 17,19,21,23。其中索引 22/23
在该实验 topic 中明确表示 TCP 掌心。

节点还发布 `/pico/smpl_palm_corrected_ik`。该 topic 是
`/pico/smpl_palm_corrected` 的同帧深拷贝，24 个关节位置完全不变，只调整肩、肘
局部坐标轴以匹配宇数 IK 约定：左肩局部 X 右乘 `+pi/2`，右肩局部 X 右乘
`-pi/2`；肘部局部 X 沿肘到腕，局部 Y 为屈伸轴并统一选择人体左侧符号，局部 Z
补成右手坐标系。符号不明确时沿用上一有效帧，tracking epoch 变化时清空该连续性
参考。若某帧几何退化，节点只跳过该帧 IK topic，并继续发布原 corrected topic。

## 启动

先启动现有 PICO bridge，确认原始骨架和左右手柄持续发布：

~~~
ros2 topic hz /pico/smpl_raw
ros2 topic hz /pico/pose/left_hand
ros2 topic hz /pico/pose/right_hand
~~~

然后用一条 launch 同时启动左右只读 TCP 掌心发布器和骨架过滤节点：

~~~
ros2 launch pico_bridge start_pico_palm_skeleton_filter.launch.py \
  left_tcp_artifact:=$HOME/.config/pico_tracker/pico_left_palm_tcp.yaml \
  right_tcp_artifact:=$HOME/.config/pico_tracker/pico_right_palm_tcp.yaml \
  left_wrist_pivot_artifact:=$HOME/.config/pico_tracker/pico_left_wrist_pivot.yaml \
  right_wrist_pivot_artifact:=$HOME/.config/pico_tracker/pico_right_wrist_pivot.yaml \
  require_wrist_pivot_artifact:=true
~~~

`pico_palm_tcp_publisher` 只读取现有 artifact，并分别把
`/pico/pose/left_hand`、`/pico/pose/right_hand` 转换成
`/pico/palm_left`、`/pico/palm_right`。它不会修改标定文件，也不需要按空格。
`pico_palm_tcp_calibrator` 只在需要重新标定 TCP 时单独运行，不再是日常运行链路的一部分。

若确实要手动运行两个 calibrator/publisher，必须关闭 launch 内置发布器以避免同一 topic
出现重复发布者：

~~~
ros2 launch pico_bridge start_pico_palm_skeleton_filter.launch.py \
  start_palm_publishers:=false
~~~

节点会自动收集每侧 60 个时间差不超过 30 ms 的配对样本，估计上臂和前臂
长度。若存在 `~/.config/pico_tracker/pico_left_wrist_pivot.yaml` 或
`pico_right_wrist_pivot.yaml`，节点还会加载已经独立标定的
`wrist_to_palm_m`。兼容旧 artifact 时仅取该三维向量的模长 `d_WH`，不再使用
其旧 XYZ 方向；运行时方向统一固定为掌心局部 `+X`：

~~~text
p_palm_requested  = p_wrist_requested + R_palm @ [d_WH, 0, 0]
p_wrist_requested = p_palm_requested  - R_palm @ [d_WH, 0, 0]
R_wrist = R_palm
~~~

因此 TCP 掌心与重建腕部的姿态严格一致，腕部请求位置始终从掌心沿局部 `-X`
得到。若该腕点超出当前估计的固定骨长可达范围，节点将腕点投影到最近可达边界，
再用原始 PICO 肘点选择分支并求三角形解析解。此时仍发布最后的可达骨架，
`reach_clamped=true`，修正骨架掌心与独立 TCP 掌心之间的距离写入
`wrist_pivot_position_residual_m`。没有 pivot artifact 时，
节点会退回到配对骨架估计的位置偏移，但仍保持 `R_wrist = R_palm`。需要强制两侧
都有 artifact 时，可传入 `require_wrist_pivot_artifact:=true`。基线只保存在本次进程内，
重启后重新建立。

## 左右臂个体骨长快速标定

默认的60帧基线仍只代表PICO SMPL比例。需要快速验证个体骨长时，运行：

~~~bash
./src/pico_bridge/scripts/pico_arm_geometry_runtime.sh \
  --side left --domain 42 --start-mode space \
  --tcp-artifact "$HOME/.config/pico_tracker/pico_left_palm_tcp.yaml" \
  --wrist-pivot-artifact "$HOME/.config/pico_tracker/pico_left_wrist_pivot.yaml"
~~~

右臂使用相同命令但传 `--side right` 和右侧 artifact。左右仅共用算法实现，
测量结果必须独立采集，禁止镜像。

该流程只采集左侧。自然下垂、静态伸直前举和上臂下垂贴身且肘约 90 度三种
姿态共同消去未知肩点：`前举腕点-屈肘腕点` 辨识上臂，
`屈肘腕点-下垂腕点` 辨识前臂。三次前举自动选择最一致的两次并记录离群阶段。
每个动作应在倒计时结束前摆好，求解自动选择最长静止后缀。PICO 原始肩/肘/腕
只验证动作角度，不作为骨长真值。`capture.npz` 保存原始掌心位置/姿态、原始腕点及由 TCP 掌心沿局部 `-X`
回算的腕点。右手套和右掌 topic 不参与。

将通过 Gate 的左右 candidate 以绝对路径传给统一运行器：

~~~bash
./src/pico_bridge/scripts/start_pico_m0_runtime.sh \
  --domain 42 \
  --left-geometry "$LEFT_GEOMETRY" \
  --right-geometry "$RIGHT_GEOMETRY" \
  --require-left-geometry --require-right-geometry \
  --viewer --record --duration 120
~~~

加载时只接受对应 side 的 `pico_<side>_arm_geometry_quick_v3`，并严格核对
TCP revision、TCP artifact SHA-256 和腕部 artifact SHA-256。任何不匹配都会
拒绝启动，不会静默改用另一套长度。

录制完成后运行：

~~~bash
ros2 run pico_bridge pico_m0_comparison_report report \
  /绝对路径/pico_m0_capture.npz \
  --output /绝对路径/pico_m0_gate.json
~~~

报告会 fail-closed 检查流长度、时间戳、tracking epoch、每侧 geometry
source/revision 稳定性、修正/回退覆盖、骨长和掌心残差、reach clamp 与肘分支跳变。

PICO-only Gate 是外骨骼挂接的强制前置条件。正式 bilateral M0 要求左右两侧都由
各自独立采集的 `pico_<side>_arm_geometry_quick_v3` 提供长度，不能镜像、复制或
复用另一侧的测量结果。左侧 quick geometry 加右侧 `raw_smpl_baseline:0` 只允许
左臂 shadow 开发检查，必须在报告中保留该来源，不能通过 bilateral M0 Gate。

下游外骨骼只消费 `/pico/smpl_palm_corrected` 及其同帧 status。status 中的每侧
`geometry_source`/`geometry_revision` 会写入新的 collar-to-spine3 attachment v2；
任一侧 TCP、腕部 pivot、geometry artifact、tracking epoch 变化后，旧 attachment
立即失效并必须重新进行固定 yaw 标定。外骨骼不会再次运行上肢 IK，也不会重复应用 TCP。

## 观察输出

~~~
ros2 topic hz /pico/smpl_palm_corrected
ros2 topic hz /pico/smpl_palm_corrected_ik
ros2 topic echo /pico/smpl_palm_corrected/status --once
ros2 topic echo /pico/smpl_palm_corrected --once
~~~

status 中每侧包含 baseline_ready、corrected、fallback_reason、
time_skew_ms、reach_clamped、wrist_palm_mode、wrist_palm_axis、
wrist_to_palm_distance_m、wrist_pivot_status、
wrist_orientation_mode 和 wrist_pivot_position_residual_m。一侧掌心超时或
几何退化时，只回退该侧四个原始 pose，另一侧和躯干/下肢继续发布。
顶层还包含 `ik_output_topic`、`ik_shoulder_frame_semantics`、
`ik_elbow_frame_semantics`、`ik_frame_valid` 和 `ik_frame_failure_reason`。
`ik_frame_valid=false` 只表示本帧没有发布 IK 坐标轴版本，不代表原修正骨架停止。

启动后检查完整输入输出链：

~~~
ros2 topic hz /pico/palm_left
ros2 topic hz /pico/palm_right
ros2 topic hz /pico/smpl_palm_corrected
ros2 topic hz /pico/smpl_palm_corrected_ik
~~~

## MuJoCo 对比

使用已有 viewer，把修正骨架设为主数据，把原始骨架设为 overlay，同时保留
掌心球和坐标轴：

~~~
ros2 run pico_bridge smpl_mujoco_visualizer \
  --topic /pico/smpl_palm_corrected_ik \
  --show-raw --raw-topic /pico/smpl_raw \
  --palm-topic /pico/palm_left \
  --right-palm-topic /pico/palm_right \
  --show-controllers --show-arm-axes \
  --raw-offset 0.45 0 0
~~~

因此 MuJoCo 主骨架显示的肩肘姿态与 ROS 2 的 IK topic 完全一致；可视化器不会
在本地再次修改肩肘坐标轴。若要查看原坐标语义，可把 `--topic` 改回
`/pico/smpl_palm_corrected`。

`--show-arm-axes` 只显示主（TCP/修正后）骨架的上肢局部 XYZ 轴；原始 PICO overlay 默认
不显示上肢坐标轴。需要调试原始骨架时，另加 `--show-raw-arm-axes`。

为避免两套骨架重叠，raw overlay 默认在 viewer 坐标中沿 X 方向偏移
0.45 m，并使用洋红色；修正骨架使用蓝色关节和灰色骨段。该偏移只影响
可视化，不会修改任何 ROS 输出或坐标系。若需要真实重合比较，显式传入
`--raw-offset 0 0 0`。

`--show-controllers` 在骨架旁绘制 `/pico/pose/head`、`/pico/pose/left_hand` 和
`/pico/pose/right_hand` 的原始头显/手柄位姿；`--show-raw` 只显示原始骨架，
不会隐式开启 controller overlay。若配置了 `--left-wrist-pivot-artifact`、
`--right-wrist-pivot-artifact`，viewer 始终从同一帧 TCP 掌心回算紫色腕球和腕到掌心连杆，
保证腕部姿态与掌心严格一致；即使 `/pico/wrist_left`、`/pico/wrist_right` 同时有数据，
也不会让不同时间基准或标定版本覆盖该结果。没有 pivot artifact 时，才使用实时 wrist
topic，再回退到主 PoseArray 的 LEFT_WRIST/RIGHT_WRIST 关节点。若 TCP 标定器没有持续运行，
viewer 会用默认的 `~/.config/pico_tracker/pico_*_palm_tcp.yaml` 和原始手柄话题自动回算掌心；
状态栏会显示 `wrist_pivot`、`topic` 或 `primary_posearray` 来源。

期望现象：

- 可达时修正后的左右 HAND 点与 TCP 掌心球重合；
- 上臂和前臂长度始终保持标定值；若 `reach_clamped=true`，修正 HAND 是最近可达点，
  TCP 掌心球继续显示原始强观测，二者距离通过 status 暴露；
- 肘部沿原始 PICO 肘部选择的分支连续变化；
- 原始骨架以 overlay 形式保留；
- 掌心超时不会冻结躯干和下肢。

## 测试

~~~
bash scripts/build.sh
source scripts/environment.sh
python -m pytest -q \
  src/pico_bridge/test/test_pico_palm_skeleton_filter_core.py \
  src/pico_bridge/test/test_pico_palm_skeleton_filter_node.py \
  src/pico_bridge/test/test_pico_palm_tcp_calibrator.py \
  src/pico_bridge/test/test_pico_palm_tcp_runtime.py \
  src/pico_bridge/test/test_launch_integration.py
git diff --check
~~~

本实验输出只能用于 PICO 骨架可行性验证；只有后续单独完成 frame adapter、
ground 对齐和外骨骼/ESEKF 验证后，才可以进入更高层 shadow pipeline。
