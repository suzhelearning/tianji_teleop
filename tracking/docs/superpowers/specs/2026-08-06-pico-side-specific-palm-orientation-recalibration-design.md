# PICO 单侧掌心姿态独立重标定设计

## 1. 目标

在不重新标定 controller 到掌心平移、掌心到手腕距离及人体上臂/前臂骨长的前提下，允许操作人员针对左臂或右臂单独重新标定 `T_controller_palm` 的旋转部分。

姿态重标定必须：

- 由独立脚本启动，不并入 TCP、腕部、骨长的完整适配顺序；
- 提供稳定的 ROS service，供后续外骨骼联合标定流程调用；
- 直接更新所选侧现有 TCP artifact 中的 `quaternion_xyzw`；
- 严格保持 `translation_m` 不变；
- 不修改另一侧 TCP artifact；
- 只更新 TCP 姿态时，现有腕部和骨长 artifact 继续有效；
- 成功后立即更新当前掌心 publisher 的内存变换，无需重启运行链；
- 失败时保持当前内存变换和磁盘 artifact 不变。

本功能仍为 shadow/M0 数据准备功能，不改变 active-control Gate。

## 2. 操作姿势和坐标约定

操作人员采用与完整 TCP 标定独立姿态阶段（第 5 次采样）相同的参考姿势：

1. 身体自然直立，头部正视前方；
2. 双臂向正前方水平伸直；
3. 左右掌心相对；
4. 手腕保持中立，不主动屈伸或侧偏；
5. 进入采集窗口后保持静止。

虽然操作动作使用双臂，标定请求一次只处理一个 `side`。未被选择的一侧不参与求解，也不修改文件。

沿用当前掌心坐标约定：掌心局部 `+X` 从腕部指向掌心/手指方向。参考姿势沿用完整 TCP 标定独立姿态阶段的 HMD-relative 定义：

```text
R_V_H_ref = I
```

其中 `V` 为 HMD，`H` 为人体掌心。该定义保证新功能和已经使用的 TCP 姿态语义一致，而不是引入第二套掌心坐标定义。

## 3. 用户接口

### 3.1 独立脚本

```bash
./scripts/calibrate_pico_palm_orientation.sh left
./scripts/calibrate_pico_palm_orientation.sh right
```

脚本职责：

- 检查选定侧 TCP artifact 存在且严格有效；
- 检查对应 service 是否可用；
- 若当前没有该侧 TCP publisher，则只启动一个临时 publisher，并在结束时有界清理；
- 显示参考动作；
- 等待操作人员按空格；
- 调用对应侧 service；
- 只打印成功/失败、旋转修正量、姿态 RMS、revision 和 artifact 路径；
- 成功或失败后明确退出并返回终端。

脚本默认继承当前工程约定：`ROS_DOMAIN_ID=120`、`ROS_LOCALHOST_ONLY=1`，并要求在项目 `pixi shell` 内运行。

### 3.2 ROS service

每侧 TCP publisher 提供一个 `std_srvs/srv/Trigger`：

```text
/pico/palm_orientation/left/calibrate
/pico/palm_orientation/right/calibrate
```

用 service 名称表达侧别，避免请求中出现无效 side，也避免一次调用意外修改双侧。后续外骨骼联合标定可以在自身姿态 Gate 通过后调用所需侧的 service。

service 在采集和写盘完成后返回。`success=false` 时 `message` 给出可操作的失败原因；`success=true` 时 `message` 返回紧凑 JSON，至少包含：

```text
side
artifact_path
calibration_revision
translation_revision
orientation_revision
tracking_epoch
sample_count
orientation_rms_rad
correction_angle_rad
```

同一 publisher 同一时间只接受一个姿态标定请求；忙时立即拒绝，不排队。

## 4. 组件边界

### 4.1 `pico_palm_orientation_core.py`

纯数学和 artifact 更新模块，不依赖 ROS：

- controller/HMD 配对样本验证；
- SO(3) 稳健平均；
- 右局部残差和协方差计算；
- TCP 平移语义指纹计算；
- orientation-only TCP 文档更新；
- legacy lineage 兼容信息生成。

### 4.2 `pico_palm_tcp_publisher.py`

继续作为该侧有效 TCP 状态的唯一运行时 owner，并新增：

- HMD、tracking epoch 订阅；
- controller/HMD 短窗口配对缓冲；
- 该侧姿态标定 service；
- 标定互斥状态；
- 成功后原子替换文件并在锁内切换内存 `TcpTransform`；
- 使用 `MultiThreadedExecutor`，使 service 等待采样时订阅回调仍可推进。

发布器不得在标定开始时提前改变当前变换。只有全部 Gate 和磁盘原子写入成功后，才切换内存姿态。

### 4.3 `calibrate_pico_palm_orientation.sh`

只负责人机交互、service discovery、可选临时 publisher 生命周期和结果展示，不重复实现求解或 artifact 写入。

## 5. 数据采集和求解

选定侧使用：

```text
/pico/pose/<side>_hand
/pico/pose/head
/pico/tracking_epoch
/pico/tracking_epoch/status
```

每对样本必须：

- pose 有限；
- 四元数可归一化；
- frame 为 `pico`；
- controller 与 HMD source timestamp 差不超过 `max_pair_skew_s`，默认 `0.03 s`；
- tracking epoch 在整个窗口中保持不变且来源明确；
- 样本时间单调；
- 不是重复 timestamp。

默认采集不少于 120 对有效样本，覆盖约 2 秒。每个样本计算：

```text
R_C_H_i = R_G_C_i^T * R_G_V_i * R_V_H_ref
R_V_H_ref = I
```

在 SO(3) 上迭代求均值：

```text
r_i = Log(R_mean^T * R_C_H_i)
R_mean <- R_mean * Exp(robust_mean(r_i))
```

使用 Huber 权重抑制少量 tracking 抖动。最终协方差和 RMS 使用内点右局部残差计算。

新的姿态相对当前 TCP 姿态的修正量为：

```text
correction_angle = norm(Log(R_C_H_old^T * R_C_H_new))
```

## 6. Gate 和失败行为

初始可配置门槛：

- 有效配对样本数 `>= 120`；
- controller/HMD skew `<= 0.03 s`；
- 姿态残差 RMS `<= 0.05236 rad`；
- 单次姿态修正角 `<= 0.7854 rad`；
- tracking epoch ambiguity 为 0；
- 采集过程中 epoch 变化为 0；
- 非有限样本 accepted 数为 0；
- 输出旋转矩阵为 proper SO(3)，输出四元数有限且归一化。

参考动作本身由操作提示和上层联合标定流程保证。原始 SMPL 肘角或腕姿态不得作为姿态标定真值；它们最多作为 diagnostics，不能因原始 SMPL 姿态误差拒绝本次重标定。

以下任一情况失败并保持原状态：

- service 已忙；
- 输入 topic 未就绪或 stale；
- tracking epoch 不明确或发生变化；
- 样本数不足；
- 时间配对失败；
- 姿态抖动超限；
- 修正角异常；
- TCP artifact 在采集期间被其他进程修改；
- candidate 验证失败；
- 原子写入失败。

## 7. TCP artifact 更新语义

姿态更新以当前严格有效 TCP artifact 为基底。以下字段必须逐值保持不变：

```text
side
pose_semantics
transform_convention
source_topic
translation_m
translation_revision
translation_fingerprint_sha256
```

旧 artifact 缺少 `translation_revision` 时，以更新前 `calibration_revision` 初始化；缺少平移指纹时现场生成。平移指纹使用 canonical JSON 的 SHA256，只覆盖：

```text
schema identifier
side
transform_convention
translation_m
```

成功后更新：

```text
quaternion_xyzw
calibration_revision += 1
orientation_revision += 1
orientation_calibrated = true
orientation_reference
orientation_calibration
covariance_upper_triangle_6x6 的 orientation block
```

`orientation_calibration` 保存方法、tracking epoch、样本数、RMS、修正角、标定时间和输入 lineage。更新前完整 TCP 文件 SHA256 被追加到 `orientation_only_ancestor_sha256`，用于兼容已有骨长 artifact。

写入流程：

1. 读取并严格验证当前 TCP；
2. 记录更新前文件 SHA256；
3. 构造 candidate；
4. 验证 candidate 的平移逐值不变、指纹不变、姿态合法；
5. 检查磁盘原文件 SHA256 仍等于步骤 2，防止并发覆盖；
6. 写入同目录临时文件并 `fsync`；
7. 原子 rename 替换；
8. 在运行时锁内切换内存旋转。

任何步骤失败时不切换内存状态。

## 8. 骨长 lineage 兼容

当前 geometry v3 同时绑定 TCP `calibration_revision` 和整个文件 SHA256，导致仅更新四元数也会误使骨长失效。新规则必须把几何依赖限定到 TCP 平移：

- 新 geometry artifact 保存 `tcp_translation_revision` 和 `tcp_translation_fingerprint_sha256`；
- geometry validator 比较当前 TCP 的平移 revision 和语义指纹；
- TCP `orientation_revision` 和完整文件 SHA256 不参与新 geometry 的有效性；
- 只更新 TCP 姿态时，骨长继续有效；
- TCP 平移改变或平移指纹不匹配时，骨长立即失效；
- wrist artifact 的现有严格 lineage 保持不变。

兼容已有 geometry v3：

- 将旧 `tcp_calibration_revision` 与当前 `translation_revision` 比较；
- 将旧 `tcp_artifact_sha256` 与当前 TCP 完整 SHA256或 `orientation_only_ancestor_sha256` 中任一值比较；
- 只有由受控 orientation-only 更新形成的祖先链才能继续接受，任意手工修改不能绕过平移指纹自一致性检查。

因此当前已经通过 Gate 的上臂/前臂长度在姿态重标定后继续有效，无需重新采集。

## 9. 运行时和并发语义

- publisher 构造掌心时始终读取锁保护的单一 `TcpTransform` 快照；
- 一个输出消息不得混用旧平移和新旋转；
- artifact 更新成功前继续发布旧掌心姿态；
- artifact 更新成功后下一帧起使用新姿态；
- 标定期间正常 palm topic 不停更；
- 另一侧 publisher 完全不受影响；
- PICO tracking reset/reconnect 会使当前采集失败，但不会自动删除持久姿态标定；
- 该功能不自动触发，每次由脚本或外部 service caller 明确触发。

## 10. 测试要求

### 10.1 数学单元测试

- 已知 controller/HMD 旋转恢复 `R_C_H`；
- quaternion 符号翻转不影响 SO(3) 均值；
- 少量 outlier 被 Huber 抑制；
- 右局部 residual 和 finite-difference 结果一致；
- 非 proper rotation、NaN、样本不足被拒绝。

### 10.2 Artifact 测试

- orientation-only 更新后 `translation_m` 逐值不变；
- translation revision 和 fingerprint 不变；
- calibration/orientation revision 增加；
- 原 artifact SHA 进入 ancestor lineage；
- 写入失败不改变原文件；
- 并发修改导致 compare-before-swap 失败；
- orientation covariance block 为 PSD；
- 手工改变 translation 而不更新指纹必须失败。

### 10.3 Geometry lineage 测试

- 新 semantic lineage 在姿态更新前后均有效；
- legacy geometry 在一次和多次受控姿态更新后仍有效；
- TCP 平移改变后 geometry 失效；
- wrist hash 改变后 geometry 仍按现有规则失效；
- 非 ancestor 的旧 TCP hash 不得接受。

### 10.4 ROS/脚本测试

- left service 只修改 left 文件和内存状态；
- right service 只修改 right；
- busy request 立即拒绝；
- epoch change、stale、skew、低样本数和高 RMS 均失败关闭；
- 成功后无需重启 publisher 即在下一帧使用新旋转；
- script 在已有 service 时不启动重复 publisher；
- script 在 service 缺失时启动并有界清理临时 publisher；
- 成功、失败和 Ctrl-C 都返回终端且无残留进程。

## 11. 验收标准

1. `./scripts/calibrate_pico_palm_orientation.sh left|right` 可独立运行。
2. 单侧成功标定后只更新该侧 TCP `quaternion_xyzw`。
3. TCP `translation_m`、腕部距离、上臂和前臂长度完全不变。
4. 当前 publisher 不重启即可使用新姿态。
5. 已激活 geometry artifact 在 orientation-only 更新后仍通过严格 validator。
6. TCP 平移发生变化时 geometry artifact 必须失败。
7. 外骨骼流程可通过稳定 ROS service 调用同一功能。
8. 所有失败路径保持旧文件和旧运行时姿态。
9. `pico_bridge` 全部单元、集成和脚本测试通过。
10. 不提交 recordings 或真机采集数据。

## 12. 非目标

- 不重新估计 controller 到掌心平移；
- 不重新估计掌心到手腕距离；
- 不重新估计人体骨长；
- 不同时标定左右侧；
- 不使用原始 SMPL 腕姿态作为真值；
- 不修改 PICO pelvis；
- 不引入 DLS、QP、ESEKF 或外骨骼 prior；
- 不允许 active control 直接消费未经既有 Gate 审批的新输出。
