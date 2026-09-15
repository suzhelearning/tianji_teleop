# PICO 掌心重力水平姿态标定设计

## 目标

完整 TCP 姿态标定和单独姿态重标定都不再继承头显的俯仰与横滚。标定姿势中，掌心 pitch/roll 相对 PICO 世界重力水平，yaw 跟随头显朝向。

## 坐标与目标姿态

- `G`：PICO world/ground，`+Z` 为重力反方向。
- `V`：头显。
- `C`：所选侧 controller。
- `H`：人体掌心。

从 `R_G_V` 的局部 `+X` 轴提取水平 heading：将 `R_G_V[:,0]` 投影到 `G` 的 XY 平面并归一化为 `x_level`，设置 `z_level=[0,0,1]`，再以 `y_level=z_level×x_level` 构造 `R_G_L=[x_level,y_level,z_level]`。若头显 `+X` 接近竖直而无法稳定投影，则拒绝样本。

标定目标为：

```text
R_C_H = R_G_C^T * R_G_L
```

因此运行时 `R_G_H=R_G_L`：pitch/roll 为零，yaw 与头显一致。左右侧使用相同目标坐标约定；另一条手臂只用于操作人员摆出对称参考姿势，不参与所选侧计算。

## 共享求解

`pico_palm_orientation_core.py` 提供唯一的 `gravity_leveled_heading_rotation()`。完整 TCP 标定器和运行时姿态重标服务都调用该函数，禁止各自实现 Euler 角剥离逻辑。

多帧求解继续在 SO(3) 上进行 Huber 均值和 RMS Gate。单独姿态重标定沿用显式 tracking epoch、30 ms 配对偏差和至少 120 个样本。

## 完整 TCP 第五阶段

第 1～4 次空格仍只标定平移。位置通过后，操作人员双臂向前水平伸直、掌心相对；第 5 次空格启动姿态采集窗口，而不是立即保存一帧。

完整 TCP 标定器订阅显式 tracking epoch/status，并复用 `OrientationCaptureBuffer`：

- controller 与 head corrected/source stamp 最大偏差 30 ms；
- tracking epoch 必须来自显式来源且采集期间不变；
- 至少采集 120 个配对样本；
- 默认超时 5 s；
- SO(3) RMS 不超过 0.05236 rad；
- 失败时保留已求出的内存平移但不写 candidate，可重新按空格采集姿态；
- 成功后一次性写入完整 TCP candidate 并退出。

完整 artifact 的 orientation covariance 使用多帧残差协方差，质量字段记录样本数、RMS、tracking epoch、参考类型 `gravity_leveled_hmd_heading`。

## 兼容性

- `translation_m`、平移 revision/fingerprint 和平移 Gate 不变。
- 腕部与骨长 artifact schema 不变。
- 单独姿态更新仍只增加 orientation revision，绝不修改 TCP translation。
- 旧 TCP 文件仍可加载；新标定文件明确写入新的 orientation reference/method。
- active control 不在本改动范围内。

## 测试

1. 头显含任意小幅 pitch/roll 时，水平参考帧输出 pitch/roll 为零且 yaw 保持。
2. 头显水平 `+X` 投影退化时拒绝。
3. 多帧姿态求解不复制头显 tilt。
4. 完整 TCP 第 4 次只锁定平移，第 5 次启动窗口，样本不足不保存。
5. 120 个有效配对样本后保存，translation 不变。
6. 超过 30 ms 的配对样本被拒绝，epoch 改变使采集失败。
7. 单独姿态服务集成测试验证输出为水平 yaw，而不是完整头显姿态。
