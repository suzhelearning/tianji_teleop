# 1.62 m 录制的合成人体映射实验

本次仅离线执行生产映射与几何闭合检查，没有运行 IK、仿真执行器或设备。
没有修改映射算法、在线配置或原始录制。

## 输入与方法

- 源数据：`recordings/shared_root/dls_interactive_5axzx3cg/input.tjvr`，4471 帧，49.8751 秒。
- SHA256：`ee2c35391fcb02879ad9f54b4a1f7a63551f80123be88e89d36adeb0dc844474`。
- 与 zhoujie 的 `cal-ba50d9be9c374fb19002324c38539757` 1.62 m 标定骨长一致；不能据此证明采集时配置来源。
- 当前配置：`control/config/qp_ik_pico_shared_root_dls.yaml`。
- 原始帧的时间、旋转、骨段方向保持不变；围绕桥接根 `[0,0,1.121]` 改变肩点偏移和骨段长度，重新计算 CRC。
- 这是 **M0 输出之后**的合成骨架。旧协议头部姿态不改，仅用于共享根 mapping-only 审计，不得用于在线执行器回放。
- 结果、各组合成数据及逐帧日志：`recordings/shared_root/synthetic_body_20260920_1/`，结果索引 `results.json`。

## 结果

下表差值是相对基线的双侧掌心映射目标差异，**不是 IK 跟踪误差**。
P90 在全部有效帧的两侧掌心距离上计算。

| 合成条件 | 掌心差值 P90 | 最大差值 |
|---|---:|---:|
| 等比例身高 1.45、1.50、1.75、1.85、1.95 m | < 1e-12 m | < 1e-12 m |
| 上臂、前臂均缩短 10%，肩宽和腕掌不变 | 1.32 cm | 1.63 cm |
| 上臂、前臂均加长 10%，肩宽和腕掌不变 | 1.30 cm | 1.63 cm |
| 肩宽缩小 15%，骨段长度不变 | 2.48 cm | 3.82 cm |
| 肩宽增大 15%，骨段长度不变 | 1.84 cm | 2.82 cm |
| 上臂加长 10%、前臂缩短 10% | 6.15 cm | 6.49 cm |

含基线共 11 组，每组 4471/4471 帧产生有效目标；最终 filtered 几何闭合均为
4471/4471。姿态矩阵相对基线差值为零。源文件、配置、审计程序前后指纹不变。

`mapped_palm_outside_workspace_ratio` 是偏好目标的诊断，不是最终目标拒绝率：
例如基线 15 帧、窄肩 70 帧触发共同平移修正，最大修正分别为 1.96 mm、2.78 mm，
最终均闭合。几何闭合不验证关节限位、碰撞或 IK 可解性。

## 解释与限制

当前尺度归一化在这段动作上确实抵消了等比例身高变化，不是只对 1.62 m 生效。
人体比例变化则不会完全抵消；尤其上臂/前臂比例改变时，同样关节方向会自然产生
不同的掌心位置。因此这些差值本身不能直接判为算法错误，但说明不能承诺换人后
完全同样的操作手感。

该实验没有覆盖原始 PICO 人体估计、按身高模板重建/截断、真实 TCP 标定差异、
不同人的动作习惯和传感噪声，也没有动作标签与真人验收门限。不能替代真人测试。

## 复现及验证

在仓库根执行，输出目录必须不存在。以下是本次本机数据的复现命令，录制和个人标定
不保证随 Git clone 提供。换用其他数据时同时替换 `--trace` 与 `--calibration-dir`；
工具校验其骨长一致性，并读取标定身高作为缩放基准，不能把别人的录制套用本次标定。
原生审计程序需先通过 `pixi run build` 构建。

```bash
pixi run --locked python control/scripts/audit_synthetic_body_mapping.py \
  --trace recordings/shared_root/dls_interactive_5axzx3cg/input.tjvr \
  --calibration-dir profiles/zhoujie/pico-simple/cal-ba50d9be9c374fb19002324c38539757 \
  --profile control/config/qp_ik_pico_shared_root_dls.yaml \
  --auditor control/build/tianji_shared_root_trace_audit \
  --output-dir recordings/shared_root/synthetic_body_new_run
pixi run --locked python -m unittest discover -s control/tests -p test_synthetic_body_mapping.py
```

实验执行轮次：11 组原生离线映射审计成功；新增 Python 测试 7/7 通过；`git diff --check` 通过。
未运行完整 CTest；未进行 IK、真实输入仿真或真机验收。

使用入口：[项目 README](../../../README.md)、[控制说明](../../README.md)、
[新人员标定与适配边界](../../../docs/pico-simple-calibration.md#换人身高与臂长适配)。
