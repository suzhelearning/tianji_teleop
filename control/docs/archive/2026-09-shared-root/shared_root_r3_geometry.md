# R3：闭合几何契约与纯函数验证

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

日期：2026-09-18。阶段 1R 几何证据及阶段 4 闭合原语，不是完整 R3 接线验收。

后续进度：TargetBuilder/continuity/guidance 的 R3 接线见
[接线验证报告](shared_root_r3_wiring.md)。下文“尚未接线”和独立 filtered
覆盖率记录保留为本阶段历史，不代表当前实现状态。

## Artifact

`shared_root_robot_geometry.yaml` contract_version=2，SHA256：
`41a747a39a7f8ba09311b3a143292296b6d72c1d5999dea6de5599bcb611b937`。
模型仍为 `marvin_m6_wuji2_shared_root.xml`，TCP 在 Link7 中为 `[0,-0.1615,0] m`。
左右分别冻结肩 Link1、肘 Link4、腕位置 Link5、solver tcp、L1/L2、
T_Link7_TCP、T_TCP_WristCenter。WristCenter 为原点在实际腕中心、轴平行 TCP 的
虚拟坐标系，不代表真实 Link5 姿态固定。闭合原语不读取旧局部 proxy 向量，
也不硬编码 0.1615。读取器和指纹同步更新。

## 跨模型证据

`SharedRootGeometry.R3ClosureExtrinsicsAcrossModelsAndJointPoses`：
左右各 100 组姿态，采样比例为
`0.5+0.49*sin((k+1)*(j+1)*0.6180339887498948)`，覆盖模型各关节范围；
另保留零位、Home、非零姿态既有测试。只 FK，无动力学或设备。

| 最大误差 | 左 | 右 |
|---|---:|---:|
| TCP 系腕中心向量变化/m | 4.67109e-16 | 3.26615e-16 |
| Link5/6/7 原点差/m | 0 | 0 |
| Pinocchio/MuJoCo 位置差/m | 1.32892e-6 | 1.34066e-6 |
| TCP 姿态差/rad | 2.36611e-6 | 2.86113e-6 |
| 采样骨段长度与参考长度差/m | 7.61996e-8 | 7.62113e-8 |

外参不变量容差 1e-10 m，跨模型容差 1e-5 m/rad。
不能把跨模型容差当作目标链代数闭合容差。

## 闭合原语

新增 `shared_root_closure.{hpp,cpp}`，调用者显式传入只读已接受肘分支。
目标掌 pose 不变，固定肩点和两段长度，用两球交圆重建肘，TCP 外参反推腕；
`hand=palm.position`。不可达返回状态，不投影、不拉长骨段。
测试覆盖解析交圆、完全伸直、内侧折叠、无历史退化、有历史恢复、肩腕重合、
非有限输入、工作空间内/外及两侧 200 个 FK pose。

首次测试错误地要求闭合后肘点完全等于 FK 偏好，失败；改用模型容差仍有
2 个样本超出（最大 4.21137e-5 m）。固定参考骨长与不同姿态样本微差在接近
伸直时会放大，因此肘点是投影偏好而非硬约束。最终保留该位移为诊断，
严格检查输出骨长/末点 <=1e-9 m，没有放松闭合门或掩盖失败历史。

## zhoujie 4388 帧纯几何候选检查

输入仍为 `zhoujie_50s_20260917_235744_GLTAPL/input.tjvr`，SHA256
`8d92f338056a40f73602e6d029c807d807dea29cc6e378b15946d85d3354ab62`。

| 层 | 双侧几何有效 | 任一侧不可达 | 分支未定 |
|---|---:|---:|---:|
| raw | 4376/4388（99.7265%） | 12 | 0 |
| filtered | 4383/4388（99.8861%） | 5 | 0 |

有效输出最大骨长误差 3.33067e-16 m，末点到掌目标误差 0。
没有伪造控制接受历史；这只是逐源帧几何覆盖率，不含 blend/控制接受/连续失效区间，
没有动作标签，不代替 per-action 启用门或真实设备验收。

## 当前实施边界

本轮最终验证：相关 CTest 13/13 通过，Python 55 通过、2 个完整参考循环用例
主动排除（尚未接入 R3，不能拿 R2 回放作为 R3 证据）。`git diff --check` 通过。
旧 XML、pico2_hands 和 pico2_sim.sh 零差异；未运行设备、完整仓库回归或真机。

闭合函数及诊断已实现，但原 TargetBuilder/continuity/guidance 接线仍是 R2。
因此现有映射 Viewer 仍显示旧的分离目标，不应当作 R3 画面。
下一步必须一起实现 raw/filtered/blended 闭合、每侧已接受分支只读快照、
双侧控制参考 ack 原子提交、Home/epoch/recovery 历史处理和不可达恢复测试。
尤其当前 ack 仅用于恢复结束，不能直接拿它冒充每周期分支提交。
默认 enabled=false；旧路线、驱动、QP、PICO2 文件未修改。不提交/push。

复现：

```bash
cmake --build control/build --parallel 4
ctest --test-dir control/build -R '^test_shared_root_' --output-on-failure
control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root.yaml \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr --mapping-only
```
