# TCP 0.1615 m：映射层离线复核

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

日期：2026-09-18。依据 V2.1(5) 的输入、几何、滤波和连续性契约，
采用用户静态确认的 Link7 局部 TCP `[0,-0.1615,0] m`。
本次优先验证映射，不调 QP 权重、不增加迭代，不启动设备或 Viewer。

## 可复现输入

- profile：`control/config/qp_ik_pico_shared_root.yaml`（默认关闭）
- 模型：`control/models/marvin_m6_wuji2_shared_root.xml`
- 模型 SHA256：`7ddd13b21c483b10a4c691570f5fac041b3c1ee479989e8a8bd341f26482c617`
- geometry SHA256：`06ad68e406a8b7474683f43b35b76d6a3be7d82e5654a85b73a55aecc78fb3b3`
- profile SHA256：`f9262d6186315341e1daeb9693b16eebfd14c7d6dd38f997099a0e29be97e9ca`
- trace：`recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr`
- trace SHA256：`8d92f338056a40f73602e6d029c807d807dea29cc6e378b15946d85d3354ab62`

新增 `--mapping-only` 在映射检查后退出，不构造 IK 求解器或控制器，
无 socket、mj_step、执行端发布。模型加载只做离线 FK。

```bash
cmake --build control/build --parallel 4
control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root.yaml \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr \
  --mapping-only
ctest --test-dir control/build \
  -R 'test_shared_root_|^test_(config|spark_upper_retarget|pico_skeleton_overlay|spark_guidance)$' \
  --output-on-failure
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest \
  control/tests/test_shared_root_trace_cli.py control/tests/test_shared_root_trace_audit.py \
  control/tests/test_shared_root_contract.py control/tests/test_shared_root_palm_priority_profile.py \
  -k 'not recorded_reference_loop' -q
```

## 本轮结果

4388/4388 源帧有效，0 帧拒绝。掌目标最大间距 1.81572 m，无间距门限。

| 检查量 | 结果 |
|---|---:|
| raw 双掌相对向量与共同变换公式的最大差 | 2.42843e-16 m |
| raw 中心映射公式最大差 | 1.22799e-16 m |
| raw/filtered 骨段长度最大误差 | 2.22045e-16 m |
| raw/filtered 固定肩点最大偏差 | 0 m |
| raw 姿态矩阵与规定变换的最大差（Frobenius 范数） | 0 |
| raw 掌目标与臂形 hand 末点间距 p90 | 93.5182 mm |
| filtered 掌目标与臂形 hand 末点间距 p90 | 94.1056 mm |
| filtered 与 raw 掌目标位置差 p90 | 39.5611 mm |

几何等式检查阈值为 1e-9。间距和滤波差按双侧样本汇总，
不是 IK 残差、实际机器人误差或时延；不可混用。滤波差包含运动及尺度滤波影响。
未给真实录制人工补造动作标签。合掌、交叉等分动作契约由已有合成测试覆盖。

本轮 CTest 12/12 通过（包含新 TCP FK、连续性、guidance 与旧分支回归）；
Python 46 通过，2 个完整参考循环回放用例主动排除，未把旧 TCP 的回放结果沿用。
`git diff --check` 通过；旧模型、`pico2_hands/**` 和 `pico2_sim.sh` 零差异。
未运行完整仓库回归、硬件输入、动力学或真机验收。

## 判断与待验收项

映射代数与固定骨长契约通过，但 **不能宣布预期动作效果已通过**。
共同尺度映射掌目标与固定骨长按方向重建臂形是两条不同构造，
新 TCP 下仍存在约 9.4 cm 的 p90 末点分离，发生在 IK 之前。
这不是已证明的 QP 故障，也不因模型 TCP 点移到掌心附近就自动消失。

V2.1 允许 proxy 与 palm 分离；如果用户要求二者在每个映射目标中重合，
需要明确改变目标构造（例如由掌 pose 反推腕点并协调肘形），
而不能只改可视化线段或提高 QP 权重。本轮没有擅自实施这种设计变更。
下一步应独立展示映射骨架及 palm 目标，确认这种分离是否满足用户预期。
保持默认关闭；不提交/push，等待验收。
