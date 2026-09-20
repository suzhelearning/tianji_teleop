# Shared-root V2.1-R3：Phase A 阶段交付索引

> 历史归档：保留当时的配置、测试与未完成项，不代表当前状态或运行指令。
> 当前入口与验收边界见 [交互仿真说明](../../verification/ceres_interactive_sim.md)。

> 后续进展：本页主体保留首次收尾的历史状态。2026-09-18 的真实输入已完成七类
> 动作采集和用户确认，见[动作证据](shared_root_actions_20260918.md)。用户随后明确
> 要求不可达目标采用可达近似，新增[独立投影实验](shared_root_reachable_projection.md)，
> 这是对原方案“不投影”规则的明确授权扩展，原拒绝配置不变。当前边界和最新诊断
> 见[本轮交付复核](shared_root_phase_a_followup_20260918.md)，不要将下文“动作标注缺失”
> 作为当前状态。

日期：2026-09-18。工作区 HEAD：`b0713576724499d229e12155d7db1daed096e9fe`。
本轮按阶段 1R～6 收尾，不新增功能，不进入 Phase B，不改求解器或默认路线。
**实现交付、软件回归、最终启用验收是不同结论。** 本文件不授予设备或真机运动权限。

## 交付对应关系

| 阶段 | 实现与证据 |
|---|---|
| 1R 输入与模型契约 | 固定 TJVR 双肩坐标系、M0 重建掌心和单次 basis；完整 R3 几何 artifact、双模型左右多姿态核验，见[输入契约](../../shared_root_input_contract.md)与[几何报告](shared_root_r3_geometry.md) |
| 2 输入适配 | `shared_root_input.*`；固定偏移只扣一次，禁止 runtime 姿态来源 fallback，不重建世界 yaw |
| 3 尺度与滤波 | `shared_root_morphology.*`、target builder；PROVISIONAL/CONFIDENT、固定窗口、唯一源帧推进，不做 Phase B shadow reacquire |
| 4 唯一掌心与臂形闭合 | `shared_root_closure.*`、target builder；保留 affine 掌目标，固定腕掌几何、两球交圆肘选择；raw/filtered/control 末点均来自 palm，见[接线报告](shared_root_r3_wiring.md) |
| 5 最小连续性 | `shared_root_continuity.*`、pipeline；HOLD/INVALID/RECOVERING，不回退 legacy；模型快照恢复，同周期双侧外部参考 ACK 后提交分支历史 |
| 6 guidance 与离线验证 | `shared_root_guidance.cpp` 及旧 guidance 隔离分支；关节约束保留、disabled 直走旧分支；原录制回放、[覆盖率报告](shared_root_r3_coverage.md)、[动作统计入口](../../verification/shared_root_action_coverage.md) |

当前仍为实验配置。上述表格表示实现和已有证据的位置，不代表各动作已取得
现场验收，也不表示每个 IK 目标都可达、无碰撞或准确跟踪。

## 当前冻结指纹

本轮重新读取文件验证：

| 文件 | SHA256 |
|---|---|
| shared_root_tjvr_input_contract.yaml | `68304d5b76b063996bdf386b28d103af1d998f7f9a801acb5a55960aaa2fccda` |
| shared_root_robot_geometry.yaml（v2） | `41a747a39a7f8ba09311b3a143292296b6d72c1d5999dea6de5599bcb611b937` |
| qp_ik_pico_shared_root.yaml | `f9262d6186315341e1daeb9693b16eebfd14c7d6dd38f997099a0e29be97e9ca` |
| marvin_m6_wuji2_shared_root.xml | `7ddd13b21c483b10a4c691570f5fac041b3c1ee479989e8a8bd341f26482c617` |

原模型和 PICO2 的 TCP 不变。tracking/profile 已有未提交修改来自此前简化标定，
本轮不清理、不覆盖，也不将其归为 R3 控制层修改。

## 本轮复测

```bash
ctest --test-dir control/build --output-on-failure -j 1
```

执行前检查了注册项：包含无窗口 Viewer 软件回归及本机合成 UDP，
不启动 PICO/M0 驱动、不采集现场设备、不向真机发布。
结果：**98/98 项通过**，耗时 **128.12 s**，没有失败或跳过。
包含 shared-root、旧 SPARK、模型/协议、状态机、快照、录制和无窗口合成输入集成。
这是本轮完整 CTest，不拿历史局部回归代替；不等于真实设备或操作系统实时验收。
`git diff --check` 通过；PICO2 目录/入口、real_robot 和原 XML 相对 HEAD 零差异。

### 当前 R3 尺度 A/B 补测

```bash
control/build/tianji_shared_root_trace_audit \
  control/config/qp_ik_pico_shared_root.yaml \
  recordings/shared_root/zhoujie_50s_20260917_235744_GLTAPL/input.tjvr --scale-ablation
```

两种尺度共用 morphology 输入、各自保留 builder/IK history；只有两组几何均有效
且四侧求解均接受的同一源帧集合才比较误差。**这是 filtered pose IK，不含 guidance
恢复、headroom、velocity-QP 或实际执行**，不能用其误差冒充最终控制跟踪效果。
候选无效会让两组重新准备滤波；不是生产 pipeline 的完整回放。

| 指标 | 各向异性 | 统一尺度 |
|---|---:|---:|
| 完整源帧 | 4388 | 4388 |
| 成对比较帧 | 3588 | 3588 |
| 排除帧 | 800 | 800 |
| 求解拒绝侧样本 / 预算耗尽侧样本 | 0 / 0 | 0 / 0 |
| palm p90 / m | 7.41932e-6 | 5.27535e-6 |
| B 系双掌关系误差 p90 / m | 1.16760e-5 | 1.05312e-5 |
| 靠近安全关节限位的侧样本 | 291 | 388 |

排除约 18.23% 的源帧后才能取得上述共同集合。这里的 800 是比较工具的排除数，
不是生产路线不可达比例，也不能与 standalone mapping 的 12 帧拒绝直接互换。
当前输出没有逐模式细分这 800 帧的所有原因，因此不据此宣称某种尺度工作空间更优。
小残差只说明所选成对集合的 pose IK 结果，不说明全动作覆盖率、滤波响应或机器人
参考跟踪同样优秀。本轮不改变尺度、权重、迭代或启用配置。

## 剩余验收边界（首次收尾历史记录）

1. **代表性动作标注缺失**：当前 zhoujie 录制没有可信的动作时间区间。
   总体覆盖率不能代替前伸、靠近、交叉、高低、单手、共同运动、伸直边界的分项结果。
2. **启用门限尚未评审冻结**：不可自行选择 95%/99% 或最长失效阈值；
   动作统计工具不提供自动启用或变更配置的功能。
3. **实测反馈等尚未验证**：actual、设备输入下的仿真、动力学、碰撞安全、真机
   不由离线回放或合成输入测试替代。严格秩亏/更广覆盖的动作边界也不因近奇异测试
   通过而被宣称全部覆盖。

固定双掌间距 1.6 m 门已删除；关节保护及两连杆几何有效性保留。
10 mm 掌误差、12 mm 关系误差以及姿态精度按现行方案属于诊断，不为此改 QP。

因此保留 `spark_shared_root.enabled:false`、`phase_a_accepted:false`。
阶段交付后等待真实动作标注/验收评审；不继续扩展离线工具来替代缺失证据，
也不擅自通过启动设备绕过验收门。未 commit/push、未创建分支、未进入 Phase B。
