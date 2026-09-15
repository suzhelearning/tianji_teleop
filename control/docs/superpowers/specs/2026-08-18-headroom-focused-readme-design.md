# Headroom 主线 README 重构设计

## 目标

将根目录 README 从多代算法混合说明重构为当前 `main` 的单一主线文档，核心突出
`spark_upper_qpoases_headroom_feedforward_velocity_qp`。历史算法资料保留，但迁移到
`docs/legacy_algorithms.md`，不再打断实时遥操和回放的主阅读路径。

## README 结构

README 控制在约 150–200 行，按以下顺序组织：

1. 工程定位：标题和首段直接说明 PICO、SPARK Headroom、Velocity QP、MuJoCo。
2. 构建：保留 Pixi 安装、配置、编译和测试命令。
3. PICO 实时遥操：PICO_tracker 一键启动以及天机侧无参数 Viewer 命令。
4. 默认配置：用短表列出算法、模型、控制层、状态源、UDP、骨架 Overlay 和控制频率。
5. 主算法：只描述当前数据流、Headroom 衰减、输出连续性、限位处理和 settled-hold。
6. 显式启动与遥测：保留完整可复制命令和 CSV 路径。
7. MuJoCo TJVR 回放：保留 `output_continuity_retest.tjvr` 的双终端可视化流程。
8. 操作、安全与验证：仅保留主算法日常需要的键位、MuJoCo 边界说明和测试命令。
9. 历史入口：链接 `docs/legacy_algorithms.md`，不在 README 展开旧算法。

主算法数据流写为：

```text
PICO TJVR v4
→ SPARK 两阶段 qpOASES IK
→ 关节速度与笛卡尔前馈
→ Headroom 余量衰减
→ Velocity QP
→ 输出连续性与 settled-hold
→ MuJoCo model_reference
```

## 历史文档迁移

创建 `docs/legacy_algorithms.md`，集中保存：

- hierarchical constrained QP 与 null-space DLS；
- Cartesian OTG velocity/acceleration profile；
- 旧 SPARK guided/direct/pose/fixed-feedforward A/B 模式；
- 历史 QP 决策变量、目标函数、约束和积分公式；
- 历史 benchmark、headless 验收命令和验证文档索引；
- 与旧交互模式相关的完整键位说明。

不把当前 Headroom 主算法的实时启动、显式启动和 TJVR 回放说明复制进 legacy 文档。
legacy 文档开头明确这些内容仅用于回归、对照和历史理解，当前推荐入口始终是根 README。

## 保留与删除规则

- 所有当前主线命令必须可直接复制运行。
- 保留“只用于 MuJoCo、不能直接用于实机”的安全声明。
- 保留无 PICO 输入时的 stale/hold 行为、UDP 端口冲突提示和 `/tmp` 回放遥测说明。
- README 删除长篇通用 QP 数学推导、旧 profile 命令、旧 A/B 评价和重复说明。
- 不修改任何代码、配置、算法参数、benchmark 数据或测试行为。

## 验证

- README 为 150–200 行，并且主算法全名清晰出现于默认值、显式命令和回放说明。
- README 不再展开 `hierarchical_qp`、`nullspace_dls` 或 Cartesian OTG 算法细节。
- `docs/legacy_algorithms.md` 包含上述历史主题，并链接回根 README。
- 所有 Markdown 相对链接存在，命令中的模型、配置、轨迹和 replay 工具路径存在。
- `git diff --check` 通过，改动仅限 README、legacy 文档、设计和实施计划。
