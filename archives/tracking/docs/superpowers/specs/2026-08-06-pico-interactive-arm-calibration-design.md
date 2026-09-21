# PICO 单臂交互式标定入口设计

## 目标

将 `scripts/calibrate_pico_arm.sh` 作为左右手臂标定的唯一入口，同时支持：

- 任意选择一个单项标定；
- 新操作人员按依赖顺序完成整套标定；
- 非交互式命令调用，便于测试和自动化；
- 默认 `ROS_DOMAIN_ID=120`、`ROS_LOCALHOST_ONLY=1`；
- 左右侧 artifact 独立保存，不镜像、不复用测量结果。

本次同时把同一仓库 `pico_project` 工作树中已有的
`pico_palm_wrist_calibrator.py` 及其测试移植到当前仓库。

## 统一入口

无参数运行：

```bash
./scripts/calibrate_pico_arm.sh
```

顶层菜单：

```text
1) 单项标定
2) 新操作人员完整适配
3) 查看当前标定状态
4) 退出
```

单项标定依次询问侧别和项目：

```text
侧别: left | right
项目: tcp | wrist | geometry
```

用户可任意选择项目，但脚本必须执行依赖检查。

非交互接口：

```bash
./scripts/calibrate_pico_arm.sh left tcp
./scripts/calibrate_pico_arm.sh left wrist
./scripts/calibrate_pico_arm.sh left geometry
./scripts/calibrate_pico_arm.sh left all
```

右侧将 `left` 替换为 `right`。

## 标定项目与依赖

### TCP

执行：

```bash
ros2 run pico_bridge pico_palm_tcp_calibrator \
  --side <side> \
  --output ~/.config/pico_tracker/pico_<side>_palm_tcp.yaml
```

TCP 无 artifact 前置依赖。进程成功退出后，脚本验证输出文件存在、侧别正确、
标定有效，再报告成功。

### 掌心到手腕 pivot

执行移植后的：

```bash
ros2 run pico_bridge pico_palm_wrist_calibrator \
  --side <side> \
  --output ~/.config/pico_tracker/pico_<side>_wrist_pivot.yaml
```

前置条件是该侧 TCP artifact 有效，并且 `/pico/palm_<side>` 正在发布。
标定过程中保持手腕固定，绕掌心航向方向提供足够旋转激励，按空格开始采集。

输出必须满足：

- `valid: true`；
- `side` 与选择一致；
- `transform_convention: wrist_to_palm`；
- 三维偏移有限；
- sample count、RMS 和 condition number 通过标定器 Gate。

### 上臂和前臂骨长

继续调用现有 `pico_arm_geometry_pixi.sh` 和几何标定器。

前置条件：

- 同侧 TCP artifact 有效；
- 同侧 wrist pivot artifact 有效；
- PICO driver、tracking epoch 和原始 SMPL 话题可用。

只有 `pico_<side>_arm_geometry_quick_v3`、`valid: true`、
`candidate_status: accepted` 的候选才能原子安装到：

```text
~/.config/pico_tracker/pico_<side>_arm_geometry.yaml
```

## 新操作人员完整适配

`all` 模式严格执行：

```text
TCP -> wrist pivot -> upper/forearm geometry
```

规则：

1. 每一步完成后重新读取并验证本轮输出 artifact；
2. 当前步骤失败立即停止，禁止进入下一步；
3. 已成功生成的前序 artifact 保留；
4. 后续步骤不得把旧 artifact 当成本轮步骤成功；
5. 左右侧分别运行 `all`，不自动把一侧结果复制到另一侧。

为区分旧文件和本轮结果，脚本在每步启动前记录目标文件状态，并要求成功退出后
目标文件的内容摘要或修改状态发生变化，随后再执行 schema 验证。

## 查看当前状态

状态页分别显示左右侧三项 artifact：

- 文件路径；
- missing / invalid / valid；
- side、schema/artifact type、calibration revision（若存在）；
- geometry 是否与当前 TCP、wrist artifact lineage/hash 匹配。

状态查看只读，不自动运行标定，也不修改 artifact。

## 腕部标定器移植

移植范围：

- `src/pico_bridge/scripts/pico_palm_wrist_calibrator.py`；
- `src/pico_bridge/test/test_pico_palm_wrist_calibrator.py`；
- CMake 安装和 pytest 注册。

算法保持原实现语义：固定腕部，使用多姿态掌心位姿求解

```text
p_palm = p_wrist + R_palm * r_wrist_to_palm
```

默认 `motion=yaw`，将掌面法向分量固定为零；第一版不改写求解算法，只处理当前分支
的安装、运行环境、输出验证和交互入口集成。

## 进程与错误处理

- 脚本只在当前 PICO 仓库的 `pixi shell` 中运行；
- 所有子进程继承 domain 120 和 localhost-only 默认值；
- 不启动或停止外骨骼程序；
- 不删除或修改 `recordings/` 中的既有数据；
- Ctrl-C 只终止当前标定步骤；
- 缺少依赖时给出具体路径和下一条可执行命令；
- 单项标定失败后返回非零，但仍允许用户重新运行并选择其他项目。

## 验证

必须覆盖：

1. 无参数菜单选择 left/right 和三个项目；
2. 非交互 `left|right` 与 `tcp|wrist|geometry|all` 参数解析；
3. wrist 在缺少 TCP 时 fail closed；
4. geometry 在缺少 TCP 或 wrist 时 fail closed；
5. `all` 严格按 TCP、wrist、geometry 顺序执行；
6. 某一步失败后后续步骤不执行；
7. 左右输出路径独立；
8. status 模式只读；
9. 移植腕部求解器的 yaw/full 数学测试；
10. Bash 语法、PICO 构建和完整 `pico_bridge` 回归。

## 非目标

- 不修改 TCP、骨长求解数学；
- 不将左右测量结果互相镜像；
- 不把该入口扩展成 PICO 驱动或 M0 的总启动器；
- 不修改外骨骼仓库；
- 不提交代码。
