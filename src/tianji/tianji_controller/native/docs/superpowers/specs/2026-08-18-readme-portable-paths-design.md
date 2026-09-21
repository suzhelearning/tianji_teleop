# README 可移植路径设计

## 背景

当前根 `README.md` 的构建、PICO 实时遥操和 TJVR 回放示例包含
`/home/zj/current_robotics/...` 本机绝对路径。其他开发者将仓库克隆到不同目录后，
无法直接复用这些命令。MuJoCo 回放遥测还写入 `/tmp`，不利于将结果与仓库内现有
benchmark 目录统一管理。

## 目标

- 根 README 不包含绑定到某个用户或工作站的绝对路径。
- 仓库内命令默认从 `TJ_arm_control` 仓库根目录执行。
- 外部仓库或工具路径使用清晰的可替换占位符。
- 回放输出写入仓库内既有的 `benchmark_results/pico_live/`。
- 自动检查 README，防止本机 `/home/...` 路径再次进入文档。

## 方案

1. 删除进入 `TJ_arm_control` 本机目录的 `cd /home/zj/...` 命令，在相关章节开头明确
   说明命令应从仓库根目录运行。
2. 将 PICO Tracker 路径表示为 `<PICO_tracker目录>`，将外部 TJVR 回放工具表示为
   `<vr_data目录>/tools/replay_pico_udp_trace.py`。占位符表达依赖关系，但不假设三个
   仓库的安装位置或相对布局。
3. 将 `/tmp/pico_headroom_main_replay*.csv` 改为
   `benchmark_results/pico_live/pico_headroom_main_replay*.csv`，并在命令前创建输出目录。
4. 增加轻量文档回归测试，扫描根 README 中的 `/home/`、`/Users/` 等用户目录前缀；
   合法的命令参数、相对路径和尖括号占位符不受影响。

## 验证

- 先运行路径扫描并确认旧 README 会因 `/home/zj/...` 失败。
- 修改后再次运行，确认根 README 不再包含机器绑定路径。
- 检查所有仓库内相对路径真实存在。
- 运行文档回归测试以及完整 CTest，确保 Viewer 和已有测试不受文档修改影响。

## 提交边界

设计规格单独提交。实施提交只包含 README 路径修复及对应文档回归测试，不纳入
`benchmark_results/` 下未跟踪的运行数据。
