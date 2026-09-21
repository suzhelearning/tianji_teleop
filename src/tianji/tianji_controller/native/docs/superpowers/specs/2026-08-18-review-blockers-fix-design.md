# 推送前审查阻塞项修复设计

## 背景

对 `origin/main...HEAD` 的 Standards 与 Spec 双轴审查发现：README 将本地 TJVR
轨迹写成仓库内可用基准、显式遥测命令未创建输出目录、回放章节缺少模型和状态源说明，
两个默认启用 PICO 的 Viewer 配置测试还会共享 UDP 端口 15000。

用户明确要求不把 6.0 MiB 的 `output_continuity_retest.tjvr` 提交到仓库。因此文档必须
将其表达为用户自行录制或提供的本地资产，不能继续暗示干净克隆已包含该文件。

## README 行为

- “完整显式启动”在写入遥测前运行 `mkdir -p benchmark_results/pico_live`。
- 录制示例同时创建 `benchmark_results/pico_live/traces`，并通过 `--pico-record` 生成
  可重复使用的 TJVR 文件。
- 回放示例设置
  `TRACE_FILE=benchmark_results/pico_live/traces/output_continuity_retest.tjvr`，运行
  `test -f "$TRACE_FILE"` 检查前置资产，再把 `"$TRACE_FILE"` 传给回放工具。
- 回放章节明确该轨迹不随仓库发布，可以由实时遥操录制生成，也可以由测试人员放到
  指定位置。
- 回放章节明确算法、`velocity` 控制层、`model_reference` 状态源和
  `models/marvin_m6_qp_pico_fast.xml` 模型。

这项用户决策取代早期 Headroom README 设计中“命令中的轨迹路径存在”的要求；仓库
仍保证回放命令结构和生成轨迹的流程完整，但不分发实测人员数据。

## 测试隔离

`test_viewer_default_profile`、`test_viewer_explicit_profile` 和
`test_viewer_pico_profile` 都可能启动默认 PICO UDP receiver。三者设置同一个 CTest
`RESOURCE_LOCK pico_udp_15000`，确保任何并行 CTest 调度下不会同时使用端口 15000。
使用资源锁而不是把所有测试设为 `RUN_SERIAL`，使不占用该端口的测试仍可并行执行。

## 回归检查

扩展 README CMake 检查，验证：

- 不出现 `/home/` 或 `/Users/` 本机路径；
- 存在创建遥测目录的命令；
- 存在本地轨迹文件检查和 `--pico-record` 说明；
- 回放章节包含模型与状态源。

对三项 Viewer profile 测试执行并行重复测试，验证资源锁生效。随后运行完整 CTest、
重新执行 Standards 与 Spec 双轴审查。只有不存在推送阻塞项时才推送 `main`。

## 提交边界

实施提交只包含 README、CTest 配置和文档回归检查。所有
`benchmark_results/pico_live/` 内容，包括 `output_continuity_retest.tjvr`，保持未跟踪，
不进入提交或 push。
