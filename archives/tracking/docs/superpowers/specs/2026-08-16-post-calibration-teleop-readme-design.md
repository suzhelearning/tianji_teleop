# 标定后天机 PICO 遥操指令更新设计

## 目标

让用户完成左右侧 `TCP → 掌心到手腕 → 上臂/前臂骨长` 标定后，能在快速启动章节立即找到天机 PICO 遥操的一键启动和停止命令，不再把 PICO 单独验证命令误认为完整遥操链路。

## README 结构

- 同步更新 `README.md` 和 `README.zh-CN.md`。
- 在统一标定菜单和状态检查之后，将运行指令明确分成两种用途：
  - PICO 单独验证：`./scripts/start_pico_m0.sh --viewer`
  - 天机 PICO 遥操：`./scripts/start_tianji_pico_teleop.sh`
- 天机遥操入口说明一键脚本自动启动 PICO driver、M0 修正骨架及其 MuJoCo Viewer、ROS 2 → TJVR bridge，并在启动前清理历史链路实例。
- 在同一处给出 `./scripts/stop_tianji_pico_teleop.sh`。
- 保留后续详细的 Tianji Spark 遥操章节作为运行状态、tmux 和 DLS/Spark 接收端说明，不重复长启动命令。

## 验证

- 检查两个 README 都包含标定顺序、一键启动和独立停止命令。
- 检查快速启动段落明确区分 PICO-only 与 Tianji teleoperation。
- 运行 Markdown 差异检查，确保没有尾随空格或格式错误。
