# Tianji PICO 独立停止脚本设计

## 目标

提供 `./scripts/stop_tianji_pico_teleop.sh`，让用户无需记忆 `--stop` 参数即可停止一键遥操链路。

## 行为边界

- 停止脚本将参数转交给同目录的 `start_tianji_pico_teleop.sh --stop`。
- 只停止启动器管理的 `pico_tianji_teleop` tmux session 及其窗口内进程。
- 不使用进程名匹配或 `pkill`，不影响其他 ROS 2 工程。
- 不关闭 PICO APK，不删除 ADB 端口转发。
- 可从任意当前目录调用；脚本通过自身路径定位启动器。
- 重复运行保持幂等，由现有启动器报告 session 已停止。

## 验证

- 契约测试检查独立停止脚本存在、可执行，并使用 `exec` 调用受管启动器的 `--stop`。
- 使用假 tmux 环境验证停止存在的 session 和重复停止行为。
- 运行 Bash 语法检查及完整脚本测试。
