# Tianji PICO 一键启动默认 M0 Viewer 设计

## 目标

`./scripts/start_tianji_pico_teleop.sh` 启动 PICO 侧遥操链路时，默认同时显示 M0 的 PICO 骨架 MuJoCo Viewer，方便检查原始骨架和掌心修正后的 IK-frame 骨架。

## 行为

- `m0` tmux 窗口使用 `./scripts/start_pico_m0.sh --viewer`。
- Viewer 主骨架继续订阅 `/pico/smpl_palm_corrected_ik`，并叠加 `/pico/smpl_raw`；不改变现有骨架算法、话题或天机 UDP bridge。
- Viewer 进程异常退出时，M0 filter 继续运行，沿用 `start_pico_m0_pixi.sh` 已有的故障隔离行为。
- 一键脚本仍不启动 `TJ_arm_control_DLS_IK` 的 DLS/Spark 机械臂 Viewer。
- README.md 和 README.zh-CN.md 明确区分默认启动的 PICO M0 骨架 Viewer 与未启动的接收端 DLS/Spark Viewer。

## 验证

- 脚本契约测试要求一键启动器的 M0 命令包含 `--viewer`。
- 先运行该测试并确认旧实现失败，再修改启动脚本使测试通过。
- 运行 `test_pico_m0_scripts.py`，确认现有 M0、tmux 和 Viewer 行为无回归。
- 使用 shell 语法检查验证启动脚本。
