# Tianji PICO 独占式一键启动设计

## 目标

`./scripts/start_tianji_pico_teleop.sh` 每次正常启动时先清理历史 PICO driver、M0 和 Tianji bridge，再建立唯一的新链路，避免不同工作目录或 ROS domain 的残留进程重复发布或重复发送 UDP。

## 启动顺序

1. 保留 `--status`、`--stop` 和 `--help` 的现有非启动行为。
2. 正常启动或 `--detach` 先验证 Pixi、tmux、ADB 设备和已构建 workspace；硬件预检失败时不破坏当前链路。
3. 若受管 `pico_tianji_teleop` tmux session 存在，先终止该 session。
4. 扫描 `/proc`，按精确可执行文件名和 ROS launch 文件名识别三类历史链路，不限制工作目录或 `ROS_DOMAIN_ID`：
   - driver：`start_pico_bridge.launch.py`、`pico_bridge_node`、`pico_smpl_ground`
   - M0：`start_pico_palm_skeleton_filter.launch.py`、`pico_palm_tcp_publisher`、`pico_palm_skeleton_filter`、`smpl_mujoco_visualizer`
   - bridge：`start_tianji_mujoco_teleop.launch.py`、`tianji_mujoco_teleop_bridge`
5. 对目标进程依次发送 SIGINT、SIGTERM、SIGKILL，并在各阶段等待退出；排除清理器自身及其祖先进程。
6. 仍有目标存活时启动失败；全部清理后创建新的 driver、M0 Viewer 和 bridge tmux 窗口。

## 安全边界

- 不使用模糊 `pkill -f`，不按通用 `ros2`、`python` 或路径前缀杀进程。
- 不关闭 PICO APK，不删除 ADB 转发，不影响其他 ROS 节点。
- 历史目标包含旧工作目录和 ROS domain 99/120。
- `stop_tianji_pico_teleop.sh` 继续只停止受管 tmux session。

## 验证

- 单元测试使用临时 `/proc` 数据验证精确识别、祖先进程排除和非目标保留。
- 启动器假环境测试验证已有 session 会被停止并重新创建，而不是 attach 或跳过启动。
- 测试清理失败时 fail closed。
- 运行完整脚本测试、Python/Bash 语法检查和差异检查。
