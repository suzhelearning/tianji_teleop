# 今日工作总结 | 2026-07-21

## 今日完成

### PICO + 双 IMU900

- 完成参考 `imu_ros2` 驱动移植和 C++ 脚部融合，左脚使用 `/dev/ttyUSB0`，右脚使用 `/dev/ttyUSB1`。
- 完成 A 键自动标定：左右 IMU900 顺序 zero-Z、等待 1 秒并采集 60 帧。
- 修复 ACK 超时、串口占用、脚部坐标轴/左右镜像和融合骨架不发布问题。
- MuJoCo 支持原始/融合骨架叠加；融合结果为 `/pico/smpl_fused`，原始结果为 `/pico/smpl`。

### Odin Lite

- 完成 Odin Lite 驱动迁移、连接测试和时间戳对齐，高频里程计实测约 400.6 Hz。
- 昨日 Odin 异常重新定位为供电导致设备失连。
- 本次 9 轴 IMU 复测发现躯干磁干扰严重，约 10 分钟 yaw 漂移 90°，需要电子硬件排查高干扰源。

### PNP / DynaIP

- 完成 RTX 5080 推理适配、三模型同屏比较和 PL-A/PL-S1 接口诊断。
- 完成 V43–V45 评估；PNP 整体更接近真值，当前集中进行 PNP 接口复现和 V46 验证。

### 文档和代码

- README 更新为中英文双语，补充 PICO、IMU900、标定和 MuJoCo 用法，并移除开发机绝对路径。
- `feature/pico-foot-imu-fusion` 已合并到 `main`，合并提交为 `7b42e37`，feature 分支已删除。

## 关键结论

- `/pico/smpl` 无数据通常是 PICO body tracking 未运行，无法形成有效的 24 点数据。
- IMU900 报 `serial port already in use` 时，需要先停止其他 IMU 节点。
- 标定问题已通过顺序 ACK、稳定等待和 fresh 样本采集修复。
- 当前躯干 yaw 漂移主要判断为磁干扰，优先进行硬件排查，不再盲目调整软件参数。

## 验证结果

- 当前工程：7 个包构建成功，73 项测试通过，0 错误、0 失败。
- PNP 项目：92 项测试通过。
- `main` 与 `personal/main` 已同步于 `7b42e37`。

## 下一步

- 验证双 IMU900 磁场日志和磁场稳定性。
- 排查躯干 IMU 的电子抗磁问题。
- 完成 Odin Lite 完整标定与 Full-body QP 联合测试。
- 完成 V46 的 DIP 因果闭环评估。
