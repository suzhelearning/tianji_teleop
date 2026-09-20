# 2026-09-20：遥操与远端 main 合并验证

合并本地 `456e5f6` 与远端 `61465ac`，保留两边历史，不强制覆盖远端。

## 冲突处理

- README 保留人员标定、默认 DLS＋Ruckig、Manus／外骨骼与 PICO2 入口。
- 远端共用 Home、Mocap／Regrind 说明移至根目录 `README-mocap.md`；真机流程同步至 `README-reference.md`。
- Pixi 保留 Manus 独立环境和远端六个 Mocap task。后者仍要求另行准备 `.venv`，本轮未迁移其环境。
- 共用配置与 CMake 自动合并后重新构建；不改变 DLS／Ceres 的 Home 配置和授权方式。
- 个人 `profiles/zhoujie`、录制和日志不纳入提交。

## 本轮实际验证

| 命令 | 结果 |
|---|---|
| `pixi run --locked build` | 成功，包含新 `mocap_tcp_worker` |
| `pixi run --locked test-native` | 106/106，170.30 秒 |
| `pixi run --locked test-sim` | 46/46 |
| `pixi run --locked -e tracking test-pico-simple` | 133/133 |
| `pixi run --locked -e tracking test-sim-user` | 66/66 |
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pixi run --locked -e manus python -m pytest manus/tests retargeting/tests/test_native_geometry.py retargeting/tests/test_tj_wuji2_hand_bridge.py -q` | 43/43，hppfcl 弃用警告 |
| 下述 Mocap／真机离线测试 | 286 通过、7 跳过 |
| README 三页本地文件链接、合并 Pixi task 解析、`git diff --check` | 通过 |

Mocap／真机测试最初在 tracking 环境因缺少 SciPy 收集失败；单独运行真机测试为
209 通过、6 项因缺少 SciPy 失败。随后用 tracking Python 建立临时
`venv --system-site-packages`，仅在临时环境安装 SciPy 1.17.1，再执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /tmp/tianji-merge-tests-TJ5C8V/bin/python -m pytest \
  tests/test_h5_motive_replay.py tests/test_mocap_hardware_port.py \
  tests/test_mocap_policy_native.py tests/test_mocap_policy_port.py \
  tests/test_mocap_policy_runtime.py real_robot/tests -q -rs
```

7 项跳过均因未安装 PyTorch；不声称已验证 Regrind 策略推理。
临时路径仅为本轮验证记录，不是运行依赖。未修改现有 Pixi 环境来补装 SciPy。

本轮未启动现场输入或真机，未验证新 clone 完整安装、真实 Motive／PICO 输入、
GPU 推理或硬件闭环。软件回归不能替代这些验收。
