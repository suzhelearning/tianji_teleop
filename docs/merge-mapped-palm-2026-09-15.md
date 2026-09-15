# mapped-palm 工作区合并记录（2026-09-15）

## 来源与保留范围

来源为 `PICO_Hand_Tracking/tianji_teleop_full/tianji_teleop` 的工作区，包含尚未提交的
mapped-palm 迁移及后续修复；共同 Git 基线为 `bbf192b`。
目标合并前 HEAD 为 `d9dc305`（原生外骨骼输入），工作区干净。
本次为文件级合并，未创建提交、未暂存、未 push；来源文件和正在运行的来源仿真未修改。

- 合入完整 mapped-palm C++ 源码、模型/TCP、配置、构建脚本和测试。
- 合入仿真 direct 显示、C 标定、H/Home、手动 R 和受控 S 恢复。
- 合入真机后端选择、C/Enter 标定锁定、反馈静止判定、有界重同步显式选项和 Viewer 清理异常报告。
- 合入骨长对称化工具、标定接线、源端 ROS 环境修正，以及来源的录制压缩改进。
- README 按共同基线人工处理冲突：保留目标原生外骨骼入口、环境与 TJH2 直发说明，
  不恢复来源已过时的 JSON 外骨骼桥接。
- 目标 `exoskeleton/`、`exo.sh`、`tianji/`、人员档案、厂商 SDK 和既有手部协议保持不变。
  真机配置仅新增 `staged_motion.settle_speed_rad_s=0.03`，IP、Home、限位和手部参数未覆盖。

不复制 `.git`、来源虚拟环境/构建缓存、现场日志、录制数据、来源 `profiles/zj`、
LFS 展开的 Manus 二进制/示例数据及旧 `bridge.sh`/JSON 外骨骼适配器。
原生后端是在目标本地重新构建，不借用来源可执行文件。

## 本目录验证

以下结果来自本次合并后的目标目录，而不是来源历史测试：

| 命令 | 结果 |
| --- | --- |
| `bash scripts/build_mapped_palm.sh` | 编译成功，模型 CTest 1/1 通过 |
| `.pixi/envs/default/bin/python -m unittest discover -s real_robot/tests -q` | 170 项通过，无真实驱动调用 |
| `.pixi/envs/default/bin/python -m unittest discover -s sim/tests -q` | 15 项通过 |
| `.pixi/envs/default/bin/python -m unittest discover -s control/mapped_palm/tests -q` | 14 项通过，1 项跳过 |
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest tests -q` | 60 项通过 |
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .pixi/envs/default/bin/python -m pytest data_collection/tests/test_compress.py data_collection/tests/test_recording_keys.py -q` | 17 项通过 |
| `git diff --check` | 通过 |

跳过项为需要显式 `MAPPED_PALM_REFERENCE_ROOT` 的参考工程测试；未重新执行完整历史 TJVR 数值对照。
pytest 关闭自动插件加载，避免宿主 ROS 的 Python 3.10 插件混入 Python 3.11 测试环境。
构建完成前首次真机 mock 因缺少可执行文件失败，完成构建后上述 170 项全部重跑通过。

## 环境与现场边界

本次创建目标 Pixi 默认环境，安装 MuJoCo 3.10.0、PyYAML、pytest 8.4.2、h5py 和 OpenCV
用于验证。运行时可显式设置 `TIANJI_PYTHON="$PWD/.pixi/envs/default/bin/python"`；
未创建或覆盖 `.venv`，未安装目标的 ROS tracking 或外骨骼独立环境，未替换厂商 SDK。
Python 附加包按本地测试环境安装，没有新增跨工程绝对路径运行依赖；完整安装流程仍见 README。

本次没有启动任何现场会话、操作 ADB、连接硬件或发送真机命令。
来源目录的 PICO＋VR 双臂真机成功记录不能替代本目录的现场验收；切换目录前须结束旧执行器，
核对目标人员档案和依赖，再按只读预检、C 标定、多次 Enter 流程测试。
详见 [真机操作及边界](mapped-palm-real-readiness.md) 和 [构建与仿真入口](mapped-palm-port.md)。
