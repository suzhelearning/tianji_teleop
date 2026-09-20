# 共享根历史归档（2026-09）

本目录保留历史证据，不作为当前启动指令或最终验收结论。旧绝对路径、build-ceres、
文件指纹、测试数字和“尚未实现”均属于当时版本，不能套用到当前工程。
本次只整理文档；没有重新运行历史试验，也没有删除原始录制或 JSON 数值。

## 当前入口

- [TCP/身高标定与 Pixi 输入](../../../../docs/pico-simple-calibration.md)
- [输入契约](../../shared_root_input_contract.md)
- [映射设计与实现边界](../../spark_shared_root_retarget.md)
- [交互仿真与验收清单](../../verification/ceres_interactive_sim.md)
- [后端、限位与移植差异](../../verification/shared_root_ceres_f615b8c.md)
- [DLS 核对与三后端对照](../../verification/shared_root_three_way_20260919.md)
- [动作采集](../../verification/shared_root_action_capture.md) / [动作统计](../../verification/shared_root_action_coverage.md)

## 已合并主结论的原报告（11 份）

阶段、几何、接线和统计口径合并到映射设计；当前验收边界合并到交互仿真；
源模型限位、后端接线与安全差异合并到后端说明。下面原文完整保留，避免丢失旧版本证据。

- [shared_root_ceres_limits_20260919.md](shared_root_ceres_limits_20260919.md)
- [shared_root_ceres_port_20260918.md](shared_root_ceres_port_20260918.md)
- [shared_root_phase_a_followup_20260918.md](shared_root_phase_a_followup_20260918.md)
- [shared_root_phase_a_guidance.md](shared_root_phase_a_guidance.md)
- [shared_root_phase_a_stage1.md](shared_root_phase_a_stage1.md)
- [shared_root_r3_coverage.md](shared_root_r3_coverage.md)
- [shared_root_r3_geometry.md](shared_root_r3_geometry.md)
- [shared_root_r3_handoff.md](shared_root_r3_handoff.md)
- [shared_root_r3_wiring.md](shared_root_r3_wiring.md)
- [shared_root_spark_source_limits.md](shared_root_spark_source_limits.md)
- [spark_shared_root_results.md](spark_shared_root_results.md)

## 历史实验与数据（12 份 Markdown、4 份 JSON）

- [shared_root_actions_20260918.md](shared_root_actions_20260918.md)
- [shared_root_ceres_limits_20260919.json](shared_root_ceres_limits_20260919.json)
- [shared_root_ceres_port_20260918.json](shared_root_ceres_port_20260918.json)
- [shared_root_ceres_trial_20260918.json](shared_root_ceres_trial_20260918.json)
- [shared_root_ceres_trial_20260918.md](shared_root_ceres_trial_20260918.md)
- [shared_root_layers_20260918.md](shared_root_layers_20260918.md)
- [shared_root_mapping_transitions_20260918.md](shared_root_mapping_transitions_20260918.md)
- [shared_root_multiseed_20260918.md](shared_root_multiseed_20260918.md)
- [shared_root_palm_priority_trial.md](shared_root_palm_priority_trial.md)
- [shared_root_reachable_projection.md](shared_root_reachable_projection.md)
- [shared_root_reference_replay_20260918.md](shared_root_reference_replay_20260918.md)
- [shared_root_scale_regression_20260918.md](shared_root_scale_regression_20260918.md)
- [shared_root_tcp1615_mapping.md](shared_root_tcp1615_mapping.md)
- [shared_root_tracking_tuning_20260918.json](shared_root_tracking_tuning_20260918.json)
- [shared_root_tracking_tuning_20260918.md](shared_root_tracking_tuning_20260918.md)
- [shared_root_zhoujie_trace_20260918.md](shared_root_zhoujie_trace_20260918.md)

## 简化标定的早期阶段

- [候选阶段历史记录](pico-simple-calibration-history.md)

移动后已修正文档相对链接；各轮证据保持独立，不将旧 TCP、旧模型或不同统计分母混合比较。
