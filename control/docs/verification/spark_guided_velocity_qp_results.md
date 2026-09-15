# Spark-guided velocity-QP comparison

All tracking statistics use live, non-stale PICO samples.

| run | L pos P95 (mm) | R pos P95 (mm) | L rot P95 (rad) | R rot P95 (rad) | cycle P99 (us) | failures | deadline misses |
|---|---:|---:|---:|---:|---:|---:|---:|
| velocity_qp_baseline | 305.32 | 240.17 | 0.4276 | 0.3430 | 601.74 | 0 | 0 |
| spark_no_hard_jerk | 154.11 | 158.47 | 0.2050 | 0.1860 | 646.09 | 0 | 0 |
| spark_hard_jerk | 250.00 | 294.08 | 0.2010 | 0.2688 | 713.45 | 0 | 0 |

## Joint-reference hard-bound audit

| arm | q violations | qdot violations | qddot violations | jerk violations | max |qdot| | max |qddot| | max |jerk| |
|---|---:|---:|---:|---:|---:|---:|---:|
| left | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |
| right | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |

Joint telemetry resets: 2.

## Test setup

- Source: `vr_data/converted_inputs/tjvr/pico_fast_motion_20260812_205428_v4.tjvr`
- Input: 2134 atomic TJVR v4 frames, 656 bytes/frame, 24.081 s
- Controller: 200 Hz, model-state feedback, velocity-level QP
- Hard limits audited from the command reference: joint position, velocity,
  acceleration and jerk
- Spark hard-jerk limits were the existing configured values; no limit value
  was raised or lowered.

The final hard-jerk run completed with zero control failures and zero deadline
misses. All logged reference samples stayed inside all four reported joint
bounds. The higher position error relative to the no-hard-jerk Spark run is a
real bandwidth trade-off: at 5 ms, the configured jerk limits permit only
5 rad/s² (J1-J3) or 7.5 rad/s² (J4-J7) acceleration change per control sample.
It should not be hidden by retuning QP tracking weights.

The old baseline and Spark route do not use identical Cartesian target
construction: the baseline consumes the legacy relative palm mapping, whereas
Spark reconstructs the robot-length upper limb. Their comparison is therefore
an end-to-end same-input A/B, not a pure solver-only benchmark.
