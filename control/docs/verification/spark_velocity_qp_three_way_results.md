# Spark-guided velocity-QP comparison

All tracking statistics use live, non-stale PICO samples.

| run | algorithm | L pos P95 (mm) | R pos P95 (mm) | L rot P95 (rad) | R rot P95 (rad) | posture active L/R | IK accepted L/R | cycle P99 (us) | failures | deadline misses |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| guided | spark_guided_velocity_qp | 250.81 | 204.37 | 0.2684 | 0.1809 | 1.000/1.000 | 1.000/1.000 | 695.54 | 0 | 0 |
| direct | spark_direct_velocity_qp | 324.79 | 267.46 | 0.2540 | 0.2086 | 1.000/1.000 | 1.000/1.000 | 674.58 | 0 | 0 |
| pose_only | spark_pose_velocity_qp | 242.90 | 257.09 | 0.2125 | 0.1933 | 0.000/0.000 | 0.000/0.000 | 585.57 | 0 | 0 |

## Joint-reference hard-bound audit

| run | arm | q violations | qdot violations | qddot violations | jerk violations | max |qdot| | max |qddot| | max |jerk| |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| guided | left | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |
| guided | right | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |

guided joint telemetry resets: 2.

| direct | left | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |
| direct | right | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |

direct joint telemetry resets: 2.

| pose_only | left | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |
| pose_only | right | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |

pose_only joint telemetry resets: 2.

## Interpretation

- Removing the seven-joint Ruckig stage did not improve Cartesian tracking: the
  direct Spark posture guide increased position P95 on both arms. Ruckig is
  therefore not the sole cause of the previously observed lag or poor response.
- Pose-only has the lowest cycle P99 and the best left-arm position/orientation
  P95 in this trace, but its right-arm position P95 is worse than guided mode.
  It also deliberately provides no Spark joint-posture guidance, so Cartesian
  tracking alone selects the redundant elbow configuration.
- All three modes reached the configured dynamic bounds without violating them.
  Reachability, target motion, and the shared hard bounds remain important
  contributors to the residual Cartesian error.

## MuJoCo visual replay

The direct and pose-only modes were replayed sequentially with all 2134 packets
from `pico_fast_motion_20260812_205428_v4.tjvr`. Both replays completed without
solver failures or deadline misses. The Viewer simultaneously rendered:

- corrected PICO skeleton: cyan/magenta;
- Spark robot-length skeleton: orange/green;
- Spark shoulder bridge: yellow.

Visual-run telemetry is stored beside the deterministic comparison as
`spark_direct_velocity_qp_visual*.csv` and
`spark_pose_velocity_qp_visual*.csv`.
