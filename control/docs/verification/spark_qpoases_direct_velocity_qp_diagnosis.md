# SPARK two-stage qpOASES direct/velocity-QP diagnosis

All tracking statistics use live, non-stale PICO samples.

| run | algorithm | L pos P95 (mm) | R pos P95 (mm) | L rot P95 (rad) | R rot P95 (rad) | posture active L/R | IK accepted L/R | cycle P99 (us) | failures | deadline misses |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| direct | spark_upper_qpoases_direct | 1.32 | 11.07 | 0.0004 | 0.0415 | 0.000/0.000 | 1.000/1.000 | 353.48 | 0 | 0 |
| velocity_qp | spark_upper_qpoases_velocity_qp | 240.96 | 309.07 | 0.2119 | 0.2008 | 1.000/1.000 | 1.000/1.000 | 653.83 | 0 | 0 |

## Joint-reference hard-bound audit

| run | arm | q violations | qdot violations | qddot violations | jerk violations | max |qdot| | max |qddot| | max |jerk| |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| direct | left | 0 | 33341 | 22248 | 0 | 159.547958 | 31892.581673 | 12633004.640600 |
| direct | right | 0 | 32993 | 19943 | 0 | 137.148612 | 27470.879284 | 10837970.128400 |

direct joint telemetry resets: 2.

| velocity_qp | left | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |
| velocity_qp | right | 0 | 0 | 0 | 0 | 3.141600 | 90.000000 | 1500.000000 |

velocity_qp joint telemetry resets: 2.

## Data-quality check

Both runs contain 5400 control rows and only two reset rows. The discontinuity
statistics below exclude reset rows, so the direct-mode peaks are not reset or
stale-transition artifacts.

| run | arm | metric | P50 | P95 | P99 | maximum |
|---|---|---|---:|---:|---:|---:|
| direct | left | qdot (rad/s) | 0.0014 | 10.0624 | 24.1521 | 159.5480 |
| direct | left | qddot (rad/s^2) | 177.31 | 3047.28 | 6229.81 | 31892.58 |
| direct | left | jerk (rad/s^3) | 100223.82 | 1079603.46 | 2208652.90 | 12633004.64 |
| direct | right | qdot (rad/s) | 0.0005 | 7.7412 | 20.3390 | 137.1486 |
| direct | right | qddot (rad/s^2) | 96.98 | 2441.54 | 5480.47 | 27470.88 |
| direct | right | jerk (rad/s^3) | 53771.04 | 879978.16 | 1955265.96 | 10837970.13 |
| velocity_qp | left | qdot (rad/s) | 1.3491 | 3.1416 | 3.1416 | 3.1416 |
| velocity_qp | left | qddot (rad/s^2) | 16.77 | 60.00 | 82.50 | 90.00 |
| velocity_qp | left | jerk (rad/s^3) | 1000.00 | 1500.00 | 1500.00 | 1500.00 |
| velocity_qp | right | qdot (rad/s) | 1.4926 | 3.1416 | 3.1416 | 3.1416 |
| velocity_qp | right | qddot (rad/s^2) | 21.54 | 62.42 | 82.57 | 90.00 |
| velocity_qp | right | jerk (rad/s^3) | 1000.00 | 1500.00 | 1500.00 | 1500.00 |

## Diagnosis

The two-stage position IK reaches the SPARK palm targets accurately in direct
mode, but its successive redundant joint solutions are not dynamically
continuous. The velocity QP is therefore not the origin of all joint motion:
it receives a rapidly changing posture reference that already contains large
inter-cycle changes.

The velocity QP enforces the configured dynamic bounds exactly, but the same
hard bounds prevent it from following those discontinuous joint/posture and
fast Cartesian requests. The resulting 0.24--0.31 m position P95 is dominated
by saturation and task conflict, rather than a low Cartesian objective weight
alone.

The next technically justified change is to make the two-stage position IK
branch-continuous before tuning the velocity QP: use the previous accepted
`q_ik` as a continuity reference with an explicit `||delta_q||` or
`||q-q_previous||` term and branch/hysteresis handling. Do not send direct mode
to physical hardware.

## MuJoCo visual replay

Both modes were replayed sequentially with all 2134 v4 packets. The Viewer
simultaneously displayed the corrected PICO skeleton and processed SPARK
robot-length skeleton. Both visual sessions completed without protocol or
solver failure; timeout status 124 is the configured automatic Viewer close.
