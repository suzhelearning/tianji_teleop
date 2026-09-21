# Spark Direct / Pose Velocity-QP A/B Design

## Objective

Add two isolated PICO replay modes that identify whether poor response in
`spark_guided_velocity_qp` comes from the seven-axis Ruckig posture reference
or from the Spark-reconstructed Cartesian target. Preserve every existing
algorithm and all current joint-limit values.

## Algorithms

### `spark_direct_velocity_qp`

```text
PICO corrected skeleton (TJVR v4)
  -> Spark robot-length skeleton scaling
  -> Spark two-stage position IK: q_ik
  -> qdot_posture = clamp(K * wrap(q_ik - q_model))
  -> soft posture objective in the existing velocity-level QP

Spark scaled palm pose
  -> existing Cartesian OTG
  -> existing Cartesian velocity servo
  -> same velocity-level QP Cartesian task
```

This route bypasses `SparkPostureReference7` and its seven-axis Ruckig. The
Spark joint solution remains a weak redundancy guide; it is never integrated
or sent directly to the model. The final QP retains the current position,
velocity, acceleration, jerk, braking and outward constraints.

### `spark_pose_velocity_qp`

```text
PICO corrected skeleton (TJVR v4)
  -> Spark robot-length skeleton scaling
  -> scaled left/right palm pose only
  -> existing Cartesian OTG
  -> existing Cartesian velocity servo
  -> existing velocity-level QP
```

This route does not run Spark position IK and does not inject a Spark posture
task. Redundancy is selected by the existing QP regularization/continuity terms
and the `outward_only` hard barrier. Skipping position IK also establishes the
lowest-compute Spark-pose baseline.

The existing `spark_guided_velocity_qp` remains unchanged as the third A/B
member: Spark position IK -> seven-axis Ruckig -> soft posture QP objective.

## Isolation and configuration

Add explicit `IkAlgorithm` values and CLI parsing for both modes. Reuse the
current Spark scaler, Cartesian target blending, stale/epoch reset logic and
velocity-QP backend. Add no new global control profile and change no existing
limit value. Spark-specific hard jerk enablement applies identically to all
three Spark modes so the A/B changes only posture-reference construction.

The direct posture gain and maximum magnitude use the existing
`spark_upper_qpoases.posture_position_gain` and the current joint velocity
bounds. No second limiter is added.

## Runtime behavior

- A valid v4 frame updates both arms atomically.
- `spark_direct_velocity_qp` accepts a posture task only when both Spark IK
  results are accepted; otherwise it keeps the previous valid IK guide while
  Cartesian tracking continues.
- `spark_pose_velocity_qp` needs only valid scaled palm targets and therefore
  cannot fail because of Spark position IK.
- PICO timeout holds the Cartesian target and disables direct posture motion.
- Tracking-epoch changes reset scaling/blending and synchronize all state to
  the current model state.
- Both modes force `outward_only` and remain MuJoCo/model-state-only in the
  initial evaluation.

## Telemetry

Record the selected algorithm, Spark target validity, IK activity/acceptance,
direct posture velocity, q/qdot/qddot/jerk and all effective bounds. In the
pose-only mode, Spark IK diagnostics remain explicitly inactive/zero rather
than containing stale values.

When `--pico-skeleton-overlay` is enabled, render both the corrected PICO
source skeleton and the robot-length Spark-scaled skeleton. Use distinct colors
and derive the Spark overlay from the same blended `SparkUpperTargets` consumed
by control. All three Spark modes show the scaled skeleton, including pose-only.

## Verification

1. Unit tests prove direct mode computes the configured proportional posture
   velocity without advancing Ruckig.
2. Unit tests prove pose-only mode emits no posture task and does not invoke
   Spark position IK.
3. Existing `spark_guided_velocity_qp` tests remain unchanged.
4. Replay the same 2134-frame TJVR v4 fast-motion trace for all three modes.
5. Require zero malformed packets, control failures and hard-bound violations.
6. Compare Cartesian P50/P95/P99, cycle P99, Spark IK acceptance, joint
   velocity/acceleration/jerk and visible shoulder/elbow branch stability.
7. Run each new mode graphically in MuJoCo with the PICO skeleton overlay.

## Acceptance

The change is accepted when both new modes are independently selectable, the
full regression suite passes, deterministic replay has no control failure or
joint-bound violation, and the two graphical replays can be run from the same
trace without changing configuration files.
