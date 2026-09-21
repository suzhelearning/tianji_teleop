# SPARK Feedforward Velocity-QP Output Continuity Design

## Problem

Live PICO telemetry shows that the SPARK feedforward joint reference is
substantially smoother than the final velocity-QP command.  Even away from
joint limits, the final command repeatedly reaches the hard jerk envelope.
The current SPARK posture objective has a joint-reference weight of `3.0e4`
but only `3.0e3` of velocity continuity.  Its only acceleration-trend
predictor is the global hierarchical-QP jerk weight of `2.0e-2`, which is too
small to influence this mode.

Fixed-trace experiments confirmed that input filtering and reducing the
SPARK position gain do not address the amplification.  Raising only the
SPARK velocity-continuity weight from `3.0e3` to `3.0e4` reduced left/right
hard-jerk saturation from `49.1%/39.9%` to `29.8%/24.4%` without increasing
mean Cartesian position error.  Combining that continuity weight with a
`3.0e4` acceleration-trend predictor reduced saturation further to
`23.8%/19.4%`, with mean position error changing from `115.4/78.9 mm` to
`115.7/78.7 mm` on the same trace.

## Scope

The change applies only to SPARK joint-reference posture tasks used by
`spark_upper_qpoases_headroom_feedforward_velocity_qp`.  It does not change
the QP decision variable, Cartesian task, task scaling, SPARK IK, collision
handling, hard joint position/velocity/acceleration/jerk/braking bounds, or
other algorithms.

## Objective

For an active SPARK feedforward posture task, augment the existing objective
with two independent continuity terms:

```text
0.5 * w_reference * ||qdot - qdot_spark||^2
+ 0.5 * w_velocity * ||qdot - qdot_previous||^2
+ 0.5 * w_jerk *
      ||qdot - (qdot_previous + qddot_previous * dt)||^2
```

`w_velocity` rejects rapid acceleration demand.  `w_jerk` preserves the
previous acceleration trend and therefore reduces acceleration reversals.
Both remain soft objectives; the existing hard feasible set remains final.

## Interfaces and defaults

Add `jerk_smoothness_weight` to `JointVelocityPostureTask`, defaulting to
zero so existing producers are unchanged.  Add the two source-specific
continuity weights to `SparkHeadroomFeedforwardVelocityQpConfig` and load them
from `spark_headroom_feedforward_velocity_qp`.  Keeping them out of the shared
SPARK configuration prevents the historical fixed-feedforward algorithm from
changing.

The PICO teleoperation profile uses:

```yaml
spark_headroom_feedforward_velocity_qp:
  joint_reference_smoothness_weight: 3.0e4
  joint_reference_jerk_smoothness_weight: 3.0e4
```

The SPARK feedforward guidance producer copies both continuity weights into
the posture task only when headroom-feedforward mode is selected.  Fixed
feedforward retains the shared `3.0e3` velocity-continuity weight and a zero
jerk-continuity weight.  The QP builder applies the jerk term only for
`kSparkFeedforwardJointReference`; other posture sources retain their current
behavior.

## Validation

Unit tests must verify the exact Hessian and gradient contribution, including
the predicted target `qdot_previous + qddot_previous * dt`.  Configuration
tests must verify loading and positive-value validation.  Guidance tests must
verify propagation into the generated posture task.  The complete test suite
must pass, followed by the fixed `pico_fast_motion_20260812_205428_v4.tjvr`
replay.  Acceptance requires zero control failures and hard-bound violations,
lower jerk saturation than the current configuration, and no material mean
Cartesian tracking regression.

No changes are committed unless explicitly requested by the user.
