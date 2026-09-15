# SPARK Headroom Velocity-QP Strong Output Damping Design

## Goal

Prioritize removal of visible arm oscillation while accepting a measured
`2--4 mm` increase in mean Cartesian position error.  Preserve commit
`130b2ce` as the rollback point.

## Diagnosis

The live `output_continuity_live` run contains a dominant `3--4 Hz` joint
acceleration oscillation rather than an `8--12 Hz` PICO tremor component.
Left-arm headroom is below `0.01` for `73.5%` of live samples, and its
acceleration bounds are active for 1653 cycles.  When a bound is active, the
left J2/J4 acceleration RMS is amplified by `2.21/1.57` relative to the smooth
SPARK feedforward reference.  This identifies insufficient velocity-QP output
damping under task conflict as the primary cause.

## Change

Change only the headroom-feedforward posture velocity-continuity weight:

```yaml
spark_headroom_feedforward_velocity_qp:
  joint_reference_smoothness_weight: 1.0e5
  joint_reference_jerk_smoothness_weight: 3.0e4
```

The first term penalizes `qdot - qdot_previous`; the second continues to
penalize deviation from `qdot_previous + qddot_previous * dt`.  No source code,
QP decision variable, Cartesian gain, feedforward gain, task scaling, SPARK
IK, collision behavior, or hard position/velocity/acceleration/jerk/braking
bound changes are in scope.  Other algorithms retain their existing weights.

## Evidence and acceptance

The fixed PICO trace with this exact temporary configuration produced:

- left/right jerk saturation `10.5%/8.0%`, down from `23.2%/19.7%`;
- left/right `3--5 Hz` acceleration power reductions of `24%/31%`;
- mean position-error increases of `2.4/3.8 mm`;
- zero control failures, deadline misses, and recorded hard-bound violations.

Implementation is accepted when the profile loads `1.0e5`, all 76 CTest
cases pass, the fixed trace reproduces zero failures and violations, and live
PICO testing shows the visible left-arm oscillation is reduced.  Live visual
acceptance remains necessary because the recorded trace cannot reproduce every
operator pose and limit interaction.
