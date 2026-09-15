# SPARK joint-reference-dominant velocity QP verification

The deterministic replay used all 2134 packets from
`pico_fast_motion_20260812_205428_v4.tjvr` (24.081 s). Direct position IK was
kept unchanged. The experimental velocity-QP route bypassed Cartesian OTG and
used the same two-stage position-IK result for both objectives:

```text
qdot_goal = clamp(Kq * (q_ik - q_model), joint velocity limits)
V_des = J(q_model) * qdot_goal + CartesianPoseFeedback
```

The QP joint-reference weight was `3e4`. Existing position, velocity,
acceleration, jerk and braking bounds were not changed.

## Raw SPARK/PICO target tracking

These values compare the unfiltered target pose against the MuJoCo arm pose.
This is the fair input-to-output comparison: the older route's ordinary
telemetry error is measured against its delayed OTG reference instead.

| run | L position P95 | R position P95 | L orientation P95 | R orientation P95 |
|---|---:|---:|---:|---:|
| Direct position IK | 1.33 mm | 11.10 mm | 0.0004 rad | 0.0416 rad |
| Previous Cartesian-primary velocity QP | 554.88 mm | 500.50 mm | 1.6611 rad | 1.7894 rad |
| Joint-reference-dominant velocity QP | 308.84 mm | 278.84 mm | 1.0390 rad | 1.1371 rad |

The new route reduces raw-target lag substantially and keeps the arm closer to
the two-stage IK branch. It cannot reproduce Direct mode instantaneously
because Direct requires joint peaks far beyond the configured dynamic limits.

## Hard-bound audit

The joint-reference velocity QP completed with zero control failures and zero
deadline misses. Both arms had zero position, velocity, acceleration and jerk
bound violations. Observed maxima were `4 rad/s`, `90 rad/s^2` and
`1500 rad/s^3`, exactly within the unchanged configured envelopes.

Direct remains a visualization-only posture baseline: on this trace it reached
approximately `160 rad/s`, `3.19e4 rad/s^2` and `1.26e7 rad/s^3`.
