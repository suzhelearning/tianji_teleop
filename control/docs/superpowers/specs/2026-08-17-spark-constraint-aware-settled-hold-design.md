# SPARK Constraint-Aware Settled Hold Design

## Goal

Eliminate endpoint limit cycles after the operator settles or the PICO stream
becomes stale, without reducing Cartesian gains or changing the velocity-QP
joint position, velocity, acceleration, jerk, braking, outward, or collision
constraints.

## Evidence

The fixed visual replay ends with the left arm oscillating after all SPARK
posture and feedforward commands have decayed to zero. During the final stale
two-second window:

- left reference velocity reaches `4 rad/s`;
- left reference acceleration reaches `90 rad/s^2`;
- left reference jerk reaches `1500 rad/s^3`;
- mean Cartesian position error is about `13 mm`, with a `67 mm` maximum;
- right arm settles normally.

The remaining loop is therefore caused by the Cartesian servo continuing to
chase the last constrained pose, not by the SPARK arm-angle objective.

## Scope

The change applies only to
`spark_upper_qpoases_headroom_feedforward_velocity_qp`. Historical fixed
feedforward, Cartesian OTG, direct velocity-QP, and acceleration-QP algorithms
remain unchanged.

## State Machine

Each arm owns an independent settled-hold state.

```text
TRACKING
  | stale immediately
  | or dual-source stationary + exhausted headroom for 0.30 s
  v
BRAKING_HOLD
  | raw PICO and SPARK target both confirm renewed motion
  v
TRACKING
```

`dual-source stationary` means either the raw corrected palm or the retargeted
SPARK palm is below both existing stationary linear and angular thresholds. A
source is moving when either its linear or angular speed exceeds the matching
threshold. A new command is considered intentional only when both sources are
moving. This preserves the noise rejection used by the SPARK joint-reference
hold while allowing translation-only and rotation-only commands to release.

`exhausted headroom` means the filtered per-arm headroom scale is at or below
`0.05`. It must remain exhausted while the source is stationary for `0.30 s`
before live tracking enters hold. Stale input bypasses the dwell and enters
hold immediately.

## Hold Behavior

While `BRAKING_HOLD` is active:

1. Set the Cartesian reference pose to the current model TCP every cycle.
2. Set Cartesian reference twist to zero.
3. Replace the SPARK posture target with zero joint velocity while retaining
   its existing soft posture weight and smoothness term.
4. Let the existing velocity-QP acceleration and jerk bounds brake the current
   command; do not reset `qdot_prev` or `qddot_prev`.
5. Continue applying every existing hard safety constraint.

Following the current model TCP during braking prevents a captured pose from
creating a return command while nonzero joint velocity is being decelerated.
Once the command reaches rest, the reference naturally remains at the settled
model pose.

## Release Behavior

Release requires both raw PICO and SPARK target motion to exceed the existing
stationary thresholds on a fresh source frame. On release:

- clear the dwell timer and hold flag;
- restart the existing feedforward reference from the current model state;
- restart target blending from the current model pose;
- resume the latest live target without changing Cartesian gains.

This prevents a discontinuous jump from the settled pose back to an old
reference.

## Configuration

Add the following fields under
`spark_headroom_feedforward_velocity_qp`:

```yaml
settled_hold_enabled: true
settled_hold_dwell_seconds: 0.30
settled_hold_headroom_enter: 0.05
```

Input validation requires a positive dwell and a headroom threshold in
`[0, 1]` when the feature is enabled.

## Diagnostics

Expose per-arm telemetry for:

- settled-hold active;
- settled-hold dwell time;
- settled-hold reason (`none`, `stale`, or `headroom_exhausted`).

These fields distinguish a controller stability hold from PICO timeout,
solver failure, or ordinary task scaling.

## Testing

Unit tests cover:

1. stale input enters hold immediately;
2. live stationary input requires exhausted headroom for the full dwell;
3. healthy headroom never enters hold;
4. one noisy motion source cannot release hold;
5. both motion sources release hold and restart from model state;
6. hold emits current model pose, zero Cartesian twist, and zero posture target;
7. fixed-feedforward behavior remains unchanged.

Integration verification replays the same 2134-frame TJVR trace with MuJoCo
visualization. In the final two stale seconds, both arms must satisfy:

- reference `|qdot|` below `0.02 rad/s` at the end;
- reference `|qddot|` below `0.2 rad/s^2` at the end;
- no control failure, deadline miss, or hard-bound violation.

Fast-motion common-frame tracking metrics must not regress by more than 5%
relative to the current dual-source stationary-hold replay.

## Implemented Verification

The 2134-frame fixed TJVR trace was replayed both headlessly and with MuJoCo
visualization. The final two seconds were stale and held for both arms.

```text
                         left                 right
end |qdot| max           0.000106 rad/s       0.000134 rad/s
end |qddot| max          1.27e-8 rad/s^2      1.64e-8 rad/s^2
end |jerk| max           2.03e-12 rad/s^3     2.54e-12 rad/s^3
live settled-hold cycles 0                    0
```

On common live PICO source frames, position P95 changed from `320.36 mm` to
`308.79 mm` for the left arm and from `257.42 mm` to `253.25 mm` for the right
arm. Thus the stale stabilization did not reduce fast-motion tracking in this
trace. The Viewer reported zero deadline misses, control failures, snapshot
drops, and telemetry drops.
