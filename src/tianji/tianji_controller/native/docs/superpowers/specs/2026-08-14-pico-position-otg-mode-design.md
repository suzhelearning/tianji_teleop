# PICO Position OTG Mode Design

## Goal

Reduce PICO-driven end-effector oscillation without making effective tracking
slower. Keep the acceleration-level QP, arm-angle task, orientation OTG, joint
limits, acceleration limits, jerk limits, and the existing velocity-tracking
OTG behavior intact.

## Evidence

An identical 2584-frame PICO UDP replay compared three acceleration-QP input
paths. In the final implementation, relative to the velocity-tracking OTG,
left/right position-error P95 fell from 349/309 mm to 274/229 mm,
greater-than-3-Hz actual-position RMS fell from 20.8/18.2 mm to 13.8/11.1 mm,
effective lag fell from 95/100 ms to 90/80 ms, and acceleration-bound activity
fell from 37.2%/28.7% to 27.0%/23.3%. All 2584 datagrams were received, the
same 2574 frames were accepted, and there were zero control failures. Direct
QP was smoother only because it lagged to about 140 ms and did not track the
target adequately.

## Selected Design

Add one optional Cartesian OTG configuration field:

```yaml
cartesian_otg:
  translation_position_mode: true
```

When `translation_position_mode` is true:

- translation Ruckig always uses `ControlInterface::Position`;
- the Ruckig translation target velocity is zero, even if an upstream twist is
  present;
- second-order translation prediction is bypassed;
- timestamp-aware linear twist remains available only to stationary-hold and
  motion-intent logic; it is not consumed as Ruckig velocity feedforward;
- the configured translation velocity, acceleration, jerk, stationary hold,
  and target pose remain active;
- orientation processing is unchanged.

When the field is false or omitted, current velocity-tracking behavior remains
unchanged. This provides a one-line A/B and rollback path.

The PICO teleoperation configuration enables position mode. Other
configurations keep the default false unless explicitly changed.

## Data Flow

```text
PICO mapped pose and timestamp
  -> timestamp-aware target state
  -> raw mapped position target (no second-order position prediction)
     + estimated twist for stationary-hold intent detection
  -> Cartesian position OTG (target velocity = 0)
  -> Cartesian acceleration servo
  -> acceleration-level QP
  -> qddot / qdot_ref / q_ref
```

The mode does not filter or alter PICO orientation and does not change QP
decision variables, objectives, constraints, or reference integration.

## Compatibility and Failure Handling

- The new field is optional and defaults to false.
- Existing YAML files and command lines continue to load unchanged.
- Invalid target poses, stale input, Ruckig failure, pause/resume, and PICO
  epoch reset keep their existing behavior.
- Returning to the prior algorithm requires only setting
  `translation_position_mode: false`.

## Testing

Unit tests must prove:

- configuration parsing accepts true and defaults to false;
- position mode suppresses manual target position prediction while preserving
  the estimated linear twist for stationary-hold intent detection;
- position mode does not move when only target twist changes and target pose is
  stationary;
- velocity-tracking mode retains its existing behavior.

Regression verification must include the complete CTest suite and the same
2584-frame UDP replay. Compare common accepted sequences for target-to-actual
position/orientation error, greater-than-3-Hz motion, effective lag, QP bound
activation, solver failures, and deadline misses.

## Acceptance

- No increase in effective position lag relative to the current 90 ms replay
  baseline.
- Position-error P95 and greater-than-3-Hz actual position motion both improve
  on each arm.
- No control failures and no change to hard joint constraints.
- The original velocity-tracking mode remains selectable with one YAML field.

## Non-Goals

- Retuning arm-angle gains or acceleration;
- changing acceleration/jerk hard limits;
- changing orientation filtering or OTG;
- changing the acceleration QP formulation;
- solving all residual Cartesian error in this change.
