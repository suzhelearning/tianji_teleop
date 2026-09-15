# Upper-Arm Outward-Only Constraint Design

## Goal

Add a deterministic redundancy mode that ignores the PICO skeleton arm angle,
keeps the 6D end-effector pose as the primary task, allows the upper arm to hang
vertically or move outward, and prevents the elbow from moving inward toward the
torso.

The implementation stays on `feature/pico-mujoco-teleop-v1` in the current
worktree. Existing PICO pose mapping, Position OTG, Cartesian servo gains, joint
position/velocity/acceleration/jerk limits, collision behavior, and recorded
`benchmark_results/` data remain unchanged.

## Selected Behavior

The Viewer exposes three comparable redundancy modes:

1. `pico`: existing PICO skeleton arm-angle tracking.
2. `default_down`: existing fixed world-down arm-angle tracking.
3. `outward_only`: new unilateral upper-arm constraint with no arm-angle
   objective.

`G` cycles through the three modes. The startup mode remains `pico` unless a
new command-line override selects another mode for deterministic headless or
interactive tests.

In `outward_only`, PICO still controls both end-effector positions and
orientations. Only the skeleton-derived arm-angle task is disabled.

## Geometry

Use the robot model's shoulder and elbow positions and translational Jacobians.
The fixed world outward directions are:

```text
left_outward  = world +Y
right_outward = world -Y
```

For each arm:

```text
r = p_elbow - p_shoulder
n = side-specific outward unit vector
h = n^T r - minimum_outward_distance
J_h = n^T (J_elbow - J_shoulder)
hdot = J_h qdot
```

The default minimum outward distance is zero. Therefore:

- `h = 0`: a vertically hanging upper arm is allowed;
- `h > 0`: outward motion is allowed;
- `h < 0`: the elbow is inward of the shoulder plane and must recover outward.

Forward/backward motion and elbow height are not constrained.

## QP Constraints

### Velocity level

Use a first-order control barrier inequality:

```text
J_h qdot >= -velocity_gain * h
```

At the boundary this prohibits inward velocity. Away from the boundary it
allows bounded inward velocity that decelerates before reaching the plane.

### Acceleration level

Use a second-order control barrier inequality:

```text
J_h qddot >=
    -Jdot_h_qdot
    -acceleration_kd * hdot
    -acceleration_kp * h
```

`Jdot_h_qdot` is computed from the same central-difference kinematic samples
used for Cartesian `Jdot*qdot`, so the new constraint does not add another pair
of MuJoCo forward-kinematics evaluations per arm.

Default parameters are:

```yaml
upper_arm_outward:
  minimum_outward_distance_m: 0.0
  velocity_gain: 8.0
  acceleration_kp: 100.0
  acceleration_kd: 20.0
```

The requested lower bound is clipped only when it exceeds the maximum value
physically achievable under the existing joint box bounds. This preserves QP
feasibility while commanding the strongest available outward recovery. Such a
clip is exposed in diagnostics and is not treated as successful enforcement.

## QP Interface

Extend the fixed qpOASES constraint matrix from six to seven rows:

```text
rows 0..5: existing Cartesian equality
row 6:     optional upper-arm outward inequality
```

Rows 0..5 keep equal lower and upper bounds. Row 6 uses the barrier lower bound
and positive infinity as its upper bound. When `outward_only` is inactive, row
6 is zero with free lower/upper bounds. A regression test must prove that the
legacy six-dimensional solution remains unchanged within solver tolerance.

Both the velocity hierarchical QP and acceleration QP use this representation.
No decision variables are added:

```text
velocity QP:     x = [qdot(7), slack(6)]
acceleration QP: x = [qddot(7), slack(6)]
```

## Controller Data Flow

```text
PICO end-effector pose
  -> existing Relative Mapping and Position OTG
  -> Cartesian velocity/acceleration command

robot shoulder/elbow model geometry
  -> outward distance/Jacobian
  -> mode selection
       pico/default_down: existing arm-angle task
       outward_only:      arm-angle task disabled + QP barrier enabled
  -> existing joint hard bounds
  -> velocity- or acceleration-level QP
  -> existing reference integration
```

## Diagnostics

Add per-arm telemetry fields:

- outward distance in metres;
- constraint enabled/active state;
- requested and effective lower bound;
- achieved outward velocity or acceleration;
- feasibility-clipped state;
- residual to the effective inequality.

The Viewer status shows the selected mode. Existing joint q/qdot/qddot/jerk
plots remain unchanged.

## Error Handling

- Non-finite geometry disables the outward constraint for that arm and marks
  diagnostics invalid; it must not poison the Cartesian QP.
- A zero or non-finite outward Jacobian disables the constraint for that cycle.
- If the requested barrier bound exceeds the joint-box achievable maximum, use
  the achievable maximum and record `feasibility_clipped=true`.
- Existing solver cold retry, fallback, reference watchdog, and hard joint
  limits remain unchanged.

## Alternatives Considered

### Fixed `default_down`

Already available and stable, but it unnecessarily forces the elbow toward one
direction even when an outward posture would better satisfy the endpoint.

### Soft outward penalty

Requires fewer solver changes but cannot guarantee that the elbow stays out of
the torso-side half-space when Cartesian demand is large.

### Hard outward half-space barrier

Selected because it constrains only the forbidden direction, leaves all other
redundancy free, and keeps end-effector tracking represented by the existing
Cartesian equality and slack.

## Tests and Comparison

Unit and integration tests must cover:

- left `+Y` and right `-Y` sign conventions;
- down/outward states allowed and inward states corrected;
- velocity and acceleration inequality construction;
- disabled-row equivalence with legacy QP behavior;
- general-constraint validation and qpOASES hotstart;
- mode cycling and command-line selection;
- non-finite and feasibility-clipped handling;
- complete Viewer and controller suites.

Run the same 2,584-frame, 29.802-second PICO replay in all three modes:

```text
pico
default_down
outward_only
```

Compare, per arm:

- minimum and P1 outward distance;
- cycles with outward distance below `-1 mm`;
- Cartesian position/orientation P50/P95/P99/max;
- solver failures, deadline misses, and feasibility clips;
- qdot/qddot/jerk hard-bound ratios and active-bound counts.

## Acceptance

- `outward_only` does not consume PICO arm directions.
- No inward excursion below `-1 mm` after the initial recovery interval.
- Zero hard joint-bound violations.
- Zero persistent QP failures; any feasibility clip is reported explicitly.
- End-effector position/orientation P95 does not regress by more than 20%
  relative to the same-packet `pico` run.
- Disabled outward constraints preserve legacy QP solutions within numerical
  solver tolerance.
- Full Tianji test suite passes.

## Non-Goals

- Limiting elbow height;
- limiting forward/backward upper-arm motion;
- changing PICO endpoint mapping or orientation;
- changing joint speed, acceleration, jerk, braking, or collision limits;
- adding a strict arm-angle target in `outward_only` mode;
- changing the QP decision variables.
