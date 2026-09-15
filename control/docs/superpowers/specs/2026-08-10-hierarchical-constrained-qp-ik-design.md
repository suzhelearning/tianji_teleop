# Hierarchical Constrained QP-IK Core Design

Date: 2026-08-10

## Goal

Replace the MuJoCo viewer's default 7-variable weighted-tracking QP with the
13-variable constrained QP core from
`tianji_vr_qp_ik_hierarchical_plan_v4_ALL_PLAINTEXT.md`. Preserve the current
marker interaction, validated world-frame kinematics, hard joint safety,
dual-arm all-or-nothing behavior, and non-blocking viewer architecture.

Add a runtime-selectable null-space DLS baseline so QP and DLS can be compared
under identical targets, Cartesian servo parameters, timing, joint bounds, and
reference integration.

## Scope

This phase includes:

- 200 Hz velocity-level IK in the current C++ MuJoCo viewer;
- per-arm 13-variable QP: 7 joint velocities plus 6 Cartesian slack values;
- hard joint position and velocity bounds;
- explicit soft Cartesian tracking equality with L2 slack;
- velocity regularization, posture preference, and continuity objectives;
- persistent qpOASES state and hot start;
- separate `q_ref`, `q_measured`, `qdot_prev`, and previous slack state;
- runtime hierarchical-QP/null-space-DLS switching;
- slack, active-bound, equality-residual, reference-error, and timing telemetry;
- unit, integration, headless, and manual GUI verification.

This phase excludes VR input and prediction, SDK transport, hard acceleration
bounds without trusted model data, ProxSuite, collision constraints, coupled
14-DoF optimization, dynamics, torque control, and strict lexicographic HQP.

## Preserved Behavior

- Marker picking and captured world/local drag behavior do not change.
- World-frame pose error remains paired with the validated world-frame TCP
  Jacobian.
- Left and right arms solve independently, but one failed arm freezes both
  references for that cycle.
- Existing Marker, Jacobian, SO(3), legacy solver, safety, snapshot, and
  headless tests are not deleted or weakened.
- Existing OSQP and 7-variable qpOASES code remains for legacy tests and
  offline regression during migration.

## Architecture

```text
target pose
    -> world-frame SE(3) error
    -> Cartesian servo Vd = Kp*error
    -> shared hard joint bounds
    -> selected backend
       |-> HierarchicalQpIk7
       |-> NullspaceDlsIk7
    -> shared result validation
    -> dual-arm acceptance
    -> q_ref_next = q_ref + qdot*dt
    -> MuJoCo qpos + mj_forward
```

Control timing is:

```text
rate = 200 Hz
dt   = 0.005 s
```

The renderer remains independent and consumes the latest control snapshot.

## Common IK Contract

```text
ArmIkInput:
    q_measured
    q_ref
    qdot_prev
    J_world
    Vd_world
    q_min
    q_max
    qdot_min
    qdot_max
    dt
```

```text
ArmIkResult:
    qdot
    slack
    status
    solve_time_us
    iterations
    equality_residual
    active_position_bound_count
    active_velocity_bound_count
    qdot_max_ratio
```

For DLS, `Vd - J*qdot` is reported as the slack-equivalent tracking violation
so A/B telemetry has the same semantics.

## Coordinate Conventions

The existing Cartesian order is retained:

```text
[linear_x, linear_y, linear_z,
 angular_x, angular_y, angular_z]
```

```text
e_position = p_target - p_current

e_orientation =
    so3_log(R_target*transpose(R_current))
```

The error and MuJoCo TCP Jacobian are both in the world frame.

```text
Vd_linear =
    clamp_norm(
        Kp_position .* e_position,
        max_linear_velocity
    )

Vd_angular =
    clamp_norm(
        Kp_orientation .* e_orientation,
        max_angular_velocity
    )
```

VR velocity feedforward is absent from this phase.

## Reference State

Each arm stores:

```text
q_ref
qdot_prev
slack_prev
```

Initialization and nominal reset use:

```text
q_ref      = q_measured
qdot_prev  = 0
slack_prev = 0
```

Normal integration is:

```text
q_ref_next = q_ref + qdot_solution*dt
```

`q_measured` is used for FK, Jacobian construction, hard bounds, and reference
tracking diagnostics. The reference is not reconstructed from measured state
on every cycle. MuJoCo applies `q_ref` directly, so measured/reference error is
normally near zero, but the separation prepares the controller for later SDK
feedback.

## Hard Joint Bounds

```text
velocity_lower_i = -velocity_scale*model_velocity_limit_i
velocity_upper_i =  velocity_scale*model_velocity_limit_i
```

```text
position_lower_i =
    (q_min_i + position_margin - q_measured_i) / dt

position_upper_i =
    (q_max_i - position_margin - q_measured_i) / dt
```

```text
qdot_lower_i = max(velocity_lower_i, position_lower_i)
qdot_upper_i = min(velocity_upper_i, position_upper_i)
```

Any `qdot_lower_i > qdot_upper_i` rejects the cycle before solve. Hard
acceleration bounds are omitted until trusted per-joint values exist;
continuity is handled as a low-priority objective.

## Hierarchical Constrained QP

### Decision Variable and Equality

```text
x = [qdot(7), slack(6)]
x dimension = 13
```

Slack follows the project Cartesian order. Tracking is represented by:

```text
J*qdot + slack = Vd
```

For finite `Vd`, `qdot = 0` and `slack = Vd` provide a task-feasible assignment
whenever zero joint velocity lies inside the hard bounds.

### Posture Reference

```text
q_nominal = 0.5*(q_min + q_max)

qdot_nominal =
    -nominal_gain*(q_measured - q_nominal)
```

### Objective

```text
minimize:

    0.5*slack^T*W_slack*slack
  + 0.5*lambda_reg*||qdot||^2
  + 0.5*posture_weight*||qdot - qdot_nominal||^2
  + 0.5*continuity_weight*||qdot - qdot_prev||^2
```

```text
W_slack =
    diag(
        slack_weight_position,
        slack_weight_position,
        slack_weight_position,
        slack_weight_orientation,
        slack_weight_orientation,
        slack_weight_orientation
    )
```

### Standard QP Form

```text
H =
    [H_qdot       0]
    [      0 W_slack]

H_qdot =
    (lambda_reg + posture_weight + continuity_weight)*I_7
```

```text
g = [g_qdot, 0, 0, 0, 0, 0, 0]

g_qdot =
    -posture_weight*qdot_nominal
    -continuity_weight*qdot_prev
```

```text
A_equal = [J, I_6]

lower_equal = Vd
upper_equal = Vd
```

Only the first seven variables receive physical bounds. Slack uses qpOASES
positive and negative infinity sentinels. Physical inputs, outputs, and
residuals must remain finite.

The priority semantics are:

```text
hard joint safety
    >
high-weight Cartesian slack
    >
regularization, posture, and continuity
```

This is a constrained single-QP hierarchy, not strict lexicographic HQP.

## qpOASES Solver

The new persistent dense solver uses:

```text
SQProblem:
    variables   = 13
    constraints = 6
```

The six constraints are equalities because their lower and upper values both
equal `Vd`. The first cycle initializes; later cycles hotstart updated Hessian,
gradient, equality matrix, variable bounds, and equality bounds while reusing
solver history where supported.

The new Viewer path uses qpOASES only. ProxSuite is not added in this phase.

## Null-Space DLS Baseline

```text
J_pinv =
    J^T*inverse(J*J^T + damping^2*I_6)

N = I_7 - J_pinv*J

qdot_raw =
    J_pinv*Vd
    + N*qdot_nominal
```

The raw velocity is uniformly scaled:

```text
qdot = alpha*qdot_raw
alpha in [0, 1]
```

For positive components exceeding the upper bound:

```text
alpha = min(alpha, qdot_upper_i/qdot_raw_i)
```

For negative components exceeding the lower bound:

```text
alpha = min(alpha, qdot_lower_i/qdot_raw_i)
```

Uniform scaling preserves direction. Per-joint clipping is not used.

## Runtime Selection

```text
Q = hierarchical constrained QP
D = null-space DLS
```

On switch:

```text
retain q_ref
qdot_prev  = 0
slack_prev = 0
reset QP solver state
cancel active marker drag
```

The switch does not reset nominal posture or create a joint-position jump. The
old Viewer `O/P` switching is removed from the new main path; legacy solver
tests remain.

## Validation and Failure Policy

Before solve, validate finite state, target twist, Jacobian, QP matrices, and
equality data; positive-definite Hessian; and a valid hard-bound intersection.

After solve, validate solved status, finite joint velocity and slack, hard
bounds, equality residual, and next-reference position margin.

```text
equality_residual =
    ||J*qdot + slack - Vd||
```

On any failure:

```text
qdot = 0
do not integrate q_ref
qdot_prev = 0
mark cycle rejected
```

If either arm fails, neither arm reference is integrated. A subsequent valid
cycle may recover automatically.

## Diagnostics

Per arm, snapshot, overlay, and telemetry expose:

```text
position_error_norm
orientation_error_norm
slack_position_norm
slack_orientation_norm
equality_residual
active_position_bound_count
active_velocity_bound_count
qdot_max_ratio
reference_error_max_abs
solve_time_us
solver_iterations
```

The overlay also identifies `hierarchical_qp` or `nullspace_dls`.

## Initial Configuration

```yaml
controller:
  rate_hz: 200.0

cartesian_servo:
  kp_position: [4.0, 4.0, 4.0]
  kp_orientation: [3.0, 3.0, 3.0]
  max_linear_velocity: 1.00
  max_angular_velocity: 3.14

hierarchical_qp:
  lambda_reg: 1.0e-4
  posture_weight: 1.0e-3
  continuity_weight: 1.0e-3
  slack_weight_position: 1.0e4
  slack_weight_orientation: 3.0e3
  equality_tolerance: 1.0e-8

dls:
  damping: 1.0e-3
```

Existing position margin and model velocity limits remain configurable. The
Cartesian gains are held constant for the first A/B so algorithm effects are
not confused with gain tuning.

## Testing

Builder tests verify exact 13-variable Hessian, gradient, equality, and bound
expansion; dimensions; slack feasibility; posture and continuity terms; and
invalid data rejection.

Solver tests cover full-rank, singular, and zero Jacobians; equality residual;
hard-bound activation; unreachable-task slack; warm start; and failed solves.

DLS tests cover the damped pseudoinverse, null-space posture, finite singular
behavior, uniform hard-bound scaling, and direction preservation.

Controller tests cover independent reference/measured state, no integration on
failure, dual-arm all-or-nothing behavior, switch continuity, state reset,
equality validation, and next-reference safety.

QP and DLS run from identical initial state and target sequence in:

1. central-workspace position tracking;
2. central-workspace position and orientation tracking;
3. fast continuous targets;
4. near-joint-limit motion;
5. singular start;
6. unreachable target;
7. runtime switching.

Compare position/orientation RMS error, settling time, joint-velocity variation,
maximum velocity ratio, slack, active bounds, solve-time percentiles, and
failure count.

## Acceptance

- Full CTest completes with zero failures.
- Existing tests are not removed or weakened.
- Reachable final position error is below `0.002 m`.
- Reachable final orientation error is below `1 degree`.
- Joint position margin and velocity limits are never violated.
- Singular scenarios retain finite bounded joint velocity.
- Unreachable targets retain solved QP status, produce non-zero slack, and keep
  the reference inside hard bounds.
- Equality residual remains within configured tolerance.
- QP solve-time P99 is below `5 ms`.
- GUI arrow/ring drag remains continuous.
- `Q`/`D` switching produces no joint jump.
- Fast dragging produces no visible chatter.
- Unreachable dragging raises slack while preserving hard safety.
- Returning to a reachable target restores tracking without reset.

## Migration

The hierarchical constrained QP becomes the default Viewer algorithm. Legacy
7-variable weighted-QP production code remains until the new implementation
passes all acceptance scenarios. Removing legacy production code is a separate
reviewed cleanup task.

Implementation remains on `feature/mujoco-cpp-qp-ik-v1` in the existing linked
worktree; no merge, push, branch deletion, or worktree removal is part of this
design.
