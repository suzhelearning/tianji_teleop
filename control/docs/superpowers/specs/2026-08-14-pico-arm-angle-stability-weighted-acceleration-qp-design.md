# PICO Arm-Angle Stability and Weighted Acceleration-QP Design

## Goal

Remove occasional PICO-driven elbow-angle twitch and substantially reduce the
right-arm angle mismatch while preserving the current Position OTG, Cartesian
acceleration servo, joint position/velocity/acceleration/jerk hard bounds, PICO
relative mapping, end-effector orientation mapping, and velocity-level QP.

The implementation starts from commit `cec4f0c` on
`feature/pico-mujoco-teleop-v1`. Recorded CSV files and `benchmark_results/`
remain outside Git.

The robot-side implementation remains in the current TJ worktree and branch.
The PICO-side confidence gate belongs to
`/home/zj/current_robotics/PICO_tracker_tianji_mujoco_teleop_v1`, whose
`feature/tianji-mujoco-teleop-v1` worktree already contains user changes.
Implementation must inspect and preserve those changes, add only the minimal
geometry/test delta, and must not switch or clean either worktree. The bridge
packet layout remains unchanged.

## Evidence

The 147.965 s live recording `pico_position_otg_live.csv` contains 29,590
active PICO cycles and zero control failures. It nevertheless shows:

- right arm-angle error P95 of 0.986 rad versus 0.551 rad on the left;
- a maximum adjacent-cycle right arm-angle-rate change of 3.741 rad/s;
- right arm-angle acceleration request saturation during 8.14% of all active
  cycles;
- a 7.32 s interval with mean right error 1.40 rad, 95.8% arm-angle request
  saturation, and an effective joint-acceleration bound active during 97.7%
  of cycles, while the left arm remained near its reference.

The current acceleration QP does not contain a weighted arm-angle objective.
It solves the Cartesian acceleration/slack problem first and then applies an
exact-Cartesian-nullspace correction. When the primary solution consumes the
available joint bounds, the secondary correction cannot track PICO.

There are also two conditioning gaps:

- the PICO bridge normalizes a human elbow radial projection down to 0.1 mm,
  which amplifies skeleton noise when the human arm is nearly straight;
- the robot arm-angle Jacobian divides by the projected reference norm down to
  0.001. The current formula's Jacobian norm increases from about 0.57 at a
  0.095 projection to about 56 at 0.00107 before switching discontinuously to
  fallback behavior.

## Considered Approaches

### 1. Projection conditioning only

Add hysteresis and previous-direction hold around both singularities. This is
the smallest and safest twitch fix, but it cannot resolve the measured
multi-second mismatch when joint bounds leave no strict nullspace correction.

### 2. Projection conditioning plus weighted acceleration-QP task

Condition both reference boundaries and move the acceleration-level arm-angle
task into the QP objective with a configurable weight. Cartesian acceleration
remains represented by the existing equality with weighted slack, so its
priority stays higher while a controlled amount of Cartesian slack can be
traded for human-like elbow posture. This is the selected approach.

### 3. Hard arm-angle hierarchy

Add a second lexicographic QP stage or arm-angle equality. This provides a
strict priority statement but is unnecessarily invasive for a 7-DoF arm and
can make the Cartesian/arm-angle combination infeasible under the existing
hard bounds.

## Selected Architecture

### Human skeleton conditioning

In the PICO bridge, treat the human elbow-plane direction as undefined when
the shoulder-elbow radial projection about the shoulder-wrist axis is below
0.02 m. The palm target remains valid; only that side's arm direction becomes
invalid.

When a live PICO frame contains an invalid arm direction, the Tianji-side
`ArmDirectionReferenceManager` keeps the previous valid direction and marks it
as `previous`. It must not rotate toward world-down. Stale PICO, disabled PICO,
or no previous direction still uses the deterministic world-down reference.
No UDP packet layout changes are required.

### Robot reference-projection conditioning

Add these `arm_angle` configuration values:

```yaml
reference_projection_hold_enter: 0.05
reference_projection_hold_exit: 0.10
```

The values are dimensionless norms of a unit reference direction projected
onto the plane normal to the robot shoulder-wrist axis. Below the enter
threshold, keep the previous in-plane reference. Remain held until the norm
exceeds the exit threshold, then reacquire through the existing 8 rad/s rate
limit. During hold, use the normalized held projection for Jacobian
differentiation so the implementation never divides by a near-zero reference
projection.

Initial operation without a previous projection keeps the existing
side-specific deterministic outward fallback.

### Weighted arm-angle acceleration objective

Add this acceleration-QP parameter:

```yaml
acceleration_qp:
  arm_angle_weight: 10.0
```

For an active scalar task with Jacobian `j`, target acceleration `a_des`, and
activation `gamma`, add the following objective term:

```text
0.5 * arm_angle_weight * gamma * (j * qddot - a_des)^2
```

This contributes:

```text
H_qddot += arm_angle_weight * gamma * j^T * j
g_qddot += -arm_angle_weight * gamma * a_des * j^T
```

The existing six-dimensional Cartesian equality, position/orientation slack
weights, regularization, posture/jerk objectives, and every hard joint bound
remain unchanged. The acceleration controller consumes the QP result directly
instead of applying the current post-QP nullspace refinement. The
velocity-level controller and QP retain their existing arm-angle path.

`arm_angle_weight: 0` disables the new acceleration-level objective and gives
a direct rollback path.

### Diagnostics

Extend arm-angle diagnostics and telemetry with:

- robot reference projection norm;
- robot arm-angle Jacobian norm;
- whether previous projection is being held;
- requested and achieved arm-angle acceleration;
- arm-angle acceleration residual.

These fields make skeleton/reference conditioning distinguishable from QP
bound saturation without changing the live joint plot or control timing.

## Data Flow

```text
corrected PICO shoulder/elbow/wrist
  -> human radial confidence gate
  -> live direction manager (invalid means hold previous)
  -> robot reference-projection hysteresis
  -> bounded arm-angle error/Jacobian
  -> acceleration target: Kp * error - Kd * current_rate
  -> weighted arm-angle term inside Cartesian acceleration QP
  -> existing joint hard bounds
  -> qddot -> qdot_ref -> q_ref
```

The Position OTG translation and orientation references bypass this change.

## Error Handling

- A degenerate human elbow direction invalidates only that arm-angle input;
  both end-effector pose targets continue normally.
- A live invalid direction holds the previous arm direction; a stale or
  disabled PICO source transitions toward deterministic world-down.
- A robot-side singular projection holds its previous in-plane direction with
  hysteresis rather than normalizing noise.
- A non-finite task is disabled for that cycle; Cartesian control and safety
  bounds remain active.
- QP infeasibility and fallback behavior remain unchanged.

## Testing

Tests must cover:

- bridge rejection below the 0.02 m human radial threshold while preserving
  palm targets and the opposite arm;
- previous-direction hold for a live invalid PICO side and world-down behavior
  for stale/no-PICO operation;
- projection hysteresis, bounded Jacobian norm, and continuous reacquisition;
- exact acceleration-QP Hessian/gradient contributions for active, inactive,
  and zero-weight arm-angle tasks;
- acceleration-controller use of the weighted QP result without post-solve
  nullspace mutation;
- parsing, validation, default compatibility, telemetry schema, and PICO
  Viewer integration;
- complete CTest and PICO bridge test suites.

The recorded raw UDP replay is run before and after with identical packets.
Live acceptance additionally records a new CSV and evaluates the existing
red-capable diagnostic command.

## Acceptance

- Zero control failures and no hard-bound violations.
- No change to Position OTG, endpoint mapping, orientation mapping, or
  velocity-level QP behavior.
- No reference-projection Jacobian spike in the new conditioning tests.
- On comparable live PICO motion, right arm-angle error P95 must decrease from
  0.986 rad, maximum adjacent-cycle arm-angle-rate change must decrease from
  3.741 rad/s, and end-effector position/orientation error must not regress by
  more than 20%.
- The new telemetry must identify whether any remaining mismatch comes from a
  held/invalid reference or an acceleration task residual under active bounds.

## Non-Goals

- Changing joint speed, acceleration, braking, or jerk limits;
- changing Cartesian OTG gains or limits;
- changing PICO hand orientation or relative position mapping;
- adding collision constraints;
- converting the velocity-level QP to acceleration level;
- implementing strict lexicographic HQP.
