# End-Effector Priority and Arm-Angle Nullspace Design

## Purpose

Improve PICO-driven orientation response in both velocity-level and
acceleration-level QP control without sacrificing the arm-angle reference that
keeps the robot elbow aligned with the human upper limb.

The end-effector six-dimensional Cartesian task is the primary task. The arm
angle is a redundancy task and must never increase the primary task residual.
The design remains compatible with a future 200 Hz Tianji SDK joint-impedance
controller.

## Confirmed constraints

- Keep the robot joint velocity limit at 180 deg/s, or 3.1416 rad/s.
- Do not raise the model joint velocity limits to create artificial tracking
  performance.
- The current MuJoCo PICO profile may retain its high simulation acceleration
  limits. A future hardware profile must use the SDK PNVA acceleration limits:
  7.854 rad/s^2 for joints 1-2 and 15.708 rad/s^2 for joints 3-7.
- Keep the control loop at 200 Hz so its 5 ms reference period matches the
  Tianji SDK's recommended real-time point-streaming period.
- Preserve the current default-down arm-angle reference whenever PICO is not
  live.
- Do not add a raw torque-output path in this change. The available SDK torque
  mode is an internal joint/cartesian impedance controller driven by joint
  position targets and K/D parameters.

## Root cause

The current single weighted QP puts arm-angle tracking and end-effector slack
in one objective. In the acceleration QP, the effective orientation slack
weight is:

```text
slack_weight_orientation / slack_angular_scale^2
= 3000 / 15^2
= 13.33
```

The arm-angle acceleration weight is 20. A conflict therefore makes the solver
prefer arm-angle tracking over end-effector orientation tracking. The velocity
QP has a less severe version of the same structural problem.

Fast PICO motion also exposes an artificial velocity mismatch: the Cartesian
OTG can request more than 3 rad/s while the velocity servo clamps its output to
3 rad/s before the QP sees it. The QP then spends most fast-motion cycles at
joint velocity or acceleration bounds.

## Selected architecture

### Primary QP

The velocity QP solves only the primary Cartesian task and existing safety,
posture, regularization, and continuity terms:

```text
J qdot + s = V_des
qdot_lower <= qdot <= qdot_upper
```

The acceleration QP similarly solves:

```text
J qddot + s = A_des - Jdot qdot
qddot_lower <= qddot <= qddot_upper
```

The arm-angle cost is removed from both primary Hessians and gradients.

### Bounded one-dimensional nullspace refinement

For a full-row-rank 6-by-7 Jacobian, compute a normalized nullspace vector `n`
from the final column of the full right-singular-vector matrix:

```text
J n ~= 0
```

Starting from the accepted primary solution `x_primary`, refine only along
that direction:

```text
x_refined = x_primary + alpha n
```

For the scalar arm-angle Jacobian `j_arm` and desired arm-angle rate or
acceleration `u_arm`:

```text
alpha_desired =
    (u_arm - j_arm x_primary) / (j_arm n)
```

Intersect the scalar alpha interval induced by every existing joint lower and
upper bound, then clamp `alpha_desired` into that interval. Because the
correction lies in the Jacobian nullspace, it preserves the primary Cartesian
velocity or acceleration and its slack to numerical tolerance.

If the Jacobian, task, or bounds are non-finite, if the nullspace residual is
too large, or if `abs(j_arm n)` is too small, return the primary solution
unchanged. A secondary task is never allowed to turn an accepted primary QP
solution into a rejected cycle.

### Shared module boundary

A focused nullspace-refinement function will live with the arm-angle module.
It consumes the primary seven-vector, Cartesian Jacobian, scalar arm-angle
task, and joint bounds. It returns the refined seven-vector plus diagnostics:
whether refinement was active, selected alpha, arm-angle value before and
after, and the Cartesian residual introduced by refinement.

Both velocity and acceleration controllers use this same function after their
primary solver succeeds and before integrating references.

## Response configuration

For the PICO MuJoCo profile:

- Raise `cartesian_servo.max_angular_velocity` from 3 to 6 rad/s. This removes
  the artificial Cartesian clamp while retaining the 3.1416 rad/s per-joint
  hard bounds inside the QP.
- Raise the effective orientation slack priority in both primary QPs so
  orientation cannot be cheaply discarded relative to regularization or
  posture. Position remains the higher-priority Cartesian component.
- Do not raise model joint velocity limits.
- Do not change PICO target orientation mapping or add another input filter.

The OTG reference may remain faster than the robot in some directions. The QP
must expose this as bounded slack instead of distorting the result with a
competing arm-angle objective.

## Future Tianji SDK integration

The eventual hardware path is:

```text
200 Hz QP q_ref
    -> SetImpJointMode(arm, 100, 100, K, D)
    -> FX_OnSetVelEstStep(arm, 5)
    -> SetJointPostionCmd(arm, q_ref_deg)
```

Before hardware use, add a separate hardware profile with per-joint PNVA
acceleration limits and independently validated K/D values. This design does
not assume that torque/impedance mode removes motor velocity, current, torque,
or braking constraints.

## Safety and failure behavior

- Primary QP validation remains unchanged.
- Nullspace refinement uses the exact bounds already supplied to the primary
  QP and cannot exceed them.
- The refined solution is checked for finiteness, joint-bound compliance, and
  Cartesian residual preservation.
- Failed secondary refinement falls back to the accepted primary solution.
- Existing PICO stale, reference-tracking, joint-limit, and solver watchdogs
  remain active.

## Verification

Automated tests will establish these behaviors before implementation:

1. A conflicting arm-angle request improves arm-angle tracking without changing
   `J x` beyond numerical tolerance.
2. Refinement respects all joint bounds and clamps alpha when necessary.
3. Degenerate or non-finite secondary tasks leave the primary result unchanged.
4. Velocity and acceleration controller tests verify that enabling arm angle
   does not degrade primary Cartesian tracking.
5. The existing Cartesian OTG benchmark and fast PICO UDP replay compare
   orientation error, slack, active bounds, control failures, and deadlines
   against the recorded baseline.
6. The full unit and integration test suite remains green.

## Out of scope

- Direct raw joint-torque commands.
- Tianji SDK networking and hardware mode switching.
- K/D tuning on the physical robot.
- Raising physical joint velocity limits above the SDK PNVA values.
- General multi-task or whole-body lexicographic HQP.
