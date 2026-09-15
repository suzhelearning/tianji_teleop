# Command-model kinematics for QP IK

Date: 2026-08-10

## Goal

Improve Cartesian following under inner position-servo lag by separating the
state used for nominal IK from the state used for physical safety.

The controller shall use its integrated command state (`q_ref`) to evaluate
forward kinematics, the TCP Jacobian, Cartesian tracking error, and the nominal
QP state. It shall continue to read actual joint feedback (`q_actual`) every
cycle for physical-limit checks, tracking-error watchdogs, fault handling, and
startup/resynchronization.

This follows the useful part of Codeit's continuous-motion pattern: initialize
the model from actual position, then advance the model from target position.
It does not reinterpret `setMp()` as hardware feedback and does not discard
actual feedback.

## Problem

The current controller already integrates the accepted QP velocity into
`q_ref`, but each following cycle obtains the TCP pose and Jacobian from the
actual MuJoCo state. With a real position servo, `q_actual` trails `q_ref`.
Feeding that lag back into the outer Cartesian controller causes the QP to
compensate for an error already being handled by the inner servo. This can
increase phase lag, overshoot, and oscillation.

The desired separation is:

```text
nominal control model: q_model = q_ref
physical observation:  q_feedback = q_actual
```

## Considered approaches

### 1. Keep actual-state kinematics

Continue computing FK and Jacobians at `q_actual`. This is simple and always
matches the measured mechanism, but it preserves the double-feedback behavior
and makes the outer loop sensitive to inner-loop delay. It is not selected.

### 2. Command-model nominal IK plus actual-feedback safety

Compute nominal kinematics at `q_ref`, while retaining actual state for safety
and supervision. This is selected because it gives the outer controller a
coherent internal trajectory without hiding real tracking failures.

### 3. Blend actual and command states

Use `q_model = q_ref + beta * (q_actual - q_ref)`. This can be useful when the
model has substantial bias, but it adds another coupled gain and partially
reintroduces servo delay. It is deferred until telemetry demonstrates a model
mismatch that supervision alone cannot handle.

## Architecture

### Arbitrary-state kinematics

`MujocoRobot` will expose a read-only kinematics operation for one arm at an
arbitrary seven-joint position. The result contains at least:

```text
ArmKinematicSample:
    tcp_pose_world
    tcp_jacobian_world
```

The operation will use dedicated scratch `mjData` (or equivalent isolated
MuJoCo state), set only the requested arm coordinates while copying the rest of
the current configuration needed by the model, run forward kinematics, and
return the pose and Jacobian. It must not mutate the main simulation state,
actual feedback, or commanded state.

Keeping pose and Jacobian in one operation guarantees they are evaluated at
the same configuration and avoids duplicate forward passes.

### State roles

At startup, reset, and explicit controller re-enable:

```text
q_actual = measured arm position
q_ref    = q_actual
q_model  = q_ref
```

During continuous operation:

```text
T_model = FK(q_ref)
J_model = J(q_ref)
e_x     = Log(T_target * inverse(T_model))
V_des   = V_feedforward + Kp * e_x
qdot    = solve_qp(J_model, q_ref, V_des, constraints)
q_ref_next = q_ref + dt * qdot
```

The position-level QP's command trajectory bounds are evaluated from `q_ref`,
so the recursively generated command cannot cross configured joint limits.
Actual position is checked separately against the physical envelope and is
used to calculate command-following error.

### Feedback supervision

For each arm, calculate:

```text
e_tracking = q_ref - q_actual
tracking_error_max = max(abs(e_tracking))
```

The existing warning and stop thresholds remain the first implementation of
the watchdog:

- Below warning: run nominal IK without injecting actual servo error into FK.
- Warning to stop: scale the Cartesian command using the existing tracking
  scale policy.
- At or above stop: freeze the affected command reference and report the
  watchdog state.

A severe tracking event must not silently assign `q_ref = q_actual` during
normal operation. Resynchronization is allowed only on an explicit reset,
controller restart/re-enable, or an established fault-recovery transition.
This avoids command discontinuities.

Actual joint positions remain authoritative for detecting a physical limit
violation. Future collision constraints likewise remain based on actual state
or a separately validated swept-state prediction.

### Diagnostics

Diagnostics shall distinguish the two concepts rather than label both as
"current":

```text
q_ref, q_actual
tcp_model, tcp_actual
model_target_position_error
model_target_orientation_error
tracking_error_max
tracking_scale / watchdog state
```

Existing fields may be retained for compatibility, but their state source must
be documented. Telemetry should make it possible to tell whether a delay comes
from Cartesian reference generation, IK saturation, or inner-servo tracking.

## Safety and failure behavior

- QP command-position bounds use `q_ref` and remain hard constraints.
- Velocity bounds remain hard constraints.
- Actual joint feedback is checked against configured physical joint limits.
- The tracking watchdog can scale or freeze command progression.
- Failure to evaluate command-model kinematics rejects that arm's update and
  leaves its previous safe `q_ref` unchanged.
- Existing solver failure and stale-target handling remain in force.
- No control path may continue indefinitely without fresh actual feedback.

For this in-process MuJoCo backend, `armPosition()` synchronously reads the
controller-owned `mjData`, so freshness is guaranteed by construction. A future
hardware adapter must enforce timestamp/sequence timeout before exposing an
equivalent feedback read; command publication must not advance that feedback
freshness indicator.

The base MuJoCo harness writes `q_ref` directly to its model, so normal tests
have zero artificial servo lag. Lag behavior will be tested by explicitly
injecting measured-state lag while preserving the controller command state.

## Test strategy

Tests will be added before implementation for these observable behaviors:

1. Arbitrary-state FK/Jacobian evaluation returns the requested configuration's
   kinematics and leaves the main robot state unchanged.
2. With injected feedback lag below the warning threshold, nominal TCP pose,
   Jacobian, and QP command are computed from the same `q_ref` as a no-lag run.
3. Crossing the warning threshold scales command progression; crossing the stop
   threshold freezes the affected reference.
4. The integrated `q_ref` remains inside command joint limits even when actual
   feedback lags.
5. Reset/re-enable synchronizes `q_ref` to `q_actual` exactly once.
6. Diagnostics expose model pose, actual pose, and joint tracking error as
   separate quantities.
7. Existing unit tests and headless integration tests continue to pass.

## Downstream branch propagation

Implementation starts on `feature/mujoco-cpp-qp-ik-v1` as shared
infrastructure. After verification:

1. Merge the base change into `feature/cartesian-otg-velocity-qp-v1`.
2. In the velocity branch, independently repair orientation OTG settling using
   tangent-space Ruckig position tracking and an endpoint settle condition.
3. Merge the velocity branch into
   `feature/cartesian-otg-acceleration-qp-v1`.
4. In acceleration mode, evaluate `J` and `Jdot*qdot` from command-model
   `q_ref` and `qdot_ref`; retain actual `q` and `qdot` for supervision.

Each branch is tested and committed independently so the three modes remain
directly comparable.

## Non-goals

- Removing actual joint feedback from the controller.
- Adding Cartesian OTG to the base branch.
- Adding acceleration-level QP or `Jdot*qdot` to the base branch.
- Implementing collision avoidance in this change.
- Automatically blending actual and command states.
- Raising gains or limits as a substitute for fixing state semantics.
