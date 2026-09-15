# Cartesian OTG Dual-Branch Design

## Goal

Implement and compare two VR teleoperation controllers without replacing the
validated `feature/mujoco-cpp-qp-ik-v1` baseline:

1. Cartesian OTG plus the existing velocity-level constrained QP.
2. The same Cartesian OTG plus an acceleration-level constrained QP.

Both controllers run at 200 Hz, preserve the existing world-frame convention,
left/right arm isolation, joint-limit margins, reference watchdog, telemetry,
headless viewer, and deterministic benchmarks.

## Branch topology and visible worktrees

```text
feature/mujoco-cpp-qp-ik-v1 @ f5ac6cb
  |
  +-- feature/cartesian-otg-velocity-qp-v1
        worktree: ../TJ_arm_control_cartesian_otg_velocity_qp_v1
        |
        +-- feature/cartesian-otg-acceleration-qp-v1
              worktree: ../TJ_arm_control_cartesian_otg_acceleration_qp_v1
```

The acceleration branch is created only after the velocity OTG branch passes
its complete verification. This gives both implementations one identical OTG
and preserves velocity OTG as the B controller in the A/B/C comparison.

## Shared Cartesian reference generator

Each arm owns a `CartesianReferenceGenerator` with explicit state:

```text
translation: p_ref, v_ref, a_ref
rotation:    R_ref, w_ref, alpha_ref
```

The raw VR target manager remains responsible for accepting monotonic source
timestamps, estimating target linear/angular velocity only on new source
frames, filtering those estimates, rejecting out-of-order frames, and marking
stale targets. It no longer extrapolates the final servo target when OTG mode
is active.

The reference generator is responsible for trajectory shaping:

- translation: Ruckig 3-DoF position interface with target position, filtered
  target velocity, and zero target acceleration;
- orientation: world-frame SO(3) shortest-path reference with angular velocity,
  acceleration, and jerk norm limits;
- stale target or Hold: retain the latest accepted pose and drive target
  velocity to zero, allowing the reference to decelerate continuously;
- activation/reset: initialize reference pose from the measured TCP and all
  reference derivatives from zero.

The output type contains reference pose, twist, acceleration, stale state, and
OTG status for each arm. Invalid OTG output holds only the affected arm and is
visible in telemetry.

## Velocity-level branch

The velocity branch retains the existing decision vector and solver:

```text
x = [qdot(7), slack(6)]
J(q_measured) qdot + slack = V_des
```

The Cartesian command becomes:

```text
V_des = V_ref + Kp * LogSE3(T_ref * inverse(T_measured))
```

The old differentiated-target `Kff` path remains available only for the direct
baseline configuration. OTG configurations use unit reference feedforward and
do not apply prediction or Kff a second time.

Existing model velocity, one-step position, command-acceleration, and braking
bounds remain hard QP bounds. The reference watchdog, independent arm failure
containment, qpOASES hotstart, slack normalization, and q-reference integration
remain in place.

The branch adds a deterministic benchmark that runs:

```text
A: direct target + velocity QP
B: Cartesian OTG + velocity QP
```

against identical trajectories and reports tracking error, phase lag,
Cartesian/reference jerk, joint velocity variation, active bounds, slack,
solver timing, and failures.

## Acceleration-level branch

The acceleration branch reuses the exact OTG module and adds a separate
acceleration controller rather than changing the velocity solver interface.

Per-arm state is:

```text
q_ref, qdot_ref, qddot_previous, slack_previous
```

Measured kinematics are:

```text
T, J from q_measured
V = J qdot_measured
Jdot_qdot from a model-based directional Jacobian derivative
```

The acceleration servo is:

```text
A_des = A_ref + Kd (V_ref - V_measured) + Kp pose_error
```

The acceleration QP is:

```text
x = [qddot(7), slack(6)]
J qddot + slack = A_des - Jdot_qdot
```

Its objective contains dimensionally normalized Cartesian acceleration slack,
qddot regularization, qddot continuity/jerk cost, and weak posture
acceleration. Hard bounds intersect configured qddot limits, predicted qdot
limits, second-order predicted q-position limits, and braking-aware predicted
velocity limits. Jerk starts as a soft cost; hard jerk bounds remain optional.

Reference integration uses:

```text
qdot_ref_next = qdot_ref + qddot * dt
q_ref_next = q_ref + qdot_ref * dt + 0.5 * qddot * dt^2
```

The branch extends the MuJoCo kinematic harness to expose consistent q and qdot
state. This validates acceleration kinematics and controller mathematics, but
is explicitly not an actuator-dynamics or hardware-safety claim.

The deterministic benchmark runs:

```text
A: direct target + velocity QP
B: Cartesian OTG + velocity QP
C: Cartesian OTG + acceleration QP
```

## Dependencies and interfaces

- Add Ruckig as a pinned build dependency and link it only through the OTG
  module.
- Keep qpOASES as the 13-variable QP solver for both levels.
- Add measured arm velocity and model `Jdot*qdot` methods to `MujocoRobot` in
  the acceleration branch only.
- Keep target ingestion, reference generation, servo construction, QP solve,
  integration, and telemetry as separate testable modules.
- Configuration uses separate `cartesian_otg`, `cartesian_acceleration_servo`,
  and `acceleration_qp` sections. Existing configs retain their behavior through
  explicit defaults.

## Failure handling

- Non-finite VR, OTG, kinematic, bound, or QP data rejects the affected arm.
- Out-of-order VR frames do not mutate accepted target or velocity state.
- Stale VR drives OTG target velocity to zero; it does not zero reference state
  discontinuously.
- Infeasible joint bounds freeze the affected arm reference and clear only its
  solver history.
- Excessive `q_ref-q_measured`, and in the acceleration branch
  `qdot_ref-qdot_measured`, progressively scale then freeze the affected arm.
- Shared malformed dual-arm input remains a shared rejection.

## Verification gates

Velocity branch:

- OTG unit tests for translation limits, SO(3) shortest path, jerk continuity,
  reset, Hold, stale input, and invalid input;
- servo tests proving `V_ref + Kp*error` with no duplicate Kff;
- existing 23-test regression suite;
- A/B deterministic benchmark with finite metrics and zero QP failures;
- four-second headless viewer with zero command/control failures and deadline
  misses.

Acceleration branch:

- qdot mapping and `Jdot*qdot` numerical-consistency tests;
- acceleration servo and qddot-bound unit tests;
- acceleration-QP equality, objective, bound, hotstart, and failure tests;
- second-order reference integration and watchdog tests;
- complete inherited regression suite;
- A/B/C benchmark with finite metrics, zero acceleration-QP failures, hard
  bounds respected, and reported phase-lag/jerk tradeoff;
- four-second headless viewer and real-time timing telemetry.

## Non-goals

- strict lexicographic HQP;
- 14-DoF coupled dual-arm QP;
- collision constraints or torque/dynamics control;
- claiming simulated qddot is directly executed by Tianji hardware;
- merging either experiment into the baseline automatically;
- hiding worktrees inside the project directory.
