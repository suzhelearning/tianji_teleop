# MuJoCo high-response Cartesian profile

Date: 2026-08-10

## Goal

Reduce dynamic lag in both Cartesian OTG branches while preserving their
current accurate, non-oscillatory endpoint settling. This profile targets the
MuJoCo Viewer only and is not a manufacturer-approved real-robot profile.

The deterministic baseline shows that both controllers reach the configured
Cartesian acceleration and jerk ceilings. The acceleration controller also
reaches the 20 rad/s^2 joint-acceleration ceiling in several dynamic
scenarios. Normal reachable trajectories have small QP slack and zero solver
failures, so increasing Cartesian gains is not the first intervention.

## Selected approach

Use one high-response limit set through the complete command chain instead of
changing only the OTG. The two branches shall use the same Cartesian OTG
limits:

```text
translation velocity:      2.0 m/s
translation acceleration: 12.0 m/s^2
translation jerk:        120.0 m/s^3

angular velocity:          8.0 rad/s
angular acceleration:     40.0 rad/s^2
angular jerk:             400.0 rad/s^3
```

The velocity branch shall allow the generated reference through its Cartesian
velocity servo:

```text
maximum linear command:  2.5 m/s
maximum angular command: 8.0 rad/s
```

Its joint-velocity QP shall keep the URDF joint-velocity hard limits while
using:

```text
maximum joint acceleration: 60.0 rad/s^2
braking acceleration:       45.0 rad/s^2
```

The acceleration branch shall use the same OTG limits and shall additionally
use:

```text
Cartesian linear acceleration command:   30.0 m/s^2
Cartesian angular acceleration command: 100.0 rad/s^2
maximum joint acceleration:              60.0 rad/s^2
braking acceleration:                    45.0 rad/s^2
```

The acceleration profile's velocity-level fallback constraints shall be
updated to the same 60/45 rad/s^2 values so A/B/C comparisons use coherent
limits.

## Deliberately unchanged behavior

- Cartesian position and orientation `Kp/Kd` remain unchanged.
- Joint position limits, the joint-4 human-elbow limit and joint velocity
  limits remain hard constraints.
- Reference tracking watchdog thresholds remain unchanged.
- Orientation settle thresholds and the persistent tangent-space OTG state
  remain unchanged.
- Command-model kinematics continue to use `q_ref` or `q_ref/qdot_ref`, while
  actual feedback remains authoritative for supervision.
- Hard joint jerk remains disabled in the acceleration profile; OTG jerk stays
  bounded at the Cartesian reference layer.

## Alternatives not selected

Only increasing Cartesian `Kp` does not remove an upstream OTG rate limit and
risks overshoot. Bypassing OTG during dragging gives lower delay but discards
the acceleration and jerk continuity that fixed endpoint oscillation. An
adaptive or error-dependent OTG profile is deferred until the fixed
high-response profile has measured results.

## Verification

Run the existing deterministic Cartesian benchmark before and after the
configuration changes. For reachable dynamic scenarios, the new profile must:

- reduce phase lag or settling time in straight, reversal, stop and
  orientation motion;
- keep solver failures at zero and all hard-bound checks true;
- keep every reported metric finite;
- avoid materially increasing settled position or orientation error;
- preserve orientation endpoint convergence tests;
- pass all branch CTest suites and an 800-cycle headless Viewer run with no
  control failures or deadline misses.

Because this is an upper-limit MuJoCo profile, increased acceleration and jerk
are expected. The benchmark comparison shall report that trade-off explicitly
rather than hiding it behind a pass/fail aggregate.
