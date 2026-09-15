# SPARK qpOASES Direct and Velocity-QP Diagnosis

## Goal

Isolate whether abnormal Tianji arm posture originates in the SPARK-scaled
two-stage qpOASES position IK or in the downstream velocity-QP path.

## Preserved behavior

- Keep the current branch and workspace.
- Keep every existing algorithm selectable and unchanged.
- Keep the current joint position, velocity, acceleration, jerk, and braking
  limit values unchanged.
- Use the existing corrected PICO upper-limb skeleton, SPARK robot-length
  retargeting, two-stage qpOASES position IK, MuJoCo model, telemetry, and
  skeleton overlay implementations.
- Do not send commands to a physical robot.

## Diagnostic mode: `spark_upper_qpoases_direct`

Data flow:

```text
corrected PICO upper-limb skeleton
  -> SPARK robot-length retargeting
  -> two-stage qpOASES position IK
  -> q_ik
  -> direct MuJoCo arm qpos assignment
```

This mode bypasses Cartesian OTG, Cartesian velocity servo, velocity QP,
Ruckig, and reference integration. A valid bilateral solution updates both
arms atomically. An invalid or rejected frame holds the last valid model
configuration. Direct assignment is intentionally not dynamically smooth; it
is an IK isolation test.

The Viewer displays the corrected PICO skeleton and processed SPARK skeleton
at the same time. Telemetry records target skeleton, q_ik, direct q command,
position/orientation residuals, solver status, and discontinuities between
successive q_ik samples.

## Filtered mode: `spark_upper_qpoases_velocity_qp`

Data flow:

```text
corrected PICO upper-limb skeleton
  -> SPARK robot-length retargeting
  -> two-stage qpOASES position IK
  -> q_ik
  -> qdot_posture = Kq * (q_ik - q_model)
  -> existing Cartesian velocity-level QP
  -> existing reference integrator
  -> MuJoCo
```

`q_ik` is a joint-posture reference, not a numerical warm start for the
velocity-QP decision. The QP decision remains `qdot`. The Cartesian tracking
task remains primary; the posture objective softly minimizes
`||qdot - qdot_posture||^2`. `qdot_posture` is clamped by the existing joint
velocity limits before entering the objective. Existing hard position,
velocity, acceleration, jerk, and braking bounds remain authoritative.

## Comparison sequence

Use the same initial joint state and the same deterministic v4 PICO fast-motion
trace for both modes:

1. Replay `spark_upper_qpoases_direct` in MuJoCo and inspect elbow branch,
   shoulder posture, wrist orientation, joint flips, and q_ik discontinuities.
2. Only if the direct IK remains valid and visually coherent, replay
   `spark_upper_qpoases_velocity_qp`.
3. Compare Cartesian residuals, q/qdot/qddot/jerk, solver failures, active hard
   bounds, and visual arm posture.

Interpretation:

- Direct mode abnormal: investigate SPARK scaling or two-stage position IK;
  downstream velocity QP is not the cause.
- Direct mode normal but velocity-QP mode abnormal: investigate posture weight,
  Cartesian/posture conflict, model-state/reference integration, or active
  dynamic bounds.
- Both modes normal: later hardware/interface behavior is outside this MuJoCo
  diagnostic scope.

## Verification

- Unit tests prove direct mode bypasses trajectory and velocity-QP updates.
- Unit tests prove filtered mode constructs the bounded posture velocity from
  `q_ik - q_model`.
- Viewer integration tests prove both modes are selectable and produce finite
  bilateral diagnostics.
- Existing test suite remains green.
- Deterministic replay completes with zero malformed packets and no non-finite
  joint commands.
