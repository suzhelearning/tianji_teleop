# PICO Cartesian OTG Velocity Tracking Port Design

## Goal

Move the already benchmarked Cartesian moving-target tracking behavior from
`feature/mujoco-cpp-qp-ik-v1` into the PICO MuJoCo teleoperation worktree so
the live PICO test actually exercises the reduced-lag reference generator.

## Scope and compatibility

The port changes only the Cartesian OTG configuration, reference generator,
timestamp-aware target state, their unit tests, and
`config/qp_ik_pico_teleop.yaml`. Existing PICO protocol, resynchronization,
arm-angle, Viewer, telemetry, and QP modifications remain untouched. The new
behavior is opt-in; configurations without the new keys retain Position-OTG.

## Moving-target translation

When the source is live and target speed exceeds the stationary threshold,
Ruckig uses its Velocity interface with:

```text
v_command = clamp_norm(v_target + 30 * (p_target - p_reference), v_max)
```

The configured acceleration and jerk limits remain active. When the target is
stationary or stale, translation returns to Position mode to converge exactly
without a proportional-velocity tail. Orientation generation is unchanged.

## Timestamp-aware prediction

Only a newly accepted source-timestamped PICO frame updates target velocity and
acceleration. The 200 Hz control loop predicts from the most recent frame by
its actual receive age, capped at 15 ms:

```text
p_predict = p_frame + v_frame * h + 0.5 * a_frame * h^2
v_predict = v_frame + a_frame * h
```

Acceleration is low-pass filtered and norm-clamped to the configured Cartesian
translation acceleration limit. Repeated control samples never re-estimate it.

## PICO profile and A/B fallback

`qp_ik_pico_teleop.yaml` enables tracking gain `30`, stationary threshold
`1e-4 m/s`, and 15 ms prediction. Setting
`translation_tracking_enabled: false` and
`translation_prediction_enabled: false` restores the old behavior for A/B.

## Verification

- Existing Position-OTG tests continue to pass.
- A moving-circle regression proves tracking mode reduces synchronous error
  while preserving acceleration and jerk limits.
- TargetManager tests prove second-order frame-age prediction and no repeated
  estimator update.
- The PICO YAML loads the enabled values.
- The complete PICO worktree builds and relevant CTest targets pass without
  altering pre-existing uncommitted files outside this scope.
