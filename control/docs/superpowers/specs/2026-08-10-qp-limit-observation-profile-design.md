# QP Interactive-Tracking Profile Design

## Goal

Improve interactive endpoint tracking without relying on excessively large
pose-error gains. Estimate target velocity from successive target poses in the
world frame, low-pass filter it, and add it as feedforward before the final
Cartesian velocity clamps and QP joint constraints.

## Configuration

The opt-in `config/qp_ik_qp_limit.yaml` profile uses:

```yaml
cartesian_servo:
  kp_position: [15.0, 15.0, 15.0]
  kp_orientation: [10.0, 10.0, 10.0]
  kff_linear: 0.8
  kff_angular: 0.8
  feedforward_filter_cutoff_hz: 15.0
  prediction_horizon_seconds: 0.015
  target_timeout_seconds: 0.075
  max_linear_velocity: 1.2
  max_angular_velocity: 4.0

joint_limits:
  margin_rad: 0.05
  velocity_scale: 1.00
  max_acceleration_rad_s2: 20.0
  braking_acceleration_rad_s2: 15.0

hierarchical_qp:
  slack_position_scale: 1.0
  slack_orientation_scale: 2.0

safety:
  reference_tracking_warn_rad: 0.10
  reference_tracking_stop_rad: 0.20
```

The standard `config/qp_ik_hierarchical.yaml` remains unchanged. Profiles
without the optional feedforward keys use zero feedforward and retain their
existing behavior.

## Control Flow

Each manual target command carries a monotonic source timestamp. The target
manager updates its twist estimator only when a newer source frame arrives and
uses the source timestamp delta with `poseErrorWorld`. A first-order 15 Hz
low-pass filter suppresses pose-difference noise. The 200 Hz control loop
predicts at most 15 ms from the last received frame; feedforward becomes zero
after that horizon, input older than 75 ms is marked stale for telemetry, and
hold mode publishes zero target twist.

The Cartesian servo computes:

```text
v_des = Kp_position * position_error + kff_linear * target_linear_velocity
w_des = Kp_orientation * orientation_error + kff_angular * target_angular_velocity
```

All terms use the world frame, matching the TCP Jacobian. Linear and angular
norm clamps are applied after feedback and feedforward are combined. The
resulting desired twist then enters the existing hierarchical QP.

Joint bounds intersect model velocity, one-step position, braking-distance,
and commanded-acceleration limits. Position and braking safety override a
conflicting comfort acceleration bound. Slack Hessian weights are divided by
the square of their linear/angular scales. A reference-tracking watchdog
scales the desired twist between warning and stop thresholds, then freezes
only the arm whose position reference has run too far ahead. Independent
single-arm solver failures likewise hold only the failed arm; shared malformed
input still rejects both arms.

The DLS comparison backend scales from the previous velocity projected into
the current feasible box, rather than always scaling from zero. This keeps the
A/B path feasible when acceleration bounds temporarily exclude zero.

## Preserved Safety Constraints

- Keep `joint_limits.velocity_scale` at `1.00` in the opt-in profile.
- Keep `joint_limits.margin_rad` at `0.05 rad`.
- Keep every model position limit, including bilateral Joint4
  `[-2.5307, 0.0] rad`.
- Keep finite-value validation, solver validation, and QP joint safety bounds
  enabled.
- Do not change the standard hierarchical profile or benchmark thresholds.

## Verification

- Unit-test feedback/feedforward composition and post-composition clamps.
- Unit-test source-timestamp target-twist estimation, finite prediction,
  timeout, out-of-order rejection, and hold-mode reset.
- Unit-test acceleration, braking, one-step position, and velocity-bound
  intersections, including safety-over-comfort conflict handling.
- Unit-test reference watchdog and independent-arm failure containment.
- Unit-test dimensionally scaled slack Hessian weights.
- Assert zero-feedforward defaults for the unchanged standard profile.
- Run the complete CTest suite with the standard profile.
- Run the four-second interactive headless sequence with the opt-in profile;
  it must complete without control or command failures.

The earlier `50/30/100/100` experiment reached a maximum joint-velocity ratio
of `1.0`, showing that larger feedback gains mainly drove the QP into joint
bounds. This profile instead targets motion phase lag while using feedback to
remove residual pose error.

## Non-goals

- Removing Cartesian pose feedback.
- Removing Cartesian or joint-space limits.
- Changing model geometry or physical joint ranges.
- Adding task-scaling beta or strict lexicographic HQP.
- Retuning the standard profile.
