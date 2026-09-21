# Live Headroom, Intent, and Telemetry Fix Design

## Goal

Fix the measured live PICO failure mode without changing the Velocity-Level QP,
SPARK mapping, model-state feedback, or any joint velocity, acceleration, and
jerk limit.

## Evidence

The 126.43 s live capture solved every QP and violated no active bound. However,
instantaneous jerk was the headroom minimum for about 79.5% of cycles, effective
feedforward was nonzero for only 20.6% left and 16.1% right, and the full-band
PICO twist exceeded the stationary thresholds for more than 99% of cycles.
The main telemetry also reported zero velocity-level acceleration usage and NaN
jerk bounds even though the Velocity QP was enforcing hard jerk bounds.

## Design

### Trustworthy velocity-level telemetry

The Viewer derives velocity-level acceleration usage from the controller's
accepted `qddot_prev` and `joint_limits.max_acceleration_rad_s2`. Plot and CSV
jerk bounds use `joint_limits` in velocity mode and
`joint_acceleration_limits` in acceleration mode. This is diagnostics-only and
does not alter the QP feasible set.

### Persistent jerk headroom

The headroom governor keeps the instantaneous jerk usage for safety evidence,
but gates feedforward with a bounded exponential usage envelope. Usage attacks
over 50 ms and releases over 150 ms. A single jerk-limited cycle therefore does
not close feedforward, while sustained saturation still converges toward zero
headroom. Velocity, acceleration, and task-scale headroom remain unchanged and
can still immediately dominate.

### Asymmetric stationary intent

`SparkPalmTwistDecision` exposes the existing low-frequency twist. Settled-hold
entry and stationary joint-reference hold use this low-frequency intent and the
existing 0.30 s dwell. Hold release continues to use the full-band twist from
both the SPARK target and raw PICO palm. Thus tremor/noise cannot prevent entry,
while deliberate motion retains the fast release path.

The configured entry thresholds become 0.02 m/s and 0.10 rad/s. Release uses
twice those thresholds and requires corroborated motion from both sources.

### Resynchronization

The existing resynchronization path already seeds from current model state and
restarts the 150 ms target blend. Tests will assert no reference-position jump
and no stale feedforward carry-over. No extra cooldown is added unless this
test fails, avoiding unnecessary latency.

## Acceptance

- Focused and full tests pass.
- Velocity-mode CSV has nonzero acceleration usage when qddot is nonzero and
  finite configured jerk bounds.
- An isolated jerk spike does not collapse feedforward; sustained saturation
  does reduce it.
- Low-frequency stationary input enters hold after 0.30 s; deliberate motion
  releases promptly.
- Fixed 2134-frame replay has no failures or hard-bound violations and improves
  effective feedforward availability without more than 5% position-error P95
  regression.
- No Git commit is created.
