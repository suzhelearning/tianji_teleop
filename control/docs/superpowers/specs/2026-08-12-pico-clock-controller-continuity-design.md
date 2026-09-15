# PICO clock and controller-continuity design

## Goal

Remove avoidable Cartesian tracking latency and discontinuities while preserving the
existing fixed-rate 200 Hz QP/OTG loop and all joint safety bounds.

## Design

PICO target age uses the receiver's monotonic timestamp. The control integration
step remains the configured fixed `dt`; wall time is used only for target freshness,
prediction, and command timestamps. Translation prediction is capped at its configured
horizon, but the capped target twist remains available until the target timeout. At
timeout the target is frozen exactly and its twist becomes zero. Configuration rejects
a prediction horizon longer than the timeout.

Velocity and acceleration controllers expose a shared reference motion state containing
`q`, `qdot`, and `qddot`. Viewer switches transfer that state into the newly active
controller. QP algorithm changes reset solver internals without erasing valid motion
history.

On a numerical QP failure, the solver receives one cold-start retry. If the retry still
fails, the controller advances with a bounded fallback: velocity control continues the
last valid acceleration for the failed cycle and projects that velocity into the current
feasible bounds; acceleration control applies the feasible acceleration closest to
braking to zero. An emergency hold is retained only for invalid bounds or an unsafe
integrated state. Solver failures remain observable in diagnostics even when the
bounded fallback prevents a command discontinuity.

Shared TargetManager behavior is ported identically to the velocity, acceleration, and
PICO worktrees. Regression tests cover timeout boundaries, state transfer, switching,
fallback bounds, solver telemetry, and temporary-file isolation.
