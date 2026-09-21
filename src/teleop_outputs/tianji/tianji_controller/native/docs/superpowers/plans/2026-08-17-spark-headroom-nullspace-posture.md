# SPARK Headroom Stationary-Reference Implementation Plan

**Goal:** Stop stationary-pose skeleton noise from driving the SPARK joint
posture reference without reducing fast teleoperation response.

**Architecture:** Keep the velocity QP and posture objective unchanged. In the
headroom algorithm, accept a new joint posture reference only when both the raw
PICO palm and the retargeted SPARK palm confirm motion.

## Constraints

- Keep the velocity-QP decision and Cartesian equality unchanged.
- Keep all joint, braking, outward and collision constraints unchanged.
- Keep fixed-feedforward and historical algorithm behavior unchanged.
- Keep raw motion-intent telemetry for diagnosis.

## Completed tasks

- [x] Correlate live oscillation windows with raw and SPARK palm motion.
- [x] Reproduce the source-domain mismatch with a focused regression test.
- [x] Add independent tests for raw-palm spikes and skeleton-only jitter.
- [x] Implement the dual-source stationary gate in `src/spark_guidance.cpp`.
- [x] Run guidance and feedforward-reference unit tests.
- [x] Replay the fixed fast-motion TJVR trace and compare common source frames.
- [ ] Re-run live PICO stationary hold to verify the intermittent endpoint
      oscillation is removed on the original hardware/input path.

## Rejected experiments

- Exact-nullspace QP posture objective: worsened tracking and constraint usage.
- Zero-headroom posture deactivation: caused posture drift and headroom lockout.
- Hard rejection of large IK branch changes: rejected persistent valid targets
  and caused severe left-arm tracking loss.
