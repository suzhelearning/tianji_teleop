# MuJoCo Diagnostic Environment and Ground Design

## Status

This specification supersedes the earlier fixed-`Z=0` ground design in commit
`bf24cf0`. Live data showed that pressing the PICO A button places the PICO
world origin near the head, so physical floor height is not generally world
`Z=0`.

## Goal

Make the SMPL MuJoCo viewer useful for diagnosing PICO/Odin alignment by
showing a clear reference environment, a stable estimated floor, and explicit
world and pelvis coordinate axes.

## Scope

- Keep the primary skeleton and optional raw PICO overlay behavior.
- Improve only `smpl_mujoco_visualizer`; do not alter Odin calibration or
  runtime transforms in this change.
- Preserve the existing command-line interface.
- Use the reference `exo_body_viewer` scene style without importing its robot
  model or dependencies.

## Scene Appearance

The generated MuJoCo model will contain:

- a light sky/background;
- a high-contrast checkerboard ground material;
- a directional light plus a balanced headlight;
- a ground plane shared by the primary and raw skeletons;
- a world coordinate triad at the locked floor origin;
- a pelvis coordinate triad for the primary skeleton;
- when `--show-raw` is enabled, a second pelvis triad for the raw PICO
  skeleton.

Axis colors follow the standard convention: X red, Y green, and Z blue. The
primary pelvis axes are opaque; the raw pelvis axes use thinner, translucent
geometry so the two frames remain distinguishable.

## Floor Estimation

### Source

When `--show-raw` is enabled and valid raw samples exist, floor estimation uses
the raw `/pico/smpl` foot poses. This prevents an incorrect Odin transform from
moving the visual reference used to diagnose that transform. Otherwise it uses
the primary skeleton.

### Foot-Sole Height

For each left and right foot box, compute the lowest world-Z point from its
center, orientation, and box half-extents. The calculation accounts for foot
pitch and roll rather than subtracting a fixed vertical thickness.

### Stable Lock

- Collect a rolling window of 30 complete two-foot samples.
- A window is stable when the peak-to-peak range of all sole heights is at most
  `0.02 m`.
- When stable, lock the ground height to the median of all sole heights in the
  window.
- Hide the plane and world axes until the first successful lock so an arbitrary
  floor is never presented as measured truth.
- After locking, never move the ground in response to walking or foot motion.

### Reset

Subscribe to `/pico/world_reset`. Each received event clears the previous
floor estimate, hides the ground, and starts a new 30-frame stable lock. This
matches the PICO A-button world reset and Odin calibration workflow.

If samples are invalid or unstable, retain the unlocked state and continue
collecting. Skeleton rendering remains available while the estimator waits.

## Camera

Keep the current one-time camera initialization around the primary pelvis.
Adopting the reference viewer's fixed world look-at would often miss this
project's head-origin PICO coordinate frame. Scene lighting and materials are
copied conceptually; camera tracking behavior is not.

## Diagnostic Boundary

The viewer must display the incoming poses faithfully. It must not silently
rotate or level `/pico/smpl_odin`. Live synchronized samples already show that
the corrected pelvis can have a large pitch and place the feet above the
pelvis. World and pelvis axes will make that upstream transform error visible;
the Odin extrinsic/runtime fix will be a separate change after real-device
inspection.

## Testing

- Unit-test checkerboard, lighting, hidden-ground, and axis geometry in the
  generated XML.
- Unit-test oriented foot-box sole-height calculation.
- Unit-test stable-window locking, unstable-window rejection, and reset.
- Unit-test preference for raw samples when both streams are available.
- Run focused viewer tests, project build, and the full test suite.

## Acceptance Criteria

- The floor is visually clear and does not appear at world `Z=0` merely by
  assumption.
- Standing still after PICO reset locks the plane once at the measured sole
  height.
- Moving afterward does not drag the plane with either foot.
- Pressing PICO A clears and recomputes the plane.
- World and raw/corrected pelvis axis direction can be compared directly.
- Existing raw-overlay and primary-only commands continue to work.
