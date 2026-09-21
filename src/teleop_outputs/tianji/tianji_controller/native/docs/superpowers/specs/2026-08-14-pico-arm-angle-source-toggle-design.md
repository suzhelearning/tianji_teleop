# PICO Arm-Angle Source Runtime Toggle Design

## Goal

Provide a fair live A/B comparison between PICO skeleton-guided arm angle and
the fixed default-down arm angle used by the baseline Viewer. The toggle must
leave PICO end-effector poses, Cartesian OTG, QP formulation, gains, limits,
reference integration, and MuJoCo model unchanged.

## Considered approaches

1. **Control-thread source toggle (selected).** Send a Viewer command through
   the existing SPSC command queue and switch only the requested arm-direction
   references. This preserves thread ownership and isolates the experimental
   variable.
2. Toggle `arm_angle.enabled` and rebuild the controllers. This disables the
   arm-angle task instead of reproducing the baseline default-down task, so it
   is not a fair comparison.
3. Maintain separate YAML files or executables. This risks unrelated parameter
   drift and makes live comparison slower and less reproducible.

## Behavior

- PICO arm-angle mode is the startup default, preserving current behavior.
- Pressing `G` toggles between `pico` and `default_down` arm-angle sources.
- PICO end-effector pose processing continues in both modes.
- When PICO is stale or disabled, the effective arm-angle reference remains the
  default-down fallback regardless of the selected mode.
- A source toggle does not reset `ArmDirectionReferenceManager`; its existing
  angular rate limit provides a continuous transition between references.
- PICO epoch resets retain their existing full reset behavior.

## Data flow

```text
GLFW G key
  -> ViewerCommand::kTogglePicoArmAngleSource
  -> control-thread pico_arm_angle_source_enabled
  -> select requested arm directions
       PICO live and enabled: latest PICO arm directions
       otherwise: default-down references
  -> existing ArmDirectionReferenceManager rate limiter
  -> existing velocity- or acceleration-level QP arm-angle task
```

## Observability

- Add a boolean to `ViewerSnapshot` showing the requested PICO arm-angle mode.
- Show `arm angle: pico/default-down` and the `G` shortcut in the Viewer
  overlay.
- Add the mode to the headless completion summary for integration tests.
- Keep the existing per-arm telemetry fields (`left_arm_angle_source` and
  `right_arm_angle_source`) as the record of the effective source applied to
  each QP.

## Testing

- Unit-test command/snapshot transport for the new command type and mode flag.
- Extend the PICO Viewer integration sequence to toggle the source and assert
  that the headless summary reports default-down mode.
- Verify the focused integration tests, then run the complete CTest suite.

## Non-goals

- No changes to PICO relative mapping, Cartesian OTG, QP decision variables,
  arm-angle gains, joint bounds, collision handling, or robot model.
- No attempt to tune which source performs better in this change.
