# PICO + Odin Lite Review Fixes Design

## Goal

Harden the production PICO-to-Odin pipeline before further hardware testing.
The fixes cover the A-button calibration handoff, canonical skeleton frame
validation, install-safe Odin Lite odometry-only startup, bounded point-cloud
allocation, complete launch configuration, and low-risk maintainability issues
identified during the `main...feature/odin-pelvis-extrinsic-calibration` review.

## Scope

This change applies to:

- `pico_smpl_ground` and `start_pico_bridge.launch.py`;
- `odin_pelvis_calibrator` and `odin_pelvis_runtime`;
- the ROS 2 Odin Lite driver and SDK point-frame assembler;
- regression and integration tests for those paths.

It does not rewrite existing Git history, alter the saved
`T_pelvis_odin` convention, change the PICO wire protocol, or split the entire
calibrator node into new modules.

## A-button calibration handoff

The same `/pico/world_reset` event resets the ground normalizer and requests an
Odin installation calibration. The calibrator must not begin its neutral timer
while canonical `/pico/smpl` is intentionally silent.

On an A event while ready, the calibrator enters a pending-start state. It
clears PICO stationarity history and waits for a fresh `/pico/smpl` message with
`header.frame_id == "pico_ground"` plus fresh Odin input. The first valid input
pair makes the session ready and starts the neutral phase. Input-age checks do
not cancel a pending request during the expected ground-lock pause. A second A
event while an actual calibration attempt is active still cancels that attempt;
repeated events while pending are idempotent.

This preserves the existing two-second neutral, excitation, final-neutral, and
quality-validation phases while removing the 0.5-second race.

## Canonical frame contract

The installation calibrator accepts `/pico/smpl` only when its frame is
`pico_ground`. The runtime applies the same validation to both `/pico/smpl` and
`/pico/smpl_fused`. Empty or different frame IDs are rejected with throttled
diagnostics and never enter synchronization or alignment buffers.

The expected skeleton frame is a non-empty ROS parameter with default
`pico_ground`, passed through `odin_select.launch.py`. Corrected skeleton and
pelvis outputs continue using the configured output frame, also defaulting to
`pico_ground`.

## Install-safe Odin Lite configuration

The ROS 2 Odin driver installs its `config/` directory into
`share/odin_ros_driver_rev1/config`. Configuration lookup uses the installed
ament package-share path rather than the compile-time `__FILE__` source path.
An absolute configuration path remains accepted for diagnostics.

Channel configuration loading becomes fail-closed. If the explicitly selected
`control_command_odom.yaml` cannot be found, parsed, or validated, driver
initialization fails before any stream starts. It must never fall back to the
all-channels defaults. The normal full profile remains available when selected
explicitly.

The odometry-only profile continues enabling only the Odin odometry channel;
raw/SLAM point clouds, cameras, and Odin IMU remain disabled.

## Point-frame memory bound

Point-frame payload capacity is reserved with
`MaxPointPayloadBytes(is_slam)`, exactly one 256x192 logical frame. Existing
append checks remain authoritative: impossible UDP gaps or oversized payloads
drop and reset the frame without invoking a callback.

## Launch configuration

`start_pico_bridge.launch.py` exposes and forwards:

- `smpl_raw_topic`;
- `smpl_topic`;
- `world_reset_topic`;
- `smpl_output_frame`.

Defaults remain `/pico/smpl_raw`, `/pico/smpl`, `/pico/world_reset`, and
`pico_ground`. Existing foot geometry and stability parameters are unchanged.

`odin_select.launch.py` exposes the expected skeleton input frame and forwards
it to the runtime. The standalone calibrator keeps its strict canonical topic
contract and uses the same default frame.

## Maintainability changes

Runtime fields currently named `raw_*` but carrying canonical `/pico/smpl`
data are renamed `canonical_*`. Stream-selection booleans that affect multiple
queues or publishers are replaced by small enums for skeleton kind and Odin
rate. The duplicated rigid-body `skew()` helper moves into the shared SE(3)
utility.

The calibrator class is not broadly decomposed in this fix. Such a refactor is
not required for correctness and would expand hardware-test risk.

## Error handling

- A pending calibration waits for canonical input instead of timing out.
- Wrong-frame skeletons are dropped, never relabelled.
- Missing or invalid Odin channel configuration prevents driver startup.
- Invalid point-frame sequences reset bounded assembler state.
- Existing valid installation YAML is never modified by these failures.

## Testing

Tests are added before production changes and must demonstrate the original
failure:

1. An A event followed by more than 0.5 seconds without canonical SMPL remains
   pending, then starts calibration when canonical SMPL resumes.
2. The calibrator and runtime reject `pico`, empty, and other skeleton frames;
   `pico_ground` remains accepted.
3. The installed package contains `control_command_odom.yaml`, and loading a
   missing explicitly selected profile fails closed.
4. Point-frame payload reservation and append behavior stay within one frame.
5. Bridge and Odin launch files expose and forward the new parameters.
6. Existing height, rigid re-anchoring, restart, stale-data, raw/fused
   independence, and unrelated-device-clock tests remain green.

Final verification runs `pixi run build`, `pixi run test`, and verbose colcon
test-result aggregation for the complete selected workspace.
