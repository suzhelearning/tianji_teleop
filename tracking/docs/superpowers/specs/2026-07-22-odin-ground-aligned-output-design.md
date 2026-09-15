# Canonical Ground-Aligned PICO SMPL Design

## Goal

Make `/pico/smpl` the canonical body skeleton for this project, expressed in a
normal ground frame: the floor is `Z=0`, both foot soles rest on the floor in
the initial neutral stance, and pelvis/head heights are positive. Odin
calibration, Odin runtime correction, foot-IMU fusion, recording, and MuJoCo
must all consume this canonical skeleton.

Preserve the APK payload as `/pico/smpl_raw` for diagnostics.

## Topic Contract

- `/pico/smpl_raw`: the 24 poses received from the PICO APK without geometric
  modification. Its frame remains the APK's head-origin PICO frame.
- `/pico/smpl`: the same 24 poses translated into `pico_ground`, with the
  locked initial foot-sole height at `Z=0`.

`pico_bridge` no longer publishes APK payloads directly as `/pico/smpl`.
Instead it publishes `/pico/smpl_raw`. A dedicated C++ ground-normalization
component subscribes to `/pico/smpl_raw` and `/pico/world_reset`, then publishes
the canonical `/pico/smpl`.

Using a separate component keeps TCP decoding independent from calibration
state and prevents an accidental subscription/publish loop.

## A-Button State Machine

The PICO APK keeps its existing A-button behavior: it establishes a
head-origin, level, Z-up coordinate frame and sends the `0x06` world-reset
event. `pico_bridge` continues to publish that event as `/pico/world_reset`.

The normalizer behaves as follows:

1. **Uncalibrated:** on startup, publish no `/pico/smpl`; wait for an A-button
   event. This prevents pre-reset Unity coordinates from entering downstream
   processing.
2. **Collecting:** on `/pico/world_reset`, discard the previous floor and all
   queued samples. Accept only complete, finite 24-pose raw frames received
   after the event.
3. **Stable lock:** collect a rolling window of 30 two-foot samples. A window
   is valid when all left/right sole heights span no more than `0.02 m` and the
   two feet agree within the configured tolerance.
4. **Publishing:** lock the median sole height as the session floor. Translate
   every joint position by `(0, 0, -floor_height)` and publish it as
   `/pico/smpl`. Preserve every joint orientation and X/Y coordinate.
5. **Reset:** a later A-button event immediately stops canonical publication,
   clears the previous lock, and starts collection again.

The locked floor never follows walking, crouching, jumping, or lifting a foot.

## Foot-Sole Estimate

Use SMPL poses 10 and 11 as left and right foot frames. Compute each sole's
lowest world-Z point from its position, normalized orientation, and
configurable box half-extents. This accounts for initial foot pitch and roll;
subtracting a constant Z thickness would be incorrect when a foot is tilted.

Defaults match the raw PICO foot geometry used by the MuJoCo viewer:

```text
half extents = [0.115, 0.050, 0.022] metres
stable window = 30 frames
stability tolerance = 0.02 metres
```

Invalid poses, non-normalizable quaternions, missing joints, and unstable feet
do not produce a floor lock. The node logs a throttled reason and continues
waiting.

## Coordinate Semantics

The transform is translation-only:

```text
p_ground = p_raw + [0, 0, -floor_height]
q_ground = q_raw
```

It does not level the pelvis, change heading, or alter body proportions. The
canonical output uses `header.frame_id = "pico_ground"`.

Expected neutral-standing values are approximately:

```text
foot soles: Z = 0
pelvis:      Z = 0.8 to 1.1 m
head:        Z = 1.5 to 1.9 m
```

## Downstream Integration

Existing downstream topic names remain unchanged:

- `pico_foot_imu_fusion` subscribes to `/pico/smpl` and therefore produces a
  ground-aligned `/pico/smpl_fused`;
- `odin_pelvis_calibrator` calibrates `T_pelvis_odin` from the ground-aligned
  `/pico/smpl` and raw Odin odometry;
- `odin_pelvis_runtime` initializes from `/pico/smpl` or
  `/pico/smpl_fused`, so `/pico/smpl_odin` and `/pico/smpl_fused_odin` retain
  the same ground origin;
- recorders continue recording `/pico/smpl`, now with the documented
  `pico_ground` frame semantics;
- MuJoCo treats canonical and derived skeletons as ground-aligned and places
  its plane at world `Z=0`.

The persisted `T_pelvis_odin` remains only a rigid mounting transform. Floor
height is per-session state and is never saved in the extrinsics YAML.

`/pico/pose/head`, `/pico/pose/left_hand`, and `/pico/pose/right_hand` remain
raw APK pose topics in this change. Their frame is not relabeled as
`pico_ground`.

## Launch Behavior

Normal PICO launch files start the ground normalizer alongside `pico_bridge`.
Parameters are exposed for:

- raw and canonical topic names;
- output frame name;
- foot half-extents;
- stable window size;
- stability tolerance;
- whether an A-button event is required before the first lock.

The production default requires A. A test/diagnostic override may allow an
automatic first lock, but must not be enabled by default.

## Failure Behavior

- Before A or before a valid floor lock, `/pico/smpl` is intentionally silent.
- A reset interrupts publication immediately, so downstream nodes cannot mix
  old and new coordinate frames.
- If the APK disconnects, no synthetic skeleton is published.
- If floor lock cannot be established, raw data remains available on
  `/pico/smpl_raw` for diagnosis.
- Odin and foot-IMU state machines continue responding to the same world-reset
  event and naturally wait for canonical `/pico/smpl` to resume.

## Compatibility and Documentation

This deliberately changes the meaning and `frame_id` of `/pico/smpl`. Update
the English and Chinese READMEs, test-command guide, topic tables, and MuJoCo
examples. Document `/pico/smpl_raw` as the only head-origin diagnostic stream.

Do not silently remap old recordings. A recorded message's `frame_id` decides
whether it is raw `pico` or canonical `pico_ground` data.

## Verification

- Unit-test oriented foot-sole calculation, stable-window locking, unstable
  rejection, invalid data, reset, and translation of all 24 poses.
- Integration-test that startup is silent until A, A clears an old lock, and
  exactly post-reset frames can establish the new floor.
- Prove `/pico/smpl_raw` matches the decoded APK positions/orientations while
  `/pico/smpl` differs only by one common Z translation.
- Prove foot-IMU fusion and Odin calibration/runtime continue consuming
  `/pico/smpl` without topic remaps and preserve the ground frame.
- Prove MuJoCo keeps its plane at `Z=0` for canonical and Odin-derived topics.
- Run the focused tests, full project build, and full test suite before
  real-device testing.

## Acceptance Criteria

- Before A, `/pico/smpl` publishes no messages.
- Standing still after A establishes a floor within the 30-frame window and
  publishes both foot soles at `Z=0` within tolerance.
- Pelvis and head heights are positive and anatomically plausible.
- Walking does not move the floor.
- A second reset produces no mixed-frame output and recomputes the floor.
- `/pico/smpl_raw` remains geometrically identical to the APK body payload.
- Odin calibration and corrected skeletons are based on canonical
  ground-aligned `/pico/smpl`.
