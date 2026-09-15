# PICO + IMU900 Foot Pipeline Fix Design

## Goal

Make the PICO + dual-foot IMU900 pipeline safe for real-device validation. The
calibrated neutral pose must not move or rotate either PICO foot, foot pitch and
roll must be visible in MuJoCo, and the production C++ path must be covered by
the workspace build and tests.

## Topic semantics

- `/pico/smpl` remains untouched raw PICO data.
- `/pico/smpl_fused` remains a 24-pose global-frame `PoseArray`. PICO ankle
  positions and orientations are preserved. Only `LEFT_FOOT` and `RIGHT_FOOT`
  positions/orientations are replaced by the IMU-derived foot pose. Its header,
  timestamp, and frame ID remain those of `/pico/smpl`.
- `/pico/ankle_relative` is a two-pose `PoseArray` ordered left, right. Each
  orientation is `inverse(q_shank) * q_foot`, where the PICO knee orientation is
  the global shank orientation. Positions are zero because this topic expresses
  rotations only. The header matches `/pico/smpl_fused`.

This avoids mixing relative-joint and global-pose semantics inside one 24-pose
array.

## Calibration and fusion math

For each side, calibration records averaged neutral values:

- PICO pelvis orientation `q_pelvis0`;
- corrected IMU orientation `q_imu0`;
- PICO foot orientation `q_foot0`;
- PICO world offset `v_world0 = p_foot0 - p_ankle0`.

Runtime relative IMU motion is:

```text
q_rel = inverse(q_pelvis) * q_imu * inverse(q_imu0) * q_pelvis0
```

The global fused foot orientation is:

```text
q_foot = q_pelvis * q_rel * inverse(q_pelvis0) * q_foot0
```

The local ankle-to-foot vector is calibrated in the final foot frame:

```text
v_local = inverse(q_foot0) * v_world0
p_foot  = p_ankle + q_foot * v_local
```

Therefore at the calibration pose `q_rel = identity`, `q_foot = q_foot0`, and
`p_foot = p_foot0`. Automated tests must enforce this invariant.

Mount corrections remain independently configurable for left and right. Zero,
non-finite, or malformed input quaternions are rejected instead of converted to
identity.

## Calibration state and error handling

The calibration service succeeds only after a recent 24-pose PICO frame and
both recent IMU messages exist. Samples are accepted only while pelvis and foot
angular change stay below configurable stability thresholds. Calibration is
blocked while either IMU ready topic is false, including startup, invalid
orientation, serial loss, or driver reset/reconnect.

Before calibration, with stale inputs, or during reset, the node publishes no
fused output and emits throttled diagnostic warnings. Reset clears all baselines
and requires calibration again.

## IMU900 driver

The vendored driver remains behaviorally aligned with the reference driver and
publishes one ready boolean per serial channel. The runtime profile uses the two
standalone IMU900 serial ports and the standard binary protocol.

- Commands stored with `\n` set `startup_command_append_crlf` and
  `shutdown_command_append_crlf` to false.
- Initial serial-open failure enters the same retry loop as a later disconnect.
- Driver readiness is cleared on invalid frames and serial loss; fusion uses both
  ready topics as a publication and calibration gate.
- Obsolete parameters not declared by the reference driver are removed.
- The package license is Apache-2.0, matching the copied source package.

## MuJoCo rendering

Joint spheres and capsule bones remain position-driven. Each foot also gets an
oriented box geom centered from ankle and foot positions and rotated by the
fused `LEFT_FOOT`/`RIGHT_FOOT` quaternion. The box has width and height, so roll
about the ankle-to-toe axis remains visible even when joint positions do not
change.

## Code organization and tests

Quaternion and calibration math moves into a small C++ header/library used by
the production fusion node and C++ unit tests. The unused Python fusion node and
its duplicate tests are removed.

Tests cover:

- neutral calibration reproduces PICO foot pose exactly;
- known pitch, roll, and yaw deltas;
- invalid quaternion rejection;
- stale input and reset publication gating;
- ankle-relative orientation ordering;
- IMU command suffix configuration;
- MuJoCo foot orientation conversion.

`pico_imu900_driver` is added to the normal Pixi build and test tasks. Final
verification includes both package builds, registered tests, `git diff --check`,
and a documented real-device checklist. Hardware success is not claimed until
the two physical IMUs and PICO have been exercised.
