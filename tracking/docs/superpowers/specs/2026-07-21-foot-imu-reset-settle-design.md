# Foot IMU Reset Settling Design

## Goal

Make PICO-triggered foot calibration follow the reference torso-IMU pattern:
perform the hardware reset first, allow the IMU900 AHRS to settle, then capture
a software calibration baseline. The foot pipeline uses a shorter one-second
settling interval and sixty calibration frames.

## Trigger and reset sequence

The existing `/pico/world_reset` event remains the only automatic trigger. On a
new event, `pico_foot_imu_fusion` invalidates the current fused baseline and any
calibration samples, then starts a new generation of the transaction.

The node sends the existing IMU900 `zero_z_axis` commands sequentially:

1. call `/im900/left_foot/zero_z_axis` and require a successful ACK;
2. call `/im900/right_foot/zero_z_axis` and require a successful ACK;
3. wait `imu_reset_settle_sec`, which defaults to `1.0` second.

The reset command remains `zero_z_axis`. This design does not add
`restore_world_axes` because the reference IMU900 torso configuration also uses
`zero_z_axis` by default.

## Post-reset sampling

IMU messages received during the settling interval must not become calibration
inputs. When the one-second timer expires, the fusion node rejects both cached
IMU orientations and waits for a new valid sample from each ready channel.

After both post-settle samples are fresh and a valid 24-pose PICO frame is
available, the node collects `calibration_samples=60` stable frames using the
existing motion rejection and baseline calculation. `/pico/smpl_fused` remains
suppressed until calibration completes.

## Cancellation and failure behavior

- A repeated `/pico/world_reset` cancels the previous generation, cancels its
  timer, and restarts the sequence from the left IMU reset.
- A failed or unavailable reset service fails the transaction and leaves no
  fused baseline.
- An IMU ready transition to false, invalid orientation, or stale data during
  calibration cancels sampling and leaves no baseline.
- `/pico_foot_imu_fusion/reset` cancels the automatic transaction and timer.
- The manual `/pico_foot_imu_fusion/calibrate` service keeps its current
  behavior and does not send hardware commands or apply the settle delay.

## Configuration and observability

`start_pico_foot_fusion.launch.py` exposes:

- `imu_reset_settle_sec`, default `1.0`;
- `calibration_samples`, default changed from `90` to `60`.

Logs distinguish reset ACK completion, the one-second settling phase,
post-settle fresh-data detection, sampling start, and calibration completion.
The test command documentation uses the same defaults.

## Verification

Automated tests cover the settling state transition, stale timer generations,
timer cancellation, launch defaults, and the existing reset/ACK flow. The full
workspace build and test suite must pass. Real-device validation presses A while
standing still and verifies that sampling begins about one second after both
ACKs, uses sixty frames, and only then resumes `/pico/smpl_fused`.
