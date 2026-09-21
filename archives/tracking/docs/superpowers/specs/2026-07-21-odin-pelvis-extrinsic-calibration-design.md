# Odin Lite–Pelvis Extrinsic Calibration Design

## Goal

Add a one-time, interactive calibration for the rigid transform between an Odin Lite mounted on the back of the pelvis and the anatomical pelvis reported by PICO. Persist the validated mounting transform and use it during normal PICO or PICO+foot-IMU operation.

The design must also handle the fact that Odin initializes its own position, including `z`, at zero on every startup. The persisted mounting transform and the per-run world-origin alignment are separate states.

## Coordinate conventions

The anatomical pelvis frame uses:

- `+X`: human forward;
- `+Y`: human left;
- `+Z`: upward.

The persisted transform is named `T_pelvis_odin`. It is the pose of the Odin sensor frame expressed in the pelvis frame and maps Odin-frame coordinates into pelvis-frame coordinates.

The approximate physical installation is only an optimization prior: Odin is behind the pelvis, its forward axis is approximately opposite the human forward axis, and its vertical axis is approximately upward. The solver must not hard-code an exact axis mapping.

For a raw Odin pose `T_odin_world_odin`, the unanchored pelvis pose is:

```text
T_odin_world_pelvis
  = T_odin_world_odin × inverse(T_pelvis_odin)
```

## Scope

The feature consists of two C++ executables in `pico_odin`:

1. `odin_pelvis_calibrator`: an interactive, one-time trajectory calibration program;
2. `odin_pelvis_runtime`: a normal-operation transform and skeleton-root anchoring node.

Raw topics remain unchanged. The feature adds corrected outputs and does not modify PICO bridge decoding or foot-IMU fusion mathematics.

## Calibration inputs and trigger

The calibrator subscribes directly to:

- `/pico/smpl` (`geometry_msgs/msg/PoseArray`), with pose index 0 as the PICO pelvis;
- `/raw/odom/odin_highfreq` (`nav_msgs/msg/Odometry`).

The user starts calibration by pressing lowercase `a` in the calibrator terminal. This is a keyboard command, not a PICO controller event. PICO controller A and the existing `/pico/world_reset` behavior are unchanged.

The calibrator never consumes corrected pelvis or Odin-anchored skeleton topics. A `/pico/world_reset` received during sample collection invalidates and clears that attempt, because the PICO world frame changed during calibration.

## Calibration state machine

### 1. Ready check

Before accepting keyboard `a`, the calibrator verifies that both inputs are present, finite, monotonic, and recent. It checks normalized quaternions and reports input rates and timestamp behavior.

### 2. Initial neutral window

After `a`, the user stands upright and still for two seconds. This window establishes noise levels and confirms that the initial pose is suitable for calibration.

### 3. One excitation sequence

The terminal asks the user to perform one continuous sequence:

1. slowly lean the pelvis/body forward;
2. slowly lean backward;
3. return upright;
4. rotate slightly left;
5. rotate slightly right;
6. return upright and remain still.

Only one forward/back cycle and one left/right cycle are required. The program recognizes completion when the required pitch and yaw excursions have occurred and a final two-second stationary window is observed. The default maximum attempt duration is configurable.

Initial defaults are a minimum 12-degree pitch range, a minimum 12-degree yaw range, at least 80 synchronized pose pairs, and a 30-second attempt timeout. All are ROS parameters so hardware testing can adjust them without changing the algorithm.

Pitch motion makes the main rearward/vertical lever arm observable. The small yaw motion prevents the rotation and translation solve from becoming degenerate. If either excitation is insufficient, the attempt is rejected instead of saving a weak solution.

### 4. Temporal alignment

PICO and Odin Lite expose independent device-monotonic timestamps with unrelated epochs. Their `header.stamp` values are therefore never compared across streams. Each callback records a host `steady_clock` receipt timestamp; the original device timestamp is retained only for monotonicity, restart, and same-stream continuity checks.

The calibrator estimates the residual transport lag between the two host-receipt timelines using motion-magnitude correlation, then interpolates the high-rate Odin trajectory to each PICO receipt timestamp using linear translation and quaternion slerp. The default lag search range, maximum interpolation gap, and correlation quality are parameters. A lag at the search boundary or poor correlation rejects the attempt.

The residual lag is session state, not an installation property. It is never persisted in the extrinsics file. During normal operation the C++ runtime begins with zero receipt-time lag, which is sufficient for the stationary per-run world alignment, and continuously estimates the current run's lag when observable motion becomes available. Until a valid estimate exists, it pairs by host receipt time. Accepted lag updates are bounded and smoothed before affecting interpolation.

### 5. Hand-eye solve

For synchronized samples, form relative motions:

```text
A_ij = inverse(O_i) × O_j
B_ij = inverse(P_i) × P_j
```

The fixed sensor-to-pelvis transform `X = inverse(T_pelvis_odin)` satisfies:

```text
A_ij × X = X × B_ij
```

The initial solve uses a standard quaternion/SVD rotation solution followed by a linear least-squares translation solution. A robust refinement minimizes SE(3) hand-eye residuals across many sample pairs.

The refinement estimates a complete rotation but uses the approximate rear-facing installation as a soft prior. Translation is expressed and reported in the pelvis frame: `x` and `z` are optimized from the trajectory, while `y` has a strong zero-centered prior. Broad, configurable physical bounds prevent implausible back-mounted solutions without forcing exact installation dimensions.

The previous valid calibration may initialize the optimizer, but it is never treated as a measurement and is never modified unless the new result passes validation.

## Calibration validation

The result is accepted only when all checks pass:

- sufficient pitch and yaw excitation;
- enough synchronized sample pairs;
- acceptable host-receipt lag correlation and interpolation gap;
- well-conditioned hand-eye equations;
- rotation and translation residuals below configurable limits;
- solved position inside configurable back-mounted physical bounds;
- solved rotation reasonably close to the broad installation prior;
- the final neutral window predicts an upright pelvis with the expected right-handed axes.

Every rejection reports the failing metric and leaves the previous file untouched.

## Persisted calibration

The default path is portable and outside the source tree:

```text
~/.config/pico_tracker/odin_pelvis_extrinsics.yaml
```

The path can be overridden by a ROS parameter or launch argument. Saving uses a temporary file, validation reread, and atomic rename.

The YAML contains:

- schema version and `valid: true`;
- explicit `T_pelvis_odin` convention text;
- translation in metres;
- quaternion in explicit `xyzw` order;
- row-major 4×4 matrix for inspection;
- source topics and calibration timestamp;
- sample count and temporal-correlation quality;
- pitch/yaw excitation ranges;
- rotational and translational residual metrics;
- solver condition metrics.

It does not contain a PICO initial height or a per-run world alignment.
It also does not contain a PICO/Odin device-clock or transport-delay offset. Legacy schema files that contain `time_offset_sec` remain readable, but runtime synchronization ignores that field.

## Per-run world alignment and height

Odin starts each run with its position, including `z`, at zero. Therefore the runtime node must not interpret the corrected Odin `z` as absolute pelvis height.

After loading `T_pelvis_odin`, the runtime node waits for a stable host-receipt-time synchronized PICO pelvis and Odin window. Let:

```text
C_0 = T_odin_world_odin(0) × inverse(T_pelvis_odin)
P_0 = initial PICO pelvis pose
```

The per-run world alignment is:

```text
S = P_0 × inverse(C_0)
```

Normal output is:

```text
T_output_pelvis(t)
  = S × T_odin_world_odin(t) × inverse(T_pelvis_odin)
```

Consequently, the initial output height is the PICO pelvis height and subsequent height is that initial height plus the installation-corrected Odin displacement. Rotation-induced lever-arm displacement is included by the full transform composition.

The per-run alignment exists only in memory. It is rebuilt:

- automatically from the first stable synchronized window after startup;
- after `/pico/world_reset` using the next stable synchronized window;
- after Odin timestamps, frame IDs, or pose continuity indicate a driver/device restart.

None of these events writes the persisted installation transform.

## Runtime outputs

The runtime node subscribes to both raw Odin odometry rates and to the optional raw and foot-fused PICO skeleton topics. It publishes:

- `/calibrated/odom/pelvis`;
- `/calibrated/odom/pelvis_highfreq`;
- `/pico/smpl_odin` when `/pico/smpl` is available;
- `/pico/smpl_fused_odin` when `/pico/smpl_fused` is available.

Raw inputs remain unchanged.

For each skeleton frame, the runtime node interpolates the high-rate Odin buffer at the synchronized host-receipt timestamp, computes the rigid root change from the live PICO pelvis to the Odin-derived pelvis, and applies the same transform to all 24 joint positions and orientations. This preserves the skeleton geometry and prevents root-only replacement from tearing the body.

The output frame ID is the PICO-aligned runtime world frame. Frames with stale or unavailable synchronized Odin data are dropped with throttled diagnostics. Corrected pelvis odometry can continue through temporary PICO loss after the per-run alignment is established, but skeleton output still requires a PICO skeleton frame.

Odometry twist and populated covariance fields are transformed using the rigid-body adjoint. Unknown covariance remains explicitly unknown rather than being invented.

## Launch integration

`pico_odin/odin_select.launch.py` gains parameters for:

- enabling the pelvis runtime node;
- selecting the extrinsics file;
- corrected topic names;
- synchronization and session-alignment thresholds.

The `enable_pelvis_runtime` launch argument accepts `auto`, `true`, or `false` and defaults to `auto`: enabled for Odin Lite and disabled for Odin1. When enabled, the node waits safely for PICO topics. The existing bridge and foot-IMU launch files do not duplicate the transform implementation. Whichever skeleton topic is active produces its corresponding corrected output.

The standalone calibration flow starts PICO bridge and Odin Lite with `enable_pelvis_runtime:=false`, then runs `odin_pelvis_calibrator`. This keeps calibration terminals focused on raw data. The calibrator still enforces raw input topic names so a corrected topic cannot accidentally feed the solve.

## Failure behavior

- Missing or invalid calibration file: raw topics continue; corrected outputs remain disabled with a clear error.
- Input timeout, non-finite pose, or invalid/non-normalized quaternion: reject the affected frame and report a throttled warning.
- PICO world reset during installation calibration: cancel and clear the attempt.
- Odin restart during installation calibration: cancel and clear the attempt.
- Odin restart during normal operation: clear only the in-memory session alignment and reacquire it.
- Insufficient excitation or ill-conditioned solve: do not save.
- Failed atomic save or failed reread: preserve the previous file and report failure.

## Testing

### Unit tests

- Transform convention and inversion tests for `T_pelvis_odin`.
- Synthetic hand-eye trajectories with a known rearward/vertical/pitch mounting transform.
- Noisy, time-offset synthetic data with one pitch cycle and one yaw cycle.
- Degenerate static, pitch-only, insufficient-range, and timestamp-failure rejection tests.
- Runtime height test: Odin starts at `z=0`, PICO pelvis starts at a nonzero height, and Odin delta height is added correctly.
- Session reset tests proving that world reset changes only in-memory alignment.
- YAML round-trip, schema validation, and failed-save preservation tests.
- Whole-skeleton rigid re-anchoring tests for all 24 positions and orientations.

### ROS integration tests

- Synthetic publishers at PICO and Odin rates exercise interactive-state logic without hardware.
- Raw topics remain unchanged while corrected topics use the calibrated transform.
- Both `/pico/smpl_odin` and `/pico/smpl_fused_odin` are produced independently when their sources exist.
- Missing calibration and stale synchronization suppress corrected output without stopping raw drivers.
- PICO and Odin source timestamps with deliberately unrelated epochs still synchronize through host receipt time.
- Non-finite positions and invalid quaternions never enter synchronization buffers or corrected outputs.
- Odin restart detection on either odometry rate clears the shared in-memory session alignment before either corrected odometry rate can publish again.

### Hardware validation

1. Measure the approximate Odin rearward and vertical offsets for comparison only.
2. Run one calibration excitation sequence and inspect saved quality metrics.
3. Verify corrected initial pelvis height equals PICO height.
4. Verify forward, left, and upward motions have the expected signs.
5. Verify pitch, roll, and yaw directions individually.
6. Restart Odin and confirm the persistent installation transform is unchanged while session height alignment is reacquired.
7. Re-run installation calibration and confirm the old file is replaced only after validation succeeds.

The final verification runs the complete workspace build and test suite in addition to the new package tests.

## Git workflow

Development and testing occur on `feature/odin-pelvis-extrinsic-calibration`. Existing unrelated worktree changes are preserved and excluded from feature commits.
