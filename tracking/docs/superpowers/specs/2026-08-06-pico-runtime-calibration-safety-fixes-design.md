# PICO Runtime and Calibration Safety Fixes Design

## Goal

Preserve the current palm-constrained skeleton geometry while making reset,
reconnect, calibration artifact activation, and interactive calibration
fail-closed and deterministic.

## Invariants

- `T_controller_palm`, palm-local positive-X wrist geometry, calibrated arm
  lengths, and `correct_side()` mathematics do not change.
- A palm sample may only be paired with a raw skeleton sample from the same
  explicit tracking epoch.
- When an epoch changes, cached palms, learned baseline samples, and temporal
  elbow history from the old PICO world are discarded.
- Raw SMPL shoulder/elbow/wrist geometry is diagnostic evidence only. Invalid
  raw diagnostic vectors may produce warnings but may not veto otherwise valid
  palm-derived arm lengths.
- Only a schema-v3 geometry artifact that passes the same strict lineage,
  quality, covariance, and Gate validation as runtime may become active.
- Runtime does not search `recordings/` or select a candidate implicitly.
- Wrist calibration accepts only finite `pico`-frame samples from one stable,
  explicit tracking epoch.
- Ctrl-C stops and cleans up the current calibration step without leaving a
  duplicate palm publisher.
- No files under `recordings/` are modified or deleted and no Git commit is
  created by this work.

## Components

### Epoch-aware skeleton filter

`PalmSample` carries the epoch captured at callback time. Both epoch topics
must agree and use an explicit source before palms or raw skeletons are
accepted for correction. An epoch transition clears both palm caches, all
baseline-learning deques, and `previous_elbow_position`. Loaded individual
bone lengths remain valid because they are person geometry, not a world-frame
attachment; only world-dependent runtime history is reset.

While epoch state is unavailable or inconsistent, the node continues to
publish the unmodified raw skeleton and reports an epoch fallback reason.

### Diagnostic-only raw SMPL evidence

Static-pose arm lengths continue to come from palm-derived wrist positions.
Raw shoulder/elbow/wrist angles and the raw upper-arm direction are computed
opportunistically. Degenerate raw vectors yield `null`/warning diagnostics and
must not throw before structural palm gates run.

### Shared strict artifact validation

The interactive script calls a Python validator that enforces the same
geometry contract as the runtime loader and checks it against the currently
active TCP and wrist SHA-256 values. Wrist artifacts require schema, side,
convention, finite vector, sample count, RMS, and condition-number Gates.
Status output reports schema/type, revision, and lineage match.

The M0 wrapper uses only the active geometry artifacts under
`~/.config/pico_tracker/`. It never scans `recordings/`.

### Wrist calibration and process lifecycle

The wrist calibrator subscribes to explicit epoch/status topics, clears an
unfinished capture on epoch transition, and rejects wrong-frame or non-finite
poses. Finite solution/residual/condition checks precede writing `valid: true`.
The shell menu owns background publisher cleanup through a per-step trap and
returns to the menu after a failed single operation.

### Durable epoch allocation

Epoch reservation uses an advisory lock covering read-increment-write,
`fsync()`s the temporary file, atomically renames it, and `fsync()`s the parent
directory. Concurrent bridge processes therefore cannot reserve the same
epoch.

## Verification

- Unit tests reproduce epoch transitions during baseline and correction.
- Unit tests prove degenerate raw SMPL diagnostics do not veto palm geometry.
- Artifact tests reject stale hashes, bad schema, failed Gates, and non-finite
  wrist inputs.
- Shell tests verify no recordings scan, menu recovery, and publisher cleanup.
- A concurrent C++ test verifies unique epoch reservations.
- Full `pico_bridge` tests, build, Bash syntax, and `git diff --check` pass.
