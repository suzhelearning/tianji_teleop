# PICO corrected-skeleton stale-palm handling

## Context

The corrected skeleton node publishes a raw SMPL copy before attempting each
side's palm correction. When the nearest palm sample is temporarily outside
the timestamp skew window, the side returns early and the raw copy is emitted
for that frame. This creates a discontinuity between corrected frames. The
tremor recordings showed this as a roughly 70 mm static-pose outlier and made
the corrected stream unsuitable for spectral analysis.

## Decision

For a side that already has a valid corrected result, a short `palm_stale`
condition holds the last valid corrected positions and orientations for that
side. The output is explicitly marked `palm_stale_hold`; it is not marked as
currently corrected. If no corrected result exists yet, the existing raw-SMPL
fallback remains unchanged for startup/baseline collection.

The held result is cleared whenever the tracking epoch changes. A held result
must never cross a new PICO world/calibration epoch.

The tremor analysis filters corrected-palm samples by the status topic's
per-side `corrected` flag. Therefore stale-hold samples are excluded from the
corrected spectral diagnostic, while the raw controller stream remains the
primary human-motion estimator.

## Acceptance criteria

1. A stale palm after a valid correction preserves the previous corrected
   four-joint side output and reports `palm_stale_hold` plus
   `output_mode=hold_last_corrected`.
2. A stale palm before any valid correction still emits raw SMPL data.
3. Epoch transition clears the held corrected positions and orientations.
4. Existing PICO bridge tests remain green.
5. The recorded tremor analysis reports the number of status-filtered stale
   samples and no longer treats the static corrected-palm fallback spike as a
   spectral result.
