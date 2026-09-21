# PICO corrected stale-hold implementation plan

## Scope

Repair the corrected skeleton stream and rerun the existing tremor analysis
for `recordings/pico_tremor_20260812_071333` without disturbing unrelated dirty
worktree changes.

## Tasks

1. Add focused regression tests for stale-hold behavior and epoch clearing.
   Run the focused test before implementation and confirm it fails for the
   current raw fallback.
2. Extend per-side runtime state with the last valid corrected output. Hold
   it only for a stale palm, preserve raw startup fallback, and clear it on
   tracking-epoch transitions. Run the focused and package tests.
3. Update `scripts/analyze_pico_tremor.py` to read per-frame status JSON and
   exclude corrected samples whose side is not currently corrected. Preserve
   status-filter counts in the quality CSV/report.
4. Rerun the analysis, validate the summary/notebook/artifact, rebuild the
   report, and inspect the generated HTML structurally and with headless
   Chrome. Report the raw-controller tremor findings and corrected-stream
   cleanup separately.
