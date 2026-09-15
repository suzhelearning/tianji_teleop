# PICO Trace Algorithm-Matrix Benchmark Design

## Goal

Use `benchmark_results/pico_live/traces/output_continuity_retest.tjvr` as the
canonical real-PICO input and compare every compatible algorithm in the current
branch without changing its source timing, model, initial posture, controller
rate, or hard constraints. Produce a reproducible primary ranking for algorithms
that share the same SPARK target and a clearly separated appendix for algorithms
with different target semantics.

## Canonical input and provenance

The runner validates the TJVR container before any test, records its SHA256,
packet count, packet size, source duration, Git commit, dirty-worktree status,
config hash, model hash, command line, and algorithm list in a manifest. A
completed algorithm may be resumed only when all provenance fields match.

Every run uses:

- `config/qp_ik_pico_teleop.yaml`;
- `models/marvin_m6_qp_pico_fast.xml`;
- model-state-only feedback;
- velocity control for all SPARK guidance algorithms;
- the original TJVR source timestamps and 200 Hz control loop;
- one Viewer process at a time so cycle-time measurements are not polluted by
  competing benchmark processes.

## Algorithm groups

The primary ranking contains five algorithms whose targets originate from the
same SPARK upper-body retargeting and two-stage qpOASES IK path:

1. `spark_upper_qpoases_direct`;
2. `spark_upper_qpoases_velocity_qp`;
3. `spark_upper_qpoases_cartesian_otg_velocity_qp`;
4. `spark_upper_qpoases_feedforward_velocity_qp`;
5. `spark_upper_qpoases_headroom_feedforward_velocity_qp`.

The appendix runs the remaining current-branch algorithms that pass preflight:

- `hierarchical_qp`;
- `nullspace_dls`;
- `spark_guided_velocity_qp`;
- `spark_direct_velocity_qp`;
- `spark_pose_velocity_qp`.

Appendix algorithms retain measured results but do not compete for the primary
accuracy winner because their target-generation semantics differ.

## Execution

Before the full matrix, each algorithm receives a short trace slice to verify
CLI compatibility, finite telemetry, PICO-live input, zero control failures,
and the expected algorithm identity. The full runner then assigns a unique UDP
port, starts a headless Viewer, replays all 9442 frames with a short lead, waits
for clean termination, and validates telemetry before marking the algorithm
complete. Failure leaves prior successful algorithms resumable and produces an
explicit failed state in the manifest.

The runner writes one Cartesian telemetry CSV, one joint telemetry CSV, and one
stdout log per algorithm. Benchmark result directories remain untracked.

## Fair sample sets

Two result sets are reported:

- **all-valid:** all common PICO-live source sequences, including limit events,
  jump rejection, and resynchronization recovery;
- **clean-common:** common source sequences after excluding startup and a fixed
  recovery interval following every tracking epoch/reset transition.

The canonical target for the primary group is selected from the non-OTG SPARK
velocity-QP run. Target agreement is checked for every primary algorithm before
ranking. Disagreement beyond tolerance removes the affected source frame from
clean-common statistics and is reported rather than silently comparing unlike
targets.

## Metrics

Cartesian tracking metrics include position and orientation Mean/P50/P95/RMSE/
maximum, a 0--300 ms causal lag scan, lag-compensated RMSE, TCP motion-amplitude
ratio, and overshoot ratio. Runtime metrics include cycle Mean/P95/P99/maximum,
acceptance, control failures, solver failures, and deadline misses.

Joint metrics include per-arm and per-joint qdot/qddot/jerk P50/P95/P99/maximum,
2.5--5 Hz acceleration energy, adjacent-cycle deltas, reset count, active
position/velocity/acceleration/braking bounds, and violations of recorded
position/velocity/acceleration/jerk bounds. A hard-bound violation invalidates
that algorithm's safety result.

## Reports

The report directory contains:

- `summary_main.csv` and `summary_all.csv`;
- a Chinese `README.md` with ranking, caveats, and provenance;
- six-panel left/right XYZ tracking time series over the highest-motion window;
- 3D TCP trajectories;
- tracking-error CDFs;
- position and orientation lag scans;
- accuracy/latency/amplitude/runtime summary figures;
- q/qdot/qddot/jerk joint-continuity figures;
- constraint activation and hard-bound audit tables.

The main figures contain only the five fair primary algorithms to remain
readable. Appendix figures contain the remaining algorithms separately.

## Acceptance

The benchmark is complete when all algorithms pass preflight or are explicitly
classified as incompatible, every compatible full replay terminates cleanly,
the five primary runs receive all 9442 datagrams with zero malformed/CRC/
reordered packets and zero control failures, no recorded hard bound is violated,
all summary values are finite, all requested figures are generated, and rerunning
the command resumes without repeating valid completed runs.
