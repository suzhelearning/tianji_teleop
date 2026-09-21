# PICO Trace Algorithm-Matrix Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run the canonical 9442-frame PICO trace through the current branch's algorithm matrix and generate reproducible accuracy, latency, smoothness, constraint, and visualization reports.

**Architecture:** A resumable runner validates provenance and executes one Viewer/replay pair at a time. A separate analyzer aligns common PICO source frames, computes fair main and appendix metrics, audits joint hard bounds, and creates CSV/Markdown/PNG reports.

**Tech Stack:** Python 3, C++ Viewer CLI, TJVR v1/v4, CSV, Numpy, Scipy, Matplotlib, CMake/CTest.

## Global Constraints

- Use `benchmark_results/pico_live/traces/output_continuity_retest.tjvr` unchanged.
- Use one algorithm process at a time for runtime fairness.
- Keep model, config, model-state-only policy, 200 Hz controller, and hard constraints identical.
- Rank only the five primary algorithms with common SPARK target semantics.
- Keep generated benchmark data outside Git.

---

### Task 1: Resumable sequential benchmark runner

**Files:**
- Create: `scripts/run_pico_trace_algorithm_benchmark.py`
- Create: `tests/test_run_pico_trace_algorithm_benchmark.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Viewer path, replay-tool path, TJVR path, config, model, output directory, algorithm list.
- Produces: `manifest.json`, per-algorithm telemetry/joint CSV/log files, preflight status, and resumable completion records.

- [x] **Step 1: Write failing unit tests** for TJVR metadata/hash extraction, deterministic command construction, manifest provenance mismatch, completed-run validation, and preflight trace slicing.
- [x] **Step 2: Verify RED** by running the new Python unittest target and observing missing-module failures.
- [x] **Step 3: Implement the runner** with primary/appendix algorithm constants, exclusive sequential execution, unique UDP ports, bounded subprocess shutdown, manifest atomic replacement, validation of 9442 datagrams and zero protocol/control failures, and `--resume` behavior.
- [x] **Step 4: Verify GREEN** with the focused Python unit tests and a short real Viewer preflight for every algorithm.

### Task 2: Common-frame metrics and safety audit

**Files:**
- Create: `scripts/analyze_pico_trace_algorithm_benchmark.py`
- Create: `tests/test_analyze_pico_trace_algorithm_benchmark.py`

**Interfaces:**
- Consumes: runner manifest and per-algorithm Cartesian/joint telemetry.
- Produces: `summary_main.csv`, `summary_all.csv`, `joint_summary.csv`, `constraint_audit.csv`, and in-memory aligned result objects used by plotting.

- [x] **Step 1: Write failing tests** using synthetic telemetry for common-frame intersection, reset-window exclusion, target-agreement filtering, lag recovery, percentile metrics, 2.5--5 Hz energy, active-bound counts, and hard-bound violation detection.
- [x] **Step 2: Verify RED** because the analyzer module does not exist.
- [x] **Step 3: Implement metric loading/alignment** with all-valid and clean-common sample sets, canonical target handling, finite-value checks, 0--300 ms lag scan, overshoot/amplitude metrics, runtime metrics, joint continuity metrics, and numerical-tolerance safety audit.
- [x] **Step 4: Verify GREEN** with all analyzer tests.

### Task 3: Main and appendix report generation

**Files:**
- Modify: `scripts/analyze_pico_trace_algorithm_benchmark.py`
- Modify: `tests/test_analyze_pico_trace_algorithm_benchmark.py`

**Interfaces:**
- Consumes: aligned primary and appendix results from Task 2.
- Produces: Chinese `README.md`, six-panel tracking time series, 3D trajectories, CDFs, lag scans, summary charts, and left/right 4x7 joint-continuity figures.

- [x] **Step 1: Add failing artifact tests** requiring all requested report filenames and readable non-empty PNGs.
- [x] **Step 2: Implement plotting/reporting** with stable colors, readable primary-only figures, separate appendix figures, highest-motion-window selection, and provenance/caveat text.
- [x] **Step 3: Verify focused tests** and run `py_compile` on both scripts.

### Task 4: Full canonical benchmark and verification

**Files:**
- Create under ignored data: `benchmark_results/pico_trace_algorithm_matrix_20260818/`
- Modify: this plan checklist only.

**Interfaces:**
- Consumes: completed runner and analyzer.
- Produces: final metrics, plots, ranking, safety audit, and reproducible commands.

- [x] **Step 1: Run preflight for all ten algorithms** and classify any incompatible algorithm explicitly.
- [x] **Step 2: Run/resume the complete sequential matrix** until every compatible algorithm has processed all 9442 packets.
- [x] **Step 3: Generate main and appendix reports** and inspect summary/safety outputs; constrained QP algorithms have zero hard-bound violations, while unsafe/failing algorithms are explicitly excluded from the recommended ranking.
- [x] **Step 4: Run focused tests and the full CTest suite**, `git diff --check`, process cleanup checks, and report exact output paths without committing generated benchmark data.
