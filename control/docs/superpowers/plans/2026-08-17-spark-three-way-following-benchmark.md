# SPARK Three-Way Following Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build and run a reproducible raw-PICO-intent following benchmark for the three approved SPARK control chains.

**Architecture:** A standalone Python analyzer consumes existing telemetry CSV files, validates their common target grain, aligns samples by `pico_sequence`, computes zero-lag and lag-compensated metrics, and exports static figures plus a Markdown report. It does not modify any controller.

**Tech Stack:** Python 3, csv, NumPy, Matplotlib, unittest/CTest.

## Global Constraints

- Preserve all three controller implementations and configurations.
- Use raw SPARK palm as common intent, not each algorithm's internal reference.
- Exclude startup/resynchronization blend windows from headline metrics.
- Keep benchmark results untracked.

---

### Task 1: Alignment and lag metrics

**Files:**
- Create: `scripts/compare_spark_three_way_following.py`
- Create: `tests/test_compare_spark_three_way_following.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- `quaternion_distance(left, right) -> float`
- `tracking_errors(actual_rows, target_rows, valid_sequences, lag_frames) -> tuple[np.ndarray, np.ndarray]`
- `best_lag(...) -> LagResult`

- [ ] Write synthetic delayed-trajectory tests and verify they fail because the analyzer does not exist.
- [ ] Implement alignment, quaternion distance, exclusion, target agreement, and lag scan.
- [ ] Run the focused unittest and CTest.

### Task 2: Report and figures

**Files:**
- Modify: `scripts/compare_spark_three_way_following.py`
- Test: `tests/test_compare_spark_three_way_following.py`

**Interfaces:**
- CLI accepts three telemetry files, two optional joint telemetry files, exclusion ranges, max lag, and output directory.
- CLI writes `summary.csv`, `README.md`, and five PNG figures.

- [ ] Add a failing end-to-end synthetic CSV test.
- [ ] Implement summary, dynamic-limit statistics, report, and plots.
- [ ] Run focused tests and `git diff --check`.

### Task 3: Real replay analysis and QA

**Files:**
- Generate only under: `benchmark_results/spark_three_way_20260817/`

- [ ] Run the analyzer on the three 2134-frame replay outputs.
- [ ] Independently recompute headline metrics for validation.
- [ ] Inspect every rendered PNG for clipping, misleading scales, and label errors.
- [ ] Run the full repository test suite serially.

