# Cartesian FRF Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a deterministic MuJoCo empirical Cartesian FRF benchmark that compares four existing velocity-QP pipelines over both arms, seven working points, and six SE(3) channels.

**Architecture:** A small C++ excitation library owns deterministic chirp and SO(3) perturbation math. A headless C++ runner drives the existing controller and SPARK guidance paths at 200 Hz while publishing synthetic corrected-skeleton frames at 100 Hz, and writes synchronized CSV. Python analysis computes H1/coherence/group delay and a parallel orchestrator runs and resumes the full factorial scan.

**Tech Stack:** C++17, Eigen, MuJoCo, existing Tianji controller/SPARK modules, GoogleTest, Python 3, NumPy, SciPy and Matplotlib.

## Global Constraints

- Keep the current branch and worktree.
- Do not modify the four control algorithms, QP decision variables, hard constraints, or existing benchmark behavior.
- Use fixed sample-index time with controller `dt=0.005 s` and synthetic PICO source `dt=0.010 s`.
- Use model state as feedback; do not connect the Tianji SDK or real hardware.
- Use SO(3) exponential/logarithm for rotational excitation and measurement.
- Full matrix is 4 algorithms × 2 arms × 7 working points × 6 channels = 336 cases.
- FRF conclusions use only bins with coherence at least 0.8.

---

### Task 1: Deterministic excitation primitive

**Files:**
- Create: `include/tianji_qp_ik/cartesian_frf.hpp`
- Create: `src/cartesian_frf.cpp`
- Create: `tests/test_cartesian_frf.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `FrfExcitationConfig`, `FrfChannel`, `logChirpSample(config, sample)`, and `perturbPose(center, channel, value)`.
- Guarantees: zero input during warmup/settle, deterministic chirp during the excitation interval, and SO(3)-valid rotational targets.

- [ ] **Step 1: Write failing unit tests** for repeatability, warmup/settle zero values, endpoint instantaneous frequency, and SO(3) axis projection.
- [ ] **Step 2: Run** `cmake --build build --target test_cartesian_frf && ./build/test_cartesian_frf`; expect compile failure because the new interface is missing.
- [ ] **Step 3: Implement the minimal excitation library** using logarithmic phase `phi(t)=2*pi*f0*T/log(f1/f0)*(exp(log(f1/f0)*t/T)-1)` and `expSO3(axis*value)`.
- [ ] **Step 4: Rebuild and run the focused test**; expect all FRF excitation tests to pass.

### Task 2: Headless four-pipeline runner

**Files:**
- Create: `apps/benchmark_cartesian_frf.cpp`
- Create: `tests/test_cartesian_frf_runner.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: existing `DualArmController`, `DualArmSparkGuidance`, `PicoTeleopFrame`, `ArmMotionState`, and Task 1 excitation functions.
- Produces: one raw CSV per case plus a metadata preamble containing algorithm, arm, working point, channel, rates, amplitudes and validity.
- CLI: `tianji_cartesian_frf_benchmark --config PATH --model PATH --urdf PATH --algorithm NAME --arm left|right --working-point NAME --channel NAME --output PATH [--warmup S --chirp S --settle S]`.

- [ ] **Step 1: Write a failing Python integration test** that invokes a shortened direct-QP case and asserts deterministic row count, required columns, finite values and identical repeated input columns.
- [ ] **Step 2: Run** `python3 tests/test_cartesian_frf_runner.py --binary ./build/tianji_cartesian_frf_benchmark`; expect failure because the executable is absent.
- [ ] **Step 3: Implement CLI parsing, working-point validation, 100/200 Hz scheduling, synthetic corrected skeleton generation, algorithm-to-SPARK-mode mapping, controller stepping and CSV telemetry.** For SPARK cases, feed `updatePicoFrame()` only every two control samples and use the guidance-produced Cartesian target/reference/posture task. For direct QP, feed the generated Cartesian target directly.
- [ ] **Step 4: Build and run the integration test twice**; expect byte-identical time/input columns and no controller failures.

### Task 3: H1 FRF analysis and plotting

**Files:**
- Create: `scripts/analyze_cartesian_frf.py`
- Create: `tests/test_analyze_cartesian_frf.py`

**Interfaces:**
- Produces: `estimate_frf(u, y, fs, nperseg)`, per-case metrics JSON/CSV, Bode/coherence figures, four-algorithm overlays, and working-point envelopes.
- Uses: `scipy.signal.welch/csd/coherence`, unwrapped phase, numerical group delay, and coherence mask `>=0.8`.

- [ ] **Step 1: Write failing tests** using a synthetic first-order low-pass, a pure-delay signal, and unrelated noise.
- [ ] **Step 2: Run** `python3 -m unittest tests/test_analyze_cartesian_frf.py -v`; expect import failure.
- [ ] **Step 3: Implement H1, magnitude, phase, group delay, valid-bin filtering, bandwidth/peak/latency summaries and plotting.**
- [ ] **Step 4: Run the analyzer tests**; require recovered cutoff and delay within test tolerances and low coherence for independent noise.

### Task 4: Resumable multicore factorial orchestration

**Files:**
- Create: `scripts/run_cartesian_frf_benchmark.py`
- Create: `tests/test_run_cartesian_frf_benchmark.py`

**Interfaces:**
- CLI: `--jobs N`, `--algorithms`, `--arms`, `--working-points`, `--channels`, `--resume`, `--smoke`, and output root.
- Produces: `raw/*.csv`, `summary.csv`, `figures/*.png`, `README.md`, `manifest.json`, and failure manifest.

- [ ] **Step 1: Write failing tests** for deterministic 336-case expansion, filter expansion, and resume skipping only valid completed CSV files.
- [ ] **Step 2: Run** `python3 -m unittest tests/test_run_cartesian_frf_benchmark.py -v`; expect import failure.
- [ ] **Step 3: Implement process-pool execution with explicit per-case commands, atomic `.tmp` to `.csv` publication, manifest updates and analyzer invocation.**
- [ ] **Step 4: Run orchestration tests and a two-algorithm smoke matrix**; require raw CSV, summary and figures.

### Task 5: Full scan, verification, and report

**Files:**
- Create: `docs/verification/cartesian_frf_benchmark_results.md`
- Generate: `benchmark_results/cartesian_frf_<timestamp>/`

**Interfaces:**
- Consumes: Tasks 1–4.
- Produces: evidence-backed comparison of bandwidth, phase lag, group delay, coherence, tracking continuity, solver failures and hard-bound violations.

- [ ] **Step 1: Run focused C++ and Python tests**, then the full CTest suite.
- [ ] **Step 2: Run the complete 336-case matrix using available CPU cores** with deterministic case isolation and resume enabled.
- [ ] **Step 3: Inspect failure manifest and rerun only failed cases; do not silently replace invalid working points.**
- [ ] **Step 4: Generate four-algorithm Bode/coherence overlays and P10/P50/P90 working-point envelopes.**
- [ ] **Step 5: Write the verification report** with exact commands, result directory, invalid cases, hard-bound violations, and an explicit answer about which pipeline best balances tracking and smoothness.
