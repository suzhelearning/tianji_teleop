# PICO Headroom Global Viewer Default Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the no-argument Viewer and the PICO profile default to PICO teleoperation with SPARK Headroom Feedforward Velocity QP and model-reference control.

**Architecture:** Keep the existing CLI override mechanism and algorithm implementations unchanged. Align the `Options` defaults with `qp_ik_pico_teleop.yaml`, then lock the behavior with startup-output tests and document both the short default command and the explicit reproducibility command.

**Tech Stack:** C++20, CMake/CTest, YAML configuration, MuJoCo Viewer, Markdown.

## Global Constraints

- Default config is `config/qp_ik_pico_teleop.yaml`.
- Default model is `models/marvin_m6_qp_pico_fast.xml`.
- Default UDP endpoint is `127.0.0.1:15000`, with PICO teleop and skeleton overlay enabled.
- Default control is `velocity` with `spark_upper_qpoases_headroom_feedforward_velocity_qp`.
- Default control state source is `model_reference`.
- Explicit CLI options retain highest priority, and no historical algorithm is removed.
- Raw benchmark data is outside the change scope.

---

### Task 1: Lock the new defaults with failing tests

**Files:**
- Modify: `tests/assert_viewer_config.cmake`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: Viewer startup key/value lines written to stdout.
- Produces: Assertions for config path, control level, algorithm, state source, and PICO UDP bind.

- [ ] **Step 1: Extend the startup assertion helper**

Add optional checks equivalent to:

```cmake
foreach(expected IN ITEMS
    "algorithm=${EXPECTED_ALGORITHM}"
    "control_state_source=${EXPECTED_STATE_SOURCE}"
    "pico_udp_bind=${EXPECTED_PICO_BIND}")
  string(FIND "${viewer_stdout}" "${expected}" expected_index)
  if(expected_index EQUAL -1)
    message(FATAL_ERROR "Viewer stdout did not contain '${expected}'.")
  endif()
endforeach()
```

Guard each check with its corresponding `DEFINED` condition so existing callers remain compatible.

- [ ] **Step 2: Change the default and PICO profile expectations**

Make `test_viewer_default_profile` invoke `tests/assert_viewer_config.cmake` without `--config` or `--model`, expecting:

```text
config/qp_ik_pico_teleop.yaml
velocity
spark_upper_qpoases_headroom_feedforward_velocity_qp
model_reference
127.0.0.1:15000
```

Update `test_viewer_pico_profile` from acceleration to velocity and add the algorithm/state-source expectations.

- [ ] **Step 3: Build the test metadata and verify the tests fail**

Run:

```bash
cmake --build build -j
ctest --test-dir build -R 'test_viewer_(default|pico)_profile' --output-on-failure
```

Expected: failure because the Viewer still reports the acceleration config/defaults.

### Task 2: Implement aligned Viewer and PICO configuration defaults

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `config/qp_ik_pico_teleop.yaml`

**Interfaces:**
- Consumes: Existing `Options`, YAML loader, and CLI override fields.
- Produces: No-argument PICO Headroom startup while preserving explicit overrides.

- [ ] **Step 1: Change `Options` defaults**

Set the initial fields to:

```cpp
std::string config_path{"config/qp_ik_pico_teleop.yaml"};
std::string model_path{"models/marvin_m6_qp_pico_fast.xml"};
bool pico_teleop{true};
bool pico_skeleton_overlay{true};
```

Keep bind address and port at `127.0.0.1` and `15000`.

- [ ] **Step 2: Align the PICO YAML profile**

Set:

```yaml
controller:
  rate_hz: 200.0
  model_state_only: true
control:
  level: velocity
ik:
  algorithm: spark_upper_qpoases_headroom_feedforward_velocity_qp
```

Leave all tuned Headroom and safety parameters unchanged.

- [ ] **Step 3: Rebuild and verify focused tests pass**

Run:

```bash
cmake --build build -j
ctest --test-dir build -R 'test_viewer_(default|pico)_profile|pico_viewer_integration_spark_upper_qpoases_headroom_feedforward_velocity_qp' --output-on-failure
```

Expected: all selected tests pass when loopback UDP sockets are allowed.

### Task 3: Make README describe the new main path

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: Defaults implemented by Task 2.
- Produces: A short primary command and a complete explicit reproduction command.

- [ ] **Step 1: Replace the old Spark-guided primary example**

Use this primary command after configure/build:

```bash
OMP_WAIT_POLICY=ACTIVE OMP_PROC_BIND=close OMP_PLACES=cores \
  ./build/tianji_qp_ik_viewer
```

State that it defaults to the PICO fast model, UDP `127.0.0.1:15000`, velocity control, Headroom Feedforward Velocity QP, skeleton overlay, and model-reference control.

- [ ] **Step 2: Add the explicit reproducibility command**

Document the full equivalent invocation with `--config`, `--model`, `--pico-teleop`, `--pico-skeleton-overlay`, `--pico-bind`, `--pico-port`, `--control-level velocity`, `--algorithm spark_upper_qpoases_headroom_feedforward_velocity_qp`, and `--model-state-only`.

- [ ] **Step 3: Check documentation consistency**

Run:

```bash
rg -n 'spark_guided_velocity_qp|qp_ik_cartesian_otg_acceleration.yaml|headroom_feedforward_velocity_qp' README.md
git diff --check
```

Expected: historical examples remain explicitly scoped; the PICO primary example names Headroom Feedforward Velocity QP and no whitespace errors are reported.

### Task 4: Full regression verification

**Files:**
- Verify: all modified files

**Interfaces:**
- Consumes: Tasks 1-3.
- Produces: Evidence that the new default and all historical explicit modes remain valid.

- [ ] **Step 1: Run the complete suite**

Run in an environment allowed to create loopback UDP sockets:

```bash
ctest --test-dir build --output-on-failure
```

Expected: 80/80 tests pass.

- [ ] **Step 2: Verify no-argument startup summary**

Run:

```bash
./build/tianji_qp_ik_viewer --headless --duration 1
```

Expected stdout includes:

```text
viewer_config=config/qp_ik_pico_teleop.yaml
control_state_source=model_reference
pico_udp_bind=127.0.0.1:15000
algorithm=spark_upper_qpoases_headroom_feedforward_velocity_qp
control_level=velocity
```

- [ ] **Step 3: Review the final diff**

Run:

```bash
git diff --check
git diff --stat
git status --short
```

Expected: only the planned source, config, test, README, and plan files are changed; existing untracked benchmark directories remain unmodified and unstaged.

### Task 5: Document MuJoCo TJVR visual replay

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: Current no-argument Viewer defaults and
  `benchmark_results/pico_live/traces/output_continuity_retest.tjvr`.
- Produces: Reproducible two-terminal MuJoCo visual replay instructions.

- [ ] **Step 1: Correct the stale global-default description**

Replace the configuration-and-safety paragraph that names
`config/qp_ik_cartesian_otg_acceleration.yaml` as the no-argument default. Name the PICO config,
fast model, Headroom Feedforward Velocity QP, velocity control, model-reference state, and UDP
endpoint. State that historical profiles require explicit parameters.

- [ ] **Step 2: Add the two-terminal replay commands**

Document a 120-second Viewer command with `/tmp/pico_headroom_main_replay.csv` and
`/tmp/pico_headroom_main_replay_joints.csv`, followed by:

```bash
python3 /home/zj/current_robotics/TJ_arm/vr_data/tools/replay_pico_udp_trace.py \
  --input benchmark_results/pico_live/traces/output_continuity_retest.tjvr \
  --host 127.0.0.1 --port 15000 --lead 0.5
```

State that the reference trace contains 9442 packets over 106.887 seconds and that successful
replay telemetry reports `spark_upper_qpoases_headroom_feedforward_velocity_qp` with `velocity`.

- [ ] **Step 3: Verify documentation consistency**

Run:

```bash
rg -n 'output_continuity_retest|spark_upper_qpoases_headroom_feedforward_velocity_qp|qp_ik_cartesian_otg_acceleration.yaml' README.md
git diff --check
```

Expected: the old acceleration profile appears only in explicitly scoped historical commands;
the replay section contains the trace, algorithm, and two `/tmp` telemetry paths.
