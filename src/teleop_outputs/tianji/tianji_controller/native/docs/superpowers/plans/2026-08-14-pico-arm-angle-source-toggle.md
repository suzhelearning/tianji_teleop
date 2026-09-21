# PICO Arm-Angle Source Toggle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a live `G`-key A/B switch between PICO skeleton arm directions and the baseline default-down arm directions without changing end-effector targets, OTG, QP, or limits.

**Architecture:** Put source-mode semantics and selection in the arm-angle module, transport toggle requests through the existing Viewer SPSC command queue, and keep the selected mode owned by the 200 Hz control thread. Publish the selected mode in `ViewerSnapshot`; the existing `ArmDirectionReferenceManager` performs continuous rate-limited transitions.

**Tech Stack:** C++17, Eigen, GLFW, MuJoCo, GoogleTest, Python integration test, CMake/CTest.

## Global Constraints

- PICO skeleton mode remains the startup default.
- `G` toggles only the arm-angle reference source.
- Do not change PICO end-effector poses, Cartesian OTG, QP gains, decision variables, joint constraints, or MuJoCo model.
- Do not reset the arm-direction rate limiter on a source toggle.
- PICO stale/disabled input always falls back to default-down.
- Do not stage or commit `benchmark_results/`.

---

### Task 1: Arm-angle source mode semantics

**Files:**
- Modify: `include/tianji_qp_ik/arm_angle.hpp`
- Modify: `src/arm_angle.cpp`
- Test: `tests/test_arm_angle.cpp`

**Interfaces:**
- Produces: `ArmAngleReferenceMode`, `toString(ArmAngleReferenceMode)`, `toggleArmAngleReferenceMode(ArmAngleReferenceMode)`, and `selectArmDirectionReferences(ArmAngleReferenceMode, bool, const DualArmDirectionReferences&)`.
- Consumes: existing `DualArmDirectionReferences` and `defaultArmDirectionReferences()`.

- [ ] **Step 1: Write failing mode-selection tests**

Add tests that require the new API:

```cpp
TEST(ArmAngleReferenceModeTest, TogglesBothDirections) {
  EXPECT_EQ(toggleArmAngleReferenceMode(ArmAngleReferenceMode::kPico),
            ArmAngleReferenceMode::kDefaultDown);
  EXPECT_EQ(toggleArmAngleReferenceMode(ArmAngleReferenceMode::kDefaultDown),
            ArmAngleReferenceMode::kPico);
}

TEST(ArmAngleReferenceModeTest, SelectsPicoOnlyWhenLiveAndRequested) {
  DualArmDirectionReferences pico;
  pico.left = {true, Eigen::Vector3d::UnitX(),
               ArmDirectionReferenceSource::kPico};
  pico.right = pico.left;
  EXPECT_EQ(selectArmDirectionReferences(ArmAngleReferenceMode::kPico, true,
                                         pico).left.source,
            ArmDirectionReferenceSource::kPico);
  EXPECT_EQ(selectArmDirectionReferences(ArmAngleReferenceMode::kDefaultDown,
                                         true, pico).left.source,
            ArmDirectionReferenceSource::kDefaultDown);
  EXPECT_EQ(selectArmDirectionReferences(ArmAngleReferenceMode::kPico, false,
                                         pico).left.source,
            ArmDirectionReferenceSource::kDefaultDown);
}
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `cmake --build build --target test_arm_angle && ./build/test_arm_angle`

Expected: compilation fails because `ArmAngleReferenceMode` and its functions do not exist.

- [ ] **Step 3: Implement the minimal source-mode API**

Declare the enum and functions in `arm_angle.hpp`; implement exact two-state toggling, string names `pico`/`default_down`, and selection that returns PICO directions only for live PICO mode and otherwise returns `defaultArmDirectionReferences()`.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run: `cmake --build build --target test_arm_angle && ./build/test_arm_angle`

Expected: all arm-angle tests pass.

### Task 2: Command and snapshot transport

**Files:**
- Modify: `include/tianji_qp_ik/telemetry.hpp`
- Modify: `tests/test_snapshot_exchange.cpp`

**Interfaces:**
- Consumes: `ArmAngleReferenceMode` from Task 1.
- Produces: `ViewerCommandType::kTogglePicoArmAngleSource` and `ViewerSnapshot::arm_angle_reference_mode`.

- [ ] **Step 1: Write failing transport tests**

Extend snapshot tests with:

```cpp
ViewerSnapshot snapshot;
snapshot.arm_angle_reference_mode = ArmAngleReferenceMode::kDefaultDown;
exchange.tryPublish(snapshot);
ViewerSnapshot latest;
ASSERT_TRUE(exchange.tryReadLatest(latest));
EXPECT_EQ(latest.arm_angle_reference_mode,
          ArmAngleReferenceMode::kDefaultDown);

ViewerCommand command;
command.type = ViewerCommandType::kTogglePicoArmAngleSource;
EXPECT_EQ(command.type, ViewerCommandType::kTogglePicoArmAngleSource);
```

- [ ] **Step 2: Run the focused test and verify RED**

Run: `cmake --build build --target test_snapshot_exchange`

Expected: compilation fails because the snapshot field and command enumerator do not exist.

- [ ] **Step 3: Add the command and snapshot fields**

Add `kTogglePicoArmAngleSource` to `ViewerCommandType` and initialize the snapshot field to `ArmAngleReferenceMode::kPico`.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run: `cmake --build build --target test_snapshot_exchange && ./build/test_snapshot_exchange`

Expected: all snapshot-exchange tests pass.

### Task 3: Viewer control-loop integration and observability

**Files:**
- Modify: `apps/run_qp_ik_viewer.cpp`
- Modify: `tests/test_pico_viewer_integration.py`
- Modify: `README.md`

**Interfaces:**
- Consumes: the source-mode helpers, Viewer command, and snapshot field from Tasks 1-2.
- Produces: `G` runtime toggle, control-thread source selection, Viewer overlay status, help text, and `arm_angle_mode` headless summary field.

- [ ] **Step 1: Write failing integration assertions**

Require the headless summary to contain the startup mode:

```python
summary = parse_summary(stdout)
assert summary["arm_angle_mode"] == "pico"
```

Add this assertion to both the normal PICO stream path and no-input monitor path.

- [ ] **Step 2: Run the PICO integration test and verify RED**

Run: `ctest --test-dir build --output-on-failure -R pico_viewer_integration_acceleration`

Expected: failure reporting missing summary key `arm_angle_mode`.

- [ ] **Step 3: Integrate the runtime toggle**

Add a control-thread local initialized as:

```cpp
ArmAngleReferenceMode arm_angle_reference_mode{ArmAngleReferenceMode::kPico};
```

Pass it by reference to `processCommand`, toggle it for
`kTogglePicoArmAngleSource`, and select references with:

```cpp
const DualArmDirectionReferences requested_arm_directions =
    selectArmDirectionReferences(arm_angle_reference_mode,
                                 pico_freshness.live,
                                 latest_pico_arm_directions);
```

Do not reset `arm_direction_manager` for this command. Copy the mode into every
Viewer snapshot. In the GLFW callback, map `G` to the new command. Display the
selected mode in the status overlay and add `G arm-angle source` to help text.
Append `arm_angle_mode=<pico|default_down>` to the headless completion summary.

- [ ] **Step 4: Document operation**

Update the PICO Viewer README controls to state that `G` switches only the
arm-angle source and leaves PICO end-effector control active.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run:

```bash
cmake --build build --parallel
ctest --test-dir build --output-on-failure -R 'test_(arm_angle|snapshot_exchange)|pico_viewer_integration_(velocity|acceleration)'
```

Expected: focused unit and PICO integration tests pass.

- [ ] **Step 6: Run complete verification**

Run:

```bash
git diff --check
ctest --test-dir build --output-on-failure
```

Expected: no whitespace errors and all tests pass.

- [ ] **Step 7: Commit implementation**

Stage only the source, tests, README, spec, and plan files; do not stage
`benchmark_results/`.

```bash
git add README.md apps/run_qp_ik_viewer.cpp \
  include/tianji_qp_ik/arm_angle.hpp include/tianji_qp_ik/telemetry.hpp \
  src/arm_angle.cpp tests/test_arm_angle.cpp \
  tests/test_snapshot_exchange.cpp tests/test_pico_viewer_integration.py \
  docs/superpowers/plans/2026-08-14-pico-arm-angle-source-toggle.md
git commit -m "feat: toggle PICO arm-angle reference source"
```
