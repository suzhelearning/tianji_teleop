# Odin Lite–Pelvis Extrinsic Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a one-time PICO/Odin trajectory calibrator that persists `T_pelvis_odin`, plus a runtime node that restores PICO pelvis height and anchors raw or foot-fused skeletons to calibrated Odin motion.

**Architecture:** Extend the existing `pico_odin` package with a small Eigen SE(3) core, a constrained hand-eye solver, atomic YAML persistence, an interactive calibration node, and a runtime alignment node. Installation calibration consumes only raw PICO/Odin trajectories; normal operation loads the persistent extrinsic and maintains a separate in-memory session transform.

**Tech Stack:** ROS 2 Humble, C++17, Eigen3, yaml-cpp, `rclcpp`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `ament_cmake_gtest`, Python launch tests.

## Global Constraints

- Develop on `feature/odin-pelvis-extrinsic-calibration`.
- Preserve `/raw/odom/odin*`, `/pico/smpl`, and `/pico/smpl_fused` unchanged.
- Store only `T_pelvis_odin` persistently; never persist PICO height or session alignment.
- Keyboard lowercase `a` starts installation calibration; PICO controller A behavior is unchanged.
- One forward/back excitation cycle and one left/right excitation cycle are required.
- Default persisted path is `~/.config/pico_tracker/odin_pelvis_extrinsics.yaml`, with parameter override.
- Existing unrelated changes, especially `src/pico_bridge/config/imu900_feet.yaml`, must remain outside feature commits.
- New behavior is test-driven and the full workspace test suite must pass.

---

## File structure

- `src/pico_odin/include/pico_odin/se3.hpp`: pose composition, inversion, interpolation, averaging, finite checks, ROS conversions.
- `src/pico_odin/include/pico_odin/extrinsic_solver.hpp`: trajectory sample types, solver options/results, excitation and residual validation API.
- `src/pico_odin/src/extrinsic_solver.cpp`: timestamp pairing, hand-eye rotation/translation solve, robust refinement and quality metrics.
- `src/pico_odin/include/pico_odin/extrinsics_file.hpp`: persisted schema and load/save API.
- `src/pico_odin/src/extrinsics_file.cpp`: XDG path expansion, YAML validation and atomic save.
- `src/pico_odin/include/pico_odin/runtime_alignment.hpp`: per-run alignment and rigid skeleton re-anchoring API.
- `src/pico_odin/src/runtime_alignment.cpp`: stable-window alignment and pose-array transformation.
- `src/pico_odin/src/odin_pelvis_calibrator_node.cpp`: subscriptions, keyboard state machine, prompts, cancellation and save.
- `src/pico_odin/src/odin_pelvis_runtime_node.cpp`: load extrinsic, reset/reacquire session alignment, corrected publishers.
- `src/pico_odin/test/test_se3.cpp`: transform convention tests.
- `src/pico_odin/test/test_extrinsic_solver.cpp`: synthetic known-transform, noise, offset and degeneracy tests.
- `src/pico_odin/test/test_extrinsics_file.cpp`: schema and safe-save tests.
- `src/pico_odin/test/test_runtime_alignment.cpp`: nonzero initial height and 24-joint anchoring tests.
- `src/pico_odin/test/test_launch_integration.py`: launch defaults and executable wiring.
- `src/pico_odin/launch/odin_select.launch.py`: optional runtime node and portable calibration path.
- `src/pico_odin/CMakeLists.txt`, `src/pico_odin/package.xml`: build and dependency declarations.
- `README.md`, `README.zh-CN.md`: calibration and normal-operation commands.

---

### Task 1: SE(3) core and package conversion

**Files:**
- Create: `src/pico_odin/include/pico_odin/se3.hpp`
- Create: `src/pico_odin/test/test_se3.cpp`
- Modify: `src/pico_odin/CMakeLists.txt`
- Modify: `src/pico_odin/package.xml`

**Interfaces:**
- Produces: `pico_odin::Pose3`, `compose`, `inverse`, `interpolate`, `mean_pose`, `angular_distance`, `to_pose_msg`, and `from_pose_msg`.
- Consumes: Eigen3 and `geometry_msgs/msg/Pose`.

- [ ] **Step 1: Write failing transform tests**

Add tests that assert:

```cpp
const Pose3 pelvis_T_odin{Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())),
                         Eigen::Vector3d(-0.25, 0.0, 0.08)};
const Pose3 odin_world_T_odin{Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()};
const Pose3 odin_world_T_pelvis = compose(odin_world_T_odin, inverse(pelvis_T_odin));
EXPECT_NEAR(compose(pelvis_T_odin, inverse(pelvis_T_odin)).translation.norm(), 0.0, 1e-12);
EXPECT_NEAR(interpolate(Pose3{}, Pose3{Eigen::Quaterniond::Identity(), {0, 0, 1}}, 0.5)
              .translation.z(), 0.5, 1e-12);
```

- [ ] **Step 2: Run the targeted test and verify it fails**

Run: `pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --ctest-args -R test_se3 --output-on-failure'`

Expected: build/test target failure because `Pose3` and `test_se3` do not exist.

- [ ] **Step 3: Implement the minimal SE(3) core**

Define:

```cpp
struct Pose3 {
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
};

Pose3 compose(const Pose3 & a_T_b, const Pose3 & b_T_c);
Pose3 inverse(const Pose3 & a_T_b);
Pose3 interpolate(const Pose3 & a, const Pose3 & b, double alpha);
Pose3 mean_pose(const std::vector<Pose3> & poses);
double angular_distance(const Eigen::Quaterniond & a, const Eigen::Quaterniond & b);
bool finite(const Pose3 & pose);
```

Normalize quaternion results and hemisphere-align quaternion averaging.

- [ ] **Step 4: Build and run `test_se3`**

Run: `pixi run build && pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --ctest-args -R test_se3 --output-on-failure'`

Expected: `test_se3` passes.

- [ ] **Step 5: Commit the SE(3) core**

```bash
git add src/pico_odin/CMakeLists.txt src/pico_odin/package.xml \
  src/pico_odin/include/pico_odin/se3.hpp src/pico_odin/test/test_se3.cpp
git commit -m "feat(pico_odin): add SE3 transform core"
```

### Task 2: Safe extrinsics persistence

**Files:**
- Create: `src/pico_odin/include/pico_odin/extrinsics_file.hpp`
- Create: `src/pico_odin/src/extrinsics_file.cpp`
- Create: `src/pico_odin/test/test_extrinsics_file.cpp`
- Modify: `src/pico_odin/CMakeLists.txt`

**Interfaces:**
- Consumes: `Pose3` from Task 1.
- Produces: `ExtrinsicsDocument`, `default_extrinsics_path()`, `load_extrinsics(path)`, and `save_extrinsics_atomic(path, document)`.

- [ ] **Step 1: Write failing schema and preservation tests**

Use a temporary directory and verify:

```cpp
ExtrinsicsDocument doc;
doc.pelvis_T_odin = known_pose;
doc.valid = true;
doc.metrics.sample_count = 180;
save_extrinsics_atomic(path, doc);
const auto loaded = load_extrinsics(path);
EXPECT_TRUE(loaded.valid);
EXPECT_LT(angular_distance(loaded.pelvis_T_odin.rotation, known_pose.rotation), 1e-10);
EXPECT_LT((loaded.pelvis_T_odin.translation - known_pose.translation).norm(), 1e-10);
```

Write invalid replacement data to a separate candidate and assert the existing valid file remains byte-for-byte unchanged.

- [ ] **Step 2: Run the targeted test and verify it fails**

Run: `pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --ctest-args -R test_extrinsics_file --output-on-failure'`

Expected: missing persistence API/target failure.

- [ ] **Step 3: Implement versioned YAML load/save**

Define:

```cpp
struct CalibrationMetrics {
  int sample_count{0};
  double time_offset_sec{0.0};
  double pitch_range_rad{0.0};
  double yaw_range_rad{0.0};
  double rotation_rms_rad{0.0};
  double translation_rms_m{0.0};
  double condition_number{0.0};
};

struct ExtrinsicsDocument {
  int schema_version{1};
  bool valid{false};
  Pose3 pelvis_T_odin;
  CalibrationMetrics metrics;
  std::string calibrated_at;
};
```

Require schema version 1, finite normalized transforms, `valid: true`, explicit quaternion order, and a matrix consistent with translation/quaternion fields. Expand `~` using the current user home only inside `default_extrinsics_path()`. Save to a sibling temporary file, reread it, then rename atomically.

- [ ] **Step 4: Run persistence tests**

Run: `pixi run build && pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --ctest-args -R test_extrinsics_file --output-on-failure'`

Expected: all persistence tests pass.

- [ ] **Step 5: Commit persistence support**

```bash
git add src/pico_odin/CMakeLists.txt src/pico_odin/include/pico_odin/extrinsics_file.hpp \
  src/pico_odin/src/extrinsics_file.cpp src/pico_odin/test/test_extrinsics_file.cpp
git commit -m "feat(pico_odin): persist validated pelvis extrinsics"
```

### Task 3: Constrained hand-eye solver

**Files:**
- Create: `src/pico_odin/include/pico_odin/extrinsic_solver.hpp`
- Create: `src/pico_odin/src/extrinsic_solver.cpp`
- Create: `src/pico_odin/test/test_extrinsic_solver.cpp`
- Modify: `src/pico_odin/CMakeLists.txt`

**Interfaces:**
- Consumes: `Pose3`, `CalibrationMetrics`.
- Produces: `solve_extrinsics(const std::vector<TimedPose>& pico, const std::vector<TimedPose>& odin, const SolverOptions&) -> SolverResult`.

- [ ] **Step 1: Write failing known-transform and degeneracy tests**

Generate deterministic trajectories with one pitch cycle and one yaw cycle:

```cpp
const Pose3 expected_pelvis_T_odin{
  Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(0.14, Eigen::Vector3d::UnitY())),
  Eigen::Vector3d(-0.27, 0.01, 0.09)};
const auto data = make_synthetic_hand_eye(expected_pelvis_T_odin, 0.025, 0.002, 0.001);
const auto result = solve_extrinsics(data.pico, data.odin, SolverOptions{});
ASSERT_TRUE(result.success) << result.message;
EXPECT_LT(angular_distance(result.pelvis_T_odin.rotation,
                           expected_pelvis_T_odin.rotation), 0.035);
EXPECT_LT((result.pelvis_T_odin.translation -
           expected_pelvis_T_odin.translation).norm(), 0.03);
```

Also assert static-only and pitch-only trajectories fail with explicit `insufficient excitation` or `ill-conditioned` messages.

- [ ] **Step 2: Run tests and verify red state**

Run: `pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --ctest-args -R test_extrinsic_solver --output-on-failure'`

Expected: missing solver API/target failure.

- [ ] **Step 3: Implement synchronization and the closed-form solve**

Define:

```cpp
struct TimedPose { double stamp_sec; Pose3 pose; };
struct SolverOptions {
  double min_pitch_range_rad{12.0 * M_PI / 180.0};
  double min_yaw_range_rad{12.0 * M_PI / 180.0};
  std::size_t min_pairs{80};
  double max_offset_sec{0.30};
  double offset_step_sec{0.002};
  double y_prior_sigma_m{0.05};
};
struct SolverResult {
  bool success{false};
  std::string message;
  Pose3 pelvis_T_odin;
  CalibrationMetrics metrics;
};
```

Estimate stream offset from angular-speed magnitude correlation, interpolate Odin poses at PICO timestamps, construct separated relative-motion pairs, solve rotation using quaternion SVD, and solve translation using stacked linear least squares.

- [ ] **Step 4: Add constrained robust refinement and validation**

Refine six tangent-space parameters by damped Gauss-Newton on `log(inverse(A*X) * (X*B))`, with Huber weights and a zero-centered pelvis-frame `y` prior. Validate excitation, sample count, condition, residuals, rearward `x`, broad `z/y` bounds, and broad angular distance from the rear-facing prior. Return `pelvis_T_odin = inverse(X)`.

- [ ] **Step 5: Run solver and package tests**

Run: `pixi run build && pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --event-handlers console_direct+'`

Expected: known-transform/noise/offset tests pass and degenerate trajectories are rejected.

- [ ] **Step 6: Commit the solver**

```bash
git add src/pico_odin/CMakeLists.txt src/pico_odin/include/pico_odin/extrinsic_solver.hpp \
  src/pico_odin/src/extrinsic_solver.cpp src/pico_odin/test/test_extrinsic_solver.cpp
git commit -m "feat(pico_odin): solve constrained pelvis extrinsics"
```

### Task 4: Interactive installation calibrator

**Files:**
- Create: `src/pico_odin/src/odin_pelvis_calibrator_node.cpp`
- Create: `src/pico_odin/include/pico_odin/calibration_session.hpp`
- Create: `src/pico_odin/test/test_calibration_session.cpp`
- Modify: `src/pico_odin/CMakeLists.txt`
- Modify: `src/pico_odin/package.xml`

**Interfaces:**
- Consumes: raw PICO pelvis and raw Odin high-frequency odometry, `solve_extrinsics`, atomic persistence.
- Produces: executable `odin_pelvis_calibrator` and testable `CalibrationSession` state transitions.

- [ ] **Step 1: Write failing session-state tests**

Verify:

```cpp
CalibrationSession session(options);
EXPECT_EQ(session.phase(), CalibrationPhase::kWaitingForInputs);
session.observe_valid_inputs(now);
EXPECT_TRUE(session.start(now));
EXPECT_EQ(session.phase(), CalibrationPhase::kNeutral);
session.on_world_reset();
EXPECT_EQ(session.phase(), CalibrationPhase::kCancelled);
EXPECT_TRUE(session.samples().empty());
```

Add a successful neutral → excitation → final-neutral transition using synthetic velocities and an Odin restart cancellation test.

- [ ] **Step 2: Run tests and verify red state**

Run: `pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --ctest-args -R test_calibration_session --output-on-failure'`

Expected: missing session API/target failure.

- [ ] **Step 3: Implement the state machine**

Create phases `kWaitingForInputs`, `kReady`, `kNeutral`, `kExcitation`, `kFinalNeutral`, `kSolving`, `kSaved`, `kRejected`, and `kCancelled`. Use two-second neutral windows, configured excitation thresholds, a 30-second timeout, monotonic timestamps, and explicit cancellation reasons.

- [ ] **Step 4: Implement the ROS node and keyboard input**

Subscribe with sensor-data QoS, convert pose index 0 and odometry into `TimedPose`, and monitor `/pico/world_reset`. A small nonblocking stdin thread accepts lowercase `a` and `q`; it posts state changes without touching ROS objects directly. On successful completion, call the solver, print all quality metrics, and atomically save only accepted results.

- [ ] **Step 5: Build and run calibrator tests**

Run: `pixi run build && pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --event-handlers console_direct+'`

Expected: session tests pass and `ros2 pkg executables pico_odin` lists `odin_pelvis_calibrator`.

- [ ] **Step 6: Commit the interactive calibrator**

```bash
git add src/pico_odin/CMakeLists.txt src/pico_odin/package.xml \
  src/pico_odin/include/pico_odin/calibration_session.hpp \
  src/pico_odin/src/odin_pelvis_calibrator_node.cpp \
  src/pico_odin/test/test_calibration_session.cpp
git commit -m "feat(pico_odin): add interactive pelvis calibration"
```

### Task 5: Runtime session alignment and skeleton anchoring

**Files:**
- Create: `src/pico_odin/include/pico_odin/runtime_alignment.hpp`
- Create: `src/pico_odin/src/runtime_alignment.cpp`
- Create: `src/pico_odin/src/odin_pelvis_runtime_node.cpp`
- Create: `src/pico_odin/test/test_runtime_alignment.cpp`
- Modify: `src/pico_odin/CMakeLists.txt`

**Interfaces:**
- Consumes: persisted `T_pelvis_odin`, PICO pelvis, raw Odin odometry, raw/fused PoseArray.
- Produces: corrected pelvis odometry and rigidly anchored skeleton arrays.

- [ ] **Step 1: Write failing height and skeleton tests**

Assert the central invariant:

```cpp
RuntimeAlignment alignment(pelvis_T_odin);
alignment.initialize(pico_world_T_pelvis0, odin_world_T_odin0);
const Pose3 out0 = alignment.corrected_pelvis(odin_world_T_odin0);
EXPECT_NEAR(out0.translation.z(), pico_world_T_pelvis0.translation.z(), 1e-10);

Pose3 moved = odin_world_T_odin0;
moved.translation.z() += 0.12;
EXPECT_NEAR(alignment.corrected_pelvis(moved).translation.z(),
            pico_world_T_pelvis0.translation.z() + 0.12, 1e-9);
```

Create 24 synthetic joint poses, anchor to a rotated/translated corrected pelvis, and verify every pairwise bone vector receives the same rigid rotation without length change.

- [ ] **Step 2: Run tests and verify red state**

Run: `pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --ctest-args -R test_runtime_alignment --output-on-failure'`

Expected: missing runtime alignment API/target failure.

- [ ] **Step 3: Implement per-run alignment**

Define:

```cpp
class RuntimeAlignment {
 public:
  explicit RuntimeAlignment(Pose3 pelvis_T_odin);
  void clear();
  void initialize(const Pose3 & pico_world_T_pelvis0,
                  const Pose3 & odin_world_T_odin0);
  bool initialized() const;
  Pose3 corrected_pelvis(const Pose3 & odin_world_T_odin) const;
  std::vector<Pose3> anchor_skeleton(const std::vector<Pose3> & pico_joints,
                                     const Pose3 & corrected_root) const;
};
```

Compute `S = P_0 * inverse(O_0 * inverse(T_pelvis_odin))`. Clear it on world reset or Odin discontinuity. Require a stationary synchronized window before calling `initialize`.

- [ ] **Step 4: Implement the runtime ROS node**

Load and validate the YAML at startup. Subscribe to raw Odin normal/high-frequency odometry, `/pico/smpl`, `/pico/smpl_fused`, and `/pico/world_reset`. Publish:

```text
/calibrated/odom/pelvis
/calibrated/odom/pelvis_highfreq
/pico/smpl_odin
/pico/smpl_fused_odin
```

Maintain a bounded timestamp buffer, interpolate corrected pelvis to PICO timestamps, transform all 24 poses, preserve raw topics, and throttle stale/missing-input warnings. Transform populated twist/covariance with the rigid adjoint; retain unknown covariance markers.

- [ ] **Step 5: Run runtime tests and inspect executables**

Run: `pixi run build && pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --event-handlers console_direct+ && source install/setup.bash && ros2 pkg executables pico_odin'`

Expected: runtime tests pass and both new executables are listed.

- [ ] **Step 6: Commit runtime alignment**

```bash
git add src/pico_odin/CMakeLists.txt src/pico_odin/include/pico_odin/runtime_alignment.hpp \
  src/pico_odin/src/runtime_alignment.cpp src/pico_odin/src/odin_pelvis_runtime_node.cpp \
  src/pico_odin/test/test_runtime_alignment.cpp
git commit -m "feat(pico_odin): anchor pelvis and skeleton to Odin"
```

### Task 6: Launch integration and documentation

**Files:**
- Create: `src/pico_odin/test/test_launch_integration.py`
- Modify: `src/pico_odin/launch/odin_select.launch.py`
- Modify: `src/pico_odin/CMakeLists.txt`
- Modify: `README.md`
- Modify: `README.zh-CN.md`
- Modify: `pico_full_test_commands.txt`

**Interfaces:**
- Consumes: both new executables and the default extrinsics path.
- Produces: `enable_pelvis_runtime:=auto|true|false` launch behavior and complete user commands.

- [ ] **Step 1: Write failing launch-contract tests**

Assert the launch source declares `enable_pelvis_runtime`, `pelvis_extrinsics_file`, starts `odin_pelvis_runtime`, defaults `auto` to enabled only for `odin_impl:=lite`, and can disable runtime during calibration.

- [ ] **Step 2: Run the launch test and verify it fails**

Run: `pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --ctest-args -R test_launch_integration --output-on-failure'`

Expected: assertions fail because launch integration is absent.

- [ ] **Step 3: Implement launch arguments and node wiring**

Parse `auto`, `true`, and `false` strictly in `_setup`. Expand no machine-specific path in launch; pass an empty path to let the node use its XDG default. Start the runtime after the selected Odin driver in the same launch description.

- [ ] **Step 4: Document calibration and normal operation**

Document these portable flows in both READMEs and the command reference:

```bash
# Terminal 1: raw Odin Lite for installation calibration
ros2 launch pico_odin odin_select.launch.py \
  odin_impl:=lite enable_pelvis_runtime:=false

# Terminal 2: PICO bridge
adb forward tcp:9999 tcp:9999
ros2 launch pico_bridge start_pico_bridge.launch.py

# Terminal 3: press keyboard a and follow prompts
ros2 run pico_odin odin_pelvis_calibrator

# Normal run: loads persisted extrinsics automatically for Odin Lite
ros2 launch pico_odin odin_select.launch.py odin_impl:=lite
```

Explain the initial PICO height plus Odin displacement rule, world-reset reacquisition, corrected topics, and failure diagnostics.

- [ ] **Step 5: Run package tests and documentation checks**

Run: `pixi run build && pixi run bash -lc 'colcon test --base-paths src --packages-select pico_odin --event-handlers console_direct+'`

Run: `git diff --check`

Expected: all `pico_odin` tests pass and no whitespace errors exist.

- [ ] **Step 6: Commit launch and documentation**

```bash
git add src/pico_odin/launch/odin_select.launch.py src/pico_odin/test/test_launch_integration.py \
  src/pico_odin/CMakeLists.txt README.md README.zh-CN.md pico_full_test_commands.txt
git commit -m "docs: add Odin pelvis calibration workflow"
```

### Task 7: Full verification and hardware handoff

**Files:**
- Modify only if verification exposes a defect in files already owned by Tasks 1–6.

**Interfaces:**
- Consumes: complete feature branch.
- Produces: build/test evidence and exact hardware test commands.

- [ ] **Step 1: Run the complete workspace build**

Run: `pixi run build`

Expected: all selected workspace packages build successfully.

- [ ] **Step 2: Run the complete workspace test suite**

Run: `pixi run test`

Run: `pixi run bash -lc 'colcon test-result --test-result-base build --verbose'`

Expected: zero errors and zero failures, including all previously passing tests.

- [ ] **Step 3: Audit raw-topic preservation and branch scope**

Run:

```bash
git diff main...HEAD --check
git status --short --branch
rg -n '/home/zj' README.md README.zh-CN.md pico_full_test_commands.txt src/pico_odin
```

Expected: no machine-specific documentation paths; unrelated dirty files remain uncommitted; raw topic names are unchanged.

- [ ] **Step 4: Prepare the hardware verification sequence**

Provide commands to check:

```bash
ros2 topic hz /pico/smpl
ros2 topic hz /raw/odom/odin_highfreq
ros2 topic echo /calibrated/odom/pelvis_highfreq --once
ros2 topic echo /pico/smpl_odin --once
ros2 topic echo /pico/smpl_fused_odin --once
```

The user verifies one excitation calibration, nonzero initial pelvis height, XYZ signs, individual roll/pitch/yaw directions, restart reacquisition, and preservation of the prior YAML after a deliberately failed calibration.

- [ ] **Step 5: Commit only genuine verification fixes**

If no defect is found, create no empty commit. If a defect is found, add only the relevant files and use `fix(pico_odin): <specific verified defect>`.
