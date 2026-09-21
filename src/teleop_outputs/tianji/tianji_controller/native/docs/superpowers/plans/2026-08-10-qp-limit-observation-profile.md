# QP Limit-Observation Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a separate high-gain Cartesian profile that exposes hierarchical QP joint-bound behavior without weakening the standard QP/DLS regression profile.

**Architecture:** Copy the validated hierarchical YAML profile into a dedicated experiment file and change only four Cartesian servo parameters. The standard profile remains the CTest authority; a new configuration test protects the experiment profile, while headless and benchmark runs provide QP behavior evidence.

**Tech Stack:** YAML, C++17, GoogleTest, CMake/CTest, MuJoCo, qpOASES

## Global Constraints

- Create `config/qp_ik_qp_limit.yaml`; do not modify `config/qp_ik_hierarchical.yaml`.
- Experimental `kp_position` is exactly `[50.0, 50.0, 50.0]`.
- Experimental `kp_orientation` is exactly `[30.0, 30.0, 30.0]`.
- Experimental `max_linear_velocity` and `max_angular_velocity` are exactly `100.0`.
- Keep `joint_limits.velocity_scale` at `1.00` and `joint_limits.margin_rad` at `0.05 rad` in both profiles.
- Keep the bilateral Joint4 range at `[-2.5307, 0.0] rad`.
- Do not change solver weights, safety validation, or standard benchmark acceptance thresholds.
- Keep the current branch and linked worktree.

---

### Task 1: Add the independent QP limit-observation profile

**Files:**
- Create: `config/qp_ik_qp_limit.yaml`
- Modify: `tests/test_config.cpp`

**Interfaces:**
- Consumes: `loadConfig(const std::string&)` and `QpIkConfig::cartesian_servo`.
- Produces: an opt-in hierarchical QP profile whose desired twist is normally constrained by QP joint bounds rather than Cartesian clamps.

- [x] **Step 1: Write the failing profile-loading test**

Add this test to `tests/test_config.cpp`:

```cpp
TEST(Config, LoadsQpLimitObservationProfile) {
  const auto standard_path = std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) /
                             "config" / "qp_ik_hierarchical.yaml";
  const auto path = std::filesystem::path(TIANJI_PROJECT_SOURCE_DIR) /
                    "config" / "qp_ik_qp_limit.yaml";
  const QpIkConfig standard = loadConfig(standard_path.string());
  const QpIkConfig config = loadConfig(path.string());
  EXPECT_EQ(config.ik_algorithm, IkAlgorithm::kHierarchicalQp);
  EXPECT_TRUE(config.cartesian_servo.kp_position.isApprox(
      Eigen::Vector3d::Constant(50.0)));
  EXPECT_TRUE(config.cartesian_servo.kp_orientation.isApprox(
      Eigen::Vector3d::Constant(30.0)));
  EXPECT_DOUBLE_EQ(config.cartesian_servo.max_linear_velocity, 100.0);
  EXPECT_DOUBLE_EQ(config.cartesian_servo.max_angular_velocity, 100.0);
  EXPECT_DOUBLE_EQ(config.joint_limits.velocity_scale, 1.00);
  EXPECT_DOUBLE_EQ(config.joint_limits.margin_rad, 0.05);
}
```

The completed test also compares every non-Cartesian field in `QpIkConfig`
against `standard`, including controller, algorithm, QP/hierarchical-QP/DLS,
joint-limit, solver-backend, safety, and trajectory settings. This makes any
future drift outside the four intended Cartesian values fail the regression.

- [x] **Step 2: Build and verify RED**

Run:

```bash
cmake --build build --target test_config -j
./build/test_config --gtest_filter=Config.LoadsQpLimitObservationProfile
```

Expected: the test fails by throwing `cannot open config file` because
`config/qp_ik_qp_limit.yaml` does not exist.

- [x] **Step 3: Create the experimental YAML**

Copy every standard hierarchical profile setting verbatim, except for this
block in `config/qp_ik_qp_limit.yaml`:

```yaml
cartesian_servo:
  kp_position: [50.0, 50.0, 50.0]
  kp_orientation: [30.0, 30.0, 30.0]
  max_linear_velocity: 100.0
  max_angular_velocity: 100.0
```

Keep these safety values unchanged:

```yaml
joint_limits:
  margin_rad: 0.05
  velocity_scale: 1.00
```

- [x] **Step 4: Verify GREEN**

Run:

```bash
./build/test_config --gtest_filter=Config.LoadsQpLimitObservationProfile
```

Expected: one test passes with no failures.

- [x] **Step 5: Record experimental QP/DLS metrics**

Run:

```bash
./build/tianji_hierarchical_ik_benchmark \
  --config config/qp_ik_qp_limit.yaml \
  --model models/marvin_m6_qp_test.xml \
  --steps 600 \
  --output /tmp/tianji_qp_limit_observation.csv
```

Expected: all hierarchical-QP rows report `failures=0`; central, combined,
and singular QP final pose errors remain below `0.002 m` and `1 degree`.
The process reports `benchmark_complete accepted=0` and exits `2` because the
DLS singular row is outside the experimental profile's acceptance scope; QP
rows are evaluated separately.

- [x] **Step 6: Run experimental headless acceptance**

Run:

```bash
./build/tianji_qp_ik_viewer \
  --config config/qp_ik_qp_limit.yaml \
  --model models/marvin_m6_qp_test.xml \
  --headless --duration 4 \
  --telemetry /tmp/tianji_qp_limit_observation_telemetry.csv
```

Expected: exit zero with `accepted=1`, `control_failures=0`,
`command_failures=0`, and `completed_stage=10`.

- [x] **Step 7: Verify the unchanged standard profile**

Run:

```bash
git diff --exit-code HEAD -- config/qp_ik_hierarchical.yaml
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Expected: the standard profile has no diff and all 22 tests pass, including
the strict standard QP/DLS benchmark.

- [x] **Step 8: Commit**

```bash
git add config/qp_ik_qp_limit.yaml tests/test_config.cpp \
  docs/superpowers/plans/2026-08-10-qp-limit-observation-profile.md
git commit -m "feat: add QP limit-observation profile"
```
