# SPARK Feedforward Joint-Limit Consistency Implementation Plan

> **For agentic workers:** Execute inline in the current workspace. Do not use
> subagents and do not commit.

**Goal:** Prevent the SPARK jerk-limited joint reference from crossing the
velocity QP's margin-protected joint limits.

**Architecture:** Pass the active `JointLimitConfig` into
`SparkFeedforwardReference7` and project each candidate next velocity through
the existing `computeJointVelocityBounds()` function before integration.

**Tech Stack:** C++20, Eigen, GoogleTest, MuJoCo, qpOASES.

## Global Constraints

- Keep the current branch and worktree.
- Do not change configured velocity, acceleration, braking, or jerk values.
- Do not commit.

### Task 1: Reproduce the inconsistent margin

**Files:**
- Test: `tests/test_spark_feedforward_reference.cpp`

- [ ] Add a test with physical limits `[-4, 4]`, margin `0.05`, and an inward
  moving feedforward state whose target stops at `-3.95`.
- [ ] Assert every step keeps `q >= -3.95`, respects derivative limits, and
  settles without repeated inward motion at the boundary.
- [ ] Run `test_spark_feedforward_reference` and observe failure because the
  current reference can enter `[-4.0, -3.95)`.

### Task 2: Reuse the velocity-QP feasible set

**Files:**
- Modify: `include/tianji_qp_ik/spark_feedforward_reference.hpp`
- Modify: `src/spark_feedforward_reference.cpp`
- Modify: `src/spark_guidance.cpp`
- Test: `tests/test_spark_feedforward_reference.cpp`

- [ ] Replace separate acceleration/jerk constructor inputs with the active
  `JointLimitConfig`.
- [ ] Build `maximum_acceleration_` and `maximum_jerk_` from that config while
  retaining reference scaling.
- [ ] Project candidate `qdot` with `computeJointVelocityBounds()` before
  updating q/qdot/qddot/jerk.
- [ ] Run focused reference and guidance tests until green.

### Task 3: Regression and replay

**Files:**
- Generate: `benchmark_results/pico_live/joint_limit_consistency_fix/`

- [ ] Run `git diff --check`, the complete serial CTest suite, and verify all
  tests pass.
- [ ] Replay `pico_fast_motion_20260812_205428_v4.tjvr` through the headroom
  feedforward velocity-QP mode.
- [ ] Verify zero control failures and zero q/qdot/qddot/jerk violations.
- [ ] Verify feedforward q never enters the controller's protected margin.
- [ ] Run a MuJoCo visual replay and leave all changes uncommitted.

