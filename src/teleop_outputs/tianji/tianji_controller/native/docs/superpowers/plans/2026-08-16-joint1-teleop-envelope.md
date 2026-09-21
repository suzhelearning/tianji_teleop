# Joint 1 Teleoperation Envelope Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent unsafe shoulder-base rotation by limiting left J1 to at least -90 degrees and right J1 to at most +90 degrees.

**Architecture:** Encode the limits in both MuJoCo model joint ranges. `MujocoRobot` already imports these ranges into the existing position, velocity, acceleration, jerk, and braking bounds, so no new QP constraint is required.

**Tech Stack:** MuJoCo XML, C++ GoogleTest, CMake/CTest.

## Global Constraints

- Keep the current branch and worktree.
- Do not commit.
- Left J1 range: `[-1.5708, 3.1067] rad`.
- Right J1 range: `[-3.1067, 1.5708] rad`.
- Apply the same ranges to `marvin_m6_qp_test.xml` and `marvin_m6_qp_pico_fast.xml`.

---

### Task 1: Lock and verify the J1 model limits

**Files:**
- Modify: `tests/test_mujoco_robot.cpp`
- Modify: `models/marvin_m6_qp_test.xml`
- Modify: `models/marvin_m6_qp_pico_fast.xml`

**Interfaces:**
- Consumes: `MujocoRobot::mapping(ArmSide).limits`.
- Produces: side-specific J1 limits consumed by all existing QP bound builders.

- [ ] **Step 1: Write the failing test**

Add assertions that left J1 lower is `-1.5708`, left J1 upper remains `3.1067`, right J1 lower remains `-3.1067`, and right J1 upper is `1.5708`.

- [ ] **Step 2: Verify RED**

Run `./build/test_mujoco_robot --gtest_filter='MujocoRobot.*JointLimits*'`; the new assertions must fail against the current `±3.1067` J1 ranges.

- [ ] **Step 3: Implement the model limits**

Change only the two J1 ranges in each model:

```xml
<joint name="Joint1_L" ... range="-1.5708 3.1067" .../>
<joint name="Joint1_R" ... range="-3.1067 1.5708" .../>
```

- [ ] **Step 4: Verify GREEN and regressions**

Run the focused model test, controller tests, and full serial CTest. Expected result: all tests pass and no QP dimensions change.

- [ ] **Step 5: Run the recorded PICO replay**

Replay `pico_fast_motion_20260812_205428.tjvr` in headless velocity mode and confirm zero bound violations/control failures before visual testing.
